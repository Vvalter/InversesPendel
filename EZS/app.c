#nclude <cyg/hal/hal_arch.h>
#include <cyg/kernel/kapi.h>

#include <stdio.h>
#include <math.h>
#include <assert.h>

#include "ezs_counter.h"
#include "ezs_serial.h"
#include "ezs_sensor.h"
#include "ezs_stopwatch.h"
#include "ezs_adc.h"
#include "ezs_dac.h"
#include "ezs_io_fel.h"
#include "ezs_fft.h"
#include "ezs_interpolation.h"
#include "ezs_plot.h"
#include "ezs_plot_pds.h"

/*
*  Define: POLLING, HINTERGRUND oder UNTERBRECHER 
    #define POLLING
    #define HINTERGRUND
    #define UNTERBRECHER
*/
#define POLLING
//#define UNTERBRECHER
//#define HINTERGRUND

#define A6

#define SERIAL_IRQ CYGNUM_HAL_INTERRUPT_UART2

#define SAMPLING_TASK_PRIORITY 11
#define FLANK_TASK_PRIORITY 12
#define DISPLAY_SIGNAL_TASK_PRIORITY 14
#define DISPLAY_PDS_TASK_PRIORITY 16
#define DISPLAY_TRIGGER_TASK_PRIORITY 17 //TODO
#define ANALYSIS_TASK_PRIORITY 15
#define DECODING_TASK_PRIORITY 18 //TODO // Unused

#ifdef UNTERBRECHER
#define POLLING_TASK_PRIORITY 5 //TODO
#endif

#ifdef HINTERGRUND
#define POLLING_TASK_PRIORITY 25 //TODO
#endif

#ifdef POLLING
#define POLLING_TASK_PRIORITY 13 //TODO
#endif

#define STATEMACHINE_TASK_PRIORITY 18 //TODO

#define SAMPLING_TASK_PERIOD 4
#define FLANK_TASK_PERIOD 4
#define DISPLAY_SIGNAL_TASK_PERIOD 250
#define DISPLAY_PDS_TASK_PERIOD 1000
#define ANALYSIS_TASK_PERIOD 1000
#define POLLING_TASK_PERIOD 20 //TODO

#define FLANK_WCET 1

#define SAMPLING_TASK_PHASE 0
#define FLANK_TASK_PHASE 1
#define ANALYSIS_TASK_PHASE 0
#define DISPLAY_SIGNAL_TASK_PHASE 15
#define DISPLAY_PDS_TASK_PHASE 35 //20 + 15
#define POLLING_TASK_PHASE 0

#define STACKSIZE    (CYGNUM_HAL_STACK_SIZE_MINIMUM+4096)

#define PDS_LENGTH 32
#define TIME_DOMAIN_LENGTH (2 * PDS_LENGTH)
static cyg_uint32 s_time_domain[TIME_DOMAIN_LENGTH];
static cyg_uint32 s_trigger_domain[TIME_DOMAIN_LENGTH];
static float s_frequency_domain[PDS_LENGTH];
static unsigned int s_position = 0;

#define SERIAL_BUFFER_LENGTH 15
static char s_serial_buffer[SERIAL_BUFFER_LENGTH];
static unsigned int s_serial_position;
static bool s_command_decodable = false;

static volatile uint8_t char_in;

#ifdef POLLING
static volatile uint8_t polling_flag = 0;
#endif 

static cyg_uint32 watch_t7 = 0;

static cyg_flag_t cflag;
static cyg_flag_t flankflag;
static volatile cyg_uint8 state = 0;


static cyg_handle_t sampling_task_alarm_handle;
static cyg_handle_t flank_task_alarm_handle;
static cyg_handle_t analysis_task_alarm_handle;
static cyg_handle_t display_signal_task_alarm_handle;
static cyg_handle_t display_pds_task_alarm_handle;
#ifdef POLLING
static cyg_handle_t polling_task_alarm_handle;
#endif

static cyg_alarm sampling_task_alarm;
static cyg_alarm flank_task_alarm;
static cyg_alarm analysis_task_alarm;
static cyg_alarm display_signal_task_alarm;
static cyg_alarm display_pds_task_alarm;
#ifdef POLLING
static cyg_alarm polling_task_alarm;
#endif

static cyg_handle_t mailbox_handle;
static cyg_mbox mailbox;


#ifdef A6
static cyg_handle_t mailbox2_handle;
static cyg_mbox mailbox2;
#endif

enum CommandStatus
{
	CommandComplete,
	CommandIncomplete
};

static enum CommandStatus packet_receive(char c)
{
    //ezs_printf("packet_receive, pos: %d\n", s_serial_position);
	assert(s_serial_position < SERIAL_BUFFER_LENGTH);
    if(c == '\n'){
        //ezs_printf("packet_receive newline\n");
        s_serial_buffer[s_serial_position] = '\0';
        s_serial_position = 0;
        enum CommandStatus cs = CommandComplete;
        return cs;
    }else{
        //ezs_printf("packet_receive something\n");
        s_serial_buffer[s_serial_position] = c;
        s_serial_position = (s_serial_position + 1) % SERIAL_BUFFER_LENGTH;
        enum CommandStatus cs = CommandIncomplete;
        return cs;
    }
    return 0;
}

enum Command
{
	DisplayTime = (1 << 1),
	DisplayPDS  = (1 << 2),
	TriggerOn   = (1 << 3),
	TriggerOff  = (1 << 4),
	TLevelRise  = (1 << 5),
	TLevelFall  = (1 << 6),
	Invalid     = 0x00,
};

char *commands[6] = {"display signal", "display pds", "trigger on", "trigger off", "tlevel rise", "tlevel fall"};

enum State
{
	Display =(1 << 0),
	Trigger =(1 << 1),
	TLevel =(1 << 2),

};
//Little helper functions.
static cyg_tick_count_t ms_to_cyg_ticks(cyg_uint32 ms)
{
	cyg_resolution_t resolution = cyg_clock_get_resolution(cyg_real_time_clock());
    cyg_tick_count_t val = ((100 * ms * resolution.divisor)/(resolution.dividend/10000)); //ticks
	return val;
}

static cyg_tick_count_t ms_to_ezs_ticks(cyg_uint32 ms) {
    cyg_resolution_t res = ezs_counter_get_resolution(); // get resolution ns/ticks
	return (1000000 * ms * res.divisor)/res.dividend; //ticks
}

enum Command decode_command(void)
{
    //ezs_printf("Decoding now\n");
	enum Command ret = Invalid;
    int i;
    for(i = 0; i < 6; i++){
        if(strncmp(s_serial_buffer, commands[i], SERIAL_BUFFER_LENGTH) == 0){
            break;
        }
    }
    switch(i){
        case 0:
            ret = DisplayTime;
            break;
        case 1:
            ret = DisplayPDS;
            break;
        case 2:
            ret = TriggerOn;
            break;
        case 3:
            ret = TriggerOff;
            break;
        case 4:
            ret = TLevelRise;
            break;
        case 5:
            ret = TLevelFall;
            break;
    }
    //ezs_printf("Decoded: %d\n", ret);
	return ret;
}

cyg_uint32 serial_isr_handler(cyg_vector_t vector, cyg_addrword_t data)
{

	if (ezs_serial_char_available())
	{
        ezs_watch_start(&watch_t7);
	    char_in = ezs_serial_getc();
		cyg_interrupt_acknowledge(vector);
		return CYG_ISR_CALL_DSR;
	}
	else
	{
		return CYG_ISR_HANDLED;
	}

}

static cyg_uint8     polling_task_stack[STACKSIZE];
static cyg_handle_t  polling_task_handle;
static cyg_thread    polling_task_thread;
/*
static cyg_uint8     decode_task_stack[STACKSIZE];
static cyg_handle_t  decode_task_handle;
static cyg_thread    decode_task_thread;
*/
void serial_dsr_handler(cyg_vector_t vec, cyg_ucount32 count, cyg_addrword_t data)
{
   
 
    if(packet_receive(char_in) == CommandComplete){
        // Unterbrecherbetrieb, decode_command unterbricht alle
#ifdef UNTERBRECHER
        cyg_thread_resume(polling_task_handle);
#endif //Unterbrecher

        // Hintergrundbetrieb, task mit nieder Prio resumen
#ifdef HINTERGRUND
        cyg_thread_resume(polling_task_handle);
#endif //Hintergrund

#ifdef POLLING
        polling_flag = 1;
#endif //POLLING
    }
  
}


// periodischer Zusteller
// T7
static void polling_task_entry(cyg_addrword_t data)
{
	while (1)
	{
        //ezs_printf("Polling task\n");
#ifdef POLLING
        cyg_interrupt_disable();
        while(polling_flag == 0){
            cyg_interrupt_enable();
            cyg_thread_suspend(cyg_thread_self());
            cyg_interrupt_disable();
        }
        cyg_interrupt_enable();
        polling_flag = 0;
#endif //POLLING
        enum Command c = decode_command();
        cyg_resolution_t res = ezs_counter_get_resolution(); // get resolution ns/ticks
        cyg_uint32 diff = ezs_watch_stop(&watch_t7);
        cyg_uint32 diff_ns = diff*res.dividend/res.divisor;
        ezs_printf("T7 %d\n", diff_ns);
        cyg_flag_setbits(&cflag, c);
		cyg_thread_suspend(cyg_thread_self());
	}
}

// Decode Task
/*
static void decode_task_entry(cyg_addrword_t data)
{
	while (1)
	{
        ezs_printf("Decode task\n");
		cyg_thread_suspend(cyg_thread_self());
	}
}
*/

// Zustandsmaschine
// T8
static cyg_uint8     statemachine_task_stack[STACKSIZE];
static cyg_handle_t  statemachine_task_handle;
static cyg_thread    statemachine_task_thread;
static void statemachine_task_entry(cyg_addrword_t data)
{
	while (1)
	{
        cyg_flag_value_t val = cyg_flag_wait(&cflag, 0b11111111, CYG_FLAG_WAITMODE_OR | CYG_FLAG_WAITMODE_CLR);

        switch(val){
            case DisplayTime:
                state |= Display;
                cyg_alarm_disable(analysis_task_alarm_handle);
                cyg_alarm_disable(display_pds_task_alarm_handle);
                //cyg_alarm_disable(flank_task_alarm_handle);
                cyg_alarm_enable(display_signal_task_alarm_handle);
                break;
            case DisplayPDS:
                state &= ~Display; //(1 << 0);
                cyg_alarm_enable(analysis_task_alarm_handle);
                cyg_alarm_enable(display_pds_task_alarm_handle);
                cyg_alarm_disable(display_signal_task_alarm_handle);
                //cyg_alarm_disable(flank_task_alarm_handle);
                break;
            case TriggerOn:
                state |= Trigger; //1 << 1;
                cyg_alarm_disable(analysis_task_alarm_handle);
                cyg_alarm_disable(display_pds_task_alarm_handle);
                //cyg_alarm_enable(flank_task_alarm_handle);
                cyg_alarm_disable(display_signal_task_alarm_handle);
                break;
            case TriggerOff:
                state &= ~Trigger; //(1 << 1);
                cyg_alarm_disable(analysis_task_alarm_handle);
                cyg_alarm_disable(display_pds_task_alarm_handle);
                //cyg_alarm_disable(flank_task_alarm_handle);
                cyg_alarm_enable(display_signal_task_alarm_handle);
                break;
            case TLevelRise:
                state |= TLevel; //1 << 2;
                break;
            case TLevelFall:
                state &= ~TLevel; //(1 << 2);
                break;
        }

        ezs_printf("State: %d\n", state);
	}
}

// T9
static cyg_uint8     display_trigger_task_stack[STACKSIZE];
static cyg_handle_t  display_trigger_task_handle;
static cyg_thread    display_trigger_task_thread;
static void display_trigger_task_entry(cyg_addrword_t data)
{
    while(1){
        void *message = cyg_mbox_get(mailbox_handle);
        //ezs_printf("display trigger\n");
        ezs_plot((cyg_uint32*)message, TIME_DOMAIN_LENGTH, CYG_FB_DEFAULT_PALETTE_RED, CYG_FB_DEFAULT_PALETTE_RED);
        //cyg_thread_suspend(cyg_thread_self());
    }
}

// T1
static cyg_uint8     sampling_task_stack[STACKSIZE];
static cyg_handle_t  sampling_task_handle;
static cyg_thread    sampling_task_thread;
static void sampling_task_entry(cyg_addrword_t data)
{
	while (1)
	{
        //ezs_printf("T1\n");
        cyg_uint8 val = ezs_adc_read();
        s_time_domain[s_position] = val;
        s_position = (s_position+1) % TIME_DOMAIN_LENGTH;
        if(state & Trigger){
            cyg_flag_setbits(&flankflag, 1);
        }
		cyg_thread_suspend(cyg_thread_self());
	}
}

// T2
static cyg_uint8     flank_task_stack[STACKSIZE];
static cyg_handle_t  flank_task_handle;
static cyg_thread    flank_task_thread;
static void flank_task_entry(cyg_addrword_t data)
{
    cyg_uint8 prev_val;
    cyg_uint8 cur_val = 0;
    cyg_uint8 thresh = 188;
	while (1)
	{
        cyg_flag_wait(&flankflag, 1, CYG_FLAG_WAITMODE_CLR);
        //ezs_printf("t2 resumed\n");
        prev_val = cur_val;
        cur_val = s_time_domain[s_position];
        if(cur_val > thresh && prev_val < thresh && (state & TLevel)){
            //rising
            cyg_uint32 local_pos = s_position;
            int i;
            for(i = 0; i < TIME_DOMAIN_LENGTH; i++){
                s_trigger_domain[i] = s_time_domain[(local_pos + i) % TIME_DOMAIN_LENGTH];
            }
            cyg_mbox_put(mailbox_handle, s_trigger_domain);
            //cyg_thread_resume(display_trigger_task_handle);
        }else if(cur_val < thresh && prev_val > thresh && (~state & TLevel)){
            //falling
            cyg_uint32 local_pos = s_position;
            int i;
            for(i = 0; i < TIME_DOMAIN_LENGTH; i++){
                s_trigger_domain[i] = s_time_domain[(local_pos + i) % TIME_DOMAIN_LENGTH];
            }
            //cyg_thread_resume(display_trigger_task_handle);
            cyg_mbox_put(mailbox_handle, s_trigger_domain);

        }
		//cyg_thread_suspend(cyg_thread_self());
	}
}

// T3
static cyg_uint8     analysis_task_stack[STACKSIZE];
static cyg_handle_t  analysis_task_handle;
static cyg_thread    analysis_task_thread;
static void analysis_task_entry(cyg_addrword_t data)
{
	while (1)
	{
        cyg_resolution_t res = ezs_counter_get_resolution(); // get resolution ns/ticks
        cyg_uint32 watch = 0;
        //cyg_interrupt_disable();
        ezs_watch_start(&watch);


        cyg_uint32 local_pos = s_position;
        cyg_uint32 values[TIME_DOMAIN_LENGTH];
        int i;
        for(i = 0; i < TIME_DOMAIN_LENGTH; i++){
            values[i] = s_time_domain[(local_pos + i) % TIME_DOMAIN_LENGTH];
        }
        ezs_easy_pds(values, s_frequency_domain, TIME_DOMAIN_LENGTH);


        cyg_uint32 diff = ezs_watch_stop(&watch);
        //cyg_interrupt_enable();
        cyg_uint32 diff_ns = diff*res.dividend/res.divisor;
        //ezs_printf("T3 %d\n", diff_ns);

#ifdef A6
        cyg_mbox_put(mailbox2_handle, s_frequency_domain);
#endif

		cyg_thread_suspend(cyg_thread_self());
	}
}


// T4
static cyg_uint8     display_signal_task_stack[STACKSIZE];
static cyg_handle_t  display_signal_task_handle;
static cyg_thread    display_signal_task_thread;
static void display_signal_task_entry(cyg_addrword_t data)
{
	while (1)
	{
        cyg_resolution_t res = ezs_counter_get_resolution(); // get resolution ns/ticks
        cyg_uint32 watch = 0;
        //cyg_interrupt_disable();
        ezs_watch_start(&watch);


        cyg_uint32 local_pos = s_position;
        cyg_uint32 values[TIME_DOMAIN_LENGTH];
        int i;
        for(i = 0; i < TIME_DOMAIN_LENGTH; i++){
            values[i] = s_time_domain[(local_pos + i) % TIME_DOMAIN_LENGTH];
        }
        ezs_plot(values, TIME_DOMAIN_LENGTH, CYG_FB_DEFAULT_PALETTE_RED, CYG_FB_DEFAULT_PALETTE_RED);


        cyg_uint32 diff = ezs_watch_stop(&watch);
        //cyg_interrupt_enable();
        cyg_uint32 diff_ns = diff*res.dividend/res.divisor;
        //ezs_printf("T4 %d\n", diff_ns);


		cyg_thread_suspend(cyg_thread_self());
	}
}

// T5
static cyg_uint8     display_pds_task_stack[STACKSIZE];
static cyg_handle_t  display_pds_task_handle;
static cyg_thread    display_pds_task_thread;
static void display_pds_task_entry(cyg_addrword_t data)
{
	while (1)
	{
        cyg_resolution_t res = ezs_counter_get_resolution(); // get resolution ns/ticks
        cyg_uint32 watch = 0;
        //cyg_interrupt_disable();
        ezs_watch_start(&watch);

#ifdef A6
        void *message = cyg_mbox_get(mailbox2_handle);
        ezs_plot_pds((float*)message, PDS_LENGTH, CYG_FB_DEFAULT_PALETTE_CYAN, FB_BLACK);
#else
        ezs_plot_pds(s_frequency_domain, PDS_LENGTH, CYG_FB_DEFAULT_PALETTE_CYAN, FB_BLACK);
#endif



        cyg_uint32 diff = ezs_watch_stop(&watch);
        //cyg_interrupt_enable();
        cyg_uint32 diff_ns = diff*res.dividend/res.divisor;
        //ezs_printf("T5 %d\n", diff_ns);
#ifndef A6
		cyg_thread_suspend(cyg_thread_self());
#endif
	}
}

static void sampling_task_alarmfn(cyg_handle_t alarmH, cyg_addrword_t data)
{
	cyg_thread_resume(sampling_task_handle);
}

static void flank_task_alarmfn(cyg_handle_t alarmH, cyg_addrword_t data)
{
	cyg_thread_resume(flank_task_handle);
}

static void analysis_task_alarmfn(cyg_handle_t alarmH, cyg_addrword_t data)
{
	cyg_thread_resume(analysis_task_handle);
}

static void display_signal_task_alarmfn(cyg_handle_t alarmH, cyg_addrword_t data)
{
	cyg_thread_resume(display_signal_task_handle);
}

static void display_pds_task_alarmfn(cyg_handle_t alarmH, cyg_addrword_t data)
{
	cyg_thread_resume(display_pds_task_handle);
}

#ifdef POLLING
static void polling_task_alarmfn(cyg_handle_t alarmH, cyg_addrword_t data)
{
	cyg_thread_resume(polling_task_handle);
}
#endif


static cyg_handle_t real_time_counter;

static cyg_addrword_t data_dummy = 0;
static cyg_interrupt serial_intr;
static cyg_handle_t  serial_isr_handle;

void cyg_user_start(void)
{
	ezs_fb_init();
	ezs_counter_init();
	ezs_sensors_init();
	ezs_fel_serial_init();
	cyg_interrupt_create(SERIAL_IRQ,
	                     1,
	                     0,
	                     serial_isr_handler,
	                     serial_dsr_handler,
	                     &serial_isr_handle,
	                     &serial_intr) ;
	cyg_interrupt_attach(serial_isr_handle);
	cyg_interrupt_unmask(SERIAL_IRQ);

	cyg_thread_create(SAMPLING_TASK_PRIORITY, &sampling_task_entry, 0, "sampling task",
	                  sampling_task_stack, STACKSIZE,
	                  &sampling_task_handle, &sampling_task_thread);

	cyg_thread_create(FLANK_TASK_PRIORITY, &flank_task_entry, 0, "flank task",
	                  flank_task_stack, STACKSIZE,
	                  &flank_task_handle, &flank_task_thread);
    cyg_thread_resume(flank_task_handle);

	cyg_thread_create(ANALYSIS_TASK_PRIORITY, &analysis_task_entry, 0, "analysis task",
	                  analysis_task_stack, STACKSIZE,
	                  &analysis_task_handle, &analysis_task_thread);
	cyg_thread_create(DISPLAY_SIGNAL_TASK_PRIORITY, &display_signal_task_entry, 0, "display signal task",
	                  display_signal_task_stack, STACKSIZE,
	                  &display_signal_task_handle, &display_signal_task_thread);
	cyg_thread_create(DISPLAY_PDS_TASK_PRIORITY, &display_pds_task_entry, 0, "display pds task",
	                  display_pds_task_stack, STACKSIZE,
	                  &display_pds_task_handle, &display_pds_task_thread);
#ifdef A6
    cyg_thread_resume(display_pds_task_handle);
#endif
	cyg_thread_create(DISPLAY_TRIGGER_TASK_PRIORITY, &display_trigger_task_entry, 0,
                      "display trigger task",
	                  display_trigger_task_stack, STACKSIZE,
	                  &display_trigger_task_handle, &display_trigger_task_thread);
    cyg_thread_resume(display_trigger_task_handle);

	cyg_thread_create(STATEMACHINE_TASK_PRIORITY, &statemachine_task_entry, 0, "statemachine task",
	                  statemachine_task_stack, STACKSIZE,
	                  &statemachine_task_handle, &statemachine_task_thread);
    cyg_thread_resume(statemachine_task_handle);
    /*
	cyg_thread_create(DECODING_TASK_PRIORITY, &decode_task_entry, 0, "decode task",
	                  decode_task_stack, STACKSIZE,
	                  &decode_task_handle, &decode_task_thread);
    */
#ifdef HINTERGRUND
	cyg_thread_create(POLLING_TASK_PRIORITY, &polling_task_entry, 0, "polling task",
	                  polling_task_stack, STACKSIZE,
	                  &polling_task_handle, &polling_task_thread);
#endif //Hintergrund
#ifdef POLLING
	cyg_thread_create(POLLING_TASK_PRIORITY, &polling_task_entry, 0, "polling task",
	                  polling_task_stack, STACKSIZE,
	                  &polling_task_handle, &polling_task_thread);
#endif //POLLING
#ifdef UNTERBRECHER
	cyg_thread_create(POLLING_TASK_PRIORITY, &polling_task_entry, 0, "polling task",
	                  polling_task_stack, STACKSIZE,
	                  &polling_task_handle, &polling_task_thread);
#endif //UNTERBRECHER

	cyg_clock_to_counter(cyg_real_time_clock(), &real_time_counter);
	cyg_uint32 timebase = cyg_current_time() + 3;

#ifdef POLLING
	cyg_alarm_create(real_time_counter, polling_task_alarmfn, data_dummy, &polling_task_alarm_handle, &polling_task_alarm);
	cyg_alarm_initialize(polling_task_alarm_handle, timebase + ms_to_cyg_ticks(POLLING_TASK_PHASE), ms_to_cyg_ticks(POLLING_TASK_PERIOD));
#endif //POLLING

	cyg_alarm_create(real_time_counter, sampling_task_alarmfn, data_dummy, &sampling_task_alarm_handle, &sampling_task_alarm);
	cyg_alarm_initialize(sampling_task_alarm_handle, timebase + ms_to_cyg_ticks(SAMPLING_TASK_PHASE), ms_to_cyg_ticks(SAMPLING_TASK_PERIOD));
/*
	cyg_alarm_create(real_time_counter, flank_task_alarmfn, data_dummy, &flank_task_alarm_handle, &flank_task_alarm);
	cyg_alarm_initialize(flank_task_alarm_handle, timebase + ms_to_cyg_ticks(FLANK_TASK_PHASE), ms_to_cyg_ticks(FLANK_TASK_PERIOD));
*/
	cyg_alarm_create(real_time_counter, analysis_task_alarmfn, data_dummy, &analysis_task_alarm_handle, &analysis_task_alarm);
	cyg_alarm_initialize(analysis_task_alarm_handle, timebase + ms_to_cyg_ticks(ANALYSIS_TASK_PHASE), ms_to_cyg_ticks(ANALYSIS_TASK_PERIOD));

	cyg_alarm_create(real_time_counter, display_signal_task_alarmfn, data_dummy, &display_signal_task_alarm_handle, &display_signal_task_alarm);
	cyg_alarm_initialize(display_signal_task_alarm_handle, timebase + ms_to_cyg_ticks(DISPLAY_SIGNAL_TASK_PHASE), ms_to_cyg_ticks(DISPLAY_SIGNAL_TASK_PERIOD));
#ifndef A6
	cyg_alarm_create(real_time_counter, display_pds_task_alarmfn, data_dummy, &display_pds_task_alarm_handle, &display_pds_task_alarm);
	cyg_alarm_initialize(display_pds_task_alarm_handle, timebase + ms_to_cyg_ticks(DISPLAY_PDS_TASK_PHASE), ms_to_cyg_ticks(DISPLAY_PDS_TASK_PERIOD));
#endif
    cyg_flag_init(&cflag);
    cyg_flag_setbits(&cflag, 2);
    cyg_flag_init(&flankflag);

    cyg_mbox_create(&mailbox_handle, &mailbox);
#ifdef A6
    cyg_mbox_create(&mailbox2_handle, &mailbox2);
#endif
}
