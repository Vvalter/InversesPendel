#include <stdlib.h>
#include <math.h>

#include "ezs_io.h"
#include "state_machine.h"
#include "lib.h"

#define REGEL_INTERVALL TIMER_TICKS_PER_MICROSECOND * 125

int printf(const char*, ...);

static int32_t lastPositionWaggon;
static int16_t lastPositionPendulum;
static double error_sum;

static const double MAX_ERROR_SUM = 50000000;

static int16_t lastError;
static uint32_t lastTime;

static int cnt = 0;

static const double Kp = -0.90;
static const double Ki = -0.000000002;
static const double Kd = -0.5 * 700 * 100000;


enum state handle_pid_init(struct PeripheralState *st) {
        lastPositionWaggon = st->positionWaggon;
        lastPositionPendulum = st->positionPendulum;
        lastTime = st->time;
        error_sum = 0;
        cnt = 0;
        return PID_STEP;
}

enum state handle_pid_step(struct PeripheralState *st) {
        if (st->tasterLeft || st->tasterRight) {
                ezs_printf("%d %d IDLE\n", st->tasterLeft, st->tasterRight);
                return IDLE;
        }
        if (abs(st->time-lastTime) >= REGEL_INTERVALL) {
                uint32_t dt = abs(lastTime - st->time);
                //if (dt > 20000) dt = 2000;
                int16_t da = lastPositionPendulum - st->positionPendulum;
                int32_t dp = lastPositionWaggon - st->positionWaggon;
                int16_t sollWinkel = 0;
                int16_t istWinkel = convertPendulumAngleToUpper(st->positionPendulum);

                if (abs(istWinkel) > 150) {
                        breakMotor();
                        return PID_STEP;
                }

                int16_t error = sollWinkel - istWinkel;
                double error_d = (error-lastError) / ((double)dt);
                error_sum += ((double)error) * ((double)dt);

                if (error_sum > MAX_ERROR_SUM) error_sum = MAX_ERROR_SUM;
                else if (error_sum < -MAX_ERROR_SUM) error_sum = -MAX_ERROR_SUM;

                lastError = error;
                
                double res = Kp * error + Ki * error_sum + Kd * error_d;
                if (cnt++ >= 100) {
                        cnt = 0;
                        ezs_printf("dt: %u error: %d error_sum: %f error_d %f res: %f = %f + %f + %f\n", (unsigned int) dt, (int)error, error_sum, error_d, (double)res, Kp*error, Ki * error_sum, Kd * error_d);
                }


                if (res > 0) {
                        if (dp < 0) {
                                breakMotor();
                        } else {
                                driveLeft(fmin(fabs(res), 1.0f));
                        }
                } else {
                        if (dp > 0) {
                                breakMotor();
                        } else {
                                driveRight(fmin(fabs(res), 1.0f));
                        }
                }


                lastPositionWaggon = st->positionWaggon;
                lastPositionPendulum = st->positionPendulum;
                lastTime = st->time;
        }
        return PID_STEP;
}
