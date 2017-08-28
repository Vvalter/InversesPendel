f = open('MotorGrosGetriebeLeerlauf3.log', 'r')
lines = f.readlines()

vals = [map(int, l.split()) for l in lines]

def tickToTime(tick):
    TICKS_PER_SECOND = 42*10**6
    return float(tick)/TICKS_PER_SECOND

for i in range(1, len(vals)):
    t = vals[i][0]
    j = i
    su = 1
    while j > 0 and t - vals[j][0] < 10**6:
        j -= 1
        su += vals[j][1]

    dt = t - vals[j][0]
    print tickToTime(t), float(su)/dt

