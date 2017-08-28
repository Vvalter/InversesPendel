f = open('MotorGrosGetriebeLeerlauf2.log', 'r')
lines = f.readlines()

vals = []
for l in lines:
    a,b = map(int, l.split())
    vals.append( (a,b)) 


def tickToTime(tick):
    TICKS_PER_SECOND = 42*10**6
    return float(tick)/TICKS_PER_SECOND

num = 1
for i in range(num, len(vals)):
    dt = vals[i][0] - vals[i-num][0]
    dp = float(vals[i][1])
    print tickToTime(vals[i][0]), (dp/tickToTime(dt))/2400
