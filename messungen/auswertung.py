import sys

def tickToTime(tick):
    TICKS_PER_SECOND = 42*10**6
    return float(tick)/TICKS_PER_SECOND


if len(sys.argv) != 2:
    print "usage: python auswertung.py <filename>"
    sys.exit(1)

filename = sys.argv[1]
f = open(filename, 'r')

lines = f.readlines()
if lines[0].startswith('Start'):
    lines.pop(0)
if lines[0].startswith('START'):
    lines.pop(0)
if lines[-1].startswith('END'):
    lines.pop()

vals = [map(int, l.split()) for l in lines]

num = 100
for i in range(num, len(vals)):
    dt = vals[i][0] - vals[i-num][0]
    dp = 0
    for j in range(0, num):
        dp += vals[i-j][1]

    #print dt, dp
    print tickToTime(vals[i][0]), (dp/tickToTime(dt))/2400
