import argparse

parser = argparse.ArgumentParser(description='calculate and print velocity from stance length and duration.')
# parser.add_argument('--duration', help='list of average stance durations')
parser.add_argument("-d", "--duration", nargs=6, metavar=('a', 'b', 'c', 'd', 'e', 'f'),
        help="list of average stance duration for the six legs [lf, rf, lm, rm, lr, rr]", type=float)
# parser.add_argument('--length', help='list of average stance length')
parser.add_argument("-l", "--length", nargs=6, metavar=('h', 'i', 'j', 'k', 'l', 'm'),
        help="list of average stance lengths for the six legs [lf, rf, lm, rm, lr, rr]", type=float)

args = parser.parse_args()

durations = args.duration
lengths = args.length

velocity = [lengths[i] / durations[i] for i in range(0, len(lengths))]
text = ' & '.join([str(round(vel, 3)) for vel in velocity])

print(text, end="")
