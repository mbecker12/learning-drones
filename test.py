"""
created by Jan Schiffeler on 24.11.12019
changed by

python 3.6.5
numpy 1.


"""
from PID import PID

if __name__ == "__main__":

    t = PID(5, 5, 5, 0.05, 100, [60, 140], 1, 1, 10)

    out = t.calculate(50)

