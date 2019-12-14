"""
created by Jan Schiffeler on 24.11.12019
changed by

python 3.6.5
numpy 1.


"""
from PID import PID
import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":

    t = PID(kp=2.5, ki=0, kd=0, timeStep=0.05, setValue=10, integralRange=4, calculateFlag="rangeExit")

    x = np.arange(6,20,1)
    input = 15*np.sin(x)
    out = []
    for o in input:
        out.append(t.calculate(o))

    plt.figure()
    plt.plot(x,input)
    plt.plot(x,out)
    plt.legend(["input","output"])
    plt.show()


