# noinspection PyAttributeOutsideInit
import numpy as np


class PID:
    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        timeStep: float,
        setValue,
        calculateFlag,
        integralRange: float = 2,
        outputLimitRange=[-1, 1],
    ):
        """ PID class, where ki = kp/ti and kd = kp*td  constants set  from the parameters.py file
            :param setValue  is the value we want the PID to reach

            :param calculateFlag is  string that corresponds to a differentiation of the calculate function with the help
            of a dictionary defined in the constructor as calculateDictionary
            :param outputLimitRange is an array with the limits of the output, will be set constant values in the parameters file
            :function calculate is the method we use to find the output of the PID
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = timeStep
        self.integralError = 0
        self.integralRange = integralRange
        self.setValue = setValue
        self.previousError = 0
        self.previousControlVal = 0
        self.outputLimitRange = outputLimitRange
        self.calculateDictionary = {
            "noFlush": self.calculate_no_clear,
            "signChange": self.calculate_error_sign,
            "rangeExit": self.calculate_range_exit,
        }
        self.calculate = self.calculateDictionary[calculateFlag]

    def calculate_no_clear(self, controlValue):
        """
        This is a method to calculate output without flushing of error.
        The derivative part at the first time of calculate call is 0.
        :param controlValue: PV
        """
        error = self.setValue - controlValue
        self.accumulate_error(error)

        if self.previousControlVal != 0:
            controlValDiff = controlValue - self.previousControlVal
        else:
            controlValDiff = 0

        output = (
            self.kp * error
            + self.ki * self.integralError * self.dt
            - self.kd * controlValDiff / self.dt
        )
        self.previousControlVal = controlValue

        if self.previousControlVal != 0:
            controlValDiff = controlValue - self.previousControlVal
        else:
            controlValDiff = 0

        output = (
            self.kp * error
            + self.ki * self.integralError * self.dt
            - self.kd * controlValDiff / self.dt
        )
        self.previousControlVal = controlValue

        return self.check_output(output)

    def calculate_error_sign(self, controlValue):
        """
        Calculate but flush error when the previous error and the current error have different signs.
        (When error is 0 it is not stored as previous error)
        """
        error = self.setValue - controlValue

        errorSign = self.previousError * error

        if errorSign < 0:
            self.integralError = 0

        self.accumulate_error(error)

        if self.previousControlVal != 0:
            controlValDiff = controlValue - self.previousControlVal
        else:
            controlValDiff = 0

        output = (
            self.kp * error
            + self.ki * self.integralError * self.dt
            - self.kd * controlValDiff / self.dt
        )
        self.previousControlVal = controlValue

        if error != 0:
            self.previousError = error

        return self.check_output(output)

    def calculate_range_exit(self, controlValue):
        """
        Same calculate, but flush error when error is greater than the integral range.
        """
        error = self.setValue - controlValue

        if np.abs(error) > self.integralRange:
            self.integralError = 0

        self.accumulate_error(error)

        if self.previousControlVal != 0:
            controlValDiff = controlValue - self.previousControlVal
        else:
            controlValDiff = 0

        output = (
            self.kp * error
            + self.ki * self.integralError * self.dt
            - self.kd * controlValDiff / self.dt
        )
        self.previousControlVal = controlValue
        return self.check_output(output)

    def accumulate_error(self, error):
        if np.abs(error) < self.integralRange:
            self.integralError += error

    def check_output(self, output):
        if output < self.outputLimitRange[0]:
            output = self.outputLimitRange[0]
        elif output > self.outputLimitRange[1]:
            output = self.outputLimitRange[1]

        return output

    def set_setpoint(self, point):
        self.setValue = point
