# noinspection PyAttributeOutsideInit
class PID:
    def __init__(self, kp, ki, kd, timeStep, setValue, integralRange, calculateFlag):
        """ PID class, where ki = kp/ti and kd = kp*td set as input
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
        self.calculateDictionary = {"noFlush": self.calculate_no_clear, "signChange": self.calculate_error_sign,
                                    "rangeExit": self.calculate_range_exit}
        self.calculate = self.calculateDictionary[calculateFlag]

    def calculate_no_clear(self, controlValue):

        error = self.setValue - controlValue
        self.accumulate_error(error, controlValue)

        if self.previousControlVal != 0:
            controlValDiff = controlValue - self.previousControlVal
        else:
            controlValDiff = 0

        output = self.kp * error + self.ki * self.integralError * self.dt - self.kd * controlValDiff / self.dt
        self.previousControlVal = controlValue

        return output

    def calculate_error_sign(self, controlValue):
        error = self.setValue - controlValue

        errorSign = self.previousError * error

        if errorSign < 0:
            self.integralError = 0
        else:
            self.accumulate_error(error, controlValue)

        if self.previousControlVal != 0 :
            controlValDiff = controlValue - self.previousControlVal
        else:
            controlValDiff = 0

        output = self.kp * error + self.ki * self.integralError * self.dt - self.kd * controlValDiff / self.dt
        self.previousControlVal = controlValue

        if error != 0:
            self.previousError = error

        return output

    def calculate_range_exit(self, controlValue):
        error = self.setValue - controlValue

        if controlValue > self.integralRange[1] or controlValue < self.integralRange[0]:
            self.integralError = 0
        else:
            self.accumulate_error(error, controlValue)

        if self.previousControlVal != 0:
            controlValDiff = controlValue - self.previousControlVal
        else:
            controlValDiff = 0

        output = self.kp * error + self.ki * self.integralError * self.dt - self.kd * controlValDiff / self.dt
        self.previousControlVal = controlValue

        return output

    def accumulate_error(self, error, controlValue):

        if self.integralRange[0] < controlValue < self.integralRange[1]:
            self.integralError += error
