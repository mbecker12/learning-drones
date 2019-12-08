# noinspection PyAttributeOutsideInit
class PID:
    def __init__(self, kc, ti, td, timeStep, setValue, integralRange, calculateFlag):
        self.kc = kc
        self.ki = kc / ti
        self.kd = kc * td
        self.dt = timeStep
        self.integralError = 0
        self.integralRange = integralRange
        self.setValue = setValue
        self.previousControlVal = 0
        self.calculateDictionary = {"noFlush": self.calculate_no_clear, "signChange": self.calculate_error_sign,
                                    "rangeExit": self.calculate_range_exit}
        self.calculate = self.calculateDictionary[calculateFlag]

    def calculate_no_clear(self, controlValue):

        error = self.setValue - controlValue
        self.accumulate_error(error, controlValue)
        controlValDiff = controlValue - self.previousControlVal
        output = self.kc * error + self.ki * self.integralError * self.dt - self.kd * controlValDiff / self.dt
        self.previousControlVal = controlValue

        return output

    def calculate_error_sign(self, controlValue):
        error = self.setValue - controlValue

        previousError = self.integralError

        if previousError * error < 0:
            self.integralError = 0
        else:
            self.accumulate_error(error, controlValue)

        controlValDiff = controlValue - self.previousControlVal
        output = self.kc * error + self.ki * self.integralError * self.dt - self.kd * controlValDiff / self.dt
        self.previousControlVal = controlValue

        return output

    def calculate_range_exit(self, controlValue):
        error = self.setValue - controlValue

        if controlValue > self.integralRange[1] or controlValue < self.integralRange[0]:
            self.integralError = 0
        else:
            self.accumulate_error(error, controlValue)

        controlValDiff = controlValue - self.previousControlVal
        output = self.kc * error + self.ki * self.integralError * self.dt - self.kd * controlValDiff / self.dt
        self.previousControlVal = controlValue

        return output

    def accumulate_error(self, error, controlValue):

        if self.integralRange[0] < controlValue < self.integralRange[1]:
            self.integralError += error
