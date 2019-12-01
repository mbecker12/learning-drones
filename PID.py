# noinspection PyAttributeOutsideInit
class PID:
    def __init__(self, kc, ti, td, timeStep, setValue, integralRange, integralFlag, clearErrorFlag, bias):
        self.kc = kc
        self.ti = ti
        self.td = td
        self.dt = timeStep
        self.integralError = 0
        self.integralRange = integralRange
        self.setValue = setValue
        self.previousControlVal = 0
        self.integralFlag = integralFlag
        self.clearErrorFlag = clearErrorFlag
        self.bias = bias

    def calculate(self, controlValue):

        error = self.setValue - controlValue

        previousError = self.integralError

        if self.integralFlag == 0:
            self.integralError += error
        elif controlValue < self.integralRange[1] or controlValue > self.integralRange[0]:
            self.integralError += error

        if self.clearErrorFlag == 0 and (previousError > 0 and self.integralError < 0) or (previousError < 0 and self.integralError > 0):
            self.integralError = 0
        elif self.clearErrorFlag == 1 and (controlValue > self.integralRange[1] or controlValue < self.integralRange[0]):
            self.integralError = 0

        controlValDiff = controlValue - self.previousControlVal
        output = self.kc * error + self.kc / self.ti * self.integralError * self.dt - self.kc * self.ti * controlValDiff / self.dt
        self.previousControlVal = controlValue

        return output
