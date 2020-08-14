
class trackingPD:
    def __init__(
        self,
        Kp=1.0,
        Ki=0.0,
        Kd=0.0,
        targetpos=0.0,
        targetvel=0.0
    ):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.targetpos = targetpos
        self.targetvel = targetvel

    def setTargetPoint(self, targetpos, targetvel):
        self.targetpos = targetpos
        self.targetvel = targetvel

    def tunings(self, Kp, Ki, Kd):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd

    def calculate(self, pos, vel):
        err_p = self.targetpos - pos
        err_v = self.targetvel - vel

        p = self.Kp * err_p
        d = self.Kd * err_v

        u = p + d
        return u

