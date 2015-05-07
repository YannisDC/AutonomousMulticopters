class Controller():
    def __init__(self, pGain):
        self.leanAngleMax = 45.0 # degrees
        self.yawRate = 200.0 # degrees per second
        self.rollValueMin = 1132
        self.rollValueMid = 1525
        self.rollValueMax = 1927
        self.pitchValueMin = 1126
        self.pitchValueMid = 1521
        self.pitchValueMax = 1921
        self.yawValueMin = 1128
        self.yawValueMid = 1525
        self.yawValueMax = 1923
        self.throttleValueMin = 1123
        self.throttleValueMax = 1921
        self.Kp = pGain
        self.Ki = 0.5
        self.Kd = 0.5

        self.rollP = float(self.rollValueMax - self.rollValueMin) / (2 * self.leanAngleMax) # roll parameter, PWM change per degree
        self.pitchP = float(self.pitchValueMax - self.pitchValueMin) / (2 * self.leanAngleMax) # pitch parameter, PWM change per degree
        self.yawP = float(self.yawValueMax - self.yawValueMin) / (2 * self.yawRate) # parameter, PWM change per degree/second

    def control_angles(self, roll, pitch, yaw, x, y, z):
        rollDesired = int(round( self.rollValueMid - (y/100 * self.Kp) * self.rollP ))# FIXME, if perfect cannot devide by 0, setpoint is y=0
        pitchDesired = int(round( self.pitchValueMid - (x/100 * self.Kp) * self.pitchP )) # FIXME, if perfect cannot devide by 0, setpoint is x=0
        yawDesired = self.yawValueMid
        throttleDesired = 1300
        return rollDesired, pitchDesired, yawDesired, throttleDesired

c = Controller(2)
for i in range(-400,401,100):
    for j in range(-400,401,100):
        print c.control_angles(1,1,1,float(i),float(j),1) # All floats as input
