from math import *
import numpy as np
import cv2
import time
from droneapi.lib import VehicleMode, Location

def fProject(x,P_M,K):
    ax, ay, az, tx, ty, tz = x[0], x[1], x[2], x[3], x[4], x[5]

    Rx = np.array([[1,0,0],[0,cos(ax),-sin(ax)],[0,sin(ax),cos(ax)]],float)
    Ry = np.array([[cos(ay),0,sin(ay)],[0,1,0],[-sin(ay),0,cos(ay)]],float)
    Rz = np.array([[cos(az),-sin(az),0],[sin(az),cos(az),0],[0,0,1]],float)
    R = np.dot(Rz,np.dot(Ry,Rx))

    t = np.array([[tx],[ty],[tz]])
    Mext = np.concatenate((R,t),axis=1)

    ph = np.dot(np.dot(K,Mext),P_M)
    ph1=np.divide(ph[0],ph[2])
    ph2=np.divide(ph[1],ph[2])
    
    a=np.array([ph1,ph2])
    a=np.reshape(a, np.shape(P_M)[1]*2, order='F')
    
    return a

def find_center(color,colorname):
    contours,hierarchy = cv2.findContours(color,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    
    if len(contours)>0:
        # find biggest blob
        max_area = 0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                best_blob = contour
        if max_area > 10:
            # find centroid
            M = cv2.moments(best_blob)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(im,(cx,cy),3,255,-1)
            (xcenter,ycenter),(MA,ma),angle = cv2.fitEllipse(best_blob)
        else:
            print colorname+" too small"
        return(int(xcenter),int(ycenter))
    else:
        print colorname+" not found!"
        return(0,0)

def nothing(xx):
    pass

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




# Start of program
# Initialiseer camera, APM en matrices
cap = cv2.VideoCapture(0)

x=np.array([0,0,0,0,0,20])
P_M = np.array([[0,0,10,10,25],[0,15,15,0,0],[0,0,0,0,0],[1,1,1,1,1]])
f=813
cx=340
cy=224
K = np.array([[f,0,cx],[0,f,cy],[0,0,1]])

api = local_connect()
print "connected"

v = api.get_vehicles()[0]
print "vehicle found"

if v.mode.name == "INITIALISING":
    print "Vehicle still booting, try again later"
    return

# Initialiseer controller
c = Controller(2)
    
while(True):
    # Beeldacquisitie
    ret, img = cap.read()

    # Beeldverwerking
    im = cv2.blur(img,(3,3))
    
    # 1. Filter/threshhold op HSV range
    hsv = cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(hsv,np.array((15, 85, 85)), np.array((30, 255, 255)))
    orange = cv2.inRange(hsv,np.array((0, 85, 85)), np.array((15, 255, 255)))
    blue = cv2.inRange(hsv,np.array((50, 85, 85)), np.array((110, 255, 255)))
    pink = cv2.inRange(hsv,np.array((120, 85, 85)), np.array((170, 255, 255)))
    green = cv2.inRange(hsv,np.array((40, 85, 85)), np.array((50, 255, 255)))
    thresh2 = yellow.copy()

    # 2. Vind zwaartepunt van de blobs
    c_yellow=find_center(yellow,"yellow")
    c_blue=find_center(blue,"blue")
    c_orange=find_center(orange,"orange")
    c_green=find_center(green,"green")
    c_pink=find_center(pink,"pink")

    # 3. Maak een array van de getrackte punten
    y0 = [[c_blue[0]],[c_blue[1]],[c_orange[0]],[c_orange[1]],[c_pink[0]],[c_pink[1]],[c_yellow[0]],[c_yellow[1]],[c_green[0]],[c_green[1]]]

    # Projecteer model
    y = fProject(x,P_M,K)

    for i in xrange(0,len(y),2):
        cv2.rectangle(img,(int(y[i])-2,int(y[i+1])-2),(int(y[i])+2,int(y[i+1])+2),[0,96,144],-1)

    e = 0.00001
    J1 = (( fProject(x+[e,0,0,0,0,0],P_M,K) - y )/e)[np.newaxis,:].T
    J2 = (( fProject(x+[0,e,0,0,0,0],P_M,K) - y )/e)[np.newaxis,:].T
    J3 = (( fProject(x+[0,0,e,0,0,0],P_M,K) - y )/e)[np.newaxis,:].T
    J4 = (( fProject(x+[0,0,0,e,0,0],P_M,K) - y )/e)[np.newaxis,:].T
    J5 = (( fProject(x+[0,0,0,0,e,0],P_M,K) - y )/e)[np.newaxis,:].T
    J6 = (( fProject(x+[0,0,0,0,0,e],P_M,K) - y )/e)[np.newaxis,:].T
    J=np.column_stack((J1,J2,J3,J4,J5,J6))

    dy = y0-(y[np.newaxis,:].T)
    print "The residual error="+str(np.linalg.norm(dy))
    pinvj=np.linalg.pinv(J)
    dx=np.dot(pinvj,dy)

    # Accuraatheidstest
    if abs(np.linalg.norm(dx)/np.linalg.norm(x))< 0.000001:
        # Accuraat
        rollPWM, pitchPWM, yawPWM, throttlePWM = c.control_angles(y[0], y[1], y[2], y[3], y[4], y[5])

        # Check of autonome navigatie wordt toegestaan door de piloot
        if (v.mode.name == "STABILIZE") and (v.channel_readback['6'] > 1600) :
            print "Overriding a RC channel"
            v.channel_override = { "1" : rollPWM, "2" : pitchPWM, "3" : throttlePWM, "4" : yawPWM }
            v.flush()
            print "Current overrides are:", v.channel_override
            print "RC readback:", v.channel_readback
            time.sleep(1)
        
    else:
        # Update state estimation
        dx=np.transpose(dx)
        dx=dx[0,:]
        x=x+dx
    
# Sluit camera's en eventuele openstaande windows
cap.release()
cv2.destroyAllWindows()
