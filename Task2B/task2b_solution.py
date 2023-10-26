#python

reference_frame = 15
spherical_joint = 16
bike_respondable = 17
front_motor = 18
yaw_setpoint = 0


def sysCall_init():

    
    # do some initialization here
    pass

def sysCall_actuation():
    global reference_frame
    global spherical_joint
    global bike_respondable
    global front_motor
    global yaw_setpoint
    print("YAW setpoint = ")
    print(yaw_setpoint)
    x1 = sim.getJointVelocity(front_motor)
    x5=sim.getObjectOrientation(bike_respondable,reference_frame)
    x6=sim.alphaBetaGammaToYawPitchRoll(x5[0],x5[1],x5[2])
    x4=(x6[1])
    x2= (yaw_setpoint - x6[0] - 0.002)
    A,B = sim.getObjectVelocity(bike_respondable)
    x3=B[1]
    k=[-1.03379 , -0.8 ,  0.82359  , 9.36946]
    U = -k[0]*x1 - k[1]*x2 + k[2]*x3 + k[3]*x4
    sim.setJointTargetVelocity(front_motor,U)
    #print(x2)
    #print(x6[0])
    #print(U)
    # put your actuation code here
    pass

def sysCall_sensing():
    global yaw_setpoint
    yaw_setpoint = sim.getFloatSignal("yaw_setpoint")
    #yaw_setpoint = 0.5236009999999999
    # put your sensing code here
    pass

def sysCall_cleanup():
    # do some clean-up here
    pass

# See the user manual or the available code snippets for additional callback functions and details

