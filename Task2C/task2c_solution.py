#python

reference_frame = 15
spherical_joint = 16
bike_respondable = 17
front_motor = 18
yaw_setpoint = 0
x6 = 0
drive_speed = 0
def sysCall_init():
    
    # do some initialization here
    pass

def sysCall_actuation():
    global reference_frame
    global spherical_joint
    global bike_respondable
    global front_motor
    global yaw_setpoint
    global x6
    global drive_speed
    print("YAW setpoint = ")
    print(yaw_setpoint)
    x1 = sim.getJointVelocity(front_motor)
    x5=sim.getObjectOrientation(bike_respondable,reference_frame)
    x6=sim.alphaBetaGammaToYawPitchRoll(x5[0],x5[1],x5[2])
    x4=(x6[1])
    x2= (yaw_setpoint - x6[0])
    A,B = sim.getObjectVelocity(bike_respondable)
    x3=B[1]
    k=[-1.006385 , -0.051623  , 0.890870 ,  9.352082]
    U = -k[0]*x1 - k[1]*x2 + k[2]*x3 + k[3]*x4
    sim.setJointTargetVelocity(front_motor,U)
    sim.setJointTargetVelocity(23,drive_speed)
    #print(x6[0])
    #print(U)
    # put your actuation code here
    pass

def sysCall_sensing():
    global yaw_setpoint
    global drive_speed
    global x6
    ############### Keyboard Input ##############
    ############### Keyboard Input ##############
    message,data,data2 = sim.getSimulatorMessage()
    
    if (message == sim.message_keypress):    
        if (data[0]==2007): # forward up arrow
            drive_speed = -20 #add drive wheel speed here
        
        if (data[0]==2008): # backward down arrow
            drive_speed = 20#add drive wheel speed here
        
        if (data[0]==2009): # left arrow key
            yaw_setpoint = 3#change yaw_setpoint for required turning over here
                
        if (data[0]==2010): # right arrow key
            yaw_setpoint = -3#change yaw_setpoint for required turning over here
    else:
        drive_speed = 0# This is an example, decide what's best
        yaw_setpoint = x6[0]# # This is an example, decide what's best
    #########################################
    # put your sensing code here
    pass

def sysCall_cleanup():
    # do some clean-up here
    pass

# See the user manual or the available code snippets for additional callback functions and details
