#This section imports the important libraries to be used.
import numpy as np                     #for numerical operation with arrays
import matplotlib.pyplot as plt        #for plotting graph
import support_files_car as sfc        #contains support fxn and constants
import matplotlib.gridspec as gridspec #grid layout to plot subplots within a fig



# Create an object for the support functions.
support=sfc.SupportFilesCar()   #SupportFilesCar() class provides fxn and constants
constants=support.constants

# Load the constant values needed in the main file
Ts=constants['Ts']                   # sampling time
outputs=constants['outputs']         # number of outputs (psi, Y)
x_dot=constants['x_dot']             # constant longitudinal velocity
time_length=constants['time_length'] # duration of the manoeuvre
PID_switch = constants['PID_switch'] 

# Generate the refence signals
t=np.arange(0,time_length+Ts,Ts) # time from 0 to 10 seconds, sample time (Ts=0.02 second)
r=constants['r']
f=constants['f']
psi_ref,X_ref,Y_ref=support.trajectory_generator(t,r,f)   #import trajectories
sim_length=len(t)                 # Number of control loop iterations


# Load the initial states
y_dot=0.
psi=0.
psi_dot=0.
Y=Y_ref[0]+0.

states=np.array([y_dot,psi,psi_dot,Y])
statesTotal=np.zeros((len(t),len(states))) # It will keep track of all your states during the entire manoeuvre
statesTotal[0][0:len(states)]=states

# Load the initial input
U1=0 # Input at t = -1 s (steering wheel angle in rad (delta))
UTotal=np.zeros(len(t)) # To keep track all your inputs over time
UTotal[0]=U1


# Initiate the controller - simulation loops
for i in range(0,sim_length-1):



    ######################### PID #############################################

    if i==0:
        e_int_pid_yaw=0
        e_int_pid_Y=0
    if i>0:
        e_pid_yaw_im1=psi_ref[i-1]-old_states[1]
        e_pid_yaw_i=psi_ref[i]-states[1]
        e_dot_pid_yaw=(e_pid_yaw_i-e_pid_yaw_im1)/Ts
        e_int_pid_yaw=e_int_pid_yaw+(e_pid_yaw_im1+e_pid_yaw_i)/2*Ts
        Kp_yaw=constants['Kp_yaw']
        Kd_yaw=constants['Kd_yaw']
        Ki_yaw=constants['Ki_yaw']
        U1_yaw=Kp_yaw*e_pid_yaw_i+Kd_yaw*e_dot_pid_yaw+Ki_yaw*e_int_pid_yaw

        e_pid_Y_im1=Y_ref[i-1]-old_states[3]
        e_pid_Y_i=Y_ref[i]-states[3]
        e_dot_pid_Y=(e_pid_Y_i-e_pid_Y_im1)/Ts
        e_int_pid_Y=e_int_pid_Y+(e_pid_Y_im1+e_pid_Y_i)/2*Ts
        Kp_Y=constants['Kp_Y']
        Kd_Y=constants['Kd_Y']
        Ki_Y=constants['Ki_Y']
        U1_Y=Kp_Y*e_pid_Y_i+Kd_Y*e_dot_pid_Y+Ki_Y*e_int_pid_Y

        U1=U1_yaw+U1_Y


    old_states=states
    ######################### PID END #########################################

    # Establish the limits for the real inputs (max: pi/6 radians)

    if U1 < -np.pi/6:
        U1=-np.pi/6
    elif U1 > np.pi/6:
        U1=np.pi/6
    else:
        U1=U1

    # Keep track of your inputs as you go from t=0 --> t=7 seconds
    UTotal[i+1]=U1

    # Compute new states in the open loop system (interval: Ts/30)
    states=support.open_loop_new_states(states,U1)
    statesTotal[i+1][0:len(states)]=states
    # print(i)





# Plot the world
plt.plot(X_ref,Y_ref,'b',linewidth=2,label='The trajectory')
plt.plot(X_ref,statesTotal[:,3],'--r',linewidth=2,label='Car position')
plt.xlabel('x-position [m]',fontsize=15)
plt.ylabel('y-position [m]',fontsize=15)
plt.grid(True)
plt.legend(loc='upper right',fontsize='small')
plt.ylim(-X_ref[-1]/2,X_ref[-1]/2) # Scale roads (x & y sizes should be the same to get a realistic picture of the situation)
plt.show()


# Plot the the input delta(t) and the outputs: psi(t) and Y(t)
plt.subplot(3,1,1)
plt.plot(t,UTotal[:],'r',linewidth=2,label='steering wheel angle')
plt.xlabel('t-time [s]',fontsize=12)
plt.ylabel('steering wheel angle [rad]',fontsize=12)
plt.grid(True)
plt.legend(loc='upper right',fontsize='small')

plt.subplot(3,1,2)
plt.plot(t,psi_ref,'b',linewidth=2,label='Yaw_ref angle')
plt.plot(t,statesTotal[:,1],'--r',linewidth=2,label='Car yaw angle')
plt.xlabel('t-time [s]',fontsize=12)
plt.ylabel('psi_ref-position [rad]',fontsize=12)
plt.grid(True)
plt.legend(loc='center right',fontsize='small')

plt.subplot(3,1,3)
plt.plot(t,Y_ref,'b',linewidth=2,label='Y_ref position')
plt.plot(t,statesTotal[:,3],'--r',linewidth=2,label='Car Y position')
plt.xlabel('t-time [s]',fontsize=12)
plt.ylabel('y-position [m]',fontsize=12)
plt.grid(True)
plt.legend(loc='center right',fontsize='small')
plt.show()


