import numpy as np
import matplotlib.pyplot as plt

class SupportFilesCar:   #collection of functions interact with main file

    def __init__(self):
        ''' Load the constants that do not change'''

        # Constants
        m=1500
        Iz=3000
        Caf=19000
        Car=33000
        lf=2
        lr=3
        Ts=0.02


        outputs=2 # number of outputs
        x_dot=20 # car's longitudinal velocity
        lane_width=7 # [m]
        nr_lanes=5 # 6 lanes [m]
        
        r=4 #amplitude of sinusoidal fnc
        f=0.01 #frequency of sinusoidal dnc
        time_length = 10 # [s] - duration of the entire manoeuvre

        PID_switch = 1

        Kp_yaw=700
        Kd_yaw=3
        Ki_yaw=5

        Kp_Y=70
        Kd_Y=3
        Ki_Y=5

        ### PID END ###

        trajectory=3 # You should only choose 1,2,3,4,5

        self.constants={'m':m, 'Iz':Iz, 'Caf':Caf, 'Car':Car, 'lf':lf, 'lr':lr,\
            'Ts':Ts,'outputs':outputs, 'x_dot':x_dot,\
            'r':r, 'f':f, 'time_length':time_length, 'lane_width':lane_width,\
            'PID_switch':PID_switch, 'Kp_yaw':Kp_yaw, 'Kd_yaw':Kd_yaw, 'Ki_yaw':Ki_yaw,\
            'Kp_Y':Kp_Y, 'Kd_Y':Kd_Y, 'Ki_Y':Ki_Y, 'trajectory':trajectory}

        return None

    def trajectory_generator(self,t,r,f):
        '''This method creates the trajectory for a car to follow'''

        Ts=self.constants['Ts']
        x_dot=self.constants['x_dot']
        trajectory=self.constants['trajectory']

        # Define the x length, depends on the car's longitudinal velocity
        x=np.linspace(0,x_dot*t[-1],num=len(t))

        # Define trajectories

        if trajectory==1:           #straight line
            y=-9*np.ones(len(t))    
        elif trajectory==2:         #hyperbolic
            y=9*np.tanh(t-t[-1]/2)  
        elif trajectory==3:         #sinusoidal 
            y=1*r*np.sin(2*np.pi*f*x)
        elif trajectory==4:         #parabolic
            y=0.0025*x*x
        elif trajectory==5:         #parabolic
            y=2.5*np.sqrt(x)          
        else:
            print("For trajectories, only choose 1, 2, 3, 4, or 5 as an integer value")
            exit()

        # Vector of x and y changes per sample time
        dx=x[1:len(x)]-x[0:len(x)-1]
        dy=y[1:len(y)]-y[0:len(y)-1]

        # Define the reference yaw angles
        psi=np.zeros(len(x))
        psiInt=psi
        psi[0]=np.arctan2(dy[0],dx[0])
        psi[1:len(psi)]=np.arctan2(dy[0:len(dy)],dx[0:len(dx)])

        # We want the yaw angle to keep track the amount of rotations
        dpsi=psi[1:len(psi)]-psi[0:len(psi)-1]
        psiInt[0]=psi[0]
        for i in range(1,len(psiInt)):
            if dpsi[i-1]<-np.pi:
                psiInt[i]=psiInt[i-1]+(dpsi[i-1]+2*np.pi)
            elif dpsi[i-1]>np.pi:
                psiInt[i]=psiInt[i-1]+(dpsi[i-1]-2*np.pi)
            else:
                psiInt[i]=psiInt[i-1]+dpsi[i-1]

        return psiInt,x,y



    def open_loop_new_states(self,states,U1):
        '''This function computes the new state vector for one sample time later'''

        # Get the necessary constants
        m=self.constants['m']
        Iz=self.constants['Iz']
        Caf=self.constants['Caf']
        Car=self.constants['Car']
        lf=self.constants['lf']
        lr=self.constants['lr']
        Ts=self.constants['Ts']
        x_dot=self.constants['x_dot']

        current_states=states
        new_states=current_states
        y_dot=current_states[0]
        psi=current_states[1]
        psi_dot=current_states[2]
        Y=current_states[3]

        sub_loop=30  #Chop Ts into 30 pieces
        for i in range(0,sub_loop):
            # Compute the the derivatives of the states
            y_dot_dot=-(2*Caf+2*Car)/(m*x_dot)*y_dot+(-x_dot-(2*Caf*lf-2*Car*lr)/(m*x_dot))*psi_dot+2*Caf/m*U1
            psi_dot=psi_dot
            psi_dot_dot=-(2*lf*Caf-2*lr*Car)/(Iz*x_dot)*y_dot-(2*lf**2*Caf+2*lr**2*Car)/(Iz*x_dot)*psi_dot+2*lf*Caf/Iz*U1
            Y_dot=np.sin(psi)*x_dot+np.cos(psi)*y_dot

            # Update the state values with new state derivatives
            y_dot=y_dot+y_dot_dot*Ts/sub_loop
            psi=psi+psi_dot*Ts/sub_loop
            psi_dot=psi_dot+psi_dot_dot*Ts/sub_loop
            Y=Y+Y_dot*Ts/sub_loop

        # Take the last states
        new_states[0]=y_dot
        new_states[1]=psi
        new_states[2]=psi_dot
        new_states[3]=Y

        return new_states
