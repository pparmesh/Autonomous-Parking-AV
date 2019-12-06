"""
The LQR controller for lateral control of vehicles in Carla.
Author: Ashish Roongta
SafeAI lab
Carnegie Mellon University
Copyright @ SafeAI lab-Carnegie Mellon University
"""
import numpy as np
import matplotlib.pyplot as plt
import control
import os
import scipy
from scipy.ndimage import gaussian_filter1d
import scipy.signal
import time

class Controller2D(object):
    def __init__(self, vehicle,waypoints,carla):
        self._vehicle=vehicle
        self._controller=carla.VehicleControl()
        loc=vehicle.get_transform()
        self._current_x          = loc.location.x
        self._current_y          = loc.location.y
        self._current_yaw        = loc.rotation.yaw
        self._current_vX      = 0
        self._desired_vY      = 0
        # self._current_frame      = 0
        # self._current_timestamp  = 0
        self._start_control_loop = True
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._waypoints[:,1]=-self._waypoints[:,1]
        self._conv_rad_to_steer  = 0.28
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi
        self._lr=1.5   #length from rear tire to center of mass
        self._lf=1.0    # length from front tire to the center of mass
        self._Ca=13000  # cornering stiffness of each tire
        self._Iz=3500   # Yaw inertia
        self._f=0.3   # friction coefficient
        phy=vehicle.get_physics_control()
        # phy.mass=2000
        # vehicle.apply_physics_control(phy)
        self._m=phy.mass  # mass of the vehicle
        self._g=10  # acceleration to the gravity (m/s^2)
        self._last_x=0.0  #to store the last x position of the vehicle
        self._last_y=0.0  # to store the previous y position of the vehicle
        self._t=time.time()
        self._last_timestamp=0.0
        self._dt=1/30   # dt for fixed time step, at 30 fps
        self._last_yaw=0.0
        self._look_ahead=10
        self._curv_ld=10
        self._frame=0.0
        self._last_vx_error=0.0
        self.v_desired=2*np.ones(self._waypoints.shape[0])
        self.traj_curv = self.curvature(self._waypoints)


    def update_values(self):
        loc=self._vehicle.get_transform()
        self._current_x         = loc.location.x
        self._current_y         = loc.location.y
        self._current_yaw       = self._vehicle.get_transform().rotation.yaw
        self._current_vX     = self._vehicle.get_velocity().x
        self._current_vY=self._vehicle.get_velocity().y
        self._frame+=1
        # self._current_timestamp = timestamp
        # self._current_frame     = frame
        # if self._current_frame:
        #     self._start_control_loop = True

    def dlqr(self,A,B,Q,R):
        '''
        Function to solve the Ricardi equation
        ref http://www.kostasalexis.com/lqr-control.html
        '''
        X=np.matrix(scipy.linalg.solve_discrete_are(A,B,Q,R))
        #  Computing the LQR gain
        K=np.matrix(scipy.linalg.inv(B.T*X*B+R)*(B.T*X*A))
        return K
    
    def PID_longitudanal(self,dt,vx_error):
        """
        Function to compute the throttle and the brake output using a PID controller.
        """
        kp=15
        kd=0.1
        ki=-150
        integral_error=vx_error+self._last_vx_error
        derivative_error=vx_error-self._last_vx_error
        delta=kp*vx_error+ki*integral_error*dt+kd*derivative_error/dt
        if delta>0:
            throttle_output=delta
            brake_output=0.0
        else:
            throttle_output=0.0
            brake_output=0.5

        self._last_vx_error=vx_error
        return throttle_output,brake_output


    def find_nearest_points(self,X, Y, traj):
        dist_sq = np.zeros(traj.shape[0])
        for j in range(traj.shape[0]):
            dist_sq[j] = (traj[j,0] - X)**2 + (traj[j,1] - Y)**2
        minDistSqure, minIdx = min((dist_sq[i], i) for i in range(len(dist_sq)))
        return np.sqrt(minDistSqure), minIdx
    
    def curvature(self,waypoints):
        '''
        Function to compute the curvature of the reference trajectory.
        Returns an array containing the curvature at each waypoint
        '''
        # waypoints=np.asarray(waypoints)
        x=waypoints[:,0]
        y=waypoints[:,1]
        sig=10
        # xp = scipy.ndimage.filters.gaussian_filter1d(input=x,sigma=sig,order=1)
        # xpp = scipy.ndimage.filters.gaussian_filter1d(input=x,sigma=sig,order=2)
        # yp = scipy.ndimage.filters.gaussian_filter1d(input=y,sigma=sig,order=1)
        # ypp = scipy.ndimage.filters.gaussian_filter1d(input=y,sigma=sig,order=2)
        # curv=np.zeros(len(waypoints))
        # for i in range(len(xp)):
        #     curv[i] = ((xp[i]*ypp[i] - yp[i]*xpp[i])/(xp[i]**2 + yp[i]**2)**1.5)

        x1=gaussian_filter1d(x,sigma=sig,order=1,mode="wrap")
        x2=gaussian_filter1d(x1,sigma=sig,order=1,mode="wrap")
        y1=gaussian_filter1d(y,sigma=sig,order=1,mode="wrap")
        y2=gaussian_filter1d(y1,sigma=sig,order=1,mode="wrap")
        curv=np.divide(np.abs(x1*y2-y1*x2),np.power(x1**2+y1**2,3./2))
        return curv
    
    def wrap2pi(self,a):
        return (a + np.pi) % (2 * np.pi) - np.pi

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = (np.pi/180)*self._current_yaw
        waypoints       = self._waypoints
        last_x=self._last_x
        last_y=self._last_y
        last_yaw=self._last_yaw
        vehicle=self._vehicle
        
        # --Changing LHS to RHS
        yaw=-yaw
        y=-y
        # -------------------
        vX              = self._current_vX
        vY=-self._current_vY
        
        dt=self._dt   #fixed time step dt for fps 3-
        # dt=time.time()-self._t  #  Variable time step dt
        # print(dt)

        d_yaw=(yaw-self._last_yaw)/dt
        Ca=self._Ca
        Iz=self._Iz
        lr=self._lr
        lf=self._lf
        m=self._m
        # vx,vy=self.d_velocities(dt,x,y,last_x,last_y,yaw)   #callin function to compute the x and y velocites
        vy=vY*np.cos(yaw)-vX*np.sin(yaw)
        vx=vY*np.sin(yaw)+vX*np.cos(yaw) 

        vx=max(vx,0.1)
        # print('vehicle speed={}, vx={},vy={}, yaw={}'.format(v,vx,vy,yaw*180/np.pi))
        curv=self.traj_curv #self.curvature(waypoints) #computing the curvatue of the reference trajectory at each index
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0
        min_idx=0


        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            
   
            if self._frame>1:

                A=[[0,1,0,0],[0,-4*Ca/(m*vx),4*Ca/m,2*Ca*(lr-lf)/(m*vx)],[0,0,0,1],[0,2*Ca*(lr-lf)/(Iz*vx),2*Ca*(lf-lr)/Iz,-2*Ca*(lr*lr+lf*lf)/(Iz*vx)]]
                
                B=[[0],[2*Ca/m],[0],[2*Ca*lf/Iz]]

                C=np.identity(4)

                D=[[0],[0],[0],[0]]

                Q=[[1,0,0,0],[0,1,0,0],[0,0,1000,0],[0,0,0,1]]
                
                R=10

                # #  State Space System (Continuous)
                sys_cont=scipy.signal.StateSpace(A,B,C,D)

                # #  Discretizing the state space system
                sys_disc=sys_cont.to_discrete(dt)


                A_d=sys_disc.A
                B_d=sys_disc.B


                # Computing the LQR Gain
                K=-self.dlqr(A_d,B_d,Q,R)
                
                #  computing the closest reference waypoint index and distance to it
                min_dis,min_idx=self.find_nearest_points(x,y,waypoints)
                #  Computiong the look ahead index 
                if min_idx<(len(waypoints)-self._look_ahead):
                    idx_fwd=self._look_ahead
                else:
                    idx_fwd=len(waypoints)-min_idx-1

                if min_idx<(len(waypoints)-self._curv_ld):
                    idx_ld_curv=self._curv_ld
                else:
                    idx_ld_curv=len(waypoints)-min_idx-1

                #  Computing the desired yaw 
                yaw_desired=np.arctan2((waypoints[min_idx+idx_fwd,1]-y),(waypoints[min_idx+idx_fwd,0]-x))
                d_yaw_desired=vx*curv[min_idx+idx_ld_curv]
                e=np.zeros(4)
                
                # Computing the state errors
                e[0]=(y-waypoints[min_idx+idx_fwd,1])*np.cos(yaw_desired)-(x-waypoints[min_idx+idx_fwd,0])*np.sin(yaw_desired)
                e[2]=self.wrap2pi(yaw-yaw_desired)
                e[1]=vy+vx*e[2]
                e[3]=d_yaw-d_yaw_desired
                # print(e)

                error=np.matrix(e)

                #  Computing the desired steering output
                steer_output=float(-K*np.transpose(error))*self._conv_rad_to_steer
                # print(steer_output)
                # print('steer:',steer_output)



                V_n = 2
                # -----Bang Bang Longitudanal Control------------
                # if np.linalg.norm(np.array([vx,vy]))<V_n:
                #     throttle_output=1.0
                #     brake_output=0.0
                # else:
                #     throttle_output=0.0
                #     brake_output=0.0

                # ------Longitudanal PID control-----------
                throttle_output,brake_output=self.PID_longitudanal(dt,self.v_desired[min_idx]-vx)
   
            if min_idx >= (len(waypoints) -3):
                throttle_output = 0
                steer = 0
                brake_output = 1.0            
            self._controller.throttle=throttle_output
            self._controller.steer=max(-1.1,(min(1.1,steer_output)))
            self._controller.brake=brake_output
            vehicle.apply_control(self._controller)
            # if min_idx==(len(waypoints)-1):
            #     return True
    
        self._last_x=x
        self._last_y=y
        self._last_yaw=yaw
        self._t=time.time()
        return False