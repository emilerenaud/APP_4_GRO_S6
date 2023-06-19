#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May 14 23:19:16 2020

@author: alex
------------------------------------


Fichier d'amorce pour les livrables de la problématique GRO640'


"""

import numpy as np

from pyro.control  import robotcontrollers
from pyro.control.robotcontrollers import EndEffectorPD
from pyro.control.robotcontrollers import EndEffectorKinematicController


###################
# Part 1
###################

def dh2T( r , d , theta, alpha ):
    """

    Parameters
    ----------
    r     : float 1x1
    d     : float 1x1
    theta : float 1x1
    alpha : float 1x1
    
    4 paramètres de DH

    Returns
    -------
    T     : float 4x4 (numpy array)
            Matrice de transformation

    """
    
    T = np.zeros((4,4))
    
    ###################
    # Votre code ici
    ###################
    T = np.matrix([[np.cos(theta) , -np.sin(theta)*np.cos(alpha) , np.sin(theta)*np.sin(alpha), r*np.cos(theta)],
                  [np.sin(theta) , np.cos(theta)*np.cos(alpha) , -np.cos(theta)*np.sin(alpha) , r*np.sin(theta)],
                  [0 , np.sin(alpha) , np.cos(alpha) , d],
                  [0 , 0 , 0 , 1]])
    
    return T



def dhs2T( r , d , theta, alpha ):
    """

    Parameters
    ----------
    r     : float nx1
    d     : float nx1
    theta : float nx1
    alpha : float nx1
    
    Colonnes de paramètre de DH

    Returns
    -------
    WTT     : float 4x4 (numpy array)
              Matrice de transformation totale de l'outil

    """
    
    WTT = np.zeros((4,4))
    
    ###################
    # Votre code ici
    ###################
   
    T = np.zeros((4,4))
    for i in range(len(r)):
        T = dh2T(r[i],d[i],theta[i],alpha[i])
        if(i==0): # first time copy T in WTT
            WTT = T
        else:
            WTT = WTT @ T
    
    return WTT


def f(q):
    """
    

    Parameters
    ----------
    q : float 6x1
        Joint space coordinates

    Returns
    -------
    r : float 3x1 
        Effector (x,y,z) position

    """
    r = np.zeros((3,1))
    
    ###################
    # Votre code ici
    ###################

    # Robot parameters DH table
    d = [0.147,0,0,0,0.2175]
    theta = [q[0],q[1]-(np.pi/2),q[2],q[3]+(np.pi/2),q[4]]
    r = [0.033,0.155,0.135,0,0]
    alpha = [-np.pi/2,0,0,np.pi/2,0]

    # Transformation matrices
    WTT = dhs2T(r,d,theta,alpha)
    # print(WTT)
    r = WTT[0:3,3]
    return r

###################
# Part 2
###################
    
class CustomPositionController( EndEffectorKinematicController ) :
    
    ############################
    def __init__(self, manipulator ):
        """ """
        
        EndEffectorKinematicController.__init__( self, manipulator, 1)
        
        ###################################################
        # Vos paramètres de loi de commande ici !!
        ###################################################
        
    
    #############################
    def c( self , y , r , t = 0 ):
        """ 
        Feedback law: u = c(y,r,t)
        
        INPUTS
        y = q   : sensor signal vector  = joint angular positions      dof x 1
        r = r_d : reference signal vector  = desired effector position   e x 1
        t       : time                                                   1 x 1
        
        OUPUTS
        u = dq  : control inputs vector =  joint velocities             dof x 1
        
        """
        
        # Feedback from sensors
        q = y
        
        # Jacobian computation
        J = self.J( q )
        
        # Ref
        r_desired   = r
        r_actual    = self.fwd_kin( q )
        
        # Error
        e  = r_desired - r_actual

        # Effector space speed
        dr_r = e * self.gains
        
        ################
        dq = np.zeros( self.m )  # place-holder de bonne dimension
        
        ##################################
        # Votre loi de commande ici !!!
        ##################################

        # eq 8.23 Solution des moindres carrés
        gain = 0.2
        dq = np.linalg.inv( J.T @ J + gain**2 * np.identity(self.m)) @ np.dot(J.T, e)
        
        return dq
    
    
###################
# Part 3
###################
        

        
class CustomDrillingController( robotcontrollers.RobotController ) :
    """ 

    """
    
    ############################
    def __init__(self, robot_model ):
        """ """
        
        super().__init__( dof = 3 )
        
        self.robot_model = robot_model
        
        # Label
        self.name = 'Custom Drilling Controller'
        self.position = False
        self.drill = False        
        
    #############################
    def c( self , y , r , t = 0 ):
        """ 
        Feedback static computation u = c(y,r,t)
        
        INPUTS
        y  : sensor signal vector     p x 1
        r  : reference signal vector  k x 1
        t  : time                     1 x 1
        
        OUPUTS
        u  : control inputs vector    m x 1
        
        """
        
        # Ref
        f_e = r
        
        # Feedback from sensors
        x = y
        [ q , dq ] = self.x2q( x )
        
        # Robot model
        r = self.robot_model.forward_kinematic_effector( q ) # End-effector actual position
        J = self.robot_model.J( q )      # Jacobian matrix
        g = self.robot_model.g( q )      # Gravity vector
        # H = self.robot_model.H( q )      # Inertia matrix
        # C = self.robot_model.C( q , dq ) # Coriolis matrix
        # d = self.robot_model.d( q , dq ) # Friction vector
        
            
        ##################################
        # Votre loi de commande ici !!!
        ##################################
        
        u = np.zeros(self.m)  # place-holder de bonne dimension
        # print(r)
        # print (u.shape)
        # First, the end-effector position is controlled to the desired position
        Kp = np.diag([1000,1000,1000])
        Kd = np.diag([50,50,50])
        r_d = [0.25,0.25,0.4]
        f_e = [0,0,-200]

        if (self.position == False):
            if (np.sqrt((r_d[0]-r[0])**2 + (r_d[1]-r[1])**2) + (r_d[2]-r[2])**2 > 0.01):
                u = J.T @ ( Kp @ ( r_d - r ) + Kd @ ( - J @ dq ) ) + g    # End-effector impedance law
            else:
                self.position = True
        else:
            if (self.drill == False):
                # drill down with force control
                if (r[2] > 0):
                    u = J.T @ f_e + g   
                else:
                    self.drill = True
                    u = g  # stop the robot in place
        
        return u
        
    
###################
# Part 4
###################
        
    
def goal2r( r_0 , r_f , t_f ):

    # Time discretization

    l = 1000 # nb of time steps

    # Number of DoF for the effector only

    m = 3

    r = np.zeros((m,l))

    dr = np.zeros((m,l))

    ddr = np.zeros((m,l))

   

    for i in range(l):

        r[:,i] = ((3/(t_f**2))*((t_f*(i/l))**2) - (2/(t_f**3))*((t_f*(i/l))**3)) * (r_f - r_0) + r_0

        dr[:,i] = ((6/(t_f**2))*(t_f*(i/l)) - (6/(t_f**3))*((t_f*(i/l))**2)) * (r_f - r_0)

        ddr[:,i] = ((6/(t_f**2)) - (12/(t_f**3))*(t_f*(i/l))) * (r_f - r_0)

    return r, dr, ddr




def dJq(q, dq, l1, l2, l3):

   

    c1  = np.cos( q[0] )

    s1  = np.sin( q[0] )

    c2  = np.cos( q[1] )

    s2  = np.sin( q[1] )

    c3  = np.cos( q[2] )

    s3  = np.sin( q[2] )

    c23 = np.cos( q[2] + q[1] )

    s23 = np.sin( q[2] + q[1] )

    # dJ = [[-c1*(l3*c23 + l2*c2)*dq[0], -c1*(l3*c23 + l2*c2)*dq[1], -l3*c23*c1*dq[2]],

    #       [-s1*(l3*c23 + l2*c2)*dq[0], -s1*(l3*c23 + l2*c2)*dq[1], -l3*c23*s1*dq[2]],

    #       [0, (-l3*s23 - l2*s2)*dq[1], -l3*s23*dq[2]]]

    dJ = [[(-c1*(l3*c23 + l2*c2)*dq[0]) + ((-s1*(-l3*s23 - l2*s2))*dq[1]) - ((s1*(-l3*s23)*dq[2])) , ((s1*(l3*s23 + l2*s2))*dq[0]) - ((-c1*(l3*c23 + l2*c2))*dq[1]) - ((c1*(l3*c23)*dq[2])), ((l3*s23 * s1)*dq[0]) + ((l3*c23*c1)*dq[1]) + (-l3*c23*c1*dq[2])],

          [(-s1*(l3*c23 + l2*c2)*dq[0]) + ((c1*(-l3*s23 - l2*s2))*dq[1]) + ((c1*(-l3*s23))*dq[2]), ((c1*(l3*s23 + l2*s2))*dq[0]) - ((s1*(l3*c23 + l2*c2))*dq[1]) - ((s1*(l3*c23))*dq[2]), ((-l3*s23*c1)*dq[0]) + ((-l3*c23*s1)*dq[1]) - (l3*c23*s1*dq[2])],

          [0, ((-l3*s23 - l2*s2)*dq[1]) + ((-l3*s23)*dq[2]), ((-l3*s23)*dq[1])+(-l3*s23*dq[2])]]




    return dJ




def r2q( r, dr, ddr , manipulator ):

    """
    Parameters

    ----------

    r   : numpy array float 3 x l

    dr  : numpy array float 3 x l

    ddr : numpy array float 3 x l

    manipulator : pyro object

    Returns

    -------

    q   : numpy array float 3 x l

    dq  : numpy array float 3 x l

    ddq : numpy array float 3 x l

    """

    # Time discretization

    l = r.shape[1]

   

    # Number of DoF

    n = 3

    # Joint length

    l1 = manipulator.l1

    l2 = manipulator.l2

    l3 = manipulator.l3


    # Output dimensions

    q = np.zeros((n,l))

    dq = np.zeros((n,l))

    ddq = np.zeros((n,l))

    dJ = np.zeros((3,3))

           

    xr = np.sqrt(r[0,:]**2 + r[1,:]**2)




    for i in range(l):

        q[0,i] = np.arctan(r[1,i]/r[0,i])

        q[2,i] = -np.arccos(((xr[i]**2 + (r[2,i]-l1)**2) - (l2**2 + l3**2))/(2*l2*l3))

        q[1,i] = np.arctan(r[2,i]/xr[i]) + np.arctan((l2*np.sin(q[2,i]))/(l1 + (l2*np.cos(q[2,i]))))




    for i in range(l):        

        dq[:,i] = np.linalg.inv( manipulator.J(q[:,i])) @ dr[:,i]




    for i in range(l):

        ddq[:,i] = np.linalg.inv( manipulator.J(q[:,i])) @ (ddr[:,i] - dJq(q[:,i], dq[:,i], l1, l2, l3) @ dq[:,i])



    return q, dq, ddq


def q2torque( q, dq, ddq , manipulator ):
    """

    Parameters
    ----------
    q   : numpy array float 3 x l
    dq  : numpy array float 3 x l
    ddq : numpy array float 3 x l
    
    manipulator : pyro object 

    Returns
    -------
    tau   : numpy array float 3 x l

    """
    # Time discretization
    l = q.shape[1]
    
    # Number of DoF
    n = 3
    
    # Output dimensions
    tau = np.zeros((n,l))
    
    #################################
    # Votre code ici !!!
    ##################################
    
    
    return tau


# main 
if __name__ == "__main__":
    # test function dh2T
    # T = dh2T( 1 , 1 , 1 , 1 )
    # print( T )

    # test function dhs2T
    # T = dhs2T( [1,1,1,1,1] , [1,1,1,1,1] , [1,1,1,1,1] , [1,1,1,1,1] )
    # print( T )

    # test function f
    r = f( [0,0,0,0,0] )
    # print( r )
    print ("X : " + str(r[0]) + "\nY : " + str(r[1]) + "\nZ : " + str(r[2]) )