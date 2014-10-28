# -*- coding: utf-8 -*-
from __future__ import division
import numpy as np
from PIL import Image as im
import os
import scipy.io as sio
import viz
import vizact
import vizshape
#import vizmatplottwo
import scipy
import vizmenu
import time


def CreateTheRoom():

    """<START OF DEFINING THE WORLD>"""

    #disables the original light
    viz.disable(viz.LIGHT0)

    topLight = viz.addLight() 
    topLight.setPosition(0,4,0)
    topLight.setEuler( 0, 90 ,0 )
    #topLight.direction(0,-1,0) 
    topLight.spread(270) 
    topLight.intensity(2000) 
    #topLight.spotexponent(2)     
    sideLight = viz.addLight()
    sideLight.setPosition(0,5,0)
    sideLight.setEuler(0,0,90)

    #creates the floor plane to be 35x80
    #rotated around the Z axis(perpendicular to Y axis)
    floor = vizshape.addPlane(size=(35.0,80.0), axis=vizshape.AXIS_Y, cullFace=False)
    #makes the floor look like wood
    floor.texture(viz.addTexture('images/tile_wood.jpg'))
    #moves the floor +20 meters in the z direction
    floor.setPosition(0,0,20)

    """
    Could use Gabe's mocap interaface here to create a room easily
    and play with textures. 

    If performance is an issue, set cullFace to True.
    """
    #adds the wall(plane) farthest from the hand/person
    #35x10 meters**2
    #rotated around the X axis (perpendicular to Z axis)
    frontWall = vizshape.addPlane(size=(35,10), axis=vizshape.AXIS_Z, cullFace = False)
    #moves the wall to match the edge of the 
    frontWall.setPosition(0,5,60)
    #makes the wall appear white
    frontWall.color(viz.WHITE)

    #adds the wall(plane) that when facing the frontWall, is to the camera's left
    #wall is 80x10 meters**2
    #wall is rotated about the Y axis(perpendicular to X axis)
    leftWall = vizshape.addPlane(size=(80,10), axis=vizshape.AXIS_X, cullFace = False)
    #shifts the wall to match the edge of the floor
    leftWall.setPosition(-17.5,5,20)
    #makes the wall appear white
    leftWall.color(viz.WHITE)

    #adds a wall(plane) that when facing the frontWall is to the camera's right
    #wall is 80x10 meters**2
    #wall is rotated about the Y axis(perpendicular to X axis)
    rightWall = vizshape.addPlane(size=(80,10), axis=vizshape.AXIS_X, cullFace = False)
    #shifts the wall to match the edge of the floor
    rightWall.setPosition(17.5,5,20)
    #makes the wall appear white
    rightWall.color(viz.WHITE)

    #adds a wall(plane) that when facing the frontWall is to the camera's back
    #wall is 35x10 meters**2
    #wall is rotated about the X axis(perpendicular to Z axis)
    backWall = vizshape.addPlane(size=(35,10), axis=vizshape.AXIS_Z, cullFace = False)
    #shifts the wall to match the edge of the floor
    backWall.setPosition(0,5,-20)
    #makes the wall appear white
    backWall.color(viz.WHITE)

    #adds a ceiling(plane) that when facing the frontWall is to the camera's topside
    #wall is 35x80 meters**2
    #wall is rotated about the Z axis(perpendicular to Y axis)
    ceiling = vizshape.addPlane(size=(35.0,80.0), axis=vizshape.AXIS_Y, cullFace=False)
    #makes the ceiling appear Skyblue in color
    ceiling.color(viz.SKYBLUE)
    #shifts the ceiling to rest on top of the four walls
    ceiling.setPosition(0,10,20)

    #add a meter marker at 0 meters along the z axis of the room
    #meter0 = vizshape.addPlane(size = (5,.3), axis = vizshape.AXIS_Y, cullFace = False)
    #meter0.setPosition(0,.3, 0)
    #makes the meter marker appear yellow
    #meter0.color(viz.WHITE)


    """</END OF DEFINING THE WORLD>"""

def FindIndex(Array,Value):
    
    Index =[]
    for index, number in enumerate(Array):
        if number == Value:
            Index.append(index)
        
    return Index

def ExtractDataFromMatFile(FileName):
    
    global Ball_Pos_XYZ, Ball_Vel_XYZ, Ball_Pix_XYZ, Paddle_Pos_XYZ, Paddle_Quat_WXYZ
    global View_Pos_XYZ, View_Quat_WXYZ, T, EventFlag
    global TrialEndIndex, TrialStartIndex;
    RawMatFile = sio.loadmat(FileName);

    Ball_X = map(float, RawMatFile['ballPos_XYZ'][:,0])
    Ball_Y = map(float, RawMatFile['ballPos_XYZ'][:,1])
    Ball_Z = map(float, RawMatFile['ballPos_XYZ'][:,2])

    Ball_Pos_XYZ = np.array([Ball_X, Ball_Y, Ball_Z], dtype = float)

    Ball_Vel_X = map(float, RawMatFile['ballVel_XYZ'][:,0])
    Ball_Vel_Y = map(float, RawMatFile['ballVel_XYZ'][:,1])
    Ball_Vel_Z = map(float, RawMatFile['ballVel_XYZ'][:,2])
    Ball_Vel_XYZ = np.array([Ball_Vel_X, Ball_Vel_Y, Ball_Vel_Z], dtype = float);

    Ball_Pix_X = map(float, RawMatFile['ballPix_XYDist'][:,0])
    Ball_Pix_Y = map(float, RawMatFile['ballPix_XYDist'][:,1])
    Ball_Pix_Z = map(float, RawMatFile['ballPix_XYDist'][:,2])
    Ball_Pix_XYZ = np.array([Ball_Pix_X, Ball_Pix_Y, Ball_Pix_Z], dtype = float);

    Paddle_X = map(float, RawMatFile['paddlePos_XYZ'][:,0])
    Paddle_Y = map(float, RawMatFile['paddlePos_XYZ'][:,1])
    Paddle_Z = map(float, RawMatFile['paddlePos_XYZ'][:,2])
    Paddle_Pos_XYZ = np.array([Paddle_X, Paddle_Y, Paddle_Z], dtype = float);

    #Fix Me (Kamran)
    Paddle_Quat_X = map(float, RawMatFile['paddleQUAT_WXYZ'][:,0])
    Paddle_Quat_Y = map(float, RawMatFile['paddleQUAT_WXYZ'][:,1])
    Paddle_Quat_Z = map(float, RawMatFile['paddleQUAT_WXYZ'][:,2])
    Paddle_Quat_W = map(float, RawMatFile['paddleQUAT_WXYZ'][:,3])
    #Fix Me
    Paddle_Quat_WXYZ = np.array([Paddle_Quat_X, Paddle_Quat_Y, Paddle_Quat_Z, Paddle_Quat_W ], dtype = float);

    View_X = map(float, RawMatFile['view_XYZ_Pos'][:,0])
    View_Y = map(float, RawMatFile['view_XYZ_Pos'][:,1])
    View_Z = map(float, RawMatFile['view_XYZ_Pos'][:,2])
    View_Pos_XYZ = np.array([View_X, View_Y, View_Z], dtype = float);

    #Fix Me (Kamran)
    View_Quat_X = map(float, RawMatFile['Quat_Matrix'][:,0])
    View_Quat_Y = map(float, RawMatFile['Quat_Matrix'][:,1])
    View_Quat_Z = map(float, RawMatFile['Quat_Matrix'][:,2])
    View_Quat_W = map(float, RawMatFile['Quat_Matrix'][:,3])
    #Fix Me
    View_Quat_WXYZ = np.array([View_Quat_X, View_Quat_Y, View_Quat_Z, View_Quat_W], dtype = float);

    T = map(float, RawMatFile['FrameTime'])
    EventFlag = map(float, RawMatFile['EventFlag'])

    print 'Total Number of Frames = ', len(T), '\n'
    print 'Total Number of Event Flags = ', len(EventFlag), '\n'
    print 'Ball Pos Size', Ball_Pos_XYZ
    print 'Ball Vel Size', Ball_Vel_XYZ.shape
    print 'Ball Pix XYZ Size', Ball_Pix_XYZ.shape
    print 'Paddle Pos Size', Paddle_Pos_XYZ.shape
    print 'Paddle Quat Size', View_Quat_WXYZ.shape
    print 'View Pos Size', View_Pos_XYZ.shape
    print 'View Quat Size', View_Quat_WXYZ.shape

    TrialStartIndex = FindIndex(EventFlag,1)
    TrialEndIndex = FindIndex(EventFlag,3)
    TrialNumber = len(TrialEndIndex)
    print 'Number of Trials are=\n', TrialNumber
    print 'Start Indexes =', TrialStartIndex
    print 'End Indexes =', TrialEndIndex

def CreateVisualObjects():

    global ball, Head, Hand, GazeLine, EyeBallLine;
    #creats a sphere(the ball) with radius of 5cm
    ball = vizshape.addSphere(radius = .05)
    #colors the ball red
    ball.color(viz.YELLOW)
    ball.visible(True)

    Origin = vizshape.addAxes()
    Origin.setPosition(0,0,0)
    #creats a sphere(the ball) with radius of 5cm
    #Head = vizshape.addCone( radius = 0.5, height = 0.8)
    Head = vizshape.addArrow(1, color = viz.YELLOW_ORANGE)
    #colors the ball red
    Head.color(viz.PURPLE)
    Head.visible(True)
    Head.setScale(.2,.2,.3)

    #creats a sphere(the hand) with radius of 10cm
    Hand = vizshape.addCylinder( height = 0.02, radius = 0.2)
    #colors the hand red
    Hand.color(viz.RED)
    Hand.visible(True)

    # Creating a Line to represent Gaze Vector
    viz.startLayer(viz.LINES)
    viz.vertex(0,0,0)
    viz.vertex(0,0,3)
    viz.vertexColor(viz.GREEN)
    GazeLine = viz.endLayer() # Object will contain both points and lines
    GazeLine.visible(True)
    GazeLine.setScale(5,5,5)


    # Creating a Line to represent Eye-Ball Vector
    viz.startLayer(viz.LINES)
    viz.vertex(0,0,0)
    viz.vertex(0,0,3)
    viz.vertexColor(viz.YELLOW)
    EyeBallLine = viz.endLayer() # Object will contain both points and lines
    EyeBallLine.visible(True)
    EyeBallLine.setScale(5,5,5)

def Quaternion2Matrix(Q):
    Q = Q/np.linalg.norm(Q); # Ensure Q has unit norm
    
    # Set up convenience variables
    x = Q[0]; y = Q[1]; z = Q[2]; w = Q[3];
    w2 = w*w; x2 = x*x; y2 = y*y; z2 = z*z;
    xy = x*y; xz = x*z; yz = y*z;
    wx = w*x; wy = w*y; wz = w*z;
    
    M = np.array([[w2+x2-y2-z2 , 2*(xy - wz) , 2*(wy + xz) ,  0],
         [ 2*(wz + xy) , w2-x2+y2-z2 , 2*(yz - wx) ,  0 ],
         [ 2*(xz - wy) , 2*(wx + yz) , w2-x2-y2+z2 ,  0 ],
         [     0      ,       0     ,       0     ,  1 ]], dtype = float);
    return M;

#def FindMinimumAngle(v1, v2, fr):
#    
#    global lEyeRotationMatrix, lEyeOffsetMatrix, View_Quat_WXYZ;
#    Error = 2000;
#    Angle = 0;
#    Vector = v1
#    for alfa in (np.linspace(-20*np.pi/180, 0*np.pi/180, num = 10)):
#        lEyeRotationMatrix[0,:] = [np.cos(alfa), np.sin(alfa), 0, 0];
#        lEyeRotationMatrix[1,:] = [-np.sin(alfa), np.cos(alfa), 0, 0];
#        ViewRotation = Quaternion2Matrix(View_Quat_WXYZ[:,fr]);
#        Result = lEyeOffsetMatrix.dot(v2);
#        Result = lEyeRotationMatrix.dot(Result);
#        v = ViewRotation.dot(Result);
#        ErrorTemp = (np.abs(np.linalg.norm(np.cross(v1[0:3],v[0:3]))));
#        
#        if ErrorTemp < Error :
#            Error = ErrorTemp
#            Angle = alfa
#            Vector = v
#    #print 'Error,', Error,' alfa',Angle*180/np.pi
#    return Angle,Vector

def SetRotationAngle(angle):
    global alpha;
    print 'Screen Rotation Set to ', angle;
    alpha = angle*(np.pi/180);

def onTimer(num):
#for counter in range(50,60):#TrialNumber):
    global counter, FrameNumber, GazeLine, LeftEyeShift, nearH, nearW;
    global lEyeOffsetMatrix, lEyeRotationMatrix;
    global TrialStartIndex, TrialEndIndex;

    StartIndex = TrialStartIndex[counter]
    EndIndex   = TrialEndIndex[counter]
#    for FrameNumber in range(StartIndex, EndIndex):
    #print 'F=', FrameNumber,'P=', Ball_Pos_XYZ[:,FrameNumber],'Q=',View_Quat_WXYZ[:,FrameNumber], '\n'
    ball.setPosition(*Ball_Pos_XYZ[:,FrameNumber]) # X[:,FrameNumber])#
    
    V1 = Ball_Pos_XYZ[:,FrameNumber] - View_Pos_XYZ[:,FrameNumber] ;
    V1 = np.hstack((V1,1))
    V1.reshape(4,1)
    # normalize vector to unit length


    BallSx = (Ball_Pix_XYZ[0,FrameNumber] - 0.5)*nearW;
    BallSy = (Ball_Pix_XYZ[1,FrameNumber] - 0.5)*nearH;
    BallWorldDir_XYZ = np.array([BallSx, BallSy, 1, 1], dtype = float)

    lEyeRotationMatrix[0,:] = [np.cos(alpha), 0,np.sin(alpha), 0];
    lEyeRotationMatrix[1,:] = [0, 1, 0, 0];
    lEyeRotationMatrix[2,:] = [-np.sin(alpha), 0,np.cos(alpha), 0];
    lEyeRotationMatrix[3,:] = [0, 0, 0, 1];
    
    ViewRotation = Quaternion2Matrix(View_Quat_WXYZ[:,FrameNumber]);
    Result = lEyeOffsetMatrix.dot(BallWorldDir_XYZ);
    Result = lEyeRotationMatrix.dot(Result);
    V2 = ViewRotation.dot(Result);        
    V2.reshape(4,1);
    
    Head.setPosition(*View_Pos_XYZ[:,FrameNumber])
    Head.setQuat(*View_Quat_WXYZ[:,FrameNumber])
    
    Hand.setPosition(*Paddle_Pos_XYZ[:,FrameNumber])
    # Fix Me Paddle Quaternion should be recorded in the raw data file
    # For Now it is rotated to stand parallel to the fron wall
    Hand.setQuat(*Paddle_Quat_WXYZ[:,FrameNumber])
    #Hand.setEuler(0,90,0)
    
    #GazeLine.setPosition(*View_Pos_XYZ[:,FrameNumber])
    GazeLine.setVertex(0, View_Pos_XYZ[0,FrameNumber], View_Pos_XYZ[1,FrameNumber], View_Pos_XYZ[2,FrameNumber], viz.ABS_GLOBAL)
    #GazeLine.setVertex(0, 0, 0, 0);    
    #GazeLine.setVertex(1, -View_Pos_XYZ[0,FrameNumber] + Ball_Pix_XYZ[0,FrameNumber], -View_Pos_XYZ[1,FrameNumber] - Ball_Pix_XYZ[1,FrameNumber],-View_Pos_XYZ[2,FrameNumber] - Ball_Pix_XYZ[2,FrameNumber], viz.ABS_GLOBAL)
    #Vertex = V2[0:3];# + View_Pos_XYZ[:, FrameNumber]
    GazeLine.setVertex(1, V2[0] + View_Pos_XYZ[0, FrameNumber], V2[1] + View_Pos_XYZ[1, FrameNumber], V2[2] + View_Pos_XYZ[2, FrameNumber], viz.ABS_GLOBAL)
    #GazeLine.setQuat(*View_Quat_WXYZ[:,FrameNumber])
    
    #EyeBallLine.setPosition(*View_Pos_XYZ[:,FrameNumber])
    EyeBallLine.setVertex(0, View_Pos_XYZ[0,FrameNumber], View_Pos_XYZ[1,FrameNumber], View_Pos_XYZ[2,FrameNumber], viz.ABS_GLOBAL)
    EyeBallLine.setVertex(1, Ball_Pos_XYZ[0,FrameNumber], Ball_Pos_XYZ[1,FrameNumber], Ball_Pos_XYZ[2,FrameNumber], viz.ABS_GLOBAL)
    
    
    FrameNumber = FrameNumber + 1 
        #time.sleep(1)


if __name__ == '__main__':
    
    global lEyeOffsetMatrix, lEyeRotationMatrix;
    global FrameNumber, counter, TrialStartIndex, TrialEndIndex;
    global nearW,nearH;
    
    nearH = 1.2497;
    nearW = 1.5622;
    viz.setMultiSample(4)
    viz.fov(60)
    viz.go()
    ExtractDataFromMatFile('RawMat_exp_data-2014-10-10-11-35.mat');
    CreateTheRoom()
    CreateVisualObjects()
    #sets where the camera view is located
    viz.MainView.setPosition([-0.95528644323349, 1.6438825130462646, -0.6167752742767334])
    #sets the angle at which the camera view is pointed
    viz.MainView.setEuler(10.763678550720215, -0.0, 0.0)
    lEyeOffsetMatrix = np.array([[1, 0, 0, -0.03],[0, 1, 0, 0], [0, 0, 1, 0],[0, 0, 0, 1]], dtype = float)
    lEyeRotationMatrix = np.eye(4, dtype = float);
    counter = 1
    FrameNumber = TrialStartIndex[0] - 100;
    SetRotationAngle(1.6);
    #if the timer goes off go to the onTimer function
    viz.callback(viz.TIMER_EVENT,onTimer)
    viz.starttimer(1, .077, viz.FOREVER)