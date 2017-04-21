#!/usr/bin/env python
import adapy, openravepy, numpy, prpy, rospy
from IPython import embed
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from openravepy import *
#import offscreen_render
import cv2
from Node import Node
from prpy.planning import CBiRRTPlanner

currTheta = 0
currR = 0
currHeight = 0

#traj1 = openravepy.planningutils.RetimeActiveDOFTrajectory(traj, robot, False, 0.2, 0.2, "LinearTrajectoryRetimer", "")
#traj = a.PlanToEndEffectorOffset(robot, [0.25, 0.25, 0], 0.1)
# a = CBiRRTPlanner()


def PlanToTransform(env, robot, transform):
    
    handle = openravepy.misc.DrawAxes(env, transform);
    iksolver = robot.arm.GetIkSolver()
    param = openravepy.IkParameterization(transform, openravepy.IkParameterizationType.Transform6D)
    solution = iksolver.Solve(param, robot.GetActiveDOFValues(),  0)
    print solution.GetSolution()
    print robot.GetActiveDOFIndices()
    import IPython
    IPython.embed()
    if solution.GetSolution() != []:
       traj =  robot.PlanToConfiguration(solution.GetSolution(), execute=True)
       return traj;
    else:
       return False;
    # a = CBiRRTPlanner()
    # traj = a.PlanToEndEffectorPose(robot, transform, jointstarts=[robot.GetActiveDOFValues()])
    # traj = a.PlanToEndEffectorPose(robot, transform)
    # traj = a.PlanToEndEffectorOffset(robot, [0.25, 0.25, 0], 0.1)
    # traj1 = openravepy.planningutils.RetimeActiveDOFTrajectory(traj, robot, False, 0.5, 0.5, "LinearTrajectoryRetimer", "")
    # robot.ExecuteTrajectory(traj, execute=True)
    embed()


def PlanToOffset(env, robot, offset):
    transform = robot.arm.GetEndEffectorTransform()
    transform[0:3, 3] += offset;
    traj = PlanToTransform(env, robot, transform);
    return traj

def sweepBack(theta):
    r_increment = 0.05
    max_r = 0.5
    min_r = 0.15
    curr_r = 0.5

    while curr_r > min_r:
        translate = numpy.array([[1, 0, 0, curr_r * numpy.cos(theta)],
                        [0, 1, 0, curr_r * numpy.sin(theta)],
                        [0, 0, 1, 0.25],
                        [0, 0, 0, 1]])

        z_angle = numpy.pi/2 + theta
        z_rot = numpy.array([[numpy.cos(z_angle), -numpy.sin(z_angle), 0, 0.0],
                    [numpy.sin(z_angle), numpy.cos(z_angle), 0, 0.0],
                    [0, 0, 1, 0.0],
                    [0, 0, 0, 1]])
        end_pose_final = numpy.dot(translate, z_rot)

        PlanToTransform(env, robot, end_pose_final)

        curr_r = curr_r - r_increment

def sweepForward(theta):
    r_increment = 0.05
    max_r = 0.5
    min_r = 0.15
    curr_r = 0.15

    while min_r < curr_r:
        translate = numpy.array([[1, 0, 0, curr_r * numpy.cos(theta)],
                        [0, 1, 0, curr_r * numpy.sin(theta)],
                        [0, 0, 1, 0.25],
                        [0, 0, 0, 1]])

        z_angle = numpy.pi/2 + theta
        z_rot = numpy.array([[numpy.cos(z_angle), -numpy.sin(z_angle), 0, 0.0],
                    [numpy.sin(z_angle), numpy.cos(z_angle), 0, 0.0],
                    [0, 0, 1, 0.0],
                    [0, 0, 0, 1]])
        end_pose_final = numpy.dot(translate, z_rot)

        PlanToTransform(env, robot, end_pose_final)

        curr_r = curr_r - r_increment

def goToHome():

    global currTheta
    global currR
    global currHeight

    currTheta = 0 #numpy.around(numpy.pi/4, 4)
    currR = 0.35
    currHeight = 0.1

    embed()

    robot.arm.PlanToNamedConfiguration("home", execute=True)
    # moveTo(currTheta, currR, currHeight)
    ### RANDOM FUCKING TEST SHIT
    # transform = robot.arm.GetEndEffectorTransform()
    # transform[0,3] = transform[0,3] + 0.05
    # transform[1,3] = transform[1,3] + 0.05
    # handle = openravepy.misc.DrawAxes(env, transform);
    # iksolver = robot.arm.GetIkSolver()
    # param = openravepy.IkParameterization(transform, openravepy.IkParameterizationType.Transform6D)
    # solution = iksolver.Solve(param, robot.GetActiveDOFValues(),  0)
    # print solution.GetSolution()
    # print robot.GetActiveDOFIndices()
    # import IPython
    # IPython.embed()
    # if solution.GetSolution() != []:
    #    traj =  robot.PlanToConfiguration(solution.GetSolution(), execute=True)
    #    return traj;
    # else:
    #    return False;


def moveTo(theta, r, height):
    translate = numpy.array([[1, 0, 0, r * numpy.cos(theta)],
                    [0, 1, 0, r * numpy.sin(theta)],
                    [0, 0, 1, height],
                    [0, 0, 0, 1]])

    z_angle = numpy.pi/2 + theta
    z_rot = numpy.array([[numpy.cos(z_angle), -numpy.sin(z_angle), 0, 0.0],
                [numpy.sin(z_angle), numpy.cos(z_angle), 0, 0.0],
                [0, 0, 1, 0.0],
                [0, 0, 0, 1]])
    end_pose_final = numpy.dot(translate, z_rot)

    return PlanToTransform(env, robot, end_pose_final)

def motionTo(desired_theta, desired_r, desired_height):

    global currTheta
    global currR
    global currHeight

    move_increment = 0.05
    angle_increment = 0.02
    
    #while numpy.abs(currTheta - desired_theta) >= (angle_increment/2):

    #    if currTheta > desired_theta:
    #        currTheta = numpy.around(currTheta - angle_increment, 4)
    #    else:
    #        currTheta = numpy.around(currTheta + angle_increment, 4)

    #    traj = moveTo(currTheta, currR, currHeight)

    if currTheta != desired_theta:

        currTheta = desired_theta
        traj = moveTo(currTheta, currR, currHeight)

    while numpy.abs(currHeight - desired_height) >= (move_increment/2):

        if currHeight > desired_height:
            testHeight = numpy.around(currHeight - move_increment, 4)
        else:
            testHeight = numpy.around(currHeight + move_increment, 4)

        traj = moveTo(currTheta, currR, testHeight)

        if traj == False:
            return
        else:
            currHeight = testHeight

    while numpy.abs(currR - desired_r) >= (move_increment/2):

        if currR > desired_r:
            testR = numpy.around(currR - move_increment, 4)
        else:
            testR = numpy.around(currR + move_increment, 4)

        traj = moveTo(currTheta, testR, currHeight)

        if traj == False:
            return
        else:
            currR = testR

rospy.init_node('test_scenario', anonymous = True)
openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Error)
openravepy.misc.InitOpenRAVELogging();

env, robot = adapy.initialize(env_path='/home/yyn/Code/catkin_ws/src/babysitter/src/models/babysitter.env.xml', attach_viewer='rviz', sim=False)
# env, robot = adapy.initialize(env_path='/home/yyn/Code/catkin_ws/src/babysitter/src/models/babysitter.env.xml', attach_viewer='rviz', sim=True)
manip = robot.arm



#manip.SetIkSolver(openravepy.RaveCreateIkSolver(env, 'NloptIK'))
#robot.SetActiveDOFs([2,3,4,5,6,7])
robot.arm.SetActive()
values = robot.GetActiveDOFValues()
values[1] = values[1] - 0.3
robot.PlanToConfiguration(values, execute=True)
ada_pose = numpy.array([[1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0.01],
                        [0, 0, 0, 1]])
#box_pose = numpy.array([[1, 0, 0, 0.25],
#                        [0, 1, 0, -0.25],
#                        [0, 0, 1, 0.6],
#                        [0, 0, 0, 1]])
#end_pose = numpy.array([[1, 0, 0, 0.25],
#                        [0, 1, 0, 0.25],
#                        [0, 0, 1, 0],
#                        [0, 0, 0, 1]])
#bowl = env.GetBodies()[0]
#bowl.SetTransform(box_pose)
robot.SetTransform(ada_pose)
goToHome()

# camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
#                            [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
#                            [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
#                            [ 0.        ,  0.        ,  0.        ,  1.        ]])
# robot.GetEnv().GetViewer().SetCamera(env.GetViewer().GetCameraTransform())
# image = env.GetViewer().GetCameraImage(640,480,  env.GetViewer().GetCameraTransform(),[640,640,320,240])
# cv2.imshow("Image window", image)
# cv2.waitKey(3)
# pub = rospy.Publisher('image', Image, queue_size=10)


#manip.SetIkSolver(openravepy.RaveCreateIkSolver(env, 'NloptIK'))
#robot.SetActiveDOFs([2,3,4,5,6,7])
#robot.arm.SetActive()
#values = robot.GetActiveDOFValues()
#values[1] = values[1] - 0.3
#robot.PlanToConfiguration(values, execute=True)

#env = openravepy.Environment()
#env.SetViewer('rviz')

#box = env.GetBodies()[0]
#PlanToTransform(env,robot,end_pose)

#arm_transform[1][3] = box_transform[1][3] + 0.1
#arm_transform[2][3] = box_transform[2][3] + 0.1

# # Create the sensor
# sensor = openravepy.RaveCreateSensor(env, 'offscreen_render_camera')
# # Set its intrinsics (fx, fy, cx, cy, near, far)
# sensor.SendCommand('setintrinsic 529 525 328 267 0.01 10')
# # And its resolution in pixels
# sensor.SendCommand('setdims 640 480')
# #Initialize the sensor. Right now the size of the sensor can't be changed after you do this.
# # It will also open up an annoying opengl window just to get context.
# sensor.Configure(openravepy.Sensor.ConfigureCommand.PowerOn)

#s = env.GetSensor("BaseCamera1")
#data = s.GetSensorData()
#image = data.imagedata
#cv2.imshow("Image window", image)
#cv2.waitKey(3)

#from IPython import embed
embed()
