#!/usr/bin/env python
import adapy, openravepy, numpy, prpy, rospy
from IPython import embed
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from openravepy import *
#import offscreen_render
import cv2

def PlanToTransform(env, robot, transform):
    handle = openravepy.misc.DrawAxes(env, transform);
    iksolver = robot.arm.GetIkSolver()
    param = openravepy.IkParameterization(transform, openravepy.IkParameterizationType.Transform6D)
    solution = iksolver.Solve(param, robot.GetActiveDOFValues(),  0)
    traj =  robot.PlanToConfiguration(solution.GetSolution(),execute=True)
    return traj;

def PlanToOffset(env, robot, offset):
    transform = robot.arm.GetEndEffectorTransform()
    transform[0:3, 3] += offset;
    traj = PlanToTransform(env, robot, transform);
    return traj

rospy.init_node('test_scenario', anonymous = True)
openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Error)
openravepy.misc.InitOpenRAVELogging();

env, robot = adapy.initialize(env_path='/home/mrsd/Code/ra_ws/src/babysitter/src/models/babysitter.env.xml', attach_viewer='rviz', sim=True)
manip = robot.arm



#manip.SetIkSolver(openravepy.RaveCreateIkSolver(env, 'NloptIK'))
#robot.SetActiveDOFs([2,3,4,5,6,7])
robot.arm.SetActive()
values = robot.GetActiveDOFValues()
values[1] = values[1] - 0.3
robot.PlanToConfiguration(values, execute=True)
ada_pose = numpy.array([[1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0.8],
                        [0, 0, 0, 1]])
box_pose = numpy.array([[1, 0, 0, 0.2],
                        [0, 1, 0, 0.2],
                        [0, 0, 1, 1.4],
                        [0, 0, 0, 1]])
end_pose = numpy.array([[1, 0, 0, 0.2],
                        [0, 1, 0, 0.2],
                        [0, 0, 1, 0.6],
                        [0, 0, 0, 1]])
env.GetBodies()[0].SetTransform(box_pose)
robot.SetTransform(ada_pose)

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

box = env.GetBodies()[1]
PlanToTransform(env,robot,end_pose)

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

from IPython import embed
embed()
