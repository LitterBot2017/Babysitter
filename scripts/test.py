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
from prpy.planning import SnapPlanner
import sys
from xml.dom import minidom
import rospy
from std_msgs.msg import Float32
from april_tracker.msg import pose
from prpy.planning.adapters import PlanToEndEffectorOffsetTSRAdapter
from time import time

planner = CBiRRTPlanner()
snapPlanner = SnapPlanner()

currTheta = 0
currR = 0
currHeight = 0


inToOutTrajectories = []
outTrajectories = []
inTrajectories = []

def PlanToTransform(env, robot, transform):
    
    with env:
        handle = openravepy.misc.DrawAxes(env, transform);
    iksolver = robot.arm.GetIkSolver()
    param = openravepy.IkParameterization(transform, openravepy.IkParameterizationType.Transform6D)
    solution = iksolver.Solve(param, robot.GetActiveDOFValues(),  0)
    print solution.GetSolution()
    print robot.GetActiveDOFIndices()
    #import IPython
    #IPython.embed()
    if solution.GetSolution() != []:
       traj =  robot.PlanToConfiguration(solution.GetSolution(), execute=True)
       return traj;
    else:
       return False;

def SweepIn(distance=0.3, execute=True):
    
    global currTheta
    global currR
    global currHeight

    # Sweep in
    traj = planner.PlanToEndEffectorOffset(robot, [-numpy.sin(currTheta), numpy.cos(currTheta), 0], distance)
    openravepy.planningutils.RetimeActiveDOFTrajectory(traj, robot, False, 0.2, 0.2, "LinearTrajectoryRetimer", "")
    currR = currR - distance
    if execute:
        robot.ExecuteTrajectory(traj)
    return traj

def SweepOut(distance=0.3, execute=True):
    
    global currTheta
    global currR
    global currHeight

    # Sweep out
    traj = planner.PlanToEndEffectorOffset(robot, [numpy.sin(currTheta), -numpy.cos(currTheta), 0], distance)
    openravepy.planningutils.RetimeActiveDOFTrajectory(traj, robot, False, 0.2, 0.2, "LinearTrajectoryRetimer", "")
    currR = currR + distance
    if execute:
        robot.ExecuteTrajectory(traj)
    return traj

def PlanToOffset(env, robot, offset):
    transform = robot.arm.GetEndEffectorTransform()
    transform[0:3, 3] += offset;
    with env:
        traj = PlanToTransform(env, robot, transform);
    return traj

def loadTrajectories():

    global inToOutTrajectories
    global outTrajectories
    global inTrajectories

    trajInToOutFile = minidom.parse('TrajInToOuts.xml')
    trajInToOuts = trajInToOutFile.getElementsByTagName('trajectory')
    angle = 0
    for trajInToOut in trajInToOuts:
        print('Loading in to out trajectory for %d degrees' % angle)
        with env:
            currTraj = RaveCreateTrajectory(env, '')
        currTraj.deserialize(trajInToOut.toxml())
        inToOutTrajectories.append(currTraj)
        angle = angle + 1

    trajOutFile = minidom.parse('TrajOuts.xml')
    trajOuts = trajOutFile.getElementsByTagName('trajectory')
    angle = 0
    for trajOut in trajOuts:
        print('Loading out trajectory for %d degrees' % angle)
        with env:
            currTraj = RaveCreateTrajectory(env, '')
        currTraj.deserialize(trajOut.toxml())
        outTrajectories.append(currTraj)
        angle = angle + 1

    trajInFile = minidom.parse('TrajIns.xml')
    trajIns = trajInFile.getElementsByTagName('trajectory')
    angle = 0
    for trajIn in trajIns:
        print('Loading in trajectory for %d degrees' % angle)
        with env:
            currTraj = RaveCreateTrajectory(env, '')
        currTraj.deserialize(trajIn.toxml())
        inTrajectories.append(currTraj)
        angle = angle + 1

def goToHome():
    goToInHome()

def goToConfig(config, theta, r, height):

    global currTheta
    global currR
    global currHeight

    print("\n\n\nGoing to config\n\n\n")
    print(theta)
    print(r)
    print(height)
    currTheta = theta
    currR = r
    currHeight = height + numpy.random.rand()*0.2 + 0.05

    update()
    while True:
        try:
            traj = planner.PlanToConfiguration(robot, config)
            currHeight = height
            #chains = PlanToEndEffectorOffsetTSRAdapter.CreateTSRChains(robot, [0, 0, -1], 0.01)
            #currHeight = height
            #traj = planner.PlanToConfiguration(robot, config, tsr_chains=chains, psample=0.1, smoothingitrs=0)
            #if traj.GetNumWaypoints() <= 2:
            #    print("\n\n\n!!!!!!\nSnapPlanner essentially failed.\n!!!!!!!\n\n\n")
            #    midpoint = traj.GetWaypoint(0) + traj.GetWaypoint(1)
            #    midpoint[0:6] = (midpoint[0:6]/2)
            #    import IPython
            #    IPython.embed()
            #    continue
            speed = numpy.random.rand()*0.5 + 0.25
            openravepy.planningutils.RetimeActiveDOFTrajectory(traj, robot, False, speed, speed, "LinearTrajectoryRetimer", "")
            robot.ExecuteTrajectory(traj)
            break
        except:
            print("\n\n\n!!!!!!\nSnapPlanner failed in goToConfig.\n!!!!!!!!\n\n\n")
            import IPython
            IPython.embed()

def goToInHome():
    goToConfig(getInHomeConfig(), 0, 0.2, 0.05)

def goToOutHome():
    goToInHome()
    robot.ExecuteTrajectory(inToOutTrajectories[0])
    #goToConfig(getOutHomeConfig(), 0, 0.5, 0.05)

def getInHomeConfig():

    global inToOutTrajectories
    global outTrajectories
    global inTrajectories

    if inToOutTrajectories == []:
        loadTrajectories()

    return inToOutTrajectories[0].GetWaypoint(0)[0:6]

def getOutHomeConfig():

    global inToOutTrajectories
    global outTrajectories
    global inTrajectories

    if inToOutTrajectories == []:
        loadTrajectories()

    return inToOutTrajectories[0].GetWaypoint(inToOutTrajectories[0].GetNumWaypoints()-1)[0:6]

def update():

    global currTheta
    global currR
    global currHeight

    moveTo(currTheta, currR, currHeight)

def moveTo(theta, r, height):
    theta = theta - numpy.pi/2
    translation = numpy.array([[0, -1, 0, r * numpy.cos(theta)],
                    [-1, 0, 0, r * numpy.sin(theta)],
                    [0, 0, -1, height],
                    [0, 0, 0, 1]])

    z_angle = -theta
    z_rot = numpy.array([[numpy.cos(z_angle), -numpy.sin(z_angle), 0, 0.0],
                [numpy.sin(z_angle), numpy.cos(z_angle), 0, 0.0],
                [0, 0, 1, 0.0],
                [0, 0, 0, 1]])
    end_pose_final = numpy.dot(translation, z_rot)

    return PlanToTransform(env, robot, end_pose_final)

def motionTo(desired_theta, desired_r, desired_height):

    global currTheta
    global currR
    global currHeight

    move_increment = 0.05
    angle_increment = 0.02
    
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

# env, robot = adapy.initialize(env_path='/home/yyn/Code/catkin_ws/src/babysitter/src/models/babysitter.env.xml', attach_viewer='rviz', sim=True)
env, robot = adapy.initialize(env_path='/home/yyn/Code/catkin_ws/src/babysitter/src/models/babysitter.env.xml', attach_viewer='rviz', sim=False)
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

robot.arm.PlanToNamedConfiguration("home", execute=True)
goToHome()

def computeTrajectories():

    global currTheta
    global currR
    global currHeight

    goToHome()
    degree = numpy.pi/180
    angle = 0
    currTheta = 0
    trajInToOuts = '<trajectories>\n'
    trajIns = '<trajectories>\n'
    trajOuts = '<trajectories>\n'

    moveTo(currTheta, currR, currHeight)
    while angle <= 90:
        angleRad = angle * (numpy.pi/180)
        while True:
            try:
                print("Computing/executing in to out traj for %d degrees" % angle)
                origR = currR
                trajInToOut = SweepOut()
                if angle == 0:
                    zeroConfigIn = trajInToOut.GetWaypoint(0)[0:6]
                    zeroConfigOut = trajInToOut.GetWaypoint(trajInToOut.GetNumWaypoints() -1)[0:6]
                    trajIn = trajInToOut
                    trajOut = trajInToOut
                    trajInToOutRev = openravepy.planningutils.ReverseTrajectory(trajInToOut)
                    robot.ExecuteTrajectory(trajInToOutRev)
                    currR = origR
                    print("Return 0 degrees to home")
                else:
                    while True:
                        try:
                            print("Computing arm from %d to 0 degrees on out" % angle)
                            #distance = numpy.sqrt(2 * 0.5 * 0.5 * (1 - numpy.cos(angleRad)))
                            #chains = PlanToEndEffectorOffsetTSRAdapter.CreateTSRChains(robot, [numpy.cos(angleRad), numpy.sin(angleRad), 0], distance)
                            #trajOutReverse = planner.PlanToConfiguration(robot, zeroConfigOut, tsr_chains=chains, psample=0.1, smoothingitrs=0)
                            trajOutReverse = snapPlanner.PlanToConfiguration(robot, zeroConfigOut)
                            print("Reversing %d to 0 degrees on out" % angle)
                            trajOut = openravepy.planningutils.ReverseTrajectory(trajOutReverse)
                            openravepy.planningutils.RetimeActiveDOFTrajectory(traj, robot, False, 0.2, 0.2, "LinearTrajectoryRetimer", "")
                            break
                        except Exception, e:
                            print(str(e))
                            print("Computing arm from %d to 0 degrees on out failed, trying again..." % angle)

                    print("Sweeping back in")
                    trajInToOutRev = openravepy.planningutils.ReverseTrajectory(trajInToOut)
                    robot.ExecuteTrajectory(trajInToOutRev)
                    currR = origR
                    while True:
                        try:
                            #import IPython
                            #IPython.embed()
                            print("Computing from %d to 0 degrees for in" % angle)
                            #distance = numpy.sqrt(2 * 0.2 * 0.2 * (1 - numpy.cos(angleRad)))#(angle/180.0) * numpy.pi * 0.2
                            #chains = PlanToEndEffectorOffsetTSRAdapter.CreateTSRChains(robot, [numpy.cos(angleRad), numpy.sin(angleRad), 0], distance)
                            #trajInReverse = planner.PlanToConfiguration(robot, zeroConfigIn, tsr_chains=chains, psample=0.1, smoothingitrs=0)
                            trajInReverse = snapPlanner.PlanToConfiguration(robot, zeroConfigIn)
                            print("Reversing %d to 0 degrees on in" % angle)
                            trajIn = openravepy.planningutils.ReverseTrajectory(trajInReverse)
                            openravepy.planningutils.RetimeActiveDOFTrajectory(traj, robot, False, 0.2, 0.2, "LinearTrajectoryRetimer", "")
                            break
                        except Exception, e:
                            print(str(e))
                            print("Computing arm from 0 to %d degrees on in failed, trying again..." % angle)
                break
            except Exception, e:
                print(str(e))
                print("Computing in to out trajectory for %d degrees failed, trying again..." % angle)
        trajInToOuts += trajInToOut.serialize()
        trajIns += trajIn.serialize()
        trajOuts += trajOut.serialize()

        angle = angle + 1
        currTheta = degree * angle
        moveTo(currTheta, currR, currHeight)

    trajInToOuts = trajInToOuts + '</trajectories>\n'
    trajInToOutFile = open("TrajInToOuts1.xml", "w")
    trajInToOutFile.write(trajInToOuts)
    trajInToOutFile.close()

    trajOuts = trajOuts + '</trajectories>\n'
    trajOutFile = open("TrajOuts1.xml", "w")
    trajOutFile.write(trajOuts)
    trajOutFile.close()

    trajIns = trajIns + '</trajectories>\n'
    trajInFile = open("TrajIns1.xml", "w")
    trajInFile.write(trajIns)
    trajInFile.close()

armExecuting = False
armEndTime = time()
def sweepAt(angle):

    global inToOutTrajectories
    global outTrajectories
    global inTrajectories
    global armExecuting

    #import IPython
    #IPython.embed()
    #if numpy.abs(numpy.linalg.norm(robot.arm.GetDOFValues()-getOutHomeConfig())) > 0.05:
    #    print("hasdfa1234")
    #    goToOutHome()


    if inToOutTrajectories == []:
        loadTrajectories()

    if angle == 0:
        robot.ExecuteTrajectory(openravepy.planningutils.ReverseTrajectory(inToOutTrajectories[0]))
        robot.ExecuteTrajectory(inToOutTrajectories[0])
    else:
        index = int(angle)#int(numpy.floor(angle))
        robot.ExecuteTrajectory(outTrajectories[index])
        robot.ExecuteTrajectory(openravepy.planningutils.ReverseTrajectory(inToOutTrajectories[index]))
        robot.ExecuteTrajectory(inToOutTrajectories[index])
        robot.ExecuteTrajectory(openravepy.planningutils.ReverseTrajectory(outTrajectories[index]))

def sweepAtSnap(angle):

    global inToOutTrajectories
    global outTrajectories
    global inTrajectories

    import IPython
    IPython.embed()

    if inToOutTrajectories == []:
        loadTrajectories()

    if angle == 0:
        numWaypoints = inToOutTrajectories[0].GetNumWaypoints()
        traj = snapPlanner.PlanToConfiguration(robot, inToOutTrajectories[0].GetWaypoint(numWaypoints - 1))
        openravepy.planningutils.RetimeActiveDOFTrajectory(traj, robot, False, 0.2, 0.2, "LinearTrajectoryRetimer", "")
        robot.ExecuteTrajectory(traj)
    else:
        index = int(numpy.floor(angle))

        # Moves to out config for angle
        numWaypoints = outTrajectories[index].GetNumWaypoints()
        traj = snapPlanner.PlanToConfiguration(robot, outTrajectories[index].GetWaypoint(numWaypoints - 1)[0:6])
        openravepy.planningutils.RetimeActiveDOFTrajectory(traj, robot, False, 0.2, 0.2, "LinearTrajectoryRetimer", "")
        robot.ExecuteTrajectory(traj)

        # Moves to in config for angle
        numWaypoints = inTrajectories[index].GetNumWaypoints()
        traj = snapPlanner.PlanToConfiguration(robot, inTrajectories[index].GetWaypoint(numWaypoints - 1)[0:6])
        openravepy.planningutils.RetimeActiveDOFTrajectory(traj, robot, False, 0.2, 0.2, "LinearTrajectoryRetimer", "")
        robot.ExecuteTrajectory(traj)

        # Moves to out config for angle
        numWaypoints = outTrajectories[index].GetNumWaypoints()
        traj = snapPlanner.PlanToConfiguration(robot, outTrajectories[index].GetWaypoint(numWaypoints - 1)[0:6])
        openravepy.planningutils.RetimeActiveDOFTrajectory(traj, robot, False, 0.2, 0.2, "LinearTrajectoryRetimer", "")
        robot.ExecuteTrajectory(traj)

def pose_callback(msg):
    
    global armExecuting
    global armEndTime
    print("jhere1")
    if armExecuting == False and (time() - armEndTime) > 1:
        armExecuting = True
    else:
        return
    anki_distance = msg.dist
    anki_angle = int(numpy.floor(float(msg.angle) / numpy.pi * 180))
    print(anki_distance)
    print(anki_angle)
    if anki_distance > 200 and anki_angle != -1:
        sweepAt(anki_angle)
    armExecuting = False
    armEndTime = time()


rospy.Subscriber("pose", pose, pose_callback, queue_size=1)

def main():
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

embed()
