#!/usr/bin/env python
import sys

# from pyrsistent import T
import rospy
from geometry_msgs.msg import Twist
from april_detection.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose2D
import numpy as np
import time
import math
scale = 0.3
"""
The class of the pid controller.
"""
class PIDcontroller:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = np.array([0.0,0.0,0.0])
        self.I = np.array([0.0,0.0,0.0])
        self.lastError = np.array([0.0,0.0,0.0])
        self.timestep = 0.1
        self.maximumValue = 0.1

    def setTarget(self, targetx, targety, targetw):
        """
        set the target pose.
        """
        # self.I = np.array([0.0,0.0,0.0])
        # self.lastError = np.array([0.0,0.0,0.0])
        self.target = np.array([targetx, targety, targetw])

    def setTarget(self, state):
        """
        set the target pose.
        """
        # self.I = np.array([0.0,0.0,0.0])
        # self.lastError = np.array([0.0,0.0,0.0])
        self.target = np.array(state)

    def getError(self, currentState, targetState):
        """
        return the difference between two states
        """
        result = targetState - currentState
        result[2] = (result[2] + np.pi) % (2 * np.pi) - np.pi
        return result

    def setMaximumUpdate(self, mv):
        """
        set maximum velocity for stability.
        """
        self.maximumValue = mv

    def update(self, currentState ):
        """
        calculate the update value on the state based on the error between current state and target state with PID.
        """
        e = self.getError(currentState, self.target)

        P = self.Kp * e
        self.I = self.I + self.Ki * e * self.timestep
        I = self.I
        D = self.Kd * (e - self.lastError)
        result = P + I + D

        self.lastError = e

        # scale down the twist if its norm is more than the maximum value.
        resultNorm = np.linalg.norm(result)
        if(resultNorm > self.maximumValue):
            result = (result / resultNorm) * self.maximumValue
            self.I = 0.0

        return result

def genTwistMsg(desired_twist):
    """
    Convert the twist to twist msg.
    """
    twist_msg = Twist()
    twist_msg.linear.x  = desired_twist[0]
    twist_msg.linear.y  = desired_twist[1]
    twist_msg.linear.z  = 0
    twist_msg.angular.x = 0
    twist_msg.angular.y = 0
    twist_msg.angular.z = desired_twist[2]
    return twist_msg

def coord(twist, current_state):
    J = np.array([[np.cos(current_state[2]), np.sin(current_state[2]), 0.0],
                  [-np.sin(current_state[2]), np.cos(current_state[2]), 0.0],
                  [0.0,0.0,1.0]])
    return np.dot(J, twist)

def euler_from_quaternion(x, y, z, w):

    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

# Store the world frame coordinates of landmarks in this dictionary
AprilPoseDict = {}

# AprilPoseDict[5] = (1,-0.3,np.pi/4)
# AprilPoseDict[9] = (1,0.3,-np.pi/4)
# AprilPoseDict[9] = (0.6,0.0,0.0)

AprilPoseDict[9] = (1,0,0)
AprilPoseDict[4] = (1.2,0.7,-np.pi/2)
AprilPoseDict[7] = (0.9,1.5,-np.pi/2)
AprilPoseDict[8] = (0.25,1.4,-3*np.pi/4)
AprilPoseDict[5] = (-0.4,0,0)

def camera_call(cam_cmd):

    if len(camera_cmd.detections) != 0:

        global wf_robot_pos

        t_pos   = np.array([0.0,0.0,0.0])
        div     = 0
        for tag in camera_cmd.detections:
            id      = int(tag.id)
            x1      = tag.pose.position.z
            y1      = tag.pose.position.x
            qx      = tag.pose.orientation.x
            qy      = tag.pose.orientation.y
            qz      = tag.pose.orientation.z
            qw      = tag.pose.orientation.w
            y_about_x, z_about_y, x_about_z     = euler_from_quaternion(qx, qy, qz, qw)
            th1     = z_about_y

            x0, y0, th0     = AprilPoseDict[id]

            th2     = th0 - th1
            dx      = x1*np.cos(th2) + y1*np.sin(th2)
            dy      = y1*np.cos(th2) + x1*np.cos(np.pi/2 + th2)

            pos     = np.array([x0 - dx, y0 - dy, th2])
            t_pos   += pos
            div     += 1

        wf_robot_pos    = t_pos/div
        print('Robot pos in wf: ', wf_robot_pos)

    return 0

    '''
    Add steps for extraction of robot pos in WF w.r.t visible landmarks
    '''
    # _theta  = (slam_cmd.theta / 180 ) * np.pi
    wf_robot_pos    = np.array([ slam_cmd.x, slam_cmd.y, _theta ])

def navigate(_current_state, _target_state):

    if not np.array_equal(_current_state, _target_state):

        _current_state  = np.float32(_current_state)
        pid.setTarget(_target_state)

        # calculate the current twist
        update_value    = pid.update(_current_state)

        # publish the twist
        pub_twist.publish(genTwistMsg(coord(update_value, _current_state)))

        time.sleep(0.1)

        # update the current state
        _current_state = _current_state + update_value

        while(np.linalg.norm(pid.getError(_current_state, _target_state)) > 0.05): # check the error between current state and current way point
            print('Error norm: ', np.linalg.norm(pid.getError(_current_state, _target_state)))

            # Check for robot position in world frame w.r.t landmarks (April Tags)
            global wf_robot_pos
            wf_robot_pos    = np.array([0.0, 0.0, 0.0])

            # Subscribe to AprilTagDetectionArray node for updated robot position in world frame
            rospy.Subscriber('/apriltag_detection_array', AprilTagDetectionArray, camera_call, queue_size=1)

            # Update _current_state with actual position of robot in world frame
            if not np.array_equal(wf_robot_pos, _current_state) and not np.array_equal(wf_robot_pos, np.array([0.0, 0.0, 0.0])):
                print('update robot pos in if cond: ', wf_robot_pos)
                # _current_state  = wf_robot_pos

            # calculate the current twist
            update_value    = pid.update(_current_state)

            # publish the twist
            pub_twist.publish(genTwistMsg(coord(update_value, _current_state)))

            time.sleep(0.1)

            # update the current state
            _current_state  = _current_state + update_value
    else:
        pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0])))

# A function that gives possible childrens for a node (full env)
def children(node, env):

    # Directions (8-connected grid)
    move = [[0,1],[1,1],[1,0],[-1,1],[-1,0],[-1,-1],[0,-1],[1,-1]]

    # rbound,cbound = env.shape[0]-1, env.shape[1]-1
    node = np.array(node)
    ans = []
    for i in move:
        look = np.array(i)
        step = node + look
        r,c = step[0], step[1]
        if env[r,c] == 0:
            ans.append(step)
    return ans # array of np.arrays

# Heuristics (euclidean distance 2D - shorter because assumes no obstacles)
def h(node, goal):
    node = np.array(node)
    goal = np.array(goal)
    heur = np.linalg.norm(node-goal)
    return heur

parent = {}

def Astar(robotpos, targetpos, envenv):

    '''
    Fills the array next_states.

    arguments: number of nodes to expand 'n', start node 'robotpos', current goal position 'targetpos', env 'envenv'
    '''
    ulta_path = []

    OPEN, CLOSED = [], []
    OPEN.append((0 + h(robotpos, targetpos) ,robotpos))
    g = {tuple(robotpos):0}
    f = {tuple(robotpos): 0 + h(robotpos, targetpos)}


    while targetpos not in CLOSED:

        fi, i = OPEN.pop()
        CLOSED.append(tuple(i))
        for j in children(i, envenv):
            if tuple(j) not in CLOSED:
                if tuple(j) not in g or g[tuple(j)] > g[tuple(i)] + 1:
                    if tuple(j) in g:
                        temp = g[tuple(j)]
                        if (temp + h(temp, targetpos),j) in OPEN:
                            OPEN.remove((temp + h(temp, targetpos),j))
                    g[tuple(j)] = g[tuple(i)] + 1
                    parent[tuple(j)] = tuple(i)
                    f[tuple(j)] = g[tuple(j)] + h(j,targetpos)

                    OPEN.append((f[tuple(j)],j))
                    OPEN.sort(key = lambda x:x[0])
                    OPEN = OPEN[::-1]
    scale = 0.3048*0.1
    path_m = [np.array(targetpos)*scale]
    ulta_path = [tuple(targetpos)]
    while tuple(robotpos) not in ulta_path:
        # print('okay')
        node = ulta_path[-1]
        pnode = parent[node]

        ulta_path.append(tuple(pnode))
        path_m.append(np.array(pnode)*scale)

    return ulta_path[::-1], path_m[::-1]

if __name__ == "__main__":

    rospy.init_node("hw4")
    pub_twist           = rospy.Publisher("/twist", Twist, queue_size=1)

    env                 = np.zeros((100,100))

    # Car safety 0.5 feet
    # obs 1.5x1.5 feet

    env[0:5,:]          = 1
    env[:,0:5]          = 1
    env[-5:-1,:]        = 1
    env[:,-5:-1]        = 1

    obs                 = np.ones((20,20))
    env[40:60, 40:60]   = obs

    robotpos            = 10,90
    targetpos           = 90,10

    _, path_m           = Astar(robotpos, targetpos, env)
    path_3d             = []
    for i in range(len(path_m)-1):
        x0, x1  = path_m[i][0], path_m[i+1][0]
        y0, y1  = path_m[i][1], path_m[i+1][1]
        theta   = 0
        if x1-x0:
            theta   = np.arctan((y0-y1)/(x1-x0))
        wp      = np.array([x0,y0,theta])
        path_3d.append(wp)
    path_3d.append(np.array([x1,y1,theta]))

    # waypoints = [np.array([0,0,0]), np.array([scale*2,0,0]), np.array([scale*2,scale*2,np.pi]), np.array([0,scale*2,np.pi]), np.array([0,0,0])]

    for i in range(len(path_3d)-1):
        # init pid controller
        pid             = PIDcontroller(0.1,0.0,0.008)
        pid.I           = np.array([0.0,0.0,0.0])
        pid.lastError   = np.array([0.0,0.0,0.0])
        current_state   = path_3d[i]
        target_state    = path_3d[i+1]
        print('Current state: {} and target state: {}'.format(current_state, target_state))
        navigate(current_state, target_state)

    # rospy.Subscriber('/apriltag_detection_array', AprilTagDetectionArray, camera_call, queue_size=1)

    # rospy.spin()
    # stop the car and exit
    pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
    # np.savetxt("path.csv", path,delimiter =", ",fmt ='% s')
