#!/usr/bin/env python		
#-*-coding:utf-8-*-		
import rospy
import tf
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from math import pow, atan2, sqrt, pi, degrees
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler


def euclidean_distance(xd, yd, zd):
    '''
    computes euclidean distance given:
    xd = xa - xb
    yd = ya - yb
    '''
    return sqrt(pow((xd), 2) + pow((yd), 2) + pow((zd), 2))


class State:
    '''
    just a helper class
    '''
    def __init__(self, x = 0, y = 0, z = 0, theta = 0):
        self.x = x
        self.y = y
		self.z = z
        self.theta = theta

class PID: 
    '''
    just an other helper class that computes the pid equation
    '''
    def __init__(self,kp=1,kd=0,ki=0,dt=0.1):

        #GAINS
        self.kp = kp
        self.kd = kd
        self.ki = ki

        #TIME STEP
        self.dt = dt

        #DERROR INITIALIZATION
        self.err_previous = 0.001
        self.err_acc = 0

    def compute(self,err):

        #compute dervivative
        err_deriv = (err - self.err_previous)/self.dt
        
        #update integration
        self.err_acc = self.err_acc + self.dt * (err + self.err_previous)/2
        
        #compute pid equation
        pid = self.kp*err + self.kd*err_deriv + self.ki*self.err_acc

        #update error
        self.err_previous = err

        return pid

class Controller:
    '''
    main class of the ros node controlling the robot.
    '''

    def __init__(self):

        #initialization of the ros node and relevant pub/sub
        rospy.init_node("PID_node")
        self.velocity_publisher = rospy.Publisher("/bebop/cmd_vel",Twist,queue_size=1)
        self.bebop_subscriber = rospy.Subscriber("/vicon/bebop/bebop", geometry_msgs.msg.TransformStamped ,self.bebop_callback)
        
        #robot current state
        self.state = State()

        #controller frequency in Hz
        self.hz=50.0
        self.rate = rospy.Rate(self.hz)
        self.dt=(1.0/self.hz)

        #define pids
        self.pid_rho = PID(kp=0.3,dt=self.dt)
        self.pid_alpha = PID(kp=0.6,dt=self.dt)
        self.pid_beta = PID(kp=0.3,dt=self.dt)



    def bebop_callback(self,msg):
        #updates the current state 
        self.state.x = msg.transform.translation.x
        self.state.y = msg.transform.translation.y
		self.state.z = msg.transform.translation.z
        (_,_,self.state.theta) = euler_from_quaternion([msg.transform.rotation.x, 
        										msg.transform.rotation.y,
                                                msg.transform.rotation.z,
                                                msg.transform.rotation.w])

    def move_to_goal(self,goal_pose):

        #variable initialization
        vel_msg = Twist()
        tolerance_position = 0.01
        tolerance_orientation = 0.1

        #retrieve goal
        goal_x = goal_pose.position.x
        goal_y = goal_pose.position.y
		goal_z = goal_pose.position.z
        (_,_,goal_theta) = euler_from_quaternion([goal_pose.orientation.x,goal_pose.orientation.y,goal_pose.orientation.z,goal_pose.orientation.w])

        rho = euclidean_distance((goal_x - self.state.x),(goal_y - self.state.y),(goal_z - self.state.z))

        while rho >= tolerance_position:
            rospy.loginfo("Distance from goal:"+str(rho))
            rospy.loginfo("My orientation is: "+str(degrees(self.state.theta)) + "but my goal is: " + str(degrees(goal_theta)))

            #errors
            rho = euclidean_distance((goal_x - self.state.x),(goal_y - self.state.y),(goal_z - self.state.z))
	    	err_x = goal_x - self.state.x
	    	err_y = goal_y - self.state.y
	    	err_z = goal_z - self.state.z
            alpha = atan2((goal_y - self.state.y), (goal_x - self.state.x)) - self.state.theta
            beta = 0

            #Compute PID
            vx = self.pid_rho.compute(err_x)
	    	vy = self.pid_rho.compute(err_y)
	    	vz = self.pid_rho.compute(err_z)
            w = self.pid_alpha.compute(alpha) - self.pid_beta.compute(beta)

            #fill message
            vel_msg.linear.x = vx 
            vel_msg.linear.y = vy
            vel_msg.linear.z = vz
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = w

            #debugging
            print(vel_msg.linear.x)
	    	print(vel_msg.linear.y)
	    	print(vel_msg.linear.z)
            print(vel_msg.angular.z)
            print("_________________")

            #publish
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        #stop the robot
        vel_msg.linear.x=0.0
		vel_msg.linear.y=0.0
		vel_msg.linear.z=0.0
        vel_msg.angular.z=0.0
        self.velocity_publisher.publish(vel_msg)
        rospy.loginfo("I'm here: "+ str(self.state.x) + " , " + str(self.state.y) +" , " + str(degrees(self.state.theta)))
        print("___")

        #return
        return

#test script
if __name__ == '__main__':
    try:
        x = Controller()

        #DEFINE THE GOAL
        goal1=Pose()

        goal1.position.x=2.0
        goal1.position.y=2.0
        goal1.position.z=0.8

        quaternion = quaternion_from_euler(0.0,0.0,0)

        goal1.orientation.x = quaternion[0]
        goal1.orientation.y = quaternion[1]
        goal1.orientation.z = quaternion[2]
        goal1.orientation.w = quaternion[3]

        goal2=Pose()
        goal2.position.x=3
        goal2.position.y=3
        goal2.position.z=0.6

        quaternion = quaternion_from_euler(0.0,0.0,0.4)

        goal2.orientation.x = quaternion[0]
        goal2.orientation.y = quaternion[1]
        goal2.orientation.z = quaternion[2]
        goal2.orientation.w = quaternion[3]
        
        
        #MOVE TO THE GOALS
        x.move_to_goal(goal1)
        x.move_to_goal(goal2)

        #spin
        rospy.spin()

    except rospy.ROSInterruptException:
	pass		

