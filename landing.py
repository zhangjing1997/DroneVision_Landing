
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
    return sqrt(pow((xd), 2) + pow((yd), 2) + pow((zd), 2))


class relativeState:
    def __init__(self, dx = 0, dy = 0, dz = 0):
        self.dx = dx
        self.dy = dy
		self.dz = dz

class PID:
    def __init__(self,kp=1,kd=0,ki=0,dt=0.1):

        #GAINS
        self.kp = kp
        self.kd = kd
        self.ki = ki

        #TIME STEP
        self.dt = dt

        #Default ERROR INITIALIZATION
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
        self.velocity_publisher=rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped",Twist,queue_size=1) 
        self.bebop_subscriber=rospy.Subscriber("/relative_distance", msg ,self.call_back)

        #robot current state
        self.state = State()

        #controller frequency in Hz
        self.hz=50.0
        self.rate = rospy.Rate(self.hz)
        self.dt=(1.0/self.hz)

        #define pids
        self.pid_rho = PID(kp=0.3,dt=self.dt)

    # transformation
    def callback(self, msg):
        self.state.x = msg[1]
        self.state.y = msg[0]
        self.state.z = -msg[2]

    def move_to_goal(self):

        #variable initialization
        vel_msg = Twist()
        tolerance_position = 0.01

        rho = euclidean_distance(self.state.x, self.state.y, self.state.z)

        while rho >= tolerance_position:
            rospy.loginfo("Distance from goal:"+str(rho))

            rho = euclidean_distance(self.state.x, self.state.y, self.state.z)
            err_x = self.state.x
            err_y = self.state.y
            err_z = self.state.z

            #Compute PID
            vx = self.pid_rho.compute(err_x)
            vy = self.pid_rho.compute(err_y)
            vz = self.pid_rho.compute(err_z)

            #fill message
            vel_msg.linear.x = vx 
            vel_msg.linear.y = vy
            vel_msg.linear.z = vz
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 0.0

            #debugging
            print(vel_msg.linear.x)
            print(vel_msg.linear.y)
            print(vel_msg.linear.z)
            print("_________________")

            #publish
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        #stop the robot
        vel_msg.linear.x=0.0
        vel_msg.linear.y=0.0
        vel_msg.linear.z=0.0
        self.velocity_publisher.publish(vel_msg)
        rospy.loginfo("I'm here(relative info): "+ str(self.state.x) + " , " + str(self.state.y) +" , " + str(self.state.z))
        print("___")

        return

test script
if __name__ == '__main__':
    try:
        x = Controller()

        #MOVE TO THE GOALS
        x.move_to_goal()

        #spin
        rospy.spin()

    except rospy.ROSInterruptException:
    pass        









