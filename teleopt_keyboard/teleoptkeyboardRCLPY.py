#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control Your Rover!
---------------------------
Moving around:
        w
   a    s    d
        x
w/x : increase/decrease linear velocity 
a/d : increase/decrease angular velocity 

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

ROVER_MAX_LIN_VEL = 1.40
ROVER_MAX_ANG_VEL = 4.00

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1
settings = termios.tcgetattr(sys.stdin)

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

def constrain(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    else:
      input = input
    return input

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input
    return output

def checkLinearLimitVelocity(vel):
      vel = constrain(vel, -ROVER_MAX_LIN_VEL, ROVER_MAX_LIN_VEL)
      return vel

def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -ROVER_MAX_ANG_VEL, ROVER_MAX_ANG_VEL)
    return vel

def main(args=None):	
    if args is None:
        args = sys.argv

    rclpy.init()
    node = rclpy.create_node('teleop_twist_keyboard')
		
    pub = node.create_publisher(Twist, 'cmd_vel', 10)

    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0

    try:
        print(msg)
        print(vels(target_linear_vel,target_angular_vel))
        
        while(1):
            key = getKey()
            if key == 'w' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'x' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'a' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'd' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == ' ' or key == 's' :
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print(vels(target_linear_vel, target_angular_vel))
            else:
                if (key == '\x03'):
                    break
    
            twist = Twist()
            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
            pub.publish(twist)

    except Exception as e:
        print(e)
            
    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            
if __name__=='__main__':
      main()
