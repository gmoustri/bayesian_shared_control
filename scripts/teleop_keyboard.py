#!/usr/bin/env python3
#
import os
import select
import sys

from rclpy.parameter_client import AsyncParameterClient
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.clock import Clock
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

DEFAULT_MAX_LIN_VEL = 0.5
DEFAULT_MAX_ANG_VEL = 1.0

DEFAULT_ACC_LIM_X   = 1.0
DEFAULT_ACC_LIM_TH  = 1.0

MAX_LIN_VEL = DEFAULT_MAX_LIN_VEL
MAX_ANG_VEL = DEFAULT_MAX_ANG_VEL
ACC_LIM_X   = DEFAULT_ACC_LIM_X
ACC_LIM_TH  = DEFAULT_ACC_LIM_TH

DT = 0.1  # 

LIN_VEL_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 0.1



msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d

w/s : increase/decrease linear velocity 
a/d : increase/decrease angular velocity

space key: force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""


def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], DT)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_vels(target_linear_velocity, target_angular_velocity):
    print('current: v {:.2f}\t w {:.2f}'.format(
        target_linear_velocity,
        target_angular_velocity))


def make_simple_profile(output_vel, input_vel, slop):
    if input_vel > output_vel:
        output_vel = min(input_vel, output_vel + slop)
    elif input_vel < output_vel:
        output_vel = max(input_vel, output_vel - slop)
    else:
        output_vel = input_vel

    return output_vel


def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel


def check_linear_limit_velocity(velocity):
    return constrain(velocity, -MAX_LIN_VEL, MAX_LIN_VEL)


def check_angular_limit_velocity(velocity):
    return constrain(velocity, -MAX_ANG_VEL, MAX_ANG_VEL)


def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    ROS_DISTRO = os.environ.get('ROS_DISTRO')
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('teleop_keyboard')

 # --- Read DWAL limits (once at startup) ---
    global MAX_LIN_VEL, MAX_ANG_VEL, ACC_LIM_X, ACC_LIM_TH

    target_node = '/dwal_planner/dwal_generator'
    param_client = AsyncParameterClient(node, target_node)
    node.get_logger().info(f'Checking parameters on {target_node}...')

    param_names = [
        'dwal_generator/max_trans_vel',
        'dwal_generator/max_vel_theta',
        'dwal_generator/acc_lim_x',
        'dwal_generator/acc_lim_th',
    ]

    defaults = {
        'dwal_generator/max_trans_vel': DEFAULT_MAX_LIN_VEL,
        'dwal_generator/max_vel_theta': DEFAULT_MAX_ANG_VEL,
        'dwal_generator/acc_lim_x': DEFAULT_ACC_LIM_X,
        'dwal_generator/acc_lim_th': DEFAULT_ACC_LIM_TH,
    }

    response_values = None

    if param_client.wait_for_services(timeout_sec=2.0):
        future = param_client.get_parameters(param_names)
        rclpy.spin_until_future_complete(node, future)
        result = future.result()
        if result is not None:
            response_values = dict(zip(param_names, result.values))
        else:
            node.get_logger().warning(
                f'Parameter request failed; using defaults for all: '
                f'max_trans_vel={DEFAULT_MAX_LIN_VEL}, max_vel_theta={DEFAULT_MAX_ANG_VEL}, '
                f'acc_lim_x={DEFAULT_ACC_LIM_X}, acc_lim_th={DEFAULT_ACC_LIM_TH}'
            )
    else:
        node.get_logger().warning(
            f'Could not contact {target_node}; using defaults for all: '
            f'max_trans_vel={DEFAULT_MAX_LIN_VEL}, max_vel_theta={DEFAULT_MAX_ANG_VEL}, '
            f'acc_lim_x={DEFAULT_ACC_LIM_X}, acc_lim_th={DEFAULT_ACC_LIM_TH}'
        )

    # ---- Single-point typing + assignment (end) ----

    def _resolve_double(name: str):
        """Return (value, used_default, note)."""
        default = defaults[name]

        if response_values is None:
            return default, True, 'no_response'

        pval = response_values.get(name)
        if pval is None:
            return default, True, 'missing'

        # 3 == PARAMETER_DOUBLE, 2 == PARAMETER_INTEGER
        if pval.type == 3:
            return pval.double_value, False, None
        if pval.type == 2:
            return float(pval.integer_value), False, 'coerced_int'

        # 0 == PARAMETER_NOT_SET (or other wrong type)
        return default, True, f'wrong_type({pval.type})'

    MAX_LIN_VEL, d1, n1 = _resolve_double('dwal_generator/max_trans_vel')
    MAX_ANG_VEL, d2, n2 = _resolve_double('dwal_generator/max_vel_theta')
    ACC_LIM_X,   d3, n3 = _resolve_double('dwal_generator/acc_lim_x')
    ACC_LIM_TH,  d4, n4 = _resolve_double('dwal_generator/acc_lim_th')

  #   LIN_VEL_STEP_SIZE = ACC_LIM_X * DT
  #  ANG_VEL_STEP_SIZE = ACC_LIM_TH * DT

    def _fmt(label, value, used_default, note):
        if used_default:
            return f'{label}={value} [default; {note}]'
        if note:
            return f'{label}={value} ({note})'
        return f'{label}={value}'
    
    node.get_logger().info(
        "Teleop params:\n"
        f"  {_fmt('max_trans_vel', MAX_LIN_VEL, d1, n1)}\n"
        f"  {_fmt('max_vel_theta', MAX_ANG_VEL, d2, n2)}\n"
        f"  {_fmt('acc_lim_x', ACC_LIM_X, d3, n3)}\n"
        f"  {_fmt('acc_lim_th', ACC_LIM_TH, d4, n4)}\n\n"
        "Derived step sizes:\n"
        f"  LIN_VEL_STEP_SIZE = {LIN_VEL_STEP_SIZE:.3f}\n"
        f"  ANG_VEL_STEP_SIZE = {ANG_VEL_STEP_SIZE:.3f}"
    )

    if ROS_DISTRO == 'humble':
        pub = node.create_publisher(Twist, 'cmd_vel', qos)
    else:
        pub = node.create_publisher(TwistStamped, 'cmd_vel', qos)

    status = 0
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0

    try:
        print(msg)
        while (1):
            key = get_key(settings)
            if key == 'w':
                target_linear_velocity =\
                    check_linear_limit_velocity(target_linear_velocity + LIN_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 's':
                target_linear_velocity =\
                    check_linear_limit_velocity(target_linear_velocity - LIN_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'a':
                target_angular_velocity =\
                    check_angular_limit_velocity(target_angular_velocity + ANG_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'd':
                target_angular_velocity =\
                    check_angular_limit_velocity(target_angular_velocity - ANG_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == ' ':
                target_linear_velocity = 0.0
                control_linear_velocity = 0.0
                target_angular_velocity = 0.0
                control_angular_velocity = 0.0
                print_vels(target_linear_velocity, target_angular_velocity)
            else:
                if (key == '\x03'):
                    break

            if status == 20:
                print('--')
                status = 0

            control_linear_velocity = make_simple_profile(
                control_linear_velocity,
                target_linear_velocity,
                (LIN_VEL_STEP_SIZE / 2.0))

            control_angular_velocity = make_simple_profile(
                control_angular_velocity,
                target_angular_velocity,
                (ANG_VEL_STEP_SIZE / 2.0))

            if ROS_DISTRO == 'humble':
                twist = Twist()
                twist.linear.x = control_linear_velocity
                twist.linear.y = 0.0
                twist.linear.z = 0.0

                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = control_angular_velocity

                pub.publish(twist)
            else:
                twist_stamped = TwistStamped()
                twist_stamped.header.stamp = Clock().now().to_msg()
                twist_stamped.header.frame_id = ''
                twist_stamped.twist.linear.x = control_linear_velocity
                twist_stamped.twist.linear.y = 0.0
                twist_stamped.twist.linear.z = 0.0

                twist_stamped.twist.angular.x = 0.0
                twist_stamped.twist.angular.y = 0.0
                twist_stamped.twist.angular.z = control_angular_velocity

                pub.publish(twist_stamped)

    except Exception as e:
        print(e)

    finally:
        if ROS_DISTRO == 'humble':
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            pub.publish(twist)
        else:
            twist_stamped = TwistStamped()
            twist_stamped.header.stamp = Clock().now().to_msg()
            twist_stamped.header.frame_id = ''
            twist_stamped.twist.linear.x = control_linear_velocity
            twist_stamped.twist.linear.y = 0.0
            twist_stamped.twist.linear.z = 0.0
            twist_stamped.twist.angular.x = 0.0
            twist_stamped.twist.angular.y = 0.0
            twist_stamped.twist.angular.z = control_angular_velocity
            pub.publish(twist_stamped)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
