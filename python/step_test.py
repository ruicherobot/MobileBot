#! /usr/bin/python


from lcmtypes import mbot_motor_command_t
import lcm
from time import sleep
import sys
sys.path.append("lcmtypes")


lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

stop_command = mbot_motor_command_t()
stop_command.trans_v = 0.0
stop_command.angular_v = 0.0

qs = 4.0 # 1.0 is fast, 4.0 is slow

drive_straight_command = mbot_motor_command_t()
drive_straight_command.trans_v = 0.8 / qs
drive_straight_command.angular_v = 0.0


drive_turn_left_command = mbot_motor_command_t()
drive_turn_left_command.trans_v = 0 
drive_turn_left_command.angular_v = 3.1416 / qs

drive_turn_right_command = mbot_motor_command_t()
drive_turn_right_command.trans_v = 0 
drive_turn_right_command.angular_v = - 3.1416 / qs

drive_curve_left_command = mbot_motor_command_t()
drive_curve_left_command.trans_v = 0.942 / qs #contiune straight while going left
drive_curve_left_command.angular_v = 3.1416 / qs

drive_curve_right_command = mbot_motor_command_t()
drive_curve_right_command.trans_v = 0.942 / qs #contiune straight while going right
drive_curve_right_command.angular_v = - 3.1416 / qs

#low pass on setpoint
#low pass on end
#dont stop

lc.publish("MBOT_MOTOR_COMMAND", drive_straight_command.encode())
sleep(0.3625 * qs) #30 cm

lc.publish("MBOT_MOTOR_COMMAND", drive_curve_right_command.encode())
sleep(0.5 * qs)

lc.publish("MBOT_MOTOR_COMMAND", drive_curve_left_command.encode())
sleep(0.5 * qs)

lc.publish("MBOT_MOTOR_COMMAND", drive_curve_left_command.encode())
sleep(0.5 * qs)

lc.publish("MBOT_MOTOR_COMMAND", drive_straight_command.encode())
sleep(0.725 * qs)

lc.publish("MBOT_MOTOR_COMMAND", drive_curve_right_command.encode())
sleep(0.5 * qs)

lc.publish("MBOT_MOTOR_COMMAND", drive_curve_right_command.encode())
sleep(0.5 * qs)

lc.publish("MBOT_MOTOR_COMMAND", drive_straight_command.encode())
sleep(0.725 * qs)

lc.publish("MBOT_MOTOR_COMMAND", drive_curve_left_command.encode())
sleep(0.5 * qs)

lc.publish("MBOT_MOTOR_COMMAND", drive_curve_left_command.encode())
sleep(0.5 * qs)

lc.publish("MBOT_MOTOR_COMMAND", drive_curve_right_command.encode())
sleep(0.5 * qs)

lc.publish("MBOT_MOTOR_COMMAND", drive_straight_command.encode())
sleep(0.3625 * qs)



#......................................................................

# lc.publish("MBOT_MOTOR_COMMAND", drive_straight_command.encode())
# sleep(0.725 * qs)

# lc.publish("MBOT_MOTOR_COMMAND", drive_turn_left_command.encode())
# sleep(0.5 * qs)

# lc.publish("MBOT_MOTOR_COMMAND", drive_straight_command.encode())
# sleep(0.725 * qs)

# lc.publish("MBOT_MOTOR_COMMAND", drive_turn_right_command.encode())
# sleep(0.5 * qs)

# lc.publish("MBOT_MOTOR_COMMAND", drive_straight_command.encode())
# sleep(0.725 * qs)

# lc.publish("MBOT_MOTOR_COMMAND", drive_turn_right_command.encode())
# sleep(0.5 * qs)

# lc.publish("MBOT_MOTOR_COMMAND", drive_straight_command.encode())
# sleep(1.45 * qs)

# lc.publish("MBOT_MOTOR_COMMAND", drive_turn_left_command.encode())
# sleep(0.5 * qs)

# lc.publish("MBOT_MOTOR_COMMAND", drive_straight_command.encode())
# sleep(0.725 * qs)

# lc.publish("MBOT_MOTOR_COMMAND", drive_turn_left_command.encode())
# sleep(0.5 * qs)

# lc.publish("MBOT_MOTOR_COMMAND", drive_straight_command.encode())
# sleep(1.45 * qs)

# lc.publish("MBOT_MOTOR_COMMAND", drive_turn_right_command.encode())
# sleep(0.5 * qs)

# lc.publish("MBOT_MOTOR_COMMAND", drive_straight_command.encode())
# sleep(0.725 * qs)

# lc.publish("MBOT_MOTOR_COMMAND", drive_turn_right_command.encode())
# sleep(0.5 * qs)

# lc.publish("MBOT_MOTOR_COMMAND", drive_straight_command.encode())
# sleep(0.725 * qs)

# lc.publish("MBOT_MOTOR_COMMAND", drive_turn_left_command.encode())
# sleep(0.5 * qs)

# lc.publish("MBOT_MOTOR_COMMAND", drive_straight_command.encode())
# sleep(0.725 * qs)


lc.publish("MBOT_MOTOR_COMMAND", stop_command.encode())
sleep(1.0)
