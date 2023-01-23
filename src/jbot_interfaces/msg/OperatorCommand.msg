#
# OperatorCommand.msg
#
# Provides mechanism for switching between robot modes, and sending goals
# in autonomous mode or sending remote control commands in teleop mode.
#

std_msgs/Header header

# idle, teleop, or autonomous
string bot_mode
# in teleop mode, move using this command
geometry_msgs/Twist teleop_cmd
# in autonomous mode, try to achieve this goal position
geometry_msgs/Pose goal
