# moveit IK for ikarus teleop

--> teleop of a half_icarus urdf model, where end effector position is place where OpenTeach publishes end effector position + pose


## Moveit Setup

ros2 launch moveit_setup_assistant setup_assistant.launch.py


## Test Twist command for terminal:

ros2 topic pub -r 50 /ikarus_servo/delta_twist_cmds \
  geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: 'base_link'}, \
    twist: { \
      linear: {x: 0.0, y: 0.0, z: 0.0}, \
      angular: {x: 0.5, y: 0.0, z: 0.0} \
    }}"


## start Moveit Servo

# 1) Start Servo
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger "{}"

# 2) Make sure it's not paused
ros2 service call /servo_node/unpause_servo std_srvs/srv/Trigger "{}"  # if this service exists

## see that it does something but what does it do? doesnt publish anything on /arm_controller/joint_states

ros2 topic echo /servo_node/status


# ChatGPT 

# 1) Start/unpause servo
ros2 service call /servo_node/start_servo   std_srvs/srv/Trigger "{}"
ros2 service call /servo_node/unpause_servo std_srvs/srv/Trigger "{}"

# 2) Confirm joint states
ros2 topic echo /isaac/joint_states -n 1
ros2 topic hz /isaac/joint_states

# 3) Publish twist at 50 Hz with timestamp and correct frame
ros2 topic pub -r 50 /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped "{
  header: { stamp: 'now', frame_id: 'base_link' },
  twist: { linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5} }
}"
# -> run test node instead!!!

# 4) Watch the output
ros2 topic echo /arm_controller/joint_trajectory
ros2 topic echo /servo_node/status
