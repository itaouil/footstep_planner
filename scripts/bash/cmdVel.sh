timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.85, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: -0.85, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.85}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.85}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.85, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.85, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Sequence finished!"