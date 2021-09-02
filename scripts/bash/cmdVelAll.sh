# 0.1
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: -0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.1}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "0.1 sequence finished!"

# 0.15
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: -0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.15}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.15}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.15, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.15, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "0.15 sequence finished!"

# 0.2
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: -0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.2}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "0.2 sequence finished!"

# 0.25
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.25, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: -0.25, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.25}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.25}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.25, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.25, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "0.25 sequence finished!"

# 0.3
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: -0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.3}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.3, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.3, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "0.3 sequence finished!"

# 0.35
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.35, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: -0.35, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.35}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.35}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.35, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.35, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "0.35 sequence finished!"

# 0.4
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.4, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: -0.4, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.4}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.4}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.4, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.4, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "0.4 sequence finished!"

# 0.45
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.45, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: -0.45, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.45}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.45}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.45, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.45, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "0.45 sequence finished!"

# 0.5
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: -0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "0.5 sequence finished!"

# 0.55
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.55, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: -0.55, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.55}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.55}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.55, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.55, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "0.55 sequence finished!"

# 0.6
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.6, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: -0.6, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.6}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.6}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.6, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.6, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "0.6 sequence finished!"

# 0.65
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.65, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: -0.65, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.65}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.65}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.65, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.65, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "0.65 sequence finished!"

# 0.7
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.7, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: -0.7, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.7}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.7}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.7, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.7, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "0.7 sequence finished!"

# 0.75
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.75, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: -0.75, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.75}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.75}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.75, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.75, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "0.75 sequence finished!"

# 0.8
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.8, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: -0.8, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.8}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.8}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.8, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.8, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "0.8 sequence finished!"

# 0.85
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.85, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: -0.85, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.85}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.85}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.85, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.85, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "0.85 sequence finished!"

# 0.9
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.9, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: -0.9, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.9}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.9}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.9, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.9, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "0.9 sequence finished!"