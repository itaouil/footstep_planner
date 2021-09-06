# 0.3 sequence
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Forward sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.3}}'
echo "Clockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}'
echo "Counterclockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.3, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Right sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.3, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Left sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}'
echo "Forward + clockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.3}}'
echo "Forward + counterclockwise sequence finished!"
sleep 2s

for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Forward start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.3}}'; done
echo "Clockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}'; done
echo "Counterclockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.3, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Right start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.3, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Left start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.3}}'; done
echo "Forward + clockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}'; done
echo "Forward + counterclockwise start and stop sequence finished!"
sleep 2s
echo "Sequence 0.3 finished!"