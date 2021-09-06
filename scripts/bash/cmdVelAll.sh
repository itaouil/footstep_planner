# shellcheck disable=SC2034

# 0.1 sequence
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Forward sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.1}}'
echo "Clockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}'
echo "Counterclockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Right sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Left sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}'
echo "Forward + clockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.1}}'
echo "Forward + counterclockwise sequence finished!"
sleep 2s

for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Forward start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.1}}'; done
echo "Clockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}'; done
echo "Counterclockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Right start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Left start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.1}}'; done
echo "Forward + clockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}'; done
echo "Forward + counterclockwise start and stop sequence finished!"
sleep 2s
echo "Sequence 0.1 finished!"

# 0.15 sequence
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Forward sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.15}}'
echo "Clockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.15}}'
echo "Counterclockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.15, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Right sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.15, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Left sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.15}}'
echo "Forward + clockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.15}}'
echo "Forward + counterclockwise sequence finished!"
sleep 2s

for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Forward start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.15}}'; done
echo "Clockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.15}}'; done
echo "Counterclockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.15, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Right start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.15, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Left start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.15}}'; done
echo "Forward + clockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.15}}'; done
echo "Forward + counterclockwise start and stop sequence finished!"
sleep 2s
echo "Sequence 0.15 finished!"

# 0.2 sequence
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Forward sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.2}}'
echo "Clockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}'
echo "Counterclockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Right sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Left sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}'
echo "Forward + clockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.2}}'
echo "Forward + counterclockwise sequence finished!"
sleep 2s

for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Forward start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.2}}'; done
echo "Clockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}'; done
echo "Counterclockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Right start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Left start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.2}}'; done
echo "Forward + clockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}'; done
echo "Forward + counterclockwise start and stop sequence finished!"
sleep 2s
echo "Sequence 0.2 finished!"

# 0.25 sequence
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.25, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Forward sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.25}}'
echo "Clockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.25}}'
echo "Counterclockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.25, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Right sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.25, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Left sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.25, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.25}}'
echo "Forward + clockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.25, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.25}}'
echo "Forward + counterclockwise sequence finished!"
sleep 2s

for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.25, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Forward start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.25}}'; done
echo "Clockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.25}}'; done
echo "Counterclockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.25, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Right start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.25, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Left start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.25, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.25}}'; done
echo "Forward + clockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.25, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.25}}'; done
echo "Forward + counterclockwise start and stop sequence finished!"
sleep 2s
echo "Sequence 0.25 finished!"

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

# 0.35 sequence
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.35, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Forward sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.35}}'
echo "Clockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.35}}'
echo "Counterclockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.35, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Right sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.35, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Left sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.35, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.35}}'
echo "Forward + clockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.35, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.35}}'
echo "Forward + counterclockwise sequence finished!"
sleep 2s

for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.35, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Forward start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.35}}'; done
echo "Clockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.35}}'; done
echo "Counterclockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.35, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Right start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.35, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Left start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.35, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.35}}'; done
echo "Forward + clockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.35, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.35}}'; done
echo "Forward + counterclockwise start and stop sequence finished!"
sleep 2s
echo "Sequence 0.35 finished!"

# 0.4 sequence
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.4, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Forward sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.4}}'
echo "Clockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.4}}'
echo "Counterclockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.4, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Right sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.4, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Left sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.4, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.4}}'
echo "Forward + clockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.4, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.4}}'
echo "Forward + counterclockwise sequence finished!"
sleep 2s

for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.4, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Forward start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.4}}'; done
echo "Clockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.4}}'; done
echo "Counterclockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.4, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Right start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.4, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Left start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.4, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.4}}'; done
echo "Forward + clockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.4, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.4}}'; done
echo "Forward + counterclockwise start and stop sequence finished!"
sleep 2s
echo "Sequence 0.4 finished!"

# 0.45 sequence
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.45, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Forward sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.45}}'
echo "Clockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.45}}'
echo "Counterclockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.45, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Right sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.45, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Left sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.45, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.45}}'
echo "Forward + clockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.45, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.45}}'
echo "Forward + counterclockwise sequence finished!"
sleep 2s

for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.45, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Forward start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.45}}'; done
echo "Clockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.45}}'; done
echo "Counterclockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.45, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Right start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.45, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Left start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.45, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.45}}'; done
echo "Forward + clockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.45, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.45}}'; done
echo "Forward + counterclockwise start and stop sequence finished!"
sleep 2s
echo "Sequence 0.45 finished!"

# 0.5 sequence
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Forward sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}'
echo "Clockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'
echo "Counterclockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Right sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Left sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'
echo "Forward + clockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}'
echo "Forward + counterclockwise sequence finished!"
sleep 2s

for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Forward start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}'; done
echo "Clockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'; done
echo "Counterclockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Right start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Left start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}'; done
echo "Forward + clockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'; done
echo "Forward + counterclockwise start and stop sequence finished!"
sleep 2s
echo "Sequence 0.5 finished!"

# 0.55 sequence
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.55, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Forward sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.55}}'
echo "Clockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.55}}'
echo "Counterclockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.55, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Right sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.55, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Left sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.55, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.55}}'
echo "Forward + clockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.55, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.55}}'
echo "Forward + counterclockwise sequence finished!"
sleep 2s

for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.55, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Forward start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.55}}'; done
echo "Clockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.55}}'; done
echo "Counterclockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.55, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Right start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.55, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Left start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.55, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.55}}'; done
echo "Forward + clockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.55, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.55}}'; done
echo "Forward + counterclockwise start and stop sequence finished!"
sleep 2s
echo "Sequence 0.55 finished!"

# 0.6 sequence
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.6, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Forward sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.6}}'
echo "Clockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.6}}'
echo "Counterclockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.6, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Right sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.6, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Left sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.6, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.6}}'
echo "Forward + clockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.6, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.6}}'
echo "Forward + counterclockwise sequence finished!"
sleep 2s

for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.6, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Forward start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.6}}'; done
echo "Clockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.6}}'; done
echo "Counterclockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.6, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Right start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.6, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Left start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.6, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.6}}'; done
echo "Forward + clockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.6, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.6}}'; done
echo "Forward + counterclockwise start and stop sequence finished!"
sleep 2s
echo "Sequence 0.6 finished!"

# 0.65 sequence
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.65, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Forward sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.65}}'
echo "Clockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.65}}'
echo "Counterclockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.65, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Right sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.65, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Left sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.65, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.65}}'
echo "Forward + clockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.65, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.65}}'
echo "Forward + counterclockwise sequence finished!"
sleep 2s

for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.65, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Forward start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.65}}'; done
echo "Clockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.65}}'; done
echo "Counterclockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.65, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Right start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.65, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Left start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.65, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.65}}'; done
echo "Forward + clockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.65, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.65}}'; done
echo "Forward + counterclockwise start and stop sequence finished!"
sleep 2s
echo "Sequence 0.65 finished!"

# 0.7 sequence
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.7, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Forward sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.7}}'
echo "Clockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.7}}'
echo "Counterclockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.7, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Right sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.7, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Left sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.7, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.7}}'
echo "Forward + clockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.7, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.7}}'
echo "Forward + counterclockwise sequence finished!"
sleep 2s

for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.7, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Forward start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.7}}'; done
echo "Clockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.7}}'; done
echo "Counterclockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.7, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Right start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.7, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Left start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.7, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.7}}'; done
echo "Forward + clockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.7, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.7}}'; done
echo "Forward + counterclockwise start and stop sequence finished!"
sleep 2s
echo "Sequence 0.7 finished!"

# 0.75 sequence
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.75, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Forward sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.75}}'
echo "Clockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.75}}'
echo "Counterclockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.75, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Right sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.75, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Left sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.75, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.75}}'
echo "Forward + clockwise sequence finished!"
sleep 2s
timeout -sHUP 1m rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.75, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.75}}'
echo "Forward + counterclockwise sequence finished!"
sleep 2s

for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.75, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Forward start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.75}}'; done
echo "Clockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.75}}'; done
echo "Counterclockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -0.75, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Right start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.75, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; done
echo "Left start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.75, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.75}}'; done
echo "Forward + clockwise start and stop sequence finished!"
sleep 2s
for i in {1..30}; do timeout -sHUP 2s rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.75, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.75}}'; done
echo "Forward + counterclockwise start and stop sequence finished!"
sleep 2s
echo "Sequence 0.75 finished!"