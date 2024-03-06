cd /home/gr-agv-lx91/isaac_sim_ws || exit
source devel/setup.bash

if [ $# -lt 1 ]
then
  echo "please input the number of robot"
  exit
else
  robot_num=$1
fi

session="multi_robot_navigation"
tmux new-session -d -s $session

window=0
tmux rename-window -t $session:$window "roscore"
tmux send-keys -t $session:$window "roscore" C-m
echo "start roscore"
sleep 2s

window=$((window + 1))
tmux new-window -t $session:"$window" -n "rviz"
tmux send-keys -t $session:"$window" "rosparam load src/isaac_sim.config/multi_robots_config" C-m
tmux send-keys -t $session:"$window" "rviz -d ~/isaac_sim_ws/src/isaac_sim/rviz/carter.rviz" C-m
echo "start rviz and load multi robot parameters"

i=0
while [ $i -lt $((robot_num)) ]
do
  window=$((window + 1))
  tmux new-window -t $session:"$window" -n "Carter_$i"
  tmux send-keys -t $session:"$window" "source ~/isaac_sim_ws/devel/setup.bash" C-m
  tmux send-keys -t $session:"$window" "roslaunch isaac_sim navigation.launch prefix:=Carter_${i} launch_rviz:=false disable_teb:=false" C-m
  echo "No.$i robot navigation finish launching"
  i=$((i + 1))
done