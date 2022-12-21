#!/bin/bash

# Kota Kondo, Oct 2, 2022
# This bash file will set up your NUC ready for Robust MADER
# Input:
#	(0) where to instal anaconda file (eg. /home/nuc4)
#	(1) gurobi license key (eg. cadsfasd........)
#	(2) machine name (eg. nuc4)
#	(3) machine number (eg. 4)

where2install = $0
gurobi_license_key = $1
machine_name = $2
machine_number = $3

# add all the necessary shortcuts in ~/.bashrc
echo <<EOT >> ~/.bashrc 
alias cb='catkin build'
alias gs='git status'
alias gp='git push'
alias kr='killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient & pkill -f panther & pkill -f gazebo_ros & pkill -f spawn_model & pkill -f gzserver & pkill -f gzclient  & pkill -f static_transform_publisher &  killall -9 multi_robot_node & killall -9 roscore & killall -9 rosmaster & pkill rmader_node & pkill -f tracker_predictor & pkill -f swarm_traj_planner & pkill -f dynamic_obstacles & pkill -f rosout & pkill -f behavior_selector_node & pkill -f rviz & pkill -f rqt_gui & pkill -f perfect_tracker & pkill -f rmader_commands & pkill -f dynamic_corridor & tmux kill-server & pkill -f perfect_controller & pkill -f publish_in_gazebo'
alias py='python'
alias rvmd='roscd rmader && cd rviz_cfgs && rosrun rviz rviz -d rmader.rviz'
alias tks='tmux kill-server'
EOT

# sudo user update and upgrade package
echo uavswarm | sudo -S sleep 1 && sudo apt-get update
echo uavswarm | sudo -S sleep 1 && sudo apt-get upgrade -y
echo uavswarm | sudo -S sleep 1 && sudo apt-get install terminator -y

# make directory Research
cd && mkdir Research && cd Research

# install git
echo uavswarm | sudo -S sleep 1 && sudo apt install git -y
# install vim
echo uavswarm | sudo -S sleep 1 && sudo apt install vim -y

# install ROS/Noetic
echo uavswarm | sudo -S sleep 1 && sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl -y # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full -y
echo "# ROS Noetic setup" >> ~/.bashrc
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
sudo apt install python3-rosdep 
sudo rosdep init
rosdep update

# Anaconda bash file
wget https://repo.anaconda.com/archive/Anaconda3-2022.05-Linux-x86_64.sh -P $where2install
cd $where2install && chmod +x Anaconda3-2022.05-Linux-x86_64.sh && ./Anaconda3-2022.05-Linux-x86_64.sh 
souce ~/.bashrc
conda create -n python3 python=3.8 anaconda
conda activate python3
echo conda activate python3 >> ~/.bashrc

# gurobi
conda config --add channels http://conda.anaconda.org/gurobi
conda install gurobi -y
grbgetkey gurobi_license_key
wget https://packages.gurobi.com/9.5/gurobi9.5.2_linux64.tar.gz -P $where2install
cd $where2install && tar -xzf gurobi9.5.2_linux64.tar.gz
sudo mv gurobi952/ /opt

echo "# <<< Gurobi <<<" << ~/.bashrc
echo 'export GUROBI_HOME="/opt/gurobi952/linux64"'  << ~/.bashrc
echo 'export PATH="${PATH}:${GUROBI_HOME}/bin"'  << ~/.bashrc
echo 'export LD_LIBRARY_PATH="${GUROBI_HOME}/lib"'  << ~/.bashrc


cd /opt/gurobi952/linux64/src/build
make
cp libgurobi_c++.a ../../lib/

# install catkin tools
cd
sudo apt-get install cmake python3-catkin-pkg python3-empy python3-nose python3-setuptools libgtest-dev build-essential
sudo apt install python3-catkin-tools -y

# install empy
pip3 install empy
sudo apt-get install libnlopt-dev && sudo apt install libnlopt-cxx-dev -y
sudo apt-get install libncurses5-dev -y

# set up MADER
cd
mkdir ws && cd ws && mkdir src && cd src
# git clone https://github.com/mit-acl/rmader.git
git clone git@github.com:mit-acl/rmader.git
cd ..
echo "export PYTHONPATH=/home/"$machine_name"/Research/rmader_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages" >> ~/.bashrc
source ~/.bashrc
bash src/rmader/install_and_compile.sh

# git clone all the important files
# cd ~/ws/src
# git clone https://gitlab.com/mit-acl/fsw/snap-stack/comm_monitor.git
# git clone https://gitlab.com/mit-acl/fsw/snap-stack/outer_loop.git
# git clone https://gitlab.com/mit-acl/fsw/snap-stack/snap.git
# git clone https://gitlab.com/mit-acl/fsw/snap-stack/snap_sim.git
# cd rmader/submodules/snapstack_msgs/ && git fetch origin && git checkout master
# catkin build

# comment out the old python path
sed -i '/export PYTHONPATH/s/^/#/g' ~/.bashrc
echo "PYTHONPATH=/home/"$machine_name"/Research/rmader_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages" >> ~/.bashrc
source ~/.bashrc
pip install rospkg
pip install PyQt5==5.10.1
sudo apt install net-tools

# ROS MASTER URI and ROS IP
echo << EOT >> ~/.bashrc
#export ROS_MASTER_URI=http://localhost:11311 #for local machine
export ROS_MASTER_URI=http://192.168.0.19:11311
EOT
echo "export ROS_IP=192.168.100."machine_number >> ~/.bashrc

# misc
sudo apt-get install openssh-server ii -y
sed -i '/export LD_LIBRARY_PATH/s/^/#/g' ~/.bashrc
source ~/.bashrc
pip install rospkg
