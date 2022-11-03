FROM ros:noetic

LABEL Maintainer="Kota Kondo <kkondo@mit.edu>" \
      Description="Ubuntu 20.04, ROS Noetic for MADER"

RUN apt-get update && \
      apt-get upgrade -y && \
      apt-get install -q -y --no-install-recommends git tmux vim apt-utils && \
      apt-get install -q -y --no-install-recommends wget && \
      apt-get autoremove -y && \
      apt-get clean && \
      rm -rf /var/lib/apt/lists/*

ENV ROS_DIR=/opt/ros/${ROS_DISTRO}

## gurobi
RUN wget https://packages.gurobi.com/9.5/gurobi9.5.2_linux64.tar.gz -P . && \
      tar -xzf gurobi9.5.2_linux64.tar.gz && \
      rm gurobi9.5.2_linux64.tar.gz && \
      mv gurobi952/ /opt && \
# RUN echo >> ~/.bashrc
# RUN echo export GUROBI_HOME="/opt/gurobi952/linux64"  >> ~/.bashrc
# RUN echo export PATH="${PATH}:${GUROBI_HOME}/bin"  >> ~/.bashrc
# RUN echo export LD_LIBRARY_PATH="${GUROBI_HOME}/lib"  >> ~/.bashrc
      . /opt/ros/noetic/setup.sh 
WORKDIR /opt/gurobi952/linux64/src/build
RUN apt-get update && apt-get install make && \
      apt install build-essential -y --no-install-recommends && \
      make && \
      cp libgurobi_c++.a ../../lib/ && \
      . /opt/ros/noetic/setup.sh

# # install MADER
RUN mkdir -p /home/rmader_ws/src
WORKDIR /home/rmader_ws/src
# # RUN git clone https://github.com/mit-acl/rmader.git && \
# #       apt-get install python-is-python3 -y
# RUN git clone git@github.com:mit-acl/rmader.git && \
#       apt-get install python-is-python3 -y
ADD rmader /home/rmader_ws/src/
RUN apt-get install python-is-python3 -y
WORKDIR /home/rmader_ws/
RUN echo source /opt/ros/noetic/setup.bash >> ~/.bashrc
RUN apt-get install python-is-python3 -y
WORKDIR /home/rmader_ws/

# the code below is the replacement of "RUN bash src/rmader/install_and_compile.sh"
#INSTALL CGAL v4.14.2
##########################################
RUN apt-get install libgmp3-dev libmpfr-dev -y
RUN mkdir -p /home/installations/cgal
WORKDIR /home/installations/cgal
RUN wget https://github.com/CGAL/cgal/releases/download/releases%2FCGAL-4.14.2/CGAL-4.14.2.tar.xz
RUN tar -xf CGAL-4.14.2.tar.xz
WORKDIR CGAL-4.14.2/
RUN cmake . -DCMAKE_BUILD_TYPE=Release
RUN make install

#INSTALL python-catkin-tools (to be able to use catkin build)
##########################################
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
RUN wget http://packages.ros.org/ros.key -O - |  apt-key add -
RUN apt-get install python3-catkin-tools -y

#CLONE SUBMODULES, INSTALL DEPENDENCIES AND COMPILE
##########################################
WORKDIR /home/rmader_ws/src/rmader
RUN git submodule init && git submodule update
WORKDIR /home/rmader_ws
RUN DEBIAN_FRONTEND=noninteractive rosdep install --from-paths /home/rmader_ws/src --ignore-src -r -y
RUN catkin config -DCMAKE_BUILD_TYPE=Release
RUN export GUROBI_HOME="/opt/gurobi952/linux64" && \
    export PATH="${PATH}:${GUROBI_HOME}/bin" && \
    export LD_LIBRARY_PATH="${GUROBI_HOME}/lib" && \
    . /opt/ros/noetic/setup.sh && catkin build
RUN echo source /home/rmader_ws/devel/setup.bash >> ~/.bashrc

RUN echo alias cb="catkin build" >> ~/.bashrc
RUN echo alias gs="git status" >> ~/.bashrc
RUN echo alias gp="git push" >> ~/.bashrc
RUN echo alias kr="killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient & pkill -f panther & pkill -f gazebo_ros & pkill -f spawn_model & pkill -f gzserver & pkill -f gzclient  & pkill -f static_transform_publisher &  killall -9 multi_robot_node & killall -9 roscore & killall -9 rosmaster & pkill rmader_node & pkill -f tracker_predictor & pkill -f swarm_traj_planner & pkill -f dynamic_obstacles & pkill -f rosout & pkill -f behavior_selector_node & pkill -f rviz & pkill -f rqt_gui & pkill -f perfect_tracker & pkill -f rmader_commands & pkill -f dynamic_corridor & tmux kill-server & pkill -f perfect_controller & pkill -f publish_in_gazebo" >> ~/.bashrc
RUN echo alias py="python" >> ~/.bashrc
RUN echo alias rvmd="roscd rmader && cd rviz_cfgs && rosrun rviz rviz -d rmader.rviz" >> ~/.bashrc
RUN echo alias tks="tmux kill-server" >> ~/.bashrc
RUN apt-get install psmisc