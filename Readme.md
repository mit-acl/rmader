# Robust MADER: Decentralized and Asynchronous Multiagent Trajectory Planner Robust to Communication Delay #


### **Submitted to 2023 IEEE International Conference on Robotics and Automation (ICRA)**


|100ms Delay Check          |200ms Delay Check          |
| ------------------------- | ------------------------- |
<a target="_blank" href="https://youtu.be/vH09kwJOBYs"><img src="./rmader/imgs/sim_dc100ms.gif" width="470" height="260" alt="Robust MADER: Decentralized and Asynchronous Multiagent Traj. Planner Robust to Communication Delay"></a> | <a target="_blank" href="https://youtu.be/vH09kwJOBYs"><img src="./rmader/imgs/sim_dc200ms.gif" width="470" height="260" alt="Robust MADER: Decentralized and Asynchronous Multiagent Traj. Planner Robust to Communication Delay"></a> | 

|400ms Delay Check | Hardware Experiments |
| ------------------------- | ------------------------- |
|<a target="_blank" href="https://youtu.be/vH09kwJOBYs"><img src="./rmader/imgs/sim_dc400ms.gif" width="470" height="260" alt="Robust MADER: Decentralized and Asynchronous Multiagent Traj. Planner Robust to Communication Delay"></a> | <a target="_blank" href="https://youtu.be/vH09kwJOBYs"><img src="./rmader/imgs/hw.gif" width="470" height="260" style="margin:20px 20px" alt="Robust MADER: Decentralized and Asynchronous Multiagent Traj. Planner Robust to Communication Delay"></a>|  

## Citation

Please cite [Robust MADER: Decentralized and Asynchronous Multiagent Trajectory Planner Robust to Communication Delay](https://arxiv.org/abs/2209.13667) ([pdf](https://arxiv.org/abs/2209.13667), [video](https://youtu.be/vH09kwJOBYs)):

```bibtex
@article{kondo2022robust,
  title={Robust MADER: Decentralized and Asynchronous Multiagent Trajectory Planner Robust to Communication Delay},
  author={Kondo, Kota and Tordesillas, Jesus and Figueroa, Reinaldo and Rached, Juan and Merkel, Joseph and Lusk, Parker C and How, Jonathan P},
  journal={arXiv preprint arXiv:2209.13667},
  year={2022}
}
```

## General Setup

RMADER has been tested with Ubuntu 20.04/ROS Noetic

### Not Using Docker

The backend optimizer is Gurobi. Please install the [Gurobi Optimizer](https://www.gurobi.com/products/gurobi-optimizer/), and test your installation typing `gurobi.sh` in the terminal. Have a look at [this section](#issues-when-installing-gurobi) if you have any issues.

Then simply run this commands:

```bash
cd ~/ && mkdir ws && cd ws && mkdir src && cd src
git clone https://github.com/mit-acl/rmader.git
cd ..
bash src/rmader/install_and_compile.sh      
```

The script [install_and_compile.sh](https://github.com/mit-acl/mader/blob/master/install_and_compile.sh) will install [CGAL v4.12.4](https://www.cgal.org/), [GLPK](https://www.gnu.org/software/glpk/) and other ROS packages (check the script for details). It will also compile the repo. This bash script assumes that you already have ROS installed in your machine. 

### Using Docker

Install Docker using [this steps](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository), and remove the need of `sudo` following [these steps](https://docs.docker.com/engine/install/linux-postinstall/). Then follow these steps:

```bash
cd ~/ && mkdir ws && cd ws && mkdir src && cd src
git clone https://github.com/mit-acl/rmader.git
```

For Gurobi, you need to download gurobi.lic file from [Gurobi Web License Manager](https://license.gurobi.com/manager/licenses) (more info [here](https://www.gurobi.com/web-license-service/)). A gurobi.lic not obtained through WLS will **not** work on docker. Place your gurobi.lic in [docker](https://github.com/mit-acl/mader/docker) folder and execute these commands:

```bash
cd ./mader/mader/docker
docker build -t mader . #This will probably take several minutes
```
Once built, ```docker run --volume=$PWD/gurobi.lic:/opt/gurobi/gurobi.lic:ro -it mader```

<details>
  <summary> <b>Useful Docker commands</b></summary>
  
```bash
docker container ls -a  #Show a list of the containers
docker rm $(docker ps -aq) #remove all the containers
docker image ls #Show a lis of the images
docker image rm XXX #remove a specific image

### lambda machine simulation
docker build -f rmader/rmader/docker/Dockerfile -t rmader .
docker run --cpus=48 --volume=/home/kkondo/rmader_project/rmader_ws/src/rmader/rmader/docker/gurobi.lic:/opt/gurobi/gurobi.lic:ro --volume=/home/kkondo/data:/home/kota/data -it rmader
```

</details>


### Running Simulations

#### Single-agent
```
roslaunch rmader single_agent_simulation.launch
```
Now you can press `G` (or click the option `2D Nav Goal` on the top bar of RVIZ) and click any goal for the drone. 

To run many single-agent simulations in different random environments, you can go to the `scripts` folder and execute `python run_many_sims_single_agent.py`.

#### Multi-agent

> **Note**: For a high number of agents, the performance of MADER improves with the number of CPUs available in your computer. 

Open four terminals and run these commands:

```
roslaunch mader mader_general.launch type_of_environment:="dynamic_forest"
roslaunch mader many_drones.launch action:=start
roslaunch mader many_drones.launch action:=mader
roslaunch mader many_drones.launch action:=send_goal
```

(if you want to modify the drone radius, you can do so in `mader.yaml`). For the tables shown in the paper, the parameters (drone radius, max vel,...) used are also detailed in the corresponding section of the paper


#### Octopus Search
You can run the octopus search with a dynamic obstacle by simply running

```
roslaunch mader octopus_search.launch
```
And you should obtain this:

![](./mader/imgs/octopus_search.png) 

(note that the octopus search has some randomness in it, so you may obtain a different result each time you run it).

## Issues when installing Gurobi:

If you find the error:
```
“gurobi_continuous.cpp:(.text.startup+0x74): undefined reference to
`GRBModel::set(GRB_StringAttr, std::__cxx11::basic_string<char,
std::char_traits<char>, std::allocator<char> > const&)'”
```
The solution is:

```bash
cd /opt/gurobi800/linux64/src/build  #Note that the name of the folder gurobi800 changes according to the Gurobi version
sudo make
sudo cp libgurobi_c++.a ../../lib/
```

## Credits:
This package uses some C++ classes from the [DecompROS](https://github.com/sikang/DecompROS) repo (included in the `thirdparty` folder).

## Hardware experiments procedure

1. `./goal pos/ran` (argument pos: position exchange, ran: random waypoints)
2. `./mader_fly true` (set argment to true if you wanna delete all the bags recored on voxl)
3. check if all the mader hw_onboard is running onboard
4. check if vicon values are appropriate
5. in tmux windows opened by ./goal_send `setw synchronus-pane`
6. arm drones
7. (optional) video recording
8. after flight, disarm drones and stop video recording

---------

> **Approval for release**: This code was approved for release by The Boeing Company in December 2020. 
