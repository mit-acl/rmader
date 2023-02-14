# Robust MADER: Decentralized and Asynchronous Multiagent Trajectory Planner Robust to Communication Delay #

#### **Accepted to 2023 IEEE International Conference on Robotics and Automation (ICRA)**
#### **Submitted to IEEE Robotics and Automation Society (RA-L)**  

|4 agents with 2 dynamic obstacles|6 agents with 2 dynamic obstacles|
| ------------------------- | ------------------------- |
<a target="_blank" href="https://youtu.be/vH09kwJOBYs"><img src="./rmader/imgs/4agent2obs_shorter.gif" width="400" height="221" alt="Hardware experiment: 4 agent with 2 dynamic obstacles"></a> | <a target="_blank" href="https://youtu.be/vH09kwJOBYs"><img src="./rmader/imgs/6agent2obs_shorter.gif" width="400" height="221" alt="Hardware experiment: 6 agent with 2 dynamic obstacles"></a> | 

|10 agents under 300ms comm. delay|10 agents with 10 dynamic obstacles under 50ms comm. delay |
| ------------------------- | ------------------------- |
|<a target="_blank" href="https://youtu.be/vH09kwJOBYs"><img src="./rmader/imgs/rmader_cd300_sim_github.gif" width="400" height="221" alt="Simulation: 10 agents under 300ms comm. delay"></a> | <a target="_blank" href="https://youtu.be/vH09kwJOBYs"><img src="./rmader/imgs/rmader_obs_sim_github.gif" width="400" height="221" style="margin:20px 20px" alt="Simulation: 10 agents with 10 dynamic obstacles under 50ms comm. delay"></a>|  

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

The script [install_and_compile.sh](https://github.com/mit-acl/rmader/blob/master/install_and_compile.sh) will install [CGAL v4.12.4](https://www.cgal.org/), [GLPK](https://www.gnu.org/software/glpk/) and other ROS packages (check the script for details). It will also compile the repo. This bash script assumes that you already have ROS installed in your machine. 

### Using Docker

Install Docker using [this steps](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository), and remove the need of `sudo` following [these steps](https://docs.docker.com/engine/install/linux-postinstall/). Then follow these steps:

```bash
cd ~/ && mkdir ws && cd ws && mkdir src && cd src
git clone https://github.com/mit-acl/rmader.git
```

For Gurobi, you need to download gurobi.lic file from [Gurobi Web License Manager](https://license.gurobi.com/manager/licenses) (more info [here](https://www.gurobi.com/web-license-service/)). A gurobi.lic not obtained through WLS will **not** work on docker. Place your gurobi.lic in [docker](https://github.com/mit-acl/rmader/docker) folder and execute these commands:

```bash
cd ./rmader/rmader/docker
docker build -t rmader . #This will probably take several minutes
```
Once built, 
```
docker run --volume=$PWD/gurobi.lic:/opt/gurobi/gurobi.lic:ro -it rmader
```

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

### Running Multiagent Simulations with 10 agents

> **Note**: For a high number of agents, the performance of RMADER improves with the number of CPUs available in your computer. 

```
roscd rmader && cd script && python run_rmader.py
```

(if you want to modify the parameters, you can do so in `rmader.yaml`).

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

> **Approval for release**: This code was approved for release by The Boeing Company in [NEED TO FILL]. 
