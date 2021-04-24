# Semantic Segmentation for ROS in PyTorch
### Based on *Sematic Segmentation on MIT ADE20K dataset in PyTorch* by CSAIL-Vision

This is a ROS implementation for Semantic Segmentation, build upon ***semantic-segmentation-pytorch*** by **MIT CSAIL Computer Vision Lab**.

The implementation is in Python3 over PyTorch GPU, and holds training and evaluation parts of the parent project. "README_CSAIL.md" should be refered for more info over training.

## Environment
The code is tested under the following configuration.
- Ubuntu 18.04 LTS
- **CUDA=10.2**
- **PyTorch>=1.5.1**
- **ROS Melodic**


## 1. Installation

### A. Prerequisities

- Install ROS by following the official [ROS website](https://www.ros.org/install/).

- Install dependencies:
```bash
sudo apt-get install python-catkin-tools git
# install Python3 dependencies
pip3 install rospkg
pip3 install numpy scipy
pip3 install pytorch>=0.4.1 torchvision
pip3 install yacs tqdm
```

### B. Setup

Using [catkin](http://wiki.ros.org/catkin):

```bash
# Setup catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin init

# Add workspace to bashrc.
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc

# Clone repo
cd ~/catkin_ws/src
git clone https://github.com/pranay731/ros-semantic-segmentation-pytorch.git

sudo chmod +x ros-semantic-segmentation-pytorch/scripts/run_semantic_segmentation

# Build 
cd ..
catkin build

# Refresh workspace
source ~/catkin_ws/devel/setup.bash
```
A number of models are available. Default is **resnet50dilated** for encoder and **ppm_deepsup** for decoder.
Download pretrained checkpoints for models from [CSAIL Website](http://sceneparsing.csail.mit.edu/model/pytorch) and copy them to ***src/ros-semantic-segmentation-pytorch/ckpt/{modelname}/***


## 2. Usage

### A. Inference

Launch file :
- Input topic -> raw image rgb topic.
- gpu_id -> set gpu id for multiple gpu system else 0.
- cfg_filepath -> path to config file. Should be according to model.
- model_ckpt_dir -> path to directory containing downloaded checkpoints.

Configuration file contains option "imgSizes" which takes a tuple of heights, over which input image is resized for inference. This can be tweaked according to GPU capablities.

```bash
roslaunch semantic_segmentation_ros semantic_segmentation.launch
```

### B. Train and Eval

Follow "README_CSAIL.md" for Training and Evaluation. Train and Eval scripts are present in ***src/ros-semantic-segmentation-pytorch/scripts*** directory.