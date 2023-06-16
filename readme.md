# DepthClustering #

Our work is based on the original work from DepthClustering:
```
@InProceedings{bogoslavskyi16iros,
title     = {Fast Range Image-Based Segmentation of Sparse 3D Laser Scans for Online Operation},
author    = {I. Bogoslavskyi and C. Stachniss},
booktitle = {Proc. of The International Conference on Intelligent Robots and Systems (IROS)},
year      = {2016},
url       = {http://www.ipb.uni-bonn.de/pdfs/bogoslavskyi16iros.pdf}
}
```

```
@Article{bogoslavskyi17pfg,
title   = {Efficient Online Segmentation for Sparse 3D Laser Scans},
author  = {I. Bogoslavskyi and C. Stachniss},
journal = {PFG -- Journal of Photogrammetry, Remote Sensing and Geoinformation Science},
year    = {2017},
pages   = {1--12},
url     = {https://link.springer.com/article/10.1007%2Fs41064-016-0003-y},
}
```


The sudden occurred objects can be successfully detected:
![](doc/pics/unlabelled_obstacles_2.gif)
while the detection results rely on the image segmentation performance while the models we use are not included in this repository. 

## Prerequisites ##
I recommend using a virtual environment in your catkin workspace (`<catkin_ws>`
in this readme) and will assume that you have it set up throughout this readme.
Please update your commands accordingly if needed. I will be using `pipenv`
that you can install with `pip`.

### Set up workspace and catkin ###
Regardless of your system you will need to do the following steps:
```bash
cd <catkin_ws>            # navigate to the workspace
pipenv shell --fancy      # start a virtual environment
pip install catkin-tools  # install catkin-tools for building
mkdir src                 # create src dir if you don't have it already
# Now you just need to clone the repo:
git clone https://github.com/yiyihan/depthcluster_ground
```

### System requirements ###
You will need OpenCV, QGLViewer, FreeGLUT, QT5 and optionally PCL and/or
ROS. The following sections contain an installation command for various Ubuntu
systems (click folds to expand):

<details>
<summary>Ubuntu 20.04</summary>

  #### Install these packages:

```bash
sudo apt install libopencv-dev libqglviewer-dev-qt5 freeglut3-dev qtbase5-dev 
```

You might also need the latest GoogleTest binary installed on your systems. As Ubuntu is not shipped with these binaries by default, you would have to install them yourself or adapt the build script to build them from source.

</details>

### Optional requirements ###
If you want to use PCL clouds and/or use ROS for data acquisition you can install the following: 
- (optional) PCL - needed for saving clouds to disk
- (optional) ROS - needed for subscribing to topics

## How to build? ##
This is a catkin package. So we assume that the code is in a catkin workspace
and CMake knows about the existence of Catkin. It should be already taken care
of if you followed the instructions [here](#set-up-workspace-and-catkin). Then
you can build it from the project folder:

```bash
mkdir build
cd build
cmake ..
make -j4
ctest -VV  # run unit tests, optional
```

It can also be built with `catkin_tools` if the code is inside catkin
workspace:

```bash
catkin build depth_clustering
```

P.S. in case you don't use `catkin build` you [should][catkin_tools_docs] reconsider your decision.

## How to run? ##
See [examples](examples/). There are ROS nodes as well as standalone
binaries. Examples include showing axis oriented bounding boxes around found
objects (these start with `show_objects_` prefix) as well as a node to save all
segments to disk. The examples should be easy to tweak for your needs.

## Run on real world data ##
Go to folder with binaries:
```
cd <path_to_project>/build/devel/lib/depth_clustering
```

Alternatively, you can run the data from Qt GUI (as in video):
```
./qt_gui_app
```
Once the GUI is shown, click on <kbd>OpenFolder</kbd> button and choose the
folder where you have unpacked the `png` files, e.g. `data/scenario1/`.
Navigate the viewer with arrows and controls seen on screen.

#### Other data ####
There are also examples on how to run the processing on KITTI data and on ROS
input. Follow the `--help` output of each of the examples for more details.

Also you can load the data from the GUI. Make sure you are loading files with
correct extension (i.e., `*.txt` and `*.bin` for KITTI).
