# Tianracer-Mapping

Tianracer-Mapping是基于tianracer比赛项目实现的建图功能。


## Quick Start

该项目基于Ubuntu 20.04(ROS Noetic)。

首先，将要建图的world文件放到world文件夹中

```
cp your_world.world /tianracer/src/tianracer_gazebo/worlds
```

其次，在一个终端中启动仿真环境：

```
roslaunch tianracer_gazebo tianracer_bringup.launch world:=your_world_name
```

在另一个终端启动建图程序：

```
roslaunch tianracer_slam tianracer_gmapping.launch
```

在另一个终端启动控制程序：

```
roslaunch tianracer_gazebo demo_slam_teb.launch
```

建图完成后，在另一个终端保存地图，默认会保存在tianracer_gazebo/maps路径下：

```
roslaunch tianracer_slam gazebo_map_save.launch map_file:=my_map
```

建好图后，一定是不太完整的，可以到以下网站进行修改，导出png格式的地图

```
https://www.photopea.com/
```

这样导出的png格式地图会有问题，需要将其转换为pgm格式。

```
sudo apt install imagemagick
magick my_map.png -colorspace Gray -depth 8 map_fixed.pgm
```

