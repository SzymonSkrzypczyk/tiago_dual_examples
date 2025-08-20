# TIAGO DUAL EXAMPLES

## Purpose of the repo
This repository contains 6 scripts that allow controlling tiago++ robot's movements, on top of that another markdown file is included to provide solutions for problems with tiago eye-hand calibration that I've come across.

## How to use it?
To setup the repository for ROS Noetitc use run the following commands:
```bash
git pull git@github.com:SzymonSkrzypczyk/tiago_dual_examples.git
cd tiago_dual_examples
catkin build
source devel/setup.bash
```
> These commands will download and set up the environment for usage

## How to run the scripts?
To run the scripts write this command **after** sourcing the environment:
```bash
roslaunch tiago_dual_examples <selected_script.py>
```
