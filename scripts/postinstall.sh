#!/bin/bash
# DEMO=~/rmf_demos_ws
mkdir -p rmf_demos_ws/src
cd rmf_demos_ws
wget https://raw.githubusercontent.com/osrf/rmf_demos/master/rmf_demos.repos
vcs import src < rmf_demos.repos

cd src/rmf
git clone git@github.com:osrf/soss.git
git clone git@github.com:Yadunund/demo_soss_ros2.git

# Back to home
cd ../../../
