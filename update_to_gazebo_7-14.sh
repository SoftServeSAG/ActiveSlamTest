#!/usr/bin/env bash
# Setup your computer to accept software from packages.osrfoundation.org.
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

# You can check to see if the file was written correctly. For example, in Ubuntu Trusty, you can type:
cat /etc/apt/sources.list.d/gazebo-stable.list
# deb http://packages.osrfoundation.org/gazebo/ubuntu-stable trusty main

# Setup keys
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

# First update the debian database:
sudo apt-get update
sudo apt-get install gazebo7

# check by 
gazebo -v
