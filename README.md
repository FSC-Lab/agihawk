# Agihawk

This pacakge provides interface of Agilicious to Pixhawk4 SITL simulator.

# How to Install

We test the following installation steps in Ubuntu18.04 with ROS Melodic.

## I. Build and Run the Simulator

In the following section we guide you through installing and running a Gazebo simulation.

1. Clone the PX4 Firmware and all its submodules (it may take some time).

   ```bash
   cd ~
   git clone https://github.com/PX4/Firmware.git --recursive
   cd ~/Firmware
   ```

1. Install [PX4 dependencies](http://dev.px4.io/en/setup/dev_env_linux_ubuntu.html#common-dependencies). 
   ```bash
   # Install PX4 "common" dependencies.
   ./Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx
   
   # Gstreamer plugins (for Gazebo camera)
   sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly libgstreamer-plugins-base1.0-dev

1. Build the Firmware once in order to generate SDF model files for Gazebo.
   This step will actually run a simulation (that you can immediately close).

   ```bash
   # This is necessary to prevent some Qt-related errors (feel free to try to omit it)
   export QT_X11_NO_MITSHM=1

   # Build and run simulation
   make px4_sitl_default gazebo
   
   # Quit the simulation (Ctrl+C)

   # Setup some more Gazebo-related environment variables (modify this line based on the location of the Firmware folder on your machine)
   . ~/Firmware/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/Firmware ~/Firmware/build/px4_sitl_default
   ```

1. Finally, set the ROS_PACKAGE_PATH and GAZEBO_MODEL_PATH in your bashrc. Ensure your ~/.bashrc is setup correctly so that each new terminal is sourced correctly:  
    ```bash
    sudo gedit ~/.bashrc
    ```
    Ensure that at the bottom of that document you have exactly this:
    ```bash
    source /opt/ros/melodic/setup.bash
    px4_dir=~/Firmware
    source $px4_dir/Tools/setup_gazebo.bash $px4_dir $px4_dir/build/px4_sitl_default
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$px4_dir
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$px4_dir/Tools/sitl_gazebo
    ```

## II. Install Agilicious

Follow the guide in `https://github.com/uzh-rpg/agilicious` to install the Agilicious. 

## III. Install Agihawk

1. Clone the Agihawk repository to the same work space as Agilicious. Assume that the source folder is in path `~/agi_ws/src`.
    ```
    cd ~/agi_ws/src
    git clone https://github.com/FSC-Lab/agihawk
    catkin build 
    ```
1. Go to the ~/.bashrc setup and add the following line **before** the px4 firmware setups. The content should look like:
    ```bash
    source /opt/ros/melodic/setup.bash

    source ~/agi_ws/devel/setup.bash

    px4_dir=~/Firmware
    source $px4_dir/Tools/setup_gazebo.bash $px4_dir $px4_dir/build/px4_sitl_default
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$px4_dir
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$px4_dir/Tools/sitl_gazebo
    ```

# Run Agihawk With the Gazebo Simulator
Open a new terminal to automatically source the bashrc file. Use launch file to put everything together
```
roslaunch agihawk run_px4_simulation.launch 
```
