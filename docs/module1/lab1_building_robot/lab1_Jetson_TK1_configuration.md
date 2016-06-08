## Getting Started

This document describes the first steps you will need to go through in order to prepare
the Jetson TK1 board for use with Jet.

After opening the NVIDIA Jetson TK1 box, you will need the following items:

* Jetson TK1 board
* micro USB cable
* AC power adapter

In addition, you will need:

* an ethernet cable
* a router that is connected to the Internet

## Preparing the TK1

* Plug one end of the ethernet cable in Jetson TK1 and plug the other end into the router
* Do not plug in power adapter just yet (you will plug in power during the OS flash step)

## Running JetPack

Download and install the [NVIDIA Jetpack](https://developer.nvidia.com/embedded/jetpack).
You will need to be running Ubuntu 14.04 on your host computer.

Once Jetpack is up and running perform the following actions:

* Select TK1 as target board
* Click the `Clear Actions` button (upper right corner).  This will de-select all of the options.
* Select the following options for install:
  * `Driver for OS`
  * `File System`
  * `Flash OS`
  * `CUDA Toolkit for L4T`
  * `cuDNN Package`
  * `OpenCV for Tegra`
  * also select the box `Automatically resolve dependency conflicts`
* On the Network Layout page:
  * Select `Device accesses Internet via router/switch`
* On the Network Interface Selection page:
  * select `eth0` (assuming your host computer only has 1 ethernet)

## Flash the OS

After you have started the installation, the JetPack installer will download files and 
install them.  As the installer continues, it will reach a point where it will flash the 
OS to the Jetson TK1.  At this point, the installer will ask you to put the board into
Force USB Recovery Mode:

* Connect the micro USB cable to the Jetson TK1 and the host computer
* Plug the AC power adapter into AC power and connect the power jack to the board
* Enter USB Recovery Mode using the following sequence:
  * Press and release Power button
  * Hold down the Force Recovery button, then press and release Reset (while holding down Force Recovery)
  * You can check if the board is in Recovery mode by connecting to the Jetson board via ssh and running `lsusb`.  There should be a device with vendor 'NVidia Corp'
  * Begin the flashing process on the host by pressing Enter.  This will take several minutes.

## Installing Grinch Kernel

Once the install is complete, you can go ahead and disconnect the micro USB cable.

* Connect to the Jetson via ssh (username: ubuntu, password ubuntu)
  * You can find the IP of the Jetson by checking the active IPs connected to the router
* Install git

  `sudo apt-get install git`

* Download the code

  `git clone https://github.com/jsseng/nvidia_robot.git`

* Run the setup script the first time to install the Grinch kernel.  The Grinch kernel
contains drivers not installed with the stock Nvidia Ubuntu kernel.

  ```cd nvidia_robot/docs```

  ```python setup_jetson.py```

After the Grinch kernel is installed, reboot the Jetson.  You can do this by issuing
the command:

  `sudo reboot`

## Installing Jet Software

Change to the docs directory and run the setup script a second time

```cd nvidia_robot/docs```

```python setup_jetson.py```

This time, the script should detect that the Grinch kernel is installed and will proceed 
with installation of ROS.

After the script is complete, reboot the machine.  This time, Ubuntu will automatically login.

## Optimizing the Desktop Environment

In order to optimize the Ubuntu desktop, the setup script needs to run a third time. 

```cd nvidia_robot/docs```

```python setup_jetson.py```

## Setup Complete

Configuration of the Jetson TK1 is now complete.
