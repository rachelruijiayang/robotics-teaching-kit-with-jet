#This script is for installing software on a new Jetson.  This script
#assumes you have already flashed the latest version of the kernel
#using Jetpack

import os,commands,sys

noupdate=0

#----------------Testing packages----------------------------
INSTALL_DIR = '~/nvidia_robot'
WORKSPACE = '~/workspace'

#These are test commands if Jetpack did not install CUDA or OpenCV4Tegra
CUDA_DOWNLOAD_PATH = 'http://developer.download.nvidia.com/embedded/L4T/r21_Release_v3.0/'
CUDA_TOOLKIT = 'cuda-repo-ubuntu1404-6-5-prod_6.5-42_amd64.deb'

OPENCV4TEGRA_DOWNLOAD_PATH = 'http://developer.download.nvidia.com/embedded/OpenCV/L4T_21.1/'
OPENCV4TEGRA = 'libopencv4tegra-repo_ubuntu1404_2.4.10.1_amd64.deb'

#os.system('cd dist; wget -nc ' + CUDA_DOWNLOAD_PATH + CUDA_TOOLKIT)
#os.system('cd dist; wget -nc ' + OPENCV4TEGRA_DOWNLOAD_PATH + OPENCV4TEGRA)

#----------------Grinch Kernel----------------------------
def update_kernel():
    print('-----Checking for grinch kernel-----')
    cmd = 'uname -r'
    (exitstatus, outtext) = commands.getstatusoutput(cmd)
    if 'gdacac' in outtext:
        os.system('mkdir dist')
        print('\t-----Installing grinch kernel with additional drivers-----')
        #running NVIDIA kernel
        os.system('cd dist; git clone ' + 'https://github.com/jetsonhacks/installGrinch.git')
        os.system('cd dist/installGrinch; ./installGrinch.sh')
        os.system('rm -f -r dist')
        quit()
    else:
        print('\t-----Grinch kernel detected-----')
        os.system('cd dist; rm -f -r installGrinch')
        os.system('rm -f -r dist')

#----------------ROS Installation----------------------------
def install_ros():
    global noupdate

    #set locale
    print('-----Setting locale-----')
    os.system('sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX')

    #set sources.list
    print('-----Setting sources.list-----')
    os.system('sudo sh -c \'echo \"deb http://packages.ros.org/ros/ubuntu trusty main\" > /etc/apt/sources.list.d/ros-latest.list\'')

    #set up keys
    print('-----Setup keys-----')
    os.system('wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -')

    if not noupdate:
        print('-----Updating package index-----')
        os.system('sudo apt-get update')

    print('-----Installing ROS base-----')
    os.system('sudo apt-get -y install ros-jade-ros-base')

    #initialize rosdep
    print('-----Installing rosdep-----')
    os.system('sudo apt-get -y install python-rosdep')
    os.system('sudo rosdep init')

    if not noupdate:
        print('-----Running rosdep update-----')
        os.system('rosdep update')

    #environment setup
    print('-----Setup environment-----')
    os.system('echo \"source /opt/ros/jade/setup.bash\" >> ~/.bashrc')
    os.system('source ~/.bashrc')

    #add rosinstall
    print('-----Setup rosinstall-----')
    os.system('sudo apt-get -y install python-rosinstall')

    #arduino-rosserial
    print('-----Setup rosserial-arduino-----')
    os.system('sudo apt-get -y install ros-jade-rosserial-arduino')
    os.system('sudo apt-get -y install ros-jade-rosserial')
    os.system('sudo apt-get -y install ros-jade-eigen-conversions')
    os.system('sudo apt-get -y install ros-jade-tf2-geometry-msgs')
    os.system('sudo apt-get -y install ros-jade-angles')
    os.system('sudo apt-get -y install arduino')

    #web_video_server
    print('-----Setup web-video-server-----')
    os.system('sudo apt-get -y install ros-jade-web-video-server')

    #rosbridge_suite
    print('-----Setup rosbridge_suite-----')
    os.system('sudo apt-get -y install ros-jade-rosbridge-suite')
    os.system('sudo apt-get -y install ros-jade-rospy-tutorials')

    #rosjoy
    print('-----Setup rosjoy-----')
    os.system('sudo apt-get -y install ros-jade-joy')

    #cv_camera
    print('-----Setup cv_camera-----')
    os.system('sudo apt-get -y install ros-jade-cv-camera')

    #teleop_twist_joy
    print('-----Setup teleop_twist_joy-----')
    os.system('sudo apt-get -y install ros-jade-teleop-twist-joy')

    #roslint
    print('-----Setup roslint-----')
    os.system('sudo apt-get -y install ros-jade-roslint')

    #python-pip
    print('-----Setup pip-----')
    os.system('sudo apt-get -y install python-pip')

    #python flask
    print('-----Setup flask-----')
    os.system('sudo pip install flask')

    #------------------Packages for video streaming----------------------

    #testing packages for cv-bridge
    os.system('sudo apt-get -y install ros-jade-cv-bridge')
    os.system('sudo apt-get -y install ros-jade-image-transport')
    #os.system('cd ../rosjetson/src; git clone https://github.com/ros-perception/vision_opencv.git')
    #os.system('cd ../rosjetson/src; git clone https://github.com/ros-perception/image_pipeline.git')
    #os.system('cd ../rosjetson/src; git clone https://github.com/ros-perception/image_common.git')
    #os.system('sudo apt-get -y install yaml-cpp0.5')
    #os.system('sudo apt-get -y install libyaml-cpp-dev')
    #os.system('sudo apt-get -y install libgtk2.0-dev')
    #os.system('sudo apt-get -y install libgtk-3-dev')

    #symlinks for OpenCV4Tegra
    #The symlink for libopencv_ocl does not point to the correct library.  It
    #points to libopencv_tegra which is good enough to satisfy the linker.
    os.system('''cd /usr/lib/arm-linux-gnueabihf;
    sudo ln -s ../libopencv_calib3d.so.2.4.12    libopencv_calib3d.so.2.4.8;
    sudo ln -s ../libopencv_contrib.so.2.4.12    libopencv_contrib.so.2.4.8;
    sudo ln -s ../libopencv_core.so.2.4.12       libopencv_core.so.2.4.8;
    sudo ln -s ../libopencv_features2d.so.2.4.12 libopencv_features2d.so.2.4.8;
    sudo ln -s ../libopencv_flann.so.2.4.12      libopencv_flann.so.2.4.8;
    sudo ln -s ../libopencv_gpu.so.2.4.12        libopencv_gpu.so.2.4.8;
    sudo ln -s ../libopencv_highgui.so.2.4.12    libopencv_highgui.so.2.4.8;
    sudo ln -s ../libopencv_imgproc.so.2.4.12    libopencv_imgproc.so.2.4.8;
    sudo ln -s ../libopencv_legacy.so.2.4.12     libopencv_legacy.so.2.4.8;
    sudo ln -s ../libopencv_ml.so.2.4.12         libopencv_ml.so.2.4.8;
    sudo ln -s ../libopencv_objdetect.so.2.4.12  libopencv_objdetect.so.2.4.8;
    sudo ln -s ../libopencv_tegra.so.2.4.12      libopencv_ocl.so.2.4.8;
    sudo ln -s ../libopencv_photo.so.2.4.12      libopencv_photo.so.2.4.8;
    sudo ln -s ../libopencv_stitching.so.2.4.12  libopencv_stitching.so.2.4.8;
    sudo ln -s ../libopencv_superres.so.2.4.12   libopencv_superres.so.2.4.8;
    sudo ln -s ../libopencv_video.so.2.4.12      libopencv_video.so.2.4.8;
    sudo ln -s ../libopencv_videostab.so.2.4.12  libopencv_videostab.so.2.4.8;''')

#----------------Arduino Installation----------------------------
def setup_catkin_workspace():
    global WORKSPACE

    os.system('source /opt/ros/jade/setup.bash')
    os.system('mkdir -p ' + WORKSPACE + '/src; cd ' + WORKSPACE + '/src; catkin_init_workspace')

    os.system('cd ' + WORKSPACE + '/src; git clone https://github.com/NVIDIAGPUTeachingKit/rosjet.git')
    os.system('cd ' + WORKSPACE + '/src; mkdir labs')

#----------------Arduino Installation----------------------------
def install_arduino():
    global WORKSPACE

    #building arduino libraries
    print('-----Building arduino libraries-----')
    os.system('cd ' + WORKSPACE + '/src; git clone https://github.com/ros-drivers/rosserial.git')
    os.system('bash -c \"source /opt/ros/jade/setup.bash; cd ' + WORKSPACE + '; catkin_make\"')
    os.system('bash -c \"source /opt/ros/jade/setup.bash; cd ' + WORKSPACE + '; catkin_make install\"')

    os.system('bash -c \"source ' + WORKSPACE + '/install/setup.bash; cd ' + WORKSPACE + '/arduino/src; rm -f -r ros_lib; rosrun rosserial_arduino make_libraries.py .\"')

#----------------System Optimizations----------------------------
def post_install_config():

    #make the power button shutoff the TK1 cleanly
    os.system('gsettings set org.gnome.settings-daemon.plugins.power button-power shutdown')

    #disable screen lock
    os.system('gsettings set org.gnome.desktop.screensaver lock-enabled false')

    #disable Unity animations
    #TODO - need to figure out how to do this from the command line
    os.system('sudo apt-get -y install compizconfig-settings-manager')
    os.system('gsettings set org.gnome.desktop.interface enable-animations false')

    #turn off Unity online search
    os.system('gsettings set com.canonical.Unity.Lenses remote-content-search none')

    #set timezone
    os.system('timedatectl set-timezone America/Los_Angeles')

    #autologin ubuntu user
    os.system('echo \'[SeatDefaults]\\nautologin-user=ubuntu\' > login_file; sudo mv login_file /etc/lightdm/lightdm.conf')

    #enable remote desktop
    os.system('gsettings set org.gnome.Vino enabled true')
    os.system('gsettings set org.gnome.Vino disable-background true')
    os.system('gsettings set org.gnome.Vino prompt-enabled false')
    os.system('gsettings set org.gnome.Vino require-encryption false')

    #make alias for sourcing setup.bash
    os.system('echo \"alias sr=\'source ~/nvidia_robot/rosjetson/devel/setup.bash\'\" >> ~/.bashrc')

def main():
    global noupdate
    if 'noupdate' in sys.argv:
        noupdate=1

    if 'config' in sys.argv:
        post_install_config()
        quit()

    update_kernel()
    install_ros()
    setup_catkin_workspace()
    install_arduino()
    post_install_config()

main()
