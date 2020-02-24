FROM osrf/ros:melodic-desktop-bionic

COPY . /home/ros/src

RUN apt-get update && apt-get install -y wget

# Set up Dataspeed repository
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 66F84AE1EB71A8AC108087DCAF677210FF6D3CDA
RUN sh -c 'echo "deb [ arch=amd64 ] http://packages.dataspeedinc.com/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-dataspeed-public.list'
RUN apt-get update
RUN sh -c 'echo "yaml http://packages.dataspeedinc.com/ros/ros-public-'$ROS_DISTRO'.yaml '$ROS_DISTRO'" > /etc/ros/rosdep/sources.list.d/30-dataspeed-public-'$ROS_DISTRO'.list'
RUN rosdep update

# Download and install binaries of code for the course
RUN mkdir /bin_download && cd /bin_download && wget http://www.secs.oakland.edu/~mtradovn/ece_6460/avs_melodic_release.tar.gz && wget http://www.secs.oakland.edu/~mtradovn/ece_6460/custom-gazebo_9.6.999-1_amd64.deb && tar -xf avs_melodic_release.tar.gz -C . && dpkg -i --force-all *.deb && apt-get install -f -y

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
