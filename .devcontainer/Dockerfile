FROM althack/ros2:foxy-dev

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
&& sudo apt-get upgrade -y\ 
&& apt-get -y install --no-install-recommends \
cython3 \
git \
micro \
python3-dev \
python3-pip \
ros-${ROS_DISTRO}-rqt \
ros-${ROS_DISTRO}-rqt-tf-tree \
ros-${ROS_DISTRO}-rviz2 \
# Clean up
&& apt-get autoremove -y \
&& apt-get clean -y \
&& rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

RUN sudo pip3 install -U pip

RUN git clone -b main https://github.com/f1tenth/f1tenth_gym.git /opt/f1tenth_gym \
&& cd /opt/f1tenth_gym \
&& sudo pip3 install -e .

RUN git clone -b foxy-devel https://github.com/f1tenth/range_libc.git /opt/range_libc \
&& cd /opt/range_libc/pywrapper \
&& ./compile.sh
