FROM ros:humble-ros-base

COPY . .

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Update apt
RUN apt-get update -y

# Install basic packages
RUN apt-get install -y build-essential
RUN apt-get install -y apt-utils
RUN apt-get install -y nautilus
RUN apt-get install -y nano
RUN apt-get install -y python-pip
RUN apt-get install -y curl
RUN apt-get install -y git
RUN apt-get install -y wget
RUN apt-get install -y gpg
RUN apt-get install -y gdb
RUN apt-get install -y ca-certificates

# Install external packages with apt
RUN apt-get install -y libgeotiff-dev libboost-all-dev libeigen3-dev libpcap-dev
RUN apt-get install -y ros-humble-point-cloud-msg-wrapper ros-humble-tf2-ros ros-humble-tf2

WORKDIR /root

RUN bash -c "mkdir -p mapora_ws/src && cd /root/mapora_ws/src && git clone https://github.com/leo-drive/mapora.git"

WORKDIR /root/mapora_ws/src/mapora

# Build thirdparty packages
RUN ./scripts/build.sh

WORKDIR /root/mapora_ws

RUN bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1"

WORKDIR /root/mapora_ws

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
