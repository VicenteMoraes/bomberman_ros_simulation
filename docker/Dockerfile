FROM osrf/ros:melodic-desktop-full
RUN apt update && apt full-upgrade -y

RUN apt install -y morse-simulator python3-morse-simulator
RUN apt install -y curl gnupg2 lsb-release
RUN apt install -y python3-dev python3-pip python3-yaml python-rospkg
RUN pip3 install rospkg
RUN apt install -y xauth libfreetype6 libglu1-mesa libxi6 libxrender1 xz-utils
RUN apt install -y mesa-utils libegl1-mesa libegl1-mesa-dev libgbm-dev libgbm1 libgl1-mesa-dev libgl1-mesa-dri libglu1-mesa libglu1-mesa-dev
RUN apt install -y libopenexr-dev zlib1g-dev libfontconfig1 libgl1-mesa-glx libxmu-dev
RUN apt update && apt install -y mesa-utils apt-utils xvfb llvm-dev
RUN apt-get -y autoremove \
    && rm -rf /var/lib/apt/lists/*

#NVIDIA
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

RUN apt update
RUN apt install -y ros-melodic-map-server ros-melodic-gmapping ros-melodic-navigation
