# syntax=docker/dockerfile:1.4
FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=ja_JP.UTF-8 \
    LC_ALL=ja_JP.UTF-8 \
    TZ=Asia/Tokyo \
    ROS_DISTRO=humble

ENV USER=${USERNAME} \
    HOME=/home/${USERNAME} \
    ROS_WS=/home/${USERNAME}/navigation_ws

ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=compute,graphics,utility,display

RUN apt-get update && apt-get install -y --no-install-recommends \
      locales \
      sudo \
      bash-completion \
      build-essential \
      cmake \
      git \
      curl \
      wget \
      vim \
      python3-pip \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-vcstool \
      ros-humble-rviz2 \
      ros-humble-raspimouse-msgs \
      libyaml-cpp-dev \
      libboost-filesystem-dev \
      libboost-thread-dev \
      libeigen3-dev \
      libhdf5-dev \
      libgl1-mesa-glx \
      libglvnd0 \
      mesa-utils \
      xauth \
    && locale-gen ja_JP ja_JP.UTF-8 \
    && update-locale LANG=ja_JP.UTF-8 \
    && ln -snf /usr/share/zoneinfo/${TZ} /etc/localtime \
    && echo ${TZ} > /etc/timezone \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep update

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -d ${HOME} -s /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN mkdir -p ${ROS_WS}/src \
    && chown -R ${USER_UID}:${USER_GID} ${HOME}

RUN git config --system --add safe.directory ${ROS_WS}

USER ${USERNAME}
WORKDIR ${ROS_WS}

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ${HOME}/.bashrc \
    && echo "if [ -f ${ROS_WS}/install/setup.bash ]; then source ${ROS_WS}/install/setup.bash; fi" >> ${HOME}/.bashrc \
    && echo "export ROS_WS=${ROS_WS}" >> ${HOME}/.bashrc \
    && echo "export QT_X11_NO_MITSHM=1" >> ${HOME}/.bashrc

CMD ["/bin/bash"]
