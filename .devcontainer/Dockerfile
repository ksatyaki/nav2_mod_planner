FROM osrf/ros:humble-desktop-full

ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib
ENV PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH
ENV PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.8/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2

ENV DEBIAN_FRONTEND=dialog

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV QT_X11_NO_MITSHM=1

RUN apt-get update && apt-get install -y \
  # opengl stuff
  libgl1-mesa-glx \
  libgl1-mesa-dri \
  x11-apps \
  mesa-utils \
  libglvnd0 \
  libgl1 \
  libglx0 \
  libegl1 \
  libxext6 \
  libx11-6 \
  # other stuff
  bash-completion \
  build-essential \
  cmake \
  gdb \
  git \
  tig \
  # pylint3 \
  python3-argcomplete \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-vcstool \
  vim \
  ranger \
  wget \
  gdbserver \
  htop \
  xterm \
  unzip \
  ros-${ROS_DISTRO}-ament-package \
  ros-${ROS_DISTRO}-desktop-full \
  # Install ros distro testing packages
  ros-${ROS_DISTRO}-ament-lint \
  ros-${ROS_DISTRO}-launch-testing \
  ros-${ROS_DISTRO}-launch-testing-ament-cmake \
  ros-${ROS_DISTRO}-launch-testing-ros \
  xdg-utils \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp 

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ARG USERNAME=ros
ARG WORKSPACE=/home/${USERNAME}

ENV HOME=/home/${USERNAME}
ENV WS=${WORKSPACE}

ARG CMAKE_BUILD_TYPE=RelWithDebInfo

ENV DEBIAN_FRONTEND=dialog

RUN addgroup --gid $USER_GID ${USERNAME} && \
useradd -s /bin/bash -m -c "AppleBot ROS User" ${USERNAME} -p "" -u ${USER_UID} -g ${USER_GID} -G sudo && \
    echo '${USERNAME} ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers && \
    chmod 0440 /etc/sudoers && \
    chmod g+w /etc/passwd

RUN adduser ros dialout
RUN groupadd render
RUN usermod -a -G video ros
RUN usermod -a -G render ros

# Create source folder
WORKDIR $WS
RUN mkdir ws

# Install ros dependencies
COPY --chown=$USERNAME ../ ws

RUN apt-get update
RUN apt-get dist-upgrade -y
RUN rosdep update
RUN rosdep install --from-paths ws --ignore-src -y
RUN rm -rf /var/lib/apt/lists/*

USER $USERNAME
RUN bash -c "$(curl -fsSL https://raw.githubusercontent.com/ohmybash/oh-my-bash/master/tools/install.sh)"
RUN sed -i 's/font/pure/g' ${HOME}/.bashrc

RUN sudo chown -R ${USERNAME}:${USERNAME} /home/${USERNAME} 
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd /home/${USERNAME}/ws && \
    colcon build --merge-install --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE

RUN echo "if [ -f /home/ros/install/setup.bash ]; then source /home/ros/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc
RUN echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /home/${USERNAME}/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/ros/.bashrc
RUN echo "ulimit -n 1024" >> /home/ros/.bashrc