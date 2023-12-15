FROM althack/ros2:humble-cuda-gazebo-nvidia

ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV COLCON_PREFIX_PATH=/opt/ros/humble
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib
ENV PATH=/opt/ros/humble/bin:$PATH
ENV PYTHONPATH=/opt/ros/humble/lib/python3.8/site-packages
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
  ros-humble-ament-package \
  ros-humble-desktop-full \
  # Install ros distro testing packages
  ros-humble-ament-lint \
  ros-humble-launch-testing \
  ros-humble-launch-testing-ament-cmake \
  ros-humble-launch-testing-ros \
  python3-autopep8 \
  && rm -rf /var/lib/apt/lists/* \
  && rosdep init || echo "rosdep already initialized"

RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 3FA7E0328081BFF6A14DA29AA6A19B38D3D831EF \
    && echo "deb https://download.mono-project.com/repo/ubuntu stable-focal main" | tee /etc/apt/sources.list.d/mono-official-stable.list \
    && apt-get update \
    && apt-get -y install --no-install-recommends \
    openssh-client \
    gpg \
    openssh-client \
    mplayer \
    git-core \
    libu2f-udev \
    apt-transport-https \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 3FA7E0328081BFF6A14DA29AA6A19B38D3D831EF \
    && echo "deb https://download.mono-project.com/repo/ubuntu stable-focal main" | tee /etc/apt/sources.list.d/mono-official-stable.list \
    && apt-get update \
    && apt-get -y install --no-install-recommends \
    xdg-utils \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

## Install cyclone DDS
RUN apt-get update \ 
    && apt-get -y install ros-humble-rmw-cyclonedds-cpp \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ARG USERNAME=ros
ARG WORKSPACE=/home/${USERNAME}

ENV HOME=/home/${USERNAME}
ENV WS=${WORKSPACE}

ARG CMAKE_BUILD_TYPE=RelWithDebInfo

ENV DEBIAN_FRONTEND=dialog

RUN 

# Create source folder
WORKDIR $WS
RUN mkdir ws

# Install ros dependencies
COPY --chown=$USERNAME ../ ws

RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths ws --ignore-src -y && \
    rm -rf /var/lib/apt/lists/*

USER $USERNAME
RUN sudo chown -R ${USERNAME}:${USERNAME} /home/${USERNAME} 
RUN . /opt/ros/humble/setup.sh && \
    cd /home/${USERNAME}/ws && \
    colcon build --merge-install --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE

RUN echo "if [ -f /home/ros/install/setup.bash ]; then source /home/ros/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc
RUN echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /home/${USERNAME}/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/ros/.bashrc

RUN sudo adduser ${USERNAME} dialout