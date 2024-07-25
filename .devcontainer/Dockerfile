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
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
  lsb-release \
  tmux \
  gnupg

RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update
RUN apt-get install ignition-fortress ros-${ROS_DISTRO}-turtlebot4-simulator -y


RUN pip install tmule

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG WORKSPACE=/home/${USERNAME}/ws

ENV HOME=/home/${USERNAME}
ENV WS=${WORKSPACE}

ARG CMAKE_BUILD_TYPE=RelWithDebInfo

ENV DEBIAN_FRONTEND=dialog

RUN echo "Creating user ${USERNAME} with UID ${USER_UID} and GID ${USER_GID}"

RUN addgroup --gid $USER_GID ${USERNAME} && \
useradd -s /bin/bash -m -c "AppleBot ROS User" ${USERNAME} -p "" -u ${USER_UID} -g ${USER_GID} -G sudo && \
    echo '${USERNAME} ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers && \
    chmod 0440 /etc/sudoers && \
    chmod g+w /etc/passwd

RUN adduser ros dialout
RUN groupadd render
RUN usermod -a -G video ros
RUN usermod -a -G render ros

# Create source folder and copy contents of this repo and other repos in this workspace. 
WORKDIR $WS
RUN mkdir src
COPY --chown=$USERNAME . $WORKSPACE/src/

USER $USERNAME
RUN sudo chown -R ros:ros /home/ros

RUN sudo apt-get update && \
    rosdep fix-permissions && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --merge-install --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE -DCMAKE_EXPORT_COMPILE_COMMANDS=1

RUN bash -c "$(curl -fsSL https://raw.githubusercontent.com/ohmybash/oh-my-bash/master/tools/install.sh)"
RUN sed -i 's/font/pure/g' ${HOME}/.bashrc

RUN sudo apt-get update
RUN sudo apt-get dist-upgrade -y
RUN sudo rm -rf /var/lib/apt/lists/*


RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
RUN echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /home/${USERNAME}/.bashrc
RUN echo "ulimit -n 1024" >> /home/${USERNAME}/.bashrc