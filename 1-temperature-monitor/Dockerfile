# Use official ROS 2 Humble base image (already Ubuntu 22.04 + ROS 2 preinstalled)
FROM ros:humble

# Set noninteractive mode for apt to avoid user prompts
ARG ws_name
ENV WS_NAME=${ws_name}
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Install required apt packages
RUN apt update -y && apt upgrade -y

RUN apt-get update && apt-get install -y \
    bash-completion \
    git \
    git-lfs \
    iputils-ping \
    nano \
    openssh-client \
    python3-colcon-common-extensions \
    python3-pip \
    python3-vcstool \
    sudo \
    tmux \
    trash-cli \
    tree \
    vim \
    wget
RUN python_version=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')" ) \
    && echo "Python version is $python_version" \
    && externally_managed_file_path="/usr/lib/python${python_version}/EXTERNALLY-MANAGED" \
    && if [ -f "$externally_managed_file_path" ]; then mv "$externally_managed_file_path" "$externally_managed_file_path.old"; fi
RUN pip3 install -U -I pre-commit
RUN rm -rf /var/lib/apt/lists/*

WORKDIR /root/${WS_NAME}

# Source ROS 2 setup in bash shells
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /root/${WS_NAME}/install/setup.bash" >> /root/.bashrc && \
    export RCUTILS_COLORIZED_OUTPUT=1

CMD ["/bin/bash"]
