FROM ubuntu:jammy-20260210.1
# 避免交互提示
ENV DEBIAN_FRONTEND=noninteractive

# ---------------------------------------------------------
# 1. 换源与基础环境 (解决证书与基础包缺失问题)
# ---------------------------------------------------------
RUN sed -i 's/[a-z\.]*archive.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list && \
    sed -i 's/security.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list && \
    # 关键：先用 http 绕过证书检查
    sed -i 's/https/http/g' /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends ca-certificates curl gnupg2 lsb-release locales && \
    # 装好证书后再切回 https
    sed -i 's/http/https/g' /etc/apt/sources.list && \
    # 设置中文环境
    locale-gen zh_CN zh_CN.UTF-8 && \
    update-locale LANG=zh_CN.UTF-8 LC_ALL=zh_CN.UTF-8

ENV LANG=zh_CN.UTF-8
ENV LC_ALL=zh_CN.UTF-8

# ---------------------------------------------------------
# 2. 安装基础工具
# ---------------------------------------------------------
RUN apt-get install -y --no-install-recommends \
    sudo software-properties-common tmux wget build-essential git && \
    add-apt-repository universe -y

# ---------------------------------------------------------
# 3. ROS2 Humble 源 (使用国内中继)
# ---------------------------------------------------------
# 注意：raw.githubusercontent.com 经常无法访问，这里直接从中科院或清华的镜像站获取 Key
RUN curl -sSL https://mirrors.tuna.tsinghua.edu.cn/rosdistro/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg || \
    curl -sSL https://gitee.com/ohhuo/rosdistro/raw/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list

# ---------------------------------------------------------
# 4. Gazebo Garden 源 (使用 OSRF 稳定版)
# ---------------------------------------------------------
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/gazebo-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list

# ---------------------------------------------------------
# 5. 安装 ROS2 & Gazebo & Bridge (合并安装减少重试次数)
# ---------------------------------------------------------
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    ros-dev-tools \
    gz-garden \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-image && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------
# 6. 环境配置
# ---------------------------------------------------------
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# GUI 支持
ENV QT_X11_NO_MITSHM=1
ENV XDG_RUNTIME_DIR=/tmp/runtime-root

WORKDIR /workspace
