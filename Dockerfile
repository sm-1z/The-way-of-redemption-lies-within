# 使用Ubuntu 20.04镜像
FROM ubuntu:focal

# 避免在docker构建过程中出现时区选择等交互式问题
ENV TZ=Asia/Shanghai \
    DEBIAN_FRONTEND=noninteractive

# 设置时区
ENV TZ=Etc/UTC

# 切换软件源至清华源
RUN sed -i 's/archive.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list && \
    sed -i 's/security.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list

# 更新软件包和安装必要的包
RUN apt-get update && apt-get install -y \
    ca-certificates \
    gnupg2 \
    lsb-release \
    tzdata \
    sudo && \
    echo $TZ > /etc/timezone && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata && \
    useradd -m user -s /bin/bash && \
    echo 'user:ca' | chpasswd && \
    adduser user sudo && \
    echo 'user ALL=(ALL) NOPASSWD: ALL' > /etc/sudoers.d/user

# 设置工作目录为用户家目录
WORKDIR /home/user

# 忽略APT HTTPS源的证书验证错误
RUN echo 'Acquire::https::packages.ros.org::Verify-Peer "false";' | sudo tee /etc/apt/apt.conf.d/99verify-peer.conf

# 切换到新创建的用户
USER user

# 添加ROS GPG密钥
RUN sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654


sudo apt-key adv --keyserver https://keyserver.ubuntu.com --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key F42ED6FBAB17C654


# 添加ROS软件源
RUN sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'

# 更新软件包列表并安装ROS Noetic Desktop Full
RUN sudo apt-get update && \
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y ros-noetic-desktop-full

RUN sudo apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

RUN sudo apt-get install -y python3-rosdep

RUN sudo apt-get install -y python3-pip && sudo pip install rosdepc && sudo rosdepc init && rosdepc update


# 安装Git
RUN sudo DEBIAN_FRONTEND=noninteractive apt-get install -y git bash-completion python3-catkin-tools proxychains wget

# 配置Git全局用户名和电子邮件
RUN git config --global user.name "user" && \
    git config --global user.email "user@user.com"

# 创建工作目录并克隆指定的Git仓库
RUN mkdir -p /home/user/hector_ws/src && mkdir -p /home/user/host

# 启用 Bash 自动补全
RUN echo "if [ -f /etc/bash_completion ]; then . /etc/bash_completion; fi" >> ~/.bashrc

# 设置工作目录为hector_ws
WORKDIR /home/user/hector_ws

# 初始化catkin工作空间
RUN . /opt/ros/noetic/setup.sh && \
    catkin_init_workspace src && \
    cd ..

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && . ~/.bashrc

# 配置 proxychains
RUN sudo sed -i 's/^socks4.*/socks5 127.0.0.1 1089/' /etc/proxychains.conf

# 设置 pip 使用清华源
RUN mkdir -p ~/.pip && \
    sudo echo "[global]" > ~/.pip/pip.conf && \
    sudo echo "index-url = https://pypi.tuna.tsinghua.edu.cn/simple" >> ~/.pip/pip.conf

RUN pip3 install scipy
RUN pip3 install numpy
RUN pip3 install Cython
RUN pip3 install mujoco-py
RUN pip3 install mujoco

RUN sudo apt-get install -y libglfw3-dev

RUN cd /home/user/hector_ws/src && \
    git clone --recursive https://gitee.com/HackVoyager/Hector_Simulation.git && \
    cd Hector_Simulation && git checkout pai_control

# 新建thirdParty目录并克隆MuJoCo和Pinocchio代码
RUN mkdir -p /home/user/hector_ws/thirdParty && \
    cd /home/user/hector_ws/thirdParty && \
    git clone --recursive https://gitee.com/HackVoyager/pinocchio.git && \
    cd pinocchio && mkdir build && cd build && cmake -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_TESTING=OFF .. && \
    make -j20 && sudo make install

# 下载 MuJoCo
RUN cd /home/user/hector_ws/thirdParty && \
    wget https://gitee.com/HackVoyager/mujoco/releases/download/3.1.5/mujoco-3.1.5-linux-x86_64.tar.gz \
    # 解压文件
    && tar -xzf mujoco-3.1.5-linux-x86_64.tar.gz \
    # 删除下载的压缩文件以减少镜像大小
    && mv mujoco-3.1.5 mujoco315

RUN rm /home/user/hector_ws/thirdParty/mujoco-3.1.5-linux-x86_64.tar.gz


RUN cd /home/user/hector_ws/thirdParty && \
    git clone --recursive https://gitee.com/HackVoyager/lcm.git && \
    cd lcm && mkdir build && cd build && cmake .. && \
    make -j20 && sudo make install

# Clone the repository and attempt to build it using catkin
RUN cd /home/user/hector_ws/src && \
    git clone --recursive https://gitee.com/HackVoyager/eigenpy.git

# Use /bin/bash to source the ROS setup script and then build the workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.sh && \
    cd /home/user/hector_ws && \
    catkin build eigenpy"

RUN cd /home/user/hector_ws/src/Hector_Simulation/Hector_ROS_Simulation/pai_control && \
    sed -i 's/mujoco::mujoco/mujoco/g' CMakeLists.txt && \
    sed -i '/find_package(pinocchio REQUIRED)/a include_directories("/home/user/hector_ws/thirdParty/mujoco315/include")\nlink_directories("/home/user/hector_ws/thirdParty/mujoco315/lib")' CMakeLists.txt

RUN /bin/bash -c "source /opt/ros/noetic/setup.sh && \
    catkin config --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && \
    cd /home/user/hector_ws && \
    catkin build"
    
RUN sudo apt install -y clangd-10
RUN /bin/bash -c "source /home/user/hector_ws/devel/setup.sh"

CMD ["bash"]