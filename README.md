# wrs-plugin
A Choreonoid plugin for World Robot Summit

## 1. REQUIREMENTS

### Platforms
- Ubuntu 22.04 LTS (Humble Hawksbill)
- Ubuntu 24.04 LTS (Jazzy Jalisco)

## 2. HOW TO BUILD WRS SIMULATION ENVIRONMENT
### Step1: Install ROS2 (Humble Hawksbill)
```bash
# Add the ROS 2 apt repository
$ sudo apt install software-properties-common
$ sudo add-apt-repository universe
$ sudo apt update && sudo apt install curl -y
$ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
$ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
$ sudo apt update
$ sudo apt upgrade
$ sudo apt install ros-humble-desktop
$ sudo apt install ros-humble-compressed-image-transport
$ sudo apt install python3-colcon-common-extensions

# Sourcing the setup script (for bash)
$ echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

### Step2: Make ``ROBOT PACKAGE``
- Template robot package: https://github.com/wrs-sim/wrs-robot-template
- Template drone package: https://github.com/wrs-sim/wrs-drone-template

### Step3: Build Choreonoid
```bash
# Clean-Build Choreonoid With ROS2
$ mkdir -p ~/ws/src
$ cd ~/ros2_ws/src
$ git clone https://github.com/choreonoid/choreonoid.git
$ git clone https://github.com/choreonoid/choreonoid_ros.git
$ git clone --recursive https://github.com/wrs-sim/wrs-plugin choreonoid/ext/wrs-plugin
$ git clone https://github.com/k38-suzuki/choreonoid_joy2.git
```

#### AGX Dynamics installation reference
- https://choreonoid.org/en/documents/latest/agxdynamics/install/install-agx-ubuntu.html
- Movie for installation (Click on the image to view simulation on youtube)

[![](https://img.youtube.com/vi/SxmwYl_gPEY/0.jpg)](https://youtu.be/SxmwYl_gPEY) 

#### HAIROWorldPlugin installation reference
Please obtain this software before building Choreonoid's WRS environment specification.
Competitors are requested to contact the competition secretariat.
Others should contact JAEA Suzuki at "suzuki.kenta38[a]jaea.go.jp".
(Change the [a] to @ when you send us an email.)

**Move ``hairo-world-plugin`` & ``ROBOT PACKAGE`` to ``choreonoid/ext/``.**

```bash
$ choreonoid/misc/script/install-requisites-ubuntu-22.04.sh
$ sudo ./choreonoid/ext/hairo-world-plugin/misc/script/install-requisites-ubuntu-22.04.sh
$ cd ~/ros2_ws
$ colcon build --symlink-install --cmake-args -DBUILD_AGX_DYNAMICS_PLUGIN=ON -DBUILD_AGX_BODYEXTENSION_PLUGIN=ON -DBUILD_SCENE_EFFECTS_PLUGIN=ON -DBUILD_HAIRO_WORLD_PLUGIN=ON -DENABLE_INSTALL_RPATH_USE_LINK_PATH=ON
```

**By building Choreonoid, your model and project files will be cloned to ``install`` directory, and your controller files will be compiled. When you want to update your model, project and controller files, you should build Choreonoid again.**

### Step4: Edit Material file
When you want to use your own UGV, edit ``share/default/materials.yaml``.

Replace "YourRobot" on line 212 with the body name of your own UGV, and replace "CHASSIS" on line 213 with the root link of your own UGV.
```
    reference_body: YourRobot
    reference_link: CHASSIS
```

## 3. HOW TO LAUNCH CHOREONOID
```bash
$ cd ~
$ cd ~/ros2_ws
$ source install/setup.bash
$ ros2 run choreonoid_ros choreonoid ~/ros2_ws/src/choreonoid/ext/wrs-plugin/registration/registration_wrs2020.yaml --wrs-util ts1
```

| YAML file | Arguments | Details |
| ---- | ---- | ---- |
| registration_wrs2020.yaml | ``ts1`` | Load WRS2020 TS1 environment |
| registration_wrs2020.yaml | ``ts2`` | Load WRS2020 TS2 environment |
| registration_wrs2020.yaml | ``ts3`` | Load WRS2020 TS3 environment |
| registration_wrs2020.yaml | ``ts4`` | Load WRS2020 TS4 environment |
| registration_wrs2025.yaml | ``testrun`` | Load WRS2025 test run environment |
| registration_wrs2025.yaml | ``ps1`` | Load WRS2025 PS-1 environment |
| registration_wrs2025.yaml | ``ps2`` | Load WRS2025 PS-2 environment |
| registration_wrs2025.yaml | ``ps3`` | Load WRS2025 PS-3 environment |
| registration_wrs2025.yaml | ``ps4`` | Load WRS2025 PS-4 environment |
| registration_wrs2025.yaml | ``ps12`` | Load WRS2025 PS-12 environment |
| registration_wrs2025.yaml | ``ps34`` | Load WRS2025 PS-34 environment |
