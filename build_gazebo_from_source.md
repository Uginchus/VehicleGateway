# Building Gazebo from Source

In order to make rebuilds faster, we suggest creating two colcon workspaces: one for this project (`vehicle_gateway`) and one for Gazebo Harmonic. The Gazebo workspace will take much longer to compile, but it only needs to be recompiled when changes are incorporated from Gazebo. The overlaid `vehicle_gateway` workspace is where we expect most debug-recompile cycles to occur.

To keep paths short, `vg` stands for "Vehicle Gateway". After these steps, you'll have two workspaces in the `~/vg` directory, like this:

```bash
vg
├── gz_ws
│   ├── build
│   ├── install
│   ├── log
│   └── src
└── vg_ws
    ├── build
    ├── install
    ├── log
    └── src
```

First, install ROS 2 Jazzy using the `.deb` packages using APT [according to these instructions](http://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html)
```bash
.~/pragma/Ros/ros2_ws/protocol/install_ros2_jazzy_deb.sh
```

Next, install a few dependencies and set up our workspace source directories:
```bash
sudo apt install python3-kconfiglib python3-jinja2 python3-jsonschema ros-jazzy-gps-msgs gcc-arm-none-eabi libfuse2 -y
python3 -m venv $HOME/vcs_colcon_installation
. $HOME/vcs_colcon_installation/bin/activate
pip3 install vcstool colcon-common-extensions
pip install pyros-genmsg
mkdir -p ~/vg/vg_ws/src
cd ~/vg/vg_ws/src
git clone https://github.com/Uginchus/VehicleGateway
```

Next, install Gazebo Harmonic. The full instructions are [here](https://gazebosim.org/docs/harmonic/install_ubuntu), and summarized as follows.

```bash
pip3 install vcstool colcon-common-extensions

sudo apt-get update

sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update
mkdir -p ~/vg/gz_ws/src
cd ~/vg/gz_ws/src
curl -O https://raw.githubusercontent.com/gazebo-tooling/gazebodistro/master/collection-harmonic.yaml
vcs import < collection-harmonic.yaml

sudo apt install -y $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ')
```

Now we can compile Gazebo Harmonic:

```bash
cd ~/vg/gz_ws
colcon build --merge-install --symlink-install
source ~/vg/gz_ws/install/setup.bash
echo "source ~/vg/gz_ws/install/setup.bash" >> ~/.bashrc
```
Now you should have Gazebo available in `~/vg/gz_ws`.

We can now build the Vehicle Gateway itself, by overlaying its workspace on top of the Gazebo Harmonic workspace as well as the ROS 2 Jazzy system install. The Vehicle Gateway build will also download and build the Betaflight firmware, to allow software-in-the-loop (SITL) simulation:

```bash
cd ~/vg/vg_ws
rosdep update && rosdep install --from-paths src --ignore-src -y
source ~/vg/gz_ws/install/setup.bash
source /opt/ros/jazzy/setup.bash
```

Install gcc-arm-none-eabi-10.3-2021.10
```bash
sudo apt-get install libasio-dev
sudo apt-get remove gcc-arm-none-eabi -y
wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/10.3-2021.10/gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2
tar -xjf gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2
sudo mv gcc-arm-none-eabi-10.3-2021.10 /opt/
export PATH=/opt/gcc-arm-none-eabi-10.3-2021.10/bin:$PATH
arm-none-eabi-gcc --version
echo 'export PATH=/opt/gcc-arm-none-eabi-10.3-2021.10/bin:$PATH' >> ~/.bashrc
source ~/.bashrc
```

```bash
sudo apt-get update
sudo apt-get install ros-jazzy-control-toolbox -y

colcon build --merge-install --symlink-install

sudo apt-get update
sudo apt-get install socat -y
```

Additional
```
pip install pyserial
```
