# lecture2025

13.84, -1.05, 0.4 -> 16.1, -1.05, 1.98

## setup

```sh
docker pull (private repo)/m1s_agent_cuda
# ~/ros will be linked to ~/projects/agent, and ~/genesis_ws will be linked to ~/projects/agent/genesis
mkdir -p projects/agent/agent_system_ws
mkdir -p projects/agent/genesis

docker run --rm -ti -v ./projects:/home/kou/projects -v /mnt/hoge:/mnt/hoge -e=DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v "${XAUTHORITY:-$HOME/.Xauthority}:$IMG_HOME/.Xauthority:rw" -m 8192m --device /dev/dri --gpus all -p 6006:6006 (private repo)/m1s_agent_cuda tmux

# No.5
cd ~/genesis_ws
python3.10 -m venv genesis_env
source ~/genesis_ws/genesis_env/bin/activate
git clone https://github.com/kindsenior/Genesis.git -b agent_system_lecture2025
cd Genesis
pip install -r requirements_Ubuntu20.04_agent-system_cuda.txt

# No.7
curl -fLO https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.7.0%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.7.0+cpu.zip

# No.2
mkdir -p ~/ros/agent_system_ws/src
cd ~/ros/agent_system_ws/src
git clone https://github.com/na-trium-144/agent-system-lecture2025.git lecture2025
vcs import --input lecture2025/.rosinstall
cd ..
# rosdep install --from-paths src -r -i -y
# cd src/choreonoid/misc/script
# ./install-requisites-ubuntu-20.04.sh
source /opt/ros/noetic/setup.bash
cd ~/ros/agent_system_ws
catkin build  # depends on libtorch
```

## train & run

```sh
source ~/genesis_ws/genesis_env/bin/activate
python ./framy_model.py
```

```sh
source ~/ros/agent_system_ws/devel/setup.bash
source ~/genesis_ws/genesis_env/bin/activate
export PYOPENGL_PLATFORM=glx
export PYTHONPATH=
python ./genesis/go2_train.py
PYTHONPATH=`pwd`/genesis python ./inference_tutorial/scripts/dump_training_data.py -l ./logs/go2-walking/test --ckpt 100
tensorboard --logdir logs/go2-walking/test/ --host 0.0.0.0
python ./genesis/go2_eval.py
```

```sh
cd ~/genesis_ws
curl -fLO https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.7.0%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.7.0+cpu.zip
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${HOME}/genesis_ws/libtorch/lib
source /opt/ros/noetic/setup.bash
export PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages
cd ~/ros/agent_system_ws
catkin build
```

```sh
source ~/ros/agent_system_ws/devel/setup.bash
TARGET_PATH=`pwd`/logs/go2-walking/test LANG=C.utf-8 LC_ALL=C.utf-8 choreonoid ./framy.cnoid
```
