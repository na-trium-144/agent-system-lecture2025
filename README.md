# lecture2025

```sh
source ~/genesis_ws/genesis_env/bin/activate
python ./hoge_model.py
```

```sh
source ~/ros/agent_system_ws/devel/setup.bash
source ~/genesis_ws/genesis_env/bin/activate
export PYOPENGL_PLATFORM=glx
PYTHONPATH= python ./genesis/go2_train.py
PYTHONPATH=`pwd`/genesis python ./inference_tutorial/scripts/dump_training_data.py -l ./logs/go2-walking/test --ckpt 100
PYTHONPATH= python ./go2_eval.py
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
TARGET_PATH=`pwd`/logs/go2-walking/test LANG=C.utf-8 LC_ALL=C.utf-8 choreonoid ./hoge.cnoid
```
