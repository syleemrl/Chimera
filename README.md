## Installation
We checked code works in Ubuntu 18.04 and 20.04.

### Setup

All dependencies can be installed at once by the command below.
```bash
./install.sh
```
This script download sources on ``c_env`` and ``py_env`` and installs on the same directory.

### Building
You can build via:
```bash
./run_cmake.sh
cd build
make -jN
```

##  Render
You can render the input motion file/trained network by executing ``render`` file in ``build/render``. 
You should activate the installed virtual environment by ``source py_env/bin/activate`` before running.

To open renderer, you need render config file in ``data/render``
```bash
cd build/render
./render --config=FILE_NAME
```

## Train

All possible arguments are written in ``network/ppo.py``.
```bash
cd network
python3 ppo.py --test_name=TEST_NAME --nslave=N --sim_config=FILE_NAME
```

For example, to train chimera1 walk,
```bash
cd network
python3 ppo.py --test_name=chimera1_walk --nslave=8 --sim_config=chimera1/config_walk.txt
```

### Input Files
To train assembled character, you need 

source characters (skeleton) in ``data/character``

source motions in ``data/motion``

assembled character (skeleton) in ``data/character/assembled``

charactor configurations for simulation in ``data/config``

configurations for overall training in ``data/config``

