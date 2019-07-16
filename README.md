# CollaborativeBaxter

## Repo for KTH - FACT Project demonstrator of collaborative operations

- Pickup object
- Assembly task
- Inspection task

## Dependencies (to be installed separately)

- MoveIt! (`sudo apt install ros-kinetic-moveit*`)
- Python (tested with Python 3.5.2 + 2.7.12 installed, working with Python 2.7.12)
- `sudo easy_install -U scikit-learn==0.18.1` (last version compatible with both Python 2.7 and 3.4, also possible to install it via `pip`) + `sudo apt install python-sklearn`
- OpenCV (tested with OpenCV 3.3.1-dev)

## Installation

- Edit `baxter_ridgeback.sh` file --> YOUR_IP (check `ifconfig` output)
