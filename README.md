# Self-Playing XiangQi Autonomous Robot

In this project, the task is to design an autonomous robot that can self-play XiangQi. Interaction with the user is also possible with move inputs into the terminal.

## Requirements

You should implement the navigation and motion planning algorithms by yourself to accomplish the task.

### Project Materials (10%)

Clear and readable project report (4%)
Runnable codebase (4%)
Clear video results (2%)

### Bonus (10%)

5%: if you implement trajectory planning for the robot arm (including inverse dynamics)

## Installation

1. Create a new Anaconda environment: `conda create -n xiangqi python=3.10`

1. Activate the environment: `conda activate xiangqi`

1. Install pip dependencies: `pip3 install "numpy<2" opencv-python`

1. Install conda dependencies: `conda install conda-forge::pybullet`

1. (If you do not already have the codebase) `git clone git@github.com:aureliony/CS4278-Project.git`

## Run the simulation

Run the simulation (Pikafish vs. Pikafish) with the command `python -m simulation.chess`.

To play as red against Pikafish, run `python -m simulation.chess --humans 1`.

To play against yourself or a friend, run `python -m simulation.chess --humans 2`.

## References

PyBullet Documentation: <https://pybullet.org/wordpress/index.php/forum-2/>

You can also use ChatGPT or Cursor to get the API of PyBullet.
