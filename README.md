# Manipulator GUI

A collection of graphical user interfaces for the FRANKA EMIKA Panda robot manipulator project.

![Manipulator GUI](https://i.imgur.com/placeholder.png)

## Overview

This package provides a comprehensive set of graphical user interfaces for configuring, training, and evaluating robot control models:

- **Setup Menu**: For Xsense data processing, DMP creation, and configuration
- **Training RL Menu**: For training reinforcement learning models with various algorithms
- **Evaluation and Run Menu**: For running experiments and evaluating models

## Features

- **Unified Launcher**: Central GUI to launch specific tools
- **Interactive Visualizations**: Real-time graphs for trajectory visualization
- **Configuration Utilities**: Parameter adjustment for various algorithms
- **Training Progress Monitoring**: Live training metrics and model performance
- **Experiment Management**: Tools for organizing and running experiments

## Architecture

The package is organized into the following components:

```
manipulator_gui/
├── manipulator_gui/        # Python modules
│   ├── GUI.py             # Main launcher
│   ├── SetupMenu.py       # Setup interface
│   ├── TrainRLMenu.py     # RL training interface
│   ├── EvalnRunMenu.py    # Evaluation interface
│   ├── __main__.py        # Entry point
│   └── __init__.py
└── resources/             # GUI resources (images, icons, etc.)
```

## Dependencies

- ROS 2 Humble
- Python 3.8+
- Tkinter (for GUI components)
- PIL/Pillow (for image processing)
- matplotlib (for plotting)
- manipulator package
- manipulator_skill_acquisition package
- manipulator_control_strategies package

## Installation

1. Install required system dependencies:
   ```bash
   sudo apt-get update
   sudo apt-get install python3-tk
   pip3 install pillow matplotlib
   ```

2. Clone the repository into your workspace:
   ```bash
   cd ~/ws_manipulator/src
   git clone https://github.com/username/manipulator_gui.git
   ```

3. Build the workspace:
   ```bash
   cd ~/ws_manipulator
   colcon build --packages-select manipulator_gui
   source install/setup.bash
   ```

## Usage

### Running the Main GUI

After building, you can run the GUI launcher using:

```bash
# Source the workspace
source ~/ws_manipulator/install/setup.bash

# Launch the GUI
ros2 run manipulator_gui GUI
```

### Using Individual GUI Components

You can also run individual GUI components directly:

```bash
# Setup Menu
ros2 run manipulator_gui SetupMenu

# Training RL Menu
ros2 run manipulator_gui TrainRLMenu

# Evaluation and Run Menu
ros2 run manipulator_gui EvalnRunMenu
```

### GUI Functionality

#### Setup Menu
- Import and process Xsense motion capture data
- Create and configure Dynamic Movement Primitives (DMPs)
- Visualize and adjust trajectories

#### Training RL Menu
- Select RL algorithm (TD3, DDPG, SAC)
- Configure training parameters
- Monitor training progress
- Save trained models

#### Evaluation and Run Menu
- Load saved models and DMPs
- Configure evaluation scenarios
- Run experiments with the robot
- Visualize and analyze results

## Troubleshooting

If you encounter issues with the GUI:

1. Check if all dependencies are installed:
   ```bash
   pip3 install pillow matplotlib
   sudo apt-get install python3-tk
   ```

2. Verify resource paths:
   ```bash
   ls -la ~/ws_manipulator/install/manipulator_gui/share/manipulator_gui/resources/
   ```
   
3. Check for error messages in the console output

4. Try running with debug output:
   ```bash
   python3 -m manipulator_gui.GUI --debug
   ```

## License

This project is licensed under the Apache License 2.0. 