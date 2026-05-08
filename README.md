# Repository Structure

```text
ROBOPharma-GUI/
│
├── documentation/     # User guides and system documentation
├── launch/            # Launch scripts for the ROS 2 system
├── logs/              # Database files and runtime logs
├── ros2_ws/src/       # ROS 2 workspace source packages
├── simulation/        # CoppeliaSim .ttt scene files
├── .gitignore
└── README.md
```

### Directory Details

- **`documentation/`**  
  Contains technical and non technical user guides, setup instructions, and troubleshooting documentation.

- **`launch/`**  
  Contains scripts used to launch the ROS 2 bridge and GUI nodes.  
  > **Note:** File paths must be updated to point to correct launch directory before use.

- **`logs/`**  
  Stores the system database, errors log and images from detection results.

- **`ros2_ws/src/`**  
  ROS 2 workspace source directory containing the custom packages:
  - `bridge`
  - `gui`
  - `custom_interfaces`

- **`simulation/`**  
  Contains the CoppeliaSim `.ttt` simulation scene file.


---

# Prerequisites

Before setting up the project, ensure the following software is installed:

- **ROS 2** (Developed with Jazzy Jalisco)
- **CoppeliaSim** (Developed with V4.10.0)
- **Python**
- **colcon** build tools

Example installation packages for ROS 2:

```bash
sudo apt install python3-colcon-common-extensions
```

---

# Setup & Installation

## 1. Build the ROS 2 Workspace

The `ros2_ws` directory currently only contains the `src` folder.  
Build the ROS 2 packages using `colcon`:

```bash
cd ros2_ws
colcon build
source install/setup.bash
```

---

## 2. Configure the CoppeliaSim ROS 2 Interface

This project uses the `sim_ros2_interface` from the `simROS2` package, follow steps at https://github.com/CoppeliaRobotics/simROS2 to install.

To enable custom message types to be used between ROS 2 and CoppeliaSim:

### Add Custom Message Types

Follow the official `simROS2` documentation to include your custom messages in the interface build process.

### Move the Plugin File

Add <depend>custom_interfaces</depend> to package.xml, and custom_interfaces to ament_target_dependencies in sim_ros2_interface package. Then build using colcon build. 

After successfully building the `simROS2` interface:

1. Locate the generated plugin file:

```text
libSimROS2.so
```

2. Copy or move this file into your root CoppeliaSim installation directory.

This allows CoppeliaSim to load the ROS 2 plugin during startup.

---

## 3. Update Launch File Paths

Before running the system:

1. Navigate to the `launch/` directory.
2. Open each launch script.
3. Update all directory paths to match the absolute paths on your local machine.

---

# Running the System

Run the required launch file from the `launch/` directory to start:

Example:

```bash
ros2 launch bridge launchSystem.launch.py
```

---

# Optional Desktop Executable

A desktop shortcut or executable launcher can be created to automate:

- Starting CoppeliaSim
- Launching ROS 2 nodes
- Opening the GUI

This allows the system to be launched directly from the desktop.

In the main launch folder there is a script which points to the bridge launch file


```bash
chmod +x /path/to/your/launch/master_launch.sh
nano ~/Desktop/robopharma_launcher.desktop
```

```bash
[Desktop Entry]
Version=1.0
Name=ROBOPharma_GUI
Comment=Starts GUI system
Exec=/path/to/your/launch/master_launch.sh
Icon=utilities-terminal
Terminal=true
Type=Application
Categories=Utility;
```


This can then be clicked to launch the whole system.

(NOTE: ensure that all paths in the master_launch are correct)
---

# Documentation

Detailed setup instructions, GUI usage, system interaction, and troubleshooting information can be found in:

```text
documentation/
```

---

# Database & Logs

The local database and generated runtime logs are stored in:

```text
logs/
```


# Developer
For any problems or questions with the system contact Roberta Griffiths: birdie.griffithe@gmail.com
