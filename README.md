moonshot_project/
├── Dockerfile
├── docker-compose.yml  # Optional, for multi-container or service orchestration
├── README.md           # Project documentation
├── scripts/            # Utility scripts for setup, build, run
│   ├── build.sh        # Script to build the Docker image
│   ├── run.sh          # Script to run the Docker container
│   └── setup_host.sh   # Script for host machine setup (e.g., dependencies)
├── config/             # Configuration files
│   ├── kinova_config/  # Arm-specific configs (e.g., URDF, YAML for each arm)
│   │   ├── arm1.yaml
│   │   ├── arm2.yaml
│   │   ├── arm3.yaml
│   │   └── arm4.yaml
│   ├── lidar_config/   # LIDAR sensor configs
│   │   └── lidar.yaml  # General LIDAR params or per-instance configs
│   └── table_config/   # Moving table configs
│       ├── table1.yaml
│       └── table2.yaml
├── ros2_ws/            # ROS2 workspace
│   ├── build/          # Build artifacts (git-ignored)
│   ├── install/        # Install artifacts (git-ignored)
│   ├── log/            # Logs (git-ignored)
│   └── src/            # Source packages
│       ├── kinova_gen3_lite_control/  # Package for controlling Kinova Gen3 Lite arms
│       │   ├── launch/                # Launch files for arms
│       │   │   ├── all_arms.launch.py # Launch all 4 arms
│       │   │   ├── arm_group1.launch.py  # Arms on table 1
│       │   │   └── arm_group2.launch.py  # Arms on table 2
│       │   ├── config/                # Package-specific configs (symlink or copy from ../config)
│       │   ├── src/                   # Source code (e.g., controllers, nodes)
│       │   ├── CMakeLists.txt
│       │   └── package.xml
│       ├── moving_table_control/      # Package for controlling ceiling-mounted moving tables
│       │   ├── launch/                # Launch files for tables
│       │   │   └── tables.launch.py
│       │   ├── src/                   # Source code for table movement logic
│       │   ├── CMakeLists.txt
│       │   └── package.xml
│       ├── lidar_integration/         # Package for LIDAR sensors
│       │   ├── launch/                # Launch files for LIDAR
│       │   │   └── lidar.launch.py
│       │   ├── config/                # LIDAR-specific configs
│       │   ├── src/                   # Nodes for LIDAR data processing
│       │   ├── CMakeLists.txt
│       │   └── package.xml
│       ├── multi_robot_system/        # Top-level package for system coordination
│       │   ├── launch/                # Main launch files
│       │   │   ├── simulation.launch.py  # For Gazebo or similar
│       │   │   └── real_hardware.launch.py
│       │   ├── urdf/                  # Robot description files (URDF/XACRO)
│       │   │   ├── full_system.xacro  # Combined model for all components
│       │   │   ├── kinova_gen3_lite.xacro  # Base model for each arm
│       │   │   ├── moving_table.xacro      # Model for each table
│       │   │   └── lidar.xacro            # LIDAR sensor model
│       │   ├── rviz/                  # RViz configs for visualization
│       │   │   └── default.rviz
│       │   ├── src/                   # Coordination nodes
│       │   ├── CMakeLists.txt
│       │   └── package.xml
│       └── dependencies/              # External packages (e.g., Kinova ROS2 drivers)
│           └── kinova_ros2_driver/    # Example: Official or community Kinova ROS2 package
└── volumes/            # Persistent volumes for Docker
    └── data/           # Shared data (e.g., logs, recordings)