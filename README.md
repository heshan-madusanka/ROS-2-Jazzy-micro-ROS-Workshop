# ROS-2-Jazzy-micro-ROS-Workshop
ROS2-jazzy, Rviz, micro_ROS, ESP32

Task 01
=======
Developing URDF models using only simple geometry primitives like box, cylinder, and sphere becomes impractical as robot complexity increases. For accurate Gazebo simulation you need 3D models that closely match the real robot. For that reason, we can use a SolidWorks-to-URDF exporter to create robot models with complex geometry. (Simple URDF models remain important because URDF has limitations, for example in modeling closed-loop joints.)

Install sw_urdf_exporter and set up the assembly
------------------------------------------------
- Install the sw_urdf_exporter add-in. Follow the official guide here: [https://wiki.ros.org/sw_urdf_exporter](https://wiki.ros.org/sw_urdf_exporter)
- Download the servo_motor from the repository and open the assembly file servo_assembly.SLDASM in SolidWorks. Inspect the assembly to understand the defined links and mates.
- In SolidWorks, navigate to Tools -> Export as URDF to run the exporter.

Configure base link and child links
-----------------------------------
- Select the base_link geometry and assign it as the base link.
- Set the number of child links to 1.
- Rename the empty link to link_01 and the corresponding joint to joint_01.
- For joint_01 choose type revolute_joint and set its related geometry to LINK_1.
![sample_model image](images/urdf_exporter.gif)

Preview and export
------------------
- Click Preview, then click Export. This will apply the changes and open a new window.
- In the new window, review and confirm joint parameters and link properties.
- When everything is correct, choose Export URDF and Meshes.
- Save the output as servo_description in a suitable folder on your system.

Build a ROS 2 package from the exported URDF
--------------------------------------------
The exporter produces a ROS 1-style package layout and launch files. ROS 2 uses different package manifests, build tools, and launch conventions, so you must repackage and adapt the files before trying to visualize or simulate.
- Create a new package in your ROS 2 workspace called servo_description.
- Copy the exported URDF and the meshes folder from the SolidWorks exporter into your new package.
- Follow the step-by-step guide in ROS2-Jazzy-Workshop-01 notes to visualize the model in rviz2. That guide covers the exact commands and launch files required for Jazzy.
![rviz_display.launch view](images/rviz_display_view)

# micro-ROS
Installing micro-ROS
====================
follow the instructions detailed [here](https://micro.ros.org/docs/tutorials/core/first_application_linux/)).

    # Create a workspace and download the micro-ROS tools
    mkdir microros_ws
    cd microros_ws
    git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
    
    # Update dependencies using rosdep
    sudo apt update && rosdep update
    rosdep install --from-paths src --ignore-src -y
    
    # Install pip
    sudo apt-get install python3-pip
    
    # Build micro-ROS tools and source them
    colcon build
    source install/local_setup.bash

These instructions will setup a workspace with a ready-to-use micro-ROS build system. This build system is in charge of downloading the required cross-compilation tools and building the apps for the required platforms.

Creating a new firmware workspace
--------------------------------
Once the build system is installed, let’s create a firmware workspace that targets all the required code and tools:

    ros2 run micro_ros_setup create_firmware_ws.sh host
For the user to create a custom application, a folder <my_app> will need to be registered in this location, containing the two files just described. Also, any such new application folder needs to be registered in src/uros/micro-ROS-demos/rclc/CMakeLists.txt by adding the following line:

    export_executable(<my_app>)

Building the firmware
---------------------

Once the app has been created, the build step is in order. given that we are compiling micro-ROS in the host machine rather than in a board, the cross-compilation implemented by the configuration step is not required in this case. We can therefore proceed to build the firmware and source the local installation:

    ros2 run micro_ros_setup build_firmware.sh
    source install/local_setup.bash

Creating the micro-ROS agent
---------------------------
The micro-ROS app is now ready to be connected to a micro-ROS agent to start talking with the rest of the ROS 2 world. To do that, let’s first of all create a micro-ROS agent:

    ros2 run micro_ros_setup create_agent_ws.sh
Now, let’s build the agent packages and, when this is done, source the installation:

    ros2 run micro_ros_setup build_agent.sh
    source install/local_setup.bash    
Add micro-ROS environment to bashrc 
-----------------------------------
You can add the ROS 2 and micro-ROS workspace setup files to your .bashrc so the files do not have to be sourced every time a new command line is opened.

    echo source ~/microros_ws/install/local_setup.bash >> ~/.bashrc





