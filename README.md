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
![sample_model image](images/solidwork_exporter_setup.gif)

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

