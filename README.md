# ROS-2-Jazzy-micro-ROS-Workshop
ROS2-jazzy, Rviz, micro_ROS, ESP32

Task 01
=======
Developing URDF models using only simple geometry primitives like box, cylinder, and sphere becomes impractical as robot complexity increases. For accurate Gazebo simulation you need 3D models that closely match the real robot. For that reason, we can use a SolidWorks-to-URDF exporter to create robot models with complex geometry. (Simple URDF models remain important because URDF has limitations, for example in modeling closed-loop joints.)

Install sw_urdf_exporter and set up the assembly
------------------------------------------------
- Install the sw_urdf_exporter add-in. Follow the official guide here: [https://wiki.ros.org/sw_urdf_exporter](https://wiki.ros.org/sw_urdf_exporter)
- Download the sample_model from the repository and open the assembly file ARM.SLDASM in SolidWorks. Inspect the assembly to understand the defined links and mates.
- Before using the add-in, suppress the three mates named **temporary_mate_I**, **temporary_mate_II**, and **temporary_mate_III**
- In SolidWorks, navigate to Tools -> Export as URDF to run the exporter.
![sample_model image](images/sample_model.png)

Configure base link and child links
-----------------------------------
- Select the base geometry and assign it as the base link.
- Set the number of child links to 1.
- Rename the empty link to link_01 and the corresponding joint to joint_01.
- For joint_01 choose type revolute_joint and set its related geometry to LINK_1.
- Select link_1, set its number of child links to 2, and give those child links and their joints clear, consistent names (for example link_02, joint_02, link_03, joint_03).
- For each new link and joint, pick the appropriate geometry element that represents that part of the model and verify the parent-child relationships are correct.
![sample_model image](images/solidwork_exporter_setup.gif)

Preview and export
------------------
- Click Preview, then click Export. This will apply the changes and open a new window.
- In the new window, review and confirm joint parameters and link properties.
- When everything is correct, choose Export URDF and Meshes.
- Save the output as sample_description in a suitable folder on your system.

Build a ROS 2 package from the exported URDF
--------------------------------------------
The exporter produces a ROS 1-style package layout and launch files. ROS 2 uses different package manifests, build tools, and launch conventions, so you must repackage and adapt the files before trying to visualize or simulate.
- Create a new package in your ROS 2 workspace called sample_description.
- Copy the exported URDF and the meshes folder from the SolidWorks exporter into your new package.
- Follow the step-by-step guide in ROS2-Jazzy-Workshop-01 notes to visualize the model in rviz2 and spawn it into Gazebo. That guide covers the exact commands and launch files required for Jazzy.

