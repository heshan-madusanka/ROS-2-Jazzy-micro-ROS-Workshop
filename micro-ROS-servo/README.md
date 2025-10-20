# Let's break down the code section by section.

Header Includes
---------------
First, we include all the necessary libraries.

    #include <micro_ros_arduino.h>

    #include <rcl/rcl.h>
    #include <rcl/error_handling.h>
    #include <rclc/rclc.h>
    #include <rclc/executor.h>
    
    #include <std_msgs/msg/float64.h>
    #include <sensor_msgs/msg/joint_state.h>
    
    #include <rosidl_runtime_c/string_functions.h>
    #include <rosidl_runtime_c/primitives_sequence_functions.h>
    
    #include <ESP32Servo.h>
    #include <math.h>
    
    // micro-ROS RMW utilities for time sync
    #include <rmw_microros/rmw_microros.h>

- **micro_ros_arduino.h**: The main library that enables micro-ROS functionality on Arduino-compatible boards.
- **rcl/... & rclc/...**: These are parts of the core ROS 2 Client Library. They provide the fundamental tools to create nodes, publishers, subscribers, and executors.
- **std_msgs/msg/float64.h**: Defines the simple message type for our command subscriber, which just contains a single floating-point number.
- **sensor_msgs/msg/joint_state.h**: This is the important one. It defines the message structure for publishing joint states, exactly matching the type we inspected in TASK 02.
- **ESP32Servo.h**: A library to control the hardware servo motor.
- **rmw_microros/rmw_microros.h**: A special header for accessing micro-ROS time synchronization features. This is crucial for getting accurate timestamps on our messages so RViz can display them correctly.

Defines and Global Variables
----------------------------
Here we define constants and declare the main variables we will use throughout the program.

    #define SERVO_PIN      13      // change to the pin you use
    #define PUBLISH_HZ     30      // joint_states publish frequency
    #define DEG2RAD(d)     ((d) * M_PI / 180.0)
    
    rcl_publisher_t joint_pub;
    rcl_subscription_t servo_sub;
    
    sensor_msgs__msg__JointState joint_msg;
    std_msgs__msg__Float64 incoming_cmd;
    
    rclc_executor_t executor;
    rclc_support_t support;
    rcl_allocator_t allocator;
    rcl_node_t node;
    
    Servo myServo;
    
    volatile double current_position_deg = 0.0;

- SERVO_PIN: The physical pin on the ESP32 that the servo's signal wire is connected to.
- PUBLISH_HZ: How many times per second we will publish the servo's position.
- rcl_publisher_t joint_pub: The object that will publish messages to /joint_states.
- rcl_subscription_t servo_sub: The object that will subscribe to servo_cmd.
- sensor_msgs__msg__JointState joint_msg: A variable to store the joint state data before we send it.
- executor, support, allocator, node: Core micro-ROS objects that manage the node, memory, and task execution.
- myServo: An object from the ESP32Servo library representing our physical servo.
- volatile double current_position_deg: A global variable to store the servo's position. It is shared between the main loop and the subscriber callback.







