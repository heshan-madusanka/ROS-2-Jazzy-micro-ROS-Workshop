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

Subscriber Callback Function
----------------------------
This function runs automatically every time a message is received on the servo_cmd topic. Its job is to read the incoming command and move the physical servo.
    
    void servo_command_callback(const void * msgin) {
      const std_msgs__msg__Float64 * msg = (const std_msgs__msg__Float64 *)msgin;
    
      // Assume incoming value is degrees. Constrain to valid servo range.
      double angle_deg = msg->data;
      if (angle_deg < 0.0) angle_deg = 0.0;
      if (angle_deg > 180.0) angle_deg = 180.0;
    
      // Update current position and move servo
      current_position_deg = angle_deg;
      myServo.write((int)angle_deg);
    }

In this function, we take the incoming angle, make sure it is within the safe 0-180 degree range, update our current_position_deg variable, and command the hardware servo to move.

The setup() Function
--------------------
This function runs once when the ESP32 boots up. It initializes the hardware, sets up the serial communication, and creates all the ROS 2 objects.
    
    void setup() {
      // ... (Serial and Servo initialization) ...
    
      set_microros_transports();
      
      // ... (micro-ROS support and node initialization) ...
    
      // Initialize publisher for sensor_msgs/JointState on /joint_states
      RCCHECK(rclc_publisher_init_default(
        &joint_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "/joint_states"
      ));
    
      // Initialize subscriber for std_msgs/Float64 on servo_cmd
      RCCHECK(rclc_subscription_init_default(
        &servo_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "servo_cmd"
      ));
    
      // Initialize executor with one subscription
      RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
      RCCHECK(rclc_executor_add_subscription(&executor, &servo_sub, &incoming_cmd, &servo_command_callback, ON_NEW_DATA));
    
      // Initialize joint_msg sequences - one joint only
      // ... (setting joint name and initializing position array) ...
    
      // Try to synchronize epoch time with the micro-ROS Agent
      rmw_ret_t sync_rc = rmw_uros_sync_session(TIME_SYNC_TIMEOUT_MS);
      
      // ...
    }

Key steps here include:

1. Attaching the servo to its pin.
2. Setting up the micro-ROS transport (Serial).
3. Initializing the node, publisher (on /joint_states), and subscriber (on servo_cmd).
4. Adding the subscription to the executor and linking it to our servo_command_callback.
5. Preparing the joint_msg structure by setting the joint name (this must match your URDF file) and initializing the data arrays.
6. Attempting to synchronize time with the ROS Agent.

The loop() Function
-------------------
This function runs continuously after setup() is complete. It has two main responsibilities: checking for new messages and publishing the current state.

    void loop() {
      // Process incoming messages and call callbacks
      RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50)));
    
      // Publish joint_states at the configured rate
      static unsigned long last_pub_ms = 0;
      // ... (timer logic) ...
    
      if ((now - last_pub_ms) >= interval_ms) {
        last_pub_ms = now;
    
        // Update joint message position from current_position_deg
        joint_msg.position.data[0] = DEG2RAD(current_position_deg);
    
        // --- stamp the header with synchronized micro-ROS epoch time ---
        int64_t time_ns = rmw_uros_epoch_nanos();
        // ... (logic to get timestamp and put it in joint_msg.header.stamp) ...
    
        // Publish joint state
        RCSOFTCHECK(rcl_publish(&joint_pub, &joint_msg, NULL));
    
        // ... (Optional debug print) ...
      }
      delay(1);
    }
    
rclc_executor_spin_some(...): This line tells the executor to process any available data. If a message has arrived for our subscriber, this is the line that will trigger the callback function.

Timed Publisher: The if statement creates a timer. At the rate of PUBLISH_HZ, it will:
- Read the current_position_deg, convert it to radians, and update the joint_msg.position.
- Get the current synchronized ROS time and add it to the message header (joint_msg.header.stamp).
- Finally, call rcl_publish() to send the message on its way to RViz.
