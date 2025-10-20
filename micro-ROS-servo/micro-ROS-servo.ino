/*
  ESP32 micro-ROS example using ESP32Servo
  - Subscribe:  "servo_cmd"         (std_msgs/msg/Float64)  -> expects degrees [0..180]
  - Publish :  "/joint_states"      (sensor_msgs/msg/JointState) -> publishes position in radians regularly

  Minimal change: synchronize micro-ROS epoch with the Agent and stamp JointState.header
  using rmw_uros_epoch_nanos() so timestamps are non-zero / consistent with ROS 2 host.
*/

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

#define SERVO_PIN      13      // change to the pin you use
#define PUBLISH_HZ     30      // joint_states publish frequency
#define DEG2RAD(d)     ((d) * M_PI / 180.0)

#define TIME_SYNC_TIMEOUT_MS 1000

rcl_publisher_t joint_pub;
rcl_subscription_t servo_sub;

sensor_msgs__msg__JointState joint_msg;
std_msgs__msg__Float64 incoming_cmd;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

Servo myServo;

// Current position in degrees, updated by subscriber callback and used by publisher
volatile double current_position_deg = 0.0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ Serial.print("RCCHECK failed: "); Serial.println(temp_rc); error_loop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ /* non-fatal, optionally log */ Serial.print("RCSOFTCHECK failed: "); Serial.println(temp_rc); } }

void error_loop() {
  while (1) {
    delay(100);
  }
}

// Subscriber callback: update current_position_deg and drive servo
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

void setup() {
  Serial.begin(115200);
  delay(200);

  // Attach servo (ESP32Servo supports optional min/max pulse widths)
  myServo.setPeriodHertz(50); // Standard servo frequency 50Hz
  myServo.attach(SERVO_PIN);  // you can also use myServo.attach(pin, minPulse, maxPulse)

  // Initialize micro-ROS transport (serial by default) and give it time
  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();

  // Initialize rclc support
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "esp32_joint_node", "", &support));

  // Initialize publisher for sensor_msgs/JointState on /joint_states
  RCCHECK(rclc_publisher_init_default(
    &joint_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/joint_states"
  ));

  // Initialize subscriber for std_msgs/Float64 on servo_cmd
  RCCHECK(rclc_subscription_init_default(
    &servo_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "servo_cmd"
  ));

  // Initialize executor with one subscription
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &servo_sub, &incoming_cmd, &servo_command_callback, ON_NEW_DATA));

  // Initialize joint_msg sequences - one joint only
  if (!rosidl_runtime_c__String__Sequence__init(&joint_msg.name, 1)) {
    error_loop();
  }
  if (!rosidl_runtime_c__String__assign(&joint_msg.name.data[0], "joint_01")) {
    error_loop();
  }

  if (!rosidl_runtime_c__double__Sequence__init(&joint_msg.position, 1)) {
    error_loop();
  }
  joint_msg.position.data[0] = 0.0;

  if (!rosidl_runtime_c__double__Sequence__init(&joint_msg.velocity, 0)) {
    error_loop();
  }
  if (!rosidl_runtime_c__double__Sequence__init(&joint_msg.effort, 0)) {
    error_loop();
  }

  // Leave header.stamp at zero initially; we'll stamp before each publish after sync
  joint_msg.header.stamp.sec = 0;
  joint_msg.header.stamp.nanosec = 0;

  // Ensure initial servo position is consistent with current_position_deg
  current_position_deg = 0.0;
  myServo.write((int)current_position_deg);
  joint_msg.position.data[0] = DEG2RAD(current_position_deg);

  // Try to synchronize epoch time with the micro-ROS Agent so rmw_uros_epoch_nanos() returns meaningful values.
  // This is the key change to avoid zero timestamps.
  rmw_ret_t sync_rc = rmw_uros_sync_session(TIME_SYNC_TIMEOUT_MS);
  if (sync_rc == RMW_RET_OK) {
    if (rmw_uros_epoch_synchronized()) {
      Serial.println("micro-ROS time sync: OK");
    } else {
      Serial.println("micro-ROS time sync: returned OK but not synchronized");
    }
  } else {
    Serial.print("micro-ROS time sync failed: ");
    Serial.println(sync_rc);
  }

  Serial.println("micro-ROS servo node (ESP32Servo) initialized");
}

void loop() {
  // Process incoming messages and call callbacks
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50)));

  // Publish joint_states at the configured rate
  static unsigned long last_pub_ms = 0;
  unsigned long now = millis();
  unsigned long interval_ms = 1000 / PUBLISH_HZ;

  if ((now - last_pub_ms) >= interval_ms) {
    last_pub_ms = now;

    // Update joint message position from current_position_deg
    joint_msg.position.data[0] = DEG2RAD(current_position_deg);

    // --- NEW: stamp the header with synchronized micro-ROS epoch time ---
    int64_t time_ns = rmw_uros_epoch_nanos();

    // If not synchronized yet, try a short sync attempt and re-read.
    if (time_ns == 0) {
      rmw_ret_t quick_rc = rmw_uros_sync_session(100);
      (void)quick_rc; // don't block on failure, we try to get non-zero time if possible
      time_ns = rmw_uros_epoch_nanos();
    }

    if (time_ns != 0) {
      joint_msg.header.stamp.sec = (int32_t)(time_ns / 1000000000LL);
      joint_msg.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000LL);
    } else {
      // As a last resort, use millis() to produce a non-zero stamp relative to device uptime.
      // This is not ideal for cross-device time consistency but prevents stamp==0.
      uint64_t ms = (uint64_t)millis();
      joint_msg.header.stamp.sec = (int32_t)(ms / 1000);
      joint_msg.header.stamp.nanosec = (uint32_t)((ms % 1000) * 1000000ULL);
    }

    // Publish joint state
    RCSOFTCHECK(rcl_publish(&joint_pub, &joint_msg, NULL));

    // Optional debug print
    Serial.print("Published /joint_states position (deg): ");
    Serial.print(current_position_deg);
    Serial.print("  stamp.sec=");
    Serial.print(joint_msg.header.stamp.sec);
    Serial.print(" stamp.nanosec=");
    Serial.println(joint_msg.header.stamp.nanosec);
  }

  // small delay to yield CPU
  delay(1);
}
