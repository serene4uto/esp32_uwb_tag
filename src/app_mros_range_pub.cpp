#include "app_mros_range_pub.h"
#include "main.h"

#ifdef UWB_LOCALIZATION_PUB_ENABLED
#include "mm3v3.h"
#include <geometry_msgs/msg/point.h>
#endif

// Macro for error checking
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

float received_anchor_distances[TAG_NUMBER_OF_ANCHOR_MAX] = {0.0}; 

rcl_allocator_t rcl_rangepub_allocator;
rclc_support_t rcl_rangepub_support;
rcl_node_t rcl_rangepub_node;
rclc_executor_t rcl_rangepub_executor;

#ifdef UWB_RANGE_PUB_ENABLED
rcl_publisher_t rcl_rangepub_publisher;
uwb_interfaces__msg__UwbRange uwb_range_msg;
#endif

// UWB Localization
#ifdef UWB_LOCALIZATION_PUB_ENABLED
geometry_msgs__msg__Point tag_position_msg;
rcl_publisher_t rcl_tag_position_publisher;
float tag_position[3] = {0.0};
float tag_distance_rmse = 0.0;
const double ANCHOR_MATRIX[TAG_NUMBER_OF_ANCHOR_MAX][3] = {
    //  {x, y, z}
  { 0.0 , 0.0 , 0.93},
  {1.683, 0.0 , 1.75},
  {1.683, 1.25, 2.065},
  { 0.0 , 1.25, 1.33},
};

static int trilat3D_4A(float *anchor_distance, float *tag_position, float &distance_rmse) {
  // for method see technical paper at
  // https://www.th-luebeck.de/fileadmin/media_cosa/Dateien/Veroeffentlichungen/Sammlung/TR-2-2015-least-sqaures-with-ToA.pdf

  static bool first_run = true;  //first time through, some preliminary work
  float b[TAG_NUMBER_OF_ANCHOR_MAX], d[TAG_NUMBER_OF_ANCHOR_MAX]; //distances from anchors
  static float Ainv[3][3], k[TAG_NUMBER_OF_ANCHOR_MAX]; //these are calculated only once
  float detA;
  float posn2[3];

  if(TAG_NUMBER_OF_ANCHOR_MAX != 4) {
    LOG_ERROR("This method is only for 4 anchors");
    return 0;
  }

  // copy distances to local storage
  for (int i = 0; i < TAG_NUMBER_OF_ANCHOR_MAX; i++) {
    d[i] = anchor_distance[i];
  }

  if (first_run) {  //intermediate fixed vectors
    first_run = false;

    float x[TAG_NUMBER_OF_ANCHOR_MAX], y[TAG_NUMBER_OF_ANCHOR_MAX], z[TAG_NUMBER_OF_ANCHOR_MAX]; //intermediate vectors
    float A[3][3];  //the A matrix for system of equations to solve

    for (int i = 0; i < TAG_NUMBER_OF_ANCHOR_MAX; i++) {
      x[i] = ANCHOR_MATRIX[i][0];
      y[i] = ANCHOR_MATRIX[i][1];
      z[i] = ANCHOR_MATRIX[i][2];
      k[i] = x[i] * x[i] + y[i] * y[i] + z[i] * z[i];
    }

    // set up the A matrix
    for (int i = 1; i < TAG_NUMBER_OF_ANCHOR_MAX; i++) {
      A[i - 1][0] = x[i] - x[0];
      A[i - 1][1] = y[i] - y[0];
      A[i - 1][2] = z[i] - z[0];
#ifdef DEBUG_TRILAT
      snprintf(line, sizeof line, "A %6.2f %6.2f %6.2f", A[i - 1][0], A[i - 1][1], A[i - 1][2]);
      LOG_INFO(line);
#endif
    }

    
    DETERMINANT_3X3 (detA, A); //calculate determinant of A
#ifdef DEBUG_TRILAT
    // check solution stability (small or zero)
    LOG_INFO("Determinant of A matrix: ", detA);
    
#endif
    
    if (fabs(detA) < 1.0e-4) {  //TODO : define as parameter
      LOG_ERROR("Singular matrix, check anchor coordinates");
      return 0;
    }

    detA = 1.0 / detA;
    SCALE_ADJOINT_3X3 (Ainv, detA, A);  //Ainv is static
  }

  // set up least squares equation
  for (int i = 1; i < 4; i++) {
    b[i - 1] = d[0] * d[0] - d[i] * d[i] + k[i] - k[0];
  }

  // solve:  2 A x posn = b

  MAT_DOT_VEC_3X3(posn2, Ainv, b);

  for (int i = 0; i < 3; i++) {
    tag_position[i] = posn2[i] * 0.5; //remove factor of 2
  }

  //rms error in measured versus calculated distances
  float x[3] = {0}, rmse = 0.0, dc = 0.0;
  for (int i = 0; i < TAG_NUMBER_OF_ANCHOR_MAX; i++) {
    x[0] = ANCHOR_MATRIX[i][0] - tag_position[0];
    x[1] = ANCHOR_MATRIX[i][1] - tag_position[1];
    x[2] = ANCHOR_MATRIX[i][2] - tag_position[2];
    dc = sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
    rmse += (d[i] - dc) * (d[i] - dc);
  }
  distance_rmse = sqrt(rmse / ((float)TAG_NUMBER_OF_ANCHOR_MAX)); //copy to global
  
  return 1;
}
#endif

// Error handling loop
static void error_loop() {
  LOG_ERROR("Error in rcl:", String(rcl_get_error_string().str));
  while(1) {
    delay(100); // Prevents busy-waiting
  }
}

void app_mros_range_pub_init()
{
  rcl_rangepub_allocator = rcl_get_default_allocator();
  
  // Initialize the support structure
  RCCHECK(rclc_support_init(&rcl_rangepub_support, 0, NULL, &rcl_rangepub_allocator));

  // Create a node
  RCCHECK(rclc_node_init_default(&rcl_rangepub_node, MROS_RANGE_PUB_TOPIC_NAME, MROS_RANGE_PUB_NAMESPACE, &rcl_rangepub_support));

  // Initialize the publisher

#ifdef UWB_RANGE_PUB_ENABLED
  RCCHECK(rclc_publisher_init_default(
    &rcl_rangepub_publisher,
    &rcl_rangepub_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(uwb_interfaces, msg, UwbRange),
    MROS_RANGE_PUB_TOPIC_NAME));
#endif

#ifdef UWB_LOCALIZATION_PUB_ENABLED
  RCCHECK(rclc_publisher_init_default(
    &rcl_tag_position_publisher,
    &rcl_rangepub_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point),
    MROS_TAG_POSITION_PUB_TOPIC_NAME));
#endif

  // Create the executor
  RCCHECK(rclc_executor_init(&rcl_rangepub_executor, &rcl_rangepub_support.context, 1, &rcl_rangepub_allocator));
}

void app_mros_range_pub_task(void *pvParameters) {

  while(1) {
    // Receive distances from the queue
    if(xQueueReceive(ranging_queue, &received_anchor_distances, portMAX_DELAY) == pdTRUE) {

#ifdef DEBUG_PRINT_RECEIVED_ANCHOR_RANGE  
    String log_buffer = "Anchor received: ";
    for (int i = 0; i < TAG_NUMBER_OF_ANCHOR_MAX; i++) {
      log_buffer += String(received_anchor_distances[i], 2);
      log_buffer += " ";
    }
    LOG_INFO(log_buffer);
#endif

#ifdef UWB_RANGE_PUB_ENABLED

      uwb_range_msg.anchor_ids.data = (uint32_t *)malloc(TAG_NUMBER_OF_ANCHOR_MAX * sizeof(uint32_t));
      uwb_range_msg.range_values.data = (float *)malloc(TAG_NUMBER_OF_ANCHOR_MAX * sizeof(float));

      if (uwb_range_msg.anchor_ids.data == NULL || uwb_range_msg.range_values.data == NULL) {
          LOG_ERROR("Failed to allocate memory for uwb_range_msg");
          while (1);
      }

      uwb_range_msg.anchor_ids.size = TAG_NUMBER_OF_ANCHOR_MAX;
      uwb_range_msg.range_values.size = TAG_NUMBER_OF_ANCHOR_MAX;
      uwb_range_msg.anchor_ids.capacity = TAG_NUMBER_OF_ANCHOR_MAX;
      uwb_range_msg.range_values.capacity = TAG_NUMBER_OF_ANCHOR_MAX;

      // Fill the message with received data
      for (int i = 0; i < TAG_NUMBER_OF_ANCHOR_MAX; i++) {
        uwb_range_msg.anchor_ids.data[i] = i + 1;
        uwb_range_msg.range_values.data[i] = received_anchor_distances[i];
      }

      // Publish the message
      RCSOFTCHECK(rcl_publish(&rcl_rangepub_publisher, &uwb_range_msg, NULL));

      // Free the memory
      free(uwb_range_msg.anchor_ids.data);
      free(uwb_range_msg.range_values.data);
#endif

      // UWB Localization
#ifdef UWB_LOCALIZATION_PUB_ENABLED

      trilat3D_4A(received_anchor_distances, tag_position, tag_distance_rmse);

      tag_position_msg.x = tag_position[0];
      tag_position_msg.y = tag_position[1];
      tag_position_msg.z = tag_position[2];

#ifdef DEBUG_PRINT_TAG_POSITION
      // Log the tag position
      LOG_INFO("Tag Position: ", String(tag_position_msg.x) + ", " + String(tag_position_msg.y) + ", " + String(tag_position_msg.z));
#endif
    
      // Publish the tag position
      RCSOFTCHECK(rcl_publish(&rcl_tag_position_publisher, &tag_position_msg, NULL));
#endif
    }
    
    // Spin the executor to handle callbacks
    // RCSOFTCHECK(rclc_executor_spin_some(&rcl_rangepub_executor, RCL_MS_TO_NS(100)));
    // delay(1);
  }
}


