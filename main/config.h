#ifndef CONFIG_H
#define CONFIG_H

#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
//192.168.4.1/


#define ESP_WIFI_SSID "carlsjr"
#define ESP_WIFI_PASS "password123"
#define ESP_WIFI_CHANNEL 1
#define MAX_STA_CONN 2
static const char* TAG = "CarModule";

#define SENSOR_FREQ 72      // sensor readings per rotation
#define SENSOR_PERIOD 4000  // ms per full rotation of sensor
#define SPIKE_THRESHOLD 40 // mm between points is considered a spike
#define DISTANCE_SIMILARITY_THRESHOLD 25 // landmarks must be within 10mm of their pair
#define MIN_SENSOR_INTERVAL 0 // minimum ms between sensor readings
#define MAP_SIZE 250 // 500x500 cm^2 map
#define MAP_RATIO 15 // cell represents this number mm^2
#define MOVE_TIME_PER_STEP 500 // 100 ms of wheel spinning inbetween slam steps
#define MOVE_SPEED 25 // wheel turns at 25% power
#define POINT_NEIGHBORHOOD_SIZE 1 // number of relevant points on each side of current point to calculate normal
#define CORRESP_NORMAL_SIMILARITY 4 // normals of a corresponding point in old space to its pair in new space have to be at most this degrees apart
#define MAX_DISTANCE_PER_ITERATION 175 // maximum distance (mm) a corresponding point can be apart from its pair
#define MAXIMUM_UNCERTAINTY_INVERVAL 0.1 // golden section minimization will stop once angle uncertainty is within 0.1 degrees
#define MAXIMUM_RRT_ITERATIONS 500 // maximum number of nodes RRT will try to create
#define COARSE_RATIO 5 // each coarse occupancy grid cell represents this many normal map cells squared
#define PLANNING_ROTATION_TOLERANCE 5.0 // if rotation at least this similar to edge then drive straight
#define MAX_ROT_PER_STEP 15.0 // rotate maximum of this number of degrees for each SLAM step
#define MAX_DIST_PER_STEP 20.0 // maximum distance robot (in mm) can move for each SLAM step
#define DEADRECKON_T_WEIGHT 0.95 // weight of deadreckoning estimation vs icp in translation computation
#define DEADRECKON_ROT_WEIGHT 0.0 // weight of deadreckoning estimation vs icp in rotation computation
#define SENSOR_OFFSET_FROM_PIVOT 70.0 // mm distance between sensor and pivot point between rear wheels
#endif