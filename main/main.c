#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gptimer.h"
#include "nvs_flash.h"
#include "string.h"
#include "driver/i2c.h"
#include "VL53L4CD_api.h"
#include "math.h"
#include "wifi.h"
#include "config.h"

int iterations = 0;

const Dev_t sensor = 0x29;
uint8_t map[MAP_SIZE][MAP_SIZE] = {0};
uint8_t map_tree[MAP_SIZE][MAP_SIZE] = {0};
bool coarse_map[MAP_SIZE / COARSE_RATIO][MAP_SIZE / COARSE_RATIO] = {false};
bool coarse_tree_map[MAP_SIZE / COARSE_RATIO][MAP_SIZE / COARSE_RATIO] = {false};
int coarse_indices[(MAP_SIZE / COARSE_RATIO) * (MAP_SIZE / COARSE_RATIO)];

volatile bool slam_restart = true, manual_left = false, manual_right = false, manual_forward = false;
#define max_coarse_index_length ((MAP_SIZE / COARSE_RATIO) * (MAP_SIZE / COARSE_RATIO))
int random_values[max_coarse_index_length];
    

typedef struct RRT_node {
    int x;
    int y;
    struct RRT_node * parent;
    struct RRT_node ** children;
    size_t child_cnt;
    size_t child_cap;
} RRT_node;

void draw_RRT_on_map(RRT_node *);
RRT_node * compute_RRT(double *);
void free_RRT(RRT_node *);
RRT_node * find_nearest_RRT_node(RRT_node *, int, int);


RRT_node * RRT_traversal_queue[MAXIMUM_RRT_ITERATIONS] = {NULL};

// direction: 0=CCW (forward), 1=CW (backward), PWMspeed is 0-100 value for % of full pwoer
void changeSpeedA(int direction, int PWMspeed) {
	if (PWMspeed >= 100) PWMspeed = 99;
    gpio_set_level(GPIO_NUM_1, 1); // stby

    // control direction
    if (direction) { // forward
        gpio_set_level(GPIO_NUM_2, 0); // ain1
        gpio_set_level(GPIO_NUM_3, 1); // ain2
    } else {
        gpio_set_level(GPIO_NUM_2, 1);
        gpio_set_level(GPIO_NUM_3, 0);
    }

    if (PWMspeed == 0) {
        gpio_set_level(GPIO_NUM_2, 0);
        gpio_set_level(GPIO_NUM_3, 0);
    }

    // pwm
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, ((1 << 10) - 1) * PWMspeed / 100);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void changeSpeedB(int direction, int PWMspeed) {
	if (PWMspeed >= 100) PWMspeed = 99;
    gpio_set_level(GPIO_NUM_1, 1);

    if (direction) {
        gpio_set_level(GPIO_NUM_7, 0);
        gpio_set_level(GPIO_NUM_23, 1);
    } else {
        gpio_set_level(GPIO_NUM_7, 1);
        gpio_set_level(GPIO_NUM_23, 0);
    }
    if (PWMspeed == 0) {
        gpio_set_level(GPIO_NUM_7, 0);
        gpio_set_level(GPIO_NUM_23, 0);
    }

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, ((1 << 10) - 1) * PWMspeed / 100);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

void i2c_master_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };

    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

void recenter_servo() {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, (uint32_t)((1500.0 / 20000.0) * ((1 << 10) - 1)));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    vTaskDelay(pdMS_TO_TICKS(500));
}



void fill_map_from_points_x_y(double points_x_y[], size_t points_len) {
    for (size_t i = 0; i < points_len; ++i) {
        double x = points_x_y[2*i], y = points_x_y[2*i+1];
        if (fabs(x) < 5.0 || fabs(y) < 5.0)
            continue; // invalid point
        int j = -(int)(y / MAP_RATIO) + MAP_SIZE / 2;
        int k = -(int)(x / MAP_RATIO) + MAP_SIZE / 2;
        if (j >= 0 && j < MAP_SIZE && k >= 0 && k < MAP_SIZE) {
            map[j][k] = iterations+1;
            map_tree[j][k] = iterations+1; // also draw on tree version of map to update when tree already drawn
        }
    }
}

// 0
void collect_range_scan(uint16_t points[], int freq, int dur) {
    int i = 0;
    int delay_ms = (double)dur / (double)freq;

    //ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, (uint32_t)((1500.0 / 20000.0) * ((1 << 10) - 1))); // TEMPORARY WHILE BATTERY DEAD    
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, (uint32_t)((500.0 / 20000.0) * ((1 << 10) - 1)));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    vTaskDelay(pdMS_TO_TICKS(500));
    while (i < freq) {
        // set PWM
        double angle = (double)i / (double)(freq-1) * 180.0 - 90.0;
        //angle = 0.0; // TEMPORARY WHILE BATTERY DEAD
        //printf("%f\n", angle);
        double pulse_microseconds = 1500.0 + (angle/90.0) * 1000.0;
        // ((1 << 10) - 1) is 1023 10-bit max value, 50hz is 20000 microseconds
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, (uint32_t)((pulse_microseconds / 20000.0) * ((1 << 10) - 1)));
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);

        // read from sensor
        uint8_t data_ready = 0;
        VL53L4CD_ResultsData_t result;
        VL53L4CD_CheckForDataReady(sensor, &data_ready);
        if (data_ready) {
            uint16_t distance = 0;
            VL53L4CD_GetResult(sensor, &result);
            if (result.range_status == 0) { 
                distance = result.distance_mm;
                if (distance < 20) distance = 0;
                //fill_map_from_point(distance, i); // TO DISPLAY POINTS DIRECTLY UNCOMMENT THIS AND COMMENT FILL_MAP_FROM IN COMPARE_LANDMARKS
            }
            else
                distance = 0;
            points[i] = distance;
            VL53L4CD_ClearInterrupt(sensor);

        }

        vTaskDelay(pdMS_TO_TICKS(delay_ms > MIN_SENSOR_INTERVAL ? delay_ms : MIN_SENSOR_INTERVAL));
        ++i;
    } 

    // discard outliers based on distance disconinuities
    i = 1;
    while (i < freq) {
        if (i < freq-1 && points[i-1] == 0 && points[i+1] == 0) // no neighbors
            points[i] = 0;
        if (points[i-1] != 0 && abs(points[i] - points[i-1]) >= SPIKE_THRESHOLD) // dist to last
            points[i-1] = 0;
        ++i;
    }
    
    if (points[1] == 0) points[0] = 0; // no neighbor discard for first and last points in scan
    if (points[freq-2] == 0) points[freq-1] = 0;
    
    recenter_servo();
}

void matrix_mult_3x3_and_3x1(double a[3][3], double b[3]) { // 3x3 matrices, a times b, output saved in b
    double res[3] = {0};
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            res[i] += a[i][j] * b[j];
        }
    }

    for (size_t i = 0; i < 3; ++i) {
        b[i] = res[i];
    }
}

void convert_point_to_2d(uint16_t dist, int i, double * x, double * y) {
    double angle = ((double)i / (SENSOR_FREQ - 1) * 180.0 - 90.0) * M_PI/180.0;
    *x = dist * sin(angle);
    *y = dist * cos(angle);
}

// 1
void transform_points(uint16_t points[], double points_x_y[], int points_len, double trans[], double rot) {
    double rotation_matrix[3][3] = {{cos(rot * M_PI / 180.0),-sin(rot * M_PI / 180.0),0},
                                    {sin(rot * M_PI / 180.0),cos(rot * M_PI / 180.0),0},
                                    {0,0,1}};
    double translation_matrix[3][3] = {{1,0,trans[0]},
                                       {0,1,trans[1]},
                                       {0,0,1}};
    for (size_t i = 0; i < points_len; ++i) {
        if (points[i] == 0) { // don't transform invalid point
            points_x_y[2*i] = 0.0;
            points_x_y[2*i+1] = 0.0;
            continue;
        }

        double x, y = 0;
        convert_point_to_2d(points[i], i, &x, &y);
        double point_matrix[3] = {x,y,1};
        matrix_mult_3x3_and_3x1(rotation_matrix, point_matrix);
        matrix_mult_3x3_and_3x1(translation_matrix, point_matrix);
        points_x_y[i*2] = point_matrix[0];
        points_x_y[i*2+1] = point_matrix[1];
    }

    
    // discard outliers (make 0,0 which is impossible to achieve otherwise)
    double prev_angle = 0.0;
    bool found_first_valid = false;
    size_t first_valid_i = 0;
    for (size_t i = 0; i < points_len; ++i) {
        if (points_x_y[2*i] == 0.0 && points_x_y[2*i+1] == 0.0) continue;
        prev_angle = atan2(points_x_y[2*i+1], points_x_y[2*i]);
        found_first_valid = true;
        first_valid_i = i;
        break;
    }
    if (!found_first_valid) return;

    for (size_t i = first_valid_i+1; i < points_len; ++i) {
        if (points_x_y[2*i] == 0.0 || points_x_y[2*i+1] == 0.0) // invalid point
            continue;
        // discard any points that are now out of consecutive rotational order -> surface direction changed so they are not occluded
        double cur_angle = atan2(points_x_y[2*i+1], points_x_y[2*i]);
        if (isnan(cur_angle))
            continue;
        //printf("Cur angle: %f\tPrev Angle: %f\n", cur_angle, prev_angle);
        if (cur_angle > prev_angle) {
            //printf("DISCARDED!\n");
            points_x_y[2*i+1] = 0.0;
            points_x_y[2*i] = 0.0;
        }
        prev_angle = cur_angle;
    }
    

} 

void get_normal_from_tangent(double * normal_x, double * normal_y, double points_x_y[], int points_len, int left_i, int right_i) {
    double tangent_x = 0, tangent_y = 0;
    int count = 0;
    for (int j = 0; j < POINT_NEIGHBORHOOD_SIZE; ++j) {
        int neighbor_i_1 = left_i-j;
        int neighbor_i_2 = right_i+j;
        if (neighbor_i_1 >= 0 && neighbor_i_2 < points_len) {
            tangent_x += points_x_y[neighbor_i_2*2] - points_x_y[neighbor_i_1*2];
            tangent_y += points_x_y[neighbor_i_2*2+1] - points_x_y[neighbor_i_1*2+1];
            ++count;
        }
    }
    
    double magnitude = sqrt(pow(tangent_x, 2) + pow(tangent_y, 2));
    if (count == 0 || magnitude < 1e-9) {
        *normal_x = 0;
        *normal_y = 0;
        return;
    }
    double normalized_tangent_x = tangent_x / magnitude;
    double normalized_tangent_y = tangent_y / magnitude; 

    *normal_x = normalized_tangent_y;
    *normal_y = -normalized_tangent_x; 
}

// 2
void compute_correspondence_pairs(double points_x_y[], size_t points_len, double old_points_x_y[], double corresp_points_x_y[], double g_angle, int * num_pairs, int * num_outliers, double combined_x_y_and_pair_dists[]) {
    for (size_t i = 0; i < points_len; ++i) {
        if (points_x_y[2*i] == 0.0 && points_x_y[2*i+1] == 0.0) continue;
        double x1 = points_x_y[2*i];
        double y1 = points_x_y[2*i+1];
        double angle = atan2(points_x_y[2*i+1], points_x_y[2*i]) * 180/M_PI + g_angle;

        size_t old_i_less_than_angle = 0;
        double old_angle_less_than_angle = -DBL_MAX;
        size_t old_i_greater_than_angle = 0;
        double old_angle_greater_than_angle = DBL_MAX;
        bool found_angle_less = false, found_angle_greater = true;
        for (size_t j = 0; j < points_len; ++j) {
            if (old_points_x_y[2*j] == 0.0 && old_points_x_y[2*j+1] == 0.0) continue;
            double angle_old_space = atan2(old_points_x_y[2*j+1], old_points_x_y[2*j]) * 180/M_PI;
            if (angle_old_space < angle && angle_old_space > old_angle_less_than_angle) {
                old_angle_less_than_angle = angle_old_space;
                old_i_less_than_angle = j;
                found_angle_less = true;
            } else if (angle_old_space > angle && angle_old_space < old_angle_greater_than_angle) {
                old_angle_greater_than_angle = angle_old_space;
                old_i_greater_than_angle = j;
                found_angle_greater = true;
            }
            //printf("angle old: %f, angle calculated: %f\n", angle_old_space, angle);
        }

        if (found_angle_less && found_angle_greater) {
            double t = (angle - old_angle_less_than_angle) / (old_angle_greater_than_angle - old_angle_less_than_angle);
            double corresp_x = old_points_x_y[2*old_i_less_than_angle] + t * (old_points_x_y[2*old_i_greater_than_angle] - old_points_x_y[2*old_i_less_than_angle]);
            double corresp_y = old_points_x_y[2*old_i_less_than_angle+1] + t * (old_points_x_y[2*old_i_greater_than_angle+1] - old_points_x_y[2*old_i_less_than_angle+1]);


            // now do check to see if will discard this point, if don't discard increment corresp_points_size and append to corresp_points_x_y, eventually return this size
            bool is_outlier = false;


            // (Rωn1) · n* > cos α
            //     (Rωn1)
            double normal_x1, normal_y1;
            int left_i = (i > 0) ? (int)i-1 : 0;
            int right_i = (i < points_len-1) ? (int)i+1 : (int)points_len-1;
            get_normal_from_tangent(&normal_x1, &normal_y1, points_x_y, points_len, left_i, right_i);
            double rot_normal_x1 = cos(g_angle * M_PI/180) * normal_x1 - sin(g_angle * M_PI/180) * normal_y1;
            double rot_normal_y1 = sin(g_angle * M_PI/180) * normal_x1 + cos(g_angle * M_PI/180) * normal_y1;
            

            //     n*
            double n_left_x, n_left_y, n_right_x, n_right_y;
            int old_less_left_i = (old_i_less_than_angle > 0) ? (int)old_i_less_than_angle-1 : 0;
            int old_less_right_i = (old_i_less_than_angle < points_len-1) ? (int)old_i_less_than_angle+1 : (int)points_len-1;
            int old_greater_left_i = (old_i_greater_than_angle > 0) ? (int)old_i_greater_than_angle-1 : 0;
            int old_greater_right_i = (old_i_greater_than_angle < points_len-1) ? (int)old_i_greater_than_angle+1 : (int)points_len-1;
            get_normal_from_tangent(&n_left_x, &n_left_y, old_points_x_y, points_len, old_less_left_i, old_less_right_i);
            get_normal_from_tangent(&n_right_x, &n_right_y, old_points_x_y, points_len, old_greater_left_i, old_greater_right_i);
            double normal_corresp_x = n_left_x + t * (n_right_x - n_left_x);
            double normal_corresp_y = n_left_y + t * (n_right_y - n_left_y);
            double normal_corresp_mag = sqrt(pow(normal_corresp_x, 2) + pow(normal_corresp_y, 2));
            if (normal_corresp_mag > 1e-9) {
                normal_corresp_x /= normal_corresp_mag;
                normal_corresp_y /= normal_corresp_mag;
            }

            //     (Rωn1) · n* > cos α
            double dot_rot_normal_and_corresp_normal = rot_normal_x1 * normal_corresp_x  + rot_normal_y1 * normal_corresp_y;
            if (dot_rot_normal_and_corresp_normal < cos(CORRESP_NORMAL_SIMILARITY * M_PI/180)) // outlier because normals directions weren't close enough
                is_outlier = true;


            // D = (Rωn1 + n*) · (P* - RωP1)
            //     (Rωn1 + n*)
            double rot_normal_x1_plus_corresp_x = rot_normal_x1 + normal_corresp_x;
            double rot_normal_y1_plus_corresp_y = rot_normal_y1 + normal_corresp_y;

            //     (P* - RωP1)
            double rot_x1 = cos(g_angle * M_PI/180) * x1 - sin(g_angle * M_PI/180) * y1;
            double rot_y1 = sin(g_angle * M_PI/180) * x1 + cos(g_angle * M_PI/180) * y1;
            double corresp_x_minus_rot_x1 = corresp_x - rot_x1;
            double corresp_y_minus_rot_y1 = corresp_y - rot_y1;

            //     D = (Rωn1 + n*) · (P* - RωP1)
            double dot_normals_and_points = rot_normal_x1_plus_corresp_x * corresp_x_minus_rot_x1 + rot_normal_y1_plus_corresp_y * corresp_y_minus_rot_y1;
            if (fabs(dot_normals_and_points) > MAX_DISTANCE_PER_ITERATION) // outlier because distance too far
                is_outlier = true;


            // otherwise add corresp_point
            if (!is_outlier) {
                corresp_points_x_y[i*2] = corresp_x;
                corresp_points_x_y[i*2+1] = corresp_y;
                ++(*num_pairs);

                combined_x_y_and_pair_dists[i*3] = rot_normal_x1_plus_corresp_x;
                combined_x_y_and_pair_dists[i*3+1] = rot_normal_y1_plus_corresp_y;
                combined_x_y_and_pair_dists[i*3+2] = dot_normals_and_points;
            } else {
                ++(*num_outliers);
            }
        }
    }
}

// 3
double compute_total_matching_distance(double points_x_y[], double corresp_points_x_y[], size_t points_len, int num_pairs, int num_outliers, double combined_x_y_and_pair_dists[], double * found_Tx, double * found_Ty) {
    if (num_pairs + num_outliers == 0) return DBL_MAX; // safety against div by 0

    
    double ATA[2][2] = {0};
    double ATb[2] = {0};

    // -> solve E(ω,T) = sum(Cxi Tx + Cyi Ty = Di) for optimal T using least squares (A^T)A * T = (A^T)b
    for (size_t i = 0; i < points_len; ++i) {
        if (points_x_y[2*i] == 0.0 || points_x_y[2*i+1] == 0.0 || corresp_points_x_y[2*i] == 0.0 || corresp_points_x_y[2*i+1] == 0.0) {
            continue; // invalid point or corresp_point
        } else {
            double combined_x = combined_x_y_and_pair_dists[i*3];
            double combined_y = combined_x_y_and_pair_dists[i*3+1];
            double d = combined_x_y_and_pair_dists[i*3+2];

            ATA[0][0] += pow(combined_x, 2);
            ATA[0][1] += combined_x * combined_y;
            ATA[1][0] += combined_y * combined_x;
            ATA[1][1] += pow(combined_y, 2);
            ATb[0] += combined_x * d;
            ATb[1] += combined_y * d;
        }
    }

    double det = ATA[0][0]*ATA[1][1] - ATA[0][1]*ATA[1][0];
    double Tx = (ATA[1][1]*ATb[0] - ATA[0][1]*ATb[1]) / det;
    double Ty = (ATA[0][0]*ATb[1] - ATA[1][0]*ATb[0]) / det;

    // sum of squared residual for each pair (distance between points in the pair), 
    double sum_squared_residuals = 0.0;
    for (size_t i = 0; i < points_len; ++i) {
        if (points_x_y[2*i] == 0.0 || points_x_y[2*i+1] == 0.0 || corresp_points_x_y[2*i] == 0.0 || corresp_points_x_y[2*i+1] == 0.0) {
            continue; // invalid point or corresp_point
        } else {
            double combined_x = combined_x_y_and_pair_dists[i*3];
            double combined_y = combined_x_y_and_pair_dists[i*3+1];
            double d = combined_x_y_and_pair_dists[i*3+2];
            double residual = combined_x*Tx + combined_y*Ty - d;
            sum_squared_residuals += pow(residual, 2);
        }
    }

    *found_Tx = Tx;
    *found_Ty = Ty;
    // total matching distance: match(ω) = 1/(np + no)(min_T(E(ω,T)) + no*(H_d)^2)
    return 1.0 / (num_pairs + num_outliers) * (sum_squared_residuals + num_outliers * pow(MAX_DISTANCE_PER_ITERATION, 2));
}

typedef struct RangeScanNode {
    double * points_x_y; // points in space of first iteration
    struct RangeScanNode * next;
    struct RangeScanNode * prev;
} RangeScanNode;

RangeScanNode* create_range_scan_node() {
    RangeScanNode* new_node = (RangeScanNode*)malloc(sizeof(RangeScanNode));
    new_node->points_x_y = (double*)malloc(SENSOR_FREQ * sizeof(double) * 2);
    new_node->next = NULL;
    new_node->prev = NULL;
    return new_node;
}

double gs_search_for_angle(double gs_lower_bound, double gs_upper_bound, double * optimal_rot, double local_points_x_y[], double prev_points_x_y[], double corresp_points_x_y[], double combined_x_y_and_pair_dists[]) {
    double r = (sqrt(5.0) - 1.0) / 2.0;
    double delta_Tx_1 = 0.0, delta_Ty_1 = 0.0, delta_Tx_2 = 0.0, delta_Ty_2 = 0.0;
    int num_pairs = 0, num_outliers = 0;
    while (fabs(gs_upper_bound - gs_lower_bound) > MAXIMUM_UNCERTAINTY_INVERVAL) {
        double x1 = gs_upper_bound - r * (gs_upper_bound - gs_lower_bound);
        double x2 = gs_lower_bound + r * (gs_upper_bound - gs_lower_bound);

        double test_rot = x1;

        memset(corresp_points_x_y, 0, SENSOR_FREQ * sizeof(double) * 2);
        memset(combined_x_y_and_pair_dists, 0, SENSOR_FREQ * sizeof(double) * 3);
        num_pairs = 0; num_outliers = 0;
        compute_correspondence_pairs(local_points_x_y, SENSOR_FREQ, prev_points_x_y, corresp_points_x_y, test_rot, &num_pairs, &num_outliers, combined_x_y_and_pair_dists);
        double tmd_1 = compute_total_matching_distance(local_points_x_y, corresp_points_x_y, SENSOR_FREQ, num_pairs, num_outliers, combined_x_y_and_pair_dists, &delta_Tx_1, &delta_Ty_1);


        test_rot = x2;

        memset(corresp_points_x_y, 0, SENSOR_FREQ * sizeof(double) * 2);
        memset(combined_x_y_and_pair_dists, 0, SENSOR_FREQ * sizeof(double) * 3);
        num_pairs = 0; num_outliers = 0;
        compute_correspondence_pairs(local_points_x_y, SENSOR_FREQ, prev_points_x_y, corresp_points_x_y, test_rot, &num_pairs, &num_outliers, combined_x_y_and_pair_dists);
        double tmd_2 = compute_total_matching_distance(local_points_x_y, corresp_points_x_y, SENSOR_FREQ, num_pairs, num_outliers, combined_x_y_and_pair_dists, &delta_Tx_2, &delta_Ty_2);
        
        
        if (tmd_1 < tmd_2) {
            gs_upper_bound = x2;
        } else {
            gs_lower_bound = x1;
        }
    }

    *optimal_rot = (gs_lower_bound + gs_upper_bound) / 2.0;
    double final_Tx = 0.0, final_Ty = 0.0;
    memset(corresp_points_x_y, 0, SENSOR_FREQ * sizeof(double) * 2);
    memset(combined_x_y_and_pair_dists, 0, SENSOR_FREQ * sizeof(double) * 3);
    num_pairs = 0; num_outliers = 0;
    compute_correspondence_pairs(local_points_x_y, SENSOR_FREQ, prev_points_x_y, corresp_points_x_y, *optimal_rot, &num_pairs, &num_outliers, combined_x_y_and_pair_dists);
    return compute_total_matching_distance(local_points_x_y, corresp_points_x_y, SENSOR_FREQ, num_pairs, num_outliers, combined_x_y_and_pair_dists, &delta_Tx_2, &delta_Ty_2);
}

RangeScanNode * SLAM_iteration(RangeScanNode * prev, double g_trans[], double * g_rot) {
    uint16_t *points = malloc(SENSOR_FREQ * sizeof(uint16_t)); // polar form: magnitude as value, angle implicit from index
    double *local_points_x_y = malloc(SENSOR_FREQ * sizeof(double) * 2);
    double *points_x_y = malloc(SENSOR_FREQ * sizeof(double) * 2);
    double *corresp_points_x_y = malloc(SENSOR_FREQ * sizeof(double) * 2);
    double *combined_x_y_and_pair_dists = malloc(SENSOR_FREQ * sizeof(double) * 3);
    int num_pairs = 0, num_outliers = 0;
    double gs_lower_bound = -30.0, gs_upper_bound = 30.0;

    printf("iteration: %d\n", iterations);
    collect_range_scan(points, SENSOR_FREQ, SENSOR_PERIOD);    
    double trans_zero[2] = {0.0, 0.0};
    transform_points(points, local_points_x_y, SENSOR_FREQ, trans_zero, 0.0);
    if (iterations > 0) {
        // use multi-hypothesis to find best angle to center golden section search around
        double best_tmd = DBL_MAX;
        double best_rot = 0.0;
        int num_coarse = 9;  // try 9 angles: -40, -30, -20, -10, 0, 10, 20, 30, 40
        double delta_Tx_tmp = 0, delta_Ty_tmp = 0;
        for (int s = 0; s < num_coarse; ++s) {
            double test_rot = -40.0 + s * (80.0 / (num_coarse - 1));
            memset(corresp_points_x_y, 0, SENSOR_FREQ * sizeof(double) * 2);
            memset(combined_x_y_and_pair_dists, 0, SENSOR_FREQ * sizeof(double) * 3);
            num_pairs = 0; num_outliers = 0;
            compute_correspondence_pairs(local_points_x_y, SENSOR_FREQ, prev->points_x_y, corresp_points_x_y, test_rot, &num_pairs, &num_outliers, combined_x_y_and_pair_dists);
            double tmd = compute_total_matching_distance(local_points_x_y, corresp_points_x_y, SENSOR_FREQ, num_pairs, num_outliers, combined_x_y_and_pair_dists, &delta_Tx_tmp, &delta_Ty_tmp);
            if (tmd < best_tmd) {
                best_tmd = tmd;
                best_rot = test_rot;
            }
        }

        // golden section search to find ω that minimizes total_matching_distance
        double refine_range = 10.0;  // search +-10 degrees around best coarse angle
        double optimal_rot = 0.0;
        gs_search_for_angle(best_rot - refine_range, best_rot + refine_range, &optimal_rot, local_points_x_y, prev->points_x_y, corresp_points_x_y, combined_x_y_and_pair_dists);

        // compute T that minimizes error given found ω using point-to-point ICP: T = mean(P*) - mean(Rω * P1)
        double final_Tx = 0.0, final_Ty = 0.0;
        memset(corresp_points_x_y, 0, SENSOR_FREQ * sizeof(double) * 2);
        memset(combined_x_y_and_pair_dists, 0, SENSOR_FREQ * sizeof(double) * 3);
        num_pairs = 0; num_outliers = 0;
        compute_correspondence_pairs(local_points_x_y, SENSOR_FREQ, prev->points_x_y, corresp_points_x_y, optimal_rot, &num_pairs, &num_outliers, combined_x_y_and_pair_dists);
        double sum_corresp_x = 0, sum_corresp_y = 0;
        double sum_rot_x = 0, sum_rot_y = 0;
        int count = 0;
        for (size_t i = 0; i < SENSOR_FREQ; ++i) {
            if (local_points_x_y[2*i] == 0.0 || local_points_x_y[2*i+1] == 0.0 || corresp_points_x_y[2*i] == 0.0 || corresp_points_x_y[2*i+1] == 0.0)
                continue;
            double rot_x = cos(optimal_rot * M_PI/180) * local_points_x_y[2*i] - sin(optimal_rot * M_PI/180) * local_points_x_y[2*i+1];
            double rot_y = sin(optimal_rot * M_PI/180) * local_points_x_y[2*i] + cos(optimal_rot * M_PI/180) * local_points_x_y[2*i+1];
            sum_corresp_x += corresp_points_x_y[2*i];
            sum_corresp_y += corresp_points_x_y[2*i+1];
            sum_rot_x += rot_x;
            sum_rot_y += rot_y;
            ++count;
        }
        if (count > 0) {
            final_Tx = (sum_corresp_x - sum_rot_x) / count;
            final_Ty = (sum_corresp_y - sum_rot_y) / count;
        }

        // update the global trans and rot variables with found ω and T
        *g_rot += optimal_rot;
        g_trans[0] += final_Tx;
        g_trans[1] += final_Ty;

        printf("Parameters found:\n\tROT: %f\n\tTRANS: [%f, %f]\n", optimal_rot, final_Tx, final_Ty);
    }

    // transform points to global space according to the new parameters
    transform_points(points, points_x_y, SENSOR_FREQ, g_trans, *g_rot);
    fill_map_from_points_x_y(points_x_y, SENSOR_FREQ);
    
    RangeScanNode* new_node = create_range_scan_node();
    memcpy(new_node->points_x_y, local_points_x_y, SENSOR_FREQ * sizeof(double) * 2); // save points in local space in new node
    new_node->prev = prev;
    prev->next = new_node;
    
   
    free(local_points_x_y);
    free(combined_x_y_and_pair_dists);
    free(corresp_points_x_y);
    free(points_x_y);
    free(points);
    return new_node;
}

void motor_rotate(double rot_needed) { // rotate rot_needed degrees
    rot_needed = fabs(rot_needed) <= MAX_ROT_PER_STEP ? rot_needed : (rot_needed > 0 ? MAX_ROT_PER_STEP : -MAX_ROT_PER_STEP);

    if (rot_needed < 0) {
        changeSpeedA(0, MOVE_SPEED);
        changeSpeedB(1, MOVE_SPEED);
    } else {
        changeSpeedA(1, MOVE_SPEED);
        changeSpeedB(0, MOVE_SPEED);
    }
    vTaskDelay(pdMS_TO_TICKS(500.0 * fabs(rot_needed) / 20.0)); // temporary hardcoded estimate formula
    changeSpeedA(0, 0);
    changeSpeedB(0, 0);
    
    printf("rot step %f degrees turned\n", rot_needed);
}

void motor_straight(double dist_needed) { // rotate rot_needed degrees
    dist_needed = dist_needed <= MAX_DIST_PER_STEP ? dist_needed : MAX_DIST_PER_STEP;

    changeSpeedA(0, MOVE_SPEED);
    changeSpeedB(0, MOVE_SPEED);
    vTaskDelay(pdMS_TO_TICKS(500.0 * fabs(dist_needed) / 13.0)); // temporary hardcoded estimate formula
    changeSpeedA(0, 0);
    changeSpeedB(0, 0);
    
    printf("dist step %f units moved\n", dist_needed);
}

RRT_node * next_RRT_node_in_path(RRT_node * root, double goal_pos[]) {
    RRT_node * cur = find_nearest_RRT_node(root, goal_pos[0], goal_pos[1]);
    
    while (cur->parent != NULL && cur->parent->parent != NULL) {
        cur = cur->parent;
    }

    return cur;    
}

typedef struct AStarNode {
    int i;
    double f; // g + h
    bool open;
} AStarNode;

AStarNode create_A_star_node(int i, double f, bool open, int parent) {
    AStarNode node;
    node.i = i; 
    node.f = f; 
    node.open = open;
    return node; 
}

double euclidean_flat_dist(double a_x, double a_y, double b_x, double b_y) {
    return sqrt(pow(b_x-a_x, 2) +  pow(b_y-a_y, 2));
}

int find_min_f(AStarNode * open, int len) {
    double min_f = DBL_MAX;
    int min_f_i = 0;
    for (int i = 0; i < len; ++i) {
        if (open[i].f < min_f && open[i].open) {
            min_f_i = i;
            min_f = open[i].f;
        }
    }
    return min_f_i;
}

void next_coarse_path_node(double start_pos[], double goal_pos[], int next_coarse_pos[]) {
    const int coarse_map_width = (MAP_SIZE / COARSE_RATIO);
    int coarse_start_pos[2] = {start_pos[0] / COARSE_RATIO, start_pos[1] / COARSE_RATIO};
    int coarse_goal_pos[2] = {goal_pos[0] / COARSE_RATIO, goal_pos[1] / COARSE_RATIO};
    int coarse_goal = (goal_pos[1] / COARSE_RATIO) * coarse_map_width + (goal_pos[0] / COARSE_RATIO);
    int total_coarse_cells = coarse_map_width * coarse_map_width;

    // do A* on cells in coarse map to find goal
    AStarNode * open = malloc(sizeof(AStarNode) * total_coarse_cells);
    int * parent = malloc(sizeof(int) * total_coarse_cells);
    double * g_score = malloc(sizeof(double) * total_coarse_cells);
    for (int i = 0; i < total_coarse_cells; ++i) {
        open[i].open = false;
        open[i].f = DBL_MAX;
        open[i].i = i;

        parent[i] = -1;
        g_score[i] = DBL_MAX;
    }

    int coarse_start = coarse_start_pos[1] * coarse_map_width + coarse_start_pos[0];
    open[coarse_start].f = euclidean_flat_dist(coarse_start_pos[0], coarse_start_pos[1], coarse_goal_pos[0], coarse_goal_pos[1]);
    open[coarse_start].open = true;
    g_score[coarse_start] = 0.0;


    while (true) {
        int cur = find_min_f(open, total_coarse_cells);
        if (!open[cur].open) break; // no open nodes left
        open[cur].open = false;
        if (cur == coarse_goal) { // found goal
            int next = cur;
            while (parent[cur] != -1) { // trace path to parent
                next = cur;
                cur = parent[cur];
            }
            next_coarse_pos[0] = next % coarse_map_width; // x
            next_coarse_pos[1] = next / coarse_map_width; // y

            free(open); 
            free(g_score);
            free(parent);
            return;
        }

        int cur_y = cur / coarse_map_width, cur_x = cur % coarse_map_width; 
        for (int i = -1; i <= 1; ++i) { // add each neighbor to queue
            for (int j = -1; j <= 1; ++j) {
                if (i == 0 && j == 0) continue; // skip self
                int neighbor_y = cur_y + i, neighbor_x = cur_x + j;
                if (neighbor_y < 0 || neighbor_x < 0 || neighbor_y >= coarse_map_width || neighbor_x >= coarse_map_width
                    || coarse_map[neighbor_y][neighbor_x]) continue; // skip if neighbor OOB or obstacle
                int neighbor = neighbor_y * coarse_map_width + neighbor_x;
                if (!open[neighbor].open && open[neighbor].f != DBL_MAX) continue; // neighbor already closed
                
                double direct_neighbor_cost = (i != 0 && j != 0) ? 1.414214 : 1.0; // if on diagonal cost is sqrt(2) otherwise 1
                double g = g_score[cur] + direct_neighbor_cost;
                if (g < g_score[neighbor]) {
                    g_score[neighbor] = g;
                    open[neighbor].f = g + euclidean_flat_dist(neighbor_x, neighbor_y, coarse_goal_pos[0], coarse_goal_pos[1]);
                    open[neighbor].open = true;
                    parent[neighbor] = cur;
                }
            }
        }
    }
    
    // no path to goal found
    next_coarse_pos[0] = coarse_start_pos[0];
    next_coarse_pos[1] = coarse_start_pos[1];
    free(open); 
    free(g_score);
    free(parent);
}

void SLAM_run() {
    RangeScanNode * head = create_range_scan_node();
    RangeScanNode * prev = head;
    double global_trans[2] = {0};
    double global_rot = 0;
    double goal_pos[2] = {125, 30}; // temporary hardcoded goal

    // generate rand values to be reused to improve RRT generation consistency
    srand(10);
    for (size_t i = 0; i < max_coarse_index_length; ++i)
        random_values[i] = rand();
    
    // first PLANNING rrt from cur position, if rotation to next edge point is within range
    // then MOVING, otherwise ROTATING until it is, then MOVING for one step
    enum {PLANNING, ROTATING, MOVING};
    int state = PLANNING;
    int next_coarse_pos[2] = {};
    double next_world_pos[2] = {};
    double target_rot = 0;
    double target_dist = 0;

    while (iterations < 100) {
        prev = SLAM_iteration(prev, global_trans, &global_rot);
        ++iterations;

        if (state == PLANNING) {
            double cur_map_pos[2] = {
                -(global_trans[0] / MAP_RATIO) + MAP_SIZE / 2, -(global_trans[1] / MAP_RATIO) + MAP_SIZE / 2
            };
            double goal_map_pos[2] = {
                -(goal_pos[0] / MAP_RATIO) + MAP_SIZE / 2, -(goal_pos[1] / MAP_RATIO) + MAP_SIZE / 2
            };

            /*
            RRT_node * RRT_root = compute_RRT(cur_map_pos);
            draw_RRT_on_map(RRT_root);

            // find path in RRT
            RRT_node * next_node = next_RRT_node_in_path(RRT_root, goal_pos);
            next_world_pos[0] = -(next_node->x - MAP_SIZE / 2) * MAP_RATIO;
            next_world_pos[1] = -(next_node->y - MAP_SIZE / 2) * MAP_RATIO;
            printf("root: %d,%d next: %d,%d\n", RRT_root->x, RRT_root->y, next_node->x, next_node->y);
            map_tree[(int)goal_pos[1]][(int)goal_pos[0]] = 253;     // draw goal on map
            map_tree[(int)next_node->y][(int)next_node->x] = 254;   // draw next node endpoint on map
            */

            // find next node in BFS on coarse_map
            next_coarse_path_node(cur_map_pos, goal_map_pos, next_coarse_pos);
            int map_cell_center_x = next_coarse_pos[0] * COARSE_RATIO + COARSE_RATIO / 2;
            int map_cell_center_y = next_coarse_pos[1] * COARSE_RATIO + COARSE_RATIO / 2;
            next_world_pos[0] = -(map_cell_center_x - MAP_SIZE / 2) * MAP_RATIO;
            next_world_pos[1] = -(map_cell_center_y - MAP_SIZE / 2) * MAP_RATIO;
            
            memcpy(map_tree, map, sizeof(uint8_t) * MAP_SIZE * MAP_SIZE);
            map_tree[(int)cur_map_pos[1]][(int)cur_map_pos[0]] = 253;   // draw start on map
            map_tree[(int)goal_pos[1]][(int)goal_pos[0]] = 253;     // draw goal on map
            map_tree[(int)map_cell_center_x][(int)map_cell_center_x] = 254;   // draw next node endpoint on map


            // to move towards next node endpoint, decide whether to rotate or move straight
            double goal_x = next_world_pos[1], goal_y = next_world_pos[0], cur_x = global_trans[1], cur_y = global_trans[0];
            double rot_needed = atan2(-(goal_y - cur_y), (goal_x - cur_x)) * 180.0 / M_PI; // atan(-y/x) because y is top to bottom on map
            double rot_relative = rot_needed - global_rot;
            while (rot_relative > 180.0) rot_relative -= 360.0;
            while (rot_relative < -180.0) rot_relative += 360.0;
            double dist_needed = sqrt(pow((goal_x - cur_x), 2) + pow((goal_y - cur_y), 2));
            if (fabs(rot_relative) > PLANNING_ROTATION_TOLERANCE) {
                target_rot = rot_needed;
                state = ROTATING;
            } else {
                target_dist = dist_needed;
                state = MOVING;
            }

            //free_RRT(RRT_root);
        } 
        
        if (state == ROTATING) {
            double rot_needed = target_rot - global_rot;
            while (rot_needed > 180.0) rot_needed -= 360.0;
            while (rot_needed < -180.0) rot_needed += 360.0;
            if (fabs(rot_needed) > PLANNING_ROTATION_TOLERANCE) {
                motor_rotate(rot_needed);
            } else {
                state = MOVING;
            }
        } 
        if (state == MOVING) {
            motor_straight(target_dist);
            state = PLANNING;
        }

    }
}



RRT_node * create_RRT_node_vals(int x, int y, RRT_node * parent, size_t child_cnt, size_t child_cap) {
    RRT_node * node = malloc(sizeof(RRT_node));
    node->x = x;
    node->y = y;
    node->parent = parent;
    node->child_cnt = child_cnt;
    node->child_cap = child_cap;
    node->children = malloc(node->child_cap * sizeof(RRT_node *));
    return node;
}

RRT_node * create_RRT_node_null() {
    RRT_node * node = malloc(sizeof(RRT_node));
    node->x = MAP_SIZE / 2;
    node->y = MAP_SIZE / 2;
    node->parent = NULL;
    node->child_cnt = 0;
    node->child_cap = 5; // arbitrary starting capacity
    node->children = malloc(node->child_cap * sizeof(RRT_node *));
    return node;
}

RRT_node * find_nearest_RRT_node(RRT_node * root, int a_x, int a_y) {
    size_t i = 0, j = 1; // j size of filled portion
    RRT_traversal_queue[0] = root;

    double closest_dist_to_a = DBL_MAX;
    RRT_node * closest_node_to_a = NULL;
    while (i < j) {
        int x = RRT_traversal_queue[i]->x, y = RRT_traversal_queue[i]->y;
        double dist_to_a = sqrt(pow(x - a_x, 2) + pow(y - a_y, 2));
        if (dist_to_a < closest_dist_to_a) {
            closest_dist_to_a = dist_to_a;
            closest_node_to_a = RRT_traversal_queue[i];
        }

        for (size_t k = 0; k < RRT_traversal_queue[i]->child_cnt; ++k)
            RRT_traversal_queue[j++] = RRT_traversal_queue[i]->children[k];
        ++i;
    }

    return closest_node_to_a;
}


void fill_coarse_map() {
    for (size_t i = 0; i < MAP_SIZE; ++i) {
        for (size_t j = 0; j < MAP_SIZE; ++j) {
            if (map[i][j] != 0)
                coarse_map[i/COARSE_RATIO][j/COARSE_RATIO] = true;
        }
    }
}

// 0 = check edge constraint on coarse_map, 1 = draw edge constraint on coarse_map, 2 = draw RRT on map
bool bresenhams_line(RRT_node * a, RRT_node * b, uint8_t edge_constraint_or_draw_rrt) {
    // use bresenhams line algorithm to walk along edge on coarse map
    int a_x = a->x / COARSE_RATIO, a_y = a->y / COARSE_RATIO, b_x = b->x / COARSE_RATIO, b_y = b->y / COARSE_RATIO;
    if (edge_constraint_or_draw_rrt == 2) {
        a_x = a->x; a_y = a->y; b_x = b->x; b_y = b->y;
    }
    int dx = abs(b_x - a_x), dy = -abs(b_y - a_y);
    int s_x = a_x < b_x ? 1 : -1, s_y = a_y < b_y ? 1 : -1;
    int error = dx + dy;

    while (true) {
        if (edge_constraint_or_draw_rrt == 0) {
            if (coarse_map[a_y][a_x] || (coarse_tree_map[a_y][a_x] && (
                (a_x != a->x / COARSE_RATIO || a_y != a->y / COARSE_RATIO) &&
                (a_x != b->x / COARSE_RATIO || a_y != b->y / COARSE_RATIO))))
                return false;
        } else if (edge_constraint_or_draw_rrt == 1) {
            coarse_tree_map[a_y][a_x] = true;
        } else if (a_y >= 0 && a_y < MAP_SIZE && a_x >= 0 && a_x < MAP_SIZE) {
            if (map_tree[a_y][a_x] == 0)
                map_tree[a_y][a_x] = 255;
        }

        if (a_x == b_x && a_y == b_y) break;

        // step
        int e2 = 2 * error;
        if (e2 >= dy) { error += dy; a_x += s_x; }
        if (e2 <= dx) { error += dx; a_y += s_y; }
    }

    return true;
}

bool edge_constraints_met(RRT_node * a, RRT_node * b) { // local planner
    if (a == NULL || b == NULL) return false;

    // prevent adding a new point too close to the other one (can't verify doesnt intersect tree)
    if (a->x / COARSE_RATIO == b->x / COARSE_RATIO && a->y / COARSE_RATIO == b->y / COARSE_RATIO)
        return false;

    return bresenhams_line(a, b, 0);
}

RRT_node * compute_RRT(double root_pos[]) {
    // reset coarse_indices
    for (size_t i = 0; i < max_coarse_index_length; ++i)
        coarse_indices[i] = i;

    fill_coarse_map();
    memset(coarse_tree_map, 0, sizeof(bool) * ((MAP_SIZE / COARSE_RATIO) * (MAP_SIZE / COARSE_RATIO)));
    RRT_node * root = create_RRT_node_vals(root_pos[0], root_pos[1], NULL, 0, 5);
    RRT_node * cur = root;
    int last_coarse_index = max_coarse_index_length-1;
    int i = 0;
    while (last_coarse_index >= 0) { // try to add a node_a made from random points in space to tree
        //size_t random_map_index = rand() % (MAP_SIZE * MAP_SIZE);
        // select index from array of coarse indices not tried yet, then map this to map index
        //size_t random_coarse_map_index_index = rand() % (last_coarse_index+1);
        size_t random_coarse_map_index_index = random_values[i++] % (last_coarse_index+1);
        size_t selected_coarse_map_index = coarse_indices[random_coarse_map_index_index];
        coarse_indices[random_coarse_map_index_index] = coarse_indices[last_coarse_index];
        coarse_indices[last_coarse_index] = selected_coarse_map_index;
        size_t random_map_index = selected_coarse_map_index * (COARSE_RATIO * COARSE_RATIO) + 
                (rand() % (COARSE_RATIO*COARSE_RATIO));
        --last_coarse_index;

        size_t random_map_y = random_map_index / MAP_SIZE, random_map_x = random_map_index % MAP_SIZE;
        RRT_node * node_a = create_RRT_node_vals(random_map_x, random_map_y, cur, 0, 5);
        RRT_node * node_b = find_nearest_RRT_node(root, node_a->x, node_a->y);

        if (edge_constraints_met(node_a, node_b)) { // add node_a to tree
            node_a->parent = node_b;

            if (node_b->child_cnt >= node_b->child_cap) {
                node_b->child_cap *= 2;
                node_b->children = realloc(node_b->children, node_b->child_cap * sizeof(RRT_node *));
            }

            node_b->children[node_b->child_cnt++] = node_a;
            
            bresenhams_line(node_a, node_b, 1); // draw edge on coarse_map
        } else {
            free(node_a->children);
            free(node_a);
        }
    }

    return root;
}

void draw_RRT_on_map(RRT_node * root) {
    memcpy(map_tree, map, sizeof(uint8_t) * MAP_SIZE * MAP_SIZE);
    size_t i = 0, j = 1; // j size of filled portion
    RRT_traversal_queue[0] = root;

    
    map_tree[root->y][root->x] = 254;
    
    while (i < j) {
        RRT_node * a = RRT_traversal_queue[i];
        for (size_t k = 0; k < a->child_cnt; ++k) {
            RRT_node * b = a->children[k];
            bresenhams_line(a, b, 2);

            RRT_traversal_queue[j++] = b;
        }
        ++i;
        
    }
}

void free_RRT(RRT_node * root) {
    size_t i = 0, j = 1; // j size of filled portion
    RRT_traversal_queue[0] = root;

    while (i < j) {
        RRT_node * node = RRT_traversal_queue[i];
        for (size_t k = 0; k < node->child_cnt; ++k) {
            RRT_node * child = node->children[k];
            RRT_traversal_queue[j++] = child;
        }
        ++i;
    }

    for (size_t k = 0; k < j; ++k) {
        free(RRT_traversal_queue[k]->children);
        free(RRT_traversal_queue[k]);
    }
}


void manual_control() {
    RangeScanNode * head = create_range_scan_node();
    RangeScanNode * prev = head;
    double global_trans[2] = {0};
    double global_rot = 0;
    memcpy(map_tree, map, sizeof(uint8_t) * MAP_SIZE * MAP_SIZE);

    while (1) {
        while (!slam_restart && !manual_left && !manual_forward && !manual_right) vTaskDelay(pdMS_TO_TICKS(50));
        if (slam_restart) {
            break;
        }
        if (manual_left) {
            motor_rotate(-20.0);
            manual_left = false;
        }
        if (manual_right) {
            motor_rotate(20.0);
            manual_right = false;
        }
        if (manual_forward) {
            motor_straight(20.0);
            manual_forward = false;
        }
        prev = SLAM_iteration(prev, global_trans, &global_rot);
        memcpy(map_tree, map, sizeof(uint8_t) * MAP_SIZE * MAP_SIZE);
        ++iterations;
    }
}

void app_main(void) 
{
    gpio_config_t io_config = {
        .pin_bit_mask = 
                (1ULL << GPIO_NUM_1) |   // STBY
                (1ULL << GPIO_NUM_2) |   // AIN1
                (1ULL << GPIO_NUM_3) |   // AIN2
                (1ULL << GPIO_NUM_7) |   // BIN1
                (1ULL << GPIO_NUM_23),   // BIN2
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&io_config);
    gpio_set_level(GPIO_NUM_1, 1); // STBY

    ledc_timer_config_t motor_ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 10000,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ledc_timer_config_t servo_ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ledc_channel_config_t ledc_channel_a = {
        .gpio_num       = GPIO_NUM_10,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config_t ledc_channel_b = {
        .gpio_num       = GPIO_NUM_11,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_1,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config_t ledc_channel_c = {
        .gpio_num       = GPIO_NUM_6,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_2,
        .timer_sel      = LEDC_TIMER_1,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_timer_config(&motor_ledc_timer);
    ledc_timer_config(&servo_ledc_timer);
    ledc_channel_config(&ledc_channel_a);
    ledc_channel_config(&ledc_channel_b);
    ledc_channel_config(&ledc_channel_c);

    changeSpeedA(0, 0);
    changeSpeedB(0, 0);

    // distance sensor i2c
    i2c_master_init();
    if (VL53L4CD_SensorInit(sensor) != 0)
        printf("Sensor init failed");
    VL53L4CD_SetRangeTiming(sensor, 50, 0);
    if (VL53L4CD_StartRanging(sensor) != 0)
        printf("Start Ranging failed");
    

    esp_err_t ret = nvs_flash_init();
    printf("nvs_flash_init: %d\n", ret);

    handle_server_init();

    recenter_servo();

    // initializing array of indices of coarse_map
    for (size_t i = 0; i < max_coarse_index_length; ++i) {
        coarse_indices[i] = i;
    }

    while (1) {
        if (slam_restart) {
            slam_restart = false;
            printf("restarting slam\n");
            iterations = 0;
            memset(map, 0, sizeof(map));
            SLAM_run();
            //manual_control();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    };
}
    