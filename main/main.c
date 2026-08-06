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
bool coarse_map_open[MAP_SIZE / COARSE_RATIO][MAP_SIZE / COARSE_RATIO] = {false};
bool coarse_tree_map[MAP_SIZE / COARSE_RATIO][MAP_SIZE / COARSE_RATIO] = {false};
bool coarse_corner_map[MAP_SIZE / COARSE_RATIO][MAP_SIZE / COARSE_RATIO] = {false};
int coarse_indices[(MAP_SIZE / COARSE_RATIO) * (MAP_SIZE / COARSE_RATIO)];
double prev_move = 0.0, prev_rot = 0.0;

volatile bool slam_restart = true, manual_left = false, manual_right = false, manual_forward = false, slam_end = false;
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
void fill_coarse_map();
void get_normal_from_tangent(double *, double *, double *, int, int, int);
bool bresenhams_line(int, int);
double euclidean_flat_dist(double, double, double, double);


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
    i = 2;
    while (i < freq) {
        if (i < freq-2 && points[i-1] == 0 && points[i+1] == 0 && points[i-2] == 0 && points[i+2] == 0) // no neighbors
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
            tangent_x += (points_x_y[neighbor_i_2*2] != 0 ? points_x_y[neighbor_i_2*2] : 0) - (points_x_y[neighbor_i_1*2] != 0 ? points_x_y[neighbor_i_1*2] : 0);
            tangent_y += (points_x_y[neighbor_i_2*2+1] != 0 ? points_x_y[neighbor_i_2*2+1] : 0) - (points_x_y[neighbor_i_1*2+1] != 0 ? points_x_y[neighbor_i_1*2+1] : 0);
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

void get_point_normals(double points_x_y[], size_t points_len, double point_normals_x_y[]) {
    for (size_t i = 0; i < points_len; ++i) {
        if (points_x_y[2*i] == 0.0 && points_x_y[2*i+1] == 0.0) continue;
        
        double normal_x1, normal_y1;
        int left_i = (i > 0) ? (int)i-1 : 0;
        int right_i = (i < points_len-1) ? (int)i+1 : (int)points_len-1;
        get_normal_from_tangent(&normal_x1, &normal_y1, points_x_y, points_len, left_i, right_i);
        point_normals_x_y[2*i] = normal_x1;
        point_normals_x_y[2*i+1] = normal_y1;
    }
}

// 2
void compute_correspondence_pairs(double points_x_y[], size_t points_len, double old_points_x_y[], double point_normals_x_y[], double old_point_normals_x_y[], double corresp_points_x_y[], double g_angle, double offset_x, double offset_y, int * num_pairs, int * num_outliers, double combined_x_y_and_pair_dists[]) {
    for (size_t i = 0; i < points_len; ++i) {
        if (points_x_y[2*i] == 0.0 && points_x_y[2*i+1] == 0.0) continue;
        double x1 = points_x_y[2*i] + offset_x;
        double y1 = points_x_y[2*i+1] + offset_y;
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
            //double normal_x1, normal_y1;
            //int left_i = (i > 0) ? (int)i-1 : 0;
            //int right_i = (i < points_len-1) ? (int)i+1 : (int)points_len-1;
            //get_normal_from_tangent(&normal_x1, &normal_y1, points_x_y, points_len, left_i, right_i);
            double normal_x1 = point_normals_x_y[2*i];
            double normal_y1 = point_normals_x_y[2*i+1];
            double rot_normal_x1 = cos(g_angle * M_PI/180) * normal_x1 - sin(g_angle * M_PI/180) * normal_y1;
            double rot_normal_y1 = sin(g_angle * M_PI/180) * normal_x1 + cos(g_angle * M_PI/180) * normal_y1;
            

            //     n*
            double n_left_x = old_point_normals_x_y[2*old_i_less_than_angle], n_left_y = old_point_normals_x_y[2*old_i_less_than_angle+1], 
                   n_right_x = old_point_normals_x_y[2*old_i_greater_than_angle], n_right_y = old_point_normals_x_y[2*old_i_greater_than_angle+1];
            //int old_less_left_i = (old_i_less_than_angle > 0) ? (int)old_i_less_than_angle-1 : 0;
            //int old_less_right_i = (old_i_less_than_angle < points_len-1) ? (int)old_i_less_than_angle+1 : (int)points_len-1;
            //int old_greater_left_i = (old_i_greater_than_angle > 0) ? (int)old_i_greater_than_angle-1 : 0;
            //int old_greater_right_i = (old_i_greater_than_angle < points_len-1) ? (int)old_i_greater_than_angle+1 : (int)points_len-1;
            //get_normal_from_tangent(&n_left_x, &n_left_y, old_points_x_y, points_len, old_less_left_i, old_less_right_i);
            //get_normal_from_tangent(&n_right_x, &n_right_y, old_points_x_y, points_len, old_greater_left_i, old_greater_right_i);
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
        } else {
            ++(*num_outliers);
        }
    }
}

// 3
double compute_tmd_point_to_plane(double points_x_y[], double corresp_points_x_y[], size_t points_len, int num_pairs, int num_outliers, double combined_x_y_and_pair_dists[], double * found_Tx, double * found_Ty) {
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
    int count = 0;
    for (size_t i = 0; i < points_len; ++i) {
        if (points_x_y[2*i] == 0.0 || points_x_y[2*i+1] == 0.0 || corresp_points_x_y[2*i] == 0.0 || corresp_points_x_y[2*i+1] == 0.0) {
            continue; // invalid point or corresp_point
        } else {
            double combined_x = combined_x_y_and_pair_dists[i*3];
            double combined_y = combined_x_y_and_pair_dists[i*3+1];
            double d = combined_x_y_and_pair_dists[i*3+2];
            double residual = combined_x*Tx + combined_y*Ty - d;
            sum_squared_residuals += pow(residual, 2);
            ++count;
        }
    }

    *found_Tx = Tx;
    *found_Ty = Ty;
    // total matching distance: match(ω) = 1/(np + no)(min_T(E(ω,T)) + no*(H_d)^2)
    return 1.0 / (count + num_outliers) * (sum_squared_residuals + num_outliers * pow(MAX_DISTANCE_PER_ITERATION, 2));
}

double compute_tmd_point_to_point(double points_x_y[], double corresp_points_x_y[], double points_normals_x_y[], size_t points_len, int num_pairs, int num_outliers) {
    if (num_pairs + num_outliers == 0) return DBL_MAX; // safety against div by 0

    double sum_squared_residuals = 0.0;
    double total_weight = 0.0;
    for (size_t i = 0; i < points_len; ++i) {
        if (points_x_y[2*i] == 0.0 || corresp_points_x_y[2*i] == 0.0) continue; // invalid point or corresp_point
        
        double dx = corresp_points_x_y[2*i] - points_x_y[2*i];
        double dy = corresp_points_x_y[2*i+1] - points_x_y[2*i+1];

        
        // weight by incident angle to direction of car
        double forward_x = 0.0, forward_y = 1.0;
        double forward_alignment = fabs(dx * forward_x + dy * forward_y);

        // weight by incident angle to sensor ray
        double ray_x = points_x_y[2*i], ray_y = points_x_y[2*i+1];
        double ray_mag = sqrt(ray_x*ray_x + ray_y*ray_y);
        ray_x /= ray_mag;
        ray_y /= ray_mag;
        double ray_alignment = fabs(points_normals_x_y[2*i] * ray_x + points_normals_x_y[2*i+1] * ray_y);
        double weight = 1.0; //0.5*pow(ray_alignment, 5);// + 1.0*pow(forward_alignment, 1);
        //double weight = pow(cos(scan_angle * M_PI/180), 6); // weight by angle: 1 at center, 0 at edges
        //if (weight <= 0.0) continue;
        
        total_weight += weight;
        sum_squared_residuals += weight * pow(dx, 2) + pow(dy, 2);
    }
    //printf("total weight %f\n", total_weight);

    return 1.0 / (total_weight + num_outliers) * (sum_squared_residuals + num_outliers * pow(MAX_DISTANCE_PER_ITERATION, 3));
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

double gs_search_for_angle(double gs_lower_bound, double gs_upper_bound, double * optimal_rot, double g_rot, double local_points_x_y[], double prev_points_x_y[], double points_normals_x_y[], double old_points_normals_x_y[], double corresp_points_x_y[], double combined_x_y_and_pair_dists[]) {
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
        double estim_x = (SENSOR_OFFSET_FROM_PIVOT * sin(g_rot*M_PI/180.0)) + (SENSOR_OFFSET_FROM_PIVOT + prev_move) * -sin((g_rot + test_rot)*M_PI/180.0);
        double estim_y = (SENSOR_OFFSET_FROM_PIVOT * -cos(g_rot*M_PI/180.0)) + (SENSOR_OFFSET_FROM_PIVOT + prev_move) * cos((g_rot + test_rot)*M_PI/180.0);
        compute_correspondence_pairs(local_points_x_y, SENSOR_FREQ, prev_points_x_y, points_normals_x_y, old_points_normals_x_y, corresp_points_x_y, test_rot, estim_x, estim_y, &num_pairs, &num_outliers, combined_x_y_and_pair_dists);
        double tmd_1 = compute_tmd_point_to_point(local_points_x_y, corresp_points_x_y, points_normals_x_y, SENSOR_FREQ, num_pairs, num_outliers);
        //double tmd_1 = compute_total_matching_distance(local_points_x_y, corresp_points_x_y, SENSOR_FREQ, num_pairs, num_outliers, combined_x_y_and_pair_dists, &delta_Tx_1, &delta_Ty_1);


        test_rot = x2;

        memset(corresp_points_x_y, 0, SENSOR_FREQ * sizeof(double) * 2);
        memset(combined_x_y_and_pair_dists, 0, SENSOR_FREQ * sizeof(double) * 3);
        num_pairs = 0; num_outliers = 0;
        estim_x = (SENSOR_OFFSET_FROM_PIVOT * sin(g_rot*M_PI/180.0)) + (SENSOR_OFFSET_FROM_PIVOT + prev_move) * -sin((g_rot + test_rot)*M_PI/180.0);
        estim_y = (SENSOR_OFFSET_FROM_PIVOT * -cos(g_rot*M_PI/180.0)) + (SENSOR_OFFSET_FROM_PIVOT + prev_move) * cos((g_rot + test_rot)*M_PI/180.0);
        compute_correspondence_pairs(local_points_x_y, SENSOR_FREQ, prev_points_x_y, points_normals_x_y, old_points_normals_x_y, corresp_points_x_y, test_rot, estim_x, estim_y, &num_pairs, &num_outliers, combined_x_y_and_pair_dists);
        
        double tmd_2 = compute_tmd_point_to_point(local_points_x_y, corresp_points_x_y, points_normals_x_y, SENSOR_FREQ, num_pairs, num_outliers);
        //double tmd_2 = compute_total_matching_distance(local_points_x_y, corresp_points_x_y, SENSOR_FREQ, num_pairs, num_outliers, combined_x_y_and_pair_dists, &delta_Tx_2, &delta_Ty_2);
        
        
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
    double estim_x = (SENSOR_OFFSET_FROM_PIVOT * sin(g_rot*M_PI/180.0)) + (SENSOR_OFFSET_FROM_PIVOT + prev_move) * -sin((g_rot + *optimal_rot)*M_PI/180.0);
    double estim_y = (SENSOR_OFFSET_FROM_PIVOT * -cos(g_rot*M_PI/180.0)) + (SENSOR_OFFSET_FROM_PIVOT + prev_move) * cos((g_rot + *optimal_rot)*M_PI/180.0);
    compute_correspondence_pairs(local_points_x_y, SENSOR_FREQ, prev_points_x_y, points_normals_x_y, old_points_normals_x_y, corresp_points_x_y, *optimal_rot, estim_x, estim_y, &num_pairs, &num_outliers, combined_x_y_and_pair_dists);
    //return compute_total_matching_distance(local_points_x_y, corresp_points_x_y, SENSOR_FREQ, num_pairs, num_outliers, combined_x_y_and_pair_dists, &delta_Tx_2, &delta_Ty_2);
    return compute_tmd_point_to_point(local_points_x_y, corresp_points_x_y, points_normals_x_y, SENSOR_FREQ, num_pairs, num_outliers);
}

RangeScanNode * SLAM_iteration(RangeScanNode * prev, double g_trans[], double * g_rot) {
    uint16_t *points = malloc(SENSOR_FREQ * sizeof(uint16_t)); // polar form: magnitude as value, angle implicit from index
    double *local_points_x_y = malloc(SENSOR_FREQ * sizeof(double) * 2);
    double *points_x_y = malloc(SENSOR_FREQ * sizeof(double) * 2);
    double *corresp_points_x_y = malloc(SENSOR_FREQ * sizeof(double) * 2);
    double *combined_x_y_and_pair_dists = malloc(SENSOR_FREQ * sizeof(double) * 3);
    double *points_normals_x_y = malloc(SENSOR_FREQ * sizeof(double) * 2);
    double *old_points_normals_x_y = malloc(SENSOR_FREQ * sizeof(double) * 2);
    int num_pairs = 0, num_outliers = 0;
    //double gs_lower_bound = -30.0, gs_upper_bound = 30.0;

    printf("iteration: %d\n", iterations);
    collect_range_scan(points, SENSOR_FREQ, SENSOR_PERIOD);    
    double trans_zero[2] = {0.0, 0.0};
    transform_points(points, local_points_x_y, SENSOR_FREQ, trans_zero, 0.0);
    get_point_normals(local_points_x_y, SENSOR_FREQ, points_normals_x_y);
    get_point_normals(prev->points_x_y, SENSOR_FREQ, old_points_normals_x_y);
    if (iterations > 0) {
        // use multi-hypothesis to find best angle to center golden section search around
        double best_tmd = DBL_MAX;
        double best_rot = 0.0;
        int num_coarse = 11;  // try this many angles between evenly spaced among prev_rot +- search_half_width
        double search_half_width = 15.0;
        double delta_Tx_tmp = 0, delta_Ty_tmp = 0;
        for (int s = 0; s < num_coarse; ++s) {
            double test_rot = (prev_rot - search_half_width) + s * (2*search_half_width / (num_coarse - 1));
            //double test_rot = -30.0 + s * (60.0 / (num_coarse - 1));
            memset(corresp_points_x_y, 0, SENSOR_FREQ * sizeof(double) * 2);
            memset(combined_x_y_and_pair_dists, 0, SENSOR_FREQ * sizeof(double) * 3);
            num_pairs = 0; num_outliers = 0;
            double estim_x = (SENSOR_OFFSET_FROM_PIVOT * sin((*g_rot)*M_PI/180.0)) + (SENSOR_OFFSET_FROM_PIVOT + prev_move) * -sin(((*g_rot) + test_rot)*M_PI/180.0);
            double estim_y = (SENSOR_OFFSET_FROM_PIVOT * -cos((*g_rot)*M_PI/180.0)) + (SENSOR_OFFSET_FROM_PIVOT + prev_move) * cos(((*g_rot) + test_rot)*M_PI/180.0);
            compute_correspondence_pairs(local_points_x_y, SENSOR_FREQ, prev->points_x_y, points_normals_x_y, old_points_normals_x_y, corresp_points_x_y, test_rot, estim_x, estim_y, &num_pairs, &num_outliers, combined_x_y_and_pair_dists);
            double tmd = compute_tmd_point_to_point(local_points_x_y, corresp_points_x_y, points_normals_x_y, SENSOR_FREQ, num_pairs, num_outliers);
            printf("s: %d, tmd: %f, num_outliers: %d\n", s, tmd, num_outliers);
            if (tmd < best_tmd) {
                best_tmd = tmd;
                best_rot = test_rot;
            }
        }

        // golden section search to find ω that minimizes total_matching_distance
        double refine_range = 3.0;  // search +- these degrees around best coarse angle
        double optimal_rot = 0.0;
        gs_search_for_angle(best_rot - refine_range, best_rot + refine_range, &optimal_rot, *g_rot, local_points_x_y, prev->points_x_y, points_normals_x_y, old_points_normals_x_y, corresp_points_x_y, combined_x_y_and_pair_dists);
        optimal_rot = optimal_rot * (1-DEADRECKON_ROT_WEIGHT) + prev_rot * (DEADRECKON_ROT_WEIGHT);
        memset(corresp_points_x_y, 0, SENSOR_FREQ * sizeof(double) * 2);
        memset(combined_x_y_and_pair_dists, 0, SENSOR_FREQ * sizeof(double) * 3);
        num_pairs = 0; num_outliers = 0;
        double dr_estim_x = (SENSOR_OFFSET_FROM_PIVOT * sin((*g_rot)*M_PI/180.0)) + (SENSOR_OFFSET_FROM_PIVOT + prev_move) * -sin(((*g_rot) + prev_rot)*M_PI/180.0);
        double dr_estim_y = (SENSOR_OFFSET_FROM_PIVOT * -cos((*g_rot)*M_PI/180.0)) + (SENSOR_OFFSET_FROM_PIVOT + prev_move) * cos(((*g_rot) + prev_rot)*M_PI/180.0);
        compute_correspondence_pairs(local_points_x_y, SENSOR_FREQ, prev->points_x_y, points_normals_x_y, old_points_normals_x_y, corresp_points_x_y, prev_rot, dr_estim_x, dr_estim_y, &num_pairs, &num_outliers, combined_x_y_and_pair_dists);
        double dr_tmd = compute_tmd_point_to_point(local_points_x_y, corresp_points_x_y, points_normals_x_y, SENSOR_FREQ, num_pairs, num_outliers);
        printf("tmd found dr %f icp %f\n", dr_tmd, best_tmd);
        if (best_tmd > dr_tmd) {// use dead reckoning rot instead if its tmd was better than the rot found by ICP
            optimal_rot = prev_rot;
            printf("chose dr rot (%f) instead of icp rot (%f) because its tmd was better (%f vs %f)\n", prev_rot, optimal_rot, dr_tmd, best_tmd);
        }

        // compute T that minimizes error given found ω using point-to-point ICP: T = mean(P*) - mean(Rω * P1)
        double final_Tx = 0.0, final_Ty = 0.0;
        memset(corresp_points_x_y, 0, SENSOR_FREQ * sizeof(double) * 2);
        memset(combined_x_y_and_pair_dists, 0, SENSOR_FREQ * sizeof(double) * 3);
        num_pairs = 0; num_outliers = 0;
        double expected_Tx = (SENSOR_OFFSET_FROM_PIVOT * sin((*g_rot)*M_PI/180.0)) + (SENSOR_OFFSET_FROM_PIVOT + prev_move) * -sin(((*g_rot) + optimal_rot)*M_PI/180.0);
        double expected_Ty = (SENSOR_OFFSET_FROM_PIVOT * -cos((*g_rot)*M_PI/180.0)) + (SENSOR_OFFSET_FROM_PIVOT + prev_move) * cos(((*g_rot) + optimal_rot)*M_PI/180.0);
        compute_correspondence_pairs(local_points_x_y, SENSOR_FREQ, prev->points_x_y, points_normals_x_y, old_points_normals_x_y, corresp_points_x_y, optimal_rot, expected_Tx, expected_Ty, &num_pairs, &num_outliers, combined_x_y_and_pair_dists);
        double sum_corresp_x = 0, sum_corresp_y = 0;
        double sum_rot_x = 0, sum_rot_y = 0;
        double total_weight = 0;
        for (size_t i = 0; i < SENSOR_FREQ; ++i) {
            if (local_points_x_y[2*i] == 0.0 || local_points_x_y[2*i+1] == 0.0 || corresp_points_x_y[2*i] == 0.0 || corresp_points_x_y[2*i+1] == 0.0)
                continue;
            //double scan_angle = ((double)i / (SENSOR_FREQ - 1) * 180.0 - 90.0) * M_PI / 180.0;

            // weight by incident angle to direction of car
            double forward_x = 0.0, forward_y = 1.0;
            double forward_alignment = fabs(points_normals_x_y[2*i] * forward_x + points_normals_x_y[2*i+1] * forward_y);

            // weight by incident angle to sensor ray
            double ray_x = local_points_x_y[2*i], ray_y = local_points_x_y[2*i+1];
            double ray_mag = sqrt(ray_x*ray_x + ray_y*ray_y);
            ray_x /= ray_mag;
            ray_y /= ray_mag;
            double ray_alignment = fabs(points_normals_x_y[2*i] * ray_x + points_normals_x_y[2*i+1] * ray_y);
            double weight = 0.1 * pow(ray_alignment, 10) + 1.0 * pow(forward_alignment, 4);
            //double weight = pow(cos(scan_angle * M_PI/180), 6); // weight by angle: 1 at center, 0 at edges
            if (weight <= 0.0) continue;


            double rot_x = cos(optimal_rot * M_PI/180) * local_points_x_y[2*i] - sin(optimal_rot * M_PI/180) * local_points_x_y[2*i+1];
            double rot_y = sin(optimal_rot * M_PI/180) * local_points_x_y[2*i] + cos(optimal_rot * M_PI/180) * local_points_x_y[2*i+1];
            sum_corresp_x += weight * corresp_points_x_y[2*i];
            sum_corresp_y += weight * corresp_points_x_y[2*i+1];
            sum_rot_x += weight * rot_x;
            sum_rot_y += weight * rot_y;
            total_weight += weight;
        }

        if (total_weight > 0) {
            //double expected_Tx = prev_move * -sin((*g_rot) * M_PI/180.0);
            //double expected_Ty = prev_move * cos((*g_rot) * M_PI/180.0);
            double icp_Tx = (sum_corresp_x - sum_rot_x) / total_weight;
            double icp_Ty = (sum_corresp_y - sum_rot_y) / total_weight;
            
            final_Tx = (DEADRECKON_T_WEIGHT * expected_Tx) + ((1.0-DEADRECKON_T_WEIGHT) * icp_Tx);
            final_Ty = (DEADRECKON_T_WEIGHT * expected_Ty) + ((1.0-DEADRECKON_T_WEIGHT) * icp_Ty);
            /*
            if (prev_move > 0.0) { // normal case
                final_Tx = (DEADRECKON_WEIGHT * expected_Tx) + ((1.0-DEADRECKON_WEIGHT) * icp_Tx);
                final_Ty = (DEADRECKON_WEIGHT * expected_Ty) + ((1.0-DEADRECKON_WEIGHT) * icp_Ty);
            } else { // for rotation, use pure icp (deadreckoning doesnt account for sensor displacement from turning)
                final_Tx = ICP_TURN_SCALAR * icp_Tx;
                final_Ty = ICP_TURN_SCALAR * icp_Ty;
            }
                */
            
            //printf("total weight %f\t icp_Tx %f\t icp_Ty %f\t expected_Tx %f\t expected_Ty %f\n", total_weight, icp_Tx, icp_Ty, expected_Tx, expected_Ty);
            
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
    fill_coarse_map();
    
    RangeScanNode* new_node = create_range_scan_node();
    memcpy(new_node->points_x_y, local_points_x_y, SENSOR_FREQ * sizeof(double) * 2); // save points in local space in new node
    new_node->prev = prev;
    prev->next = new_node;
    
    prev_move = 0.0;
    prev_rot = 0.0;
    free(points_normals_x_y);
    free(old_points_normals_x_y);
    free(local_points_x_y);
    free(combined_x_y_and_pair_dists);
    free(corresp_points_x_y);
    free(points_x_y);
    free(points);
    return new_node;
}

void motor_rotate(double rot_needed) { // rotate rot_needed degrees
    rot_needed = fabs(rot_needed) <= MAX_ROT_PER_STEP ? rot_needed : (rot_needed > 0 ? MAX_ROT_PER_STEP : -MAX_ROT_PER_STEP);
    prev_rot = rot_needed; // dead reckoning (rotation)

    double coeff = fabs(rot_needed) >= 30 ? 24.0 : -0.131139 * fabs(rot_needed) + 28.1038;
    //double coeff = 84.43506 / pow(fabs(rot_needed), 0.02) + -69.98076;

    if (rot_needed < 0) {
        changeSpeedA(0, MOVE_ROTATE_SPEED);
        changeSpeedB(1, MOVE_ROTATE_SPEED);
    } else {
        changeSpeedA(1, MOVE_ROTATE_SPEED);
        changeSpeedB(0, MOVE_ROTATE_SPEED);
    }
    vTaskDelay(pdMS_TO_TICKS(coeff * fabs(rot_needed))); // 18.8 24.4    14.2 24.0
    changeSpeedA(0, 0);
    changeSpeedB(0, 0);
}

void motor_straight(double dist_needed) { // move dist_needed mm bounded by MAX_DIST_PER_STEP
    dist_needed = dist_needed <= MAX_DIST_PER_STEP ? dist_needed : MAX_DIST_PER_STEP;
    prev_move = dist_needed; // for dead reckoning

    double coeff = dist_needed >= 75.0 ? 10.2 : 69.65524 / pow(dist_needed, 0.02) + -53.51863; // variable coefficient based on MOVE_SPEED 35
    //double coeff = -0.0532563 * dist_needed + 13.55715;

    changeSpeedA(0, MOVE_STRAIGHT_SPEED);
    changeSpeedB(0, MOVE_STRAIGHT_SPEED);
    vTaskDelay(pdMS_TO_TICKS(coeff * dist_needed)); 
    changeSpeedA(0, 0);
    changeSpeedB(0, 0);
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
    int parent;
    double g;
} AStarNode;

void swap(AStarNode ** a, AStarNode ** b) {
    AStarNode * temp = *a;
    *a = *b;
    *b = temp;
}

void pq_enqueue(AStarNode ** pq, AStarNode * open, int node_i, double node_f, bool node_open, int node_parent, int node_g, int * pq_size, int pq_max_size) {
    if (node_i < 0 || node_i >= pq_max_size) return; // oob
    if (*pq_size >= pq_max_size) return; // full
    
    int i = *pq_size;
    open[node_i].i = node_i;
    open[node_i].f = node_f;
    open[node_i].open = node_open;
    open[node_i].parent = node_parent;
    open[node_i].g = node_g;
    pq[i] = &(open[node_i]);
    ++(*pq_size);

    // keep swapping with parent to shift up to correct position
    while (i != 0 && (pq[(i-1)/2])->f > (pq[i])->f) {
        swap(&pq[(i-1)/2], &pq[i]);
        i = (i-1)/2;
    }
}

void pq_heapify_down(AStarNode ** pq, int i, int * pq_size) {
    int left = 2 * i;
    int right = 2 * i + 1;
    int smallest = i;

    if (left <= *pq_size && pq[left]->f < pq[smallest]->f)
        smallest = left;
    
    if (right <= *pq_size && pq[right]->f < pq[smallest]->f)
        smallest = right;

    if (smallest != i) {
        swap(&pq[i], &pq[smallest]);
        pq_heapify_down(pq, smallest, pq_size);
    }
}

AStarNode * pq_pop(AStarNode ** pq, int * pq_size) {
    if (*pq_size <= 0) return NULL; // underflow
    if (*pq_size == 1) {
        --(*pq_size);
        return pq[0];
    }

    AStarNode * root = pq[0];
    pq[0] = pq[*pq_size - 1];
    --(*pq_size);

    pq_heapify_down(pq, 0, pq_size);

    return root;
}

void stack_push(int * stack, int * stack_len, int stack_max_len, int item) {
    if (*stack_len >= stack_max_len) return; // full
    stack[(*stack_len)++] = item;
}

int stack_peek(int * stack, int * stack_len, int stack_max_len) {
    if (*stack_len < 1) return -1; // empty
    return stack[(*stack_len)-1];
}

int stack_pop(int * stack, int * stack_len, int stack_max_len) {
    if (*stack_len < 1) return -1; // empty
    return stack[--(*stack_len)];
}

void stack_remove(int * stack, int * stack_len, int stack_max_len, int item_i) {
    if (*stack_len < stack_max_len)
        memcpy(stack + item_i, stack + item_i+1, sizeof(int) * (*stack_len - item_i));
    --(*stack_len);
}

void stack_reorder_top(int * stack, int * stack_len, int stack_max_len, double global_trans[]) {
    if (*stack_len <= 0) return;

    int cur_coarse_map_pos[2] = {-(global_trans[0] / MAP_RATIO) + MAP_SIZE / 2 / COARSE_RATIO, -(global_trans[1] / MAP_RATIO) + MAP_SIZE / 2 / COARSE_RATIO};
    const int coarse_map_width = (MAP_SIZE / COARSE_RATIO);

    double cur_goal_dist = euclidean_flat_dist(cur_coarse_map_pos[0], cur_coarse_map_pos[1], stack[*stack_len-1] % coarse_map_width, stack[*stack_len-1] / coarse_map_width);
    if (cur_goal_dist >= SWAP_GOALS_THRESHOLD) {    // if current goal further than threshold
        int closest_goal_i = -1;
        double closest_goal_dist = cur_goal_dist;
        for (int i = 0; i < *stack_len; ++i) {      // check all the goals to try to find a closer one
            double this_goal_dist = euclidean_flat_dist(cur_coarse_map_pos[0], cur_coarse_map_pos[1], stack[i] % coarse_map_width, stack[i] / coarse_map_width);
            if (this_goal_dist < closest_goal_dist) {
                closest_goal_dist = this_goal_dist;
                closest_goal_i = i;
            }
        }

        if (closest_goal_i != -1) { // a closer goal was found
            // shuffle this goal to be new top
            int temp = stack[closest_goal_i];
            memcpy(stack + sizeof(int) * closest_goal_i, stack + sizeof(int) * (closest_goal_i+1), sizeof(int) * (*stack_len - closest_goal_i - 1));
            stack[*stack_len - 1] = temp;
        }
    }
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

// do A* on cells in coarse map to find goal
bool next_coarse_path_node(double start_pos[], int goal_stack[], int * stack_len, int next_coarse_pos[]) { // goal_stack is in coarse map space
    const int coarse_map_width = (MAP_SIZE / COARSE_RATIO);
    int coarse_start_pos[2] = {start_pos[0] / COARSE_RATIO, start_pos[1] / COARSE_RATIO};
    if (coarse_start_pos[0] < 0) coarse_start_pos[0] = 0;
    if (coarse_start_pos[0] >= coarse_map_width) coarse_start_pos[0] = coarse_map_width - 1;
    if (coarse_start_pos[1] < 0) coarse_start_pos[1] = 0;
    if (coarse_start_pos[1] >= coarse_map_width) coarse_start_pos[1] = coarse_map_width - 1;

    int coarse_goal_pos[2] = {goal_stack[(*stack_len)-1] % coarse_map_width, goal_stack[(*stack_len)-1] / coarse_map_width};
    int coarse_goal = goal_stack[(*stack_len)-1];//(goal_stack[stack_len-1] / coarse_map_width / COARSE_RATIO) * coarse_map_width + (goal_stack[stack_len-1] % coarse_map_width / COARSE_RATIO);
    int total_coarse_cells = coarse_map_width * coarse_map_width;

    int pq_size = 0;
    AStarNode ** pq = malloc(sizeof(AStarNode *) * total_coarse_cells);
    AStarNode * nodes = malloc(sizeof(AStarNode) * total_coarse_cells);

    for (int i = 0; i < total_coarse_cells; ++i) {
        nodes[i].open = false;
        nodes[i].f = DBL_MAX;
        nodes[i].i = i;
        nodes[i].parent = -1;
        nodes[i].g = DBL_MAX;
    }

    int coarse_start = coarse_start_pos[1] * coarse_map_width + coarse_start_pos[0];
    for (int i = 0; i < (*stack_len)-1; ++i) { // check if have reached any of the unreached goals        
        int goal_i_pos_x = goal_stack[i] % coarse_map_width;
        int goal_i_pos_y = goal_stack[i] / coarse_map_width;
        if (euclidean_flat_dist(goal_i_pos_x, goal_i_pos_y, coarse_goal_pos[0], coarse_goal_pos[1]) <= REACHED_DISTANCE) {
            stack_remove(goal_stack, stack_len, total_coarse_cells, i);
            --i; // have to --i because the next item in the stack will be in the position just removed from
        }
    }

    int coarse_start_dist_to_goal = euclidean_flat_dist(coarse_start_pos[0], coarse_start_pos[1], coarse_goal_pos[0], coarse_goal_pos[1]);
    if (coarse_start_dist_to_goal <= REACHED_DISTANCE) { 
        free(pq);
        free(nodes); 
        return true; // return true to signal to find a new goal
    }
    pq_enqueue(pq, nodes, coarse_start, coarse_start_dist_to_goal, true, -1, 0.0, &pq_size, total_coarse_cells);

    int closest_to_goal = -1;
    double closest_to_goal_dist = DBL_MAX;

    bool goal_found = false;

    while (true) {
        //int cur = find_min_f(open, total_coarse_cells);
        AStarNode * cur = pq_pop(pq, &pq_size);
        if (cur == NULL) break; // break if queue empty
        int cur_y = cur->i / coarse_map_width, cur_x = cur->i % coarse_map_width; 

        int cur_direct_dist_to_goal = euclidean_flat_dist(cur_x, cur_y, coarse_goal_pos[0], coarse_goal_pos[1]); // save closest cell by euclidean distance to goal for fail condition
        if (cur_direct_dist_to_goal < closest_to_goal_dist) {
            closest_to_goal = cur->i;
            closest_to_goal_dist = cur_direct_dist_to_goal;
        }        

        if (!cur->open) continue; // no open nodes left
        cur->open = false;
        //nodes[cur->i].open = false;

        if (cur->i == coarse_goal) { // found goal
            goal_found = true;
            break;
        }

        for (int i = -1; i <= 1; ++i) { // add each neighbor to queue
            for (int j = -1; j <= 1; ++j) {
                if (i == 0 && j == 0) continue; // skip self
                int neighbor_y = cur_y + i, neighbor_x = cur_x + j;
                if (neighbor_y < 0 || neighbor_x < 0 || neighbor_y >= coarse_map_width || neighbor_x >= coarse_map_width || coarse_map[neighbor_y][neighbor_x] || !coarse_map_open[neighbor_y][neighbor_x])
                    continue; // skip if neighbor is OOB or obstacle or too far from a known obstacle
                int neighbor = neighbor_y * coarse_map_width + neighbor_x;
                if (!nodes[neighbor].open && nodes[neighbor].f != DBL_MAX) continue; // neighbor already closed
                
                double direct_neighbor_cost = (i != 0 && j != 0) ? 1.414214 : 1.0; // if on diagonal cost is sqrt(2) otherwise 1
                double g = cur->g + direct_neighbor_cost; // g_score[cur->i]
                if (g < nodes[neighbor].g) {
                    pq_enqueue(pq, nodes, neighbor, g + euclidean_flat_dist(neighbor_x, neighbor_y, coarse_goal_pos[0], coarse_goal_pos[1]), true, cur->i, g, &pq_size, total_coarse_cells);
                    //open[neighbor].f = g + euclidean_flat_dist(neighbor_x, neighbor_y, coarse_goal_pos[0], coarse_goal_pos[1]);
                    //open[neighbor].open = true;
                    //parent[neighbor] = cur;
                }
            }
        }
    }


    int cur = coarse_goal;  // if the goal was reached, trace path from goal, 
    if (!goal_found) {      // otherwise, trace path from closest traversed cell by euclidean distance
        cur = closest_to_goal;
        // if is on top of or within reached proximity to this closest cell, mark the actual goal as reached (by returning true)
        // this asserts that, since the robot has reached the closest cell to the actual goal it could find a path to, and has not rerouted to find a better path, there will not be a better path
        if (closest_to_goal == -1 || euclidean_flat_dist(coarse_start_pos[0], coarse_start_pos[1], closest_to_goal % coarse_map_width, closest_to_goal / coarse_map_width) <= REACHED_DISTANCE) {
            free(pq);
            free(nodes); 
            return true;
        }
        //int new_goal_x = (closest_to_goal % coarse_map_width) * COARSE_RATIO + COARSE_RATIO / 2;
        //int new_goal_y = (closest_to_goal / coarse_map_width) * COARSE_RATIO + COARSE_RATIO / 2;
        //goal_pos[2*cur_goal+0] = new_goal_x;
        //goal_pos[2*cur_goal+1] = new_goal_y;
    }
    int next = cur;
    while (nodes[cur].parent != -1 && nodes[nodes[cur].parent].parent != -1) {    // trace path to 2nd after start     parent[cur] != -1 && parent[parent[cur]] != -1
        next = cur;
        cur = nodes[cur].parent;

        // draw each node in path on map
        int node_center_x = (next % coarse_map_width) * COARSE_RATIO + COARSE_RATIO / 2;
        int node_center_y = (next / coarse_map_width) * COARSE_RATIO + COARSE_RATIO / 2;
        map_tree[node_center_y][node_center_x] = 251;
    }
    next_coarse_pos[0] = next % coarse_map_width; // x
    next_coarse_pos[1] = next / coarse_map_width; // y

    free(pq); 
    free(nodes);
    return false;
}

void find_corners(int goal_stack[], int * stack_len, int stack_max_len, double global_trans[]) {
    int cur_coarse_map_pos[2] = {-(global_trans[0] / MAP_RATIO) + MAP_SIZE / 2 / COARSE_RATIO, -(global_trans[1] / MAP_RATIO) + MAP_SIZE / 2 / COARSE_RATIO};
    const int coarse_map_width = (MAP_SIZE / COARSE_RATIO);
    for (int i = 1; i < coarse_map_width-1; ++i) {
        for (int j = 1; j < coarse_map_width-1; ++j) {
            if (coarse_corner_map[i][j]) continue;

            int sum = 0;
            for (int k = -1; k <= 1; ++k) {
                for (int l = -1; l <= 1; ++l) {
                    if (coarse_map[i+k][j+l]) ++sum;
                }
            }

            // corners are marked if 4/9 cells occupied, and the center clear, and los check, and the 4 clear cells match 1 of 8 patterns I think look like a corner interior
            if (sum == 5 && !coarse_map[i][j] && bresenhams_line(i*coarse_map_width + j, cur_coarse_map_pos[1]*coarse_map_width + cur_coarse_map_pos[0]) && (
                (!coarse_map[i-1][j] && !coarse_map[i-1][j-1] && !coarse_map[i-1][j+1]) ||
                (!coarse_map[i+1][j] && !coarse_map[i+1][j-1] && !coarse_map[i+1][j+1]) ||
                (!coarse_map[i][j-1] && !coarse_map[i-1][j-1] && !coarse_map[i+1][j-1]) ||
                (!coarse_map[i][j+1] && !coarse_map[i-1][j+1] && !coarse_map[i+1][j+1]) ||
                (!coarse_map[i-1][j-1] && !coarse_map[i-1][j] && !coarse_map[i][j-1]) ||
                (!coarse_map[i+1][j+1] && !coarse_map[i+1][j] && !coarse_map[i][j+1]) ||
                (!coarse_map[i-1][j+1] && !coarse_map[i-1][j] && !coarse_map[i][j+1]) ||
                (!coarse_map[i+1][j-1] && !coarse_map[i+1][j] && !coarse_map[i][j-1])
            )) {
                for (int k = -1; k <= 1; ++k) {             // mark the 3x3 around as corner to avoid duplicate if skew occurs
                    for (int l = -1; l <= 1; ++l) {
                        coarse_corner_map[i+k][j+l] = true;
                    }
                }
                int node_center_x = j * COARSE_RATIO + COARSE_RATIO / 2;
                int node_center_y = i * COARSE_RATIO + COARSE_RATIO / 2;
                map[node_center_y][node_center_x] = 248;

                stack_push(goal_stack, stack_len, stack_max_len, i * coarse_map_width + j);
            }
        }
    }
}

void SLAM_run() {
    RangeScanNode * head = create_range_scan_node();
    RangeScanNode * prev = head;
    double global_trans[2] = {0};
    double global_rot = 0;
    //int rough_goals[16] = {125,60, 190,60, 190,125, 190,190, 125,190, 60,190, 60,125, 60,60}; // ^, ^>, >, v>, v, <v, <, <^
    int rough_coarse_goals[16] = {25,12, 38,12, 38,25, 38,38, 25,38, 12,38, 12,25, 12,12}; // ^, ^>, >, v>, v, <v, <, <^
    const int coarse_map_width = (MAP_SIZE / COARSE_RATIO);
    const int total_coarse_cells = coarse_map_width * coarse_map_width;
    int * goal_stack = malloc(sizeof(int) * total_coarse_cells); 
    //bool goal_reached[8] = {false, false, false, false, false, false, false, false};
    //int cur_goal = 0;
    int stack_len = 0;
    for (int i = 7; i >= 0; --i) { // add all hardcoded goals to the stack
        stack_push(goal_stack, &stack_len, total_coarse_cells, rough_coarse_goals[i*2+1]*coarse_map_width + rough_coarse_goals[i*2]);
        coarse_corner_map[rough_coarse_goals[i*2+1]][rough_coarse_goals[i*2]] = true; // add to corner map to prevent duplication as corners
    }

    
    enum {PLANNING, ROTATING, MOVING};
    int state = PLANNING;
    int next_coarse_pos[2] = {};
    double next_world_pos[2] = {};
    double target_rot = 0;
    double target_dist = 0;

    while (true) {
        if (slam_restart || slam_end) break;
        prev = SLAM_iteration(prev, global_trans, &global_rot);
        find_corners(goal_stack, &stack_len, total_coarse_cells, global_trans);
        stack_reorder_top(goal_stack, &stack_len, total_coarse_cells, global_trans);
        ++iterations;

        if (state == PLANNING) {
            memcpy(map_tree, map, sizeof(uint8_t) * MAP_SIZE * MAP_SIZE);
            double cur_map_pos[2] = {-(global_trans[0] / MAP_RATIO) + MAP_SIZE / 2, -(global_trans[1] / MAP_RATIO) + MAP_SIZE / 2};
            

            // find next node in A* path to goal on coarse_map
            while (next_coarse_path_node(cur_map_pos, goal_stack, &stack_len, next_coarse_pos)) { // keep popping goals while the robot is at the goal
                stack_pop(goal_stack, &stack_len, total_coarse_cells);
                if (stack_len <= 0) { // all the goals have been reached
                    free(goal_stack);
                    return;
                }
            }   
            int map_cell_center_x = next_coarse_pos[0] * COARSE_RATIO + 1 + (COARSE_RATIO - 1) / 2;
            int map_cell_center_y = next_coarse_pos[1] * COARSE_RATIO + 1 + (COARSE_RATIO - 1) / 2;
            next_world_pos[0] = -(map_cell_center_y - MAP_SIZE / 2) * MAP_RATIO;
            next_world_pos[1] = (map_cell_center_x - MAP_SIZE / 2) * MAP_RATIO;
            
            map_tree[(int)cur_map_pos[1]][(int)cur_map_pos[0]] = 253;   // draw start on map
            map_tree[goal_stack[stack_len-1] / coarse_map_width][goal_stack[stack_len-1] % coarse_map_width] = 253;     // draw goal on map
            //map_tree[(int)goals[2*cur_goal+1]][(int)goals[2*cur_goal+0]] = 253;     // draw goal on map
            map_tree[(int)map_cell_center_y][(int)map_cell_center_x] = 254;   // draw next node endpoint on map


            // to move towards next node endpoint, decide whether to rotate or move straight
            double goal_x = next_world_pos[0], goal_y = next_world_pos[1], cur_x = global_trans[1], cur_y = -global_trans[0];
            double rot_needed = atan2((goal_y - cur_y), (goal_x - cur_x)) * 180.0 / M_PI;
            
            // normalize both rotations
            while (rot_needed > 180.0) rot_needed -= 360.0;
            while (rot_needed < -180.0) rot_needed += 360.0;
            while (global_rot > 180.0) global_rot -= 360.0;
            while (global_rot < -180.0) global_rot += 360.0;

            double rot_relative = rot_needed - global_rot;
            while (rot_relative > 180.0) rot_relative -= 360.0;
            while (rot_relative < -180.0) rot_relative += 360.0;
            target_dist = sqrt(pow((goal_x - cur_x), 2) + pow((goal_y - cur_y), 2));//sqrt(pow((map_cell_center_x - cur_map_pos[1]), 2) + pow((map_cell_center_y - cur_map_pos[0]), 2));
            printf("goal: [%f,%f] cur: [%f,%f]\n", goal_x, goal_y, cur_x, cur_y);
            printf("dist needed: %f for rotation: %f\n", target_dist, rot_relative);
            if (fabs(rot_relative) > PLANNING_ROTATION_TOLERANCE) {
                target_rot = rot_needed;
                state = ROTATING;
            } else {
                state = MOVING;
            }
        } 
        
        if (state == ROTATING) {
            double rot_needed = target_rot - global_rot;
            while (rot_needed > 180.0) rot_needed -= 360.0;
            while (rot_needed < -180.0) rot_needed += 360.0;
            if (fabs(rot_needed) > PLANNING_ROTATION_TOLERANCE) {
                motor_rotate(rot_needed);
                //state = PLANNING; // TEMPORARY change to test if behavior is stable when planning after every rotation
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
    memset(coarse_map, 0, sizeof(bool) * max_coarse_index_length);

    // fill coarse_map with obstacles 
    for (size_t i = 0; i < MAP_SIZE; ++i) {
        for (size_t j = 0; j < MAP_SIZE; ++j) {
            if (map[i][j] != 0 && map[i][j] != 252 && map[i][j] != 250 && map[i][j] != 249 && map[i][j] != 248)
                coarse_map[i/COARSE_RATIO][j/COARSE_RATIO] = true;
        }
    }

    // inflate obstacle cells on open coarse map by 8 (each coarse cell is 7.5cm so 8 is around 2 feet)
    const int w = MAP_SIZE / COARSE_RATIO;
    bool temp[w][w];
    memcpy(temp, coarse_map, sizeof(temp));
    temp[w/2][w/2] = true; // add start position as initial nucleus of open cells
    for (int i = 0; i < w; ++i) {
        for (int j = 0; j < w; ++j) {
            if (temp[i][j]) {
                for (int d_i = -OPEN_INFLATION_CONST; d_i <= OPEN_INFLATION_CONST; ++d_i) {
                    for (int d_j = -OPEN_INFLATION_CONST; d_j <= OPEN_INFLATION_CONST; ++d_j) {
                        if (i+d_i >= 0 && i+d_i < w && j+d_j >= 0 && j+d_j < w)
                            coarse_map_open[i+d_i][j+d_j] = true;   
                    }
                }
            }
        }
    }

    
    // inflate obstacle cells by 1
    //const int w = MAP_SIZE / COARSE_RATIO;
    //bool temp[w][w];
    memcpy(temp, coarse_map, sizeof(temp));
    for (int i = 0; i < w; ++i) {
        for (int j = 0; j < w; ++j) {
            if (temp[i][j]) {
                for (int d_i= -1; d_i <= 1; ++d_i) {
                    for (int d_j= -1; d_j <= 1; ++d_j) {
                        if (i+d_i >= 0 && i+d_i < w && j+d_j >= 0 && j+d_j < w)
                            coarse_map[i+d_i][j+d_j] = true;
                    }
                }
            }
        }
    }
     


    // shade the full coarse map cells on the map
    for (int i = 0; i < w; ++i) {
        for (int j = 0; j < w; ++j) { 
            if (coarse_map_open[i][j]) { // shade open coarse cells (near obstacle)
                int k_lim = (i+1)*COARSE_RATIO;
                int l_lim = (j+1)*COARSE_RATIO;
                for (int k = i * COARSE_RATIO; k < k_lim; ++k) {
                    for (int l = j * COARSE_RATIO; l < l_lim; ++l) {
                        if (map[k][l] == 0) map[k][l] = 249;
                    }
                }
            }

            if (coarse_map[i][j]) { // shade occupied coarse cells (have obstacle)
                int k_lim = (i+1)*COARSE_RATIO;
                int l_lim = (j+1)*COARSE_RATIO;
                for (int k = i * COARSE_RATIO; k < k_lim; ++k) {
                    for (int l = j * COARSE_RATIO; l < l_lim; ++l) {
                        if (map[k][l] == 0 || map[k][l] == 249) map[k][l] = 250;
                    }
                }
            }
        }
    }
}

// 0 = check edge constraint on coarse_map, 1 = draw edge constraint on coarse_map, 2 = draw RRT on map
bool bresenhams_line(int a, int b) {
    // use bresenhams line algorithm to walk along edge on coarse map
    const int coarse_map_width = (MAP_SIZE / COARSE_RATIO);
    int a_x = a % coarse_map_width, a_y = a / coarse_map_width, b_x = b % coarse_map_width, b_y = b / coarse_map_width;
    int dx = abs(b_x - a_x), dy = -abs(b_y - a_y);
    int s_x = a_x < b_x ? 1 : -1, s_y = a_y < b_y ? 1 : -1;
    int error = dx + dy;

    while (true) {
        if (coarse_map[a_y][a_x] && (
            (a_x != a % coarse_map_width || a_y != a / coarse_map_width) &&
            (a_x != b % coarse_map_width || b_y != b / coarse_map_width)))
            return false;

        if (a_x == b_x && a_y == b_y) break;

        // step
        int e2 = 2 * error;
        if (e2 >= dy) { error += dy; a_x += s_x; }
        if (e2 <= dx) { error += dx; a_y += s_y; }
    }

    return true;
}

/*
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
        size_t random_map_index = selected_coarse_map_index * (COARSE_RATIO * COARSE_RATIO) + (rand() % (COARSE_RATIO*COARSE_RATIO));
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
*/


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
            motor_rotate(-360.0);
            manual_left = false;
        }
        if (manual_right) {
            motor_rotate(360.0);
            manual_right = false;
        }
        if (manual_forward) {
            motor_straight(200.0);
            manual_forward = false;
        }
        prev = SLAM_iteration(prev, global_trans, &global_rot);
        //find_corners();
        memcpy(map_tree, map, sizeof(uint8_t) * MAP_SIZE * MAP_SIZE);
        double cur_map_pos[2] = {-(global_trans[0] / MAP_RATIO) + MAP_SIZE / 2, -(global_trans[1] / MAP_RATIO) + MAP_SIZE / 2};
        map_tree[(int)cur_map_pos[1]][(int)cur_map_pos[0]] = 253;   // draw start on map
        ++iterations;
    }
}

void draw_coarse_gridlines() {
    const int coarse_map_width = (MAP_SIZE / COARSE_RATIO);
    for (int i = 0; i < MAP_SIZE; ++i) {
        for (int j = 0; j < MAP_SIZE; ++j) {
            if (i % COARSE_RATIO == 0 || j % COARSE_RATIO == 0) map[i][j] = 252;
        }
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
    /*
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            printf("Found device at address 0x%02X\n", addr);
        }
    }*/
    vTaskDelay(pdMS_TO_TICKS(50));
    uint8_t init_result = VL53L4CD_SensorInit(sensor);
    if (init_result != 0)
        printf("Sensor init failed %d\n", init_result);
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

    enum op_modes {AUTO, MANUAL};
    int op_mode = AUTO;
    while (1) {
        if (slam_restart) {
            slam_restart = false;
            slam_end = false;
            printf("restarting slam\n");
            iterations = 0;
            memset(map, 0, sizeof(map));
            draw_coarse_gridlines();

            gpio_set_level(GPIO_NUM_20, 0); // xshut pin on sensor
            vTaskDelay(pdMS_TO_TICKS(10));
            gpio_set_level(GPIO_NUM_20, 1);
            vTaskDelay(pdMS_TO_TICKS(10));

            SLAM_run();
            //manual_control();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    };
}
    