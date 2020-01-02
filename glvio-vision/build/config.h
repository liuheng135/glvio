#ifndef _CONFIG_H_
#define _CONFIG_H_

#define MAIN_LOOP_HZ       400

#define SCHED_DEFAULT	   SCHED_FIFO

#define GPS_UART_PATH      "/dev/ttyS2"
#define GPS_USING_UBLOX

#define LOG_PATH_ROOT	   "/mnt/"
#define LOG_COUNT_MAX	    1
#define LOG_BUF_SIZE       (10 * 1024)

#define PARAM_FILE_PATH     "/netPrivate/param"

#define USING_FLOW           1
#define FLOW_AGR_VERSION     1
#define FLOW_CAMERA_NAME     "ov7740"
#define FLOW_DELAY           (0.1f)   //  50fps ,  iir filter 10 20 400  ov7740
#define FLOW_SPEED_SCALE     (0.0035f)  //  vga  ov7740
#define FLOW_SHIFT_Z         (0.05)    //  meters
#define FLOW_RORATE_ANGLE     315.0f   //  degrees

#define RF_LOST_TIME        2.5f            // seconds

#define IMU_ROTATION	    ROTATION_PITCH_180
#define MAG_ROTATION        ROTATION_ROLL_180_YAW_90

#define LOG_ENABLE       1
#define LINK_PROTOCOL    1   /*  1 means awlink, 2 means lwlink, 3 means mavlink */

#define MAG_CALIB_STATUS    0.0f
#define ACC_CALIB_STATUS    0.0f

#define ATT_RP_SP_MAX_DEF	(15.0f / 57.0f)
#define ATT_YR_SP_MAX_DEF	2.0f

#define ATT_ROLL_P_DEF      12.0f    // 14 for 1600kv
#define ATT_ROLL_I_DEF 	    0.00f
#define ATT_ROLL_D_DEF 	    0.00f
#define ATT_ROLL_D_W_DEF    0.7f
#define ATT_ROLL_IMAX_DEF   0.0f

#define ATT_PITCH_P_DEF	    12.0f    // 14 for 1600kv
#define ATT_PITCH_I_DEF	    0.00f
#define ATT_PITCH_D_DEF	    0.00f
#define ATT_PITCH_D_W_DEF   0.7f
#define ATT_PITCH_IMAX_DEF  0.0f

#define ATT_YAW_P_DEF       2.5f
#define ATT_YAW_I_DEF       0.0f
#define ATT_YAW_D_DEF  	    0.0f
#define ATT_YAW_IMAX_DEF    0.0f

#define RATE_ROLL_P_DEF	    0.05f    // 0.1 for 1600kv
#define RATE_ROLL_I_DEF	    0.04f
#define RATE_ROLL_D_DEF	    0.004f   // 0.004 for 1600kv
#define RATE_ROLL_D_W_DEF   0.5f
#define RATE_ROLL_IMAX_DEF	0.2f

#define RATE_PITCH_P_DEF    0.05f    // 0.1 for 1600kv
#define RATE_PITCH_I_DEF    0.04f
#define RATE_PITCH_D_DEF    0.004f   // 0.004 for 1600kv
#define RATE_PITCH_D_W_DEF 	0.5f
#define RATE_PITCH_IMAX_DEF	0.2f

#define RATE_YAW_P_DEF	    0.2f
#define RATE_YAW_I_DEF	    0.03f
#define RATE_YAW_D_DEF	    0.002f
#define RATE_YAW_IMAX_DEF   0.4f

#define POS_XY_P_DEF        0.7f
#define POS_XY_I_DEF        0.0f
#define POS_XY_D_DEF        0.0f
#define POS_XY_IMAX_DEF	    0.0f

#define VEL_XY_P_DEF        3.0f
#define VEL_XY_I_DEF        1.0f
#define VEL_XY_D_DEF        0.2f
#define VEL_XY_D_W_DEF      0.5f
#define VEL_XY_IMAX_DEF	    0.4f

#define POS_Z_P_DEF         0.8f
#define POS_Z_I_DEF         0.0f
#define POS_Z_D_DEF         0.0f
#define POS_Z_IMAX_DEF      0.0f
#define POS_Z_LIMIT_DEF     20.0f

#define VEL_Z_P_DEF         0.4f
#define VEL_Z_I_DEF         0.2f
#define VEL_Z_D_DEF         0.03f
#define VEL_Z_IMAX_DEF     -0.2f
#define VEL_Z_IMIN_DEF	   -0.7f
#define VEL_Z_LIMIT_DEF		2.0f

#define ACC_Z_P_DEF         1.2f
#define ACC_Z_I_DEF         0.5f
#define ACC_Z_D_DEF         0.02f
#define ACC_Z_IMAX_DEF	    0.8f

#define MAG_OFFSET_X_DEF    0.0f
#define MAG_OFFSET_Y_DEF    0.0f
#define MAG_OFFSET_Z_DEF    0.0f
#define MAG_RADIUS_DEF      350.0f

#define ACC_CALIB_MODE_DEF	0.0f
#define ACC_OFFSET_X_DEF    0.0f
#define ACC_OFFSET_Y_DEF    0.0f
#define ACC_OFFSET_Z_DEF    0.0f
#define ACC_SCALE_X_DEF	    1.0f
#define ACC_SCALE_Y_DEF	    1.0f
#define ACC_SCALE_Z_DEF	    1.0f

#define MOTOR_MAP_0_DEF	    0.0f
#define MOTOR_MAP_1_DEF	    1.0f
#define MOTOR_MAP_2_DEF	    3.0f
#define MOTOR_MAP_3_DEF     2.0f

#define MOTOR_SPIN_DEF	    0.05f
#define MOTOR_THR_MAX_DEF   0.95f

#define MOTOR_OUT_MAX_DEF   1.00f
#define MOTOR_OUT_MIN_DEF   0.05f

#define RC_REMOTE_EN_DEF        1.0f
#define RC_DEADZONE_ROLL_DEF    0.01f
#define RC_DEADZONE_PITCH_DEF   0.01f
#define RC_DEADZONE_YAW_DEF     0.2f
#define RC_DEADZONE_THR_DEF     0.01f

#define FS_LOW_BATT_DEF         20.0f
#define FS_RC_TIMEOUT_DEF       0.5f
#define FS_ATT_LIMIT_DEF        42.0f
#define FS_IMU_TEMP_LIMIT_DEF   80.0f
#define GEOFENCE_RADIUS_DEF     100.0f 
#define GEOFENCE_ALTITUDE_DEF    80.0f

#define ATT_ROLL_OFFSET_NAME    "ATT_OFFSET_ROLL"
#define ATT_PITCH_OFFSET_NAME   "ATT_OFFSET_PITCH"
#define ATT_YAW_OFFSET_NAME     "ATT_OFFSET_YAW"
 
#define ATT_ROLL_P_NAME			"ATT_ROLL_P"
#define ATT_ROLL_I_NAME			"ATT_ROLL_I"
#define ATT_ROLL_D_NAME			"ATT_ROLL_D"
#define ATT_ROLL_D_W_NAME		"ATT_ROLL_D_W"
#define ATT_ROLL_IMAX_NAME		"ATT_ROLL_IMAX"

#define ATT_PITCH_P_NAME		"ATT_PITCH_P"
#define ATT_PITCH_I_NAME		"ATT_PITCH_I"
#define ATT_PITCH_D_NAME		"ATT_PITCH_D"
#define ATT_PITCH_D_W_NAME		"ATT_PITCH_D_W"
#define ATT_PITCH_IMAX_NAME		"ATT_PITCH_IMAX"

#define ATT_YAW_P_NAME			"ATT_YAW_P"
#define ATT_YAW_I_NAME			"ATT_YAW_I"
#define ATT_YAW_D_NAME			"ATT_YAW_D"
#define ATT_YAW_IMAX_NAME		"ATT_YAW_IMAX"

#define RATE_ROLL_P_NAME		"RATE_ROLL_P"
#define RATE_ROLL_I_NAME		"RATE_ROLL_I"
#define RATE_ROLL_D_NAME		"RATE_ROLL_D"
#define RATE_ROLL_D_W_NAME		"RATE_ROLL_D_W"
#define RATE_ROLL_IMAX_NAME		"RATE_ROLL_IMAX"

#define RATE_PITCH_P_NAME		"RATE_PITCH_P"
#define RATE_PITCH_I_NAME		"RATE_PITCH_I"
#define RATE_PITCH_D_NAME		"RATE_PITCH_D"
#define RATE_PITCH_D_W_NAME		"RATE_PITCH_D_W"
#define RATE_PITCH_IMAX_NAME	"RATE_PITCH_IMAX"

#define RATE_YAW_P_NAME			"RATE_YAW_P"
#define RATE_YAW_I_NAME			"RATE_YAW_I"
#define RATE_YAW_D_NAME			"RATE_YAW_D"
#define RATE_YAW_IMAX_NAME		"RATE_YAW_IMAX"
#define RATE_YAW_LIMIT_NAME		"RATE_YAW_LIMIT"

#define POS_XY_P_NAME			"POS_XY_P"
#define POS_XY_I_NAME			"POS_XY_I"
#define POS_XY_D_NAME			"POS_XY_D"
#define POS_XY_IMAX_NAME		"POS_XY_IMAX"

#define VEL_XY_P_NAME			"VEL_XY_P"
#define VEL_XY_I_NAME			"VEL_XY_I"
#define VEL_XY_D_NAME			"VEL_XY_D"
#define VEL_XY_IMAX_NAME		"VEL_XY_IMAX"

#define POS_Z_P_NAME			"PS_Z_P"
#define POS_Z_I_NAME			"PS_Z_I"
#define POS_Z_D_NAME			"POS_Z_D"
#define POS_Z_IMAX_NAME			"POS_Z_IMAX"
#define POS_Z_LIMIT_NAME		"POS_Z_LIMIT"

#define VEL_Z_P_NAME			"VEL_Z_P"
#define VEL_Z_I_NAME			"VEL_Z_I"
#define VEL_Z_D_NAME			"VEL_Z_D"
#define VEL_Z_IMAX_NAME			"VEL_Z_IMAX"
#define VEL_Z_IMIN_NAME			"VEL_Z_IMIN"
#define VEL_Z_LIMIT_NAME		"VEL_Z_LIMIT"
#define VEL_Z_STAB_LIMIT_NAME	"VEL_Z_STAB_LIMIT"

#define ACC_Z_P_NAME			"ACC_Z_P"
#define ACC_Z_I_NAME			"ACC_Z_I"
#define ACC_Z_D_NAME			"ACC_Z_D"
#define ACC_Z_IMAX_NAME			"ACC_Z_IMAX"

#define MAG_OFFSET_X_NAME       "MAG_OFFSET_X"
#define MAG_OFFSET_Y_NAME       "MAG_OFFSET_Y"
#define MAG_OFFSET_Z_NAME       "MAG_OFFSET_Z"
#define MAG_RADIUS_NAME         "MAG_RADIUS"
#define MAG_CALIB_STATUS_NAME   "MAG_CALIB_STATUS" 


#define ACC_CALIB_MODE_NAME		"ACC_CALIB_MODE"
#define ACC_OFFSET_X_NAME       "ACC_OFFSET_X"
#define ACC_OFFSET_Y_NAME       "ACC_OFFSET_Y"
#define ACC_OFFSET_Z_NAME       "ACC_OFFSET_Z"
#define ACC_SCALE_X_NAME		"ACC_SCALE_X"
#define ACC_SCALE_Y_NAME		"ACC_SCALE_Y"
#define ACC_SCALE_Z_NAME		"ACC_SCALE_Z"
#define ACC_CALIB_STATUS_NAME   "ACC_CALIB_STATUS" 


#define MOTOR_MAP_0_NAME		"MOTOR_MAP_0"
#define MOTOR_MAP_1_NAME		"MOTOR_MAP_1"
#define MOTOR_MAP_2_NAME		"MOTOR_MAP_2"
#define MOTOR_MAP_3_NAME		"MOTOR_MAP_3"

#define MOTOR_SPIN_NAME			"MOTOR_SPIN"
#define MOTOR_THR_MAX_NAME		"MOTOR_THR_MAX"
#define MOTOR_OUT_MAX_NAME		"MOTOR_OUT_MAX"
#define MOTOR_OUT_MIN_NAME		"MOTOR_OUT_MIN"

#define RC_REMOTE_EN_NAME		"RC_REMOTE_EN"
#define RC_DEADZONE_ROLL_NAME	"RC_DEADZONE_ROLL"
#define RC_DEADZONE_PITCH_NAME	"RC_DEADZONE_PITCH"
#define RC_DEADZONE_YAW_NAME	"RC_DEADZONE_YAW"
#define RC_DEADZONE_THR_NAME	"RC_DEADZONE_THR"

#define FS_RC_TIMEOUT_NAME		"FS_RC_TIMEOUT"
#define FS_LOW_BATT_NAME		"FS_LOW_BATT"
#define FS_ATT_LIMIT_NAME		"FS_ATT_LIMIT"
#define FS_IMU_TEMP_LIMIT_NAME	"FS_IMU_TEMP_LIMIT"

#endif
