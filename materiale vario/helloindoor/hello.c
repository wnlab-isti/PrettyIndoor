
//v_0_2015_02_06
#include "com_example_youli_helloindoor_MainActivity.h"

//v_0_2015_02_06

#include <string.h>
//#include <jni.h>
#include <android/sensor.h>
#include <android/log.h>
#include <android/looper.h>
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <assert.h>
#include <android/asset_manager.h>
#include <android/asset_manager_jni.h>


#define TAG "helloNDK"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,TAG,__VA_ARGS__)
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR,TAG,__VA_ARGS__)


// ----------------------------------------------------------------------------
// ------------------------------- C HEAD -------------------------------------
#define PI 3.14159265358979
#define D2R (0.0174532925199433)//(PI/180)
#define RM  6351887.65
#define RN  6383651.96

typedef unsigned short		UINT16;
typedef short int			INT16;
typedef signed int			INT;
typedef double				DOUBLE64;
typedef float				FLOAT32;
typedef unsigned char		UINT8;
typedef signed char			INT8;
typedef unsigned int		UINT32;
typedef int      			INT32;
typedef unsigned long long	UINT64;

//typedef enum
//{
//	SUCCESS = 0,
//	FAILURE = 1,
//} STATUS;

#define WGS84_a  (6378137)
#define WGS84_we  (0.00007292115147)
#define WGS84_e2  (0.00669438)

#ifndef SQR
#define SQR(a) ( (a)*(a) )
#endif
#define MATRIX_INVERSE_SIZE 15

//  =====================================================================================
//  ========  Settings for the data processing in the gyro_bias_tracker function ========
#define TH_MIN_T_DIFF_STEP  0.3  // 0.3 s
#define EPH_COMPARE_SIDE  3
#define LENGTH_MEMO_MA (2*EPH_COMPARE_SIDE + 1)
#define  TH_A_STEP2  9.2// 9.2
#define  TH_A_STEP1  10.5 // 9.2
#define  LENGTH_MEMO_T_STEP  10   // 20 steps
#define  LENGTH_MEMO_SENSOR_DATA  20 //100 // 100 ephs
#define  LENGTH_MEMO_POS_PDR  10 // 20 PDR STEPS




// ---- QS
#define IF_USE_CURRENT_QS          1  // ################################################

#define  EPH_DETECT_QS  10     // for Quasi-static (QS) detection
#define  TH_DETECT_QS_GYRO  (10.0 * D2R)   //Max-min

#define  TH_DETECT_QS_GYRO_NORM_20_DEG_S    (10.0 * D2R)   //Max-min

#define  TH_DETECT_QS_GYRO_NORM_5_DEG_S     (5.0 * D2R)   //Max-min
#define  N_EPH_MEMO_GYRO_REAL_QS     (20 * LENGTH_MEMO_SENSOR_DATA)


#define SL  0.6//1.2 //0.75//0.65


// New Step detection
#define  TH_MAX_MIN_VA	3.0    // Previously is 3.0
#define  N_MEMO_VA		  11
#define  I_MIN_VA_IN_MEMO  5


// --------- Reject close sensor output ----------
//#define  TH_REJECT_SENSOR     0.035
#define  TH_REJECT_GYRO_OUTPUT   0.025
#define  TH_REJECT_ACCEL_OUTPUT  0.045
#define  TH_REJECT_MAG_OUTPUT    0.045
#define  TH_REJECT_BARO_OUTPUT   0.045

// ========= Reject close sensor output ==========



// ============================================Calibration

#define SENSORS_RATE_MOBILE 20
#define N_EPH_MEMO (2*SENSORS_RATE_MOBILE)
#define N_EPH_WIN  (2*SENSORS_RATE_MOBILE)

#define  N_EPH_SMOOTH_ROLL_PITCH	20




#define IF_FEEDBACK_GYRO_BIAS_ATT  0    // 1- Only feedback 1 and 2	 // ################################################
#define IF_SEED_INI_GYRO_BIAS_ATT  1   // To be changed to a parameter for function later

#define IF_USE_MISALIGNMENT		   1	// ################################################
#define IF_PLAY_WITH_MISALIGN	   1

#define MODE_MISALIGNMENT          1   // 0-Automatic; 1-fixed
#define		FIXED_MISALIGNEMNT     (0 * D2R)



// ----------------------------------- GNSS Center ----------------------------------------
//// Initial Navigation parameters
////const DOUBLE64 INI_POS[3] = {(51+05/60+1.69/3600)*D2R, -(114+07/60+50.26/3600)*D2R, 1115};
//const DOUBLE64 ZERO_POS[3] = {0.000* D2R, 0.000* D2R, 100.00}; // West-sourth Corner
//////#define REAL_LAT	(30.525934 * D2R)     // Used for coordinate transformation
//const DOUBLE64 INI_POS[3] = {0.00007546888*D2R, 0.00*D2R, 100.00}; //{30.527872*D2R, 114.356789*D2R, 100};
////const DOUBLE64 INI_POS[3] = {51.081029 *D2R, -114.128327*D2R, 1114.000};
////const DOUBLE64 INI_POS[3] = {0.00*D2R, 0.00395/2*D2R, 100}; //{0.0055/2*D2R, 0.00395/2*D2R, 100}; //{30.527872*D2R, 114.356789*D2R, 100};
//const FLOAT32 INI_VEL[3] = {0.0, 0.0, 0.0};
//
//#define TYPE_INI_HEADING  1   // 1-Set manually
//const float INI_HEADING = 90.0 * D2R;
//
//////#define RN_COS_LON  (RN*cos(REAL_LAT))
//
//const float ini_gyro_bias_att[3] = {0.0* D2R, 0.0* D2R, 0.0* D2R};   // Unit: rad/s
//const float ini_accel_bias_att[3] = {0.0, 0.0, 0.0};   // Unit: m/ss
// ==================================== GNSS Center ========================================


// ----------------------------------- EEEL S4 ----------------------------------------
//// Initial Navigation parameters
//const DOUBLE64 ZERO_POS[3] = {51.081029 *D2R, -114.128327*D2R, 1114.000}; // West-sourth Corner
//////#define REAL_LAT	(51.081029  * D2R)     // Used for coordinate transformation
////const DOUBLE64 INI_POS[3] = {0.00007546888*D2R, 0.00*D2R, 100.00}; //{30.527872*D2R, 114.356789*D2R, 100};
//const DOUBLE64 INI_POS[3] = {51.081029 *D2R, -114.128327*D2R, 1114.000};
////const DOUBLE64 INI_POS[3] = {0.00*D2R, 0.00395/2*D2R, 100}; //{0.0055/2*D2R, 0.00395/2*D2R, 100}; //{30.527872*D2R, 114.356789*D2R, 100};
//const FLOAT32 INI_VEL[3] = {0.0, 0.0, 0.0};
//
////{0.2051* D2R, 0.4883* D2R, 1.0247* D2R}; //{-0.41811*D2R, 1.471198*D2R, 3.910519*D2R};
//
///////////////////////////////////////////////////////////////
//#define TYPE_INI_HEADING  1 //  1
//const float INI_HEADING = 0.0 * D2R;
//
//////#define RN_COS_LON  (RN*cos(REAL_LAT))
//
////const float ini_gyro_bias_att[3] = {0.2051* D2R, 0.4883* D2R, 1.0247* D2R};   // Unit: rad/s
//const float ini_gyro_bias_att[3] = {0.0650* D2R, 0.5430* D2R, 1.0336* D2R};   // Unit: rad/s
////const float ini_gyro_bias_att[3] = {-0.182206*D2R, 1.410937*D2R, 4.184422*D2R};   // Unit: rad/s
////const float ini_gyro_bias_att[3] = {-0.41811*D2R, 1.471198*D2R, 3.910519*D2R};   // Unit: rad/s
//
//const float ini_accel_bias_att[3] = {0.0, 0.0, 0.0};   // Unit: m/ss
//
//const char f_DB_bin[300] = "C:\\projects\\project_matlab\\Indoor\\SenDa2\\WIFI_DB_EEEL_20150211\\wifi_DB_matrix_0_3_2_2_100_0_0.bin";

// ==================================== EEEL S4 ========================================


//EEEL  @@@ Change place
// ----------------------------------- EEEL Xiaomi ----------------------------------------
// Initial Navigation parameters
//const DOUBLE64 ZERO_POS[3] = {51.172079 *D2R, -115.562036*D2R, 1114.000}; // West-sourth Corner
////#define REAL_LAT	(51.081029  * D2R)     // Used for coordinate transformation
//const DOUBLE64 INI_POS[3] = {0.00007546888*D2R, 0.00*D2R, 100.00}; //{30.527872*D2R, 114.356789*D2R, 100};
//const DOUBLE64 INI_POS[3] = {51.172073 *D2R, -115.562036*D2R, 1114.000};

const DOUBLE64 ZERO_POS[3] = {43.71832 *D2R, 10.42225*D2R, 1114.000}; // West-sourth Corner
const DOUBLE64 INI_POS[3] = {43.71832 *D2R, 10.42225*D2R, 1114.000};

//const DOUBLE64 INI_POS[3] = {0.00*D2R, 0.00395/2*D2R, 100}; //{0.0055/2*D2R, 0.00395/2*D2R, 100}; //{30.527872*D2R, 114.356789*D2R, 100};
const FLOAT32 INI_VEL[3] = {0.0, 0.0, 0.0};

//{0.2051* D2R, 0.4883* D2R, 1.0247* D2R}; //{-0.41811*D2R, 1.471198*D2R, 3.910519*D2R};

/////////////////////////////////////////////////////////////
#define TYPE_INI_HEADING  1 //  1
const float INI_HEADING = 0.0 * D2R;

////#define RN_COS_LON  (RN*cos(REAL_LAT))

//const float ini_gyro_bias_att[3] = {0.2051* D2R, 0.4883* D2R, 1.0247* D2R};   // Unit: rad/s
const float ini_gyro_bias_att[3] = {0* D2R, 0* D2R, 0* D2R};   // Unit: rad/s	 // ################################################
//const float ini_gyro_bias_att[3] = {0.0650* D2R, 0.5430* D2R, 1.0336* D2R};   // Unit: rad/s
//const float ini_gyro_bias_att[3] = {-0.182206*D2R, 1.410937*D2R, 4.184422*D2R};   // Unit: rad/s
//const float ini_gyro_bias_att[3] = {-0.41811*D2R, 1.471198*D2R, 3.910519*D2R};   // Unit: rad/s

const float ini_accel_bias_att[3] = {0.0, 0.0, 0.0};   // Unit: m/ss

const char f_DB_bin[300] = "C:\\projects\\project_matlab\\Indoor\\SenDa2\\WIFI_DB_EEEL_20150401\\wifi_DB.bin";

// ==================================== EEEL Xiaomi ========================================



// ----------------------------------- Parking ----------------------------------------
//// Initial Navigation parameters
//const DOUBLE64 ZERO_POS[3] = {30 *D2R, 110*D2R, 0}; // West-sourth Corner
////// #define REAL_LAT	(51.081029  * D2R)     // Used for coordinate transformation
////const DOUBLE64 INI_POS[3] = {0.00007546888*D2R, 0.00*D2R, 100.00}; //{30.527872*D2R, 114.356789*D2R, 100};
//const DOUBLE64 INI_POS[3] = {30 *D2R, 110*D2R, 0};
////const DOUBLE64 INI_POS[3] = {0.00*D2R, 0.00395/2*D2R, 100}; //{0.0055/2*D2R, 0.00395/2*D2R, 100}; //{30.527872*D2R, 114.356789*D2R, 100};
//const FLOAT32 INI_VEL[3] = {0.0, 0.0, 0.0};
//
////{0.2051* D2R, 0.4883* D2R, 1.0247* D2R}; //{-0.41811*D2R, 1.471198*D2R, 3.910519*D2R};
//
///////////////////////////////////////////////////////////////
//#define TYPE_INI_HEADING  1 //  1
//const float INI_HEADING = 0.0 * D2R;
//
//////#define RN_COS_LON  (RN*cos(REAL_LAT))
//
////const float ini_gyro_bias_att[3] = {0.2051* D2R, 0.4883* D2R, 1.0247* D2R};   // Unit: rad/s
//const float ini_gyro_bias_att[3] = {0* D2R, 0* D2R, 0* D2R};   // Unit: rad/s	 // ################################################
////const float ini_gyro_bias_att[3] = {0.0650* D2R, 0.5430* D2R, 1.0336* D2R};   // Unit: rad/s
////const float ini_gyro_bias_att[3] = {-0.182206*D2R, 1.410937*D2R, 4.184422*D2R};   // Unit: rad/s
////const float ini_gyro_bias_att[3] = {-0.41811*D2R, 1.471198*D2R, 3.910519*D2R};   // Unit: rad/s
//
//const float ini_accel_bias_att[3] = {0.0, 0.0, 0.0};   // Unit: m/ss
//
//const char f_DB_bin[300] = "C:\\projects\\project_matlab\\Indoor\\SenDa2\\Parking1\\WIFI_DB_PARKING_XIAOMI4\\wifi_DB_matrix_0_31_2_2_100_0_1.bin";
////const char f_DB_bin[300] = "C:\\projects\\project_matlab\\Indoor\\SenDa2\\WIFI_DB_PARKING_20150515\\wifi_DB_all_DB.bin";

// ==================================== Parking ========================================

#define	 REAL_LAT  (ZERO_POS[0])
#define	 RN_COS_LON		(RN*cos(REAL_LAT))


// KF parameters
// Q
const FLOAT32 tao_bg[3] = {10000, 10000, 10000};   // Correlation time for GM

//// P0
//const FLOAT32 ini_att_var[3] = {10*D2R, 10*D2R, 30*D2R};
//const FLOAT32 ini_bg_var[3] = {0.3*D2R, 0.3*D2R, 0.3*D2R};

// P0
const FLOAT32 ini_att_var[3] = {10*D2R, 10*D2R, 10*D2R};
const FLOAT32 ini_bg_var[3] = {0.1*D2R, 0.1*D2R, 0.1*D2R};



#define TYPE_INITIALIZATION  0    // 100 or 0

//  =============================================================================
//  =============================================================================
//  ------------- FOR WIFI -------------
#define  IF_USE_WIFI    1
#define  IF_DIFF_WIFI   0


#define  NUM_WIFI_DB    4



#define  IF_CODE_WIFI   0

#define  IF_DATABASE    0

#define  N_KNN   4
#define  TH_DIST_REJECT_NN     20.0
#define  TH_DIST_REJECT_NN_2   400.0
#define  MAX_DB  100
#define  MAX_RP_NUM  500
#define  MAX_AP_NUM  300   //
#define  N_CHAR_MAC  17
#define  RSS_DB_4_ZERO  100

#define  N_MEMO_WIFI	2
#define  LENGTH_MEMO_RSS   2

#define SIGMA_WIFI   15.0
//  ============= FOR WIFI =============


//  ------------- FOR MM RESULT -------------
#define  IF_USE_MM_RESULT   0

#define SIGMA_MM_RESULT   10.0

#define IF_USE_AEKF_MM     0
//  ============= FOR MM RESULT =============


//  ------------- FOR GPS RESULT -------------
//  ------------- FOR GPS RESULT -------------
#define  IF_USE_GPS_RESULT  1
#define SIGMA_GPS_RESULT   15.0
#define TH_ACCUR_RELIABLE_GPS 15.1
#define TH_ACCUR_VERY_RELIABLE_GPS 7.1    // Directly set the position as GPS location

#define IF_GPS_DEFEAT             1
#define TH_ACCUR_GPS_DEFEAT_WIFI 16.1
#define TIME_DEFEAT_WIFI_LAST  5
#define SIGMA_WIFI_DEFEATED  (SIGMA_WIFI * 10)
#define SIGMA_MM_RESULT_DEFEATED  (SIGMA_MM_RESULT * 10)

#define IF_GPS_CALIBRATE_HEIGHT        1
#define TH_ACCUR_GPS_CALIBRATE_HEIGHT  6.3
#define TH_EPH_GPS_CALI_HEIGHT         5

//  ============= FOR GPS RESULT =============


//  ============= FOR GPS RESULT =============


// -------------------- Moved to place part -------------
//const char f_DB_bin[300] = "C:\\projects\\project_matlab\\Indoor\\SenDa2\\Parking1\\WIFI_DB_PARKING_XIAOMI4\\wifi_DB_matrix_0_54_1_1_100_0_0.bin";
//const char f_DB_bin[300] = "C:\\projects\\project_matlab\\Indoor\\SenDa2\\WIFI_DB_EEEL_20150211\\wifi_DB_matrix.bin";
// ==================== Moved to place part =============

#define  N_CHAR_BUFF  (MAX_AP_NUM * 10)

//P0
#define N_STATES  3
#define INI_POS_VAR   5.0
#define INI_HEAD_VAR  (10.0 *D2R)
#define INI_SL_VAR   0.5


// Qk
//#define QK_POS   0.5   // <============= 5 m
//#define QK_HEAD  (1.0 * D2R)
//#define QK_POS   0.5   // <============= 5 m
//#define QK_HEAD  (0.1 * D2R)
//#define QK_POS   0.5   // <============= 5 m
//#define QK_HEAD  (1 * D2R)

//#define QK_POS   0.5  // <============= 5 m
//#define QK_HEAD  (2 * D2R)

//#define QK_POS   0.5  // <============= 5 m
//#define QK_HEAD  (1 * D2R)


#define QK_POS_PHASE_1    1 // 0.5  // <============= 5 m
#define QK_HEAD_PHASE_1  (3 * D2R)   // (1 * D2R)

#define T_BETWEEN_PHASE_12   10    // 10 sec

#define QK_POS_PHASE_2   1  // <============= 5 m
#define QK_HEAD_PHASE_2  (3 * D2R)

#define QK_SL  0.5



//--------------------- new for map
#define IF_ALIGN_TO_LINKS		0			// ################################################
#define TH_HEAD_CHANGE_MAX_IS_LINE  (15.0 * D2R)
#define TH_HEAD_AIGN_TO_LINK_MAX    (40.0 * D2R)
#define MAX_FLOOR_LINE_NUM  100

#define ANG_RAD_0		 0.0
#define ANG_RAD_90		 (90.0 * D2R)
#define ANG_RAD_NEG_90   (-90.0 * D2R)
#define ANG_RAD_180		 (180.0 * D2R)
#define ANG_RAD_NEG_180  (-180.0 * D2R)

#define AP_4MAP 10.0
#define AH_4MAP  5.0
#define WSH_NOT_SAME_ANG  -9999.0
#define TH_MAX_DIFF_ANG_PROJECT_TO_LINK  (45.0 * D2R)
#define TH_MIN_DIFF_ANG_PROJECT_TO_LINK  (1.0 * D2R)
#define TH_MAP_DIST_POINTS_REJECT   5.0



#define IF_PROJECT_TO_LINK  0		 // ################################################
#define N_COL_DB_FP  8
#define N_ROW_DB_FP  8    // ################ will be removed later ################
const char f_DB_fp_txt[200] = "C:\\projects\\project_matlab\\Indoor\\SenDa2\\fp_center_NE.txt";
const char string_FP[N_ROW_DB_FP][100] = {"1 1 6 90.0 8.2 0.0 8.2 21.9", "2 7 8 180.0 15.6 1.3 0.2 1.3", "3 9 10 180.0 15.6 10.3 0.2 10.3", "5 5 12 180.0 8.2 15.9 0.2 15.9", "1 6 1 -90.0 8.2 21.9 8.2 0.0", "2 8 7 0.0 0.2 1.3 15.6 1.3", "3 10 9 0.0 0.2 10.3 15.6 10.3", "5 12 5 0.0 0.2 15.9 8.2 15.9"};




// ----------------------------- Varies Attitude and Misalignment -------------------------------------
#define TH_LARGE_ACC 10.0

#define STATIC static

//#define COM_WALK_SIMPLIFIED
#define PRINT_RESULTS_MISALIGNMENT
#define METHOD_RESULTS_MISALIGNMENT

#define SENSORS_RATE_MOBILE 20

// ===== General data defs =====//
typedef unsigned short		UINT16;
typedef short int			INT16;
typedef signed int			INT;
typedef double				DOUBLE64;
typedef float				FLOAT32;
typedef unsigned char		UINT8;
typedef signed char         INT8;
typedef unsigned int		UINT32;
typedef int      			INT32;

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

typedef enum
{
	SUCCESS = 0,
	FAILURE = 1,
	INITIALIZATION_ERROR = 2,
	POSITION_ERROR = 3,
	INVALID_GPS_UPDATE = 4,
	INVALID_BARO_UPDATE = 5,
	INVALID_MAG_UPDATE = 6,
	INVALID_MULTIANTENNA_UPDATE = 7,
	INVALID_WIFI_UPDATE = 8,
	NO_WIFI_UPDATE = 9,
	INVALID_ZUPT_UPDATE = 10,
	INVALID_NHC_UPDATE = 11,
	INVALID_GPSPDR_UPDATE = 12,
	INVALID_PDR_UPDATE = 13,
	ZUPT_CALIBRATION_DONE,
	ZUPT_NOT_DONE
} STATUS;

typedef enum
{
	NOVATELL = 0,
	UBLOX = 1
} GPS_RECEIVER_TYPE;


typedef enum
{
	EMPTY = 0,
	READY = 1
} BUSDATA_STATUS;

typedef enum
{
	TPI_ENCLOSURE_ORIENTATION = 0,
	IBETOR_ENCLOSURE_ORIENTATION = 1
} ENCLOSURE_ORIENTATION;

#ifndef NStates
#define NSTates (23)
#endif

#ifndef OBSNUM
#define OBSNUM (3+3)
#endif
// =====

// ===== Constant ===== //
#define PIf			(FLOAT32)(3.14159265358979)
#define TWO_PIf		(FLOAT32)(6.28318530717958)
#define PI_BY_TWOf	(FLOAT32)(1.5707963267949)
#define D2Rf		(FLOAT32)(0.0174532925199433)
#define R2Df		(FLOAT32)(57.2957795130823)
#define ONE_BY_DATA_SIZE_MIS_MEAN (FLOAT32)(0.027777777777778)
#define ONE_BY_DATA_SIZE_MIS_STD (FLOAT32)(0.028571428571429)
#define ONE_BY_WINDOW_SIZE_MIS_MEAN (FLOAT32)(0.025)


#define SECONDS_IN_ONE_WEEK (604800)

#define MachNumber  (340.0)              // 340.0 m/s
#define Constant_Gravity  (FLOAT32)(9.8)

#define WGS84_a  (6378137)
#define WGS84_we  (0.00007292115147)
#define WGS84_e2  (0.00669438)

#ifndef MAX
#define MAX(a,b) ( ( (a)>(b) ) ? (a) : (b) )
#endif

#ifndef MIN
#define MIN(a,b) ( ( (a)<(b) ) ? (a) : (b) )
#endif

#ifndef MAX_3Var
#define MAX_3Var(a,b,c) MAX(MAX(a,c),b)
#endif

#ifndef SQR
#define SQR(a) ( (a)*(a) )
#endif



// ======================= LY
#define IF_USE_ATT_FILTER		   1     // 1 - Use attitude filter; 0-Accel + gyro

#define IF_AXIS_LEVELING_ATT	   1

#define NSTATE_ATT  6
#define MATRIX_INVERSE_SIZE 15
#define PI 3.14159265358979
#define D2R (0.0174532925199433)//(PI/180)
#define Norm_Gravity_ATT  (FLOAT32)(9.8)

//# define INI_HEAD_ATT  (180.0 *D2R )
//# define FLAG_INI_HEAD_ATT   1    // 0-0.0; 1-set manually, 2-use mag-heading


// ------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------
// ------------------------------------ Set of Gyro parameters ------------------------------------------------
// ------- Config 1 (S3) --------
//const FLOAT32 ARW_q_att[3] = {0.3 * D2R, 0.3  * D2R, 0.3  * D2R};
//const FLOAT32 q_bg_att[3] =  {0.01* D2R, 0.01 * D2R, 0.01 * D2R};
//const FLOAT32 tao_bg_att[3] = {3600, 3600, 3600};   // Correlation time for GM, sec
//const float AccelAccuracy_att[3] = {10.0, 10.0, 10.0};
//const float MagAccuracy_att[3] = {0.1, 0.1, 0.1};             //G
//# define ACCURACY_ROLL_PITCH   (100.0 * D2R)
//# define COV_ACCURACY_ROLL_PITCH   (ACCURACY_ROLL_PITCH * ACCURACY_ROLL_PITCH)

// ------- Config 2 (S4) --------
const FLOAT32 ARW_q_att[3] = {3 * D2R, 3  * D2R, 3  * D2R};
const FLOAT32 q_bg_att[3] =  {0.1* D2R, 0.1 * D2R, 0.1 * D2R};
const FLOAT32 tao_bg_att[3] = {200, 200, 200};   // Correlation time for GM, sec
const float AccelAccuracy_att[3] = {100.0, 100.0, 100.0};
const float MagAccuracy_att[3] = {0.1, 0.1, 0.1};             //G
# define ACCURACY_ROLL_PITCH   (500.0 * D2R)
# define COV_ACCURACY_ROLL_PITCH   (ACCURACY_ROLL_PITCH * ACCURACY_ROLL_PITCH)

// ==================================== Set of Gyro parameters ================================================



// ------------------------------------------------------------------------------------------------------------
const FLOAT32 sacle_Q_att = 1.0;

const FLOAT32 ini_att_var_att[3] = {20 * D2R, 20 * D2R, 120 * D2R};
const FLOAT32 ini_bg_var_att[3]  = {3 * D2R, 3 * D2R, 3 * D2R};

#define TH_AXIS_LEVELING_ATT	 (70.0 *D2R)

// ---------------- Misalignment --------------------

#define PEAKS_VECTOR_SIZE_MIS 20
#define DATA_SIZE_MIS 36
#define WINDOW_SIZE_MIS 40
#define TIME_SHIFT_MIS 2
#define NFFT_FFT_MIS 64  // FFT

#define FILTER_ORDER_MIS    4
#define FS_LP_FILTER_MIS    20
#define FC_LP_FILTER_MIS    5
#define OVERLAP_MISA_MIS    1

#define N_MEM_misO_SENSOR_DATA_MIS  40
#define N_MEM_misO_1_STEP_MIS  10


// ------------- Play with Misalignment
#define ANG_0_RAD   0
#define ANG_90_RAD  (90 * D2R)
#define ANG_180_RAD  (180 * D2R)
#define ANG_NEG_90_RAD  (-90 * D2R)
#define ANG_NEG_180_RAD  (-180 * D2R)

#define ANG_60_RAD  (60 * D2R)
#define ANG_120_RAD  (120 * D2R)
#define ANG_NEG_60_RAD  (-60 * D2R)
#define ANG_NEG_120_RAD  (-120 * D2R)


#define N_SM_MISALIGN  40
#define N_SM_MISALIGN_POCKET  40
#define TH_ROLL_EAR  (45 * D2R)
#define TH_DIFF_MIS_HANDHELD  (40 * D2R)

#define ANG_FOR_SM_MISA_135    (135 * D2R)



// ------------------------------ Barometer -------------------------------------
#define IF_USE_STAIR_CONSTRAINT		0

#define I_START_BARO_DATA	    5

#define N_EPH_SMOOTH_BARO		6
#define N_EPH_BARO_DIFF			6
#define N_MEMO_BARO_CHANGE		4
#define N_MEMO_IF_STAIR			4
#define N_MEMO_STATUS_STAIR		4
#define N_MEMO_FLOOR_NUM		4

#define TH_DIFF_BARO_STEP		1
#define TH_DETECT_STAIR		  0.3


#define TH_FLAT_FLOOR_CHANGE_HEIGHT0  0.3

#define INI_FLOOR_NUM    12
// ============================== Barometer =====================================



// ----------------------------- Magnetometer -----------------------------------
#define IF_USE_MAG_UPDATE	   0

#define IF_USE_QSMF_LIMIT     1

#define ACCURACY_HEADING_MAG  (20.0*D2R)
# define COV_ACCURACY_HEADING   (ACCURACY_HEADING_MAG * ACCURACY_HEADING_MAG)

#define MODE_MAG_UPDATE		   0
#define N_EPH_MAG_DATA         10

#define I_START_MAG_DATA	   5					// ~~~~~~~~~~~~
#define INI_DECLINATION        (15.0 * D2R)

#define TH_DETECT_QSMF       0.05
#define N_EPH_MEMO_NORM   (20 * 3)

#define TH_DETECT_QSMF_HEADING_MAG       (20.0 * D2R)

#define N_EPH_HEADING_MAG     40
#define N_EPH_MEMO_IND_QSMF   10

//#define MAG_ACCURACY		 0.05	// Unit - G [The meas vector has been normed!]   UP

#define MAG_N00_N            0.2
#define MAG_N00_E            0
#define MAG_N00_D            0.5

#define TH_TIME_DIFF_USE_ACC_MAG     0.05
// ============================= Magnetometer ====================================



// ========================== End of Varies Attitude and Misalignment ==================================



//  =============================================================================
//  ========  Parameters NOT related with the position library  ========
// ====================================================================================
//// Parameters for R and Q Matrix (3 conditions)
//// Condition 0: Large dynamic
//const FLOAT32 accelAccuracy0[3] = {1.0, 1.0, 1.0};		    //m/s^2
//const FLOAT32 magAccuracy0[3] = {30, 30, 30};				//mG
//const FLOAT32 q_ARW0[3] =  {0.3*D2R, 0.3*D2R, 0.3*D2R} ;
//const FLOAT32 q_bg0[3] =   {0.001*D2R, 0.001*D2R, 0.001*D2R};
//// Condition 1: Small dynamic, straight line or static
//const FLOAT32 accelAccuracy1[3] = {0.3, 0.3, 0.3};
//const FLOAT32 magAccuracy1[3] = {10, 10, 10};
//const FLOAT32 q_ARW1[3] =  {0.3*D2R, 0.3*D2R, 0.3*D2R} ;
//const FLOAT32 q_bg1[3] =   {0.001*D2R, 0.001*D2R, 0.001*D2R};
//// Condition 2: Small dynamic, turning
//const FLOAT32 accelAccuracy2[3] = {0.3, 0.3, 0.3};
//const FLOAT32 q_ARW2[3] =  {0.3*D2R, 0.3*D2R, 0.3*D2R} ;
//const FLOAT32 q_bg2[3] =   {0.001*D2R, 0.001*D2R, 0.001*D2R};

// Parameters for R and Q Matrix (3 conditions)
// Condition 0: Large dynamic
#define NStates 6

const FLOAT32 accelAccuracy0[3] = {3.0, 3.0, 3.0};		    //m/s^2
const FLOAT32 magAccuracy0[3] = {60, 60, 60};				//mG
const FLOAT32 q_ARW0[3] =  {0.3*D2R, 0.3*D2R, 0.3*D2R} ;
const FLOAT32 q_bg0[3] =   {0.01*D2R, 0.01*D2R, 0.01*D2R};
// Condition 1: Small dynamic, straight line or static
const FLOAT32 accelAccuracy1[3] = {0.3, 0.3, 0.3};
const FLOAT32 magAccuracy1[3] = {1, 1, 1};
const FLOAT32 q_ARW1[3] =  {0.3*D2R, 0.3*D2R, 0.3*D2R} ;
const FLOAT32 q_bg1[3] =   {0.01*D2R, 0.01*D2R, 0.01*D2R};

//const FLOAT32 accelAccuracy1[3] = {3.0, 3.0, 3.0};		    //m/s^2
//const FLOAT32 magAccuracy1[3] = {60, 60, 60};				//mG
//const FLOAT32 q_ARW1[3] =  {0.3*D2R, 0.3*D2R, 0.3*D2R} ;
//const FLOAT32 q_bg1[3] =   {0.01*D2R, 0.01*D2R, 0.01*D2R};

// Condition 2: Small dynamic, turning
const FLOAT32 accelAccuracy2[3] = {0.3, 0.3, 0.3};
const FLOAT32 q_ARW2[3] =  {0.3*D2R, 0.3*D2R, 0.3*D2R} ;
const FLOAT32 q_bg2[3] =   {0.01*D2R, 0.01*D2R, 0.01*D2R};

//const FLOAT32 accelAccuracy2[3] = {3.0, 3.0, 3.0};		    //m/s^2
//const FLOAT32 q_ARW2[3] =  {0.3*D2R, 0.3*D2R, 0.3*D2R} ;
//const FLOAT32 q_bg2[3] =   {0.01*D2R, 0.01*D2R, 0.01*D2R};

// Threshold values
//Detection of strategy
const FLOAT32 th_acc_std = 0.5;			//Detection of slow dynamic
const FLOAT32 th_turn_ang = 30.0 * D2R;	//Detection of turning
//
//const FLOAT32 th_norm_a_accel = 1.0;     // if the norm > this value, then reject accelerometer updates, m/s^2
//const FLOAT32 th_detect_QSMF = 50.0;     // Detection of Quasi-static magnetic field, mG

const FLOAT32 th_norm_a_accel0 = 1.0;     // if the norm > this value, then reject accelerometer updates, m/s^2
const FLOAT32 th_norm_a_accel1 = 1.0;     // if the norm > this value, then reject accelerometer updates, m/s^2
const FLOAT32 th_norm_a_accel2 = 1.0;     // if the norm > this value, then reject accelerometer updates, m/s^2

const FLOAT32 th_detect_QSMF0 = 20.0;     // Detection of Quasi-static magnetic field, mG
const FLOAT32 th_detect_QSMF1 = 50.0;     // Detection of Quasi-static magnetic field, mG
const FLOAT32 th_detect_QSMF2 = 50.0;     // Detection of Quasi-static magnetic field, mG


//  =============================================================================
//  ========  structures and functions ===========
typedef struct
{
	int type_initializaiton;   // 1-manually; 0-Use the value defined

	double ini_pos[3];   // Unit: rad, rad, m
	float ini_heading_platform;	 // Unit: rad


}Initialization_Pkt;


typedef struct
{
	// ---------------- Sensor data -----------------------
	int i_accel;
	double t_accel[LENGTH_MEMO_SENSOR_DATA];
	float a_x[LENGTH_MEMO_SENSOR_DATA];
	float a_y[LENGTH_MEMO_SENSOR_DATA];
	float a_z[LENGTH_MEMO_SENSOR_DATA];

	int i_gyro;
	double t_gyro[LENGTH_MEMO_SENSOR_DATA];
	float g_x[LENGTH_MEMO_SENSOR_DATA];
	double g_y[LENGTH_MEMO_SENSOR_DATA];
	double g_z[LENGTH_MEMO_SENSOR_DATA];

	int i_mag;
	double t_mag[N_EPH_MAG_DATA];
	float m_x[N_EPH_MAG_DATA];
	float m_y[N_EPH_MAG_DATA];
	float m_z[N_EPH_MAG_DATA];
	float heading_mag[N_EPH_HEADING_MAG];
	float normm[N_EPH_MEMO_NORM];
	UINT8 ind_qsmf[N_EPH_MEMO_IND_QSMF];

	int i_baro;
	double t_baro[N_EPH_SMOOTH_BARO];
	FLOAT32 baro[N_EPH_SMOOTH_BARO];
	FLOAT32 baro_sm[N_EPH_BARO_DIFF];
	FLOAT32 baro_change[N_MEMO_BARO_CHANGE];
	INT8 if_stair[N_MEMO_IF_STAIR];
	INT8 status_stair[N_MEMO_IF_STAIR];
	INT8 floor_num[N_MEMO_IF_STAIR];

	UINT8 i_memo_baro;
	UINT8 i_memo_baro_sm;
	UINT8 i_memo_baro_change;
	UINT8 i_memo_if_stair;
	UINT8 i_memo_status_stair;
	UINT8 i_memo_floor_num;
	UINT8 i_memo_normm;
	UINT8 i_memo_indi_qsmf;
	UINT8 i_memo_heading_mag;
	UINT16 i_memo_norm_gyro_QS;
	// UINT8 i_memo_if_vertical_gyro;  --->use i_gyro

	// ================= Sensor data ========================
	double gz_level[LENGTH_MEMO_SENSOR_DATA];
	float norm_gyro[LENGTH_MEMO_SENSOR_DATA];
	float heading_platform[LENGTH_MEMO_SENSOR_DATA];

	//INT8  memo_if_vertical_gyro[LENGTH_MEMO_SENSOR_DATA];
	float norm_gyro_QS_x[N_EPH_MEMO_GYRO_REAL_QS];
	float norm_gyro_QS_y[N_EPH_MEMO_GYRO_REAL_QS];
	float norm_gyro_QS_z[N_EPH_MEMO_GYRO_REAL_QS];

	int i_ma;
	float ma[LENGTH_MEMO_MA];

	int i_va;
	float va[N_MEMO_VA];
	double t_va[N_MEMO_VA];


	int i_t_step;
	double t_step[LENGTH_MEMO_T_STEP];

	int i_pos_pdr;
	double t_pos_pdr[LENGTH_MEMO_POS_PDR];
	double pos_pdr_lat[LENGTH_MEMO_POS_PDR];
	double pos_pdr_lon[LENGTH_MEMO_POS_PDR];
	double pos_pdr_hei[LENGTH_MEMO_POS_PDR];

	FLOAT32 heading_step[LENGTH_MEMO_POS_PDR];
	int i_memo_heading_step;

	// ------------- Misalignment ------------------
	int i_misa;
	FLOAT32 misalign[N_SM_MISALIGN];

	FLOAT32 fixed_misalignment;
	// baro
	float height_baro0;
	float height_pre;

} MemoIndoor;

typedef struct
{
	int rss[LENGTH_MEMO_RSS][MAX_AP_NUM];   //UINT8
	int i_rss;
	int i_memo_wifi;
	double pos_wifi_lat[N_MEMO_WIFI];
	double pos_wifi_lon[N_MEMO_WIFI];

	int N_wifi_pre;      // Used for deleting of un-updated rss
	float sum_rss_ap_pre;

} MemoIndoorWiFi;

typedef struct
{
	double pos[3];
	float heading_platform;           // ================= Platform heading !
	float heading_step;
	float sl;

	double t_step1;
	double t_step0;
	double t_s_step;

	float heading_device;

	// --------------- for height baro
	FLOAT32 height_baro;
	FLOAT32 height_baro_sm;
	FLOAT32 height_change;
	FLOAT32 flag_stair;
	FLOAT32 mean_if_stair;
	INT8 indi_status_stair;
	INT8 if_change_floor;

	FLOAT32 diff_height_to_height0;

	int floor_num;

	// --------------- for mag heading
	FLOAT32 heading_mag;
	FLOAT32 declination;

	//MemoIndoor memo;
} NavIndoor;


typedef struct
{
	FLOAT32 *pP[N_STATES];
	FLOAT32 *pQk[N_STATES];
	FLOAT32 P[N_STATES][N_STATES];
	FLOAT32 Qk[N_STATES][N_STATES];

} KF_4;




typedef struct
{
	int count_rec_step;
	int count_rec_pos;
	int count_rec_heading;
	//int count_rec_pos_wifi;

	FILE* f_out_rec_pos;
	FILE* f_out_rec_step;
	FILE* f_out_rec_heading;

} RecSol;

typedef struct
{
	int avail_sol_pdr;
	double sol_t_sol_pdr;
	double sol_pdr[3];
	double sol_pdr_NEH[3];

	int avail_sol_wifi;
	double sol_t_sol_wifi;
	double sol_wifi[3];

	int avail_sol_mm_result_used;
	double sol_t_sol_mm_result_used;
	double sol_mm_result_used[3];

	 int avail_sol_gps_result_used;
     double sol_t_sol_gps_result_used;
     double sol_gps_result_used[4];

	int avail_step;
	double sol_t_step;
	double sol_step;

	int avail_heading;
	double sol_t_heading;
	double sol_heading;

	int avail_heading_step;
	double sol_t_heading_step;
	double sol_heading_step;

	// -------------------- Baro -------------------------
	int avail_has_baro_in_step;
	double sol_t_height_baro;
	FLOAT32 height_baro_sm;
	FLOAT32 height_change;
	int flag_stair;
	int indi_status_stair;
	int floor_num;

	float height0;
	//  =================== Baro =========================


	// -------------------- Mag -------------------------
	int avail_heading_mag;
	double sol_t_heading_mag;
	FLOAT32 m_b[3];
	FLOAT32 heading_mag;
	FLOAT32 declination;
	INT8 indi_qsmf;

	//  =================== Mag =========================


	// -------------------- QS --------------------------
	UINT8 avail_show_qs;
	DOUBLE64 sol_t_qs;
	FLOAT32 norm_gyro;
	FLOAT32 diff_norm_gyro;
	UINT8 if_qs;
	UINT16 i_memo_norm_gyro_QS;
	// ==================== QS ==========================



	// ----------- Attitude and Misalignment -------------
	int avail_att_mis;
	DOUBLE64 sol_t_att_mis;
	FLOAT32 sol_att[3];
	INT8 if_vertical;
	FLOAT32 sol_bg_estimated[3];


	int avail_rp_accel;
	DOUBLE64 sol_t_rp_accel;
	float	sol_rp_accel[2];
	FLOAT32 sol_rp_accel_sm[2];


	// ---
	FLOAT32 misalign_az;
	INT8 motion_mode;
	INT8 fb_detect;
	FLOAT32 step_frequency;

	FLOAT32 misalignment_show;
	FLOAT32 misalignment_used;
	FLOAT32 heading_platform;

	// =========== Attitude and Misalignment =============


} Sol_Pkt;


typedef struct
{
    UINT8 i_memo1;
	DOUBLE64 t[N_EPH_MEMO];
	FLOAT32 normm[N_EPH_MEMO];
	FLOAT32 gyro[N_EPH_MEMO][3];
	FLOAT32 accel[N_EPH_MEMO][3];
	FLOAT32 gz_leveled[N_EPH_MEMO];
	FLOAT32 turn_ang[N_EPH_MEMO];

	FLOAT32 ind_QSMF[N_EPH_MEMO];
	FLOAT32 ind_large_accl[N_EPH_MEMO];
	FLOAT32 ind_turn_ang[N_EPH_MEMO];
	UINT8 i_win;

	UINT8 i_memo2;
	FLOAT32 std[N_EPH_MEMO];
	UINT8 ind_std[N_EPH_MEMO];
} Memo;

typedef struct
{
	FLOAT32 norma[N_EPH_WIN];

} Wind;

typedef struct
{
	DOUBLE64 t0;
	DOUBLE64 t;
	UINT32 i_da;

	FLOAT32 qbn[4];
	FLOAT32 mag_n00[3];

	DOUBLE64 pos[3];

	FLOAT32 *pP[NStates];
	FLOAT32 *pQ[6];
	FLOAT32 P[NStates][NStates];
	FLOAT32 Q[6][6];

	Memo memo;
	Wind win;
	FLOAT32 sol [1+NStates*2+6+1+2];
} Nav;


typedef struct
{
	// Gyro
	DOUBLE64 t_gyro;
	FLOAT32 da_gyro[3];
	UINT8 avail_gyro;

	//Accel
	DOUBLE64 t_accel;
	FLOAT32 da_accel[3];
	UINT8 avail_accel;

	//Baro
	DOUBLE64 t_baro;
	FLOAT32 da_baro;
	UINT8 avail_baro;

	// Mag
	DOUBLE64 t_mag;
	FLOAT32 da_mag[3];
	UINT8 avail_mag;

	//WiFi
	char name_ap[MAX_AP_NUM][N_CHAR_MAC];
	float rss_ap[MAX_AP_NUM];
	int N_wifi;
	double t_wifi;
	UINT8 avail_wifi;

	//MM result
	double lon_mm_result;
	double lat_mm_result;
	double t_mm_result;
	UINT8 avail_mm_result;

	//GPS result
	double lon_gps_result;
	double lat_gps_result;
	float height_gps_result;
	float accuracy_gps_result;
	double t_gps_result;
	UINT8 avail_gps_result;

} Meas;




// ----------------------------- Structure Attitude and Misalignment -------------------------------------

/////////////////////////////////  Start of Structures Definition ////////////////////////////////
// PCA Main and Classification1
typedef struct _PCA
{
	// Data parameters
	FLOAT32 motion_parametrs[3];				// 0 mean,	1 std ,	2  range
	FLOAT32 vertical_parametrs[3];
	FLOAT32 acceleration_norm_parametrs[3];
	FLOAT32 roll_parametrs[3];
	FLOAT32 pitch_parametrs[3];
	// Data
	FLOAT32 data_m[WINDOW_SIZE_MIS];
	FLOAT32 data_v[WINDOW_SIZE_MIS];
	FLOAT32 data_l[WINDOW_SIZE_MIS];
	FLOAT32 data_h[WINDOW_SIZE_MIS];
	FLOAT32 Acc_Mag[WINDOW_SIZE_MIS];
	//////////////////////////////////
	FLOAT32 PCA_ACCEL[WINDOW_SIZE_MIS][3];// levelled accels
	INT8 horizontal_vertical_flag;
	INT16 counter;
	UINT8 flag_run_PCA;
	STATUS status;
	//////////////////////////////////
} PCA, *PCA_Ptr;
// Peaks
typedef struct _PEAKS
{
	FLOAT32 low_val[PEAKS_VECTOR_SIZE_MIS], high_val[PEAKS_VECTOR_SIZE_MIS];
	INT8    low_ind[PEAKS_VECTOR_SIZE_MIS], high_ind[PEAKS_VECTOR_SIZE_MIS];
	INT8    num[2];	// 0 l		1 h
} PEAKS, *PEAKS_Ptr;
// Main
typedef struct _PCA_M
{
#ifndef COM_WALK_SIMPLIFIED
	FLOAT32 motion_effective_coefficient ;
	FLOAT32 motion_effective_coefficient_mean;
	FLOAT32 vertical_motion_phase;
	FLOAT32 vertical_motion_phase_mean;
#endif
	FLOAT32 vertical_motion_angle;
	FLOAT32 vertical_motion_phase_shift;
	FLOAT32 vertical_lateral_phase_shift;
	FLOAT32 lateral_range;
	FLOAT32 horizontal_acceleration_range;
	INT8    device_flag;
	UINT8   height_change_flag;
} PCA_M, *PCA_M_Ptr;
// Dangling
typedef struct _MOTION_D
{
	PEAKS peaks;
	FLOAT32 thresholds[2];		// 0 ml_thr,	1 mh_thr
} MOTION_D;
typedef struct _VERTICAL_D
{
	PEAKS peaks;
	FLOAT32 thresholds[6];		// 0 vl_thr		1 vh_thr	2 vl_thr2	3 vh_thr2	4 vl_thr3	5 vh_thr3
} VERTICAL_D;
typedef struct _PCA_D
{
	MOTION_D   motion;
	VERTICAL_D vertical;
} PCA_D, *PCA_D_Ptr;
// General
typedef struct _MOTION_G
{
	PEAKS peaks;
	PEAKS slow_peaks;
	PEAKS old_peaks;
	PEAKS local_peaks;
	FLOAT32 thresholds[4];			// 0-ml_thr,   1-mh_thr 2-ml_thr2,   3-mh_thr2
	FLOAT32 motion_low_peaks_range;			// range of motion low peaks
	FLOAT32 motion_high_peaks_range;			// range of motion high peaks
} MOTION_G;
typedef struct _VERTICAL_G
{
	PEAKS peaks;
	PEAKS slow_peaks;
	FLOAT32 thresholds[4];		// 0-vl_thr,    1-vh_thr, 2-vl_thr2	3-vh_thr2
} VERTICAL_G;
typedef struct _ATTITUDE_G
{
	PEAKS peaks;
} ATTITUDE_G;
typedef struct _XCORR_G
{
	PEAKS peaks;
	FLOAT32 x_drift[DATA_SIZE_MIS], z_drift[DATA_SIZE_MIS];
	FLOAT32 avg_cycle;
	INT8 stride_size;
} XCORR_G;
typedef struct _PCA_G
{
	MOTION_G motion;
	VERTICAL_G vertical;
	ATTITUDE_G attitude;
	XCORR_G xcorr;
	FLOAT32 misa;
	INT8 switch_me;
	INT8 i_chest;
} PCA_G, *PCA_G_Ptr;
typedef struct _PCA_C
{
	PEAKS motion_peaks;
	PEAKS vertical_peaks;
	PEAKS attitude_peaks;
	FLOAT32 atth_thr1, attl_thr1, mh_thr1, vh_thr1, vh_thr2;
	FLOAT32 atti_vec[DATA_SIZE_MIS];
	FLOAT32 att_vec[3];											// 0 mean,	1 std ,	2  range
	INT8 change_m, not_dangling_flag, b;
} PCA_C, *PCA_C_Ptr;

typedef struct
{
	DOUBLE64 t0;
	DOUBLE64 t;

	FLOAT32 qbn[4];
	FLOAT32 att[3];
	FLOAT32 Cbn[3][3];
	FLOAT32 bg_estimated[3];
	FLOAT32 ba_estimated[3];

	INT8 if_vertical;

	FLOAT32 std_att[3];
	FLOAT32 std_bg[3];

	FLOAT32 *pCbn[3];
	FLOAT32 *pxk[NSTATE_ATT];
	FLOAT32 xk[NSTATE_ATT];
	FLOAT32 *pP[NSTATE_ATT];
	FLOAT32 *pQ[NSTATE_ATT];
	FLOAT32 P[NSTATE_ATT][NSTATE_ATT];
	FLOAT32 Q[NSTATE_ATT][NSTATE_ATT];

	FLOAT32 Cbb_1[3][3];
	FLOAT32 Cbb_2[3][3];
	FLOAT32 *pCbb_1[3];
	FLOAT32 *pCbb_2[3];

	UINT8 availability;

	//FLOAT32 heading_platform;
} Att_Pkt;



typedef struct    // There is a question that if I change the sequence of the varies, the results may change!!!!!!!!
{
	PCA pca;
	PCA_M pca_m;

	FLOAT32 data_r[WINDOW_SIZE_MIS];		// Roll Vector
	FLOAT32 data_p[WINDOW_SIZE_MIS];		// Pitch Vector
	FLOAT32 acc_var_vec[WINDOW_SIZE_MIS];	// Acceleration Variance vector
	FLOAT32 rad_rot_vec[WINDOW_SIZE_MIS];	// Radius of Rotation vector
	FLOAT32 gyr_nor_vec[WINDOW_SIZE_MIS];	// Gyro Magnitude Variance vector
	FLOAT32 stp_frq_vec[WINDOW_SIZE_MIS];	// Step Frequency vector

	FLOAT32 memo_Lev_Acc_Data[3][WINDOW_SIZE_MIS];
	FLOAT32 memo_vertical_change[WINDOW_SIZE_MIS];

	FLOAT32 vertical_motion_phase_vec[WINDOW_SIZE_MIS];			// Step Frequency vector
	FLOAT32 motion_effective_coefficient_vec[WINDOW_SIZE_MIS];  // Step Frequency vector


	UINT8 i_memo;

	INT8 horizontal_vertical_flag;		// Check the vertical and horizontal orientation
	UINT8    height_change_flag;		// Check the change in the height

	FLOAT32 x_current[10];
	FLOAT32 y_current[10];
	FLOAT32 x_history_vec[19][10];
	FLOAT32 y_history_vec[19][10];

	FLOAT32 Lev_Acc_Window[3][WINDOW_SIZE_MIS];
	FLOAT32 *pLev_Acc_Window[3]; // Window size of IMU data

	UINT8 memo_mode_s2[WINDOW_SIZE_MIS];
	UINT8 i_memo_mode_s2;

	FLOAT32 memo_x_history_data[40][10];
	FLOAT32 memo_y_history_data[40][10];
	UINT8 i_memo_history_data;

	FLOAT32 step_frequency_data;

	FLOAT32 Misalign_Az1;

	//FLOAT32 Step_Frequency_Data;
	FLOAT32 Par_Values[50];             // Just for output

	//FLOAT32 Step_Frequency_Data[N_MAX_DATA_EPH];
	FLOAT32 Misalign_Az;
	FLOAT32 Misalign_Az_pre;

	UINT8 dir_s1; //dir_s[N_MAX_DATA_EPH];

	//FLOAT32 x_history_data[N_MAX_DATA_EPH][10];
	//FLOAT32 y_history_data[N_MAX_DATA_EPH][10];
	FLOAT32 vertical_motion_phase_Data;
	FLOAT32 motion_effective_coefficient_Data;
	//INT16 start_1; // = 0/*, end_1 = 0*/;		// Start and end indices of each window


	INT8  device_flag;	// To tell the device type(Phone - Tablet, Watch, ...)



	//Input_Misa_pkt in_pkt;
	FLOAT32 in_cur_Lev_Acc_Data[3];
	FLOAT32 in_cur_pca_data_h;
	FLOAT32 in_cur_pca_Acc_Mag;
	FLOAT32 in_cur_Lev_Acc_Window[3];
	FLOAT32 in_cur_RP_Data[2];
	FLOAT32 in_cur_Acc_Variance_Data;
	FLOAT32 in_cur_Rad_Rotation_Data;
	FLOAT32 in_cur_Gyr_Norm_Data;
	FLOAT32 in_cur_Step_Frequency_Data; // misa.Step_Frequency_Data[mm];
	FLOAT32 in_cur_vertical_motion_phase_Data; //misa.vertical_motion_phase_Data[mm];
	FLOAT32 in_cur_motion_effective_coefficient_Data; //misa.motion_effective_coefficient_Data[mm];
	FLOAT32 in_cur_Vertical_Change_Data;
	FLOAT32 in_cur_Height_Change_Data;

	// ---
	//FLOAT32 misalign_az; // same as Misalign_Az
	INT8 motion_mode;

	FLOAT32 Misalign_Az_For_Show;
	FLOAT32 Misalign_Az_For_Use;

	//INT8 fb_detect;  same as dir_s1
	//  FLOAT32 step_frequency;  same as step_frequency_data

	//PCA pca;
	//PCA_M pca_m;

	//UINT8 i_memo;

	//FLOAT32 data_r[WINDOW_SIZE_MIS];		// Roll Vector
	//FLOAT32 data_p[WINDOW_SIZE_MIS];		// Pitch Vector
	//FLOAT32 acc_var_vec[WINDOW_SIZE_MIS];	// Acceleration Variance vector
	//FLOAT32 rad_rot_vec[WINDOW_SIZE_MIS];	// Radius of Rotation vector
	//FLOAT32 gyr_nor_vec[WINDOW_SIZE_MIS];	// Gyro Magnitude Variance vector
	//FLOAT32 stp_frq_vec[WINDOW_SIZE_MIS];	// Step Frequency vector

	//FLOAT32 memo_Lev_Acc_Data[3][WINDOW_SIZE_MIS];
	//FLOAT32 memo_vertical_change[WINDOW_SIZE_MIS];
	//FLOAT32 vertical_motion_phase_vec[WINDOW_SIZE_MIS];			// Step Frequency vector
	//FLOAT32 motion_effective_coefficient_vec[WINDOW_SIZE_MIS];  // Step Frequency vector

	//FLOAT32 x_current[10];
	//FLOAT32 y_current[10];
	//FLOAT32 x_history_vec[19][10];
	//FLOAT32 y_history_vec[19][10];

	//FLOAT32 Lev_Acc_Window[3][WINDOW_SIZE_MIS];
	//FLOAT32 *pLev_Acc_Window[3]; // Window size of IMU data

	//UINT8	memo_mode_s2[WINDOW_SIZE_MIS];
	//UINT8	i_memo_mode_s2;
	//
	//INT8	horizontal_vertical_flag;		// Check the vertical and horizontal orientation
	//UINT8   height_change_flag;		// Check the change in the height

	//
	//FLOAT32 memo_x_history_data[40][10];
	//FLOAT32 memo_y_history_data[40][10];
	//UINT8	i_memo_history_data;

	//FLOAT32 step_frequency_data;
	//FLOAT32 vertical_motion_phase_Data;
	//FLOAT32 motion_effective_coefficient_Data;
	////INT16	start_1; 		// Start and end indices of each window
	//UINT8	dir_s1; //dir_s[N_MAX_DATA_EPH];

	//FLOAT32 Misalign_Az1;
	//FLOAT32 Misalign_Az;
	//FLOAT32 Misalign_Az_pre;
	//
	//FLOAT32 Par_Values[50];             // Just for output

	//FLOAT32 in_cur_Lev_Acc_Data[3];
	//FLOAT32 in_cur_pca_data_h;
	//FLOAT32 in_cur_pca_Acc_Mag;
	//FLOAT32 in_cur_Lev_Acc_Window[3];
	//FLOAT32 in_cur_RP_Data[2];
	//FLOAT32 in_cur_Acc_Variance_Data;
	//FLOAT32 in_cur_Rad_Rotation_Data;
	//FLOAT32 in_cur_Gyr_Norm_Data;
	//FLOAT32 in_cur_Step_Frequency_Data; // misa.Step_Frequency_Data[mm];
	//FLOAT32 in_cur_vertical_motion_phase_Data; //misa.vertical_motion_phase_Data[mm];
	//FLOAT32 in_cur_motion_effective_coefficient_Data; //misa.motion_effective_coefficient_Data[mm];
	//FLOAT32 in_cur_Vertical_Change_Data;
	//FLOAT32 in_cur_Height_Change_Data;

} Misalign_pkt;




// ========================== End of Structure Attitude and Misalignment ==================================

void indoor_navigation_withWiFi_kf(Initialization_Pkt* ini, Meas meas, Sol_Pkt*  sol);

void indoor_navigation_withWiFi(Meas meas, Sol_Pkt*  sol);
void indoor_navigation(Meas meas, Sol_Pkt* sol);
void wifi_position_tracker_smooth_int(long int mac_ap_int[MAX_AP_NUM], int rss_ap[MAX_AP_NUM], int N, int floor[1], double pos[3], int avail_wifi[1]);
void real_time_put_into_memo(MemoIndoor* memo, Meas meas,Att_Pkt *att_pkt, NavIndoor *nav);
void wifi_position_tracker_smooth(char name_ap[MAX_AP_NUM][N_CHAR_MAC], float rss_ap[MAX_AP_NUM], int N, int floor[1], double pos[3], int availability[1]);
void wifi_position_tracker(char name_ap[MAX_AP_NUM][N_CHAR_MAC], float rss_ap[MAX_AP_NUM], int N, int floor[1], double pos[2], int availability[1]);

UINT8 map_adjusting(NavIndoor *nav, Att_Pkt *att_pkt, MemoIndoor *memo);

void load_DB_int_bin_diff_co(const char * f_DB, long int DB_mac[MAX_AP_NUM], int DB_matrix_diff[MAX_RP_NUM][MAX_AP_NUM], double DB_pos[MAX_RP_NUM][3], int DB_size[2]) ;
void load_DB_int_bin_co(const char * f_DB, long int DB_mac[MAX_AP_NUM], int DB_matrix[MAX_RP_NUM][MAX_AP_NUM], double DB_pos[MAX_RP_NUM][3], int DB_size[2]);
void load_DB_int_bin(const char * f_DB, long int DB_mac[MAX_AP_NUM], int DB_matrix[MAX_RP_NUM][MAX_AP_NUM], double DB_pos[MAX_RP_NUM][3], int DB_size[2]);
void load_DB_int_bin_diff(const char * f_DB, long int DB_mac[MAX_AP_NUM], int DB_matrix_diff[MAX_RP_NUM][MAX_AP_NUM], double DB_pos[MAX_RP_NUM][3], int DB_size[2]);

long int mac2int(char mac[N_CHAR_MAC]);
int hex2dec_n(char *s, int n);
double cal_dss_int(int *v1, int *v2, int n);
double cal_dist_2D(double lat1, double lon1, double lat2, double lon2);

STATUS KF_update_indoor_float(INT16 Num, FLOAT32 *x, FLOAT32 **P, FLOAT32 *inno, FLOAT32 **H, FLOAT32 **R,INT16 NUMStates);
STATUS AEKF_MM_2_update_indoor_float(INT16 Num, FLOAT32 *x, FLOAT32 **P, FLOAT32 *inno, FLOAT32 **H, FLOAT32 **R,INT16 NUMStates);

void indoor_navigation_old(int avail_gyro, double t_gyro, float w_b[3], int avail_accel, double t_accel, float f_b[3], RecSol sol);
void initial_memo_nav(MemoIndoor* memo);
float center_heading(float heading);

void inv_gyro_bias_tracker(DOUBLE64 t_cur, FLOAT32 w_b[3], FLOAT32 f_b[3], FLOAT32 m_b[3], UINT8 flag_mag, UINT8 accuracy_flag, FLOAT32 seed_bias[3], UINT8 seed_availability, FLOAT32 gBias[3], UINT8* availability);

void GetSystemModel_6(FLOAT32 Cbn[3][3], FLOAT32 tao[3], FLOAT32 winn[3], FLOAT32 F[NStates][NStates], FLOAT32 G[NStates][6]);
void put_into_memo(Memo* momo, Wind* win, FLOAT32 t, FLOAT32 w_b[3], FLOAT32 f_b[3], FLOAT32 m_b[3], UINT8 flag_mag);
void put_into_memo_group2(Memo* momo, Wind* win, FLOAT32 t, FLOAT32 w_b[3], FLOAT32 f_b[3], FLOAT32 m_b[3], FLOAT32 ng0, UINT8 flag_mag, UINT8 tag_condition);
void initial_memo(Memo* momo, Wind* win);
void compensate_b(FLOAT32* obs, FLOAT32* bias);
FLOAT32 cal_roll(FLOAT32* f_b);
FLOAT32 cal_pitch(FLOAT32* f_b);

void multiply_matrix_with_vector_3x3( FLOAT32 Matrix[3][3], FLOAT32 vector[3], FLOAT32 vectorR[3] );
void ChangeArrayMemoryLayout_float(UINT8 r, UINT8 c , FLOAT32 **ptr, FLOAT32 *Arr, UINT8 InitToZero);
void multiply_scalar_with_matrix_nxn(FLOAT32 Scalar, FLOAT32 **Matrix, FLOAT32 **MatrixR,UINT8 cols, UINT8 rows);
void add_matrices_nxn(FLOAT32 **Matrix1, FLOAT32 **Matrix2, UINT8 nrow, UINT8 ncol, FLOAT32 **MatrixR);
void matrix_transpose_nxn_float(FLOAT32 **Matrix, UINT8 m, UINT8 n, FLOAT32 **MatrixT);
void multiply_matrices_nxn_float(FLOAT32 **Matrix1, FLOAT32 **Matrix2, UINT8 ar, UINT8 ac, UINT8 bc, FLOAT32 **MatrixR);
void multiply_matrix_with_vector_nxn(FLOAT32 **Matrix, UINT8 ar, UINT8 ac, FLOAT32 *vector, FLOAT32 *vectorR);
void make_skew_symmetric_matrix( FLOAT32 vector[3], FLOAT32 Matrix[3][3] );
void diag2_sq(FLOAT32 *A, INT16 r, FLOAT32 **X);
void find_max_min_float(FLOAT32 *vector, FLOAT32 *maxmin, UINT16 arraysize);
void find_max_min_double(DOUBLE64 *vector, DOUBLE64 *maxmin, UINT16 arraysize);
//void max_float(FLOAT32 *vector, FLOAT32 *max, UINT16 arraysize);
void min_float(FLOAT32 *vector, FLOAT32 *min, UINT16 arraysize);
void max_float(FLOAT32 *vector, FLOAT32 *max, UINT16 arraysize);
void min_double(double *vector, double *min, UINT16 arraysize);
float min_f(FLOAT32 *vector, UINT16 arraysize);
STATUS MatrixInv(FLOAT32 **A, INT16 n);
void eye_matrix_float(FLOAT32 **A, INT16 n);
void subtract_matrices_nxn(FLOAT32 **Matrix1, FLOAT32 **Matrix2, UINT8 nrow, UINT8 ncol, FLOAT32 **MatrixR);
void zero_matrix_entries_nxn_float(FLOAT32 **Matrix, INT16 row, INT16 col);
void CopyMatrix_N_Dimension(FLOAT32 **Matrix, INT16 m, INT16 n, FLOAT32 **CopyMatrix);
STATUS lu_decomposition(FLOAT32 **A, INT16 n, INT16 *indx);
static void lu_back_subsititution(FLOAT32 **A, INT16 n, INT16 *indx, FLOAT32 *b);
void vector_cross_product( FLOAT32 vector1[3], FLOAT32 vector2[3], FLOAT32 vectorR[3] );
float cal_std_float(FLOAT32* a, UINT8 n);

void Attitude_Mechanization_float(FLOAT32 preG[3], FLOAT32 curG[3], FLOAT32 nav_prev_q_bn[4], FLOAT32 nav_cur_q_bn[4],FLOAT32 nav_cur_C_bn[3][3],FLOAT32 nav_cur_att[3], FLOAT32 zeta[3]);
void Attitude_Mechanization_simple_float(FLOAT32 curG[3], FLOAT32 nav_prev_q_bn[4], FLOAT32 nav_cur_q_bn[4],FLOAT32 nav_cur_C_bn[3][3],FLOAT32 nav_cur_att[3], FLOAT32 zeta[3]);
void Euler2Dcm(FLOAT32 att[3], FLOAT32 Cbn[3][3]);
void Dcm2Quat(FLOAT32 C[3][3], FLOAT32 q[4]);
void Rvec2Quat(FLOAT32 r_vec[3], FLOAT32 d_q[4]);
void QuatPrdct( FLOAT32 q[4], FLOAT32 p[4], FLOAT32 qnew[4] );
void NormQuat(FLOAT32 q[4], FLOAT32 q_norm[4]);
void Quat2Dcm( FLOAT32 quar[4], FLOAT32 C[3][3] );
void Dcm2Euler(FLOAT32 Cbn[3][3], FLOAT32 attitude[3]);

float normalGravity(double lat, float hei);

STATUS KF_update1_float(INT16 Num, FLOAT32 *x, FLOAT32 **P, FLOAT32 *inno, FLOAT32 **H, FLOAT32 **R,INT16 NUMStates);
void INSNavFeedBack_Attitude_float( FLOAT32 x[NStates], FLOAT32 q_bn[4], FLOAT32 C_bn[3][3], FLOAT32 att[3], FLOAT32 EstimatedGyroBias[3]);

// WGS2GCJ
void wgs2gcj(double wgsLat, double wgsLng, double *gcjLat, double *gcjLng);
void gcj2wgs(double gcjLat, double gcjLng, double *wgsLat, double *wgsLnt);
void gcj2wgs_exact(double gcjLat, double gcjLng, double *wgsLat, double *wgsLnt);
double distance(double latA, double lngA, double latB, double lngB);

void load_DB_fp_string(float da_map_NE[N_ROW_DB_FP][N_COL_DB_FP], int DB_size[1]);
void load_DB_fp_txt(const char * f_FP, float da_map_NE[MAX_FLOOR_LINE_NUM][N_COL_DB_FP], int DB_size[1]);
FLOAT32 centralization_angle_deg(FLOAT32 ang);
FLOAT32 cal_mean_float(FLOAT32* a, UINT16 n);
FLOAT32 ptoject2line(float ps[2],float p1[2],float p2[2],float pp[2]);
FLOAT32 ptoject2line_can_be_outside(float ps[2],float p1[2],float p2[2],float pp[2]);
FLOAT32 dist_ang_rad(float ang1, float ang2);
FLOAT32 cal_dist_2D_NE(float x1, float y1, float x2, float y2);

FLOAT32 cal_height_baro(FLOAT32 ba);


// ----------------------------- Varies Attitude and Misalignment -------------------------------------
void misalign_estimation_mis(Misalign_pkt *misa);
void cal_misalign_1_eph_mis(Misalign_pkt *misa);
void play_with_misalign(Misalign_pkt *misa, Att_Pkt * att_pkt, MemoIndoor *memo);


void attitude_filter_att(Att_Pkt* att_pkt, DOUBLE64 t_cur, FLOAT32 w_b[3], FLOAT32 f_b[3], UINT8 flag_accel, FLOAT32 m_b[3], UINT8 flag_mag, FLOAT32 ini_heading, UINT8 flag_ini_heading, FLOAT32 seed_bias[3], UINT8 seed_availability);
void attitude_filter_without_bg_conpensation_att(Att_Pkt* att_pkt, MemoIndoor *memo, DOUBLE64 t_cur, FLOAT32 w_b[3], FLOAT32 f_b[3], UINT8 flag_accel, FLOAT32 m_b[3], UINT8 flag_mag, FLOAT32 ini_heading, UINT8 flag_ini_heading, FLOAT32 seed_bias[3], UINT8 seed_availability);
void attitude_filter_predict_att(Att_Pkt* att_pkt, DOUBLE64 t_cur, FLOAT32 w_b[3], FLOAT32 f_b[3], UINT8 flag_accel, FLOAT32 m_b[3], UINT8 flag_mag, FLOAT32 ini_heading, UINT8 flag_ini_heading, FLOAT32 seed_bias[3], UINT8 seed_availability);
void attitude_filter_update_accel_att(Att_Pkt* att_pkt, DOUBLE64 t_cur, FLOAT32 w_b[3], FLOAT32 f_b[3], UINT8 flag_accel, FLOAT32 m_b[3], UINT8 flag_mag, FLOAT32 ini_heading, UINT8 flag_ini_heading, FLOAT32 seed_bias[3], UINT8 seed_availability);



// Peak Detection
INT8 peaks_detection_mis(FLOAT32 *data, FLOAT32 delta, INT8 min_peaks_num, PEAKS_Ptr peaks, UINT8 peaks_vector_size, UINT8 data_size, UINT8 protection_use_flag);
// Peaks Frequency Calculation
#ifdef PRINT_RESULTS_MISALIGNMENT
UINT8 peak_frequency_calculation_mis(PCA_Ptr pca, FLOAT32 *acc_var_vec, FLOAT32 *acc_var, FLOAT32 *stp_frq_vec, FLOAT32 *Step_Frequency, FLOAT32 *Par_Values);
#else
UINT8 peak_frequency_calculation_mis(PCA_Ptr pca, FLOAT32 *acc_var_vec, FLOAT32 *acc_var, FLOAT32 *stp_frq_vec, FLOAT32 *Step_Frequency);
#endif
// FFT Calculation
#ifndef COM_WALK_SIMPLIFIED
void fft_mis_calculation_mis(FLOAT32 **xy_data, FLOAT32 *data_r, FLOAT32 *data_p, FLOAT32 acc_var, INT8 device_flag, UINT8 motion_unstability_flag, FLOAT32 **x_history_vec, FLOAT32 *x_current, FLOAT32 **y_history_vec, FLOAT32 *y_current, UINT8 mode2_fft_mis);
#endif
int fft_mis_calculation_mis_constSize(FLOAT32 **xy_data, FLOAT32 *data_r, FLOAT32 *data_p, FLOAT32 acc_var, INT8 device_flag, UINT8 motion_unstability_flag, FLOAT32 x_history_vec[19][10], FLOAT32 *x_current, FLOAT32 y_history_vec[19][10], FLOAT32 *y_current, UINT8 mode2_fft_mis);


// Phase 1 fast calculation
FLOAT32 calculate_phase_one_angle_mis(FLOAT32 **data, FLOAT32 mean_x, FLOAT32 mean_y);
// Phase 1 analytical calculation
FLOAT32 calculate_phase_one_angle_mis_analytical(FLOAT32 XX_sum, FLOAT32 XY_sum, FLOAT32 YY_sum, UINT8 angle_slope_flag);
// Sign Method Calculation
#ifndef COM_WALK_SIMPLIFIED
void sign_method_parameters_calculation_mis(PCA_M_Ptr pca_m, PCA_Ptr pca, FLOAT32 *motion_effective_coefficient_vec, FLOAT32 *vertical_motion_phase_vec);
#else
void sign_method_parameters_calculation_mis(PCA_M_Ptr pca_m, PCA_Ptr pca);
#endif
// Motion Classification
#ifdef PRINT_RESULTS_MISALIGNMENT
void device_use_cases_classification_mis(FLOAT32 *data_r, FLOAT32 *data_p, PCA_Ptr pca, FLOAT32 acc_var, FLOAT32 *rad_rot_vec, FLOAT32 *gyr_nor_vec, UINT8 motion_unstability_flag, INT8 device_flag, UINT8 *mode1, UINT8 *mode2, FLOAT32 *Par_Values);
#else
void device_use_cases_classification_mis(FLOAT32 *data_r, FLOAT32 *data_p, PCA_Ptr pca, FLOAT32 acc_var, FLOAT32 *rad_rot_vec, FLOAT32 *gyr_nor_vec, UINT8 motion_unstability_flag, INT8 device_flag, UINT8 *mode1, UINT8 *mode2);
#endif
INT8 classification_by_peaks_mis(PCA_C_Ptr pca_c, PCA_Ptr pca);
INT8 pocket_pattern_detection_mis(PCA_C_Ptr pca_c, PCA_Ptr pca, UINT8 h_l_flag);
// Repeated code Segments
INT8 peaks_min_distance_mis(PCA_G_Ptr pca_g, UINT8 *min_peaks_dis, UINT8 method_num, UINT8 m_local, INT8 *step);
void slope_segment_mis(INT8 peaks_num, INT8 *peaks_ind, FLOAT32 *peaks_val, FLOAT32 *data, UINT8 be_ind, UINT8 af_ind, UINT8 method_num, FLOAT32 threshold, INT8 *step, UINT8 vh_vl);
void motion_peaks_segment_mis(PEAKS_Ptr peaks, FLOAT32 *m_thrs, INT8 st_ind, INT8 en_ind, UINT8 Method_Num, UINT8 vh_vl, INT8 *step);
// Pocket
INT8 pocket_backward_forward_mis(FLOAT32 *data_r, FLOAT32 *data_p, PCA_Ptr pca, PCA_M_Ptr pca_m);
// Dangling
INT8 dangling_backward_forward_mis(PCA_Ptr pca, PCA_M_Ptr pca_m);
// Dangling Phone
#ifndef COM_WALK_SIMPLIFIED
INT8 dangling_phone_backward_forward_mis(PCA_Ptr pca, PCA_M_Ptr pca_m, PCA_D_Ptr pca_d);
#else
INT8 dangling_phone_backward_forward_mis(PCA_Ptr pca, PCA_D_Ptr pca_d);
#endif
INT8 dangling_phone_1_mis(PCA_D_Ptr pca_d, UINT8 bef_val, UINT8 aft_val, UINT8 Method_Num);
INT8 dangling_phone_2_mis(PCA_D_Ptr pca_d, PCA_Ptr pca);
INT8 dangling_phone_3_mis(PCA_D_Ptr pca_d);
INT8 dangling_phone_4_mis(PCA_D_Ptr pca_d, UINT8 bef_val, UINT8 aft_val, UINT8 Method_Num);
INT8 dangling_phone_5_mis(PCA_D_Ptr pca_d, PCA_Ptr pca);
INT8 dangling_phone_6_mis(PCA_D_Ptr pca_d);
INT8 dangling_phone_7_mis(PCA_D_Ptr pca_d, PCA_Ptr pca);
// Dangling Tablet
INT8 dangling_tablet_watch_backward_forward_mis(PCA_Ptr pca, PCA_M_Ptr pca_m, PCA_D_Ptr pca_d);
INT8 dangling_tablet_watch_1_mis(PCA_D_Ptr pca_d);
INT8 dangling_tablet_watch_2_mis(PCA_D_Ptr pca_d, FLOAT32 range_ml_vl, FLOAT32 range_mh_vh);
INT8 dangling_tablet_watch_3_mis(PCA_D_Ptr pca_d);
INT8 dangling_tablet_watch_4_mis(PCA_D_Ptr pca_d);
INT8 dangling_tablet_watch_5_mis(PCA_D_Ptr pca_d);
INT8 dangling_tablet_watch_6_mis(PCA_D_Ptr pca_d, PCA_Ptr pca);
INT8 dangling_tablet_watch_7_mis(PCA_D_Ptr pca_d, PCA_Ptr pca, FLOAT32 range_vl);
INT8 dangling_tablet_watch_8_mis(PCA_D_Ptr pca_d);
INT8 dangling_tablet_watch_9_mis(PCA_D_Ptr pca_d, PCA_Ptr pca);
INT8 dangling_tablet_watch_10_mis(PCA_D_Ptr pca_d, FLOAT32 range_vl, FLOAT32 range_vh);
INT8 dangling_tablet_watch_11_mis(PCA_D_Ptr pca_d, PCA_Ptr pca, PCA_M_Ptr pca_m);
INT8 dangling_tablet_watch_12_mis(PCA_D_Ptr pca_d);
INT8 dangling_tablet_watch_13_mis(PCA_D_Ptr pca_d, PCA_Ptr pca);
INT8 dangling_tablet_watch_14_mis(PCA_D_Ptr pca_d, PCA_Ptr pca);
INT8 dangling_tablet_watch_15_mis(PCA_D_Ptr pca_d);
INT8 dangling_tablet_watch_16_mis(PCA_D_Ptr pca_d);
INT8 dangling_tablet_watch_17_mis(PCA_D_Ptr pca_d);
INT8 dangling_tablet_watch_17_2_mis(PCA_D_Ptr pca_d);
INT8 dangling_tablet_watch_18_mis(PCA_D_Ptr pca_d, PCA_Ptr pca);
INT8 dangling_tablet_watch_19_mis(PCA_D_Ptr pca_d);
INT8 dangling_tablet_watch_20_mis(PCA_D_Ptr pca_d, PCA_Ptr pca, PCA_M_Ptr pca_m);
INT8 dangling_tablet_watch_21_mis(PCA_D_Ptr pca_d, PCA_Ptr pca);
INT8 dangling_tablet_watch_22_mis(PCA_D_Ptr pca_d);
INT8 dangling_tablet_watch_23_mis(PCA_D_Ptr pca_d);
INT8 dangling_tablet_watch_24_mis(PCA_D_Ptr pca_d, PCA_Ptr pca);
INT8 dangling_tablet_watch_25_mis(PCA_D_Ptr pca_d);
INT8 dangling_tablet_watch_26_mis(PCA_D_Ptr pca_d, PCA_Ptr pca);
// General
INT8 general_backward_forward_mis(FLOAT32 *data_r, FLOAT32 *data_p, PCA_Ptr pca, PCA_M_Ptr pca_m, FLOAT32 misa);
// General Phone and Tablet
INT8 general_phone_tablet_backward_forward_mis(PCA_G_Ptr pca_g, PCA_Ptr pca, PCA_M_Ptr pca_m);
INT8 general_phone_tablet_1_mis(PCA_G_Ptr pca_g, PCA_Ptr pca, FLOAT32 v1, FLOAT32 v2, FLOAT32 v3, FLOAT32 v4);
INT8 general_phone_tablet_2_mis(PCA_G_Ptr pca_g, PCA_Ptr pca);
INT8 general_phone_tablet_3_mis(PCA_G_Ptr pca_g, PCA_Ptr pca);
INT8 general_phone_tablet_4_mis(PCA_G_Ptr pca_g, PCA_Ptr pca);
INT8 general_phone_tablet_5_mis(PCA_G_Ptr pca_g, PCA_Ptr pca);
INT8 general_phone_tablet_6_mis(PCA_G_Ptr pca_g, PCA_Ptr pca);
INT8 general_phone_tablet_7_mis(PCA_G_Ptr pca_g);
INT8 general_phone_tablet_8_mis(PCA_G_Ptr pca_g);
INT8 general_phone_tablet_9_mis(PCA_G_Ptr pca_g);
INT8 general_phone_tablet_10_mis(PCA_G_Ptr pca_g, PCA_Ptr pca);
INT8 general_phone_tablet_11_mis(PCA_G_Ptr pca_g, PCA_Ptr pca);
INT8 general_phone_tablet_12_mis(PCA_G_Ptr pca_g);
INT8 general_phone_tablet_13_mis(PCA_G_Ptr pca_g, PCA_Ptr pca);
INT8 general_phone_tablet_14_mis(PCA_G_Ptr pca_g, PCA_Ptr pca);
INT8 general_phone_tablet_15_mis(PCA_G_Ptr pca_g, PCA_Ptr pca);
INT8 general_phone_tablet_16_mis(PCA_G_Ptr pca_g, PCA_Ptr pca);
INT8 general_phone_tablet_17_mis(PCA_G_Ptr pca_g, PCA_Ptr pca);
INT8 general_phone_tablet_18_mis(PCA_G_Ptr pca_g);
INT8 general_phone_tablet_19_mis(PCA_G_Ptr pca_g, PCA_Ptr pca);
INT8 general_phone_tablet_20_mis(PCA_G_Ptr pca_g, PCA_Ptr pca, UINT8 Method_Num);
INT8 general_phone_tablet_21_mis(PCA_G_Ptr pca_g, PCA_Ptr pca);
INT8 general_phone_tablet_22_mis(PCA_G_Ptr pca_g, PCA_Ptr pca, PCA_M_Ptr pca_m);
INT8 general_phone_tablet_23_mis(PCA_G_Ptr pca_g);
// General Watch
INT8 general_watch_backward_forward_mis(PCA_G_Ptr pca_g, PCA_Ptr pca, PCA_M_Ptr pca_m);
INT8 general_watch_method1_mis(PCA_G_Ptr pca_g, PCA_Ptr pca, PCA_M_Ptr pca_m);
INT8 general_watch_method2_mis(PCA_G_Ptr pca_g, PCA_Ptr pca);
INT8 general_watch_method3_mis(PCA_G_Ptr pca_g, PCA_Ptr pca);
INT8 general_watch_method4_mis(PCA_G_Ptr pca_g);
INT8 general_watch_method5_mis(PCA_G_Ptr pca_g);
// X_Corr
INT8 xcorr_backward_forward_mis(PCA_Ptr pca);


void INSNavFeedBack_Attitude_float_att(Att_Pkt * att_pkt, FLOAT32 f_b[3]);
void axis_leveling_att(Att_Pkt * att_pkt);
void Attitude_Mechanization_simple_float_att(FLOAT32 curG[3], FLOAT32 nav_prev_q_bn[4], FLOAT32 nav_cur_q_bn[4]);
FLOAT32 cal_roll_att(FLOAT32* f_b);
FLOAT32 cal_pitch_att(FLOAT32* f_b);
FLOAT32 cal_mean_float_att(FLOAT32* a, UINT16 n);
FLOAT32 cal_variance_float(FLOAT32* a, float mean1, UINT16 n);
void compensate_b_att(FLOAT32* obs, FLOAT32* bias);
void initialization_misalign_pkt_mis(Misalign_pkt * misa);

void Euler2Dcm_att(FLOAT32 att[3], FLOAT32 Cbn[3][3]);
void Dcm2Quat_att(FLOAT32 C[3][3], FLOAT32 q[4]);
void Rvec2Quat_att(FLOAT32 r_vec[3], FLOAT32 d_q[4]);
void QuatPrdct_att( FLOAT32 q[4], FLOAT32 p[4], FLOAT32 qnew[4] );
void NormQuat_att(FLOAT32 q[4], FLOAT32 q_norm[4]);
void Quat2Dcm_att( FLOAT32 quar[4], FLOAT32 C[3][3] );
void Dcm2Euler_att(FLOAT32 Cbn[3][3], FLOAT32 attitude[3]);

void ChangeArrayMemoryLayout_float_att(UINT8 r, UINT8 c , FLOAT32 **ptr, FLOAT32 *Arr, UINT8 InitToZero);
void multiply_scalar_with_matrix_nxn_att(FLOAT32 Scalar, FLOAT32 **Matrix, FLOAT32 **MatrixR,UINT8 cols, UINT8 rows);
void add_matrices_nxn_att(FLOAT32 **Matrix1, FLOAT32 **Matrix2, UINT8 nrow, UINT8 ncol, FLOAT32 **MatrixR);
void matrix_transpose_nxn_float_att(FLOAT32 **Matrix, UINT8 m, UINT8 n, FLOAT32 **MatrixT);
void multiply_matrices_nxn_float_att(FLOAT32 **Matrix1, FLOAT32 **Matrix2, UINT8 ar, UINT8 ac, UINT8 bc, FLOAT32 **MatrixR);
void multiply_matrix_with_vector_nxn_att(FLOAT32 **Matrix, UINT8 ar, UINT8 ac, FLOAT32 *vector, FLOAT32 *vectorR);
void make_skew_symmetric_matrix_att( FLOAT32 vector[3], FLOAT32 Matrix[3][3] );
void diag2_sq_att(FLOAT32 *A, INT16 r, FLOAT32 **X);
STATUS MatrixInv_att(FLOAT32 **A, INT16 n);
void CopyMatrix_N_Dimension_att(FLOAT32 **Matrix, INT16 m, INT16 n, FLOAT32 **CopyMatrix);
STATUS lu_decomposition_att(FLOAT32 **A, INT16 n, INT16 *indx);
static void lu_back_subsititution_att(FLOAT32 **A, INT16 n, INT16 *indx, FLOAT32 *b);
void eye_matrix_float_att(FLOAT32 **A, INT16 n);
void subtract_matrices_nxn_att(FLOAT32 **Matrix1, FLOAT32 **Matrix2, UINT8 nrow, UINT8 ncol, FLOAT32 **MatrixR);


STATUS KF_update1_float_att(INT16 Num, FLOAT32 *x, FLOAT32 **P, FLOAT32 *inno, FLOAT32 **H, FLOAT32 **R,INT16 NUMStates);
void GetSystemModel_6_att(FLOAT32 Cbn[3][3], FLOAT32 tao[3], FLOAT32 F[NSTATE_ATT][NSTATE_ATT], FLOAT32 G[NSTATE_ATT][6]);

STATUS KF_Update_Heading_Attitude(FLOAT32 *x, FLOAT32 **P,FLOAT32 att[3],FLOAT32 C_bn[3][3], FLOAT32 Heading, FLOAT32 covH);

// ======================= LY

DOUBLE64 running_avg_mis(DOUBLE64 prev, DOUBLE64 newdata,INT32 newcount);
DOUBLE64 Average_Data_mis(DOUBLE64 *data,INT16 num_elements);
void find_max_min_float_mis(FLOAT32 *vector, FLOAT32 *maxmin, UINT16 arraysize);
//DOUBLE64 calculate_correlation_mis(DOUBLE64 *X, DOUBLE64 *Y, INT16 dimension);
void Mtrx_Mul_Vctr_mis22_mis( DOUBLE64 Matrix[2][2], DOUBLE64 vector[2], DOUBLE64 vectorR[2] );
void Vctr22_Mul_Vctr22_mis( DOUBLE64 vector1[2], DOUBLE64 vector2[2], DOUBLE64 MatrixR[2][2] );
void Zeros_22_mis(DOUBLE64 Matrix0[2][2]);
void Mtrx_Mul_Mtrx_mis22_mis( DOUBLE64 Matrix1[2][2], DOUBLE64 Matrix2[2][2], DOUBLE64 MatrixR[2][2] );
void Zeros_33_mis(DOUBLE64 Matrix0[3][3]);
void Mtrx_Mul_Vctr_mis(DOUBLE64 Matrix[3][3], DOUBLE64 vector[3], DOUBLE64 vectorR[3] );
void VectorCrossProduct_mis(DOUBLE64 vector1[3], DOUBLE64 vector2[3], DOUBLE64 vectorR[3] );
void DiagMatrix_mis(DOUBLE64 vector[3], DOUBLE64 Matrix[3][3] );
void SkewMatrix_mis(DOUBLE64 vector[3], DOUBLE64 Matrix[3][3] );
void MsubM_mis(DOUBLE64 Matrix1[3][3], DOUBLE64 Matrix2[3][3], DOUBLE64 MatrixR[3][3] );
void Mtrx_Mul_Mtrx_mis( DOUBLE64 Matrix1[3][3], DOUBLE64 Matrix2[3][3], DOUBLE64 MatrixR[3][3] );
void MMT_mis( DOUBLE64 Matrix[3][3], DOUBLE64 MatrixT[3][3] );
void MaddM_mis( DOUBLE64 Matrix1[3][3], DOUBLE64 Matrix2[3][3], DOUBLE64 AddedMatrix[3][3] );
DOUBLE64 Vec_Mul_Vec_mis(DOUBLE64 vector1[3], DOUBLE64 vector2[3]);
void ScalarWithMtrx_mis(DOUBLE64 Scalar, DOUBLE64 Matrix[3][3], DOUBLE64 MatrixR[3][3]);
void MEM_mis(DOUBLE64 Matrix[3][3], DOUBLE64 CopyMatrix[3][3]);
void Unitmatrix_D_mis(DOUBLE64 **A, INT16 size);
void MV_D_mis(DOUBLE64 **Matrix, INT16 ar, INT16 ac, DOUBLE64 *vector, DOUBLE64 *vectorR);
void MMM_D_mis(DOUBLE64 **Matrix1, DOUBLE64 **Matrix2, INT16 ar, INT16 ac, INT16 bc, DOUBLE64 **MatrixR);
DOUBLE64 *Mem_Alloc_1d_mis(INT16 size);
DOUBLE64 **Mem_Alloc_2d_mis(INT16 row, INT16 col);
void MMT_D_mis(DOUBLE64 **Matrix, INT16 m, INT16 n, DOUBLE64 **MatrixT);
void MaddM_mis_D_mis(DOUBLE64 **Matrix1, DOUBLE64 **Matrix2, INT16 nrow, INT16 ncol, DOUBLE64 **MatrixR);
void MsubM_D_mis(DOUBLE64 **Matrix1, DOUBLE64 **Matrix2, INT16 nrow, INT16 ncol, DOUBLE64 **MatrixR);
void dM2d_mis(DOUBLE64 **Matrix, INT16 ar, INT16 ac, DOUBLE64 scalar, DOUBLE64 **MatrixR);
STATUS InvertMatrix_mis( DOUBLE64 **Matrix, INT16 n);
void fft_mis(INT16 Index, INT16 start_index, INT16 diff, DOUBLE64 *time, DOUBLE64 **Freqs_M, DOUBLE64 **tempFreqs_M);
void zero_padding_mis(INT16 I, INT16 N, DOUBLE64 *x, DOUBLE64 *newx);
void make_freq_axis_mis(INT16 Fs, INT16 temp1, DOUBLE64 *f);
DOUBLE64 vectors_dot_product_mis(DOUBLE64 *vec1, DOUBLE64 *vec2, INT16 wind_size);
DOUBLE64 Phase_Shift_Angle_mis(DOUBLE64 *vec1, DOUBLE64 *vec2, INT16 wind_size);
////////////// Start of FLOAT32 /////////////////////////////////////////////////////////////////
//INT8 BubbleSort_mis_2(INT8 *data_sort, INT8 num, INT8 contl);
void BubbleSort_mis(INT8 *data_sort, INT8 num, INT contl);
void ChangeArrayMemoryLayout_float_att_float_mis(INT32 r, INT32 c , FLOAT32 **ptr, FLOAT32 *Arr, UINT8 InitToZero);
void Zeros2D_float_float_mis(FLOAT32 **Matrix, INT32 row, INT32 col);
void Zeros1D_float_float_mis(FLOAT32 *vector, INT16 size);
FLOAT32 **Mem_Alloc_2d_mis_float_float_mis(INT16 row, INT16 col);
FLOAT32 *Mem_Alloc_1d_mis_float_float_mis(INT16 size);
FLOAT32 vectors_dot_product_mis_float_float_mis(FLOAT32 *vec1, FLOAT32 *vec2, INT16 wind_size);
FLOAT32 Phase_Shift_Angle_mis_float_float_mis(FLOAT32 *vec1, FLOAT32 *vec2, INT16 wind_size);
void LowPassFilter_float_float_mis(FLOAT32 *InputSignal, UINT8 fc, UINT8 fs, UINT8 FilterOrder, FLOAT32 *FilteredOutPut);
void LowPassFilter2_mis(FLOAT32 *InputSignal, UINT8 fc, UINT8 fs, UINT8 FilterOrder, FLOAT32 *FilteredOutPut, UINT8 N);
void Free_2d_float_float_mis(FLOAT32 **ppd);
void Free_1d_float_float_mis(FLOAT32 *pd);
FLOAT32 calculate_correlation_mis_float_float_mis(FLOAT32 *X, FLOAT32 *Y, INT16 dimension);
INT16 xcorr_c_float_float_mis(FLOAT32 *X, FLOAT32 *Y, INT16 N, FLOAT32 *r, INT16 * lags, INT16 control);
INT16 Calculate_PCA_float_float_mis(FLOAT32 **a1, UINT8 m, FLOAT32 **u, FLOAT32 v[2][2], FLOAT32 eps, FLOAT32 mean_x, FLOAT32 mean_y);
INT16 SVD_float_float_mis(FLOAT32 **a1, INT16 m, INT16 n, FLOAT32 **u, FLOAT32 **a, FLOAT32 **v, DOUBLE64 eps);
void SVD_1_float_float_mis(FLOAT32 **a, FLOAT32 *e, FLOAT32 *s, FLOAT32 **v, INT16 m, INT16 n);
void SVD_2_float_float_mis(FLOAT32 *fg, FLOAT32 *cs);
void MEM_mis_D_float_float_mis(FLOAT32 **Matrix, INT16 ri, INT16 re, INT16 ci, INT16 ce, FLOAT32 **CopyMatrix);
void Euler2Dcm_att_float_float_mis(FLOAT32 roll, FLOAT32 pitch, FLOAT32 heading, FLOAT32 **R);
void Euler2Dcm_att_float_float_3_3_mis(FLOAT32 roll, FLOAT32 pitch, FLOAT32 heading, FLOAT32 R[3][3]);

////////////// End of FLOAT32
void zero_matrix_entries_nxn_float_mis(FLOAT32 **Matrix, INT16 row, INT16 col);
void zero_padding_float_float_mis(INT16 I,INT16 N,FLOAT32 *x, FLOAT32 *newx);
void fft_mis_float_float_mis(INT16 Index, INT16 start_index, INT16 diff, FLOAT32 *time, FLOAT32 **Freqs_M, FLOAT32 **tempFreqs_M);

STATUS qr_decomposition_mis(FLOAT32 **A, INT16 m, INT16 n, FLOAT32 **Q, FLOAT32 **R);
void convert_to_identity_matrix_nxn_mis(FLOAT32 **A, UINT8 size);
INT16 Calculate_PCA_float_float_2_mis(FLOAT32 **a, UINT8 m, FLOAT32 **u, FLOAT32 v[2][2], FLOAT32 eps, FLOAT32 mean_x, FLOAT32 mean_y);
////////////// End of FLOAT32

// =========== End of Varies Attitude and Misalignment ===========
// =============================== C HEAD =====================================
// ============================================================================



// ----------------------------------------------------------------------------
// ------------------------------- ANDROID ------------------------------------
#define LOOPER_ID  1
//#define samp_per_sec  20
//#define  dt_us (1000000/samp_per_sec)

double tsca = 10e-9;


//  ---------
#define IF_POST_PROCESSING     0
#define DT_US_REAL_TIME   50000    // 1000000/20
#define DT_US_POST_PRO    100

char folder_post_pro[20] = "20151013072817"; // "20150727024527";

char buff_post[300] = "\0";
int da_temp_sensor_type[1] = {0};
double da_temp_sensor[4]= {0.00};

double t_start_4_wifi = 0.00;
double t_wifi = -1.00;
double t_wp;
int i_wp;

double t_start_4_gps_result = 0.00;
double t_gps_result = -1.00;
double t_gpsp;
int i_gpsp;


int i_line = 0;
double t_initial = 0.00;

#define N_MAX_WIFI_EPHS    6000
#define N_MAX_GPS_RESULT_EPHS    3000

typedef struct
{
    double t;
    char name_ap[MAX_AP_NUM][N_CHAR_MAC];
    float rss_ap[MAX_AP_NUM];
    int N_ap;
} DATA_WIFI_1_EPH;

typedef struct {
    DATA_WIFI_1_EPH  da[N_MAX_WIFI_EPHS];
    int N_eph;
} DATA_WIFI;


typedef struct
{
    double t[N_MAX_GPS_RESULT_EPHS];
    double lat[N_MAX_GPS_RESULT_EPHS];
    double lon[N_MAX_GPS_RESULT_EPHS];
    float hei[N_MAX_GPS_RESULT_EPHS];
    float accuracy[N_MAX_GPS_RESULT_EPHS];
    int N_eph;
} DATA_GPS_RESULT;

DATA_WIFI da_wifi;
DATA_GPS_RESULT da_gps_result;


void load_wifi(char *f_name_in_wifi, DATA_WIFI* da_wifi);
void load_wifi(char *f_name_in_wifi, DATA_WIFI* da_wifi)
{
	//LOGE("%s\n", f_name_in_wifi);
    FILE * f_in_wifi = NULL;
    char buff[300];
    int i;
    double t1 = -1.00;
    int d1;
    char s1[200], s2[200];
    double t_wifi_0 = -999.00;
    da_wifi->N_eph = 0;

    if((f_in_wifi = fopen(f_name_in_wifi,"rb")) == NULL){
        LOGE("The wifi file <%s> can not be opened.\n", f_name_in_wifi);
        exit(1);
    }

    while (!feof(f_in_wifi)) {
        if (fgets(buff,sizeof(buff),f_in_wifi)) {
            sscanf(buff,"%lf %s %d %s", &t1, s1, &d1, s2);
        }
        //LOGI("%lf %s %d %s", t1, s1, d1, s2);

        if (fabs(t1-t_wifi_0) > 10e-4) {
            da_wifi->N_eph ++;
            da_wifi->da[da_wifi->N_eph-1].N_ap = 1;
            da_wifi->da[da_wifi->N_eph-1].t = t1;
            strcpy(da_wifi->da[da_wifi->N_eph-1].name_ap[da_wifi->da[da_wifi->N_eph-1].N_ap-1], s1);
            da_wifi->da[da_wifi->N_eph-1].rss_ap[da_wifi->da[da_wifi->N_eph-1].N_ap-1] = d1;
            t_wifi_0 = t1;
        }
        else {
            da_wifi->da[da_wifi->N_eph-1].N_ap ++;
            strcpy(da_wifi->da[da_wifi->N_eph-1].name_ap[da_wifi->da[da_wifi->N_eph-1].N_ap-1], s1);
            da_wifi->da[da_wifi->N_eph-1].rss_ap[da_wifi->da[da_wifi->N_eph-1].N_ap-1] = d1;
            t_wifi_0 = t1;
        }
    }

    fclose(f_in_wifi);
}

void load_gps_result(char *f_name_in_gps_result, DATA_GPS_RESULT* da_gps_result);
void load_gps_result(char *f_name_in_gps_result, DATA_GPS_RESULT* da_gps_result)
{
    FILE * f_in_gps_result = NULL;
	char buff[300];
	int i;
	double temp[5];
	da_gps_result->N_eph = 0;

	if((f_in_gps_result = fopen(f_name_in_gps_result,"r")) == NULL){
		printf("The GPS result file <%s> can not be opened.\n", f_name_in_gps_result);
		exit(1);
	}

	while (!feof(f_in_gps_result)) {
		if (fgets(buff,sizeof(buff),f_in_gps_result)) {
			sscanf(buff,"%lf %lf %lf %lf %lf", temp, temp+1, temp+2, temp+3, temp+4);

			da_gps_result->N_eph ++;
			da_gps_result->t[da_gps_result->N_eph-1] = temp[0];
			da_gps_result->lat[da_gps_result->N_eph-1] = temp[1];
			da_gps_result->lon[da_gps_result->N_eph-1] = temp[2];
			da_gps_result->hei[da_gps_result->N_eph-1] = (float)temp[3];
			da_gps_result->accuracy[da_gps_result->N_eph-1] = (float)temp[4];
		}
	}

	fclose(f_in_gps_result);
}
// ===========


ASensorEventQueue* sensorEventQueue;
char str_path[300] = "";
char name_path[300] = "";
char f_DB_bin_str[300] = "";
char f_DB_bin_str_2[300] = "";
char f_DB_bin_str_3[300] = "";
char f_DB_bin_str_4[300] = "";
char f_DB_bin_str_5[300] = "";
char f_DB_bin_str_6[300] = "";
char f_DB_bin_diff_str[300] = "";
//char f_out_DB[300] = "";
char date4FileName_sensors[200] = "";
char date4FileName_wifi[200] = "";
char date4FileName_gps_result[200] = "";

char date4FileName_nav_sol[200] = "";
char date4FileName_wifi_sol[200] = "";

double haha[1] = {0.00};
FILE*  fp = NULL;
FILE*  fp_wifi_raw = NULL;
FILE*  fp_nav = NULL;
FILE*  fp_wifi = NULL;
FILE*  fp_gps_raw = NULL;

ASensorEvent event;
ASensorManager* sensorManager = NULL;
const ASensor* accSensor = NULL;
const ASensor* gyroSensor = NULL;
const ASensor* magSensor = NULL;
const ASensor* baroSensor = NULL;
const ASensor* temperSensor = NULL;

INT8 if_has_gyro = 0;
INT8 if_has_accel = 0;
INT8 if_has_baro = 0;
INT8 if_has_mag = 0;
INT8 if_has_temper = 0;


int type_da = 0;
double t_da = 0.00;
float da[3] = {0,0,0};

static int if_not_first_gyro = 0;
static int if_not_first_accel = 0;
static int if_not_first_mag = 0;
static int if_not_first_temper = 0;
static int if_not_first_baro = 0;

double t0_gyro = 0.00;
double t0_accel = 0.00;
double t0_mag = 0.00;
double t0_temper = 0.00;
double t0_baro = 0.00;


clock_t t_0, t_1;
double t_CPS = 1.0 * CLOCKS_PER_SEC;

// Only used in Android
Initialization_Pkt ini;
Sol_Pkt sol_pkt;
Meas meas;
int if_sensor_axis_change = 1;  // 1-Change axis after reading sensor outputs
double temp_4_sensor_axis_change = 0.00;
int if_pos_update_4_java = 0;
int if_pdr_initialization_completed = 0;


int if_show_wifi = 0;
int nn_wifi = 0;
char str_wifi_temp_from_java[6000] = "\0";
char str_wifi_temp_from_java_show[6000] = "\0";
char str_pdr_temp_to_java_show[200] = "\0";

static int if_this_is_not_first_wifi = 0;

//  ========   Jni functions ===========
long getCurrentTime();
jint Java_com_example_youli_helloindoor_MainActivity_getInt( JNIEnv* env, jobject this, jint xxx);
int Java_com_example_youli_helloindoor_MainActivity_setInt(JNIEnv *env, jobject object, jint xxxx_java);
jint Java_com_example_youli_helloindoor_MainActivity_getAvailPosPdr(JNIEnv *env, jobject object);
jdouble Java_com_example_youli_helloindoor_MainActivity_getPosNorth(JNIEnv *env, jobject object);
jdouble Java_com_example_youli_helloindoor_MainActivity_getPosEast(JNIEnv *env, jobject object);
jdouble Java_com_example_youli_helloindoor_MainActivity_getPosHeight(JNIEnv *env, jobject object);
jdouble Java_com_example_youli_helloindoor_MainActivity_getPdrLat(JNIEnv *env, jobject object);
jdouble Java_com_example_youli_helloindoor_MainActivity_getPdrLon(JNIEnv *env, jobject object);
jdouble Java_com_example_youli_helloindoor_MainActivity_getPdrHei(JNIEnv *env, jobject object);
jstring Java_com_example_youli_helloindoor_MainActivity_getStrPos(JNIEnv* pEnv, jobject pObj);

jint Java_com_example_youli_helloindoor_MainActivity_getAvailPosWiFi(JNIEnv *env, jobject object);
jdouble Java_com_example_youli_helloindoor_MainActivity_getWiFiTm(JNIEnv *env, jobject object);
jdouble Java_com_example_youli_helloindoor_MainActivity_getWiFiLat(JNIEnv *env, jobject object);
jdouble Java_com_example_youli_helloindoor_MainActivity_getWiFiLon(JNIEnv *env, jobject object);
jdouble Java_com_example_youli_helloindoor_MainActivity_getWiFiNorth(JNIEnv *env, jobject object);
jdouble Java_com_example_youli_helloindoor_MainActivity_getWiFiEast(JNIEnv *env, jobject object);
void Java_com_example_youli_helloindoor_MainActivity_setAvailPosWiFi(JNIEnv *env, jobject object, jint av);

jdouble Java_com_example_youli_helloindoor_MainActivity_getGPSTm(JNIEnv *env, jobject object);
jdouble Java_com_example_youli_helloindoor_MainActivity_getGPSLat(JNIEnv *env, jobject object);
jdouble Java_com_example_youli_helloindoor_MainActivity_getGPSLon(JNIEnv *env, jobject object);
jfloat Java_com_example_youli_helloindoor_MainActivity_getGPSHeight(JNIEnv *env, jobject object);
jfloat Java_com_example_youli_helloindoor_MainActivity_getGPSAccuracy(JNIEnv *env, jobject object);

jdouble Java_com_example_youli_helloindoor_MainActivity_getBaroHeight(JNIEnv *env, jobject object);
jint Java_com_example_youli_helloindoor_MainActivity_getFloorNum(JNIEnv *env, jobject object);
jfloat Java_com_example_youli_helloindoor_MainActivity_getHeight0(JNIEnv *env, jobject object);


void Java_com_example_youli_helloindoor_MainActivity_setAvailPosPdr(JNIEnv *env, jobject object, jint av);
void Java_com_example_youli_helloindoor_MainActivity_setIniPdrType(JNIEnv *env, jobject object, jint type);
void Java_com_example_youli_helloindoor_MainActivity_setIniPdrLat(JNIEnv *env, jobject object, jdouble lat);
void Java_com_example_youli_helloindoor_MainActivity_setIniPdrLon(JNIEnv *env, jobject object, jdouble lon);
void Java_com_example_youli_helloindoor_MainActivity_setIniPdrNorth(JNIEnv *env, jobject object, jdouble lat);
void Java_com_example_youli_helloindoor_MainActivity_setIniPdrEast(JNIEnv *env, jobject object, jdouble lon);
void Java_com_example_youli_helloindoor_MainActivity_setIniPdrhei(JNIEnv *env, jobject object, jdouble hei);
void Java_com_example_youli_helloindoor_MainActivity_setIniPdrHeading(JNIEnv *env, jobject object, jfloat heading);
void Java_com_example_youli_helloindoor_MainActivity_setIniPdrCompleted(JNIEnv *env, jobject object, jint a);

JNIEXPORT  jstring JNICALL  Java_com_example_youli_helloindoor_MainActivity_readFromAssets(JNIEnv* env,jclass tis,jobject assetManager,jstring filename);
void Java_com_example_youli_helloindoor_MainActivity_string2Jni(JNIEnv *env, jobject object, jobjectArray stringArray);
void Java_com_example_youli_helloindoor_MainActivity_string_path2Jni(JNIEnv *env, jobject object, jobjectArray stringArray);
void Java_com_example_youli_helloindoor_MainActivity_double2Jni(JNIEnv *env, jobject object, jdoubleArray doubleArray);
void Java_com_example_youli_helloindoor_MainActivity_sensorValue( JNIEnv* env, jobject thiz );
static int get_sensor_events(int fd, int events, void* data);
jstring Java_com_example_youli_helloindoor_MainActivity_writeToFile( JNIEnv* env,jobject thiz );
void Java_com_example_youli_helloindoor_MainActivity_disableSensor( JNIEnv* env,jobject thiz );
jstring  Java_com_example_youli_helloindoor_MainActivity_naGetHelloNDKStr(JNIEnv* pEnv, jobject pObj);

JNIEXPORT void Java_com_example_youli_helloindoor_MainActivity_passAvailbilityWiFi(JNIEnv *env, jobject obj, jint if_wifi_avail);
JNIEXPORT void Java_com_example_youli_helloindoor_MainActivity_passWiFiNAp(JNIEnv *env, jobject obj, jint N_wifi);
JNIEXPORT void Java_com_example_youli_helloindoor_MainActivity_passWiFiTm(JNIEnv *env, jobject obj, jdouble tm);
JNIEXPORT void Java_com_example_youli_helloindoor_MainActivity_passWiFiRssInt(JNIEnv *env, jobject obj, jintArray ptr);
void Java_com_example_youli_helloindoor_MainActivity_passWiFiStrMac(JNIEnv *env, jobject object, jobjectArray stringArray);

JNIEXPORT void Java_com_example_youli_helloindoor_MainActivity_passAvailbilityGPS(JNIEnv *env, jobject obj, jint if_gps_avail);
JNIEXPORT void Java_com_example_youli_helloindoor_MainActivity_passGPSTm(JNIEnv *env, jobject obj, jdouble t_gps);
JNIEXPORT void Java_com_example_youli_helloindoor_MainActivity_passGPSLat(JNIEnv *env, jobject obj, jdouble gps_lat);
JNIEXPORT void Java_com_example_youli_helloindoor_MainActivity_passGPSLon(JNIEnv *env, jobject obj, jdouble gps_lon);
JNIEXPORT void Java_com_example_youli_helloindoor_MainActivity_passGPSHei(JNIEnv *env, jobject obj, jfloat gps_height);
JNIEXPORT void Java_com_example_youli_helloindoor_MainActivity_passGPSAccuracy(JNIEnv *env, jobject obj, jfloat gps_accuracy);

JNIEXPORT void JNICALL Java_com_example_youli_helloindoor_MainActivity_setStrWiFi(JNIEnv *env, jobject clazz, jstring path);
jstring Java_com_example_youli_helloindoor_MainActivity_getStrWiFi(JNIEnv* pEnv, jobject pObj);
int find_same_char_in_string(char* str, char c, int *ii_same);

JNIEXPORT  jstring JNICALL  Java_com_example_youli_helloindoor_MainActivity_readFromAssets(JNIEnv* env,jclass tis,jobject assetManager,jstring filename)
{
//   jstring resultStr;
//   LOGI("ReadAssets");
//   AAssetManager* mgr = AAssetManager_fromJava(env, assetManager);
//   if(mgr==NULL)
//   {
//      LOGI(" %s","AAssetManager==NULL");
//      return ;
//   }
//
//    /*获取文件名并打开*/
//   jboolean iscopy;
//   const char *mfile = (*env)->GetStringUTFChars(env, filename, &iscopy);
//   AAsset* asset = AAssetManager_open(mgr, mfile,AASSET_MODE_UNKNOWN);
//   (*env)->ReleaseStringUTFChars(env, filename, mfile);
//   if(asset==NULL)
//   {
//      LOGI(" %s","asset==NULL");
//      return ;
//   }
//   /*获取文件大小*/
//   off_t bufferSize = AAsset_getLength(asset);
//   LOGI("file size         : %d\n",bufferSize);
//   char *buffer=(char *)malloc(bufferSize+1);
//   buffer[bufferSize]=0;
//   int numBytesRead = AAsset_read(asset, buffer, bufferSize);
//   LOGI(": %s",buffer);
//   resultStr = (*env)->NewStringUTF(env, buffer);
//   free(buffer);
//    /*关闭文件*/
//   AAsset_close(asset);

//   return resultStr;
	return ("\n");
}

JNIEXPORT void Java_com_example_youli_helloindoor_MainActivity_passAvailbilityWiFi(JNIEnv *env, jobject obj, jint if_wifi_avail)
{
	meas.avail_wifi = if_wifi_avail;
	//LOGD("meas.avail_wifi = %d\n", meas.avail_wifi);
}

JNIEXPORT void Java_com_example_youli_helloindoor_MainActivity_passWiFiNAp(JNIEnv *env, jobject obj, jint NN)
{
	int N_wifi = NN;
	meas.N_wifi = NN;
}

JNIEXPORT void Java_com_example_youli_helloindoor_MainActivity_passWiFiTm(JNIEnv *env, jobject obj, jdouble tm)
{
	meas.t_wifi = tm;
}

JNIEXPORT void Java_com_example_youli_helloindoor_MainActivity_passWiFiRssInt
  (JNIEnv *env, jobject obj, jintArray ptr) {

  int i;
  printf("Hello from JNI\n");
  jsize len = (*env)->GetArrayLength(env, ptr);
  jint *body = (*env)->GetIntArrayElements(env, ptr, 0);
  //for (i=0; i < len; i++)
  for (i=0; i < meas.N_wifi; i++)
  {
	  meas.rss_ap[i] = (float)body[i];
	  //LOGI("%d\n", body[i]);
	  //LOGD("%.1f\n", meas.rss_ap[i]);
  }
    //printf("Hello from JNI - element: %d\n", body[i]);
  (*env)->ReleaseIntArrayElements(env, ptr, body, 0);
}

void Java_com_example_youli_helloindoor_MainActivity_passWiFiStrMac(JNIEnv *env, jobject object, jobjectArray stringArray) {
    int stringCount = (*env)->GetArrayLength(env,stringArray);
    int i;
    //for (i=0; i<stringCount; i++) {
    for (i=0; i<meas.N_wifi; i++) {
        jstring string = (jstring) (*env)->GetObjectArrayElement(env, stringArray, i);
        const char *rawString = (*env)->GetStringUTFChars(env, string, 0);

        strcpy(meas.name_ap[i],rawString);
        //LOGI("%s\n", meas.name_ap[i]);


        (*env)->ReleaseStringUTFChars(env,string,rawString);
    }
}



JNIEXPORT void Java_com_example_youli_helloindoor_MainActivity_passAvailbilityGPS(JNIEnv *env, jobject obj, jint if_gps_avail)
{
	meas.avail_gps_result = if_gps_avail;
}

JNIEXPORT void Java_com_example_youli_helloindoor_MainActivity_passGPSTm(JNIEnv *env, jobject obj, jdouble t_gps)
{
	meas.t_gps_result = t_gps;
}

JNIEXPORT void Java_com_example_youli_helloindoor_MainActivity_passGPSLat(JNIEnv *env, jobject obj, jdouble gps_lat)
{
	meas.lat_gps_result = gps_lat;
}

JNIEXPORT void Java_com_example_youli_helloindoor_MainActivity_passGPSLon(JNIEnv *env, jobject obj, jdouble gps_lon)
{
	meas.lon_gps_result = gps_lon;
}

JNIEXPORT void Java_com_example_youli_helloindoor_MainActivity_passGPSHei(JNIEnv *env, jobject obj, jfloat gps_height)
{
	meas.height_gps_result = gps_height;
}

JNIEXPORT void Java_com_example_youli_helloindoor_MainActivity_passGPSAccuracy(JNIEnv *env, jobject obj, jfloat gps_accuracy)
{
	meas.accuracy_gps_result = gps_accuracy;
}


JNIEXPORT void JNICALL Java_com_example_youli_helloindoor_MainActivity_setStrWiFi(JNIEnv *env, jobject clazz, jstring path)
{
	if(path == NULL)
	{
		return;
	}
	jboolean isCopy;
	const char *pathStr = (*env)->GetStringUTFChars(env, path, &isCopy);
	if (pathStr == NULL) {
		return;
	}

	if (IF_POST_PROCESSING == 0)
	{
		strncpy(str_wifi_temp_from_java, pathStr, sizeof(str_wifi_temp_from_java));
		//LOGI("%s\n", str_wifi_temp_from_java);
		strcpy(str_wifi_temp_from_java_show, "\0");

		int ii_kama_str_temp[1500] = {0};
		int ii;
		char str_temp_ss[50] = "\0";
		char str_temp_1_ap[120] = "\0";
		int num_kama_str_temp = find_same_char_in_string(str_wifi_temp_from_java, ',', ii_kama_str_temp);
		meas.N_wifi = (num_kama_str_temp - 2) / 3;

		char str_cut_1[20] = "";
		strncpy(str_cut_1, str_wifi_temp_from_java+ii_kama_str_temp[0]+1, ii_kama_str_temp[1]-ii_kama_str_temp[0]-1);
		meas.t_wifi = atof(str_cut_1); // WiFi measure timestamp

		/** REVERSE Foreach wifi AP trovato salva i dati */
		for (ii=0; ii<meas.N_wifi; ii++)
		{
			char str_cut_3[20] = "";
			strncpy(str_cut_3, str_wifi_temp_from_java+ii_kama_str_temp[3*ii+1]+1, ii_kama_str_temp[3*ii+2]-ii_kama_str_temp[3*ii+1]-1);
			strcpy(meas.name_ap[ii], str_cut_3);
			char str_cut_4[10] = "";
			strncpy(str_cut_4, str_wifi_temp_from_java+ii_kama_str_temp[3*ii+2]+1, ii_kama_str_temp[3*ii+3]-ii_kama_str_temp[3*ii+2]-1);
			meas.rss_ap[ii] = (float)atoi(str_cut_4);

			char str_cut_5[50] = "";
			strncpy(str_cut_5, str_wifi_temp_from_java+ii_kama_str_temp[3*ii+3]+1, ii_kama_str_temp[3*ii+4]-ii_kama_str_temp[3*ii+3]-1);

			sprintf(str_temp_1_ap, "%.3f %s %.0f %s\n", meas.t_wifi, meas.name_ap[ii], meas.rss_ap[ii], str_cut_5);
			strcat(str_wifi_temp_from_java_show, str_temp_1_ap);
		}
		fprintf(fp_wifi_raw, "%s", str_wifi_temp_from_java_show);

		meas.avail_wifi = 1;
	}
	//

	(*env)->ReleaseStringUTFChars(env, path, pathStr);
}

jstring Java_com_example_youli_helloindoor_MainActivity_getStrWiFi(JNIEnv* pEnv, jobject pObj)
{
    return (*pEnv)->NewStringUTF(pEnv, str_wifi_temp_from_java_show);
}

int find_same_char_in_string(char* str, char c, int *ii_same)
{
    int n_str, ii, num_same_char;

    n_str = (int)strlen(str);
    num_same_char = 0;

    for (ii=0; ii<n_str; ii++) {
        if(str[ii] == c)
        {
            ii_same[num_same_char] = ii;
            num_same_char ++;
        }
    }
    return num_same_char;
}


//int Java_com_example_youli_helloindoor_MainActivity_pass_int_N_wifi(JNIEnv *env, jobject object, jint nn);
//JNIEXPORT jint JNICALL Java_com_example_youli_helloindoor_MainActivity_pass_int_rss(JNIEnv *env, jobject obj, jintArray arr);
//JNIEXPORT jint JNICALL
//Java_com_example_youli_helloindoor_MainActivity_pass_int_rss(JNIEnv *env, jobject obj, jintArray arr)
// {
//
//     // initializations, declarations, etc
//     jint *c_array;
//     jint i = 0;
//
//     // get a pointer to the array
//     c_array = (*env)->GetIntArrayElements(env, arr, NULL);
//
//     // do some exception checking
//     if (c_array == NULL) {
//         return -1; /* exception occurred */
//     }
//
//
//     // release the memory so java can have it again
//     (*env)->ReleaseIntArrayElements(env, arr, c_array, 0);
//
//     // return something, or not.. it's up to you
//     return 0;
// }
// ===========================================
// Definitions
long getCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv,NULL);
    //printf("tv:%lld,%ld\n",(long long)tv.tv_sec*1000,tv.tv_usec/1000);
    return (long)tv.tv_sec * 1000 + (long)tv.tv_usec / 1000;
}

jint Java_com_example_youli_helloindoor_MainActivity_getInt( JNIEnv* env, jobject this, jint xxx)
{
    return xxx;
}

jint Java_com_example_youli_helloindoor_MainActivity_getAvailPosPdr(JNIEnv *env, jobject object)
{
    return if_pos_update_4_java;
}
jdouble Java_com_example_youli_helloindoor_MainActivity_getPdrTm(JNIEnv *env, jobject object)
{
    return sol_pkt.sol_t_sol_pdr;
}
jdouble Java_com_example_youli_helloindoor_MainActivity_getPdrLat(JNIEnv *env, jobject object)
{
    return sol_pkt.sol_pdr[0];
}
jdouble Java_com_example_youli_helloindoor_MainActivity_getPdrLon(JNIEnv *env, jobject object)
{
    return sol_pkt.sol_pdr[1];
}
jdouble Java_com_example_youli_helloindoor_MainActivity_getPdrHei(JNIEnv *env, jobject object)
{
    return sol_pkt.sol_pdr[2];
}

jstring Java_com_example_youli_helloindoor_MainActivity_getStrPos(JNIEnv* pEnv, jobject pObj)
{
    float N_temp1 = (sol_pkt.sol_pdr[0]*D2R - ZERO_POS[0])*RM;
    float E_temp1 = (sol_pkt.sol_pdr[1]*D2R - ZERO_POS[1])*RN * cos(REAL_LAT);
    
    sprintf(str_pdr_temp_to_java_show, "%.3f,%.2f,%.2f,%.2f,",sol_pkt.sol_t_sol_pdr,E_temp1,N_temp1,sol_pkt.sol_pdr[2]);
    if_pos_update_4_java = 0;
    return (*pEnv)->NewStringUTF(pEnv, str_pdr_temp_to_java_show);
}


jint Java_com_example_youli_helloindoor_MainActivity_getAvailPosWiFi(JNIEnv *env, jobject object)
{
	return if_show_wifi;
    //return sol_pkt.avail_sol_wifi;
}
jdouble Java_com_example_youli_helloindoor_MainActivity_getWiFiTm(JNIEnv *env, jobject object)
{
    return sol_pkt.sol_t_sol_wifi;
}
jdouble Java_com_example_youli_helloindoor_MainActivity_getWiFiLat(JNIEnv *env, jobject object)
{
    return sol_pkt.sol_wifi[0];
}
jdouble Java_com_example_youli_helloindoor_MainActivity_getWiFiLon(JNIEnv *env, jobject object)
{
    return sol_pkt.sol_wifi[1];
}
jdouble Java_com_example_youli_helloindoor_MainActivity_getWiFiNorth(JNIEnv *env, jobject object)
{
	return ((sol_pkt.sol_wifi[0]*D2R - ZERO_POS[0])*RM);
}
jdouble Java_com_example_youli_helloindoor_MainActivity_getWiFiEast(JNIEnv *env, jobject object)
{
	return ((sol_pkt.sol_wifi[1]*D2R  - ZERO_POS[1])*RN * cos(REAL_LAT));
}

jdouble Java_com_example_youli_helloindoor_MainActivity_getGPSTm(JNIEnv *env, jobject object)
{
	return meas.t_gps_result;
}

jdouble Java_com_example_youli_helloindoor_MainActivity_getGPSLat(JNIEnv *env, jobject object)
{
	return meas.lat_gps_result;
}

jdouble Java_com_example_youli_helloindoor_MainActivity_getGPSLon(JNIEnv *env, jobject object)
{
	return meas.lon_gps_result;
}

jfloat Java_com_example_youli_helloindoor_MainActivity_getGPSHeight(JNIEnv *env, jobject object)
{
	return meas.height_gps_result;
}

jfloat Java_com_example_youli_helloindoor_MainActivity_getGPSAccuracy(JNIEnv *env, jobject object)
{
	return meas.accuracy_gps_result;
}


void Java_com_example_youli_helloindoor_MainActivity_setAvailPosWiFi(JNIEnv *env, jobject object, jint av) {
	if_show_wifi = av;
}

jdouble Java_com_example_youli_helloindoor_MainActivity_getBaroHeight(JNIEnv *env, jobject object)
{
    return (double)sol_pkt.height_baro_sm;
}
jint Java_com_example_youli_helloindoor_MainActivity_getFloorNum(JNIEnv *env, jobject object)
{
    return sol_pkt.floor_num;
}


jfloat Java_com_example_youli_helloindoor_MainActivity_getHeight0(JNIEnv *env, jobject object)
{
    return sol_pkt.height0;
}

jdouble Java_com_example_youli_helloindoor_MainActivity_getPosNorth(JNIEnv *env, jobject object)
{
	return ((sol_pkt.sol_pdr[0]*D2R - ZERO_POS[0])*RM);
}
jdouble Java_com_example_youli_helloindoor_MainActivity_getPosEast(JNIEnv *env, jobject object)
{
	return ((sol_pkt.sol_pdr[1]*D2R  - ZERO_POS[1])*RN * cos(REAL_LAT));
}
jdouble Java_com_example_youli_helloindoor_MainActivity_getPosHeight(JNIEnv *env, jobject object)
{
	return sol_pkt.sol_pdr[2];
}

int Java_com_example_youli_helloindoor_MainActivity_setInt(JNIEnv *env, jobject object, jint xxxx_java) {
	int xxxx = xxxx_java;
	return xxxx;
}


void Java_com_example_youli_helloindoor_MainActivity_string2Jni(JNIEnv *env, jobject object, jobjectArray stringArray) {
    int stringCount = (*env)->GetArrayLength(env,stringArray);
    int i;
    for (i=0; i<stringCount; i++) {
        jstring string = (jstring) (*env)->GetObjectArrayElement(env, stringArray, i);
        const char *rawString = (*env)->GetStringUTFChars(env, string, 0);
        // Don't forget to call `ReleaseStringUTFChars` when you're done.
        if (i==0) {
        	//strcpy(str_path, rawString);
        	if (IF_POST_PROCESSING == 0) {
        		strcpy(str_path, rawString);
        	}
        	else {
        		strncpy(str_path, rawString, strlen(rawString)-15);
				strcat(str_path, folder_post_pro);
				strcat(str_path, "/");
				LOGI("%s\n", str_path);
        	}

		}
        else if (i == 1) {
        	strcpy(name_path, rawString);
        	strcpy(f_DB_bin_str,name_path);
        	strcpy(f_DB_bin_diff_str,name_path);
        	//strcat(f_DB_bin_str, "wifi_DB_matrix_0_5_2_2_100_0_0.bin");

//        	strcat(f_DB_bin_str, "wifi_DB_all_DB_13.bin");
//            strcat(f_DB_bin_diff_str, "wifi_DB_df_all_DB_13.bin");   // Parking


			strcat(f_DB_bin_str, "wifi_DB_matrix_0_1_0_2_100_0_0_KCCI3.bin");
			//strcat(f_DB_bin_str, "wifi_DB_matrix_0_1_2_2_100_0_0.bin");
            strcat(f_DB_bin_diff_str, "wifi_DB_matrix_0_5_2_2_100_0_0_diff_fake.bin");   // Parking


            strcpy(f_DB_bin_str_2,name_path);
			strcat(f_DB_bin_str_2, "wifi_DB_matrix_0_2_0_2_100_0_0_KCCI2.bin");
			strcpy(f_DB_bin_str_3,name_path);
			strcat(f_DB_bin_str_3, "wifi_DB_matrix_0_3_0_2_100_0_0_KCCI1.bin");
			strcpy(f_DB_bin_str_4,name_path);
			strcat(f_DB_bin_str_4, "wifi_DB_matrix_0_8_0_2_100_0_0_PDC1.bin");
			strcpy(f_DB_bin_str_5,name_path);
			strcat(f_DB_bin_str_5, "wifi_DB_matrix_0_7_0_2_100_0_0_PDC2.bin");
			strcpy(f_DB_bin_str_6,name_path);
			strcat(f_DB_bin_str_6, "wifi_DB_matrix_0_7_0_2_100_0_0_PDC3.bin");

//        	strcat(f_DB_bin_str, "wifi_DB_all_DB_13_co.bin");
//        	strcat(f_DB_bin_diff_str, "wifi_DB_df_all_DB_13_co.bin");

        	//strcpy(f_out_DB,name_path);
        	//strcat(f_out_DB, "db.txt");
        }

        (*env)->ReleaseStringUTFChars(env,string,rawString);
    }
}

void Java_com_example_youli_helloindoor_MainActivity_string_path2Jni(JNIEnv *env, jobject object, jobjectArray stringArray) {
//    int stringCount = (*env)->GetArrayLength(env,stringArray);
//    int i;
//    for (i=0; i<stringCount; i++) {
//        jstring string = (jstring) (*env)->GetObjectArrayElement(env, stringArray, i);
//        const char *rawString = (*env)->GetStringUTFChars(env, string, 0);
//        // Don't forget to call `ReleaseStringUTFChars` when you're done.
//
//        strcpy(name_path, rawString);
//
//        (*env)->ReleaseStringUTFChars(env,string,rawString);
//    }
}

void Java_com_example_youli_helloindoor_MainActivity_setAvailPosPdr(JNIEnv *env, jobject object, jint av) {
	if_pos_update_4_java = av;
}
void Java_com_example_youli_helloindoor_MainActivity_setIniPdrType(JNIEnv *env, jobject object, jint type){
	ini.type_initializaiton = type;
	if (ini.type_initializaiton == 0)
	{
		//if_pdr_initialization_completed = 1;    // Use defined initial heading and position
	}
	else
	{
		// if_pdr_initialization_completed = 0;    // Set initial position and heading manually
	}
}
void Java_com_example_youli_helloindoor_MainActivity_setIniPdrLat(JNIEnv *env, jobject object, jdouble lat){
	ini.ini_pos[0] = lat*D2R;
}
void Java_com_example_youli_helloindoor_MainActivity_setIniPdrLon(JNIEnv *env, jobject object, jdouble lon){
	ini.ini_pos[1] = lon*D2R;
}

void Java_com_example_youli_helloindoor_MainActivity_setIniPdrNorth(JNIEnv *env, jobject object, jdouble north){
	ini.ini_pos[0] = north/RM+ZERO_POS[0];
}
void Java_com_example_youli_helloindoor_MainActivity_setIniPdrEast(JNIEnv *env, jobject object, jdouble east){
	ini.ini_pos[1] = east/(RN * cos(REAL_LAT)) + ZERO_POS[1];
}

void Java_com_example_youli_helloindoor_MainActivity_setIniPdrhei(JNIEnv *env, jobject object, jdouble hei){
	ini.ini_pos[2] = hei;
}
void Java_com_example_youli_helloindoor_MainActivity_setIniPdrHeading(JNIEnv *env, jobject object, jfloat heading){
	ini.ini_heading_platform = heading * D2R;
}
void Java_com_example_youli_helloindoor_MainActivity_setIniPdrCompleted(JNIEnv *env, jobject object, jint a){
	if_pdr_initialization_completed = 1;
}

void Java_com_example_youli_helloindoor_MainActivity_double2Jni(JNIEnv *env, jobject object, jdoubleArray doubleArray) {
	//jdouble* pJdouble = NULL;
	//pJdouble = (*env)->GetByteArrayElements(env, doubleArray, 0);
	//haha[0] = pJdouble[0];
	//LOGI("DouBLEEEE ===== %f", haha[0]);
}


void
Java_com_example_youli_helloindoor_MainActivity_sensorValue( JNIEnv* env, jobject thiz ) {

	int dt_us;
	strcpy(date4FileName_sensors, str_path);
	strcpy(date4FileName_wifi,str_path);
	strcpy(date4FileName_gps_result,str_path);
	strcpy(date4FileName_nav_sol, str_path);
	strcpy(date4FileName_wifi_sol, str_path);
	strcat(date4FileName_sensors,"sensors.txt");
	//strcat(date4FileName_wifi,"wifi_c.txt");
	strcat(date4FileName_wifi,"wifi.txt");
	strcat(date4FileName_gps_result,"gps.txt");
	if (IF_POST_PROCESSING == 0) {
		strcat(date4FileName_nav_sol,"nav_output_so.txt");
		strcat(date4FileName_wifi_sol,"wifi_output_so.txt");
	}
	else {
		strcat(date4FileName_nav_sol,"nav_output_post.txt");
		strcat(date4FileName_wifi_sol,"wifi_output_post.txt");
	}


	if (IF_POST_PROCESSING == 0) {
        fp =  fopen(date4FileName_sensors,"wb+");
		fp_wifi_raw =  fopen(date4FileName_wifi,"wb+");
		fp_gps_raw = fopen(date4FileName_gps_result,"wb+");
	}
	else
	{
		fp =  fopen(date4FileName_sensors,"rb+");

		if (IF_USE_WIFI)
		{
			//fp_wifi_raw =  fopen(date4FileName_wifi,"rb+");
			load_wifi(date4FileName_wifi, &da_wifi);
		}
		if(IF_USE_GPS_RESULT)
		{
			load_gps_result(date4FileName_gps_result, &da_gps_result);
		}
	}

	fp_nav = fopen(date4FileName_nav_sol,"wb+");
	fp_wifi = fopen(date4FileName_wifi_sol,"wb+");

	// The place for some initialization
	sol_pkt.avail_sol_pdr = 0;
	sol_pkt.sol_t_sol_pdr = 0.00;
	sol_pkt.sol_pdr[0] = 0.00;
	sol_pkt.sol_pdr[1] = 0.00;
	sol_pkt.sol_pdr[2] = 0.00;
	sol_pkt.avail_step = 0;
	sol_pkt.sol_t_step = 0.00;
	sol_pkt.sol_step = 0.00;
	sol_pkt.avail_heading = 0;
	sol_pkt.sol_t_heading = 0.00;
	sol_pkt.sol_heading = 0.00;

	if (IF_POST_PROCESSING == 0)
    	dt_us = DT_US_REAL_TIME;
    else
    	dt_us = DT_US_POST_PRO;


    int events, ident;
    void* sensor_data = malloc(1000);

    ALooper* looper = ALooper_forThread();

    if(looper == NULL)
    {
        looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
    }

    sensorManager = ASensorManager_getInstance();
    sensorEventQueue = ASensorManager_createEventQueue(sensorManager, looper, 3, get_sensor_events, sensor_data);

    if ((accSensor = ASensorManager_getDefaultSensor(sensorManager, ASENSOR_TYPE_ACCELEROMETER)) != NULL){
    	 if_has_accel = 1;
    	 ASensorEventQueue_enableSensor(sensorEventQueue, accSensor);
    	 ASensorEventQueue_setEventRate(sensorEventQueue, accSensor, dt_us);
    }

    if ((gyroSensor = ASensorManager_getDefaultSensor(sensorManager, ASENSOR_TYPE_GYROSCOPE))!= NULL){
    	if_has_gyro = 1;
        ASensorEventQueue_enableSensor(sensorEventQueue, gyroSensor);
        ASensorEventQueue_setEventRate(sensorEventQueue, gyroSensor,dt_us);
    }
    if ((magSensor = ASensorManager_getDefaultSensor(sensorManager, ASENSOR_TYPE_MAGNETIC_FIELD))!= NULL){
    	if_has_mag = 1;
    	ASensorEventQueue_enableSensor(sensorEventQueue, magSensor);
    	ASensorEventQueue_setEventRate(sensorEventQueue, magSensor, dt_us);
    }
    if ((temperSensor = ASensorManager_getDefaultSensor(sensorManager, 13))!= NULL){
    	if_has_temper = 1;
        ASensorEventQueue_enableSensor(sensorEventQueue, temperSensor);
        ASensorEventQueue_setEventRate(sensorEventQueue, temperSensor, dt_us);
    }
    if ((baroSensor = ASensorManager_getDefaultSensor(sensorManager, 6)) != NULL){
    	if_has_baro = 1;
    	ASensorEventQueue_enableSensor(sensorEventQueue, baroSensor);
    	ASensorEventQueue_setEventRate(sensorEventQueue, baroSensor, dt_us);
    }
}

static int get_sensor_events(int fd, int events, void* data) {
  ASensorEvent event;
  int ii = 0;
  //ASensorEventQueue* sensorEventQueue;
//  while (ASensorEventQueue_getEvents(sensorEventQueue, &event, 1) > 0
//  		 && if_pdr_initialization_completed == 1)
  while (ASensorEventQueue_getEvents(sensorEventQueue, &event, 1) > 0)
  {
	  if (IF_POST_PROCESSING == 0)
	  {
		  if (IF_USE_WIFI && meas.avail_wifi == 1)
		  {
			  meas.avail_gyro = 0;
			  meas.avail_accel = 0;
			  meas.avail_mag = 0;
			  meas.avail_baro = 0;

			  //LOGE("have wifi\n");
			  //LOGD("have wifi\n");
			  //indoor_navigation_withWiFi_kf(&ini, meas, &sol_pkt);

			  if (sol_pkt.avail_sol_wifi) {
				  if_show_wifi = 1;

//				  if (if_this_is_not_first_wifi == 1)
//				  {
//					  sol_pkt.sol_pdr[0] = sol_pkt.sol_wifi[0];
//					  sol_pkt.sol_pdr[1] = sol_pkt.sol_wifi[1];
//					  sol_pkt.sol_pdr[2] = 0.0;
//					  sol_pkt.sol_t_sol_pdr = sol_pkt.sol_t_sol_wifi;
//					  sol_pkt.avail_sol_pdr = 1;
//					  if_pos_update_4_java = 1;
//				  }
//
//				  if (if_this_is_not_first_wifi == 0)
//				  {
//					  if_this_is_not_first_wifi = 1;
//				  }

				  //fprintf(fp_wifi, "%.3f %.8f %.8f\n", sol_pkt.sol_t_sol_wifi, sol_pkt.sol_wifi[0], sol_pkt.sol_wifi[1]);
				  //printf("%.3f\n", sol_pkt.sol_t_sol_wifi);
				  //LOGE("have wifi result 22222222222222222222222222222222222222222\n");
			  }

			  meas.avail_wifi = 0;
		  }

		  if (IF_USE_GPS_RESULT && meas.avail_gps_result == 1)
	  	  {
			  meas.avail_gyro = 0;
			  meas.avail_accel = 0;
			  meas.avail_mag = 0;
			  meas.avail_baro = 0;

			  //indoor_navigation_withWiFi_kf(&ini, meas, &sol_pkt);
			  meas.avail_gps_result = 0;

			  char str_gps_1_eph[100] = "";
			  //sprintf(str_gps_1_eph, "%.3f %.8f %.8f %.3f %.3f\n", meas.t_gps_result, meas.lat_gps_result,
			  		  //meas.lon_gps_result, meas.height_gps_result, meas.accuracy_gps_result);
			  //fprintf(fp_gps_raw, "%s", str_gps_1_eph);
	  	  }

		  if(event.type == ASENSOR_TYPE_GYROSCOPE)
		  {
			  type_da = 1;
			  da[0] = event.vector.x / D2R;   //rad/s -> deg/s
			  da[1] = event.vector.y / D2R;
			  da[2] = event.vector.z / D2R;
			  t_da = event.timestamp/1000000000.0;
			  if(if_not_first_gyro)
			  {
				  t_da -= t0_gyro;
			  }
			  else
			  {
				  if_not_first_gyro = 1;
				  t0_gyro = t_da;
				  t_da = 0;
			  }

			  if (if_sensor_axis_change)
			  {   // Change axis
				  temp_4_sensor_axis_change = da[0];
				  da[0] = da[1];
				  da[1] = temp_4_sensor_axis_change;
				  da[2] *= -1;
			  }


			  meas.avail_gyro = 1;
			  meas.t_gyro = t_da;
			  meas.da_gyro[0] = da[0];
			  meas.da_gyro[1] = da[1];
			  meas.da_gyro[2] = da[2];
			  meas.avail_accel = 0;
			  meas.avail_mag = 0;
			  meas.avail_baro = 0;

			  indoor_navigation_withWiFi_kf(&ini, meas, &sol_pkt);
			  if (sol_pkt.avail_sol_pdr)
			  {
				  if_pos_update_4_java = 1;
				  fprintf(fp_nav, "%.4f %.8f %.8f %.3f\n",
							sol_pkt.sol_t_sol_pdr, sol_pkt.sol_pdr[0], sol_pkt.sol_pdr[1], sol_pkt.sol_pdr[2]);
			  }
			  LOGD("Gyro %d\t%.3f\t%.6f\t%.6f\t%.6f\n", type_da,t_da, da[0], da[1], da[2]);
			  //LOGD("str_path %s\n", str_path);
			  //LOGI("name_path %s\n", name_path);
			  fprintf(fp,"%d\t%.3f\t%.6f\t%.6f\t%.6f\n",type_da,t_da, da[0], da[1], da[2]);
		  }
		  else if(event.type == ASENSOR_TYPE_ACCELEROMETER)
		  {
			  type_da = 2;
			  t_da = event.timestamp/1000000000.0;
			  da[0] = event.acceleration.x;
			  da[1] = event.acceleration.y;
			  da[2] = event.acceleration.z;
			  if (if_sensor_axis_change) {   // Change axis
				  temp_4_sensor_axis_change = da[0];
				  da[0] = da[1];
				  da[1] = temp_4_sensor_axis_change;
				  da[2] *= -1;
			  }
			  if(if_not_first_accel)
			  {
				  t_da -= t0_accel;
			  }
			  else
			  {
				  if_not_first_accel = 1;
				  t0_accel = t_da;
				  t_da = 0;
			  }

			  meas.avail_accel = 1;
			  meas.t_accel = t_da;
			  meas.da_accel[0] = da[0];
			  meas.da_accel[1] = da[1];
			  meas.da_accel[2] = da[2];
			  meas.avail_gyro = 0;
			  meas.avail_mag = 0;
			  meas.avail_baro = 0;

			  indoor_navigation_withWiFi_kf(&ini, meas, &sol_pkt);

			  if (sol_pkt.avail_sol_pdr){
				  if_pos_update_4_java = 1;
				  fprintf(fp_nav, "%.4f %.8f %.8f %.3f\n",
							sol_pkt.sol_t_sol_pdr, sol_pkt.sol_pdr[0], sol_pkt.sol_pdr[1], sol_pkt.sol_pdr[2]);
			  }

			  LOGI("accelerometer %d\t%.3f\t%.6f\t%.6f\t%.6f\n",type_da,t_da, da[0], da[1], da[2]);
			  fprintf(fp,"%d\t%.3f\t%.6f\t%.6f\t%.6f\n",type_da,t_da, da[0], da[1], da[2]);
		  }
		  else if(event.type == ASENSOR_TYPE_MAGNETIC_FIELD)
		  {
			  type_da = 3;
			  t_da = event.timestamp/1000000000.0;
			  da[0] = event.magnetic.x;   //uT
			  da[1] = event.magnetic.y;
			  da[2] = event.magnetic.z;

			  if (if_sensor_axis_change)
			  {   // Change axis
				  temp_4_sensor_axis_change = da[0];
				  da[0] = da[1];
				  da[1] = temp_4_sensor_axis_change;
				  da[2] *= -1;
			  }
			  if(if_not_first_mag)
			  {
				  t_da -= t0_mag;
			  }
			  else
			  {
				  if_not_first_mag = 1;
				  t0_mag = t_da;
				  t_da = 0;
			  }


			  meas.avail_mag = 1;
			  meas.t_mag = t_da;
			  meas.da_mag[0] = da[0];
			  meas.da_mag[1] = da[1];
			  meas.da_mag[2] = da[2];
			  meas.avail_gyro = 0;
			  meas.avail_accel = 0;
			  meas.avail_baro = 0;

			  //indoor_navigation_withWiFi_kf(&ini, meas, &sol_pkt);

			  //LOGE("Mag %d\t%.3f\t%.6f\t%.6f\t%.6f\n",type_da,t_da, da[0], da[1], da[2]);
			  //fprintf(fp,"%d\t%.3f\t%.6f\t%.6f\t%.6f\n",type_da,t_da, da[0], da[1], da[2]);
		  }

		  else if(event.type == 13)
		  {       //AMBIENT_TEMPERATURE
			  type_da = 5;
			  t_da = event.timestamp/1000000000.0;
			  if(if_not_first_temper)
			  {
				  t_da -= t0_temper;
			  }
			  else
			  {
				  if_not_first_temper = 1;
				  t0_temper = t_da;
				  t_da = 0;
			  }
			  da[0] = event.temperature;

			  LOGE("Temperature %d\t%.3f\t%.3f\t%d\t%d\n",type_da,t_da,  da[0], 0, 0);
			  fprintf(fp,"%d\t%.3f\t%.3f\t%d\t%d\n",type_da,t_da,da[0], 0, 0);
		  }

		  else if(event.type == 6) {       //AMBIENT_TEMPERATURE
			  type_da = 4;
			  t_da = event.timestamp/1000000000.0;
			  da[0] = event.pressure;

			  if(if_not_first_baro)
			  {
				  t_da -= t0_baro;
			  }
			  else
			  {
				  if_not_first_baro = 1;
				  t0_baro = t_da;
				  t_da = 0;
			  }

			  meas.avail_baro = 1;
			  meas.t_baro = t_da;
			  meas.da_baro = da[0];
			  meas.avail_gyro = 0;
			  meas.avail_accel = 0;
			  meas.avail_mag = 0;

			  indoor_navigation_withWiFi_kf(&ini, meas, &sol_pkt);

			  LOGE("Pressure %d\t%.3f\t%.3f\t%d\t%d\n",type_da,t_da, da[0],0,0);
			  fprintf(fp,"%d\t%.3f\t%.3f\t%d\t%d\n",type_da,t_da, da[0], 0, 0);
		  }
	  }

	  // -------------------------------------------------------------------------------------------
	  else   // if IF_POST_PROCESSING == 1
	  {
	  	  i_line++;
		  if (!feof(fp)){
			  if (fgets(buff_post,sizeof(buff_post),fp))
			  {
			  	  sscanf(buff_post,"%d %lf %lf %lf %lf", da_temp_sensor_type, da_temp_sensor,da_temp_sensor+1,da_temp_sensor+2,da_temp_sensor+3);
				  type_da = da_temp_sensor_type[0];
				  t_da = da_temp_sensor[0];
				  da[0] = da_temp_sensor[1];
				  da[1] = da_temp_sensor[2];
				  da[2] = da_temp_sensor[3];

				  //  ------  WiFi
				  if (meas.t_gyro >= meas.t_accel && meas.t_gyro >= meas.t_mag &&  meas.t_gyro >= meas.t_baro) {
					  t_start_4_wifi = meas.t_gyro - t_initial;
				  }
				  else if (meas.t_accel >= meas.t_gyro && meas.t_accel >= meas.t_mag &&  meas.t_accel >= meas.t_baro) {
					  t_start_4_wifi = meas.t_accel - t_initial;
				  }
				  else if (meas.t_mag >= meas.t_gyro && meas.t_mag >= meas.t_accel &&  meas.t_mag >= meas.t_baro) {
					  t_start_4_wifi = meas.t_mag - t_initial;
				  }
				  else if (meas.t_baro >= meas.t_gyro && meas.t_baro >= meas.t_accel &&  meas.t_baro >= meas.t_mag) {
					  t_start_4_wifi = meas.t_baro - t_initial;
				  }

				  if (i_line == 1) {
					  if (IF_USE_WIFI && !IF_DATABASE)
					  {
						  for (ii=0; ii<da_wifi.N_eph; ii++) {
							  t_wifi = da_wifi.da[ii].t;

							  if (t_wifi > t_start_4_wifi) {
								  i_wp = ii+1;
								  t_wp = t_wifi;
								  break;
							  }
						  }
					  }
				  }
				  while (i_wp < da_wifi.N_eph){
					  if (t_start_4_wifi > t_wp)
					  {
						  i_wp ++;
						  t_wp = da_wifi.da[i_wp-1].t;
						  for (ii=0; ii<meas.N_wifi; ii++) {
							  strcpy(meas.name_ap[ii], da_wifi.da[i_wp-1].name_ap[ii]);
							  meas.rss_ap[ii] = da_wifi.da[i_wp-1].rss_ap[ii];
						  }
					  }
					  else {
						  break;
					  }
				  }
				  meas.avail_wifi = 0;
                  if (IF_USE_WIFI && !IF_DATABASE && fabs(t_start_4_wifi-t_wp)< 1.0/SENSORS_RATE_MOBILE && i_wp<da_wifi.N_eph) {
                      //if (IF_USE_WIFI && !IF_DATABASE && t_start_4_wifi-t_wp< 1.0/SENSORS_RATE_MOBILE && i_wp<da_wifi.N_eph) {
                      meas.avail_wifi = 1;
                      meas.t_wifi = t_wp;

                      meas.N_wifi = da_wifi.da[i_wp-1].N_ap;
                      for (ii=0; ii<meas.N_wifi; ii++) {
                          strcpy(meas.name_ap[ii], da_wifi.da[i_wp-1].name_ap[ii]);
                          meas.rss_ap[ii] = da_wifi.da[i_wp-1].rss_ap[ii];
                      }

					  LOGE("WiFi %.3f Gyro %.3f Accel %.3f\n", meas.t_gyro, meas.t_accel, meas.t_wifi);

                      i_wp ++;
                      t_wp = da_wifi.da[i_wp-1].t;
					  if (meas.avail_wifi == 1)
					  {
						  meas.avail_gyro = 0;
					  	  meas.avail_accel = 0;
						  meas.avail_mag = 0;
						  meas.avail_baro = 0;
						  indoor_navigation_withWiFi_kf(&ini, meas, &sol_pkt);

						  if (sol_pkt.avail_sol_wifi) {
							  if_show_wifi = 1;

//							  if (if_this_is_not_first_wifi == 1)
//							  {
//								 sol_pkt.sol_pdr[0] = sol_pkt.sol_wifi[0];
//								 sol_pkt.sol_pdr[1] = sol_pkt.sol_wifi[1];
//								 sol_pkt.sol_pdr[2] = 0.0;
//								 sol_pkt.sol_t_sol_pdr = sol_pkt.sol_t_sol_wifi;
//								 sol_pkt.avail_sol_pdr = 1;
//								 if_pos_update_4_java = 1;
//							  }
//
//							  if (if_this_is_not_first_wifi == 0)
//							  {
//								 if_this_is_not_first_wifi = 1;
//							  }

							  fprintf(fp_wifi, "%.3f %.8f %.8f\n", sol_pkt.sol_t_sol_wifi, sol_pkt.sol_wifi[0], sol_pkt.sol_wifi[1]);
						  }
						  meas.avail_wifi = 0;
					  }
                  }

				  //  ==== WiFi

				  // ----------------------- Use of GPS Result -----------------------
				  if (IF_USE_GPS_RESULT)
				  {
					  if (meas.t_gyro >= meas.t_accel && meas.t_gyro >= meas.t_mag &&  meas.t_gyro >= meas.t_baro) {
						  t_start_4_gps_result = meas.t_gyro - t_initial;
					  }
					  else if (meas.t_accel >= meas.t_gyro && meas.t_accel >= meas.t_mag &&  meas.t_accel >= meas.t_baro) {
						  t_start_4_gps_result = meas.t_accel - t_initial;
					  }
					  else if (meas.t_mag >= meas.t_gyro && meas.t_mag >= meas.t_accel &&  meas.t_mag >= meas.t_baro) {
						  t_start_4_gps_result = meas.t_mag - t_initial;
					  }
					  else if (meas.t_baro >= meas.t_gyro && meas.t_baro >= meas.t_accel &&  meas.t_baro >= meas.t_mag) {
						  t_start_4_gps_result = meas.t_baro - t_initial;
					  }

					  if (i_line == 1) {
						  if (IF_USE_GPS_RESULT && !IF_DATABASE) {
							  for (ii=0; ii<da_gps_result.N_eph; ii++) {
								  t_gps_result = da_gps_result.t[ii];

								  if (t_gps_result > t_start_4_gps_result) {
									  i_gpsp = ii+1;
									  t_gpsp = t_gps_result;
									  break;
								  }
							  }
						  }
					  }

					  //
					  while (i_gpsp < da_gps_result.N_eph){
						  if (t_start_4_gps_result > t_gpsp)
						  {
							  i_gpsp ++;
							  t_gpsp = da_gps_result.t[i_gpsp-1];
							  meas.lat_gps_result = da_gps_result.lat[i_gpsp-1];
							  meas.lon_gps_result = da_gps_result.lon[i_gpsp-1];
							  meas.height_gps_result = da_gps_result.hei[i_gpsp-1];
							  meas.accuracy_gps_result = da_gps_result.accuracy[i_gpsp-1];
						  }
						  else {
							  break;
						  }
					  }

					  meas.avail_gps_result = 0;
					  if (IF_USE_GPS_RESULT && !IF_DATABASE && fabs(t_start_4_gps_result-t_gpsp)< 1.0/SENSORS_RATE_MOBILE && i_gpsp<da_gps_result.N_eph) {
						  //if (IF_USE_WIFI && !IF_DATABASE && t_start_4_wifi-t_wp< 1.0/SENSORS_RATE_MOBILE && i_wp<da_wifi.N_eph) {
						  meas.avail_gps_result = 1;
						  meas.t_gps_result = t_gpsp;
						  meas.lat_gps_result = da_gps_result.lat[i_gpsp-1];
						  meas.lon_gps_result = da_gps_result.lon[i_gpsp-1];
						  meas.height_gps_result = da_gps_result.hei[i_gpsp-1];
						  meas.accuracy_gps_result = da_gps_result.accuracy[i_gpsp-1];

						  i_gpsp ++;
						  //printf("%d\n", i_wp);
						  t_gpsp = da_gps_result.t[i_gpsp-1];
					  }
				  } // End if IF_USE_GPS_RESULT


				  if (meas.avail_gps_result) {
					  //
					  meas.avail_gyro = 0;
					  meas.avail_accel = 0;
					  meas.avail_mag = 0;
					  meas.avail_baro = 0;

					  indoor_navigation_withWiFi_kf(&ini, meas, &sol_pkt);
					  meas.avail_gps_result = 0;

				  } // End if (meas.avail_wifi)

				  // ======================= Use of GPS Result =======================

				  if(type_da == 1)
				  {
					  if(if_not_first_gyro)
					  {
						  t_da -= t0_gyro;
					  }
					  else
					  {
						  if_not_first_gyro = 1;
						  t0_gyro = t_da;
						  t_da = 0;
					  }

					  meas.avail_gyro = 1;
					  meas.t_gyro = t_da;
					  meas.da_gyro[0] = da[0];
					  meas.da_gyro[1] = da[1];
					  meas.da_gyro[2] = da[2];
					  meas.avail_accel = 0;
					  meas.avail_mag = 0;
					  meas.avail_baro = 0;

					  indoor_navigation_withWiFi_kf(&ini, meas, &sol_pkt);
					  if (sol_pkt.avail_sol_pdr)
					  {
						  if_pos_update_4_java = 1;
						  fprintf(fp_nav, "%.4f %.8f %.8f %.3f\n",
									sol_pkt.sol_t_sol_pdr, sol_pkt.sol_pdr[0], sol_pkt.sol_pdr[1], sol_pkt.sol_pdr[2]);
					  }
					  //LOGD("Gyro %d\t%.3f\t%.6f\t%.6f\t%.6f\n", type_da,t_da, da[0], da[1], da[2]);
				  }
				  else if(type_da == 2)
				  {
					  if(if_not_first_accel)
					  {
						  t_da -= t0_accel;
					  }
					  else
					  {
						  if_not_first_accel = 1;
						  t0_accel = t_da;
						  t_da = 0;
					  }
					  meas.avail_accel = 1;
					  meas.t_accel = t_da;
					  meas.da_accel[0] = da[0];
					  meas.da_accel[1] = da[1];
					  meas.da_accel[2] = da[2];
					  meas.avail_gyro = 0;
					  meas.avail_mag = 0;
					  meas.avail_baro = 0;

					  indoor_navigation_withWiFi_kf(&ini, meas, &sol_pkt);

					  if (sol_pkt.avail_sol_pdr){
						  if_pos_update_4_java = 1;
						  fprintf(fp_nav, "%.4f %.8f %.8f %.3f\n",
									sol_pkt.sol_t_sol_pdr, sol_pkt.sol_pdr[0], sol_pkt.sol_pdr[1], sol_pkt.sol_pdr[2]);
					  }
					  //LOGI("accelerometer %d\t%.3f\t%.6f\t%.6f\t%.6f\n",type_da,t_da, da[0], da[1], da[2]);
				  }
				  else if(type_da == 3)
				  {
					  if(if_not_first_mag)
					  {
						  t_da -= t0_mag;
					  }
					  else
					  {
						  if_not_first_mag = 1;
						  t0_mag = t_da;
						  t_da = 0;
					  }
					  meas.avail_mag = 1;
					  meas.t_mag = t_da;
					  meas.da_mag[0] = da[0];
					  meas.da_mag[1] = da[1];
					  meas.da_mag[2] = da[2];
					  meas.avail_gyro = 0;
					  meas.avail_accel = 0;
					  meas.avail_baro = 0;
					  indoor_navigation_withWiFi_kf(&ini, meas, &sol_pkt);
					  //LOGE("Mag %d\t%.3f\t%.6f\t%.6f\t%.6f\n",type_da,t_da, da[0], da[1], da[2]);
				  }

				  else if(type_da == 5)
				  {       //AMBIENT_TEMPERATURE
					  if(if_not_first_temper)
					  {
						  t_da -= t0_temper;
					  }
					  else
					  {
						  if_not_first_temper = 1;
						  t0_temper = t_da;
						  t_da = 0;
					  }
					  type_da = 5;
					  //LOGE("Temperature %d\t%.3f\t%.3f\t%d\t%d\n",type_da,t_da,  da[0], 0, 0);
				  }

				  else if(type_da == 4) {       //AMBIENT_TEMPERATURE
					  if(if_not_first_baro)
					  {
						  t_da -= t0_baro;
					  }
					  else
					  {
						  if_not_first_baro = 1;
						  t0_baro = t_da;
						  t_da = 0;
					  }
					  meas.avail_baro = 1;
					  meas.t_baro = t_da;
					  meas.da_baro = da[0];
					  meas.avail_gyro = 0;
					  meas.avail_accel = 0;
					  meas.avail_mag = 0;

					  indoor_navigation_withWiFi_kf(&ini, meas, &sol_pkt);
					  //LOGE("Pressure %d\t%.3f\t%.3f\t%d\t%d\n",type_da,t_da, da[0],0,0);
				  }
			  }
		  }
	  }
  }
  //should return 1 to continue receiving callbacks, or 0 to unregister
  return 1;
}

jstring
Java_com_example_youli_helloindoor_MainActivity_writeToFile( JNIEnv* env,
		jobject thiz ){
	char * writeFileDir = "/sdcard/SenDa/";
	if(0 == access(writeFileDir,0)) {

	} else{
		mkdir(writeFileDir,777);
	}

	return (*env)->NewStringUTF(env, "Hello File write");
}
void
Java_com_example_youli_helloindoor_MainActivity_disableSensor( JNIEnv* env,
		jobject thiz ){
	if (if_has_accel == 1){
		ASensorEventQueue_disableSensor(sensorEventQueue, accSensor);
	}
	if (if_has_gyro == 1){
		ASensorEventQueue_disableSensor(sensorEventQueue, gyroSensor);
	}
	if (if_has_mag == 1){
		ASensorEventQueue_disableSensor(sensorEventQueue, magSensor);
	}
	if (if_has_temper == 1){
		ASensorEventQueue_disableSensor(sensorEventQueue, temperSensor);
	}
	if (if_has_baro == 1){
		ASensorEventQueue_disableSensor(sensorEventQueue, baroSensor);
	}
	fclose(fp);
	if (IF_POST_PROCESSING == 0)
	{
		fclose(fp_wifi_raw);
		fclose(fp_gps_raw);
	}
    fclose(fp_nav);
    fclose(fp_wifi);
}

jstring
Java_com_example_youli_helloindoor_MainActivity_naGetHelloNDKStr(JNIEnv* pEnv, jobject pObj)
{
    //LOGI("11111111111111111111111111111\n");
    //LOGD("22222222222222222222222222222\n");
    //LOGE("33333333333333333333333333333\n");

    return (*pEnv)->NewStringUTF(pEnv, "Hello You NDK!");
}


//int Java_com_example_youli_helloindoor_MainActivity_pass_int_N_wifi(JNIEnv *env, jobject object, jint nn) {

//	nn_wifi = nn;
//	// meas.N_wifi = nn;
//	 return 0;
//}




//
//void Java_com_example_youli_helloindoor_MainActivity_pass_char_mac(JNIEnv *env, jobject object, jobjectArray stringArray) {
//    int stringCount = (*env)->GetArrayLength(env,stringArray);
//    int i;
//    for (i=0; i<stringCount; i++) {
//        jstring string = (jstring) (*env)->GetObjectArrayElement(env, stringArray, i);
//        const char *rawString = (*env)->GetStringUTFChars(env, string, 0);
//        // Don't forget to call `ReleaseStringUTFChars` when you're done.
//
//
//        if (i==0)
//        	strcpy(str_path, rawString);
//        else if (i == 1) {
//        	strcpy(name_path, rawString);
//        	strcpy(f_DB_bin_str,name_path);
//        	strcat(f_DB_bin_str, "wifi_DB_matrix_0_5_2_2_100_0_0.bin");
//        }
//
//
//        (*env)->ReleaseStringUTFChars(env,string,rawString);
//    }
//}

// =============================== ANDROID ====================================
// ============================================================================





// ----------------------------------------------------------------------------
// ------------------------------- C SOURCE -----------------------------------

void indoor_navigation_withWiFi_kf(Initialization_Pkt* ini, Meas meas, Sol_Pkt*  sol)
{
    static UINT8 fitst_time_flag = 0;
    static NavIndoor nav;
    static KF_4 kf_4;
    static MemoIndoor memo;


    static UINT8 if_heading_initialized = 0;
    static UINT8 if_position_initialized = 0;
    static UINT8 if_nav_initialized = 0;
    static UINT8 if_has_wifi_result_for_initialization = 0;
    static UINT8 if_has_changed_Qk = 0;

    //static FLOAT32 gyro_bias[3];
    //static FLOAT32 accel_bias[3];
    int i, j, k, i_mid_in_memo, i_heading_in_memo;
    double min_t_heading_in_memo[2];
    float max_min_qs_detect[2], diff_max_min_gz;
    float ma_mid1, min_t1;
    float max_va, min_va, t_max_va, t_min_va_4_step;
    int i_eph_in_memo_min_va;
    float heading;
    double t_mid1, lat1;
    static double ts_gyro = 0.0;
    double t_diff_g_a[LENGTH_MEMO_SENSOR_DATA];

    int if_detect_step, if_update_step, if_qs;
    int if_update_pos = 0;

    static double t0_gyro = 0.00;
    static double t_pre_gyro = 0.00;
    static int if_first_t_gyro = 0;

    static double t_pre_accel = 0.00;
    static double t0_accel = 0.00;
    static int if_first_t_accel = 0;

    static double t_pre_mag = 0.00;
    static double t0_mag = 0.00;
    static int if_first_t_mag = 0;
    static int if_first_t_mag2 = 0;
    static int i_count_mag_data = 0;

    static double t_pre_baro = 0.00;
    static double t0_baro = 0.00;
    static int if_first_t_baro = 0;
    static int if_first_t_baro2 = 0;
    static int i_count_baro_data = 0;

    //WiFi
    int availability_wifi[1] = {0};
    int num_floor[1] = {1};		// Temperory
    static double pos_wifi[3] = {0.00};   // The height should be set here because it is not dealed with in the wifi function


	// GPS result
    double pos_gps_result[3] = {0.00};
    float accuracy_gps_result = 0.0;
    static int if_gps_can_defeat_wifi;
    static double t_gps_defeat_start;
    int if_during_defeat_wifi_time;

    int if_gps_accu_can_cali_hei;
    static int eph_gps_calibrate_height;

    // MM result
    double pos_mm_result[3] = {0.00};


    //Map
    //static float da_map_NE[MAX_FLOOR_LINE_NUM][8];
    static float da_map_NE[N_ROW_DB_FP][N_COL_DB_FP];
    static int nLine_link[1];
    UINT8 if_align_heading;
    //FLOAT32 ang_align;
    int i_link_align_heading = 0;

    float WSH[MAX_FLOOR_LINE_NUM], WSPD[MAX_FLOOR_LINE_NUM], TWS[MAX_FLOOR_LINE_NUM];
    float rprp_NE[MAX_FLOOR_LINE_NUM][2], DD[MAX_FLOOR_LINE_NUM]; //,  diff_heading_step_heading;
    float r1_NE[2], r2_NE[2], rs_NE[2], rp_NE[2], delta_beta1, max_tws[2], rp_Link[2], dist_4_rej_link_pos;
    float d_North, d_East;

    // KF
    FLOAT32 PHI[N_STATES][N_STATES], PHIT[N_STATES][N_STATES], PHIP[N_STATES][N_STATES], PHIPPHIT[N_STATES][N_STATES];
    FLOAT32 Hk_2[2][N_STATES],Rk_2[2][2], Zk_2[2],inno_2[2], Hk_2xk[3];
    FLOAT32 xk[N_STATES] = {0.0};
    FLOAT32* pPHI[N_STATES], *pPHIT[N_STATES], *pPHIP[N_STATES], *pPHIPPHIT[N_STATES];
    FLOAT32* pHk_2[2], *pRk_2[2];
    static float pos_NEH[3], pos_NEH_meas[3];
    int if_kf_updated = 0;

    // -----------------  Attitude and Misalignment ----------------------
    static Att_Pkt att_pkt;
    static Misalign_pkt misa;

    DOUBLE64 t_cur;
    FLOAT32 f_b[3], w_b[3], m_b[3], fb_sm[3], w_b_zero[3];
    float acc_temp;
    UINT8 flag_accel, flag_mag;
    UINT8 flag_ini_heading;
    FLOAT32 ini_heading;  //Deg
    UINT8 seed_availability;
    float seed_bias[3];

    FLOAT32 f_leveled[3];
    float f_n[3], norm_a_n, m_n[3];
    FLOAT32 gyro_norm, rad_rotation;
    FLOAT32 R_ls[3][3];

    FLOAT32 memo_acc_norm[N_MEM_misO_1_STEP_MIS];    // Put into memo later
    UINT8 i_memo_acc_norm = 0;

    float mean_acc_norm;
    float da_Acc_Variance_Data;
    INT8 height_change_Data;	 // => Height changing       2 ===> Stairs Up    3 ===> Stairs Down

    FLOAT32 misalignment_uesd = 0.0;
    // ===============  End Attitude and Misalignment ====================



    // ----------------------------- Barometer --------------------------------
    static UINT8 if_has_baro = 0;
    static FLOAT32 t_pre_stair = -99.0;

    UINT8 if_has_baro_in_step;
    static UINT8 this_is_step = 0;

    static int if_first_floor_3_KCCI = 0;
	static int if_first_floor_2_KCCI = 0;
	static int if_first_floor_1_KCCI = 0;
	static int if_first_floor_1_PDC = 0;
	static int if_first_floor_2_PDC = 0;
	static int if_first_floor_3_PDC = 0;


	static int if_this_is_first_GPS = 0;

	static int if_not_in_KCCI = 0;

	static int if_this_is_first_gps_height = 0;

	static double t_start_PDC = 0.0;
	static double t_end_PDC_1 = 0.0;
	static double t_start_PDC2 = 0.0;
    static double t_end_PDC2_1 = 0.0;

    static int if_first_start_outdoor_gps = 0;
    static double t_start_outdoor_gps = 0.0;


    static int if_start_use_wifi = 0;
    double TH_START_USE_WIFI = 30.0;
    // ============================= Barometer ================================


    // ------ For debug only --------
    //static int i_step_debug;
    // ====== For debug only ========


    // --------------------------------------------------------------------
    // Set indicators in sol to "0"
    sol->avail_sol_pdr = 0;
    sol->avail_step = 0;
    sol->avail_heading = 0;
    sol->avail_heading_step = 0;
    sol->avail_sol_wifi = 0;
    sol->avail_att_mis = 0;
    sol->avail_has_baro_in_step = 0;
    sol->avail_heading_mag = 0;
    sol->avail_show_qs = 0;
    sol->avail_sol_mm_result_used = 0;
    sol->avail_sol_gps_result_used = 0;

    // ----------------------------------- Initialization --------------------------------------------
    if (!fitst_time_flag){
        fitst_time_flag = 1;

		sol->floor_num = 3;
        // for the first wifi data
        /*meas.t_gyro = 0.00;
         meas.t_accel = 0.00;
         meas.t_mag = 0.00;
         meas.t_baro = 0.00;*/

        //
        //gyro_bias[0] = ini_bg[0];
        //gyro_bias[1] = ini_bg[1];
        //gyro_bias[2] = ini_bg[2];   // Later we can save and read gBias from a file

        if_gps_can_defeat_wifi = 0;
		t_gps_defeat_start = 0.00;
		eph_gps_calibrate_height = 0;



        att_pkt.bg_estimated[0] = ini_gyro_bias_att[0];
        att_pkt.bg_estimated[1] = ini_gyro_bias_att[1];
        att_pkt.bg_estimated[2] = ini_gyro_bias_att[2];

        att_pkt.ba_estimated[0] = ini_accel_bias_att[0];
        att_pkt.ba_estimated[1] = ini_accel_bias_att[1];
        att_pkt.ba_estimated[2] = ini_accel_bias_att[2];   // Later we can save and read gBias from a file

        //P, Qk
        for (i=0; i<N_STATES; i++) {
            for (j=0; j<N_STATES; j++) {
                kf_4.P[i][j] = 0.0;
                kf_4.Qk[i][j] = 0.0;
            }
        }
        if (N_STATES == 4)
        {
            kf_4.P[0][0] = INI_POS_VAR * INI_POS_VAR;
            kf_4.P[1][1] = INI_POS_VAR * INI_POS_VAR;
            kf_4.P[2][2] = INI_HEAD_VAR * INI_HEAD_VAR;
            kf_4.P[3][3] = INI_SL_VAR * INI_SL_VAR;

            kf_4.Qk[0][0] = QK_POS_PHASE_1 * QK_POS_PHASE_1;
            kf_4.Qk[1][1] = QK_POS_PHASE_1 * QK_POS_PHASE_1;
            kf_4.Qk[2][2] = QK_HEAD_PHASE_1 * QK_HEAD_PHASE_1;
            kf_4.Qk[3][3] = QK_SL * QK_SL;
        }
        else if (N_STATES == 3)
        {
            kf_4.P[0][0] = INI_POS_VAR * INI_POS_VAR;
            kf_4.P[1][1] = INI_POS_VAR * INI_POS_VAR;
            kf_4.P[2][2] = INI_HEAD_VAR * INI_HEAD_VAR;

            kf_4.Qk[0][0] = QK_POS_PHASE_1 * QK_POS_PHASE_1;
            kf_4.Qk[1][1] = QK_POS_PHASE_1 * QK_POS_PHASE_1;
            kf_4.Qk[2][2] = QK_HEAD_PHASE_1 * QK_HEAD_PHASE_1;
        }
        ChangeArrayMemoryLayout_float(N_STATES,N_STATES,kf_4.pP,(FLOAT32 *)kf_4.P,0);         //P
        ChangeArrayMemoryLayout_float(N_STATES,N_STATES,kf_4.pQk,(FLOAT32 *)kf_4.Qk,0);       //Qk

        // misa
        misa.Misalign_Az = 0.0;
        if (MODE_MISALIGNMENT == 1)
        {
            misa.Misalign_Az = FIXED_MISALIGNEMNT;
        }

        // -- Initialization PDR
        nav.heading_platform = 0.0;

        //if (ini->type_initializaiton == 1)
        if(TYPE_INITIALIZATION != 100)
        {
            nav.pos[0] = INI_POS[0];
            nav.pos[1] = INI_POS[1];
            nav.pos[2] = INI_POS[2];

            //nav.heading_platform = INI_HEADING;
            nav.heading_platform = INI_HEADING;    // This is platform heading
        }


        nav.heading_step = nav.heading_platform;
        nav.t_step1 = 0;
        nav.sl = SL;

        nav.floor_num = INI_FLOOR_NUM;

        //-- End Initialization PDR

        // ---- INITIALIZAION_MEMO
        memo.i_accel = 0;
        memo.i_gyro = 0;
        memo.i_mag = 0;
        memo.i_baro = 0;

        memo.i_ma = 0;
        memo.i_t_step = 0;
        memo.i_pos_pdr = 0;
        memo.i_va = 0;
        memo.i_memo_heading_step = 0;
        memo.i_misa = 0;

        memo.i_memo_baro = 0;
        memo.i_memo_baro_sm = 0;
        memo.i_memo_baro_change = 0;
        memo.i_memo_if_stair = 0;
        memo.i_memo_status_stair = 0;
        memo.i_memo_floor_num = 0;

        memo.i_memo_norm_gyro_QS = 0;
        //memo.i_memo_if_vertical_gyro = 0;

        for(i=0; i<LENGTH_MEMO_SENSOR_DATA; i++){
            memo.t_accel[i] = 0.00;
            memo.a_x[i] = 0.0;
            memo.a_y[i] = 0.0;
            memo.a_z[i] = 0.0;
            memo.t_gyro[i] = 0.00;
            memo.g_x[i] = 0.0;
            memo.g_y[i] = 0.0;
            memo.g_z[i] = 0.0;

            memo.gz_level[i] = 0.0;
            memo.heading_platform[i] = 0.0;
        }

        // baro
        memo.height_pre = 1000.0;             // Needs to change
        for (i=0; i<N_EPH_SMOOTH_BARO; i++)
        {
            memo.t_baro[i] = 0.0;
            memo.baro[i] = 0.0;
        }
        for (i=0; i<N_EPH_BARO_DIFF; i++)
        {
            memo.baro_sm[i] = 0;
        }
        for (i=0; i<N_MEMO_BARO_CHANGE; i++)
        {
            memo.baro_change[i] = 0;
        }
        for (i=0; i<N_MEMO_IF_STAIR; i++)
        {
            memo.if_stair[i] = 0;
        }
        for (i=0; i<N_MEMO_STATUS_STAIR; i++)
        {
            memo.status_stair[i] = 0;
        }
        for (i=0; i<N_MEMO_FLOOR_NUM; i++)
        {
            memo.floor_num[i] = 1;
        }

        // Mag
        nav.declination = INI_DECLINATION;
        for (i=0; i< N_EPH_MAG_DATA; i++)
        {
            memo.t_mag[i] = 0.0;
            memo.m_x[i] = 0.0;
            memo.m_y[i] = 0.0;
            memo.m_z[i] = 0.0;
        }
        for (i=0; i<N_EPH_MEMO_NORM; i++)
        {
            memo.normm[i] = 0.0;
        }
        for (i=0; i<N_EPH_MEMO_IND_QSMF; i++)
        {
            memo.ind_qsmf[i] = 0;
        }
        for (i=0; i<N_EPH_HEADING_MAG; i++)
        {
            memo.heading_mag[i] = 0.0;
        }


        //
        for(i=0; i<LENGTH_MEMO_MA; i++){
            memo.ma[i] = 0.0;
        }
        for(i=0; i<LENGTH_MEMO_T_STEP; i++) {
            memo.t_step[i] = 0.00;
        }
        for(i=0; i<LENGTH_MEMO_POS_PDR; i++) {
            memo.t_pos_pdr[i] = 0.00;
            memo.pos_pdr_lat[i] = 0.00;
            memo.pos_pdr_lon[i] = 0.00;
            memo.pos_pdr_hei[i] = 0.00;
            memo.heading_step[i] = 0.0;
        }
        for(i=0; i<N_MEMO_VA; i++)
        {
            memo.va[i] = 0.0;
            memo.t_va[i] = 0.00;
        }
        for (i=0; i<N_SM_MISALIGN; i++)
        {
            memo.misalign[i] = 0.0;
        }



        // ---- End INITIALIZAION_MEMO


        /*memo.i_pos_pdr ++;
         memo.pos_pdr_lat[memo.i_pos_pdr-1] = nav.pos[0];
         memo.pos_pdr_lon[memo.i_pos_pdr-1] = nav.pos[1];
         memo.pos_pdr_hei[memo.i_pos_pdr-1] = nav.pos[2];


         sol->avail_sol_pdr = 1;
         sol->sol_t_sol_pdr = memo.t_pos_pdr[memo.i_pos_pdr-1];
         sol->sol_pdr[0] = memo.pos_pdr_lat[memo.i_pos_pdr-1]/D2R;
         sol->sol_pdr[1] = memo.pos_pdr_lon[memo.i_pos_pdr-1]/D2R;
         sol->sol_pdr[2] = memo.pos_pdr_hei[memo.i_pos_pdr-1];*/

        //fprintf(sol->f_out_rec_pos, "%.4f %.8f %.8f %.3f\n",
        //		memo.t_pos_pdr[memo.i_pos_pdr-1], memo.pos_pdr_lat[memo.i_pos_pdr-1]/D2R,
        //		memo.pos_pdr_lon[memo.i_pos_pdr-1]/D2R, memo.pos_pdr_hei[memo.i_pos_pdr-1]);


        // --------------------new for map
        if (IF_ALIGN_TO_LINKS || IF_PROJECT_TO_LINK)
        {
            //load_DB_fp_txt(f_DB_fp_txt, da_map_NE, nLine_link);
            load_DB_fp_string(da_map_NE, nLine_link);
        }
        // ---------------------
    }  // End (!fitst_time_flag)

    pos_NEH[0] = (nav.pos[0]-ZERO_POS[0])*RM;
    pos_NEH[1] = (nav.pos[1]-ZERO_POS[1])*RN_COS_LON;
    // ===================================  End of Initialization ====================================



    // -------------------- 2015-04-03 - First should be Accel ---------------------
    if (memo.i_accel == 0)
    {
        if (meas.avail_gyro == 1)
        {
            meas.avail_gyro = 0;
        }
        if (meas.avail_mag == 1)
        {
            meas.avail_mag = 0;
        }
        if (meas.avail_baro == 1)
        {
            meas.avail_baro = 0;
        }
    }
    // ==================== 2015-04-03 - First should be Accel =====================



    // Timestamp
    if (meas.avail_gyro)
    {
        if (if_first_t_gyro) {
            meas.t_gyro -= t0_gyro;  // t0_gyro is used for time_system for gyro
            if (fabs(meas.t_gyro-t_pre_gyro) < TH_REJECT_GYRO_OUTPUT) {  // t_pre_gyro is used for gyro time in the previous epoch
                meas.avail_gyro = 0;
            }
            else {
                t_pre_gyro = meas.t_gyro;
            }

        }
        else {
            if (!memo.i_accel) {			// The first one should be ACCEL!
                meas.avail_gyro = 0;
            }
            else {
                t0_gyro = meas.t_gyro;
                meas.t_gyro = 0;
                if_first_t_gyro = 1;

                t_pre_gyro = meas.t_gyro;
            }
        }
        //t_pre_gyro = meas.t_gyro // up
    }

    if (meas.avail_accel) {
        if (if_first_t_accel) {
            meas.t_accel -= t0_accel;
            if (fabs(meas.t_accel-t_pre_accel) < TH_REJECT_ACCEL_OUTPUT) {
                meas.avail_accel = 0;
            }
            else {
                t_pre_accel = meas.t_accel;
            }
        }
        else {
            t0_accel = meas.t_accel;
            meas.t_accel = 0;
            if_first_t_accel = 1;

            t_pre_accel = meas.t_accel;
        }
        //t_pre_accel = meas.t_accel;
    }

    if (meas.avail_mag)
    {
        i_count_mag_data ++;
        meas.avail_mag = 0;

        if (!if_first_t_mag)
        {
            if (!memo.i_accel) {
                meas.avail_mag = 0;
            }
            else {
                if_first_t_mag = 1;
                t0_mag = meas.t_mag;

                t_pre_mag = meas.t_mag;
            }
        }

        if (i_count_mag_data > I_START_MAG_DATA)
        {
            i_count_mag_data = I_START_MAG_DATA;

            meas.t_mag -= t0_mag;
            meas.avail_mag = 1;
            if (fabs(meas.t_mag-t_pre_mag) < TH_REJECT_MAG_OUTPUT) {  //
                meas.avail_mag = 0;
            }
            if (!if_first_t_mag2)
            {
                if_first_t_mag2 = 1;
                // First mag <====================================================
                t_pre_mag = meas.t_mag;
            }
            //t_pre_mag = meas.t_mag;
        }
    }

    if (meas.avail_baro)
    {
        i_count_baro_data ++;
        meas.da_baro = cal_height_baro(meas.da_baro);   // pressure -> height

        meas.avail_baro = 0;

        if (!if_first_t_baro)
        {
            if (!memo.i_accel) {
                meas.avail_baro = 0;
            }
            else {
                if_first_t_baro = 1;
                t0_baro = meas.t_baro;
            }
        }

        if (i_count_baro_data > I_START_BARO_DATA)
        {
            i_count_baro_data = I_START_BARO_DATA;

            meas.t_baro -= t0_baro;
            meas.avail_baro = 1;
            if (fabs(meas.t_baro-t_pre_baro) < TH_REJECT_BARO_OUTPUT) {  //
                meas.avail_baro = 0;
            }
            if (!if_first_t_baro2)
            {
                if_first_t_baro2 = 1;
                memo.height_baro0 = meas.da_baro;   // height_baro_0

                t_pre_baro = meas.t_baro;
            }
            //t_pre_baro = meas.t_baro;
        }
        nav.height_baro = meas.da_baro;
    }



    //// ------------------ IF CUT IMU -----------------------
    //if(IF_CUT_IMU) {
    //	if (meas.avail_gyro && meas.t_gyro < T_START_CIT_IMU) {
    //		meas.avail_gyro = 0;
    //	}
    //	if (meas.avail_accel && meas.t_accel < T_START_CIT_IMU) {
    //		meas.avail_accel = 0;
    //	}
    //	if (meas.avail_mag && meas.t_mag < T_START_CIT_IMU) {
    //		meas.avail_mag = 0;
    //	}
    //	if (meas.avail_baro && meas.t_baro < T_START_CIT_IMU) {
    //		meas.avail_baro = 0;
    //	}
    //}

    //// ================== IF CUT IMU =======================



    //pos_NEH[2] = nav.pos[2];
    // Start
    if (meas.avail_gyro)		// Put into attitude algorithm later
    {
        meas.da_gyro[0] = meas.da_gyro[0] * D2R ;  //gyro_bias[0];
        meas.da_gyro[1] = meas.da_gyro[1] * D2R ; //- gyro_bias[1];
        meas.da_gyro[2] = meas.da_gyro[2] * D2R ; //gyro_bias[2];

        if (!att_pkt.if_vertical)
        {
            w_b[0] = meas.da_gyro[0];
            w_b[1] = meas.da_gyro[1];
            w_b[2] = meas.da_gyro[2];
        }
        else if (att_pkt.if_vertical == 1)   // Vertical Axis
        {
            w_b[0] =  meas.da_gyro[2];
            w_b[1] =  meas.da_gyro[1];
            w_b[2] =  -meas.da_gyro[0];
        }
        else if (att_pkt.if_vertical == -1)
        {
            w_b[0] =  -meas.da_gyro[2];
            w_b[1] =  meas.da_gyro[1];
            w_b[2] =  meas.da_gyro[0];
        }

        compensate_b_att(w_b, att_pkt.bg_estimated);

        meas.da_gyro[0] = w_b[0];
        meas.da_gyro[1] = w_b[1];
        meas.da_gyro[2] = w_b[2];
    }

    if (meas.avail_accel) {
        if (!att_pkt.if_vertical)
        {
            f_b[0] = meas.da_accel[0];
            f_b[1] = meas.da_accel[1];
            f_b[2] = meas.da_accel[2];
        }
        else if (att_pkt.if_vertical == 1)   // Vertical Axis
        {
            f_b[0] =  meas.da_accel[2];
            f_b[1] =  meas.da_accel[1];
            f_b[2] =  -meas.da_accel[0];
        }
        else if (att_pkt.if_vertical == -1)
        {
            f_b[0] =  -meas.da_accel[2];
            f_b[1] =  meas.da_accel[1];
            f_b[2] =  meas.da_accel[0];
        }

        compensate_b_att(f_b, att_pkt.ba_estimated);

        meas.da_accel[0] = f_b[0];
        meas.da_accel[1] = f_b[1];
        meas.da_accel[2] = f_b[2];
    }

    if (meas.avail_mag) {
        if (!att_pkt.if_vertical)
        {
            m_b[0] = meas.da_mag[0];
            m_b[1] = meas.da_mag[1];
            m_b[2] = meas.da_mag[2];
        }
        else if (att_pkt.if_vertical == 1)   // Vertical Axis
        {
            m_b[0] =  meas.da_mag[2];
            m_b[1] =  meas.da_mag[1];
            m_b[2] =  -meas.da_mag[0];
        }
        else if (att_pkt.if_vertical == -1)
        {
            m_b[0] =  -meas.da_mag[2];
            m_b[1] =  meas.da_mag[1];
            m_b[2] =  meas.da_mag[0];
        }

        //compensate_b_att(m_b, att_pkt.ba_estimated);

        meas.da_mag[0] = m_b[0];
        meas.da_mag[1] = m_b[1];
        meas.da_mag[2] = m_b[2];
    }

    // ----------------------------------- Put into memory --------------------------------------------
    real_time_put_into_memo(&memo, meas, &att_pkt, &nav);

    // =================================== Put into memory ============================================




    // ----------------------------------   new for mag  ---------------------------------------------
    if (meas.avail_mag)
    {
        //Euler2Dcm_att_float_float_3_3_mis(att_pkt.att[0], att_pkt.att[1], 0, R_ls);
        //for(j = 0; j < 3; j++)
        //{
        //	m_n[j] = 0.0;
        //	for(k = 0; k < 3; k++)	// bringing in the accel into the level plane
        //	{
        //		m_n[j] += R_ls[j][k]*meas.da_mag[k];
        //	}
        //}

        //nav.declination = nav.declination;
        //nav.heading_mag = -atan2(m_n[1], m_n[0]) + nav.declination;   //+


        sol->avail_heading_mag = 1;
        sol->sol_t_heading_mag = meas.t_mag;
        sol->m_b[0] = meas.da_mag[0];
        sol->m_b[1] = meas.da_mag[1];
        sol->m_b[2] = meas.da_mag[2];
        sol->heading_mag = memo.heading_mag[memo.i_memo_heading_mag-1] / D2R; //nav.heading_mag / D2R;
        sol->declination = nav.declination / D2R;
        sol->indi_qsmf = memo.ind_qsmf[memo.i_memo_indi_qsmf-1];

    }
    // ==================================   new for mag  =============================================




    // ----------------------------------   new for baro  ---------------------------------------------
    if(meas.avail_baro && this_is_step)
    {
        this_is_step = 0;

        if_has_baro_in_step = 0;
        if (memo.i_t_step > 0 && fabs(memo.t_step[memo.i_t_step-1]-meas.t_baro)<TH_DIFF_BARO_STEP)
        {
            if_has_baro_in_step = 1;
        }

        if (if_has_baro_in_step)
        {
            memo.i_memo_baro ++;		  // -------------------------------Barometer LEVEL 1
            if (memo.i_memo_baro <= N_EPH_SMOOTH_BARO )
            {
                memo.baro[memo.i_memo_baro-1] = nav.height_baro;
            }
            else
            {
                memo.i_memo_baro = N_EPH_SMOOTH_BARO;
                for (k=0; k<N_EPH_SMOOTH_BARO-1; k++)
                {
                    memo.baro[k] = memo.baro[k+1];
                }
                memo.baro[N_EPH_SMOOTH_BARO-1] = nav.height_baro;
            }

            nav.height_baro_sm = cal_mean_float(memo.baro, memo.i_memo_baro);

            memo.i_memo_baro_sm ++;		   // -------------------------------Barometer LEVEL 2
            if (memo.i_memo_baro_sm <= N_EPH_BARO_DIFF)
            {
                memo.baro_sm[memo.i_memo_baro_sm-1] = nav.height_baro_sm;
            }
            else
            {
                memo.i_memo_baro_sm = N_EPH_BARO_DIFF;
                for (k=0; k<N_EPH_BARO_DIFF-1; k++)
                {
                    memo.baro_sm[k] = memo.baro_sm[k+1];
                }
                memo.baro_sm[N_EPH_BARO_DIFF-1] = nav.height_baro_sm;
            }

            nav.height_change = 0;
            if (memo.i_memo_baro_sm == N_EPH_BARO_DIFF)		// -------------------------------Barometer LEVEL 3
            {
                nav.height_change = memo.baro_sm[N_EPH_BARO_DIFF-1] - memo.baro_sm[0];

                memo.i_memo_baro_change++;
                if (memo.i_memo_baro_change <= N_MEMO_BARO_CHANGE)
                {
                    memo.baro_change[memo.i_memo_baro_change-1] = nav.height_change;
                }
                else
                {
                    memo.i_memo_baro_change = N_MEMO_BARO_CHANGE;
                    for (k=0; k<N_MEMO_BARO_CHANGE-1; k++)
                    {
                        memo.baro_change[k] = memo.baro_change[k+1];
                    }
                    memo.baro_change[N_MEMO_BARO_CHANGE-1] = nav.height_change;
                }
            }


//            // ----------------- NEW FOR COMPETITION -------------------------
//			if (fabs(nav.height_change) < TH_FLAT_FLOOR_CHANGE_HEIGHT0)
//			{
//				memo.height_baro0 += nav.height_change;
//				printf("%.2fs, height0 = %.2f, height_change = %.2f, height = %.2f\n", meas.t_baro, memo.height_baro0,nav.height_change, meas.da_baro);
//			}
//
//
//			// ------------------------- 4 DBs ------------------------- 20151007
//			float def_height_2_initial = sol->height_baro_sm - memo.height_baro0;
//
//
//			int HEIGHT_1_FLOOR = 3;     // ################################
//
//
//			if (def_height_2_initial > HEIGHT_1_FLOOR *3)
//				num_floor[0] = 4;
//			else if (def_height_2_initial > HEIGHT_1_FLOOR *2)
//				num_floor[0] = 3;
//			else if (def_height_2_initial > HEIGHT_1_FLOOR)
//				num_floor[0] = 2;
//			else
//				num_floor[0] = 1;
//
//			sol->floor_num = num_floor[0];
//			// ========================= 4 DBs =========================
//
//			// ================= NEW FOR COMPETITION =========================

            nav.flag_stair = 0;
            if (memo.i_memo_baro_change == N_MEMO_BARO_CHANGE )
            {
                if (memo.baro_change[N_MEMO_BARO_CHANGE-1] > TH_DETECT_STAIR &&
                    memo.baro_change[N_MEMO_BARO_CHANGE-2] > TH_DETECT_STAIR &&
                    memo.baro_change[N_MEMO_BARO_CHANGE-3] > TH_DETECT_STAIR &&
                    memo.baro_change[N_MEMO_BARO_CHANGE-4] > TH_DETECT_STAIR )
                {
                    nav.flag_stair = 1;
                }
                else if (memo.baro_change[N_MEMO_BARO_CHANGE-1] < -TH_DETECT_STAIR &&
                         memo.baro_change[N_MEMO_BARO_CHANGE-2] < -TH_DETECT_STAIR &&
                         memo.baro_change[N_MEMO_BARO_CHANGE-3] < -TH_DETECT_STAIR &&
                         memo.baro_change[N_MEMO_BARO_CHANGE-4] < -TH_DETECT_STAIR )
                {
                    nav.flag_stair = -1;
                }
            }

            memo.i_memo_if_stair++;	// -------------------------------Barometer LEVEL 4
            if (memo.i_memo_if_stair <= N_MEMO_IF_STAIR)
            {
                memo.if_stair[memo.i_memo_if_stair-1] = nav.flag_stair;
            }
            else
            {
                memo.i_memo_if_stair = N_MEMO_IF_STAIR;
                for (k=0; k<N_MEMO_IF_STAIR-1; k++)
                {
                    memo.if_stair[k] = memo.if_stair[k+1];
                }
                memo.if_stair[N_MEMO_IF_STAIR-1] = nav.flag_stair;
            }

            if (memo.i_memo_if_stair == N_MEMO_IF_STAIR)
            {
                nav.mean_if_stair = cal_mean_float((FLOAT32*)memo.if_stair, N_MEMO_IF_STAIR);
                if (nav.mean_if_stair > 0.5)
                {
                    nav.indi_status_stair = 1;
                }
                else if (nav.mean_if_stair < -0.5)
                {
                    nav.indi_status_stair = -1;
                }
                else
                {
                    nav.indi_status_stair = 0;
                }
            }
            memo.i_memo_status_stair++;	// -------------------------------Barometer LEVEL 5
            if (memo.i_memo_status_stair <= N_MEMO_STATUS_STAIR)
            {
                memo.status_stair[memo.i_memo_status_stair-1] = nav.indi_status_stair;
            }
            else
            {
                memo.i_memo_status_stair = N_MEMO_STATUS_STAIR;
                for (k=0; k<N_MEMO_STATUS_STAIR-1; k++)
                {
                    memo.status_stair[k] = memo.status_stair[k+1];
                }
                memo.status_stair[N_MEMO_STATUS_STAIR-1] = nav.indi_status_stair;
            }


            nav.if_change_floor = 0;


            nav.diff_height_to_height0 = nav.height_baro_sm - memo.height_baro0;
            // ----------------  Only for GNSS_CENTER -----------------
            if (nav.indi_status_stair == 0)
            {
                if (nav.diff_height_to_height0 < -2.5)
                {
                    if (nav.floor_num == 12)
                    {
                        nav.if_change_floor = 1;
                    }
                    nav.floor_num = 11;
                }
                else if (nav.diff_height_to_height0 > -1.5)
                {
                    if (nav.floor_num == 11)
                    {
                        nav.if_change_floor = 1;
                    }
                    nav.floor_num = 12;
                }
                else
                {
                    // ...
                }
            }
            // ================  Only for GNSS_CENTER ==================


            sol->avail_has_baro_in_step = 1;
            sol->sol_t_height_baro = meas.t_baro;
            sol->height_baro_sm = nav.height_baro_sm;
            sol->height_change = nav.height_change;
            sol->flag_stair = nav.flag_stair;
            sol->indi_status_stair = nav.indi_status_stair;
            //sol->floor_num = nav.floor_num;


            // ----------------- NEW FOR COMPETITION -------------------------
			if (fabs(nav.height_change) < TH_FLAT_FLOOR_CHANGE_HEIGHT0)
			{
				// memo.height_baro0 += nav.height_change;
				printf("%.2fs, height0 = %.2f, height_change = %.2f, height = %.2f\n", meas.t_baro, memo.height_baro0,nav.height_change, meas.da_baro);
			}


			// ------------------------- 4 DBs ------------------------- 20151007
			  float def_height_2_initial = sol->height_baro_sm - memo.height_baro0;


				// int HEIGHT_1_FLOOR = 3;     // ################################

				num_floor[0] = 1;
				if (if_first_floor_3_KCCI == 0) {
					if_first_floor_3_KCCI = 1;
					memo.height_baro0 = sol->height_baro_sm;
				}
				if (if_first_floor_3_KCCI == 1 && if_first_floor_2_KCCI == 0 && if_first_floor_1_KCCI == 0 &&
					if_first_floor_1_PDC == 0 && if_first_floor_2_PDC == 0 && if_first_floor_3_PDC == 0)
				{
					num_floor[0] = 3;
					def_height_2_initial = sol->height_baro_sm - memo.height_baro0;
					if (def_height_2_initial < -2) {
						if_first_floor_2_KCCI = 1;
						memo.height_baro0 = memo.height_baro0 - 4;
					}
				}
				if (if_first_floor_3_KCCI == 1 && if_first_floor_2_KCCI == 1 && if_first_floor_1_KCCI == 0 &&
					  if_first_floor_1_PDC == 0 && if_first_floor_2_PDC == 0 && if_first_floor_3_PDC == 0)
				{
					num_floor[0] = 2;
					def_height_2_initial = sol->height_baro_sm - memo.height_baro0;
					if (def_height_2_initial < -2) {
						if_first_floor_1_KCCI = 1;
						memo.height_baro0 = memo.height_baro0 - 4;
					}
				}

//			    if (if_this_is_first_GPS == 1 && meas.accuracy_gps_result < 7.1)
//				{
//					if_first_floor_1_PDC = 1;
//				}

				if (if_first_floor_3_KCCI == 1 && if_first_floor_2_KCCI == 1 && if_first_floor_1_KCCI == 1 &&
					if_first_floor_1_PDC == 1 && if_first_floor_2_PDC == 0 && if_first_floor_3_PDC == 0) {
					if (meas.accuracy_gps_result > 23.5 && meas.t_accel - t_start_outdoor_gps > 30.0)
					{
						if (if_this_is_first_gps_height == 0)
						{
							if_this_is_first_gps_height = 1;
							num_floor[0] = 1;
                        	memo.height_baro0 = sol->height_baro_sm;

                        	t_start_PDC = meas.t_accel;

						}

					}
				}



				if (if_first_floor_3_KCCI == 1 && if_first_floor_2_KCCI == 1 && if_first_floor_1_KCCI == 1 && if_first_floor_1_PDC == 1 && if_first_floor_2_PDC == 0 && if_first_floor_3_PDC == 0) {
					def_height_2_initial = sol->height_baro_sm - memo.height_baro0;
					if (def_height_2_initial > 2.3) {
						if_first_floor_2_PDC = 1;
						memo.height_baro0 =  sol->height_baro_sm;

						t_end_PDC_1 = meas.t_accel;
						if(t_end_PDC_1 - t_start_PDC < 50)
						{
							// go back to 1
							if_first_floor_2_PDC = 0;
							num_floor[0] = 1;
							memo.height_baro0 = sol->height_baro_sm;

							t_start_PDC = t_end_PDC_1;
						}

						t_start_PDC2 = meas.t_accel;
 					}
				}



				if (if_first_floor_3_KCCI == 1 && if_first_floor_2_KCCI == 1 && if_first_floor_1_KCCI == 1 && if_first_floor_1_PDC == 1 && if_first_floor_2_PDC == 1 && if_first_floor_3_PDC == 0) {
					def_height_2_initial = sol->height_baro_sm - memo.height_baro0;
					if (def_height_2_initial > 2) {
						if_first_floor_3_PDC = 1;
						memo.height_baro0 =  sol->height_baro_sm;
						num_floor[0] = 3;

						t_end_PDC2_1 = meas.t_accel;
						if(t_end_PDC2_1 - t_start_PDC2 < 30)
						{
							// go back to 1
							if_first_floor_3_PDC = 0;
							num_floor[0] = 2;
							memo.height_baro0 = sol->height_baro_sm;

							t_start_PDC2 = t_end_PDC2_1;
						}
					}
				}

				if (if_first_floor_3_KCCI == 1 && if_first_floor_2_KCCI == 0 && if_first_floor_1_KCCI == 0 && if_first_floor_1_PDC == 0 && if_first_floor_2_PDC == 0 && if_first_floor_3_PDC == 0) {
					num_floor[0] = 3;
				}
				else if (if_first_floor_3_KCCI == 1 && if_first_floor_2_KCCI == 1 && if_first_floor_1_KCCI == 0 && if_first_floor_1_PDC == 0 && if_first_floor_2_PDC == 0 && if_first_floor_3_PDC == 0) {
					num_floor[0] = 2;
				}
				else if (if_first_floor_3_KCCI == 1 && if_first_floor_2_KCCI == 1 && if_first_floor_1_KCCI == 1 && if_first_floor_1_PDC == 0 && if_first_floor_2_PDC == 0 && if_first_floor_3_PDC == 0) {
					num_floor[0] = 1;
				}
				else if (if_first_floor_3_KCCI == 1 && if_first_floor_2_KCCI == 1 && if_first_floor_1_KCCI == 1 && if_first_floor_1_PDC == 1 && if_first_floor_2_PDC == 0 && if_first_floor_3_PDC == 0) {
					num_floor[0] = 1;
				}
				else if (if_first_floor_3_KCCI == 1 && if_first_floor_2_KCCI == 1 && if_first_floor_1_KCCI == 1 && if_first_floor_1_PDC == 1 && if_first_floor_2_PDC == 1 && if_first_floor_3_PDC == 0) {
					num_floor[0] = 2;
				}
				else if (if_first_floor_3_KCCI == 1 && if_first_floor_2_KCCI == 1 && if_first_floor_1_KCCI == 1 && if_first_floor_1_PDC == 1 && if_first_floor_2_PDC == 1 && if_first_floor_3_PDC == 1) {
					num_floor[0] = 3;
				}

				sol->floor_num = num_floor[0];


				if (if_first_floor_1_PDC == 1)
				{
					if (if_not_in_KCCI)
					{
						if_not_in_KCCI = 1;
					}
				}


//			float def_height_2_initial = sol->height_baro_sm - memo.height_baro0;
//
//
//			int HEIGHT_1_FLOOR = 3;     // ################################
//
//
//			if (def_height_2_initial > HEIGHT_1_FLOOR *3)
//				num_floor[0] = 4;
//			else if (def_height_2_initial > HEIGHT_1_FLOOR *2)
//				num_floor[0] = 3;
//			else if (def_height_2_initial > HEIGHT_1_FLOOR)
//				num_floor[0] = 2;
//			else
//				num_floor[0] = 1;
//
//			sol->floor_num = num_floor[0];
			// ========================= 4 DBs =========================

			// ================= NEW FOR COMPETITION =========================

        }  // End if (if_has_baro_in_step)
    }
    // =================================   new for baro  ================================================






    // --------------------------------- new for Initialization -------------------------------------
    if (!if_heading_initialized)
    {
        //if (ini->type_initializaiton == 100) {
        if(TYPE_INITIALIZATION == 100) {
            if (meas.avail_mag)
            {
                nav.heading_platform = memo.heading_mag[memo.i_memo_heading_mag-1];   //Heading to true north, rad
                if_heading_initialized = 1;
            }
        }
        //else if (ini->type_initializaiton == 0 || ini->type_initializaiton == 1) {
        else if (TYPE_INITIALIZATION == 0 || TYPE_INITIALIZATION == 1) {
            if_heading_initialized = 1;
        }
    }

    if (!if_position_initialized)
    {
       //if (ini->type_initializaiton == 100) {
	   if(TYPE_INITIALIZATION == 100) {
            if (if_has_wifi_result_for_initialization)
            {
                nav.pos[0] = pos_wifi[0];
                nav.pos[1] = pos_wifi[1];
                //nav.pos[2] = ini->ini_pos[2];     // height to be determined by baro
                if_position_initialized = 1;
            }
        }
        //else if (ini->type_initializaiton == 0 || ini->type_initializaiton == 1) {
		else if (TYPE_INITIALIZATION == 0 || TYPE_INITIALIZATION == 1) {
            if_position_initialized = 1;
        }
    }

    if (!if_nav_initialized) {
        if (if_heading_initialized && if_position_initialized) {
            if_nav_initialized = 1;
        }
    }
    // ================================= new for Initialization =====================================







    // ----------------------------------- Heading Calculation ----------------------------------------

    if (if_nav_initialized && meas.avail_gyro)
    {
        if (memo.i_gyro>1)
        {
            ts_gyro = memo.t_gyro[memo.i_gyro-1]-memo.t_gyro[memo.i_gyro-2];
        }
        else {
            ts_gyro = 1.0/SENSORS_RATE_MOBILE;
        }

        if (memo.i_gyro < LENGTH_MEMO_SENSOR_DATA)
        {
            memo.heading_platform[memo.i_gyro-1] = nav.heading_platform;
        } else {

            for (i=0; i<LENGTH_MEMO_SENSOR_DATA-1; i++)
            {
                memo.heading_platform[i] = memo.heading_platform[i+1];
            }
            memo.heading_platform[LENGTH_MEMO_SENSOR_DATA-1] = nav.heading_platform;
        }


        // -------------------------------- Misalignment Estimator -------------------------------------
        if (memo.i_accel > 0)
        {
            f_b[0] = memo.a_x[memo.i_accel-1];
            f_b[1] = memo.a_y[memo.i_accel-1];
            f_b[2] = memo.a_z[memo.i_accel-1];

            // --------------------------------
            sol->avail_rp_accel = 1;
            sol->sol_t_rp_accel = meas.t_gyro;
            sol->sol_rp_accel[0] = cal_roll_att(f_b);
            sol->sol_rp_accel[1] = cal_pitch_att(f_b);
            sol->sol_rp_accel[0] /= D2R;
            sol->sol_rp_accel[1] /= D2R;
        }
        else
        {
            f_b[0] = 0.0;
            f_b[1] = 0.0;
            f_b[2] = -9.8;
        }

        if (IF_USE_MISALIGNMENT)
        {
            if (MODE_MISALIGNMENT == 0)
            {
                gyro_norm = sqrt(meas.da_gyro[0]*meas.da_gyro[0] + meas.da_gyro[1]*meas.da_gyro[1] + meas.da_gyro[2]*meas.da_gyro[2]);
                rad_rotation = 1 / D2R / gyro_norm;		 // Radius of Rotation

                Euler2Dcm_att_float_float_3_3_mis(att_pkt.att[0], att_pkt.att[1], 0, R_ls);
                for(j = 0; j < 3; j++)
                {
                    f_n[j] = 0.0;
                    for(k = 0; k < 3; k++)	// bringing in the accel into the level plane
                    {
                        //f_n[j] += R_ls[j][k]*fb_sm[k];
                        f_n[j] += R_ls[j][k]*f_b[k];
                    }

                    f_leveled[j] = f_n[j];
                }
                f_n[2] += Constant_Gravity;
                norm_a_n = f_n[0]*f_n[0] + f_n[1]*f_n[1] + f_n[2]*f_n[2];

                i_memo_acc_norm ++;
                if (i_memo_acc_norm <= N_MEM_misO_1_STEP_MIS)
                {
                    memo_acc_norm[i_memo_acc_norm-1] = norm_a_n;
                }
                else
                {
                    i_memo_acc_norm = N_MEM_misO_1_STEP_MIS;
                    for (k=0; k<N_MEM_misO_1_STEP_MIS-1; k++)
                    {
                        memo_acc_norm[k] = memo_acc_norm[k+1];
                    }
                    memo_acc_norm[N_MEM_misO_1_STEP_MIS-1] = norm_a_n;
                }

                mean_acc_norm = cal_mean_float_att(memo_acc_norm, i_memo_acc_norm);
                da_Acc_Variance_Data = cal_variance_float(memo_acc_norm,mean_acc_norm,i_memo_acc_norm);

                height_change_Data = 0;       // To be changed to the real detection of floor changing

                // Fill input structure for misalignment estimation
                misa.in_cur_Lev_Acc_Data[0] = f_leveled[0];  //Lev_Acc_Data[0][mm];
                misa.in_cur_Lev_Acc_Data[1] = f_leveled[1];  //Lev_Acc_Data[1][mm];
                misa.in_cur_Lev_Acc_Data[2] = f_leveled[2];  //Lev_Acc_Data[2][mm];
                misa.in_cur_pca_data_h = 0.0;
                misa.in_cur_pca_Acc_Mag = 0.0;
                // misa.in_cur_Lev_Acc_Window[3];
                misa.in_cur_RP_Data[0] = att_pkt.att[0];
                misa.in_cur_RP_Data[1] = att_pkt.att[1];
                misa.in_cur_Acc_Variance_Data = da_Acc_Variance_Data; //Acc_Variance_Data[mm];
                misa.in_cur_Rad_Rotation_Data = rad_rotation;  //Rad_Rotation_Data[mm];
                misa.in_cur_Gyr_Norm_Data = gyro_norm; // Gyr_Norm_Data[mm];
                misa.in_cur_Step_Frequency_Data = 0.0; // misa.Step_Frequency_Data[mm];
                misa.in_cur_vertical_motion_phase_Data = 0.0; //misa.vertical_motion_phase_Data[mm];
                misa.in_cur_motion_effective_coefficient_Data = 0.0; //misa.motion_effective_coefficient_Data[mm];
                misa.in_cur_Vertical_Change_Data = att_pkt.if_vertical;
                misa.in_cur_Height_Change_Data = height_change_Data;

                misalign_estimation_mis(&misa);

                if (IF_PLAY_WITH_MISALIGN)
                {
                    play_with_misalign(&misa, &att_pkt, &memo);
                    misalignment_uesd = misa.Misalign_Az_For_Use;
                }
                else
                {
                    misalignment_uesd = misa.Misalign_Az;
                }
            }
            else if (MODE_MISALIGNMENT == 1)
            {
                misa.Misalign_Az = FIXED_MISALIGNEMNT;
                misa.motion_mode = 0;
                misalignment_uesd = FIXED_MISALIGNEMNT;
            }

        }
        else   // if MODE_MISALIGNMENT != 0
        {
            misa.Misalign_Az = 0.0;
            misa.motion_mode = 0;
            misalignment_uesd = 0.0;
        }
        // ================================ Misalignment Estimator =====================================


        // ---------------------------------- QS -----------------------------------------
        //find_max_min_float_mis(memo.gz_level, max_min_qs_detect, memo.i_gyro);
        //diff_max_min_gz = max_min_qs_detect[0] - max_min_qs_detect[1];
        //if_qs = 0;
        //if (IF_USE_CURRENT_QS && diff_max_min_gz < TH_DETECT_QS_GYRO && memo.i_gyro == LENGTH_MEMO_SENSOR_DATA)

        find_max_min_float_mis(memo.norm_gyro, max_min_qs_detect, memo.i_gyro);
        diff_max_min_gz = max_min_qs_detect[0] - max_min_qs_detect[1];

        if_qs = 0;
        if (IF_USE_CURRENT_QS && diff_max_min_gz < TH_DETECT_QS_GYRO_NORM_20_DEG_S && memo.i_gyro == LENGTH_MEMO_SENSOR_DATA)
        {
            if_qs = 1;
        }
        sol->avail_show_qs = 1;
        sol->sol_t_qs = memo.t_gyro[memo.i_gyro-1];
        sol->if_qs = if_qs;
        sol->diff_norm_gyro = diff_max_min_gz / D2R;
        sol->norm_gyro = memo.norm_gyro[memo.i_gyro-1] / D2R;
        sol->i_memo_norm_gyro_QS = memo.i_memo_norm_gyro_QS;
        // ================================== QS =========================================


        if (!if_qs)
        {
            if (IF_USE_ATT_FILTER)
            {
                if (memo.i_accel > 0)
                {
                    flag_accel = 1;
                    fb_sm[0] = cal_mean_float(memo.a_x, memo.i_accel);
                    fb_sm[1] = cal_mean_float(memo.a_y, memo.i_accel);
                    fb_sm[2] = cal_mean_float(memo.a_z, memo.i_accel);

                    // --------------------------------
                    sol->sol_rp_accel_sm[0] = cal_roll_att(fb_sm);
                    sol->sol_rp_accel_sm[1] = cal_pitch_att(fb_sm);
                    sol->sol_rp_accel_sm[0] /= D2R;
                    sol->sol_rp_accel_sm[1] /= D2R;
                }
                else
                {
                    flag_accel = 0;
                    fb_sm[0] = 0.0;
                    fb_sm[1] = 0.0;
                    fb_sm[2] = -9.8;
                }


                // ------------------------ mag input -----------------------------
                flag_mag = 0;
                //float mean_ind_qsmf = cal_mean_float((FLOAT32*)memo.ind_qsmf,memo.i_memo_indi_qsmf);
                //if (IF_USE_MAG_UPDATE && memo.i_memo_normm == N_EPH_MEMO_NORM && fabs(memo.t_mag[memo.i_mag-1]-t_cur) < TH_TIME_DIFF_USE_ACC_MAG && mean_ind_qsmf > 0.5)
                //if (IF_USE_MAG_UPDATE && memo.i_memo_normm == N_EPH_MEMO_NORM && fabs(memo.t_mag[memo.i_mag-1]-t_cur) < TH_TIME_DIFF_USE_ACC_MAG)
                if (IF_USE_MAG_UPDATE && (!IF_USE_QSMF_LIMIT || (IF_USE_QSMF_LIMIT && memo.ind_qsmf[memo.i_memo_indi_qsmf-1])))
                {
                    flag_mag = 1;
                    m_b[0] = memo.m_x[memo.i_mag-1];
                    m_b[1] = memo.m_y[memo.i_mag-1];
                    m_b[2] = memo.m_z[memo.i_mag-1];

                    //if (!memo.ind_qsmf[memo.i_memo_indi_qsmf-1])     // The use of QSMF
                    //{
                    //	flag_mag = 0;
                    //}
                }
                else
                {
                    flag_mag = 0;
                    //m_b[0] = 0.0;
                    //m_b[1] = 0.0;
                    //m_b[2] = 0.0;
                }
                // ======================== mag input =============================







                // -------------------------------- Attitude Filter ------------------------------------



                //if (memo.i_gyro == 1)
                //{
                //	ini_heading = center_heading(nav.heading_platform + misa.Misalign_Az);  //Rad
                //}

                ini_heading = center_heading(nav.heading_platform + misa.Misalign_Az);  //Rad
                flag_ini_heading = 1;
                seed_availability = IF_SEED_INI_GYRO_BIAS_ATT;
                seed_bias[0] = ini_gyro_bias_att[0];
                seed_bias[1] = ini_gyro_bias_att[1];
                seed_bias[2] = ini_gyro_bias_att[2];

                attitude_filter_without_bg_conpensation_att(&att_pkt, &memo, meas.t_gyro, meas.da_gyro, fb_sm, flag_accel, m_b, flag_mag, ini_heading, flag_ini_heading, seed_bias, seed_availability);
                //attitude_filter_without_bg_conpensation_att(&att_pkt, t_cur, w_b, fb_sm, flag_accel, m_b, flag_mag, ini_heading, flag_ini_heading, seed_bias, seed_availability);
                //attitude_filter_without_bg_conpensation_att(&att_pkt, t_cur, w_b, f_b, flag_accel, m_b, flag_mag, ini_heading, flag_ini_heading, seed_bias, seed_availability);
                nav.heading_device = att_pkt.att[2];

                // ================================ End of Attitude Filter =====================================



                //att_pkt.heading_platform = center_heading(att_pkt.att[2] - misalignment_uesd);     // Device heading - misalignment = platform heading;
                nav.heading_platform = center_heading(att_pkt.att[2] - misalignment_uesd);     // Device heading - misalignment = platform heading; att_pkt.heading_platform;


                sol->avail_att_mis = 1;
                sol->sol_t_att_mis = att_pkt.t;
                sol->sol_att[0] = att_pkt.att[0] / D2R;
                sol->sol_att[1] = att_pkt.att[1] / D2R;
                sol->sol_att[2] = att_pkt.att[2] / D2R;
                sol->if_vertical = att_pkt.if_vertical;
                sol->sol_bg_estimated[0] = att_pkt.bg_estimated[0] / D2R;
                sol->sol_bg_estimated[1] = att_pkt.bg_estimated[1] / D2R;
                sol->sol_bg_estimated[2] = att_pkt.bg_estimated[2] / D2R;

                sol->misalign_az = misa.Misalign_Az / D2R;
                sol->motion_mode = misa.motion_mode;
                sol->fb_detect = misa.dir_s1;
                sol->step_frequency = misa.step_frequency_data;

                sol->misalignment_used = misalignment_uesd / D2R;
                sol->heading_platform = nav.heading_platform / D2R;
            }
            else   // if (!IF_USE_ATT_FILTER)
            {
                nav.heading_platform += memo.gz_level[memo.i_gyro-1]*ts_gyro;
                nav.heading_platform = center_heading(nav.heading_platform);

                sol->avail_att_mis = 0;
            }

        }
        else   // QS
        {
            if (IF_USE_ATT_FILTER)
            {

                flag_accel = 0;
                flag_mag = 0;
                w_b_zero[0] = 0.0;
                w_b_zero[1] = 0.0;
                w_b_zero[2] = 0.0;

                ini_heading = center_heading(nav.heading_platform + misa.Misalign_Az);  //Rad
                flag_ini_heading = 1;
                seed_availability = IF_SEED_INI_GYRO_BIAS_ATT;
                seed_bias[0] = ini_gyro_bias_att[0];
                seed_bias[1] = ini_gyro_bias_att[1];
                seed_bias[2] = ini_gyro_bias_att[2];

                attitude_filter_without_bg_conpensation_att(&att_pkt, &memo, meas.t_gyro, w_b_zero, fb_sm, flag_accel, m_b, flag_mag, ini_heading, flag_ini_heading, seed_bias, seed_availability);
                nav.heading_device = att_pkt.att[2];
                nav.heading_platform = center_heading(att_pkt.att[2] - misalignment_uesd);     // Device heading - misalignment = platform heading; att_pkt.heading_platform;

                sol->avail_att_mis = 1;
                sol->sol_t_att_mis = att_pkt.t;
                sol->sol_att[0] = att_pkt.att[0] / D2R;
                sol->sol_att[1] = att_pkt.att[1] / D2R;
                sol->sol_att[2] = att_pkt.att[2] / D2R;
                sol->if_vertical = att_pkt.if_vertical;
                sol->sol_bg_estimated[0] = att_pkt.bg_estimated[0] / D2R;
                sol->sol_bg_estimated[1] = att_pkt.bg_estimated[1] / D2R;
                sol->sol_bg_estimated[2] = att_pkt.bg_estimated[2] / D2R;

                sol->misalign_az = misa.Misalign_Az / D2R;
                sol->motion_mode = misa.motion_mode;
                sol->fb_detect = misa.dir_s1;
                sol->step_frequency = misa.step_frequency_data;

                sol->misalignment_used = misalignment_uesd / D2R;
                sol->heading_platform = nav.heading_platform / D2R;
            }
            else   // if (!IF_USE_ATT_FILTER)
            {
                nav.heading_platform += 0.0*ts_gyro;
                nav.heading_platform = center_heading(nav.heading_platform);

                sol->avail_att_mis = 0;
            }
        }

        //sol->count_rec_heading ++;
        sol->avail_heading = 1;
        sol->sol_t_heading = memo.t_gyro[memo.i_gyro-1];
        sol->sol_heading = memo.heading_platform[memo.i_gyro-1]/D2R;
        //fprintf(sol.f_out_rec_heading, "%.4f %.4f\n", memo.t_gyro[memo.i_gyro-1], memo.heading_platform[memo.i_gyro-1]);

    }
    // =================================== End of heading calculation ===================================




    // ----------------------------------- Step detection -----------------------------------------------
    // -- Step detection and PDR
    if (if_nav_initialized && meas.avail_accel)
    {
        if_detect_step = 0;
        if_update_step = 0;
        if (memo.i_va == N_MEMO_VA)
        {
            if_detect_step = 1;
        }

        if (if_detect_step)
        {
            /*max_va = -15.0;
             min_va = -5.0;*/
            max_va = 5.0;
            min_va = 15.0;
            t_max_va = 0.0;
            t_min_va_4_step = 0.0;
            i_eph_in_memo_min_va = -1;

            for(i=0; i<N_MEMO_VA; i++){
                if (memo.va[i] > max_va)
                {
                    max_va = memo.va[i];
                    t_max_va = memo.t_va[i];
                }
                if (memo.va[i] < min_va)
                {
                    min_va = memo.va[i];
                    t_min_va_4_step = memo.t_va[i];
                    i_eph_in_memo_min_va = i+1;						////// same as matlab
                }
            }

            if_update_step = 0;
            if (i_eph_in_memo_min_va == I_MIN_VA_IN_MEMO)
            {
                //if (max_va-min_va > TH_MAX_MIN_VA && min_va < TH_A_STEP2)
                if (min_va < TH_A_STEP2)
                    // if (max_va-min_va > TH_MAX_MIN_VA)
                {
                    if_update_step = 1;
                    if (memo.i_t_step)
                    {
                        if (t_min_va_4_step-memo.t_step[memo.i_t_step-1]<TH_MIN_T_DIFF_STEP)
                        {
                            if_update_step = 0;
                        }
                    }
                }
            }


            if (if_update_step)
            {
                //sol->count_rec_step ++;
                sol->avail_step = 1;
                sol->sol_t_step = t_min_va_4_step;
                sol->sol_step = min_va;
                //fprintf(sol.f_out_rec_step, "%.4f %.4f\n", t_mid1, ma_mid1);

                memo.i_t_step ++;
                if(memo.i_t_step <=LENGTH_MEMO_T_STEP){
                    memo.t_step[memo.i_t_step-1] = t_min_va_4_step;
                } else {
                    memo.i_t_step = LENGTH_MEMO_T_STEP;
                    for (i=0; i<LENGTH_MEMO_T_STEP-1; i++) {
                        memo.t_step[i] = memo.t_step[i+1];
                    }
                    memo.t_step[LENGTH_MEMO_T_STEP-1] = t_min_va_4_step;
                }

                nav.t_step0 = nav.t_step1;
                nav.t_step1 = t_min_va_4_step;
                nav.t_s_step = nav.t_step1 - nav.t_step0;

                if_update_pos = 1;
            }
        }
        // =================================== End of Step detection =====================================




        // ------------------------------------------- PDR -----------------------------------------------
        if (if_update_pos)			// Update PDR POS
        {
            this_is_step = 1;     // For using barometer

            //  ----------------------------------- Heading Adjust --------------------
            for (i=0; i<LENGTH_MEMO_SENSOR_DATA; i++){
                t_diff_g_a[i] = fabs(t_min_va_4_step - memo.t_gyro[i]);
            }
            min_double(t_diff_g_a, min_t_heading_in_memo, LENGTH_MEMO_SENSOR_DATA);
            i_heading_in_memo = floor(min_t_heading_in_memo[1]+0.001);
            nav.heading_step = memo.heading_platform[i_heading_in_memo];


            // -------------------------   new for map   -----------------------------
            if_align_heading = 0;
            if (IF_ALIGN_TO_LINKS && if_not_in_KCCI) {
                if_align_heading = map_adjusting(&nav, &att_pkt, &memo);
            }

            // =========================   new for map  =============================

            memo.i_memo_heading_step ++;
            if(memo.i_memo_heading_step <=LENGTH_MEMO_POS_PDR){
                memo.heading_step[memo.i_memo_heading_step-1] = nav.heading_step;
            } else {
                memo.i_memo_heading_step = LENGTH_MEMO_POS_PDR;
                for (i=0; i<LENGTH_MEMO_POS_PDR-1; i++) {
                    memo.heading_step[i] = memo.heading_step[i+1];
                }
                memo.heading_step[LENGTH_MEMO_POS_PDR-1] = nav.heading_step;
            }
            //  =================================== Heading Adjust ====================


            // pos_NEH
            d_North = nav.sl*cos(nav.heading_step);
            d_East  = nav.sl*sin(nav.heading_step);

            nav.pos[0] += d_North/RM;
            nav.pos[1] += d_East/RN_COS_LON;
            //nav.pos[2] = nav.pos[2];

            pos_NEH[0] += d_North;
            pos_NEH[1] += d_East;


            //pos_NEH[0] = (nav.pos[0]-ZERO_POS[0])*RM;
            //pos_NEH[1] = (nav.pos[1]-ZERO_POS[1])*RN_COS_LON;
            //pos_NEH[2] = nav.pos[2];

            // For debug
            sol->avail_heading_step = 1;
            sol->sol_t_heading_step = t_min_va_4_step;
            sol->sol_heading_step = nav.heading_step / D2R;


            // KF prediction
            ChangeArrayMemoryLayout_float(N_STATES,N_STATES,kf_4.pP,(FLOAT32 *)kf_4.P,0);         //P


            // ------- NEW - Change Qk values after a period -----------
            if (!if_has_changed_Qk && t_min_va_4_step > T_BETWEEN_PHASE_12)
            {
                if (N_STATES == 4)
                {
                    kf_4.Qk[0][0] = QK_POS_PHASE_2 * QK_POS_PHASE_2;
                    kf_4.Qk[1][1] = QK_POS_PHASE_2 * QK_POS_PHASE_2;
                    kf_4.Qk[2][2] = QK_HEAD_PHASE_2 * QK_HEAD_PHASE_2;
                    kf_4.Qk[3][3] = QK_SL * QK_SL;
                }
                else if (N_STATES == 3)
                {
                    kf_4.Qk[0][0] = QK_POS_PHASE_2 * QK_POS_PHASE_2;
                    kf_4.Qk[1][1] = QK_POS_PHASE_2 * QK_POS_PHASE_2;
                    kf_4.Qk[2][2] = QK_HEAD_PHASE_2 * QK_HEAD_PHASE_2;
                }
                if_has_changed_Qk = 1;
            }
            // ======= NEW - Change Qk values after a period ============

            ChangeArrayMemoryLayout_float(N_STATES,N_STATES,kf_4.pQk,(FLOAT32 *)kf_4.Qk,0);       //Qk

            if (N_STATES == 4)
            {
                PHI[0][0] = 1.0; PHI[0][1] = 0.0; PHI[0][2] = -nav.sl*sin(nav.heading_step); PHI[0][3] = cos(nav.heading_step);
                PHI[1][0] = 0.0; PHI[1][1] = 1.0; PHI[1][2] = nav.sl*cos(nav.heading_step);  PHI[1][3] = sin(nav.heading_step);
                PHI[2][0] = 0.0; PHI[2][1] = 0.0; PHI[2][2] = 1.0;  PHI[2][3] = 0.0;
                PHI[3][0] = 0.0; PHI[3][1] = 0.0; PHI[3][2] = 0.0;  PHI[3][3] = 1.0;
            }
            else if (N_STATES == 3)
            {
                PHI[0][0] = 1.0; PHI[0][1] = 0.0; PHI[0][2] = -nav.sl*sin(nav.heading_step);
                PHI[1][0] = 0.0; PHI[1][1] = 1.0; PHI[1][2] = nav.sl*cos(nav.heading_step);
                PHI[2][0] = 0.0; PHI[2][1] = 0.0; PHI[2][2] = 1.0;
            }

            ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pPHI,(FLOAT32 *)PHI,0);	              //Phi
            ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pPHIT,(FLOAT32 *)PHIT,0);			  //Phi'
            ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pPHIP,(FLOAT32 *)PHIP,0);			  //Phi*P
            ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pPHIPPHIT,(FLOAT32 *)PHIPPHIT,0);	  //Phi*P*Phi'

            //printf("\nP:\n");
            //printf("%.8f\t%.8f\t%.8f\t%.8f\n", kf_4.pP[0][0],kf_4.pP[0][1],kf_4.pP[0][2],kf_4.pP[0][3]);
            //printf("%.8f\t%.8f\t%.8f\t%.8f\n", kf_4.pP[1][0],kf_4.pP[1][1],kf_4.pP[1][2],kf_4.pP[1][3]);
            //printf("%.8f\t%.8f\t%.8f\t%.8f\n", kf_4.pP[2][0],kf_4.pP[2][1],kf_4.pP[2][2],kf_4.pP[2][3]);
            //printf("%.8f\t%.8f\t%.8f\t%.8f\n", kf_4.pP[3][0],kf_4.pP[3][1],kf_4.pP[3][2],kf_4.pP[3][3]);

            matrix_transpose_nxn_float(pPHI, N_STATES, N_STATES, pPHIT);						  //PHI'
            multiply_matrices_nxn_float(pPHI, kf_4.pP, N_STATES, N_STATES, N_STATES, pPHIP);	  //Phi*P
            multiply_matrices_nxn_float(pPHIP, pPHIT, N_STATES, N_STATES, N_STATES, pPHIPPHIT);	  //Phi*P*Phi'

            //printf("\nPHIPPHIT:\n");
            //printf("%.8f\t%.8f\t%.8f\t%.8f\n", pPHIPPHIT[0][0],pPHIPPHIT[0][1],pPHIPPHIT[0][2],pPHIPPHIT[0][3]);
            //printf("%.8f\t%.8f\t%.8f\t%.8f\n", pPHIPPHIT[1][0],pPHIPPHIT[1][1],pPHIPPHIT[1][2],pPHIPPHIT[1][3]);
            //printf("%.8f\t%.8f\t%.8f\t%.8f\n", pPHIPPHIT[2][0],pPHIPPHIT[2][1],pPHIPPHIT[2][2],pPHIPPHIT[2][3]);
            //printf("%.8f\t%.8f\t%.8f\t%.8f\n", pPHIPPHIT[3][0],pPHIPPHIT[3][1],pPHIPPHIT[3][2],pPHIPPHIT[3][3]);

            add_matrices_nxn(pPHIPPHIT, kf_4.pQk, N_STATES, N_STATES, kf_4.pP);					  //P = Phi*P*Phi'+ Qk

            //printf("\nP:\n");
            //printf("%.8f\t%.8f\t%.8f\t%.8f\n", kf_4.pP[0][0],kf_4.pP[0][1],kf_4.pP[0][2],kf_4.pP[0][3]);
            //printf("%.8f\t%.8f\t%.8f\t%.8f\n", kf_4.pP[1][0],kf_4.pP[1][1],kf_4.pP[1][2],kf_4.pP[1][3]);
            //printf("%.8f\t%.8f\t%.8f\t%.8f\n", kf_4.pP[2][0],kf_4.pP[2][1],kf_4.pP[2][2],kf_4.pP[2][3]);
            //printf("%.8f\t%.8f\t%.8f\t%.8f\n", kf_4.pP[3][0],kf_4.pP[3][1],kf_4.pP[3][2],kf_4.pP[3][3]);

            if(N_STATES == 4) {

            }
        } // end if_update_pos
    } // end if avail_accel
    // =================================== End of PDR ====================================================




    // ------------------------------------ KF Updates ---------------------------------------------------
    // ------------ Updates from Map matching ---------------
    if (if_nav_initialized && if_update_pos && IF_PROJECT_TO_LINK && if_align_heading)
    {
        rs_NE[0] = pos_NEH[0];
        rs_NE[1] = pos_NEH[1];

        for (j=0; j<nLine_link[0]; j++)
        {
            // Weight the proximity of the point to each link
            r1_NE[0] = da_map_NE[j][4];
            r1_NE[1] = da_map_NE[j][5];
            r2_NE[0] = da_map_NE[j][6];
            r2_NE[1] = da_map_NE[j][7];

            if (fabs(t_min_va_4_step - 43.7)<0.1)
            {
                t_min_va_4_step = t_min_va_4_step;
            }

            // DD[j] = ptoject2line(rs_NE, r1_NE, r2_NE, rp_NE);
            DD[j] = ptoject2line_can_be_outside(rs_NE, r1_NE, r2_NE, rp_NE);

            rprp_NE[j][0] = rp_NE[0];
            rprp_NE[j][1] = rp_NE[1];
            WSPD[j] = AP_4MAP/DD[j];

            // Weight the heading and bearing of the link
            delta_beta1 = fabs(dist_ang_rad(nav.heading_step,da_map_NE[j][3]));
            if (delta_beta1 > TH_MAX_DIFF_ANG_PROJECT_TO_LINK)
            {
                WSH[j] = WSH_NOT_SAME_ANG;
            }
            else {
                if (delta_beta1 < TH_MIN_DIFF_ANG_PROJECT_TO_LINK)
                {
                    delta_beta1 = AH_4MAP / TH_MIN_DIFF_ANG_PROJECT_TO_LINK;
                }
                WSH[j] = AH_4MAP / delta_beta1;
            }

            TWS[j] = WSPD[j] + WSH[j];
        }
        max_float(TWS, max_tws, nLine_link[0]);

        i_link_align_heading = (int)max_tws[1]+1;
        rp_Link[0] = rprp_NE[i_link_align_heading-1][0];
        rp_Link[1] = rprp_NE[i_link_align_heading-1][1];

        //printf("%-.3f %-.3f\n\n", rp_Link[0], rp_Link[1]);

        dist_4_rej_link_pos = cal_dist_2D_NE(rp_Link[0],rp_Link[1],pos_NEH[0],pos_NEH[1]);
        if (dist_4_rej_link_pos < TH_MAP_DIST_POINTS_REJECT && (i_link_align_heading == 1 || i_link_align_heading == 5))
            //if (dist_4_rej_link_pos < TH_MAP_DIST_POINTS_REJECT)
        {
            pos_NEH[0] = rp_Link[0];
            pos_NEH[1] = rp_Link[1];
            nav.pos[0] = pos_NEH[0] / RM + ZERO_POS[0];
            nav.pos[1] = pos_NEH[1] / RN_COS_LON + ZERO_POS[1];

            // DEBUG
            t_min_va_4_step = t_min_va_4_step;
            t_min_va_4_step = t_min_va_4_step;

        }
        i_link_align_heading = 0;
    }
    // ======== End of Updates from map matching ===========

	if_during_defeat_wifi_time = 0;

	if (IF_GPS_DEFEAT && IF_USE_GPS_RESULT) {
		if(IF_USE_WIFI && meas.avail_wifi){
			if (if_gps_can_defeat_wifi && meas.t_wifi - t_gps_defeat_start < TIME_DEFEAT_WIFI_LAST) {
				if_during_defeat_wifi_time = 1;
			}
		}
		if(IF_USE_MM_RESULT && meas.avail_mm_result){
			if (if_gps_can_defeat_wifi && meas.t_mm_result - t_gps_defeat_start < TIME_DEFEAT_WIFI_LAST) {
				if_during_defeat_wifi_time = 1;
			}
		}
	}

	// --------------- Updates from GPS result -------------------

	if(IF_USE_GPS_RESULT && meas.avail_gps_result && if_this_is_first_GPS == 0)
	{
		meas.avail_gps_result = 0;
		if_this_is_first_GPS = 1;
	}


        if (if_nav_initialized && IF_USE_GPS_RESULT && meas.avail_gps_result && meas.accuracy_gps_result <= TH_ACCUR_RELIABLE_GPS && meas.accuracy_gps_result > 0)
        {
            if (IF_GPS_DEFEAT && !if_during_defeat_wifi_time) {
                if (meas.accuracy_gps_result < TH_ACCUR_GPS_DEFEAT_WIFI) {
                    if_gps_can_defeat_wifi = 1;
                    t_gps_defeat_start = meas.t_gps_result;

                    if (if_this_is_first_GPS == 1 && meas.accuracy_gps_result < 8.1)
					{
						if_first_floor_1_PDC = 1;

						if (if_first_start_outdoor_gps == 0)
						{
							if_first_start_outdoor_gps = 1;
							t_start_outdoor_gps = meas.t_accel;
						}
					}
                }
                else {
                    if_gps_can_defeat_wifi = 0;
                }
            }

            if (IF_GPS_CALIBRATE_HEIGHT && IF_USE_GPS_RESULT) {
                if (meas.accuracy_gps_result > TH_ACCUR_GPS_CALIBRATE_HEIGHT) {
                    eph_gps_calibrate_height = 0;
                }
                else
                {
                    eph_gps_calibrate_height ++;
                    if (eph_gps_calibrate_height > TH_EPH_GPS_CALI_HEIGHT) {
                        memo.height_baro0 = memo.height_pre;
                        eph_gps_calibrate_height = 0;
                    }
                }
            }


            pos_gps_result[0] = meas.lat_gps_result * D2R;
            pos_gps_result[1] = meas.lon_gps_result * D2R;
            pos_gps_result[2] = meas.height_gps_result;
            accuracy_gps_result = meas.accuracy_gps_result;

            sol->avail_sol_gps_result_used = 1;
            sol->sol_t_sol_gps_result_used = meas.t_gps_result;
            sol->sol_gps_result_used[0] = pos_gps_result[0]/D2R;
            sol->sol_gps_result_used[1] = pos_gps_result[1]/D2R;
            sol->sol_gps_result_used[2] = pos_gps_result[2];
            sol->sol_gps_result_used[3] = accuracy_gps_result;

            // KF Update
            pos_NEH_meas[0] = (pos_gps_result[0]-ZERO_POS[0])*RM;
            pos_NEH_meas[1] = (pos_gps_result[1]-ZERO_POS[1])*RN_COS_LON;
            //pos_NEH_meas[2] = pos_wifi[2];

            ChangeArrayMemoryLayout_float(2,N_STATES,pHk_2,(FLOAT32 *)Hk_2,1);
            ChangeArrayMemoryLayout_float(2,2,pRk_2,(FLOAT32 *)Rk_2,0);		   //Rk = diag
            if (N_STATES == 4)
            {
                Hk_2[0][0] = 1; Hk_2[0][1] = 0; Hk_2[0][2] = 0; Hk_2[0][3] = 0;
                Hk_2[1][0] = 0; Hk_2[1][1] = 1; Hk_2[1][2] = 0; Hk_2[1][3] = 0;
            }
            else if (N_STATES == 3)
            {
                Hk_2[0][0] = 1; Hk_2[0][1] = 0; Hk_2[0][2] = 0;
                Hk_2[1][0] = 0; Hk_2[1][1] = 1; Hk_2[1][2] = 0;
            }

            Zk_2[0] = pos_NEH[0] - pos_NEH_meas[0];
            Zk_2[1] = pos_NEH[1] - pos_NEH_meas[1];

            multiply_matrix_with_vector_nxn(pHk_2, 2, N_STATES, xk, Hk_2xk);       //Hx = H*x
            float meas_noise_wifi_2[2] = {accuracy_gps_result, accuracy_gps_result};
            diag2_sq(meas_noise_wifi_2, 2, pRk_2);

            inno_2[0] = Zk_2[0] - Hk_2xk[0];
            inno_2[1] = Zk_2[1] - Hk_2xk[1];


            KF_update_indoor_float(2, xk, kf_4.pP, inno_2, pHk_2, pRk_2, N_STATES);

            if_kf_updated = 1;

            t_min_va_4_step = meas.t_accel;
        }
        // ======== End of Updates from GPS result ===========

    // --------------- Updates from WiFi -------------------
    if (IF_USE_WIFI && meas.avail_wifi)
    {
    	if (meas.t_accel - t0_accel < TH_START_USE_WIFI)
    	{
    		meas.avail_wifi = 0;
    	}
    }

    if (IF_USE_WIFI && meas.avail_wifi)
    {
        //num_floor[0] = 1;

		// ----- 6 DB
        if (if_first_floor_1_PDC == 0) {
            if (sol->floor_num == 3)
                num_floor[0] = 1;
            else if (sol->floor_num == 2)
                num_floor[0] = 2;
            else
                num_floor[0] = 3;
        }
        else if (if_first_floor_1_PDC == 1) {
            if (sol->floor_num == 1)
                num_floor[0] = 4;
            else if (sol->floor_num == 2)
                num_floor[0] = 5;
            else
                num_floor[0] = 6;
        }

        wifi_position_tracker_smooth(meas.name_ap, meas.rss_ap, meas.N_wifi, num_floor, pos_wifi, availability_wifi);

        if(availability_wifi[0]) {

            if (!if_has_wifi_result_for_initialization) {
                if_has_wifi_result_for_initialization = 1;
            }

            // Output WiFi
            sol->avail_sol_wifi = 1;
            sol->sol_t_sol_wifi = meas.t_wifi;
            sol->sol_wifi[0] = pos_wifi[0]/D2R;
            sol->sol_wifi[1] = pos_wifi[1]/D2R;
            sol->sol_wifi[2] = nav.pos[2];

            // KF Update
            pos_NEH_meas[0] = (pos_wifi[0]-ZERO_POS[0])*RM;
            pos_NEH_meas[1] = (pos_wifi[1]-ZERO_POS[1])*RN_COS_LON;
            //pos_NEH_meas[2] = pos_wifi[2];

            ChangeArrayMemoryLayout_float(2,N_STATES,pHk_2,(FLOAT32 *)Hk_2,1);
            ChangeArrayMemoryLayout_float(2,2,pRk_2,(FLOAT32 *)Rk_2,0);		   //Rk = diag
            if (N_STATES == 4)
            {
                Hk_2[0][0] = 1; Hk_2[0][1] = 0; Hk_2[0][2] = 0; Hk_2[0][3] = 0;
                Hk_2[1][0] = 0; Hk_2[1][1] = 1; Hk_2[1][2] = 0; Hk_2[1][3] = 0;
            }
            else if (N_STATES == 3)
            {
                Hk_2[0][0] = 1; Hk_2[0][1] = 0; Hk_2[0][2] = 0;
                Hk_2[1][0] = 0; Hk_2[1][1] = 1; Hk_2[1][2] = 0;
            }

            Zk_2[0] = pos_NEH[0] - pos_NEH_meas[0];
            Zk_2[1] = pos_NEH[1] - pos_NEH_meas[1];

            multiply_matrix_with_vector_nxn(pHk_2, 2, N_STATES, xk, Hk_2xk);       //Hx = H*x
            float meas_noise_wifi_2[2];
			if ((!IF_USE_GPS_RESULT) || (!IF_GPS_DEFEAT)) {
				meas_noise_wifi_2[0] = SIGMA_WIFI;
				meas_noise_wifi_2[1] = SIGMA_WIFI;
			}
			else {
				if (!if_during_defeat_wifi_time) {
					meas_noise_wifi_2[0] = SIGMA_WIFI;
					meas_noise_wifi_2[1] = SIGMA_WIFI;
				}
				else {
					meas_noise_wifi_2[0] = SIGMA_WIFI_DEFEATED;
					meas_noise_wifi_2[1] = SIGMA_WIFI_DEFEATED;
				}
			}
            diag2_sq(meas_noise_wifi_2, 2, pRk_2);

            inno_2[0] = Zk_2[0] - Hk_2xk[0];
            inno_2[1] = Zk_2[1] - Hk_2xk[1];

            KF_update_indoor_float(2, xk, kf_4.pP, inno_2, pHk_2, pRk_2, N_STATES);

            if_kf_updated = 1;
        }
    }
    // ======== End of Updates from WiFi ===========


    // --------------- Updates from MM result -------------------
    if (if_nav_initialized && IF_USE_MM_RESULT && meas.avail_mm_result)
    {
        pos_mm_result[0] = meas.lat_mm_result * D2R;
        pos_mm_result[1] = meas.lon_mm_result * D2R;
        pos_mm_result[2] = nav.pos[2];

        sol->avail_sol_mm_result_used = 1;
        sol->sol_t_sol_mm_result_used = meas.t_mm_result;
        sol->sol_mm_result_used[0] = pos_mm_result[0]/D2R;
        sol->sol_mm_result_used[1] = pos_mm_result[1]/D2R;
        sol->sol_mm_result_used[2] = pos_mm_result[2];

        // KF Update
        pos_NEH_meas[0] = (pos_mm_result[0]-ZERO_POS[0])*RM;
        pos_NEH_meas[1] = (pos_mm_result[1]-ZERO_POS[1])*RN_COS_LON;
        //pos_NEH_meas[2] = pos_wifi[2];

        ChangeArrayMemoryLayout_float(2,N_STATES,pHk_2,(FLOAT32 *)Hk_2,1);
        ChangeArrayMemoryLayout_float(2,2,pRk_2,(FLOAT32 *)Rk_2,0);		   //Rk = diag
        if (N_STATES == 4)
        {
            Hk_2[0][0] = 1; Hk_2[0][1] = 0; Hk_2[0][2] = 0; Hk_2[0][3] = 0;
            Hk_2[1][0] = 0; Hk_2[1][1] = 1; Hk_2[1][2] = 0; Hk_2[1][3] = 0;
        }
        else if (N_STATES == 3)
        {
            Hk_2[0][0] = 1; Hk_2[0][1] = 0; Hk_2[0][2] = 0;
            Hk_2[1][0] = 0; Hk_2[1][1] = 1; Hk_2[1][2] = 0;
        }

        Zk_2[0] = pos_NEH[0] - pos_NEH_meas[0];
        Zk_2[1] = pos_NEH[1] - pos_NEH_meas[1];

        multiply_matrix_with_vector_nxn(pHk_2, 2, N_STATES, xk, Hk_2xk);       //Hx = H*x
        float meas_noise_wifi_2[2];

		if ((!IF_USE_GPS_RESULT) || (!IF_GPS_DEFEAT)) {
			meas_noise_wifi_2[0] = SIGMA_MM_RESULT;
			meas_noise_wifi_2[1] = SIGMA_MM_RESULT;
		}
		else {
			if (!if_during_defeat_wifi_time) {
				meas_noise_wifi_2[0] = SIGMA_MM_RESULT;
				meas_noise_wifi_2[1] = SIGMA_MM_RESULT;
			}
			else {
				meas_noise_wifi_2[0] = SIGMA_MM_RESULT_DEFEATED;
				meas_noise_wifi_2[1] = SIGMA_MM_RESULT_DEFEATED;
			}
		}

        diag2_sq(meas_noise_wifi_2, 2, pRk_2);

        inno_2[0] = Zk_2[0] - Hk_2xk[0];
        inno_2[1] = Zk_2[1] - Hk_2xk[1];


        // ----- New AEKF
        if (IF_USE_AEKF_MM == 0)
        {
            KF_update_indoor_float(2, xk, kf_4.pP, inno_2, pHk_2, pRk_2, N_STATES);
        }
        else if (IF_USE_AEKF_MM == 1)
        {
            AEKF_MM_2_update_indoor_float(2, xk, kf_4.pP, inno_2, pHk_2, pRk_2, N_STATES);
        }

        if_kf_updated = 1;

    }
    // ======== End of Updates from MM result ===========
    // =================================== End of KF updates ==============================================



    // ------------------------------------ KF feedback ---------------------------------------------------
    if (if_nav_initialized && if_kf_updated)
    {
        if (N_STATES == 4)
        {
            pos_NEH[0] -= xk[0];
            pos_NEH[1] -= xk[1];
            nav.heading_step -= xk[2];

            // ------------------------------- NEW ON HEADING FEEDBACK !!!!!!!!!!!! - -----------------------
            nav.heading_platform -= xk[2];
            att_pkt.att[2] = center_heading(att_pkt.att[2] - xk[2]);
            Euler2Dcm_att(att_pkt.att, att_pkt.Cbn);
            Dcm2Quat_att(att_pkt.Cbn, att_pkt.qbn);
            // =============================== NEW ON HEADING FEEDBACK !!!!!!!!!!!! =========================


            nav.sl -= xk[3];

            xk[0] = 0.0;
            xk[1] = 0.0;
            xk[2] = 0.0;
            xk[3] = 0.0;

            nav.pos[0] = pos_NEH[0] / RM + ZERO_POS[0];
            nav.pos[1] = pos_NEH[1] / RN_COS_LON + ZERO_POS[1];

			if (meas.avail_gps_result && meas.accuracy_gps_result <= TH_ACCUR_VERY_RELIABLE_GPS) {    // VERY RELIABLE GPS!
				nav.pos[0] = meas.lat_gps_result * D2R;
				nav.pos[1] = meas.lon_gps_result * D2R;
			}
        }
        else if (N_STATES == 3)
        {

            pos_NEH[0] -= xk[0];
            pos_NEH[1] -= xk[1];
            nav.heading_step -= xk[2];

            // ------------------------------- NEW ON HEADING FEEDBACK !!!!!!!!!!!! - -----------------------
            nav.heading_platform -= xk[2];
            //nav.heading_device -= xk[2];
            att_pkt.att[2] = center_heading(att_pkt.att[2] - xk[2]);
            Euler2Dcm_att(att_pkt.att, att_pkt.Cbn);
            Dcm2Quat_att(att_pkt.Cbn, att_pkt.qbn);
            // =============================== NEW ON HEADING FEEDBACK !!!!!!!!!!!! =========================

            xk[0] = 0.0;
            xk[1] = 0.0;
            xk[2] = 0.0;

            nav.pos[0] = pos_NEH[0] / RM + ZERO_POS[0];
            nav.pos[1] = pos_NEH[1] / RN_COS_LON + ZERO_POS[1];

			if (IF_USE_GPS_RESULT == 1 && meas.avail_gps_result && meas.accuracy_gps_result <= TH_ACCUR_VERY_RELIABLE_GPS) {    // VERY RELIABLE GPS!
                nav.pos[0] = meas.lat_gps_result * D2R;
                nav.pos[1] = meas.lon_gps_result * D2R;
            }
        }

    }


    if (if_update_pos)
    {
        memo.i_pos_pdr ++;
        if(memo.i_pos_pdr <= LENGTH_MEMO_POS_PDR){
            memo.t_pos_pdr[memo.i_pos_pdr-1] = t_min_va_4_step;
            memo.pos_pdr_lat[memo.i_pos_pdr-1] = nav.pos[0];
            memo.pos_pdr_lon[memo.i_pos_pdr-1] = nav.pos[1];
            memo.pos_pdr_hei[memo.i_pos_pdr-1] = nav.pos[2];
        } else {
            memo.i_pos_pdr = LENGTH_MEMO_POS_PDR;
            for (i=0; i<LENGTH_MEMO_POS_PDR-1; i++) {
                memo.t_pos_pdr[i] = memo.t_pos_pdr[i+1];
                memo.pos_pdr_lat[i] = memo.pos_pdr_lat[i+1];
                memo.pos_pdr_lon[i] = memo.pos_pdr_lon[i+1];
                memo.pos_pdr_hei[i] = memo.pos_pdr_hei[i+1];
            }
            memo.t_pos_pdr[LENGTH_MEMO_POS_PDR-1] = t_min_va_4_step;
            memo.pos_pdr_lat[LENGTH_MEMO_POS_PDR-1] = nav.pos[0];
            memo.pos_pdr_lon[LENGTH_MEMO_POS_PDR-1] = nav.pos[1];
            memo.pos_pdr_hei[LENGTH_MEMO_POS_PDR-1] = nav.pos[2];
        }


        // POS!
        //sol.count_rec_pos ++;
        sol->avail_sol_pdr = 1;
        sol->sol_t_sol_pdr = memo.t_pos_pdr[memo.i_pos_pdr-1];
        sol->sol_pdr[0] = nav.pos[0]/D2R;
        sol->sol_pdr[1] = nav.pos[1]/D2R;
        sol->sol_pdr[2] = nav.pos[2];


        if (IF_USE_GPS_RESULT == 1 && meas.avail_gps_result && meas.accuracy_gps_result <= TH_ACCUR_VERY_RELIABLE_GPS) {    // VERY RELIABLE GPS!
			sol->sol_pdr[0] = meas.lat_gps_result;
			sol->sol_pdr[1] = meas.lon_gps_result;
		}

        /*sol->sol_pdr[0] = memo.pos_pdr_lat[memo.i_pos_pdr-1]/D2R;
         sol->sol_pdr[1] = memo.pos_pdr_lon[memo.i_pos_pdr-1]/D2R;
         sol->sol_pdr[2] = memo.pos_pdr_hei[memo.i_pos_pdr-1];*/

        //fprintf(sol.f_out_rec_pos, "%.4f %.8f %.8f %.3f\n",
        //		memo.t_pos_pdr[memo.i_pos_pdr-1], memo.pos_pdr_lat[memo.i_pos_pdr-1]/D2R,
        //		memo.pos_pdr_lon[memo.i_pos_pdr-1]/D2R, memo.pos_pdr_hei[memo.i_pos_pdr-1]);

    }

    // Set as "0"
    //meas.avail_gyro = 0;
    //meas.avail_accel = 0;
    // =================================== End of KF feedback ==============================================

	sol->height0 = memo.height_baro0;
}



void real_time_put_into_memo(MemoIndoor* memo, Meas meas, Att_Pkt *att_pkt, NavIndoor *nav)
{
    static float roll1, pitch1, sr1, cr1, sp1, cp1;
    float norma1, f_b1[3], wd1, fz1, normm1, diff_max_min, Cbn_sm[3][3], m_n1[3], w_b[3], f_b[3], m_b[3], normg1, w_b_orig_axis[3];
    float temp_max_diff_heading_mag, temp_diff_heading_mag;
    static float sum_f[3], first_f[3];
    float max_min[2] = {0.0};
    UINT8 ind_qsmf1, if_real_qs;
    int i, j, k, i_eph, i_eph1;
    FLOAT32 f_b_same_vertical_x[LENGTH_MEMO_SENSOR_DATA], f_b_same_vertical_y[LENGTH_MEMO_SENSOR_DATA],f_b_same_vertical_z[LENGTH_MEMO_SENSOR_DATA];
    UINT8 i_f_b_same_vertical = 0;
    static DOUBLE64 t_qs_gyro_pre;


    if (meas.avail_accel)
    {
        memo->i_accel ++;
        if (memo->i_accel <= LENGTH_MEMO_SENSOR_DATA)
        {
            memo->t_accel[memo->i_accel-1] = meas.t_accel;
            memo->a_x[memo->i_accel-1] = meas.da_accel[0];
            memo->a_y[memo->i_accel-1] = meas.da_accel[1];
            memo->a_z[memo->i_accel-1] = meas.da_accel[2];
        }
        else
        {
            memo->i_accel = LENGTH_MEMO_SENSOR_DATA;
            for (i=0; i<LENGTH_MEMO_SENSOR_DATA-1; i++)
            {
                memo->t_accel[i] = memo->t_accel[i+1];
                memo->a_x[i] = memo->a_x[i+1];
                memo->a_y[i] = memo->a_y[i+1];
                memo->a_z[i] = memo->a_z[i+1];
            }
            memo->t_accel[LENGTH_MEMO_SENSOR_DATA-1] = meas.t_accel;
            memo->a_x[LENGTH_MEMO_SENSOR_DATA-1] = meas.da_accel[0];
            memo->a_y[LENGTH_MEMO_SENSOR_DATA-1] = meas.da_accel[1];
            memo->a_z[LENGTH_MEMO_SENSOR_DATA-1] = meas.da_accel[2];
        }

        f_b1[0] = cal_mean_float(memo->a_x, memo->i_accel);
        f_b1[1] = cal_mean_float(memo->a_y, memo->i_accel);
        f_b1[2] = cal_mean_float(memo->a_z, memo->i_accel);

        roll1 = cal_roll(f_b1);
        pitch1 = cal_pitch(f_b1);
        sr1 = sin(roll1); cr1 = cos(roll1);
        sp1 = sin(pitch1); cp1 = cos(pitch1);

        //fz1 = -sp1*meas.da_accel[0] + sr1*cp1*meas.da_accel[1] + cr1*cp1*meas.da_accel[2];
        fz1 = sqrt(meas.da_accel[0]*meas.da_accel[0] + meas.da_accel[1]*meas.da_accel[1] + meas.da_accel[2]*meas.da_accel[2]);
        memo->i_va ++;
        if (memo->i_va <= N_MEMO_VA){
            memo->va[memo->i_va-1] = fz1;
            memo->t_va[memo->i_va-1] = meas.t_accel;
        }
        else {
            memo->i_va = N_MEMO_VA;
            for (i=0; i<N_MEMO_VA-1; i++) {
                memo->va[i] = memo->va[i+1];
                memo->t_va[i] = memo->t_va[i+1];
            }
            memo->va[N_MEMO_VA-1] = fz1;
            memo->t_va[N_MEMO_VA-1] = meas.t_accel;
        }
    }


    if (meas.avail_gyro)
    {
        if(!memo->i_accel)
        {
            sr1 = 0.0; cr1 = 1.0;
            sp1 = 0.0; cp1 = 1.0;
        }
        wd1 = -sp1*meas.da_gyro[0] + sr1*cp1*meas.da_gyro[1] + cr1*cp1*meas.da_gyro[2];
        normg1 = sqrt(SQR(meas.da_gyro[0])+SQR(meas.da_gyro[1])+SQR(meas.da_gyro[2]));

        memo->i_gyro ++;
        if (memo->i_gyro <= LENGTH_MEMO_SENSOR_DATA)
        {
            memo->t_gyro[memo->i_gyro-1] = meas.t_gyro;
            memo->g_x[memo->i_gyro-1] = meas.da_gyro[0];
            memo->g_y[memo->i_gyro-1] = meas.da_gyro[1];
            memo->g_z[memo->i_gyro-1] = meas.da_gyro[2];
            memo->gz_level[memo->i_gyro-1] = wd1;
            memo->norm_gyro[memo->i_gyro-1] = normg1;
            //memo->memo_if_vertical_gyro[memo->i_gyro-1] = att_pkt->if_vertical;
        }
        else
        {
            memo->i_gyro = LENGTH_MEMO_SENSOR_DATA;
            for (i=0; i<LENGTH_MEMO_SENSOR_DATA-1; i++)
            {
                memo->t_gyro[i] = memo->t_gyro[i+1];
                memo->g_x[i] = memo->g_x[i+1];
                memo->g_y[i] = memo->g_y[i+1];
                memo->g_z[i] = memo->g_z[i+1];
                memo->gz_level[i] = memo->gz_level[i+1];
                memo->norm_gyro[i] = memo->norm_gyro[i+1];
                //memo->memo_if_vertical_gyro[i] = memo->memo_if_vertical_gyro[i+1];
            }
            memo->t_gyro[LENGTH_MEMO_SENSOR_DATA-1] = meas.t_gyro;
            memo->g_x[LENGTH_MEMO_SENSOR_DATA-1] = meas.da_gyro[0];
            memo->g_y[LENGTH_MEMO_SENSOR_DATA-1] = meas.da_gyro[1];
            memo->g_z[LENGTH_MEMO_SENSOR_DATA-1] = meas.da_gyro[2];
            memo->gz_level[LENGTH_MEMO_SENSOR_DATA-1] = wd1;
            memo->norm_gyro[LENGTH_MEMO_SENSOR_DATA-1] = normg1;
            //memo->memo_if_vertical_gyro[LENGTH_MEMO_SENSOR_DATA-1] = att_pkt->if_vertical;
        }


        // ------------------------------- Real QS ------------------------------------
        find_max_min_float_mis(memo->norm_gyro, max_min, memo->i_gyro);
        diff_max_min = max_min[0] - max_min[1];
        if_real_qs = 0;
        if (diff_max_min < TH_DETECT_QS_GYRO_NORM_5_DEG_S && memo->i_gyro == LENGTH_MEMO_SENSOR_DATA)
        {
            if_real_qs = 1;
        }
        if (if_real_qs == 1 )    // Real_QS
        {
            if (att_pkt->if_vertical == 0) {
                w_b_orig_axis[0] = meas.da_gyro[0];
                w_b_orig_axis[1] = meas.da_gyro[1];
                w_b_orig_axis[2] = meas.da_gyro[2];
            }
            else if (att_pkt->if_vertical == 1)	{
                w_b_orig_axis[0] = -meas.da_gyro[2];
                w_b_orig_axis[1] = meas.da_gyro[1];
                w_b_orig_axis[2] = meas.da_gyro[0];
            }
            else if (att_pkt->if_vertical == -1)   {
                w_b_orig_axis[0] = meas.da_gyro[2];
                w_b_orig_axis[1] = meas.da_gyro[1];
                w_b_orig_axis[2] = -meas.da_gyro[0];
            }

            memo->i_memo_norm_gyro_QS ++;
            if (memo->i_memo_norm_gyro_QS <= N_EPH_MEMO_GYRO_REAL_QS)
            {
                memo->norm_gyro_QS_x[memo->i_memo_norm_gyro_QS-1] = w_b_orig_axis[0];
                memo->norm_gyro_QS_y[memo->i_memo_norm_gyro_QS-1] = w_b_orig_axis[1];
                memo->norm_gyro_QS_z[memo->i_memo_norm_gyro_QS-1] = w_b_orig_axis[2];
            }
            else
            {
                memo->i_memo_norm_gyro_QS = N_EPH_MEMO_GYRO_REAL_QS;
                for (i=0; i<N_EPH_MEMO_GYRO_REAL_QS-1; i++)
                {
                    memo->norm_gyro_QS_x[i] = memo->norm_gyro_QS_x[i+1];
                    memo->norm_gyro_QS_y[i] = memo->norm_gyro_QS_y[i+1];
                    memo->norm_gyro_QS_z[i] = memo->norm_gyro_QS_z[i+1];
                }
                memo->norm_gyro_QS_x[N_EPH_MEMO_GYRO_REAL_QS-1] = w_b_orig_axis[0];
                memo->norm_gyro_QS_y[N_EPH_MEMO_GYRO_REAL_QS-1] = w_b_orig_axis[1];
                memo->norm_gyro_QS_z[N_EPH_MEMO_GYRO_REAL_QS-1] = w_b_orig_axis[2];
            }
        }
        // ============================= Real QS ================================
    }



    if (meas.avail_mag)
    {
        if (!memo->i_accel)
        {
            roll1 = 0.0; pitch1 = 0.0;
        }

        // ------------ Calculate heading_mag
        Euler2Dcm_att_float_float_3_3_mis(roll1, pitch1, 0.0, Cbn_sm);
        for(j = 0; j < 3; j++)
        {
            m_n1[j] = 0.0;
            for(k = 0; k < 3; k++)	// bringing in the accel into the level plane
            {
                m_n1[j] += Cbn_sm[j][k]*meas.da_mag[k];
            }
        }
        nav->declination = nav->declination;
        nav->heading_mag = -atan2(m_n1[1], m_n1[0]) + nav->declination;   //+

        memo->i_memo_heading_mag ++;
        if (memo->i_memo_heading_mag <= N_EPH_HEADING_MAG)
        {
            memo->heading_mag[memo->i_memo_heading_mag-1] = nav->heading_mag;
        }
        else
        {
            memo->i_memo_heading_mag = N_EPH_HEADING_MAG;
            for (i=0; i<N_EPH_HEADING_MAG-1; i++)
            {
                memo->heading_mag[i] = memo->heading_mag[i+1];
            }
            memo->heading_mag[N_EPH_HEADING_MAG-1] = nav->heading_mag;
        }

        //nav->heading_mag = cal_mean_float(memo->heading_mag, memo->i_memo_heading_mag );  // Use Mean heading!!


        //
        memo->i_mag ++;
        if (memo->i_mag <= N_EPH_MAG_DATA)
        {
            memo->t_mag[memo->i_mag-1] = meas.t_mag;
            memo->m_x[memo->i_mag-1] = meas.da_mag[0];
            memo->m_y[memo->i_mag-1] = meas.da_mag[1];
            memo->m_z[memo->i_mag-1] = meas.da_mag[2];
            //memo->if_vertical_mag[memo->i_mag-1] = att_pkt->if_vertical;

        }
        else
        {
            memo->i_mag = N_EPH_MAG_DATA;
            for (i=0; i<N_EPH_MAG_DATA-1; i++)
            {
                memo->t_mag[i] = memo->t_mag[i+1];
                memo->m_x[i] = memo->m_x[i+1];
                memo->m_y[i] = memo->m_y[i+1];
                memo->m_z[i] = memo->m_z[i+1];
                //memo->if_vertical_mag[i] = memo->if_vertical_mag[i+1];
            }
            memo->t_mag[N_EPH_MAG_DATA-1] = meas.t_mag;
            memo->m_x[N_EPH_MAG_DATA-1] = meas.da_mag[0];
            memo->m_y[N_EPH_MAG_DATA-1] = meas.da_mag[1];
            memo->m_z[N_EPH_MAG_DATA-1] = meas.da_mag[2];
            //memo->if_vertical_mag[N_EPH_MAG_DATA-1] = att_pkt->if_vertical;
        }

        normm1 = sqrt(meas.da_mag[0]*meas.da_mag[0] + meas.da_mag[1]*meas.da_mag[1] + meas.da_mag[2]*meas.da_mag[2]);

        memo->i_memo_normm ++;
        if (memo->i_memo_normm <= N_EPH_MEMO_NORM)
        {
            memo->normm[memo->i_memo_normm-1] = normm1;
        }
        else
        {
            memo->i_memo_normm = N_EPH_MEMO_NORM;
            for (i=0; i<N_EPH_MEMO_NORM-1; i++)
            {
                memo->normm[i] = memo->normm[i+1];
            }
            memo->normm[N_EPH_MEMO_NORM-1] = normm1;
        }

        // ------------ QSMF based on norm_m -----------------
        /*if (memo->i_memo_normm == N_EPH_MEMO_NORM)
         {
         find_max_min_float(memo->normm, max_min, memo->i_memo_normm);
         diff_max_min = max_min[0] - max_min[1];

         ind_qsmf1 = 0;
         if (diff_max_min < TH_DETECT_QSMF)
         {
         ind_qsmf1 = 1;
         }

         memo->i_memo_indi_qsmf ++;
         if (memo->i_memo_indi_qsmf <= N_EPH_MEMO_IND_QSMF)
         {
         memo->ind_qsmf[memo->i_memo_indi_qsmf-1] = ind_qsmf1;
         } else
         {
         memo->i_memo_indi_qsmf = N_EPH_MEMO_IND_QSMF;
         for (i=0; i<N_EPH_MEMO_IND_QSMF-1; i++)
         {
         memo->ind_qsmf[i] = memo->ind_qsmf[i+1];
         }
         memo->ind_qsmf[N_EPH_MEMO_IND_QSMF-1] = ind_qsmf1;
         }
         }*/
        // ============ QSMF based on norm_m =================

        // ------------ QSMF based on heading ----------------
        if (memo->i_memo_heading_mag == N_EPH_HEADING_MAG)
        {
            temp_max_diff_heading_mag = 0.0;
            for (i=1; i<memo->i_memo_heading_mag; i++)
            {
                temp_diff_heading_mag = center_heading(memo->heading_mag[i-1]-memo->heading_mag[0]);
                temp_diff_heading_mag = fabs(temp_diff_heading_mag);
                if (temp_diff_heading_mag > temp_max_diff_heading_mag)
                {
                    temp_max_diff_heading_mag = temp_diff_heading_mag;
                }
            }

            ind_qsmf1 = 0;
            if (temp_max_diff_heading_mag < TH_DETECT_QSMF_HEADING_MAG)
            {
                ind_qsmf1 = 1;
            }

            memo->i_memo_indi_qsmf ++;
            if (memo->i_memo_indi_qsmf <= N_EPH_MEMO_IND_QSMF)
            {
                memo->ind_qsmf[memo->i_memo_indi_qsmf-1] = ind_qsmf1;
            } else
            {
                memo->i_memo_indi_qsmf = N_EPH_MEMO_IND_QSMF;
                for (i=0; i<N_EPH_MEMO_IND_QSMF-1; i++)
                {
                    memo->ind_qsmf[i] = memo->ind_qsmf[i+1];
                }
                memo->ind_qsmf[N_EPH_MEMO_IND_QSMF-1] = ind_qsmf1;
            }
        }
        // ============ QSMF based on heading ================

    }

     if (meas.avail_baro){
		memo->height_pre = meas.da_baro;
	 }

    /*if (meas.avail_baro)
     {
     memo->i_baro ++;
     if (memo->i_baro <= N_EPH_SMOOTH_BARO)
     {
     memo->t_baro[memo->i_baro-1] = meas.t_baro;
     memo->baro[memo->i_baro-1] = meas.da_baro;
     }
     else
     {
     memo->i_baro = N_EPH_SMOOTH_BARO;
     for (i=0; i<N_EPH_SMOOTH_BARO-1; i++)
     {
     memo->t_baro[i] = memo->t_baro[i+1];
     memo->baro[i] = memo->baro[i+1];
     }
     memo->t_baro[N_EPH_SMOOTH_BARO-1] = meas.t_baro;
     memo->baro[N_EPH_SMOOTH_BARO-1] = meas.da_baro;
     }
     }*/
}



//void real_time_put_into_memo(MemoIndoor* memo, Meas meas) {
//	static float roll1, pitch1, sr1, cr1, sp1, cp1;
//	float norma1, f_b1[3], wd1, fz1;
//	static float sum_f[3], first_f[3];
//	int i;
//	if (meas.avail_accel){
//		/*if (!memo->i_accel) {
//			sum_f[0] = 0.0;
//			sum_f[1] = 0.0;
//			sum_f[2] = 0.0;
//		}*/
//
//
//		memo->i_accel ++;
//		if (memo->i_accel <= LENGTH_MEMO_SENSOR_DATA) {
//			/*sum_f[0] += meas.da_accel[0];
//			sum_f[1] += meas.da_accel[1];
//			sum_f[2] += meas.da_accel[2];
//
//			f_b1[0] = sum_f[0] / memo->i_accel;
//			f_b1[1] = sum_f[1] / memo->i_accel;
//			f_b1[2] = sum_f[2] / memo->i_accel;*/
//
//			memo->t_accel[memo->i_accel-1] = meas.t_accel;
//			memo->a_x[memo->i_accel-1] = meas.da_accel[0];
//			memo->a_y[memo->i_accel-1] = meas.da_accel[1];
//			memo->a_z[memo->i_accel-1] = meas.da_accel[2];
//		}
//		else {
//
//			/*sum_f[0] += meas.da_accel[0] - memo->a_x[0];
//			sum_f[1] += meas.da_accel[1] - memo->a_y[0];
//			sum_f[2] += meas.da_accel[2] - memo->a_z[0];
//
//			f_b1[0] = sum_f[0] / memo->i_accel;
//			f_b1[1] = sum_f[1] / memo->i_accel;
//			f_b1[2] = sum_f[2] / memo->i_accel;
//
//
//			f_b1[0] = cal_mean_float(memo->a_x, memo->i_accel);
//			f_b1[1] = cal_mean_float(memo->a_y, memo->i_accel);
//			f_b1[2] = cal_mean_float(memo->a_z, memo->i_accel);*/
//
//
//			memo->i_accel = LENGTH_MEMO_SENSOR_DATA;
//			for (i=0; i<LENGTH_MEMO_SENSOR_DATA-1; i++) {
//				memo->t_accel[i] = memo->t_accel[i+1];
//				memo->a_x[i] = memo->a_x[i+1];
//				memo->a_y[i] = memo->a_y[i+1];
//				memo->a_z[i] = memo->a_z[i+1];
//			}
//			memo->t_accel[LENGTH_MEMO_SENSOR_DATA-1] = meas.t_accel;
//			memo->a_x[LENGTH_MEMO_SENSOR_DATA-1] = meas.da_accel[0];
//			memo->a_y[LENGTH_MEMO_SENSOR_DATA-1] = meas.da_accel[1];
//			memo->a_z[LENGTH_MEMO_SENSOR_DATA-1] = meas.da_accel[2];
//		}
//		//if (LENGTH_MEMO_SENSOR_DATA > N_EPH_SMOOTH_ROLL_PITCH)
//		//{
//			//if (memo->i_accel > n_eph_smooth_roll_pitch)    // smooth roll and pitch
//			//{
//			//	f_b1[0] = cal_mean_float(memo->a_x+memo->i_accel-n_eph_smooth_roll_pitch, n_eph_smooth_roll_pitch);
//			//	f_b1[1] = cal_mean_float(memo->a_y+memo->i_accel-n_eph_smooth_roll_pitch, n_eph_smooth_roll_pitch);
//			//	f_b1[2] = cal_mean_float(memo->a_z+memo->i_accel-n_eph_smooth_roll_pitch, n_eph_smooth_roll_pitch);
//			//}
//			//else
//			//{
//			//	f_b1[0] = cal_mean_float(memo->a_x, memo->i_accel);
//			//	f_b1[1] = cal_mean_float(memo->a_y, memo->i_accel);
//			//	f_b1[2] = cal_mean_float(memo->a_z, memo->i_accel);
//			//}
//		//}
//		//else {
//
//			f_b1[0] = cal_mean_float(memo->a_x, memo->i_accel);
//			f_b1[1] = cal_mean_float(memo->a_y, memo->i_accel);
//			f_b1[2] = cal_mean_float(memo->a_z, memo->i_accel);
//		//}
//
//		roll1 = cal_roll(f_b1);
//		pitch1 = cal_pitch(f_b1);
//		sr1 = sin(roll1); cr1 = cos(roll1);
//		sp1 = sin(pitch1); cp1 = cos(pitch1);
//
//
//		/*norma1 = sqrt(meas.da_accel[0]*meas.da_accel[0]+meas.da_accel[1]*meas.da_accel[1]+meas.da_accel[2]*meas.da_accel[2]);
//		memo->i_ma ++;
//		if(memo->i_ma <= LENGTH_MEMO_MA){
//		memo->ma[memo->i_ma-1] = norma1;
//		}
//		else {
//		memo->i_ma = LENGTH_MEMO_MA;
//		for (i=0; i<LENGTH_MEMO_MA-1; i++) {
//		memo->ma[i] = memo->ma[i+1];
//		}
//		memo->ma[LENGTH_MEMO_MA-1] = norma1;
//		}*/
//
//		fz1 = -sp1*meas.da_accel[0] + sr1*cp1*meas.da_accel[1] + cr1*cp1*meas.da_accel[2];
//		memo->i_va ++;
//		if (memo->i_va <= N_MEMO_VA){
//			memo->va[memo->i_va-1] = fz1;
//			memo->t_va[memo->i_va-1] = meas.t_accel;
//		}
//		else {
//			memo->i_va = N_MEMO_VA;
//			for (i=0; i<N_MEMO_VA-1; i++) {
//				memo->va[i] = memo->va[i+1];
//				memo->t_va[i] = memo->t_va[i+1];
//			}
//			memo->va[N_MEMO_VA-1] = fz1;
//			memo->t_va[N_MEMO_VA-1] = meas.t_accel;
//		}
//	}
//
//
//	if (meas.avail_gyro) {
//		if(!memo->i_accel) {
//			sr1 = 0.0; cr1 = 1.0;
//			sp1 = 0.0; cp1 = 1.0;
//		}
//		wd1 = -sp1*meas.da_gyro[0] + sr1*cp1*meas.da_gyro[1] + cr1*cp1*meas.da_gyro[2];
//
//
//		memo->i_gyro ++;
//		if (memo->i_gyro <= LENGTH_MEMO_SENSOR_DATA) {
//			memo->t_gyro[memo->i_gyro-1] = meas.t_gyro;
//			memo->g_x[memo->i_gyro-1] = meas.da_gyro[0];
//			memo->g_y[memo->i_gyro-1] = meas.da_gyro[1];
//			memo->g_z[memo->i_gyro-1] = meas.da_gyro[2];
//			memo->gz_level[memo->i_gyro-1] = wd1;
//		}
//		else {
//			memo->i_gyro = LENGTH_MEMO_SENSOR_DATA;
//			for (i=0; i<LENGTH_MEMO_SENSOR_DATA-1; i++) {
//				memo->t_gyro[i] = memo->t_gyro[i+1];
//				memo->g_x[i] = memo->g_x[i+1];
//				memo->g_y[i] = memo->g_y[i+1];
//				memo->g_z[i] = memo->g_z[i+1];
//				memo->gz_level[i] = memo->gz_level[i+1];
//			}
//			memo->t_gyro[LENGTH_MEMO_SENSOR_DATA-1] = meas.t_gyro;
//			memo->g_x[LENGTH_MEMO_SENSOR_DATA-1] = meas.da_gyro[0];
//			memo->g_y[LENGTH_MEMO_SENSOR_DATA-1] = meas.da_gyro[1];
//			memo->g_z[LENGTH_MEMO_SENSOR_DATA-1] = meas.da_gyro[2];
//			memo->gz_level[LENGTH_MEMO_SENSOR_DATA-1] = wd1;
//		}
//
//	}
//
//	if (meas.avail_mag)
//	{
//		memo->i_gyro ++;
//		if (memo->i_gyro <= LENGTH_MEMO_SENSOR_DATA)
//		{
//			memo->t_gyro[memo->i_gyro-1] = meas.t_gyro;
//			memo->g_x[memo->i_gyro-1] = meas.da_gyro[0];
//			memo->g_y[memo->i_gyro-1] = meas.da_gyro[1];
//			memo->g_z[memo->i_gyro-1] = meas.da_gyro[2];
//			memo->gz_level[memo->i_gyro-1] = wd1;
//		}
//		else
//		{
//			memo->i_gyro = LENGTH_MEMO_SENSOR_DATA;
//			for (i=0; i<LENGTH_MEMO_SENSOR_DATA-1; i++)
//			{
//				memo->t_gyro[i] = memo->t_gyro[i+1];
//				memo->g_x[i] = memo->g_x[i+1];
//				memo->g_y[i] = memo->g_y[i+1];
//				memo->g_z[i] = memo->g_z[i+1];
//				memo->gz_level[i] = memo->gz_level[i+1];
//			}
//			memo->t_gyro[LENGTH_MEMO_SENSOR_DATA-1] = meas.t_gyro;
//			memo->g_x[LENGTH_MEMO_SENSOR_DATA-1] = meas.da_gyro[0];
//			memo->g_y[LENGTH_MEMO_SENSOR_DATA-1] = meas.da_gyro[1];
//			memo->g_z[LENGTH_MEMO_SENSOR_DATA-1] = meas.da_gyro[2];
//			memo->gz_level[LENGTH_MEMO_SENSOR_DATA-1] = wd1;
//		}
//
//	}
//}


void wifi_position_tracker_smooth_int(long int mac_ap_int[MAX_AP_NUM], int rss_ap[MAX_AP_NUM], int N, int floor[1], double pos[3], int avail_wifi[1]){
    static int first_time_flag = 0;
    static long int DB_mac[MAX_AP_NUM] = {0};
    static int DB_matrix[MAX_RP_NUM][MAX_AP_NUM];
    static double DB_pos[MAX_RP_NUM][3];
    static int DB_size[2];

    // -------------- 4 DBs -------------- 20151007
//	static long int DB_mac_2[MAX_AP_NUM] = {0};
//	static int DB_matrix_2[MAX_RP_NUM][MAX_AP_NUM];
//	static double DB_pos_2[MAX_RP_NUM][3];
//	static int DB_size_2[2];
//
//	static long int DB_mac_3[MAX_AP_NUM] = {0};
//	static int DB_matrix_3[MAX_RP_NUM][MAX_AP_NUM];
//	static double DB_pos_3[MAX_RP_NUM][3];
//	static int DB_size_3[2];
//
//	static long int DB_mac_4[MAX_AP_NUM] = {0};
//	static int DB_matrix_4[MAX_RP_NUM][MAX_AP_NUM];
//	static double DB_pos_4[MAX_RP_NUM][3];
//	static int DB_size_4[2];
	// ============== 4 DBs ===============


    static MemoIndoorWiFi memo;
    static int num_mac_diff = 0;

    double knn[N_KNN][4];
    float dss[MAX_RP_NUM];
    float min_dss[2] = {0.0};
    double w, sum_w, mean_lat_temp, mean_lon_temp, dist_temp;
    long int mac_ap = 0;
    int da_rss[MAX_AP_NUM], sum_rss_col, da_rss_before_diff[MAX_AP_NUM/10];
    int avail_wifi_temp = 0;
    int if_wifi_has_initialized = 1;
    double lat_temp, lon_temp;
    int i,j,i_min_dss, count_knn;

    float sum_rss_ap = 0.0;

    avail_wifi[0] = 0;

    // The detection of same RSS
    for (i=0; i<N; i++) {
        sum_rss_ap += rss_ap[i];
    }

    int if_same_rss_with_pre = 0;
    if (first_time_flag && N == memo.N_wifi_pre){
        if (fabs(sum_rss_ap-memo.sum_rss_ap_pre) < 10e-4){
            if_same_rss_with_pre = 1;
        }
    }
    memo.N_wifi_pre = N;
    memo.sum_rss_ap_pre = sum_rss_ap;

    //
    if (!first_time_flag){
        first_time_flag = 1;

        // Initial memo wifi
        memo.i_rss = 0;
        memo.i_memo_wifi = 0;
        for (i=0; i<LENGTH_MEMO_RSS; i++) {
            for (j=0; j<MAX_AP_NUM; j++) {
                memo.rss[i][j] = 0;
            }
        }
        for (i=0; i<N_MEMO_WIFI; i++) {
            memo.pos_wifi_lat[i] = 0.00;
            memo.pos_wifi_lon[i] = 0.00;
        }

        // End initial memo wifi

        if (IF_DIFF_WIFI == 1)
        {
            if (IF_CODE_WIFI == 0) {
                load_DB_int_bin_diff(f_DB_bin_diff_str, DB_mac, DB_matrix, DB_pos, DB_size);
            }
            else if (IF_CODE_WIFI == 1)
            {
                load_DB_int_bin_diff_co(f_DB_bin_diff_str, DB_mac, DB_matrix, DB_pos, DB_size);
            }
            num_mac_diff = DB_size[1] * DB_size[1];
        }
        else
        {
            if (1){//(IF_CODE_WIFI == 0) {
                //load_DB_int_bin(f_DB_bin_str, DB_mac, DB_matrix, DB_pos, DB_size);
                if (NUM_WIFI_DB == 1) {
					load_DB_int_bin(f_DB_bin_str, DB_mac, DB_matrix, DB_pos, DB_size);
				}
				// -------------- 4 DBs -------------- 20151007
//				else if (NUM_WIFI_DB == 4)
//				{
//					load_DB_int_bin(f_DB_bin_str, DB_mac, DB_matrix, DB_pos, DB_size);
//					load_DB_int_bin(f_DB_bin_str_2, DB_mac_2, DB_matrix_2, DB_pos_2, DB_size_2);
//					load_DB_int_bin(f_DB_bin_str_3, DB_mac_3, DB_matrix_3, DB_pos_3, DB_size_3);
//					load_DB_int_bin(f_DB_bin_str_4, DB_mac_4, DB_matrix_4, DB_pos_4, DB_size_4);
//				}
            }
            else if (IF_CODE_WIFI == 1)
            {
                load_DB_int_bin_co(f_DB_bin_str, DB_mac, DB_matrix, DB_pos, DB_size);
            }
            num_mac_diff = DB_size[1];
        }
    }


    if (if_same_rss_with_pre || N<3) {		// Less than 3 APs
        avail_wifi[0] = 0;
        lat_temp = INI_POS[0];
        lon_temp = INI_POS[1];
    }
    else {
        if (IF_DIFF_WIFI == 0)
        {
            for (i=0; i<MAX_AP_NUM; i++)
            {
                da_rss[i] = 100;
            }
        }
        else if (IF_DIFF_WIFI == 1)
        {
            for (i=0; i<MAX_AP_NUM/10; i++)
            {
                da_rss_before_diff[i] = -100;
            }
            for (i=0; i<MAX_AP_NUM; i++)
            {
                da_rss[i] = 0;
            }
        }

        // Fill the da_rss vector
        for (i=0; i<N; i++) {
            // Convert MAC
            //mac_ap = mac2int(name_ap[i]);
            mac_ap = mac_ap_int[i];

			// ------------------------- 4 DBs ------------------------- 20151007
			if (1){//(NUM_WIFI_DB == 1 || (NUM_WIFI_DB == 4 && floor[0] == 1)) {
				for (j=0; j<DB_size[1]; j++) {
					if (mac_ap == DB_mac[j]) {
						if (IF_DIFF_WIFI == 0) {
							//da_rss[j] = -(int)(rss_ap[i]);
							da_rss[j] = rss_ap[i];
						}
						else if (IF_DIFF_WIFI == 1) {
							da_rss_before_diff[j] = -(int)(rss_ap[i]);     // NEEDS TO CHANGE LATER! BECAUSE OF THE
						}

						break;
					}
				}
			}
//			else if(NUM_WIFI_DB == 4 && floor[0] != 1) {
//				if (floor[0] == 2)
//				{
//					for (j=0; j<DB_size_2[1]; j++) {
//						if (mac_ap == DB_mac_2[j]) {
//							if (IF_DIFF_WIFI == 0) {
//								//da_rss[j] = -(int)(rss_ap[i]);
//								da_rss[j] = rss_ap[i];
//							}
//							else if (IF_DIFF_WIFI == 1) {
//								da_rss_before_diff[j] = -(int)(rss_ap[i]);     // NEEDS TO CHANGE LATER! BECAUSE OF THE
//							}
//
//							break;
//						}
//					}
//				}
//				else if (floor[0] == 3)
//				{
//					for (j=0; j<DB_size_3[1]; j++) {
//						if (mac_ap == DB_mac_3[j]) {
//							if (IF_DIFF_WIFI == 0) {
//								//da_rss[j] = -(int)(rss_ap[i]);
//								da_rss[j] = rss_ap[i];
//							}
//							else if (IF_DIFF_WIFI == 1) {
//								da_rss_before_diff[j] = -(int)(rss_ap[i]);     // NEEDS TO CHANGE LATER! BECAUSE OF THE
//							}
//
//							break;
//						}
//					}
//				}
//				else if (floor[0] == 4)
//				{
//					for (j=0; j<DB_size_4[1]; j++) {
//						if (mac_ap == DB_mac_4[j]) {
//							if (IF_DIFF_WIFI == 0) {
//								//da_rss[j] = -(int)(rss_ap[i]);
//								da_rss[j] = rss_ap[i];
//							}
//							else if (IF_DIFF_WIFI == 1) {
//								da_rss_before_diff[j] = -(int)(rss_ap[i]);     // NEEDS TO CHANGE LATER! BECAUSE OF THE
//							}
//
//							break;
//						}
//					}
//				}
//			}
			// ============================ 4 DBs =================================
//
//            for (j=0; j<DB_size[1]; j++) {
//                if (mac_ap == DB_mac[j]) {
//                    if (IF_DIFF_WIFI == 0) {
//                        //da_rss[j] = -(int)(rss_ap[i]);
//                        da_rss[j] = rss_ap[i];
//                    }
//                    else if (IF_DIFF_WIFI == 1) {
//                        da_rss_before_diff[j] = -(int)(rss_ap[i]);     // NEEDS TO CHANGE LATER! BECAUSE OF THE
//                    }
//
//                    break;
//                }
//            }

        }

        if (IF_DIFF_WIFI == 1)
        {
            for (i=0; i<DB_size[1]; i++)
            {
                for (j=0; j<DB_size[1]; j++)
                {
                    da_rss[i*DB_size[1]+j] = da_rss_before_diff[i] - da_rss_before_diff[j];
                }
            }
        }


        // NEW Averaging rss
        memo.i_rss ++;
        if (LENGTH_MEMO_RSS == 1) {
            memo.i_rss = LENGTH_MEMO_RSS;
            // Do nothing
        }
        else {
            if (memo.i_rss <= LENGTH_MEMO_RSS) {
                for (j=0; j<MAX_AP_NUM; j++) {
                    memo.rss[memo.i_rss-1][j] = da_rss[j];
                }
            }
            else {
                memo.i_rss = LENGTH_MEMO_RSS;
                for (i=0; i<LENGTH_MEMO_RSS-1; i++) {
                    for (j=0; j<MAX_AP_NUM; j++) {
                        memo.rss[i][j] = memo.rss[i+1][j];
                    }
                }
                for (j=0; j<MAX_AP_NUM; j++) {
                    memo.rss[LENGTH_MEMO_RSS-1][j] = da_rss[j];
                }
            }

            // Mean
            for (j=0; j<MAX_AP_NUM; j++) {
                sum_rss_col = 0;
                for (i=0; i<memo.i_rss; i++) {
                    sum_rss_col += memo.rss[i][j];
                }
                da_rss[j] = (int)(sum_rss_col*1.0/memo.i_rss);
            }
        }
        // End Averaging rss



		// ------------------------- 4 DBs ------------------------- 20151007
		if (1){//(NUM_WIFI_DB == 1 || (NUM_WIFI_DB == 4 && floor[0] == 1)) {
			for(i=0; i<DB_size[0]; i++) {
				dss[i] = cal_dss_int(da_rss, DB_matrix[i], DB_size[1]);
			}
		}
//		else if(NUM_WIFI_DB == 4 && floor[0] != 1) {
//			if (floor[0] == 2)
//			{
//				for(i=0; i<DB_size_2[0]; i++) {
//					dss[i] = cal_dss_int(da_rss, DB_matrix_2[i], DB_size_2[1]);
//				}
//			}
//			else if (floor[0] == 3)
//			{
//				for(i=0; i<DB_size_3[0]; i++) {
//					dss[i] = cal_dss_int(da_rss, DB_matrix_3[i], DB_size_3[1]);
//				}
//			}
//			else if (floor[0] == 4)
//			{
//				for(i=0; i<DB_size_4[0]; i++) {
//					dss[i] = cal_dss_int(da_rss, DB_matrix_4[i], DB_size_4[1]);
//				}
//			}
//		}
		// ============================ 4 DBs =================================
//        for(i=0; i<DB_size[0]; i++) {
//            dss[i] = cal_dss_int(da_rss, DB_matrix[i], DB_size[1]);
//        }


        // Find KNN
        mean_lat_temp = 0.00;
        mean_lon_temp = 0.00;

		// ------------------------- 4 DBs ------------------------- 20151007
		if (1){//(NUM_WIFI_DB == 1 || (NUM_WIFI_DB == 4 && floor[0] == 1)) {
			for(i=0; i<N_KNN; i++) {
				min_float(dss, min_dss, DB_size[0]);
				knn[i][0] = min_dss[1];
				i_min_dss = (int)(knn[i][0]);
				knn[i][1] = min_dss[0];
				knn[i][2] = DB_pos[i_min_dss][0];
				knn[i][3] = DB_pos[i_min_dss][1];
				dss[i_min_dss] = 999999.0;

				mean_lat_temp += knn[i][2];
				mean_lon_temp += knn[i][3];
			}
		}
//		else if(NUM_WIFI_DB == 4 && floor[0] != 1) {
//			if (floor[0] == 2)
//			{
//				for(i=0; i<N_KNN; i++) {
//					min_float(dss, min_dss, DB_size_2[0]);
//					knn[i][0] = min_dss[1];
//					i_min_dss = (int)(knn[i][0]);
//					knn[i][1] = min_dss[0];
//					knn[i][2] = DB_pos_2[i_min_dss][0];
//					knn[i][3] = DB_pos_2[i_min_dss][1];
//					dss[i_min_dss] = 999999.0;
//
//					mean_lat_temp += knn[i][2];
//					mean_lon_temp += knn[i][3];
//				}
//			}
//			else if (floor[0] == 3)
//			{
//				for(i=0; i<N_KNN; i++) {
//					min_float(dss, min_dss, DB_size_3[0]);
//					knn[i][0] = min_dss[1];
//					i_min_dss = (int)(knn[i][0]);
//					knn[i][1] = min_dss[0];
//					knn[i][2] = DB_pos_3[i_min_dss][0];
//					knn[i][3] = DB_pos_3[i_min_dss][1];
//					dss[i_min_dss] = 999999.0;
//
//					mean_lat_temp += knn[i][2];
//					mean_lon_temp += knn[i][3];
//				}
//			}
//			else if (floor[0] == 4)
//			{
//				for(i=0; i<N_KNN; i++) {
//					min_float(dss, min_dss, DB_size_4[0]);
//					knn[i][0] = min_dss[1];
//					i_min_dss = (int)(knn[i][0]);
//					knn[i][1] = min_dss[0];
//					knn[i][2] = DB_pos_4[i_min_dss][0];
//					knn[i][3] = DB_pos_4[i_min_dss][1];
//					dss[i_min_dss] = 999999.0;
//
//					mean_lat_temp += knn[i][2];
//					mean_lon_temp += knn[i][3];
//				}
//			}
//		}
		// ============================ 4 DBs ================================

//        for(i=0; i<N_KNN; i++) {
//            min_float(dss, min_dss, DB_size[0]);
//            knn[i][0] = min_dss[1];
//            i_min_dss = (int)(knn[i][0]);
//            knn[i][1] = min_dss[0];
//            knn[i][2] = DB_pos[i_min_dss][0];
//            knn[i][3] = DB_pos[i_min_dss][1];
//            dss[i_min_dss] = 999999.0;
//
//            mean_lat_temp += knn[i][2];
//            mean_lon_temp += knn[i][3];
//        }


        mean_lat_temp /= N_KNN;   // In matlab, it is mean_pos
        mean_lon_temp /= N_KNN;

        // KNN
        // Reject RSS which is far from others
        sum_w = 0.00;
        count_knn = 0;
        lat_temp = 0.00;
        lon_temp = 0.00;
        for (i=0; i<N_KNN; i++) {
            dist_temp = cal_dist_2D(mean_lat_temp, mean_lon_temp, knn[i][2], knn[i][3]);

            if (dist_temp <= TH_DIST_REJECT_NN) {
                if (knn[i][1]<1.0) {
                    knn[i][1] = 1.0;
                }

                w = 1/knn[i][1];
                sum_w += w;
                lat_temp += w*knn[i][2];
                lon_temp += w*knn[i][3];

                count_knn ++;
            }
        }

        if (count_knn) {
            avail_wifi_temp = 1;
            lat_temp /= sum_w;
            lon_temp /= sum_w;
        }

        //
        if(!memo.i_memo_wifi && !avail_wifi_temp) {
            if_wifi_has_initialized = 0;
        }

        if (if_wifi_has_initialized) {
            if (!avail_wifi_temp && memo.i_memo_wifi>=1) {
                lat_temp = memo.pos_wifi_lat[memo.i_memo_wifi-1];
                lon_temp = memo.pos_wifi_lon[memo.i_memo_wifi-1];
            }

            if (avail_wifi_temp) {
                if (memo.i_memo_wifi < N_MEMO_WIFI) {  // HERE IS A "<". To be determined in the future
                    memo.i_memo_wifi ++;
                    memo.pos_wifi_lat[memo.i_memo_wifi-1] = lat_temp;
                    memo.pos_wifi_lon[memo.i_memo_wifi-1] = lon_temp;
                }
                else {
                    memo.i_memo_wifi = N_MEMO_WIFI;
                    for (i=0; i<N_MEMO_WIFI-1; i++) {
                        memo.pos_wifi_lat[i] = memo.pos_wifi_lat[i+1];
                        memo.pos_wifi_lon[i] = memo.pos_wifi_lon[i+1];
                    }
                    memo.pos_wifi_lat[N_MEMO_WIFI-1] = lat_temp;
                    memo.pos_wifi_lon[N_MEMO_WIFI-1] = lon_temp;
                }
            }
        }

        // Now the WiFi position is outputed only when memo.i_memo_wifi == N_MEMO_WIFI
        if (memo.i_memo_wifi == N_MEMO_WIFI) {
            mean_lat_temp = 0.00;
            mean_lon_temp = 0.00;
            for(i=0; i<N_MEMO_WIFI; i++) {
                mean_lat_temp += memo.pos_wifi_lat[i];
                mean_lon_temp += memo.pos_wifi_lon[i];
            }
            mean_lat_temp /= N_MEMO_WIFI;
            mean_lon_temp /= N_MEMO_WIFI;

            count_knn = 0;
            pos[0] = 0.00;
            pos[1] = 0.00;

            for (i=0; i<N_MEMO_WIFI; i++) {
                dist_temp = cal_dist_2D(mean_lat_temp, mean_lon_temp, memo.pos_wifi_lat[i], memo.pos_wifi_lon[i]);

                if (dist_temp <= TH_DIST_REJECT_NN_2) {
                    pos[0] += memo.pos_wifi_lat[i];
                    pos[1] += memo.pos_wifi_lon[i];

                    count_knn ++;
                }
            }

            if (count_knn) {
                avail_wifi[0] = 1;
                pos[0] /= count_knn;
                pos[1] /= count_knn;

                for (i=0; i<N_MEMO_WIFI-1; i++){
                    memo.pos_wifi_lat[i] = memo.pos_wifi_lat[i+1];
                    memo.pos_wifi_lon[i] = memo.pos_wifi_lon[i+1];
                }
                memo.pos_wifi_lat[N_MEMO_WIFI-1] = pos[0];
                memo.pos_wifi_lon[N_MEMO_WIFI-1] = pos[1];
            }
        }

    } // End if (N<3)
}


void wifi_position_tracker_smooth(char name_ap[MAX_AP_NUM][N_CHAR_MAC], float rss_ap[MAX_AP_NUM], int N, int floor[1], double pos[3], int avail_wifi[1]){
    static int first_time_flag = 0;
    static long int DB_mac[MAX_AP_NUM] = {0};
    static int DB_matrix[MAX_RP_NUM][MAX_AP_NUM];
    static double DB_pos[MAX_RP_NUM][3];
    static int DB_size[2];

    // -------------- 6 DBs -------------- 20151007
        static long int DB_mac_2[MAX_AP_NUM] = {0};
        static int DB_matrix_2[MAX_RP_NUM][MAX_AP_NUM];
        static double DB_pos_2[MAX_RP_NUM][3];
        static int DB_size_2[2];

        static long int DB_mac_3[MAX_AP_NUM] = {0};
        static int DB_matrix_3[MAX_RP_NUM][MAX_AP_NUM];
        static double DB_pos_3[MAX_RP_NUM][3];
        static int DB_size_3[2];

        static long int DB_mac_4[MAX_AP_NUM] = {0};
        static int DB_matrix_4[MAX_RP_NUM][MAX_AP_NUM];
        static double DB_pos_4[MAX_RP_NUM][3];
        static int DB_size_4[2];

        static long int DB_mac_5[MAX_AP_NUM] = {0};
        static int DB_matrix_5[MAX_RP_NUM][MAX_AP_NUM];
        static double DB_pos_5[MAX_RP_NUM][3];
        static int DB_size_5[2];

        static long int DB_mac_6[MAX_AP_NUM] = {0};
        static int DB_matrix_6[MAX_RP_NUM][MAX_AP_NUM];
        static double DB_pos_6[MAX_RP_NUM][3];
        static int DB_size_6[2];
        // ============== 6 DBs ===============

    static MemoIndoorWiFi memo;
    static int num_mac_diff = 0;

    double knn[N_KNN][4];
    float dss[MAX_RP_NUM];
    float min_dss[2] = {0.0};
    double w, sum_w, mean_lat_temp, mean_lon_temp, dist_temp;
    long int mac_ap = 0;
    int da_rss[MAX_AP_NUM], sum_rss_col, da_rss_before_diff[MAX_AP_NUM/10];
    int avail_wifi_temp = 0;
    int if_wifi_has_initialized = 1;
    double lat_temp, lon_temp;
    int i,j,i_min_dss, count_knn;

    float sum_rss_ap = 0.0;

    avail_wifi[0] = 0;

    // The detection of same RSS
    for (i=0; i<N; i++) {
        sum_rss_ap += rss_ap[i];
    }

    int if_same_rss_with_pre = 0;
    if (first_time_flag && N == memo.N_wifi_pre){
        if (fabs(sum_rss_ap-memo.sum_rss_ap_pre) < 10e-4){
            if_same_rss_with_pre = 1;
        }
    }
    memo.N_wifi_pre = N;
    memo.sum_rss_ap_pre = sum_rss_ap;

    //
    if (!first_time_flag){
        first_time_flag = 1;

        // Initial memo wifi
        memo.i_rss = 0;
        memo.i_memo_wifi = 0;
        for (i=0; i<LENGTH_MEMO_RSS; i++) {
            for (j=0; j<MAX_AP_NUM; j++) {
                memo.rss[i][j] = 0;
            }
        }
        for (i=0; i<N_MEMO_WIFI; i++) {
            memo.pos_wifi_lat[i] = 0.00;
            memo.pos_wifi_lon[i] = 0.00;
        }

        // End initial memo wifi

        if (IF_DIFF_WIFI == 1){
			if (IF_CODE_WIFI == 0) {
				load_DB_int_bin_diff(f_DB_bin_diff_str, DB_mac, DB_matrix, DB_pos, DB_size);
			}
			else if (IF_CODE_WIFI == 1) {
				load_DB_int_bin_diff_co(f_DB_bin_diff_str, DB_mac, DB_matrix, DB_pos, DB_size);
			}
			num_mac_diff = DB_size[1] * DB_size[1];
		}
		else {
			if (IF_CODE_WIFI == 0) {
				// load_DB_int_bin(f_DB_bin_str, DB_mac, DB_matrix, DB_pos, DB_size);
				if (NUM_WIFI_DB == 1) {
					load_DB_int_bin(f_DB_bin_str, DB_mac, DB_matrix, DB_pos, DB_size);
				}
				// -------------- 6 DBs -------------- 20151007
				else if (NUM_WIFI_DB == 4)
				{
					load_DB_int_bin(f_DB_bin_str, DB_mac, DB_matrix, DB_pos, DB_size);
					load_DB_int_bin(f_DB_bin_str_2, DB_mac_2, DB_matrix_2, DB_pos_2, DB_size_2);
					load_DB_int_bin(f_DB_bin_str_3, DB_mac_3, DB_matrix_3, DB_pos_3, DB_size_3);
					load_DB_int_bin(f_DB_bin_str_4, DB_mac_4, DB_matrix_4, DB_pos_4, DB_size_4);
					load_DB_int_bin(f_DB_bin_str_5, DB_mac_5, DB_matrix_5, DB_pos_5, DB_size_5);
					load_DB_int_bin(f_DB_bin_str_6, DB_mac_6, DB_matrix_6, DB_pos_6, DB_size_6);
				}
				// ============== 6 DBs ===============
			}
			else if (IF_CODE_WIFI == 1) {
				load_DB_int_bin_co(f_DB_bin_str, DB_mac, DB_matrix, DB_pos, DB_size);
			}
			num_mac_diff = DB_size[1];
		}
    }


    if (if_same_rss_with_pre || N<3) {		// Less than 3 APs
        avail_wifi[0] = 0;
        lat_temp = INI_POS[0];
        lon_temp = INI_POS[1];
    }
    else {
        if (IF_DIFF_WIFI == 0)
        {
            for (i=0; i<MAX_AP_NUM; i++)
            {
                da_rss[i] = 100;
            }
        }
        else if (IF_DIFF_WIFI == 1)
        {
            for (i=0; i<MAX_AP_NUM/10; i++)
            {
                da_rss_before_diff[i] = -100;
            }
            for (i=0; i<MAX_AP_NUM; i++)
            {
                da_rss[i] = 0;
            }
        }

        // Fill the da_rss vector
        for (i=0; i<N; i++) {
            // Convert MAC
            mac_ap = mac2int(name_ap[i]);

			// ------------------------- 6 DBs ------------------------- 20151007
			if (NUM_WIFI_DB == 1 || (NUM_WIFI_DB == 4 && floor[0] == 1)) {
				for (j=0; j<DB_size[1]; j++) {
					if (mac_ap == DB_mac[j]) {
						if (IF_DIFF_WIFI == 0) {
							da_rss[j] = -(int)(rss_ap[i]);
						}
						else if (IF_DIFF_WIFI == 1) {
							da_rss_before_diff[j] = (int)(rss_ap[i]);
						}

						break;
					}
				}
			}
			else if(NUM_WIFI_DB == 4 && floor[0] != 1) {
				if (floor[0] == 2)
				{
					for (j=0; j<DB_size_2[1]; j++) {
						if (mac_ap == DB_mac_2[j]) {
							if (IF_DIFF_WIFI == 0) {
								da_rss[j] = -(int)(rss_ap[i]);
							}
							else if (IF_DIFF_WIFI == 1) {
								da_rss_before_diff[j] = (int)(rss_ap[i]);
							}

							break;
						}
					}
				}
				else if (floor[0] == 3)
				{
					for (j=0; j<DB_size_3[1]; j++) {
						if (mac_ap == DB_mac_3[j]) {
							if (IF_DIFF_WIFI == 0) {
								da_rss[j] = -(int)(rss_ap[i]);
							}
							else if (IF_DIFF_WIFI == 1) {
								da_rss_before_diff[j] = (int)(rss_ap[i]);
							}

							break;
						}
					}
				}
				else if (floor[0] == 4)
				{
					for (j=0; j<DB_size_4[1]; j++) {
						if (mac_ap == DB_mac_4[j]) {
							if (IF_DIFF_WIFI == 0) {
								da_rss[j] = -(int)(rss_ap[i]);
							}
							else if (IF_DIFF_WIFI == 1) {
								da_rss_before_diff[j] = (int)(rss_ap[i]);
							}

							break;
						}
					}
				}
				else if (floor[0] == 5)
				{
					for (j=0; j<DB_size_5[1]; j++) {
						if (mac_ap == DB_mac_5[j]) {
							if (IF_DIFF_WIFI == 0) {
								da_rss[j] = -(int)(rss_ap[i]);
							}
							else if (IF_DIFF_WIFI == 1) {
								da_rss_before_diff[j] = (int)(rss_ap[i]);
							}

							break;
						}
					}
				}

				else if (floor[0] == 6)
				{
					for (j=0; j<DB_size_6[1]; j++) {
						if (mac_ap == DB_mac_6[j]) {
							if (IF_DIFF_WIFI == 0) {
								da_rss[j] = -(int)(rss_ap[i]);
							}
							else if (IF_DIFF_WIFI == 1) {
								da_rss_before_diff[j] = (int)(rss_ap[i]);
							}

							break;
						}
					}
				}

			}

		// ============================ 6 DBs =================================

//            for (j=0; j<DB_size[1]; j++) {
//                if (mac_ap == DB_mac[j]) {
//                    if (IF_DIFF_WIFI == 0) {
//                        da_rss[j] = -(int)(rss_ap[i]);
//                    }
//                    else if (IF_DIFF_WIFI == 1) {
//                        da_rss_before_diff[j] = (int)(rss_ap[i]);
//                    }
//
//                    break;
//                }
//            }
        }

        if (IF_DIFF_WIFI == 1)
        {
            for (i=0; i<DB_size[1]; i++)
            {
                for (j=0; j<DB_size[1]; j++)
                {
                    da_rss[i*DB_size[1]+j] = da_rss_before_diff[i] - da_rss_before_diff[j];
                }
            }
        }


        // NEW Averaging rss
        memo.i_rss ++;
        if (LENGTH_MEMO_RSS == 1) {
            memo.i_rss = LENGTH_MEMO_RSS;
            // Do nothing
        }
        else {
            if (memo.i_rss <= LENGTH_MEMO_RSS) {
                for (j=0; j<MAX_AP_NUM; j++) {
                    memo.rss[memo.i_rss-1][j] = da_rss[j];
                }
            }
            else {
                memo.i_rss = LENGTH_MEMO_RSS;
                for (i=0; i<LENGTH_MEMO_RSS-1; i++) {
                    for (j=0; j<MAX_AP_NUM; j++) {
                        memo.rss[i][j] = memo.rss[i+1][j];
                    }
                }
                for (j=0; j<MAX_AP_NUM; j++) {
                    memo.rss[LENGTH_MEMO_RSS-1][j] = da_rss[j];
                }
            }

            // Mean
            for (j=0; j<MAX_AP_NUM; j++) {
                sum_rss_col = 0;
                for (i=0; i<memo.i_rss; i++) {
                    sum_rss_col += memo.rss[i][j];
                }
                da_rss[j] = (int)(sum_rss_col*1.0/memo.i_rss);
            }
        }
        // End Averaging rss


        //
        // ------------------------- 6 DBs ------------------------- 20151007
		if (NUM_WIFI_DB == 1 || (NUM_WIFI_DB == 4 && floor[0] == 1)) {
			for(i=0; i<DB_size[0]; i++) {
				dss[i] = cal_dss_int(da_rss, DB_matrix[i], DB_size[1]);
			}
		}
		else if(NUM_WIFI_DB == 4 && floor[0] != 1) {
			if (floor[0] == 2)
			{
				for(i=0; i<DB_size_2[0]; i++) {
					dss[i] = cal_dss_int(da_rss, DB_matrix_2[i], DB_size_2[1]);
				}
			}
			else if (floor[0] == 3)
			{
				for(i=0; i<DB_size_3[0]; i++) {
					dss[i] = cal_dss_int(da_rss, DB_matrix_3[i], DB_size_3[1]);
				}
			}
			else if (floor[0] == 4)
			{
				for(i=0; i<DB_size_4[0]; i++) {
					dss[i] = cal_dss_int(da_rss, DB_matrix_4[i], DB_size_4[1]);
				}
			}
			else if (floor[0] == 5)
			{
				for(i=0; i<DB_size_5[0]; i++) {
					dss[i] = cal_dss_int(da_rss, DB_matrix_5[i], DB_size_5[1]);
				}
			}
			else if (floor[0] == 6)
			{
				for(i=0; i<DB_size_6[0]; i++) {
					dss[i] = cal_dss_int(da_rss, DB_matrix_6[i], DB_size_6[1]);
				}
			}
		}
		// ============================ 6 DBs =================================

//        for(i=0; i<DB_size[0]; i++) {
//            dss[i] = cal_dss_int(da_rss, DB_matrix[i], DB_size[1]);
//        }

        // Find KNN
        mean_lat_temp = 0.00;
        mean_lon_temp = 0.00;

        // ------------------------- 6 DBs ------------------------- 20151007
		if (NUM_WIFI_DB == 1 || (NUM_WIFI_DB == 4 && floor[0] == 1)) {
				for(i=0; i<N_KNN; i++) {
					min_float(dss, min_dss, DB_size[0]);
					knn[i][0] = min_dss[1];
					i_min_dss = (int)(knn[i][0]);
					knn[i][1] = min_dss[0];
					knn[i][2] = DB_pos[i_min_dss][0];
					knn[i][3] = DB_pos[i_min_dss][1];
					dss[i_min_dss] = 999999.0;

					mean_lat_temp += knn[i][2];
					mean_lon_temp += knn[i][3];
				}
		}
		else if(NUM_WIFI_DB == 4 && floor[0] != 1) {
			if (floor[0] == 2)
			{
				for(i=0; i<N_KNN; i++) {
					min_float(dss, min_dss, DB_size_2[0]);
					knn[i][0] = min_dss[1];
					i_min_dss = (int)(knn[i][0]);
					knn[i][1] = min_dss[0];
					knn[i][2] = DB_pos_2[i_min_dss][0];
					knn[i][3] = DB_pos_2[i_min_dss][1];
					dss[i_min_dss] = 999999.0;

					mean_lat_temp += knn[i][2];
					mean_lon_temp += knn[i][3];
				}
			}
			else if (floor[0] == 3)
			{
				for(i=0; i<N_KNN; i++) {
					min_float(dss, min_dss, DB_size_3[0]);
					knn[i][0] = min_dss[1];
					i_min_dss = (int)(knn[i][0]);
					knn[i][1] = min_dss[0];
					knn[i][2] = DB_pos_3[i_min_dss][0];
					knn[i][3] = DB_pos_3[i_min_dss][1];
					dss[i_min_dss] = 999999.0;

					mean_lat_temp += knn[i][2];
					mean_lon_temp += knn[i][3];
				}
			}
			else if (floor[0] == 4)
			{
				for(i=0; i<N_KNN; i++) {
					min_float(dss, min_dss, DB_size_4[0]);
					knn[i][0] = min_dss[1];
					i_min_dss = (int)(knn[i][0]);
					knn[i][1] = min_dss[0];
					knn[i][2] = DB_pos_4[i_min_dss][0];
					knn[i][3] = DB_pos_4[i_min_dss][1];
					dss[i_min_dss] = 999999.0;

					mean_lat_temp += knn[i][2];
					mean_lon_temp += knn[i][3];
				}
			}
			else if (floor[0] == 5)
			{
				for(i=0; i<N_KNN; i++) {
					min_float(dss, min_dss, DB_size_5[0]);
					knn[i][0] = min_dss[1];
					i_min_dss = (int)(knn[i][0]);
					knn[i][1] = min_dss[0];
					knn[i][2] = DB_pos_5[i_min_dss][0];
					knn[i][3] = DB_pos_5[i_min_dss][1];
					dss[i_min_dss] = 999999.0;

					mean_lat_temp += knn[i][2];
					mean_lon_temp += knn[i][3];
				}
			}
			else if (floor[0] == 6)
			{
				for(i=0; i<N_KNN; i++) {
					min_float(dss, min_dss, DB_size_6[0]);
					knn[i][0] = min_dss[1];
					i_min_dss = (int)(knn[i][0]);
					knn[i][1] = min_dss[0];
					knn[i][2] = DB_pos_6[i_min_dss][0];
					knn[i][3] = DB_pos_6[i_min_dss][1];
					dss[i_min_dss] = 999999.0;

					mean_lat_temp += knn[i][2];
					mean_lon_temp += knn[i][3];
				}
			}

		}
		// ============================ 6 DBs =================================
//        for(i=0; i<N_KNN; i++) {
//            min_float(dss, min_dss, DB_size[0]);
//            knn[i][0] = min_dss[1];
//            i_min_dss = (int)(knn[i][0]);
//            knn[i][1] = min_dss[0];
//            knn[i][2] = DB_pos[i_min_dss][0];
//            knn[i][3] = DB_pos[i_min_dss][1];
//            dss[i_min_dss] = 999999.0;
//
//            mean_lat_temp += knn[i][2];
//            mean_lon_temp += knn[i][3];
//        }
        mean_lat_temp /= N_KNN;   // In matlab, it is mean_pos
        mean_lon_temp /= N_KNN;

        // KNN
        // Reject RSS which is far from others
        sum_w = 0.00;
        count_knn = 0;
        lat_temp = 0.00;
        lon_temp = 0.00;
        for (i=0; i<N_KNN; i++) {
            dist_temp = cal_dist_2D(mean_lat_temp, mean_lon_temp, knn[i][2], knn[i][3]);

            if (dist_temp <= TH_DIST_REJECT_NN) {
                if (knn[i][1]<1.0) {
                    knn[i][1] = 1.0;
                }

                w = 1/knn[i][1];
                sum_w += w;
                lat_temp += w*knn[i][2];
                lon_temp += w*knn[i][3];

                count_knn ++;
            }
        }

        if (count_knn) {
            avail_wifi_temp = 1;
            lat_temp /= sum_w;
            lon_temp /= sum_w;
        }

        //
        if(!memo.i_memo_wifi && !avail_wifi_temp) {
            if_wifi_has_initialized = 0;
        }

        if (if_wifi_has_initialized) {
            if (!avail_wifi_temp && memo.i_memo_wifi>=1) {
                lat_temp = memo.pos_wifi_lat[memo.i_memo_wifi-1];
                lon_temp = memo.pos_wifi_lon[memo.i_memo_wifi-1];
            }

            if (avail_wifi_temp) {
                if (memo.i_memo_wifi < N_MEMO_WIFI) {  // HERE IS A "<". To be determined in the future
                    memo.i_memo_wifi ++;
                    memo.pos_wifi_lat[memo.i_memo_wifi-1] = lat_temp;
                    memo.pos_wifi_lon[memo.i_memo_wifi-1] = lon_temp;
                }
                else {
                    memo.i_memo_wifi = N_MEMO_WIFI;
                    for (i=0; i<N_MEMO_WIFI-1; i++) {
                        memo.pos_wifi_lat[i] = memo.pos_wifi_lat[i+1];
                        memo.pos_wifi_lon[i] = memo.pos_wifi_lon[i+1];
                    }
                    memo.pos_wifi_lat[N_MEMO_WIFI-1] = lat_temp;
                    memo.pos_wifi_lon[N_MEMO_WIFI-1] = lon_temp;
                }
            }
        }

        // Now the WiFi position is outputed only when memo.i_memo_wifi == N_MEMO_WIFI
        if (memo.i_memo_wifi == N_MEMO_WIFI) {
            mean_lat_temp = 0.00;
            mean_lon_temp = 0.00;
            for(i=0; i<N_MEMO_WIFI; i++) {
                mean_lat_temp += memo.pos_wifi_lat[i];
                mean_lon_temp += memo.pos_wifi_lon[i];
            }
            mean_lat_temp /= N_MEMO_WIFI;
            mean_lon_temp /= N_MEMO_WIFI;

            count_knn = 0;
            pos[0] = 0.00;
            pos[1] = 0.00;

            for (i=0; i<N_MEMO_WIFI; i++) {
                dist_temp = cal_dist_2D(mean_lat_temp, mean_lon_temp, memo.pos_wifi_lat[i], memo.pos_wifi_lon[i]);

                if (dist_temp <= TH_DIST_REJECT_NN_2) {
                    pos[0] += memo.pos_wifi_lat[i];
                    pos[1] += memo.pos_wifi_lon[i];

                    count_knn ++;
                }
            }

            if (count_knn) {
                avail_wifi[0] = 1;
                pos[0] /= count_knn;
                pos[1] /= count_knn;

                for (i=0; i<N_MEMO_WIFI-1; i++){
                    memo.pos_wifi_lat[i] = memo.pos_wifi_lat[i+1];
                    memo.pos_wifi_lon[i] = memo.pos_wifi_lon[i+1];
                }
                memo.pos_wifi_lat[N_MEMO_WIFI-1] = pos[0];
                memo.pos_wifi_lon[N_MEMO_WIFI-1] = pos[1];
            }
        }

    } // End if (N<3)
}

void wifi_position_tracker(char name_ap[MAX_AP_NUM][N_CHAR_MAC], float rss_ap[MAX_AP_NUM], int N, int floor[1], double pos[2], int availability[1]){
	static int first_time_flag = 0;
	static long int DB_mac[MAX_AP_NUM] = {0};
	static int DB_matrix[MAX_RP_NUM][MAX_AP_NUM];
	static double DB_pos[MAX_RP_NUM][3];
	static int DB_size[2];

	double knn[N_KNN][4], dss[MAX_AP_NUM];
	double min_dss[2] = {0.00};
	double w, sum_w, mean_lat_temp, mean_lon_temp, dist_temp;
	long int mac_ap = 0;
	int da_rss[MAX_AP_NUM];
	int i,j,i_min_dss;

	if (!first_time_flag){
		load_DB_int_bin(f_DB_bin, DB_mac, DB_matrix, DB_pos, DB_size);
		first_time_flag = 1;
	}

	for (i=0; i<MAX_AP_NUM; i++) {
		da_rss[i] = 100;
	}

	if (N<3) {					// Less than 2 APs
		availability[0] = 0;
		pos[0] = INI_POS[0];
		pos[1] = INI_POS[1];
	}
	else {
		// Fill the da_rss vector
		for (i=0; i<N; i++) {
			// Convert MAC
			mac_ap = mac2int(name_ap[i]);

			for (j=0; j<DB_size[1]; j++) {
				if (mac_ap == DB_mac[j]) {
					da_rss[j] = -(int)(rss_ap[i]);
					break;
				}
			}
		}

		for(i=0; i<DB_size[0]; i++) {
			dss[i] = cal_dss_int(da_rss, DB_matrix[i], DB_size[1]);
		}

		// Find KNN
		mean_lat_temp = 0.00;
		mean_lon_temp = 0.00;
		for(i=0; i<N_KNN; i++) {
			min_double(dss, min_dss, DB_size[0]);
			knn[i][0] = min_dss[1];
			i_min_dss = (int)(knn[i][0]);
			knn[i][1] = min_dss[0];
			knn[i][2] = DB_pos[i_min_dss][0];
			knn[i][3] = DB_pos[i_min_dss][1];
			dss[i_min_dss] = 999999.00;

			mean_lat_temp += knn[i][2];
			mean_lon_temp += knn[i][3];
		}
		mean_lat_temp /= N_KNN;
		mean_lon_temp /= N_KNN;

		// KNN
		// Reject RSS which is far from others
		pos[0] = 0.00;
		pos[1] = 0.00;

		sum_w = 0.00;
		for (i=0; i<N_KNN; i++) {
			dist_temp = cal_dist_2D(mean_lat_temp, mean_lon_temp, knn[i][2], knn[i][3]);

			if (dist_temp < TH_DIST_REJECT_NN) {
				w = 1/knn[i][1];
				sum_w += w;
				pos[0] += w*knn[i][2];
				pos[1] += w*knn[i][3];
			}
		}

		if (abs(sum_w) > 10e-6) {
			availability[0] = 1;
			pos[0] /= sum_w;
			pos[1] /= sum_w;

		}else{
			availability[0] = 0;
			pos[0] = INI_POS[0];
			pos[1] = INI_POS[1];
		}
	}
}

UINT8 map_adjusting(NavIndoor *nav, Att_Pkt *att_pkt, MemoIndoor *memo)
{
	UINT8 if_align_heading1 = 0;
	FLOAT32 ang_align,  diff_heading_step_heading;

	//if_align_heading = 0;
	if (memo->i_memo_heading_step > 5) {
		if (fabs(nav->heading_step - memo->heading_step[memo->i_memo_heading_step-1]) < TH_HEAD_CHANGE_MAX_IS_LINE &&
			fabs(memo->heading_step[memo->i_memo_heading_step-1] - memo->heading_step[memo->i_memo_heading_step-2]) < TH_HEAD_CHANGE_MAX_IS_LINE &&
			fabs(memo->heading_step[memo->i_memo_heading_step-2] - memo->heading_step[memo->i_memo_heading_step-3]) < TH_HEAD_CHANGE_MAX_IS_LINE  &&
			fabs(memo->heading_step[memo->i_memo_heading_step-3] - memo->heading_step[memo->i_memo_heading_step-4]) < TH_HEAD_CHANGE_MAX_IS_LINE &&
			fabs(memo->heading_step[memo->i_memo_heading_step-4] - memo->heading_step[memo->i_memo_heading_step-5]) < TH_HEAD_CHANGE_MAX_IS_LINE) //  &&
			//fabs(memo->heading_step[memo->i_memo_heading_step-5] - memo->heading_step[memo->i_memo_heading_step-6]) < TH_HEAD_CHANGE_MAX_IS_LINE)

		{
			if_align_heading1 = 1;
		}
	}

	if (if_align_heading1) {
		ang_align = nav->heading_step;
		if (fabs(nav->heading_step) < TH_HEAD_AIGN_TO_LINK_MAX) {
			ang_align = ANG_RAD_0;
		}
//		else if (fabs(nav->heading_step-ANG_RAD_90) < TH_HEAD_AIGN_TO_LINK_MAX) {
//			ang_align = ANG_RAD_90;
//		}
//		else if (fabs(nav->heading_step-ANG_RAD_NEG_90) < TH_HEAD_AIGN_TO_LINK_MAX) {
//			ang_align = ANG_RAD_NEG_90;
//		}
		else if (fabs(nav->heading_step-ANG_RAD_180) < TH_HEAD_AIGN_TO_LINK_MAX) {
			ang_align = ANG_RAD_180;
		}
		else if (fabs(nav->heading_step-ANG_RAD_NEG_180) < TH_HEAD_AIGN_TO_LINK_MAX) {
			ang_align = ANG_RAD_180;
		}
		diff_heading_step_heading = ang_align - nav->heading_step;
		nav->heading_step = ang_align;

		//nav->heading = nav->heading + diff_heading_step_heading;
		nav->heading_platform = center_heading(nav->heading_platform + diff_heading_step_heading);

		// ---------------------- NEW in this version ----
		nav->heading_device = center_heading(nav->heading_device + diff_heading_step_heading);
		memo->heading_platform[memo->i_gyro-1] = center_heading(memo->heading_platform[memo->i_gyro-1] + diff_heading_step_heading);
		//att_pkt->heading_platform = center_heading(att_pkt->heading_platform + diff_heading_step_heading);

		// ====================== NEW in this version ====

		att_pkt->att[2] = center_heading(att_pkt->att[2] + diff_heading_step_heading);
		Euler2Dcm_att(att_pkt->att, att_pkt->Cbn);
		Dcm2Quat_att(att_pkt->Cbn, att_pkt->qbn);
	}

	return if_align_heading1;
}


void load_DB_int_bin_diff_co(const char * f_DB, long int DB_mac[MAX_AP_NUM], int DB_matrix_diff[MAX_RP_NUM][MAX_AP_NUM], double DB_pos[MAX_RP_NUM][3], int DB_size[2]) {
    FILE * f_in_DB = NULL;
    int i,j;
    double temp_double[1];
    long int temp_DB_mac[MAX_AP_NUM] = {0};

    if((f_in_DB = fopen(f_DB,"rb")) == NULL){
        printf("The DB file <%s> can not be opened.\n", f_DB);
        exit(1);
    }

    // nrow and ncol
    fread(DB_size, 2, 1, f_in_DB);
    fread(DB_size+1, 2, 1, f_in_DB);

    // DB_mac
    for (i=0; i<DB_size[1]; i++) {
        fread(DB_mac+i, 4, 1, f_in_DB);
        temp_DB_mac[i] = DB_mac[i];
    }
    // -------
    for(i=0; i<DB_size[1]; i++) {
        DB_mac[i] = temp_DB_mac[DB_size[1]-1 - i];
    }

    // DB_pos
    for (i=0; i<DB_size[0]; i++) {
        fread(DB_pos[i], sizeof(double), 1, f_in_DB);
    }
    for (i=0; i<DB_size[0]; i++) {
        fread(DB_pos[i]+1, sizeof(double), 1, f_in_DB);
    }
    for (i=0; i<DB_size[0]; i++) {
        fread(DB_pos[i]+2, sizeof(double), 1, f_in_DB);
    }

    // DB_matrix
    for (i=0; i<DB_size[0]; i++) {
        for (j=0; j<DB_size[1]*DB_size[1]; j++) {
            //fread(DB_matrix_diff[i]+j, sizeof(double), 1, f_in_DB);
            fread(temp_double, sizeof(double), 1, f_in_DB);
            if (fabs(temp_double[0]) > 0.01) {
                temp_double[0] = 115.0 - temp_double[0];
            }
            DB_matrix_diff[i][j] = (int)(temp_double[0]);
        }
    }
}


void load_DB_int_bin_co(const char * f_DB, long int DB_mac[MAX_AP_NUM], int DB_matrix[MAX_RP_NUM][MAX_AP_NUM], double DB_pos[MAX_RP_NUM][3], int DB_size[2]) {
    FILE * f_in_DB = NULL;
    int i,j;
    long int temp_DB_mac[MAX_AP_NUM] = {0};

    if((f_in_DB = fopen(f_DB,"rb")) == NULL){
        printf("The DB file <%s> can not be opened.\n", f_DB);
        exit(1);
    }

    // nrow and ncol
    fread(DB_size, 2, 1, f_in_DB);
    fread(DB_size+1, 2, 1, f_in_DB);

    // DB_mac
    for (i=0; i<DB_size[1]; i++) {
        fread(DB_mac+i, 4, 1, f_in_DB);
        temp_DB_mac[i] = DB_mac[i];
    }
    // -------
    for(i=0; i<DB_size[1]; i++) {
        DB_mac[i] = temp_DB_mac[DB_size[1]-1 - i];
    }
    // DB_pos
    for (i=0; i<DB_size[0]; i++) {
        fread(DB_pos[i], sizeof(double), 1, f_in_DB);
    }
    for (i=0; i<DB_size[0]; i++) {
        fread(DB_pos[i]+1, sizeof(double), 1, f_in_DB);
    }
    for (i=0; i<DB_size[0]; i++) {
        fread(DB_pos[i]+2, sizeof(double), 1, f_in_DB);
    }

    // DB_matrix
    for (i=0; i<DB_size[0]; i++) {
        for (j=0; j<DB_size[1]; j++) {
            fread(DB_matrix[i]+j, 1, 1, f_in_DB);

            if (fabs(DB_matrix[i][j]-100.0) > 0.01) {
                DB_matrix[i][j] = 115 - DB_matrix[i][j];
            }

            if(DB_matrix[i][j] < 0.1){   //0->100
                DB_matrix[i][j] = RSS_DB_4_ZERO;
            }
        }
    }
}


void load_DB_int_bin(const char * f_DB, long int DB_mac[MAX_AP_NUM], int DB_matrix[MAX_RP_NUM][MAX_AP_NUM], double DB_pos[MAX_RP_NUM][3], int DB_size[2]) {
	FILE * f_in_DB = NULL;
	int i,j;

	if((f_in_DB = fopen(f_DB,"rb")) == NULL){
		printf("The DB file <%s> can not be opened.\n", f_DB);
		exit(1);
	}

	// nrow and ncol
	fread(DB_size, 2, 1, f_in_DB);
	fread(DB_size+1, 2, 1, f_in_DB);

	// DB_mac
	for (i=0; i<DB_size[1]; i++) {
		fread(DB_mac+i, 4, 1, f_in_DB);
	}
	// DB_pos
	for (i=0; i<DB_size[0]; i++) {
		fread(DB_pos[i], sizeof(double), 1, f_in_DB);
	}
	for (i=0; i<DB_size[0]; i++) {
		fread(DB_pos[i]+1, sizeof(double), 1, f_in_DB);
	}
	for (i=0; i<DB_size[0]; i++) {
		fread(DB_pos[i]+2, sizeof(double), 1, f_in_DB);
	}

	// DB_matrix
	for (i=0; i<DB_size[0]; i++) {
		for (j=0; j<DB_size[1]; j++) {
			fread(DB_matrix[i]+j, 1, 1, f_in_DB);

			if(DB_matrix[i][j] < 0.1){   //0->100
				DB_matrix[i][j] = RSS_DB_4_ZERO;
			}
		}
	}
}


void load_DB_int_bin_diff(const char * f_DB, long int DB_mac[MAX_AP_NUM], int DB_matrix_diff[MAX_RP_NUM][MAX_AP_NUM], double DB_pos[MAX_RP_NUM][3], int DB_size[2]) {
	FILE * f_in_DB = NULL;
	int i,j;
	double temp_double[1];

	if((f_in_DB = fopen(f_DB,"rb")) == NULL){
		printf("The DB file <%s> can not be opened.\n", f_DB);
		exit(1);
	}

	// nrow and ncol
	fread(DB_size, 2, 1, f_in_DB);
	fread(DB_size+1, 2, 1, f_in_DB);

	// DB_mac
	for (i=0; i<DB_size[1]; i++) {
		fread(DB_mac+i, 4, 1, f_in_DB);
	}
	// DB_pos
	for (i=0; i<DB_size[0]; i++) {
		fread(DB_pos[i], sizeof(double), 1, f_in_DB);
	}
	for (i=0; i<DB_size[0]; i++) {
		fread(DB_pos[i]+1, sizeof(double), 1, f_in_DB);
	}
	for (i=0; i<DB_size[0]; i++) {
		fread(DB_pos[i]+2, sizeof(double), 1, f_in_DB);
	}

	// DB_matrix
	for (i=0; i<DB_size[0]; i++) {
		for (j=0; j<DB_size[1]*DB_size[1]; j++) {
			//fread(DB_matrix_diff[i]+j, sizeof(double), 1, f_in_DB);
			fread(temp_double, sizeof(double), 1, f_in_DB);
			DB_matrix_diff[i][j] = (int)temp_double[0];
		}
	}
}



void indoor_navigation_old(int avail_gyro, double t_gyro, float w_b[3], int avail_accel, double t_accel, float f_b[3], RecSol sol){
	static UINT8 fitst_time_flag = 0;
	static NavIndoor nav;
	static MemoIndoor memo;

	static FLOAT32 gyro_bias[3];
	int i, j, i_mid_in_memo, i_heading_in_memo;
	double min_t_heading_in_memo[2];
	double max_min_qs_detect[2], diff_max_min_gz;
	float norma1, roll1, pitch1, f_b1[3], sr1, cr1, sp1, cp1, wd1, ma_mid1, min_t1;
	float heading;
	double t_mid1, lat1, ts_gyro;
	double t_diff_g_a[LENGTH_MEMO_SENSOR_DATA];

	int if_detect_step, if_update_step, if_qs;
	int if_update_pos = 0;

	w_b[0] *= D2R;
	w_b[1] *= D2R;
	w_b[2] *= D2R;

	//first time
	if (!fitst_time_flag){
		fitst_time_flag = 1;

		gyro_bias[0] = ini_gyro_bias_att[0];
		gyro_bias[1] = ini_gyro_bias_att[1];
		gyro_bias[2] = ini_gyro_bias_att[2];   // Later we can save and read gBias from a file

		// Nav.P
		// Nav.Q

		// -- Initialization PDR
		nav.pos[0] = INI_POS[0];
		nav.pos[1] = INI_POS[1];
		nav.pos[2] = INI_POS[2];

		if (TYPE_INI_HEADING == 1) {
			nav.heading_platform = INI_HEADING;
		} else {    // Other strategies

		}
		nav.heading_step = nav.heading_platform;
		nav.t_step1 = 0;


		//-- End Initialization PDR



		// ---- INITIALIZAION_MEMO
		memo.i_accel = 0;
		memo.i_gyro = 0;
		memo.i_ma = 0;
		memo.i_t_step = 0;
		memo.i_pos_pdr = 0;

		//memo.t_accel = 0.00;
		//memo.t_gyro = 0.00;
		//memo.t_step = 0.00;
		//memo.t_pos_pdr = 0.00;

		for(i=0; i<LENGTH_MEMO_SENSOR_DATA; i++){
			memo.t_accel[i] = 0.00;
			memo.a_x[i] = 0.0;
			memo.a_y[i] = 0.0;
			memo.a_z[i] = 0.0;
			memo.t_gyro[i] = 0.00;
			memo.g_x[i] = 0.0;
			memo.g_y[i] = 0.0;
			memo.g_z[i] = 0.0;
			memo.gz_level[i] = 0.0;
			memo.heading_platform[i] = 0.0;
		}
		for(i=0; i<LENGTH_MEMO_MA; i++){
			memo.ma[i] = 0.0;
		}
		for(i=0; i<LENGTH_MEMO_T_STEP; i++) {
			memo.t_step[i] = 0.00;
		}
		for(i=0; i<LENGTH_MEMO_POS_PDR; i++) {
			memo.t_pos_pdr[i] = 0.00;
			memo.pos_pdr_lat[i] = 0.00;
			memo.pos_pdr_lon[i] = 0.00;
			memo.pos_pdr_hei[i] = 0.00;
		}
		// ---- End INITIALIZAION_MEMO


		memo.i_pos_pdr ++;
		memo.pos_pdr_lat[memo.i_pos_pdr-1] = nav.pos[0];
		memo.pos_pdr_lon[memo.i_pos_pdr-1] = nav.pos[1];
		memo.pos_pdr_hei[memo.i_pos_pdr-1] = nav.pos[2];

		if (avail_accel){
			memo.t_pos_pdr[memo.i_pos_pdr-1] = t_accel;
		} else if (avail_gyro) {
			memo.t_pos_pdr[memo.i_pos_pdr-1] = t_gyro;
		} else {
			memo.t_pos_pdr[memo.i_pos_pdr-1] = 0.00;
		}



		// Output
		sol.count_rec_pos ++;
		fprintf(sol.f_out_rec_pos, "%.4f %.8f %.8f %.3f\n",
				memo.t_pos_pdr[memo.i_pos_pdr-1], memo.pos_pdr_lat[memo.i_pos_pdr-1]/D2R,
				memo.pos_pdr_lon[memo.i_pos_pdr-1]/D2R, memo.pos_pdr_hei[memo.i_pos_pdr-1]);

	}


	w_b[0] -= gyro_bias[0];
	w_b[1] -= gyro_bias[1];
	w_b[2] -= gyro_bias[2];




	// -- Put into memo
	if (avail_accel){
		memo.i_accel ++;
		if (memo.i_accel < LENGTH_MEMO_SENSOR_DATA) {
			memo.t_accel[memo.i_accel-1] = t_accel;
			memo.a_x[memo.i_accel-1] = f_b[0];
			memo.a_y[memo.i_accel-1] = f_b[1];
			memo.a_z[memo.i_accel-1] = f_b[2];
		}else {
			memo.i_accel = LENGTH_MEMO_SENSOR_DATA;
			for (i=0; i<LENGTH_MEMO_SENSOR_DATA-1; i++) {
				memo.t_accel[i] = memo.t_accel[i+1];
				memo.a_x[i] = memo.a_x[i+1];
				memo.a_y[i] = memo.a_y[i+1];
				memo.a_z[i] = memo.a_z[i+1];
			}
			memo.t_accel[LENGTH_MEMO_SENSOR_DATA-1] = t_accel;
			memo.a_x[LENGTH_MEMO_SENSOR_DATA-1] = f_b[0];
			memo.a_y[LENGTH_MEMO_SENSOR_DATA-1] = f_b[1];
			memo.a_z[LENGTH_MEMO_SENSOR_DATA-1] = f_b[2];
		}

		norma1 = sqrt(f_b[0]*f_b[0]+f_b[1]*f_b[1]+f_b[2]*f_b[2]);
		memo.i_ma ++;
		if(memo.i_ma < LENGTH_MEMO_MA){
			memo.ma[memo.i_ma-1] = norma1;
		} else {
			memo.i_ma = LENGTH_MEMO_MA;
			for (i=0; i<LENGTH_MEMO_MA-1; i++) {
				memo.ma[i] = memo.ma[i+1];
			}
			memo.ma[LENGTH_MEMO_MA-1] = norma1;

		}
	}

	if (avail_gyro) {
		if(!memo.i_accel) {
			roll1 = 0.0;
			pitch1 = 0.0;
		} else{
			f_b1[0] = memo.a_x[memo.i_accel-1];
			f_b1[1] = memo.a_y[memo.i_accel-1];
			f_b1[2] = memo.a_z[memo.i_accel-1];
			roll1 = cal_roll(f_b1);
			pitch1 = cal_pitch(f_b1);
		}
		sr1 = sin(roll1); cr1 = cos(roll1);
		sp1 = sin(pitch1); cp1 = cos(pitch1);
		wd1 = -sp1*w_b[0] + sr1*cp1*w_b[1] + cr1*cp1*w_b[2];

		memo.i_gyro ++;
		if (memo.i_gyro < LENGTH_MEMO_SENSOR_DATA) {
			memo.t_gyro[memo.i_gyro-1] = t_gyro;
			memo.g_x[memo.i_gyro-1] = w_b[0];
			memo.g_y[memo.i_gyro-1] = w_b[1];
			memo.g_z[memo.i_gyro-1] = w_b[2];
			memo.gz_level[memo.i_gyro-1] = wd1;
		}else {
			memo.i_gyro = LENGTH_MEMO_SENSOR_DATA;
			for (i=0; i<LENGTH_MEMO_SENSOR_DATA-1; i++) {
				memo.t_gyro[i] = memo.t_gyro[i+1];
				memo.g_x[i] = memo.g_x[i+1];
				memo.g_y[i] = memo.g_y[i+1];
				memo.g_z[i] = memo.g_z[i+1];
				memo.gz_level[i] = memo.gz_level[i+1];
			}
			memo.t_gyro[LENGTH_MEMO_SENSOR_DATA-1] = t_gyro;
			memo.g_x[LENGTH_MEMO_SENSOR_DATA-1] = w_b[0];
			memo.g_y[LENGTH_MEMO_SENSOR_DATA-1] = w_b[1];
			memo.g_z[LENGTH_MEMO_SENSOR_DATA-1] = w_b[2];
			memo.gz_level[LENGTH_MEMO_SENSOR_DATA-1] = wd1;
		}

	}

	// -- End Put into memo


	// -- Step detection and PDR
	if (avail_accel) {
		if_detect_step = 0;
		if_update_step = 0;

		if (memo.i_ma == LENGTH_MEMO_MA) {
			if_detect_step = 1;
		}
		if (if_detect_step) {
			ma_mid1 = memo.ma[EPH_COMPARE_SIDE];
			min_t1 = min_f(memo.ma, LENGTH_MEMO_MA);
			if (fabs(ma_mid1-min_t1)<10e-6 && ma_mid1<= TH_A_STEP2){
				i_mid_in_memo = memo.i_accel - EPH_COMPARE_SIDE;  //
				t_mid1 = memo.t_accel[i_mid_in_memo-1];

				if_update_step = 1;
				if (memo.i_t_step) {
					if (t_mid1-memo.t_step[memo.i_t_step-1]<TH_MIN_T_DIFF_STEP){
						if_update_step = 0;
					}
				}

				if (if_update_step) {
					sol.count_rec_step ++;
					fprintf(sol.f_out_rec_step, "%.4f %.4f\n", t_mid1, ma_mid1);

					memo.i_t_step ++;
					if(memo.i_t_step <=LENGTH_MEMO_T_STEP){
						memo.t_step[memo.i_t_step-1] = t_mid1;
					} else {
						memo.i_t_step = LENGTH_MEMO_T_STEP;
						for (i=0; i<LENGTH_MEMO_T_STEP-1; i++) {
							memo.t_step[i] = memo.t_step[i+1];
						}
						memo.t_step[LENGTH_MEMO_T_STEP-1] = t_mid1;
					}

					nav.t_step0 = nav.t_step1;
					nav.t_step1 = t_mid1;
					nav.t_s_step = nav.t_step1 - nav.t_step0;

					if_update_pos = 1;
				}
			}

			if (if_update_pos) {    // Update PDR POS
				nav.pos[0] += SL*cos(nav.heading_step)/RM;
				nav.pos[1] += SL*sin(nav.heading_step)/RN_COS_LON;
				nav.pos[2] = nav.pos[2];

				// KF prediction
				// ...

				//
				for (i=0; i<LENGTH_MEMO_SENSOR_DATA; i++){
					t_diff_g_a[i] = fabs(t_mid1 - memo.t_gyro[i]);
				}
				min_double(t_diff_g_a, min_t_heading_in_memo, LENGTH_MEMO_SENSOR_DATA);
				i_heading_in_memo = (int)(min_t_heading_in_memo[2]);
				nav.heading_step = memo.heading_platform[i_heading_in_memo];

				memo.i_pos_pdr ++;
				if(memo.i_pos_pdr < LENGTH_MEMO_POS_PDR){
					memo.t_pos_pdr[memo.i_pos_pdr-1] = t_mid1;
					memo.pos_pdr_lat[memo.i_pos_pdr-1] = nav.pos[0];
					memo.pos_pdr_lon[memo.i_pos_pdr-1] = nav.pos[1];
					memo.pos_pdr_hei[memo.i_pos_pdr-1] = nav.pos[2];
				} else {
					memo.i_pos_pdr = LENGTH_MEMO_POS_PDR;
					for (i=0; i<LENGTH_MEMO_POS_PDR-1; i++) {
						memo.t_pos_pdr[i] = memo.t_pos_pdr[i+1];
						memo.pos_pdr_lat[i] = memo.pos_pdr_lat[i+1];
						memo.pos_pdr_lon[i] = memo.pos_pdr_lon[i+1];
						memo.pos_pdr_hei[i] = memo.pos_pdr_hei[i+1];
					}
					memo.t_pos_pdr[LENGTH_MEMO_POS_PDR-1] = t_mid1;
					memo.pos_pdr_lat[LENGTH_MEMO_POS_PDR-1] = nav.pos[0];
					memo.pos_pdr_lon[LENGTH_MEMO_POS_PDR-1] = nav.pos[1];
					memo.pos_pdr_hei[LENGTH_MEMO_POS_PDR-1] = nav.pos[2];
				}
				//sol.count_rec_pos ++;
				//fprintf(sol.f_out_rec_pos, "%.4f %.8f %.8f %.3f\n",
				//		memo.t_pos_pdr[memo.i_pos_pdr-1], memo.pos_pdr_lat[memo.i_pos_pdr-1]/D2R,
				//		memo.pos_pdr_lon[memo.i_pos_pdr-1]/D2R, memo.pos_pdr_hei[memo.i_pos_pdr-1]);

			} // end if_update_pos


		} // end if_detect_step

	} // end if avail_accel
	// -- End  Step detection and PDR

	// -- Heading
	if (avail_gyro) {
		ts_gyro = 0.00;
		if (memo.i_gyro>1) {
			ts_gyro = memo.t_gyro[memo.i_gyro-1]-memo.t_gyro[memo.i_gyro-2];
			printf("ts_gyro = %.4f\n", ts_gyro);
		}

		if (memo.i_gyro < LENGTH_MEMO_SENSOR_DATA) {
			memo.heading_platform[memo.i_gyro-1] = nav.heading_platform;
		} else {

			for (i=0; i<LENGTH_MEMO_SENSOR_DATA-1; i++) {
				memo.heading_platform[i] = memo.heading_platform[i+1];
			}
			memo.heading_platform[LENGTH_MEMO_SENSOR_DATA-1] = nav.heading_platform;
		}

		// QS
		if (memo.i_gyro < EPH_DETECT_QS) {
			find_max_min_double(memo.gz_level, max_min_qs_detect, memo.i_gyro);
		} else {
			find_max_min_double((memo.gz_level)+memo.i_gyro-1-EPH_DETECT_QS, max_min_qs_detect, EPH_DETECT_QS);
		}
		diff_max_min_gz = max_min_qs_detect[0] - max_min_qs_detect[1];

		if_qs = 0;
		if (diff_max_min_gz < TH_DETECT_QS_GYRO) {
			if_qs = 1;
		}

		if (!if_qs) {
			nav.heading_platform += memo.gz_level[memo.i_gyro-1]*ts_gyro;
			nav.heading_platform = center_heading(nav.heading_platform);
		}

		sol.count_rec_heading ++;
		fprintf(sol.f_out_rec_heading, "%.4f %.4f\n", memo.t_gyro[memo.i_gyro-1], memo.heading_platform[memo.i_gyro-1]);

	}
	// -- End Heading


}





/**
 * @brief		Function to calculate gyro biases from sensor readings
 * @details
 * @param[in]	t_cur: time stamp, Unit: secs
 * @param[in]	w_b[3]: gyro readings, Unit: deg/s
 * @param[in]	f_b[3]: accel readings, Unit: m/s^2
 * @param[in]	m_b[3]: mag readings, Unit: mG
 * @param[in]	flag_mag: flag of mag
 * @param[in]	flag_accuracy
 * @param[in]	seed_bias: If seed_availability == 1, then use this array as the initial gyro biases, Unit: deg/s;
 *                         Otherwise, the initial gyro biases will start from [0; 0; 0] inside the function;
 * @param[in]	seed_availability
 * @param[out]	gBias[3]: gyro biases, Unit: deg/s
 * @param[out]	availability[1]: availability of gyro biases
 * @return 		NONE
 */

void inv_gyro_bias_tracker(DOUBLE64 t_cur, FLOAT32 w_b[3], FLOAT32 f_b[3], FLOAT32 m_b[3], UINT8 flag_mag, UINT8 accuracy_flag, FLOAT32 seed_bias[3], UINT8 seed_availability, FLOAT32 gBias[3], UINT8* availability)
{
	static UINT8 fitst_time_flag = 0;
	static UINT8 fitst_mag_flag = 0;
	static Nav nav;
	static FLOAT32 gyro_bias[3];

	UINT16 i;
	UINT8 tag_accel, tag_mag, neph_QS;
	UINT8 tag_condition = 0;
	FLOAT32 ts, ng0, norma;
	FLOAT32 curG[3],wien[3], zeta[3], f_n[3], f_n_hat[3], m_n[3], m_n_hat[3];
	DOUBLE64 pos[3];
	FLOAT32 F[NStates][NStates], G[NStates][6], PHI[NStates][NStates], Fts[NStates][NStates],  GT[6][NStates], GQ[NStates][NStates], GQGT[NStates][NStates];
	FLOAT32 PHIGQGT[NStates][NStates],PHIT[NStates][NStates], GQGTPHIT[NStates][NStates], PHIP[NStates][NStates], PHIPPHIT[NStates][NStates], Qk[6][6];
	FLOAT32 Rk3[3][3], att[3], Cbn[3][3], CbnRk3[3][3], CbnT[3][3], Zk3[3], Fn1[3][3],inno3[3], Hk3xk[3], Hk2xk[3], Rk2[3][3], CbnRk2[3][3];
	FLOAT32 EYE6[NStates][NStates], Hk3[3][NStates], Hk2[3][NStates];
	FLOAT32 xk[NStates] = {0.0};
	UINT8 if_QS = 0;
	FLOAT32 accl_accu[3], mag_acc[3];

	FLOAT32* pCbn[3], *pF[NStates], *pG[NStates], *pPHI[NStates], *pFts[NStates], *pEYE6[NStates], *pGT[NStates];
	FLOAT32* pGQ[NStates], *pGQGT[NStates], *pPHIGQGT[NStates], *pPHIT[NStates], *pGQGTPHIT[NStates], *pQk[6], *pPHIP[NStates], *pPHIPPHIT[NStates];
	FLOAT32* pHk3[3], *pRk3[3], *pCbnRk3[3], *pCbnT[3], *pHk2[3], *pRk2[3], *pCbnRk2[3];

	w_b[0] *= D2R;
	w_b[1] *= D2R;
	w_b[2] *= D2R;

	if (!fitst_time_flag){
		nav.pos[0] = INI_POS[0];
		nav.pos[1] = INI_POS[1];
		nav.pos[2] = INI_POS[2];
		fitst_time_flag = 1;

		if (seed_availability==1) {
			gyro_bias[0] = seed_bias[0] * D2R;
			gyro_bias[1] = seed_bias[1] * D2R;
			gyro_bias[2] = seed_bias[2] * D2R;
		} else {
			gyro_bias[0] = 0.0;
			gyro_bias[1] = 0.0;
			gyro_bias[2] = 0.0;
		}

		nav.P[0][0] = ini_att_var[0] * ini_att_var[0];
		nav.P[1][1] = ini_att_var[1] * ini_att_var[1];
		nav.P[2][2] = ini_att_var[2] * ini_att_var[2];
		nav.P[3][3] = ini_bg_var[0] * ini_bg_var[0];
		nav.P[4][4] = ini_bg_var[1] * ini_bg_var[1];
		nav.P[5][5] = ini_bg_var[2] * ini_bg_var[2];

		nav.Q[0][0] = q_ARW0[0] * q_ARW0[0];
		nav.Q[1][1] = q_ARW0[1] * q_ARW0[1];
		nav.Q[2][2] = q_ARW0[2] * q_ARW0[2];
		nav.Q[3][3] = q_bg0[0] * q_bg0[0];
		nav.Q[4][4] = q_bg0[1] * q_bg0[1];
		nav.Q[5][5] = q_bg0[2] * q_bg0[2];

		nav.t0 = t_cur;
		ts = 1.0/SENSORS_RATE_MOBILE;

		// Initialize memory
		initial_memo(&nav.memo, &nav.win);

		// Initial Alignment
		att[0] = cal_roll(f_b);
		att[1] = cal_pitch(f_b);
		att[2] = 0;

		Euler2Dcm(att, Cbn);
		Dcm2Quat(Cbn, nav.qbn);
		multiply_matrix_with_vector_3x3(Cbn, m_b, nav.mag_n00);
	}
	else {
		ts = (FLOAT32)(t_cur - nav.t);
	}

	nav.t = t_cur;
	if (nav.t - nav.t0 >= 2){
		(*availability) = 1;
	} else {
		(*availability) = 0;
	}

	compensate_b(w_b, gyro_bias);
	curG[0] = w_b[0]*ts;
	curG[1] = w_b[1]*ts;
	curG[2] = w_b[2]*ts;

	Quat2Dcm(nav.qbn, Cbn);
	pos[0] = nav.pos[0];
	pos[1] = nav.pos[1];
	pos[2] = nav.pos[2];

	// Attitude Mechnization
	ng0 = normalGravity(pos[0], pos[2]);
	wien[0] = WGS84_we * cos(nav.pos[0]);
	wien[1] = 0.0;
	wien[2] = -1.0 * WGS84_we * sin(nav.pos[0]);
	zeta[0] = -1.0*ts*wien[0];
	zeta[1] = -1.0*ts*wien[1];
	zeta[2] = -1.0*ts*wien[2];		// Neglecting wenn

	Attitude_Mechanization_simple_float(curG, nav.qbn, nav.qbn, Cbn, att, zeta);
	ChangeArrayMemoryLayout_float(3,3,pCbn,(FLOAT32 *)Cbn,0);

	// Strategy
	put_into_memo(&nav.memo, &nav.win, t_cur, w_b, f_b, m_b, flag_mag);
	if (nav.memo.i_memo2 && nav.memo.ind_std[nav.memo.i_memo2-1]) {  // Slow
		if (!nav.memo.ind_turn_ang[nav.memo.i_memo1-1]){
			tag_condition = 1;      // Slow, straight line
		} else{
			tag_condition = 2;		// Slow, turn
		}
	}

	put_into_memo_group2(&nav.memo, &nav.win, t_cur, w_b, f_b, m_b, ng0, flag_mag, tag_condition);

	if (!tag_condition) {
		accl_accu[0] = accelAccuracy0[0]; accl_accu[1] = accelAccuracy0[1]; accl_accu[2] = accelAccuracy0[2];
		mag_acc[0] = magAccuracy0[0]; mag_acc[1] = magAccuracy0[1]; mag_acc[2] = magAccuracy0[2];
		nav.Q[0][0] = q_ARW0[0] * q_ARW0[0]; nav.Q[1][1] = q_ARW0[1] * q_ARW0[1]; nav.Q[2][2] = q_ARW0[2] * q_ARW0[2];
		nav.Q[3][3] = q_bg0[0] * q_bg0[0]; nav.Q[4][4] = q_bg0[1] * q_bg0[1]; nav.Q[5][5] = q_bg0[2] * q_bg0[2];
	} else if (tag_condition == 1) {
		accl_accu[0] = accelAccuracy1[0]; accl_accu[1] = accelAccuracy1[1]; accl_accu[2] = accelAccuracy1[2];
		mag_acc[0] = magAccuracy1[0]; mag_acc[1] = magAccuracy1[1]; mag_acc[2] = magAccuracy1[2];
		nav.Q[0][0] = q_ARW1[0] * q_ARW1[0]; nav.Q[1][1] = q_ARW1[1] * q_ARW1[1]; nav.Q[2][2] = q_ARW1[2] * q_ARW1[2];
		nav.Q[3][3] = q_bg1[0] * q_bg1[0]; nav.Q[4][4] = q_bg1[1] * q_bg1[1]; nav.Q[5][5] = q_bg1[2] * q_bg1[2];
	} else {
		accl_accu[0] = accelAccuracy2[0]; accl_accu[1] = accelAccuracy2[1]; accl_accu[2] = accelAccuracy2[2];
		nav.Q[0][0] = q_ARW2[0] * q_ARW2[0]; nav.Q[1][1] = q_ARW2[1] * q_ARW2[1]; nav.Q[2][2] = q_ARW2[2] * q_ARW2[2];
		nav.Q[3][3] = q_bg2[0] * q_bg2[0]; nav.Q[4][4] = q_bg2[1] * q_bg2[1]; nav.Q[5][5] = q_bg2[2] * q_bg2[2];
	}

	// Prediction
	GetSystemModel_6(Cbn, (FLOAT32* )tao_bg ,wien, F, G);

	ChangeArrayMemoryLayout_float(NStates,NStates,nav.pP,(FLOAT32 *)nav.P,0);   //P
	ChangeArrayMemoryLayout_float(NStates,NStates,nav.pQ,(FLOAT32 *)nav.Q,0);   //Q
	ChangeArrayMemoryLayout_float(NStates,NStates,pF,(FLOAT32 *)F,0);		//F
	ChangeArrayMemoryLayout_float(NStates,NStates,pG,(FLOAT32 *)G,0);		//G
	ChangeArrayMemoryLayout_float(NStates,NStates,pPHI,(FLOAT32 *)PHI,1);	//Phi
	ChangeArrayMemoryLayout_float(NStates,NStates,pFts,(FLOAT32 *)Fts,0);   //F*ts
	ChangeArrayMemoryLayout_float(NStates,NStates,pEYE6,(FLOAT32 *)EYE6,1);	//I
	for (i=0; i<NStates; i++){
		EYE6[i][i] = 1.0;
	}
	ChangeArrayMemoryLayout_float(NStates,NStates,pGT,(FLOAT32 *)GT,0);     //G'
	ChangeArrayMemoryLayout_float(NStates,NStates,pGQ,(FLOAT32 *)GQ,0);		//GQ
	ChangeArrayMemoryLayout_float(NStates,NStates,pGQGT,(FLOAT32 *)GQGT,0);  //GQG'
	ChangeArrayMemoryLayout_float(NStates,NStates,pPHIGQGT,(FLOAT32 *)PHIGQGT,0);     //Phi*GQG'
	ChangeArrayMemoryLayout_float(NStates,NStates,pPHIT,(FLOAT32 *)PHIT,0);  //Phi'
	ChangeArrayMemoryLayout_float(NStates,NStates,pGQGTPHIT,(FLOAT32 *)GQGTPHIT,0);	  //GQG'*Phi'
	ChangeArrayMemoryLayout_float(NStates,NStates,pQk,(FLOAT32 *)Qk,0);		 //Qk  = 0.5*(Phi*GQGt + GQGt*Phi') * ts;
	ChangeArrayMemoryLayout_float(NStates,NStates,pPHIP,(FLOAT32 *)PHIP,0);			  //Phi*P
	ChangeArrayMemoryLayout_float(NStates,NStates,pPHIPPHIT,(FLOAT32 *)PHIPPHIT,0);	  //Phi*P*Phi'


	multiply_scalar_with_matrix_nxn(ts, pF, pFts,NStates, NStates);		//Fts = F*ts
	add_matrices_nxn(pEYE6, pFts, NStates, NStates, pPHI);				//Phi = I + F*ts
	matrix_transpose_nxn_float(pG, NStates, 6, pGT);						//G' = G'
	multiply_matrices_nxn_float(pG, nav.pQ, NStates, 6, 6, pGQ);				//GQ = G*Q
	multiply_matrices_nxn_float(pGQ, pGT, NStates, 6, NStates, pGQGT);	//GQG' = G*Q*G'
	multiply_matrices_nxn_float(pPHI, pGQGT, NStates, NStates, NStates, pPHIGQGT);	//PhiGQG' = Phi*GQG'
	matrix_transpose_nxn_float(pPHI, NStates, NStates, pPHIT);			//PHI'
	multiply_matrices_nxn_float(pGQGT, pPHIT, NStates, NStates, NStates, pGQGTPHIT);	//GQG'Phi' = GQG'*Phi';
	add_matrices_nxn(pPHIGQGT, pGQGTPHIT, NStates, NStates, pQk);         //Qk = Phi*GQGt + GQGt*Phi'
	multiply_scalar_with_matrix_nxn(0.5*ts, pQk, pQk, NStates, NStates);  //Qk = 0.5*ts * Qk

	multiply_matrices_nxn_float(pPHI, nav.pP, NStates, NStates, NStates, pPHIP);			//Phi*P
	multiply_matrices_nxn_float(pPHIP, pPHIT, NStates, NStates, NStates, pPHIPPHIT);	//Phi*P*Phi'
	add_matrices_nxn(pPHIPPHIT, pQk, NStates, NStates, nav.pP);				//P = Phi*P*Phi'+ Qk

	// UPDATE
	norma = sqrt(f_b[0]*f_b[0]+f_b[1]*f_b[1]+f_b[2]*f_b[2]) - ng0;

	tag_accel = 1;
	if (nav.memo.ind_large_accl[nav.memo.i_memo1-1]){
		tag_accel = 0;
	}
	tag_mag = 1;
	if (!flag_mag || tag_condition == 2 || !nav.memo.ind_QSMF[nav.memo.i_memo1-1]) {
		tag_mag = 0;
	}

	// Accelerometer updates
	if (tag_accel)
	{
		ChangeArrayMemoryLayout_float(3,NStates,pHk3,(FLOAT32 *)Hk3,1);
		ChangeArrayMemoryLayout_float(3,3,pRk3,(FLOAT32 *)Rk3,0);		   //Rk = diag
		ChangeArrayMemoryLayout_float(3,3,pCbnRk3,(FLOAT32 *)CbnRk3,0);  //Cbn*Rk
		ChangeArrayMemoryLayout_float(3,3,pCbnT,(FLOAT32 *)CbnT,0);  //Cbn'

		multiply_matrix_with_vector_nxn(pCbn, 3, 3, f_b, f_n);       //fn = Cbn*fb
		f_n_hat[0] = 0.0;
		f_n_hat[1] = 0.0;
		f_n_hat[2] = -1*ng0;
		make_skew_symmetric_matrix(f_n_hat, Fn1);

		Hk3[0][0] = Fn1[0][0]; Hk3[0][1] = Fn1[0][1]; Hk3[0][2] = Fn1[0][2];
		Hk3[1][0] = Fn1[1][0]; Hk3[1][1] = Fn1[1][1]; Hk3[1][2] = Fn1[1][2];
		Hk3[2][0] = Fn1[2][0]; Hk3[2][1] = Fn1[2][1]; Hk3[2][2] = Fn1[2][2];

		Zk3[0] = f_n[0]-f_n_hat[0];
		Zk3[1] = f_n[1]-f_n_hat[1];
		Zk3[2] = f_n[2]-f_n_hat[2];

		diag2_sq(accl_accu, 3, pRk3);
		multiply_matrices_nxn_float(pCbn, pRk3, 3, 3, 3, pCbnRk3);
		matrix_transpose_nxn_float(pCbn, 3, 3, pCbnT);
		multiply_matrices_nxn_float(pCbnRk3, pCbnT, 3, 3, 3, pRk3);   //Rk=Cbn*Rk*Cbn'
		multiply_matrix_with_vector_nxn(pHk3, 3, 3, xk, Hk3xk);       //Hx = H*x

		inno3[0] = Zk3[0] - Hk3xk[0];
		inno3[1] = Zk3[1] - Hk3xk[1];
		inno3[2] = Zk3[2] - Hk3xk[2];

		KF_update1_float(3, xk, nav.pP, inno3, pHk3, pRk3, NStates);
	}

	// Magnetometer updates
	if (tag_mag)
	{
		neph_QS = (UINT8) (SENSORS_RATE_MOBILE/2);
		if (nav.memo.i_memo1>neph_QS){
			if_QS = 1;
			for (i=0; i<neph_QS; i++) {
				if (fabs(nav.memo.ind_QSMF[nav.memo.i_memo1-2-i]-0)>10e-4){
					if_QS = 0;
					break;
				}
			}
		}
		if (!fitst_mag_flag || if_QS == 1){
			multiply_matrix_with_vector_3x3(Cbn, m_b, nav.mag_n00);
			fitst_mag_flag = 1;
		}

		multiply_matrix_with_vector_3x3(Cbn, m_b, m_n);
		m_n_hat[0] = nav.mag_n00[0];
		m_n_hat[1] = nav.mag_n00[1];
		m_n_hat[2] = nav.mag_n00[2];

		ChangeArrayMemoryLayout_float(3,NStates,pHk2,(FLOAT32 *)Hk2,1);
		ChangeArrayMemoryLayout_float(3,3,pRk2,(FLOAT32 *)Rk2,0);		   //Rk = diag
		ChangeArrayMemoryLayout_float(3,3,pCbnRk2,(FLOAT32 *)CbnRk2,0);  //Cbn*Rk
		ChangeArrayMemoryLayout_float(3,3,pCbnT,(FLOAT32 *)CbnT,0);  //Cbn'

		multiply_matrix_with_vector_nxn(pCbn, 3, 3, m_b, m_n);       //fn = Cbn*fb
		Zk3[0] = m_n[0]-m_n_hat[0];
		Zk3[1] = m_n[1]-m_n_hat[1];
		Zk3[2] = m_n[2]-m_n_hat[2];

		make_skew_symmetric_matrix(m_n_hat, Fn1);

		Hk2[0][0] = Fn1[0][0]; Hk2[0][1] = Fn1[0][1]; Hk2[0][2] = Fn1[0][2];
		Hk2[1][0] = Fn1[1][0]; Hk2[1][1] = Fn1[1][1]; Hk2[1][2] = Fn1[1][2];
		Hk2[2][0] = Fn1[2][0]; Hk2[2][1] = Fn1[2][1]; Hk2[2][2] = Fn1[2][2];

		diag2_sq(mag_acc, 3, pRk2);
		multiply_matrices_nxn_float(pCbn, pRk2, 3, 3, 3, pCbnRk2);
		matrix_transpose_nxn_float(pCbn, 3, 3, pCbnT);
		multiply_matrices_nxn_float(pCbnRk2, pCbnT, 3, 3, 3, pRk2);   //Rk=Cbn*Rk*Cbn'

		multiply_matrix_with_vector_nxn(pHk2, 3, 3, xk, Hk2xk);       //Hx = H*x

		inno3[0] = Zk3[0] - Hk2xk[0];
		inno3[1] = Zk3[1] - Hk2xk[1];
		inno3[2] = Zk3[2] - Hk2xk[2];

		KF_update1_float(3, xk, nav.pP, inno3, pHk2, pRk2, NStates);
	}

	// Feedback
	INSNavFeedBack_Attitude_float(xk, nav.qbn, Cbn, att, gyro_bias);

	gBias[0] = gyro_bias[0]/D2R;
	gBias[1] = gyro_bias[1]/D2R;
	gBias[2] = gyro_bias[2]/D2R;
}


void GetSystemModel_6(FLOAT32 Cbn[3][3], FLOAT32 tao[3], FLOAT32 winn[3], FLOAT32 F[NStates][NStates], FLOAT32 G[NStates][6])
{
	UINT16 i, j;
	for(i=0; i<NStates; i++){
		for (j=0; j<NStates; j++){
			F[i][j] = 0.0;
			G[i][j] = 0.0;
		}
	}
	F[0][1] = winn[2];
	F[1][0] = -winn[2]; F[1][2] = winn[0];
	F[2][1] = -winn[0];

	F[0][3] = -Cbn[0][0]; F[0][4] = -Cbn[0][1]; F[0][5] = -Cbn[0][2];
	F[1][3] = -Cbn[1][0]; F[1][4] = -Cbn[1][1]; F[1][5] = -Cbn[1][2];
	F[2][3] = -Cbn[2][0]; F[2][4] = -Cbn[2][1]; F[2][5] = -Cbn[2][2];

	F[3][3] = -1.0/ tao[0];
	F[4][4] = -1.0/ tao[1];
	F[5][5] = -1.0/ tao[2];

	G[0][0] = -Cbn[0][0]; G[0][1] = -Cbn[0][1]; G[0][2] = -Cbn[0][2];
	G[1][0] = -Cbn[1][0]; G[1][1] = -Cbn[1][1]; G[1][2] = -Cbn[1][2];
	G[2][0] = -Cbn[2][0]; G[2][1] = -Cbn[2][1]; G[2][2] = -Cbn[2][2];
	G[3][3] = 1.0;
	G[4][4] = 1.0;
	G[5][5] = 1.0;
}

void initial_memo_nav(MemoIndoor * memo){
	UINT8 i;

}


float center_heading(float heading){
	if (heading > PI) {
		heading = heading - 2*PI;
	} else if (heading < -PI) {
		heading = heading + 2*PI;
	}

	return heading;
}


void initial_memo(Memo* memo, Wind* win){
	UINT8 i;
	for (i=0; i<N_EPH_MEMO; i++){
		memo->t[i] = 0.0;
		memo->gyro[i][0] = 0.0;
		memo->gyro[i][1] = 0.0;
		memo->gyro[i][2] = 0.0;
		memo->accel[i][0] = 0.0;
		memo->accel[i][1] = 0.0;
		memo->accel[i][2] = 0.0;
		memo->normm[i] = 0.0;
		memo->std[i] = 0.0;
		memo->gz_leveled[i] = 0.0;
		memo->turn_ang[i] = 0.0;

		memo->ind_std[i] = 0;
		memo->ind_turn_ang[i] = 0;
		memo->ind_QSMF[i] = 0;
	}
	memo->i_win = 0;
	memo->i_memo1 = 0;
	memo->i_memo2 = 0;

	for (i=0; i<N_EPH_WIN; i++){
		win->norma[i] = 0.0;
	}
}

void put_into_memo(Memo* memo, Wind* win, FLOAT32 t, FLOAT32 w_b[3], FLOAT32 f_b[3], FLOAT32 m_b[3], UINT8 flag_mag){
	FLOAT32 normm1, norma1, roll1, pitch1, sr1, cr1, sp1, cp1, wd1, res_std_norma1;
	FLOAT32 memo_temp[N_EPH_MEMO],memo_temp1[N_EPH_MEMO],memo_temp2[N_EPH_MEMO],memo_temp3[N_EPH_MEMO],memo_temp4[N_EPH_MEMO];
	FLOAT32 memo_temp5[N_EPH_MEMO],memo_temp6[N_EPH_MEMO],memo_temp7[N_EPH_MEMO],memo_temp8[N_EPH_MEMO];
	FLOAT32 win_temp[N_EPH_WIN];
	UINT8 i;
	UINT8 ind_turn1, ind_std1;
	FLOAT32 turn_ang1 = 0.0;

	memo->i_win ++;
	memo->i_memo1 ++;
	norma1 = sqrt(f_b[0]*f_b[0]+f_b[1]*f_b[1]+f_b[2]*f_b[2]);
	roll1 = cal_roll(f_b);
	pitch1 = cal_pitch(f_b);
	sr1 = sin(roll1); cr1 = cos(roll1);
    sp1 = sin(pitch1); cp1 = cos(pitch1);
	wd1 = -sp1*w_b[0] + sr1*cp1*w_b[1] + cr1*cp1*w_b[2];
	if (flag_mag){
		normm1 = sqrt(m_b[0]*m_b[0]+m_b[1]*m_b[1]+m_b[2]*m_b[2]);
	} else {
		if (memo->i_memo1 != 1)
			normm1 = memo->normm[memo->i_memo1-2];
		else
			normm1 = 0.0;
	}

	if (memo->i_memo1 < N_EPH_MEMO){
		memo->t[memo->i_memo1-1] = t;
		memo->gyro[memo->i_memo1-1][0] = w_b[0];
		memo->gyro[memo->i_memo1-1][1] = w_b[1];
		memo->gyro[memo->i_memo1-1][2] = w_b[2];
		memo->accel[memo->i_memo1-1][0] = f_b[0];
		memo->accel[memo->i_memo1-1][1] = f_b[1];
		memo->accel[memo->i_memo1-1][2] = f_b[2];
		memo->normm[memo->i_memo1-1] = normm1;
		memo->gz_leveled[memo->i_memo1-1] = wd1;
	} else {
		memo->i_memo1 = N_EPH_MEMO;
		for (i=0; i<N_EPH_MEMO-1; i++){
			memo_temp[i] = memo->t[i+1];
			memo_temp1[i] = memo->gyro[i+1][0];
			memo_temp2[i] = memo->gyro[i+1][1];
			memo_temp3[i] = memo->gyro[i+1][2];
			memo_temp4[i] = memo->accel[i+1][0];
			memo_temp5[i] = memo->accel[i+1][1];
			memo_temp6[i] = memo->accel[i+1][2];
			memo_temp7[i] = memo->normm[i+1];
			memo_temp8[i] = memo->gz_leveled[i+1];
		}
		memo_temp[N_EPH_MEMO-1] = t;
		memo_temp1[N_EPH_MEMO-1] = w_b[0];
		memo_temp2[N_EPH_MEMO-1] = w_b[1];
		memo_temp3[N_EPH_MEMO-1] = w_b[2];
		memo_temp4[N_EPH_MEMO-1] = f_b[0];
		memo_temp5[N_EPH_MEMO-1] = f_b[1];
		memo_temp6[N_EPH_MEMO-1] = f_b[2];
		memo_temp7[N_EPH_MEMO-1] = normm1;
		memo_temp8[N_EPH_MEMO-1] = wd1;

		for (i=0; i<N_EPH_MEMO; i++){
			memo->t[i] = memo_temp[i];
			memo->gyro[i][0] = memo_temp1[i];
			memo->gyro[i][1] = memo_temp2[i];
			memo->gyro[i][2] = memo_temp3[i];
			memo->accel[i][0] = memo_temp4[i];
			memo->accel[i][1] = memo_temp5[i];
			memo->accel[i][2] = memo_temp6[i];
			memo->normm[i]	  = memo_temp7[i];
			memo->gz_leveled[i] = memo_temp8[i];
		}
	}

	for (i=0; i<memo->i_memo1-1; i++){
		turn_ang1 += memo->gz_leveled[i] * 1.0/SENSORS_RATE_MOBILE;
	}
	ind_turn1 = 0;
	if (fabs(turn_ang1)>th_turn_ang) {
		ind_turn1 = 1;
	}

	if (memo->i_memo1 < N_EPH_MEMO){
		memo->turn_ang[memo->i_memo1-1] = turn_ang1;
		memo->ind_turn_ang[memo->i_memo1-1] = ind_turn1;
	} else {
		memo->i_memo1 = N_EPH_MEMO;
		for (i=0; i<N_EPH_MEMO-1; i++){
			memo_temp[i] = memo->turn_ang[i+1];
			memo_temp1[i] = memo->ind_turn_ang[i+1];
		}
		memo_temp[N_EPH_MEMO-1] = turn_ang1;
		memo_temp1[N_EPH_MEMO-1] = ind_turn1;

		for (i=0; i<N_EPH_MEMO; i++){
			memo->turn_ang[i] = memo_temp[i];
			memo->ind_turn_ang[i] = memo_temp1[i];
		}
	}

	// -------------------------
    // This is for the values that need to go to window THEN go to memory,
    // like std
	if (memo->i_win < N_EPH_WIN){
		win->norma[memo->i_win-1] = norma1;
	} else{
		memo->i_win = N_EPH_WIN;
		for (i=0; i<N_EPH_WIN-1; i++){
			win_temp[i] = win->norma[i+1];
		}
		win_temp[N_EPH_WIN-1] = norma1;
		for (i=0; i<N_EPH_WIN; i++){
			win->norma[i] = win_temp[i];
		}

		res_std_norma1 = cal_std_float(win->norma,N_EPH_WIN);
		ind_std1 = 0;
		if (res_std_norma1 < th_acc_std) {
			ind_std1 = 1;
		}
		memo->i_memo2 ++;

		if (memo->i_memo2 < N_EPH_MEMO) {
			memo->std[memo->i_memo2-1] = res_std_norma1;
			memo->ind_std[memo->i_memo2-1] = ind_std1;
		} else {
			memo->i_memo2 = N_EPH_MEMO;
			for (i=0; i<N_EPH_MEMO-1; i++){
				memo_temp[i] = memo->std[i+1];
				memo_temp1[i] = memo->ind_std[i+1];
			}
			memo_temp[N_EPH_MEMO-1] = res_std_norma1;
			memo_temp1[N_EPH_MEMO-1] = ind_std1;
			for (i=0; i<N_EPH_MEMO; i++){
				memo->std[i] = memo_temp[i];
				memo->ind_std[i] = memo_temp1[i];
			}
		}
	}
}

void put_into_memo_group2(Memo* memo, Wind* win, FLOAT32 t, FLOAT32 w_b[3], FLOAT32 f_b[3], FLOAT32 m_b[3], FLOAT32 ng0, UINT8 flag_mag, UINT8 tag_condition){
	UINT8 i, nv;
	UINT8 ind_qsmf1, ind_large_accl1;
	FLOAT32 TT[SENSORS_RATE_MOBILE] = {0.0};
	FLOAT32 max_min[2] = {0.0};
	FLOAT32 diff_max_min  = 0.0;
	FLOAT32 memo_temp[N_EPH_MEMO],memo_temp2[N_EPH_MEMO], norma1;
	FLOAT32 th_norm_a_accel, th_detect_QSMF;

	// Add the conditions here
	if (!tag_condition) {
		th_detect_QSMF = th_detect_QSMF0;
		th_norm_a_accel = th_norm_a_accel0;
	} else if (tag_condition == 1) {
		th_detect_QSMF = th_detect_QSMF1;
		th_norm_a_accel = th_norm_a_accel1;
	} else {
		th_detect_QSMF = th_detect_QSMF2;
		th_norm_a_accel = th_norm_a_accel2;
	}

	//

	if (memo->i_memo1 <= SENSORS_RATE_MOBILE){
		nv = memo->i_memo1;
		for (i=0; i<nv; i++)
		{
			TT[i] = memo->normm[i];
		}
	} else{
		nv = SENSORS_RATE_MOBILE;
		for (i=0; i<nv; i++)
		{
			TT[i] = memo->normm[memo->i_memo1-nv-1+i];
		}
	}
	find_max_min_float(TT, max_min, nv);
	diff_max_min = max_min[0] - max_min[1];

	ind_qsmf1 = 0;
	if (flag_mag && diff_max_min < th_detect_QSMF0) {
		ind_qsmf1 = 1;
	}

	// Attitude Mechnization
	norma1 = sqrt(f_b[0]*f_b[0]+f_b[1]*f_b[1]+f_b[2]*f_b[2]) - ng0;
	ind_large_accl1 = 0;
	if (norma1 > th_norm_a_accel) {
		ind_large_accl1 = 1;
	}

	if (memo->i_memo1 < N_EPH_MEMO){
		memo->ind_QSMF[memo->i_memo1-1] = ind_qsmf1;
		memo->ind_large_accl[memo->i_memo1-1] = ind_large_accl1;
	} else {
		memo->i_memo1 = N_EPH_MEMO;
		for (i=0; i<N_EPH_MEMO-1; i++){
			memo_temp[i] = memo->ind_QSMF[i+1];
			memo_temp2[i] = memo->ind_large_accl[i+1];
		}
		memo_temp[N_EPH_MEMO-1] = ind_qsmf1;
		memo_temp2[N_EPH_MEMO-1] = ind_large_accl1;
		for (i=0; i<N_EPH_MEMO; i++){
			memo->ind_QSMF[i] = memo_temp[i];
			memo->ind_large_accl[i] = memo_temp2[i];
		}
	}

}

void compensate_b(FLOAT32* obs, FLOAT32* bias){
	obs[0] -= bias[0];
	obs[1] -= bias[1];
	obs[2] -= bias[2];
}

FLOAT32 cal_roll(FLOAT32* f_b){
	FLOAT32 roll = atan2(-f_b[1],-f_b[2]);
	return roll;
}
FLOAT32 cal_pitch(FLOAT32* f_b){
	FLOAT32 pitch = atan2(f_b[0],sqrt(f_b[1]*f_b[1]+f_b[2]*f_b[2]));
	return pitch;
}


void Euler2Dcm(FLOAT32 att[3], FLOAT32 Cbn[3][3])
{
	FLOAT32 cr, cp, ch, sr, sp, sh;

	cr = cos(att[0]); 	cp = cos(att[1]);	ch = cos(att[2]);
	sr = sin(att[0]); 	sp = sin(att[1]);	sh = sin(att[2]);

	Cbn[0][0] = cp * ch ;
	Cbn[0][1] = -cr*sh + sr*sp*ch;
	Cbn[0][2] = sr*sh + cr*sp*ch ;

	Cbn[1][0] = cp * sh;
	Cbn[1][1] = cr*ch + sr*sp*sh;
	Cbn[1][2] = -sr * ch + cr * sp * sh;

	Cbn[2][0] = - sp;
	Cbn[2][1] = sr * cp;
	Cbn[2][2] = cr * cp;
}

/**
 * @brief		Function to calculate quaternions from direction cosines matrix
 * @details
 * @param[in]	C: A 3x3 direction cosines matrix of type float containing attitude information
 * @param[out]	q: quaternion vector related to C in float
 * @return 		NONE
 */
void Dcm2Quat(FLOAT32 C[3][3], FLOAT32 q[4])
{
	FLOAT32 Tr = 0, Pq[4],a;
	UINT8 i, max_id;

	Tr = C[0][0] + C[1][1] + C[2][2];

	Pq[0] = 1 + Tr;
	Pq[1] = 1 + 2.0f * C[0][0] - Tr;
	Pq[2] = 1 + 2.0f * C[1][1] - Tr;
	Pq[3] = 1 + 2.0f * C[2][2] - Tr;

	max_id = 0;
	for(i=0; i<4; i++)
	{
		if (Pq[i] > Pq[max_id])	max_id = i;
	}

	switch (max_id)
	{
	  case 0:
		q[0] = 0.5f*sqrt(Pq[0]);
		a = 0.25f/q[0];
		q[1] = (C[2][1] - C[1][2])*a;
		q[2] = (C[0][2] - C[2][0])*a;
		q[3] = (C[1][0] - C[0][1])*a;
		break;
	  case 1:
		q[1] = 0.5f*sqrt(Pq[1]);
		a = 0.25f/q[1];
		q[0] = (C[2][1] - C[1][2])*a;
		q[2] = (C[1][0] + C[0][1])*a;
		q[3] = (C[0][2] + C[2][0])*a;
		break;
	  case 2:
		q[2] = 0.5f*sqrt(Pq[2]);
		a = 0.25f/q[2];
		q[0] = (C[0][2] - C[2][0])*a;
		q[1] = (C[1][0] + C[0][1])*a;
		q[3] = (C[2][1] + C[1][2])*a;
		break;
	  case 3:
		q[3] = 0.5f*sqrt(Pq[3]);
		a = 0.25f/q[3];
		q[0] = (C[1][0] - C[0][1])*a;
		q[1] = (C[0][2] + C[2][0])*a;
		q[2] = (C[2][1] + C[1][2])*a;
		break;
	}

	if (q[0] < 0)
	{
		for(i=0; i<4; i++) q[i] = -q[i];
	}
}

FLOAT32 normalGravity(double lat, FLOAT32 h){
	double a1 = 9.7803267715;
	double a2 = 0.0052790414;
	double a3 = 0.0000232718;
	double a4 = -0.000003087691089;
	double a5 = 0.000000004397731;
	double a6 = 0.000000000000721;

	double s1 = sin(lat);
	double s2 = s1 * s1;
	double s4 = s2 * s2;

	double ng = a1 * (1 + a2*s2 + a3*s4) + (a4 + a5*s2)*h + a6 * h * h;
	return (FLOAT32) ng;
}

// Num:
STATUS KF_update_indoor_float(INT16 Num, FLOAT32 *x, FLOAT32 **P, FLOAT32 *inno, FLOAT32 **H, FLOAT32 **R,INT16 NUMStates)
{
	INT16 i;
	STATUS A = FAILURE;
	FLOAT32 Ht[N_STATES][N_STATES], PHt[N_STATES][N_STATES],HPHt[N_STATES][N_STATES], U[N_STATES][N_STATES],K[N_STATES][N_STATES];
	FLOAT32 *pHt[N_STATES], *pPHt[N_STATES],*pHPHt[N_STATES], *pU[N_STATES],*pK[N_STATES];

	FLOAT32 dx[N_STATES],KH[N_STATES][N_STATES],I[N_STATES][N_STATES], IKH[N_STATES][N_STATES], IKHT[N_STATES][N_STATES];
	FLOAT32 *pKH[N_STATES],*pI[N_STATES],*pIKH[N_STATES],*pIKHT[N_STATES];

	FLOAT32 IKHP[N_STATES][N_STATES],IPIT[N_STATES][N_STATES],KT[N_STATES][N_STATES],KR[N_STATES][N_STATES],KRKT[N_STATES][N_STATES];
	FLOAT32 *pIKHP[N_STATES],*pIPIT[N_STATES],*pKT[N_STATES],*pKR[N_STATES],*pKRKT[N_STATES];


	ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pHt,(FLOAT32 *)Ht,1);
	ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pPHt,(FLOAT32 *)PHt,1);
	ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pHPHt,(FLOAT32 *)HPHt,1);
	ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pU,(FLOAT32 *)U,1);
	ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pK,(FLOAT32 *)K,1);
	ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pKH,(FLOAT32 *)KH,1);
	ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pI,(FLOAT32 *)I,1);
	ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pIKH,(FLOAT32 *)IKH,1);
	ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pIKHT,(FLOAT32 *)IKHT,1);
	ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pIKHP,(FLOAT32 *)IKHP,1);
	ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pIPIT,(FLOAT32 *)IPIT,1);
	ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pKT,(FLOAT32 *)KT,1);
	ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pKR,(FLOAT32 *)KR,1);
	ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pKRKT,(FLOAT32 *)KRKT,1);
	matrix_transpose_nxn_float(H, Num, NUMStates, pHt);  // Ht


	multiply_matrices_nxn_float(P, pHt, NUMStates, NUMStates, Num, pPHt);  // Pk(-) * Ht
	multiply_matrices_nxn_float(H, pPHt, Num, NUMStates, Num, pHPHt);    // H * Pk(-) * Ht


	add_matrices_nxn(pHPHt, R, Num, Num, pU);     // U = H * Pk(-) * Ht + R
	//A = InvertMatrix(pU, Num);
	A = MatrixInv(pU , Num);
	if (A == SUCCESS)         // U = ( H * Pk(-) * Ht + R )^-1
	{
		multiply_matrices_nxn_float(pPHt, pU, NUMStates, Num, Num, pK);  // K = Pk(-) * Ht * ( H * Pk(-) * Ht + R )^-1

		multiply_matrix_with_vector_nxn(pK, NUMStates, Num, inno, dx);       // K * (Z - H * xk(-) ) = K *inno

		multiply_matrices_nxn_float(pK, H, NUMStates, Num, NUMStates, pKH);       // K * H

		eye_matrix_float(pI, NUMStates);                      // unit matrix
		subtract_matrices_nxn(pI, pKH, NUMStates, NUMStates, pIKH);        // I - K * H

		for(i=0; i<NUMStates; i++) x[i] += dx[i];
		matrix_transpose_nxn_float(pIKH,NUMStates, NUMStates, pIKHT);
		multiply_matrices_nxn_float(pIKH, P, NUMStates, NUMStates, NUMStates, pIKHP);  //  (I - K * H) * Pk(-)
		multiply_matrices_nxn_float(pIKHP, pIKHT, NUMStates, NUMStates, NUMStates, pIPIT); // (I - K * H) * Pk(-) *(I-K*H)_t


		matrix_transpose_nxn_float(pK, NUMStates, Num, pKT);                     // Kt
		multiply_matrices_nxn_float(pK, R, NUMStates, Num, Num, pKR);              // K * R
		multiply_matrices_nxn_float(pKR, pKT, NUMStates, Num, NUMStates, pKRKT);      // K * R *Kt


		add_matrices_nxn(pIPIT, pKRKT, NUMStates, NUMStates, P);     // Pk(+) = (I - K * H) * Pk(-) *(I-K*H)_t + K * R * Kt

	}

	return A;
}

STATUS AEKF_MM_2_update_indoor_float(INT16 Num, FLOAT32 *x, FLOAT32 **P, FLOAT32 *inno, FLOAT32 **H, FLOAT32 **R,INT16 NUMStates)
{
    INT16 i, j;
    STATUS A = FAILURE;
    FLOAT32 Ht[N_STATES][N_STATES], PHt[N_STATES][N_STATES],HPHt[N_STATES][N_STATES], U[N_STATES][N_STATES],K[N_STATES][N_STATES];
    FLOAT32 *pHt[N_STATES], *pPHt[N_STATES],*pHPHt[N_STATES], *pU[N_STATES],*pK[N_STATES];

    FLOAT32 dx[N_STATES],KH[N_STATES][N_STATES],I[N_STATES][N_STATES], IKH[N_STATES][N_STATES], IKHT[N_STATES][N_STATES];
    FLOAT32 *pKH[N_STATES],*pI[N_STATES],*pIKH[N_STATES],*pIKHT[N_STATES];

    FLOAT32 IKHP[N_STATES][N_STATES],IPIT[N_STATES][N_STATES],KT[N_STATES][N_STATES],KR[N_STATES][N_STATES],KRKT[N_STATES][N_STATES];
    FLOAT32 *pIKHP[N_STATES],*pIPIT[N_STATES],*pKT[N_STATES],*pKR[N_STATES],*pKRKT[N_STATES];


    FLOAT32 CZ[2][2], *pCZ[2];
    FLOAT32 norm_inno = 0;
    FLOAT32 weight_mm = 0;

#define TH_MM_AEKF_1   20.0//SIGMA_MM_RESULT
#define TH_MM_AEKF_2  (TH_MM_AEKF_1 * 2.0)

    for (i=0; i<Num; i++) {
        norm_inno += inno[i] * inno[i];
    }
    norm_inno = sqrt(norm_inno);

    if (norm_inno < TH_MM_AEKF_1) {
        weight_mm = 1;
    }
    else if (norm_inno >= TH_MM_AEKF_1 && norm_inno < TH_MM_AEKF_2){
        weight_mm = 1000; //10 * (norm_inno / TH_MM_AEKF_1) * (norm_inno / TH_MM_AEKF_1);
    }
    else if (norm_inno >= TH_MM_AEKF_2) {
        weight_mm = 1000;
    }

    for (i=0; i<Num; i++) {
        for (j=0; j<Num; j++) {
            R[i][j] *= weight_mm;
        }
    }



    ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pHt,(FLOAT32 *)Ht,1);
    ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pPHt,(FLOAT32 *)PHt,1);
    ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pHPHt,(FLOAT32 *)HPHt,1);
    ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pU,(FLOAT32 *)U,1);
    ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pK,(FLOAT32 *)K,1);
    ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pKH,(FLOAT32 *)KH,1);
    ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pI,(FLOAT32 *)I,1);
    ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pIKH,(FLOAT32 *)IKH,1);
    ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pIKHT,(FLOAT32 *)IKHT,1);
    ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pIKHP,(FLOAT32 *)IKHP,1);
    ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pIPIT,(FLOAT32 *)IPIT,1);
    ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pKT,(FLOAT32 *)KT,1);
    ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pKR,(FLOAT32 *)KR,1);
    ChangeArrayMemoryLayout_float(N_STATES,N_STATES,pKRKT,(FLOAT32 *)KRKT,1);
    matrix_transpose_nxn_float(H, Num, NUMStates, pHt);  // Ht


    multiply_matrices_nxn_float(P, pHt, NUMStates, NUMStates, Num, pPHt);  // Pk(-) * Ht
    multiply_matrices_nxn_float(H, pPHt, Num, NUMStates, Num, pHPHt);    // H * Pk(-) * Ht


    // --------- New for AEKF -----------



//    for (i=0; i<2; i++) {
//        if (fabs(inno[i]) < 0.01) {
//            inno[i] = 0.01;
//        }
//    }
//    for (i=0; i<2; i++)
//    {
//        for (j=0; j<2; j++)
//        {
//            CZ[i][j] = (1.0/inno[j]) * (1.0/inno[i]);
//        }
//    }
//    ChangeArrayMemoryLayout_float(2,2,pCZ,(FLOAT32 *)CZ,0);
//
//    FLOAT32 xx1 = pCZ[0][0];
//    FLOAT32 xx4 = pCZ[1][1];
//
//    multiply_matrices_nxn_float(pPHt, pCZ, NUMStates, 2, 2, pK);
//
//    multiply_matrix_with_vector_nxn(pK, NUMStates, 2, inno, dx);       // K * (Z - H * xk(-) ) = K *inno
//
//    for(i=0; i<NUMStates; i++) x[i] += dx[i];
//
//    multiply_matrices_nxn_float(pK, H, NUMStates, 2, NUMStates, pKH);       // K * H
//    eye_matrix_float(pI, NUMStates);                      // unit matrix
//    subtract_matrices_nxn(pI, pKH, NUMStates, NUMStates, pIKH);        // I - K * H
//    multiply_matrices_nxn_float(pIKH, P, NUMStates, NUMStates, NUMStates, P);  //  (I - K * H) * Pk(-)
//
//    return SUCCESS;
    // ========= New for AEKF ============



    add_matrices_nxn(pHPHt, R, Num, Num, pU);     // U = H * Pk(-) * Ht + R
    //A = InvertMatrix(pU, Num);
    A = MatrixInv(pU , Num);
    if (A == SUCCESS)         // U = ( H * Pk(-) * Ht + R )^-1
    {
        multiply_matrices_nxn_float(pPHt, pU, NUMStates, Num, Num, pK);  // K = Pk(-) * Ht * ( H * Pk(-) * Ht + R )^-1

        multiply_matrix_with_vector_nxn(pK, NUMStates, Num, inno, dx);       // K * (Z - H * xk(-) ) = K *inno

        multiply_matrices_nxn_float(pK, H, NUMStates, Num, NUMStates, pKH);       // K * H

        eye_matrix_float(pI, NUMStates);                      // unit matrix
        subtract_matrices_nxn(pI, pKH, NUMStates, NUMStates, pIKH);        // I - K * H

        for(i=0; i<NUMStates; i++) x[i] += dx[i];
        matrix_transpose_nxn_float(pIKH,NUMStates, NUMStates, pIKHT);
        multiply_matrices_nxn_float(pIKH, P, NUMStates, NUMStates, NUMStates, pIKHP);  //  (I - K * H) * Pk(-)
        multiply_matrices_nxn_float(pIKHP, pIKHT, NUMStates, NUMStates, NUMStates, pIPIT); // (I - K * H) * Pk(-) *(I-K*H)_t


        matrix_transpose_nxn_float(pK, NUMStates, Num, pKT);                     // Kt
        multiply_matrices_nxn_float(pK, R, NUMStates, Num, Num, pKR);              // K * R
        multiply_matrices_nxn_float(pKR, pKT, NUMStates, Num, NUMStates, pKRKT);      // K * R *Kt


        add_matrices_nxn(pIPIT, pKRKT, NUMStates, NUMStates, P);     // Pk(+) = (I - K * H) * Pk(-) *(I-K*H)_t + K * R * Kt

    }

    return A;
}

void Attitude_Mechanization_simple_float(FLOAT32 curG[3], FLOAT32 nav_prev_q_bn[4], FLOAT32 nav_cur_q_bn[4],FLOAT32 nav_cur_C_bn[3][3],FLOAT32 nav_cur_att[3], FLOAT32 zeta[3])
{

    FLOAT32 beta1[3] = {curG[0], curG[1], curG[2]};
    FLOAT32 q[4] = {0.0};
    Rvec2Quat(beta1, q);
    QuatPrdct(nav_prev_q_bn, q, nav_cur_q_bn);

    Rvec2Quat(zeta, q);
    QuatPrdct(q, nav_cur_q_bn, nav_cur_q_bn);
    NormQuat(nav_cur_q_bn, nav_cur_q_bn);

    Quat2Dcm( nav_cur_q_bn, nav_cur_C_bn );
    Dcm2Euler( nav_cur_C_bn, nav_cur_att );
}

void Attitude_Mechanization_float(FLOAT32 preG[3], FLOAT32 curG[3], FLOAT32 nav_prev_q_bn[4], FLOAT32 nav_cur_q_bn[4],FLOAT32 nav_cur_C_bn[3][3],FLOAT32 nav_cur_att[3], FLOAT32 zeta[3])
{
    FLOAT32 beta[3] = {0.0};
    FLOAT32 beta1[3];
    FLOAT32 q[4] = {0.0};

    vector_cross_product(preG, curG, beta);
    beta[0] /= 12.0;
    beta[1] /= 12.0;
    beta[2] /= 12.0;

    beta1[0] = curG[0]+beta[0];
    beta1[1] = curG[1]+beta[1];
    beta1[2] = curG[2]+beta[2];

    Rvec2Quat(beta1, q);
    QuatPrdct(nav_prev_q_bn, q, nav_cur_q_bn);

    Rvec2Quat(zeta, q);
    QuatPrdct(q, nav_cur_q_bn, nav_cur_q_bn);
    NormQuat(nav_cur_q_bn, nav_cur_q_bn);

    Quat2Dcm( nav_cur_q_bn, nav_cur_C_bn);
    Dcm2Euler( nav_cur_C_bn, nav_cur_att);
}


STATUS KF_update1_float(INT16 Num, FLOAT32 *x, FLOAT32 **P, FLOAT32 *inno, FLOAT32 **H, FLOAT32 **R,INT16 NUMStates)
{
    INT16 i;
    STATUS A = FAILURE;
    FLOAT32 Ht[NStates][NStates], PHt[NStates][NStates],HPHt[NStates][NStates], U[NStates][NStates],K[NStates][NStates];
    FLOAT32 *pHt[NStates], *pPHt[NStates],*pHPHt[NStates], *pU[NStates],*pK[NStates];

    FLOAT32 dx[NStates],KH[NStates][NStates],I[NStates][NStates], IKH[NStates][NStates], IKHT[NStates][NStates];
    FLOAT32 *pKH[NStates],*pI[NStates],*pIKH[NStates],*pIKHT[NStates];

    FLOAT32 IKHP[NStates][NStates],IPIT[NStates][NStates],KT[NStates][NStates],KR[NStates][NStates],KRKT[NStates][NStates];
    FLOAT32 *pIKHP[NStates],*pIPIT[NStates],*pKT[NStates],*pKR[NStates],*pKRKT[NStates];



    ChangeArrayMemoryLayout_float(NStates,NStates,pHt,(FLOAT32 *)Ht,1);
    ChangeArrayMemoryLayout_float(NStates,NStates,pPHt,(FLOAT32 *)PHt,1);
    ChangeArrayMemoryLayout_float(NStates,NStates,pHPHt,(FLOAT32 *)HPHt,1);
    ChangeArrayMemoryLayout_float(NStates,NStates,pU,(FLOAT32 *)U,1);
    ChangeArrayMemoryLayout_float(NStates,NStates,pK,(FLOAT32 *)K,1);
    ChangeArrayMemoryLayout_float(NStates,NStates,pKH,(FLOAT32 *)KH,1);
    ChangeArrayMemoryLayout_float(NStates,NStates,pI,(FLOAT32 *)I,1);
    ChangeArrayMemoryLayout_float(NStates,NStates,pIKH,(FLOAT32 *)IKH,1);
    ChangeArrayMemoryLayout_float(NStates,NStates,pIKHT,(FLOAT32 *)IKHT,1);
    ChangeArrayMemoryLayout_float(NStates,NStates,pIKHP,(FLOAT32 *)IKHP,1);
    ChangeArrayMemoryLayout_float(NStates,NStates,pIPIT,(FLOAT32 *)IPIT,1);
    ChangeArrayMemoryLayout_float(NStates,NStates,pKT,(FLOAT32 *)KT,1);
    ChangeArrayMemoryLayout_float(NStates,NStates,pKR,(FLOAT32 *)KR,1);
    ChangeArrayMemoryLayout_float(NStates,NStates,pKRKT,(FLOAT32 *)KRKT,1);
    matrix_transpose_nxn_float(H, Num, NUMStates, pHt);  // Ht


    multiply_matrices_nxn_float(P, pHt, NUMStates, NUMStates, Num, pPHt);  // Pk(-) * Ht
    multiply_matrices_nxn_float(H, pPHt, Num, NUMStates, Num, pHPHt);    // H * Pk(-) * Ht


    add_matrices_nxn(pHPHt, R, Num, Num, pU);     // U = H * Pk(-) * Ht + R
    //A = InvertMatrix(pU, Num);
    A = MatrixInv(pU , Num);
    if (A == SUCCESS)         // U = ( H * Pk(-) * Ht + R )^-1
    {
        multiply_matrices_nxn_float(pPHt, pU, NUMStates, Num, Num, pK);  // K = Pk(-) * Ht * ( H * Pk(-) * Ht + R )^-1

        multiply_matrix_with_vector_nxn(pK, NUMStates, Num, inno, dx);       // K * (Z - H * xk(-) ) = K *inno

        multiply_matrices_nxn_float(pK, H, NUMStates, Num, NUMStates, pKH);       // K * H

        eye_matrix_float(pI, NUMStates);                      // unit matrix
        subtract_matrices_nxn(pI, pKH, NUMStates, NUMStates, pIKH);        // I - K * H

        for(i=0; i<NUMStates; i++) x[i] += dx[i];
        matrix_transpose_nxn_float(pIKH,NUMStates, NUMStates, pIKHT);
        multiply_matrices_nxn_float(pIKH, P, NUMStates, NUMStates, NUMStates, pIKHP);  //  (I - K * H) * Pk(-)
        multiply_matrices_nxn_float(pIKHP, pIKHT, NUMStates, NUMStates, NUMStates, pIPIT); // (I - K * H) * Pk(-) *(I-K*H)_t


        matrix_transpose_nxn_float(pK, NUMStates, Num, pKT);                     // Kt
        multiply_matrices_nxn_float(pK, R, NUMStates, Num, Num, pKR);              // K * R
        multiply_matrices_nxn_float(pKR, pKT, NUMStates, Num, NUMStates, pKRKT);      // K * R *Kt


        add_matrices_nxn(pIPIT, pKRKT, NUMStates, NUMStates, P);     // Pk(+) = (I - K * H) * Pk(-) *(I-K*H)_t + K * R * Kt

    }

    return A;
}


void INSNavFeedBack_Attitude_float( FLOAT32 x[NStates], FLOAT32 q_bn[4], FLOAT32 C_bn[3][3], FLOAT32 att[3], FLOAT32 EstimatedGyroBias[3])
{

    FLOAT32 phi_ang[3], qe[4], qbn[4];
    INT16 i;

    // ===== attitude feedback
    for(i=0; i<3; i++)
        phi_ang[i] = x[i];
    Rvec2Quat(phi_ang, qe);
    QuatPrdct(qe, q_bn, qbn);

    for(i=0; i<4; i++)
        q_bn[i] = qbn[i];
    Quat2Dcm(q_bn, C_bn);
    Dcm2Euler(C_bn, att);

    // ======= sensor error feedback

    EstimatedGyroBias[0]  += x[3];	EstimatedGyroBias[1]  += x[4];    EstimatedGyroBias[2]  += x[5];

    for(i=0; i<NStates; i++)
        x[i] = 0.0;
}


void Rvec2Quat(FLOAT32 r_vec[3], FLOAT32 q[4]){
	FLOAT32 mag2 = r_vec[0]*r_vec[0] + r_vec[1]*r_vec[1] + r_vec[2]*r_vec[2];
	FLOAT32 mag = 0.0;
	FLOAT32 s_mag = 0.0;
	FLOAT32 c, s;

	if(mag2 < PI*PI) {
		mag2 *= 0.25;
		c = 1.0 - mag2/2.0 * (1.0 - mag2/12.0 * (1.0 - mag2/30.0 ));
		s = 1.0 - mag2/6.0 * (1.0 - mag2/20.0 * (1.0 - mag2/42.0 ));

		q[0] = c;
		q[1] = s * 0.5 * r_vec[0];
		q[2] = s * 0.5 * r_vec[1];
		q[3] = s * 0.5 * r_vec[2];
	}
	else {
		mag = sqrt(mag2);
		s_mag = sin(mag/2);

		q[0] = cos(mag/2);
		q[1] = r_vec[0] * s_mag/mag;
		q[2] = r_vec[1] * s_mag/mag;
		q[3] = r_vec[2] * s_mag/mag;

		if (q[0] < 0) {
			q[0] *= -1;
			q[1] *= -1;
			q[2] *= -1;
			q[3] *= -1;
		}
	}
}

void QuatPrdct( FLOAT32 q[4], FLOAT32 p[4], FLOAT32 qnew[4] )
{
	UINT8 i;
	FLOAT32 q_temp[4];

	qnew[0] = q[0]*p[0] - q[1]*p[1] - q[2]*p[2] -q[3]*p[3];
	qnew[1] = q[0]*p[1] + q[1]*p[0] + q[2]*p[3] -q[3]*p[2];
	qnew[2] = q[0]*p[2] + q[2]*p[0] + q[3]*p[1] -q[1]*p[3];
	qnew[3] = q[0]*p[3] + q[3]*p[0] + q[1]*p[2] -q[2]*p[1];

	if( qnew[0] < 0.0f )
	{
		for(i=0; i<4; i++) q_temp[i] = -qnew[i];
		for(i=0; i<4; i++) qnew[i] = q_temp[i];
	}
}


void NormQuat(FLOAT32 q[4], FLOAT32 q_norm[4])
{
	UINT8 i;
	FLOAT32 value;

	value = ( SQR(q[0]) + SQR(q[1]) + SQR(q[2]) + SQR(q[3]) - 1.0f ) *0.5f;
	for(i=0; i<4; i++) q_norm[i] = ( 1.0f - value ) * q[i];
}

void Quat2Dcm( FLOAT32 quar[4], FLOAT32 C[3][3] )
{
	C[0][0] = quar[0] * quar[0] + quar[1] * quar[1] - quar[2] * quar[2] - quar[3] * quar[3];
	C[0][1] = 2.0f * ( quar[1] * quar[2] - quar[0] * quar[3] );
	C[0][2] = 2.0f * ( quar[1] * quar[3] + quar[0] * quar[2] );

	C[1][0] = 2.0f * ( quar[1] * quar[2] + quar[0] * quar[3] );
	C[1][1] = quar[0] * quar[0] - quar[1] * quar[1] + quar[2] * quar[2] - quar[3] * quar[3];
	C[1][2] = 2.0f * ( quar[2] * quar[3] - quar[0] * quar[1] );

	C[2][0] = 2.0f * ( quar[1] * quar[3] - quar[0] * quar[2] );
	C[2][1] = 2.0f * ( quar[2] * quar[3] + quar[0] * quar[1] );
	C[2][2] = quar[0] * quar[0] - quar[1] * quar[1] - quar[2] * quar[2] + quar[3] * quar[3];
}

void Dcm2Euler(FLOAT32 Cbn[3][3], FLOAT32 attitude[3])
{
	// between +/-pi
	attitude[0] = atan2(Cbn[2][1], Cbn[2][2]);
	// between +/-pi/2
	attitude[1] = atan( -Cbn[2][0]/sqrt(SQR(Cbn[2][1])+SQR(Cbn[2][2])) );
	// between +/-pi
	attitude[2] = atan2(Cbn[1][0], Cbn[0][0]);
}

void multiply_matrix_with_vector_3x3( FLOAT32 Matrix[3][3], FLOAT32 vector[3], FLOAT32 vectorR[3] )
{
	UINT8 i, j;
	for(i=0; i<3; i++)
	{
		vectorR[i] = 0.0f;

		for(j=0; j<3; j++)
			vectorR[i] += Matrix[i][j] * vector[j];
	}
}


void ChangeArrayMemoryLayout_float(UINT8 r, UINT8 c , FLOAT32 **ptr, FLOAT32 *Arr, UINT8 InitToZero)
{
	UINT8 i;
	for(i=0;i < r ; i++)
		ptr[i] = (FLOAT32 *)Arr+i*c;
	if(InitToZero == 1)
		zero_matrix_entries_nxn_float(ptr,r,c);
}

void multiply_scalar_with_matrix_nxn(FLOAT32 Scalar, FLOAT32 **Matrix, FLOAT32 **MatrixR,UINT8 cols, UINT8 rows)
{
	UINT8 i, j;

	for(i=0; i<rows; i++)
		for(j=0; j<cols; j++)
			MatrixR[i][j] = Scalar * Matrix[i][j];
}

void add_matrices_nxn(FLOAT32 **Matrix1, FLOAT32 **Matrix2, UINT8 nrow, UINT8 ncol, FLOAT32 **MatrixR)
{
	UINT8 i, j;

	for (i=0; i<nrow; i++)
		for(j=0; j<ncol; j++)
			MatrixR[i][j] = Matrix1[i][j] + Matrix2[i][j];
}

void matrix_transpose_nxn_float(FLOAT32 **Matrix, UINT8 m, UINT8 n, FLOAT32 **MatrixT)
{
	UINT8 i, j;

	for(i = 0; i < n ; i++)
		for(j = 0; j < m; j++)
			MatrixT[i][j] = Matrix[j][i];
}

void multiply_matrices_nxn_float(FLOAT32 **Matrix1, FLOAT32 **Matrix2, UINT8 ar, UINT8 ac, UINT8 bc, FLOAT32 **MatrixR)
{
	UINT8 i, j,k;
#ifndef COM_WALK_SIMPLIFIED
	FLOAT32 Matrix2T[15][15];
#else
	FLOAT32 Matrix2T[9][9];
#endif
	FLOAT32 sigmax;

	for(i=0;i<bc;i++)
		for(j=0;j<ac;j++)
			Matrix2T[i][j] = Matrix2[j][i];

	for ( i = 0; i < ar; i++ )
      for ( j = 0; j < bc; j++ )
      {
		  sigmax = 0.0f;

         for ( k = 0; k < ac; k++ )
			sigmax += Matrix1[i][k] * Matrix2T[j][k];
		 MatrixR[i][j] = sigmax;
      }

}

void multiply_matrix_with_vector_nxn(FLOAT32 **Matrix, UINT8 ar, UINT8 ac, FLOAT32 *vector, FLOAT32 *vectorR)
{
	UINT8 i, j;
	for (i=0; i<ar; i++)
	{
	    vectorR[i] = 0;
		for (j=0; j<ac; j++)
			vectorR[i] = vectorR[i] + Matrix[i][j] * vector[j];
	}

}

void make_skew_symmetric_matrix( FLOAT32 vector[3], FLOAT32 Matrix[3][3] )
{
	Matrix[0][0] = 0.0f;
	Matrix[0][1] = -vector[2];
	Matrix[0][2] = vector[1];

	Matrix[1][0] = vector[2];
	Matrix[1][1] = 0.0f;
	Matrix[1][2] = -vector[0];

	Matrix[2][0] = -vector[1];
	Matrix[2][1] = vector[0];
	Matrix[2][2] = 0.0f;

}

void diag2_sq(FLOAT32 *A, INT16 r, FLOAT32 **X)
{
	INT16 i, j;
	for (i=0; i<r; i++)
		for (j=0; j<r; j++)
		{
			X[i][j] = 0.0;
			if(i==j) X[i][j]=A[i]*A[i];
		}

	/*for(i = 0; i < r; ++i){
		X[i][i] = A[i];
	}*/
}

void find_max_min_float(FLOAT32 *vector, FLOAT32 *maxmin, UINT16 arraysize)
{
	UINT16 i;
	maxmin[0] = vector[0];
	maxmin[1] = vector[0];
	for (i = 1;i < arraysize;i++)
	{
		if (maxmin[0] < vector[i])
			maxmin[0] = vector[i];
		if (maxmin[1] > vector[i])
			maxmin[1] = vector[i];
	}
}

void find_max_min_double(DOUBLE64 *vector, DOUBLE64 *maxmin, UINT16 arraysize)
{
	UINT16 i;
	maxmin[0] = vector[0];
	maxmin[1] = vector[0];
	for (i = 1;i < arraysize;i++)
	{
		if (maxmin[0] < vector[i])
			maxmin[0] = vector[i];
		if (maxmin[1] > vector[i])
			maxmin[1] = vector[i];
	}
}


STATUS MatrixInv(FLOAT32 **A, INT16 n) {

	INT16 i,j;
	INT16 indx[MATRIX_INVERSE_SIZE];
	FLOAT32 *pB[MATRIX_INVERSE_SIZE];
	FLOAT32 C[MATRIX_INVERSE_SIZE];
	FLOAT32 B[MATRIX_INVERSE_SIZE][MATRIX_INVERSE_SIZE];

	for(i=0;i < n; i++)
		pB[i] = (FLOAT32 *)B+i*n;

	CopyMatrix_N_Dimension(A,n,n,pB);

	if (lu_decomposition(pB,n,indx) == FAILURE) {
		return FAILURE;
	}

	for (j=0;j<n;j++) {
		for (i=0;i<n;i++)
			A[i][j]=0.0f;

		A[j][j]=1.0f;

		for (i=0;i<n;i++)
			C[i] = A[i][j];

		lu_back_subsititution(pB,n,indx,C);

		for (i=0;i<n;i++)
			 A[i][j] = C[i];


	}
	return SUCCESS;


}

void eye_matrix_float(FLOAT32 **A, INT16 n)
{
	UINT8 i,j;
	for (i=0; i<n; i++)
	{
		for (j=0; j<n; j++)
		{
			A[i][j] = 0;
			if (i==j)
			{
				A[i][j] = 1.0;
			}
		}
	}
}

void subtract_matrices_nxn(FLOAT32 **Matrix1, FLOAT32 **Matrix2, UINT8 nrow, UINT8 ncol, FLOAT32 **MatrixR)
{
	UINT8 i, j;

	for (i=0; i<nrow; i++)
		for(j=0; j<ncol; j++)
			MatrixR[i][j] = Matrix1[i][j] - Matrix2[i][j];
}

void zero_matrix_entries_nxn_float(FLOAT32 **Matrix, INT16 row, INT16 col)
{
	INT16 i, j;
	for(i=0; i<row; i++)
		for(j=0; j<col; j++)
			Matrix[i][j] = 0.0;
}

void CopyMatrix_N_Dimension(FLOAT32 **Matrix, INT16 m, INT16 n, FLOAT32 **CopyMatrix)
{
	INT16 i, j;

	for(i=0;i<m;i++)
		for(j=0;j<n;j++)
			CopyMatrix[i][j] = Matrix[i][j];
}

STATUS lu_decomposition(FLOAT32 **A, INT16 n, INT16 *indx)
{
	FLOAT32 big,s,tmp;
	FLOAT32 vv[24];
	INT16 i,imax=0,j,k;

//    *d=1.0f;
	for (i=0;i<n;i++) {
		big=0.0f; for (j=0;j<n;j++) if ((tmp=fabs(A[i][j]))>big) big=tmp;
		if (big>0.0f)
			vv[i]=1.0f/big;
		else {
			return FAILURE;
		}
	}
	for (j=0;j<n;j++) {
		for (i=0;i<j;i++) {
			s=A[i][j];
			for (k=0;k<i;k++)
				s-=A[i][k]*A[k][j];
			A[i][j]=s;
		}
		big=0.0;
		for (i=j;i<n;i++) {
			s=A[i][j];
			for (k=0;k<j;k++)
				s-=A[i][k]*A[k][j];
			A[i][j]=s;
			if ((tmp=vv[i]*fabs(s))>=big) {
				big=tmp; imax=i;
			}
		}
		if (j!=imax) {
			for (k=0;k<n;k++) {
				tmp=A[imax][k];
				A[imax][k]=A[j][k];
				A[j][k]=tmp;
			}
			//*d=-(*d);
			vv[imax]=vv[j];
		}
		indx[j]=imax;
		if (A[j][j]==0.0f) {
			return FAILURE;
		}
		if (j!=n-1) {
			tmp=1.0f/A[j][j];
			for (i=j+1;i<n;i++)
				A[i][j]*=tmp;
		}
	}
	return SUCCESS;
}

static void lu_back_subsititution(FLOAT32 **A, INT16 n, INT16 *indx, FLOAT32 *b) {
	FLOAT32 s;
	INT16 i,ii=-1,ip,j;

	for (i=0;i<n;i++) {
		ip=indx[i]; s=b[ip]; b[ip]=b[i];

		if (ii>=0)
			for (j=ii;j<i;j++)
				s-=A[i][j]*b[j];
		else if (s)
			ii=i;

		b[i]=s;
	}
	for (i=n-1;i>=0;i--) {
		s=b[i];
		for (j=i+1;j<n;j++)
			s-=A[i][j]*b[j];

		b[i]=s/A[i][i];
	}
}



float min_f(FLOAT32 *vector, UINT16 arraysize){
	UINT16 i;
	float m1;
	m1 = vector[0];
	for (i = 1;i < arraysize;i++)
	{
		if (m1 > vector[i]){
			m1 = vector[i];
		}
	}

	return m1;
}

void min_float(FLOAT32 *vector, FLOAT32 *min, UINT16 arraysize)
{
	UINT16 i;
	min[0] = vector[0];
	min[1] = 0;
	for (i = 1;i < arraysize;i++)
	{
		if (min[0] > vector[i]){
			min[0] = vector[i];
			min[1] = i;
		}
	}
}

void max_float(FLOAT32 *vector, FLOAT32 *max, UINT16 arraysize)
{
	UINT16 i;
	max[0] = vector[0];
	max[1] = 0;
	for (i = 1;i < arraysize;i++)
	{
		if (max[0] < vector[i]){
			max[0] = vector[i];
			max[1] = i;
		}
	}
}

void min_double(double *vector, double *min, UINT16 arraysize)
{
	UINT16 i;
	min[0] = vector[0];
	min[1] = 0;
	for (i = 1;i < arraysize;i++)
	{
		if (min[0] > vector[i]){
			min[0] = vector[i];
			min[1] = i;
		}
	}
}


void vector_cross_product( FLOAT32 vector1[3], FLOAT32 vector2[3], FLOAT32 vectorR[3] )
{

	vectorR[0] = vector1[1] * vector2[2] - vector2[1] * vector1[2];
	vectorR[1] = vector2[0] * vector1[2] - vector1[0] * vector2[2];
	vectorR[2] = vector1[0] * vector2[1] - vector2[0] * vector1[1];

}

float cal_std_float(FLOAT32* a, UINT8 n) {
	UINT8 i;
	FLOAT32 mean1 = 0.0;
	FLOAT32 std1 = 0.0;

	for(i=0;i<n;i++){
		mean1 += a[i];
	}
	mean1 /= n;

	for(i=0;i<n;i++){
		std1 += (a[i]-mean1) * (a[i]-mean1);
	}
	std1 /= (n-1);
	std1 = sqrt(std1);

	return std1;
}



long int mac2int(char mac[N_CHAR_MAC]){
	long int mac_ap;

	mac_ap = 100000*hex2dec_n(mac,2) + 10000*hex2dec_n(mac+3,2) + 1000*hex2dec_n(mac+6,2)
				     + 100*hex2dec_n(mac+9,2) + 10*hex2dec_n(mac+12,2) + hex2dec_n(mac+15,2);
	return mac_ap;
}

int hex2dec_n(char *s, int n)
{
    int num = 0;
	int i, v;
    for(i=0; i<n; i++)
    {
        if      ('a' <= s[i] && s[i] <='f') { v=s[i]-97+10; }
        else if ('A' <= s[i] && s[i] <='F') { v=s[i]-65+10; }
        else if ('0' <= s[i] && s[i] <='9') { v=s[i]-48;    }
        else break;
        num *= 16;
        num += v;
    }
    return num;
}

double cal_dist_2D(double lat1, double lon1, double lat2, double lon2) {
	double d_N, d_E, dist;

	d_N = (lat2-lat1)*RM;
	d_E = (lon2-lon1)*RN*cos(lat1);

	dist = d_N*d_N + d_E*d_E;
	return sqrt(dist);
}

double cal_dss_int(int *v1, int *v2, int n) {
	double dss_sol = 0.0;
	int i;
	int dv;

	for(i=0; i<n; i++) {
		dv = v1[i] - v2[i];
		dss_sol += dv*dv;
	}
	return (sqrt(dss_sol));
}



int outOfChina(double lat, double lng) {
	if (lng < 72.004 || lng > 137.8347) {
		return 1;
	}
	if (lat < 0.8293 || lat > 55.8271) {
		return 1;
	}
	return 0;
}

void transform(double x, double y, double *lat, double *lng) {
	double xy = x * y;
	double absX = sqrt(abs(x));
	double d = (20.0*sin(6.0*x*PI) + 20.0*sin(2.0*x*PI)) * 2.0 / 3.0;

	*lat = -100.0 + 2.0*x + 3.0*y + 0.2*y*y + 0.1*xy + 0.2*absX;
	*lng = 300.0 + x + 2.0*y + 0.1*x*x + 0.1*xy + 0.1*absX;

	*lat += d;
	*lng += d;

	*lat += (20.0*sin(y*PI) + 40.0*sin(y/3.0*PI)) * 2.0 / 3.0;
	*lng += (20.0*sin(x*PI) + 40.0*sin(x/3.0*PI)) * 2.0 / 3.0;

	*lat += (160.0*sin(y/12.0*PI) + 320*sin(y/30.0*PI)) * 2.0 / 3.0;
	*lng += (150.0*sin(x/12.0*PI) + 300.0*sin(x/30.0*PI)) * 2.0 / 3.0;
}

void delta(double lat, double lng, double *dLat, double *dLng) {
	if ((dLat == NULL) || (dLng == NULL)) {
		return;
	}
	const double a = 6378245.0;
	const double ee = 0.00669342162296594323;
	transform(lng-105.0, lat-35.0, dLat, dLng);
	double radLat = lat / 180.0 * PI;
	double magic = sin(radLat);
	magic = 1 - ee*magic*magic;
	double sqrtMagic = sqrt(magic);
	*dLat = (*dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * PI);
	*dLng = (*dLng * 180.0) / (a / sqrtMagic * cos(radLat) * PI);
}

void wgs2gcj(double wgsLat, double wgsLng, double *gcjLat, double *gcjLng) {
	if ((gcjLat == NULL) || (gcjLng == NULL)) {
		return;
	}
	if (outOfChina(wgsLat, wgsLng)) {
		*gcjLat = wgsLat;
		*gcjLng = wgsLng;
		return;
	}
	double dLat, dLng;
	delta(wgsLat, wgsLng, &dLat, &dLng);
	*gcjLat = wgsLat + dLat;
	*gcjLng = wgsLng + dLng;
}

void gcj2wgs(double gcjLat, double gcjLng, double *wgsLat, double *wgsLng) {
	if ((wgsLat == NULL) || (wgsLng == NULL)) {
		return;
	}
	if (outOfChina(gcjLat, gcjLng)) {
		*wgsLat = gcjLat;
		*wgsLng = gcjLng;
		return;
	}
	double dLat, dLng;
	delta(gcjLat, gcjLng, &dLat, &dLng);
	*wgsLat = gcjLat - dLat;
	*wgsLng = gcjLng - dLng;
}

void gcj2wgs_exact(double gcjLat, double gcjLng, double *wgsLat, double *wgsLng) {
	const double initDelta = 0.01;
	const double threshold = 0.000001;
	double dLat = initDelta, dLng = initDelta;
	double mLat = gcjLat-dLat, mLng = gcjLng-dLng;
	double pLat = gcjLat+dLat, pLng = gcjLng+dLng;
	int i;
	for (i = 0; i < 30; i++) {
		*wgsLat = (mLat+pLat)/2;
		*wgsLng = (mLng+pLng)/2;
		double tmpLat, tmpLng;
		wgs2gcj(*wgsLat, *wgsLng, &tmpLat, &tmpLng);
		dLat = tmpLat - gcjLat;
		dLng = tmpLng - gcjLng;
		if ((fabs(dLat) < threshold) && (fabs(dLng) < threshold)) {
			return;
		}
		if (dLat > 0) {
			pLat = *wgsLat;
		} else {
			mLat = *wgsLat;
		}
		if (dLng > 0) {
			pLng = *wgsLng;
		} else {
			mLng = *wgsLng;
		}
	}
}

double distance(double latA, double lngA, double latB, double lngB) {
	const double earthR = 6371000;
	double x = cos(latA*PI/180) * cos(latB*PI/180) * cos((lngA-lngB)*PI/180);
	double y = sin(latA*PI/180) * sin(latB*PI/180);
	double s = x + y;
	if (s > 1) {
		s = 1;
	}
	if (s < -1) {
		s = -1;
	}
	double alpha = acos(s);
	double distance = alpha * earthR;
	return distance;
}



// ------------------ new for map
void load_DB_fp_string(float da_map_NE[N_ROW_DB_FP][N_COL_DB_FP], int nLine[1])
{
	char str_step[200];
	//char format_db_fp_line[200];
	int i;

	//strcat(format_db_fp_line, "%f");
	//for (i=1; i<N_COL_DB_FP; i++) {
	//	strcat(format_db_fp_line, " %f");
	//}

	nLine[0] = 0;

	for (i=0; i<N_ROW_DB_FP; i++) {
		strcpy(str_step, string_FP[i]);
		sscanf(str_step,"%f %f %f %f %f %f %f %f", da_map_NE[i], da_map_NE[i]+1, da_map_NE[i]+2,
			da_map_NE[i]+3, da_map_NE[i]+4, da_map_NE[i]+5, da_map_NE[i]+6, da_map_NE[i]+7);

		da_map_NE[i][3] *= D2R;
		nLine[0] ++;
	}
}


void load_DB_fp_txt(const char * f_FP, float da_map_NE[N_ROW_DB_FP][8], int nLine[1])
{
	FILE * f_in_fp = NULL;
	char buff[300];
	int i_line = 0;

	if((f_in_fp = fopen(f_FP,"r")) == NULL){
		printf("The FP file <%s> can not be opened.\n", f_FP);
		exit(1);
	}

	while (fgets(buff,sizeof(buff),f_in_fp) != NULL) {

		sscanf(buff,"%f %f %f %f %f %f %f %f", da_map_NE[i_line], da_map_NE[i_line]+1, da_map_NE[i_line]+2,
			da_map_NE[i_line]+3, da_map_NE[i_line]+4, da_map_NE[i_line]+5, da_map_NE[i_line]+6, da_map_NE[i_line]+7);

		da_map_NE[i_line][3] *= D2R;
		i_line ++;
	}

	/*while (!feof(f_in_fp)) {
		if (fgets(buff,sizeof(buff),f_in_fp)) {
			sscanf(buff,"%f %f %f %f %f %f %f %f", da_map_NE[i_line], da_map_NE[i_line]+1, da_map_NE[i_line]+2,
				da_map_NE[i_line]+3, da_map_NE[i_line]+4, da_map_NE[i_line]+5, da_map_NE[i_line]+6, da_map_NE[i_line]+7);
		}
		da_map_NE[i_line][3] *= D2R;
		i_line ++;
	}*/
	nLine[0] = i_line;

	fclose(f_in_fp);

}

FLOAT32 centralization_angle_deg(FLOAT32 ang) {
	if (ang > 180.0){
		return (ang - 360.0);
	} else if (ang < -180.0){
		return (ang + 360.0);
	} else {
		return ang;
	}
}

FLOAT32 cal_mean_float(FLOAT32* a, UINT16 n) {
	UINT16 i;
	FLOAT32 mean1 = 0.0;

	for(i=0;i<n;i++){
		mean1 += a[i];
	}
	return (mean1 / n);
}


FLOAT32 ptoject2line(float ps[2],float p1[2],float p2[2],float pp[2]) {
	float D_min, xs, ys, x1, y1, x2, y2, xp, yp, norm12, norm12_2, D, D1, D2, min_x, min_y, max_x, max_y;
	float TH_D = 1;
	float TH_D2 = 0.1;
	xs = ps[0]; ys = ps[1];
	x1 = p1[0]; y1 = p1[1];
	x2 = p2[0]; y2 = p2[1];

	norm12_2 = (x1-x2)*(x1-x2)+(y1-y2)*(y1-y2);
	norm12 = sqrt(norm12_2);

	D = fabs((xs*(y1-y2)-ys*(x1-x2)+(x1*y2-x2*y1)))/norm12;
	xp = ((x2-x1)*(xs*(x2-x1)+ys*(y2-y1))+(y2-y1)*(x1*y2-x2*y1))/norm12_2;
	yp = ((y2-y1)*(xs*(x2-x1)+ys*(y2-y1))-(x2-x1)*(x1*y2-x2*y1))/norm12_2;

	D1 = sqrt((xs-x1)*(xs-x1)+(ys-y1)*(ys-y1));
	D2 = sqrt((xs-x2)*(xs-x2)+(ys-y2)*(ys-y2));

	if (x1>x2) {
		min_x = x2;
		max_x = x1;
	}
	else {
		min_x = x1;
		max_x = x2;
	}
	if (y1>y2) {
		min_y = y2;
		max_y = y1;
	}
	else {
		min_y = y1;
		max_y = y2;
	}

	if (((fabs(yp-y1)<TH_D && fabs(yp-y2)<TH_D) && (xp >= min_x && xp <= max_x)) ||
		((fabs(xp-x1)<TH_D && fabs(xp-x2)<TH_D) && (yp >= min_y && yp <= max_y)) ||
		(xp >= min_x && xp <= max_x && yp >= min_y && yp <= max_y))
	{
		pp[0] = xp;
		pp[1] = yp;
		D_min = D;
	}
	else {
		if (D1 <= D2) {
			pp[0] = x1;
			pp[1] = y1;
			D_min = D1;
		} else {
			pp[0] = x2;
			pp[1] = y2;
			D_min = D2;
		}
	}

	if (D_min < TH_D2) {
		D_min = TH_D2;
	}

	return D_min;
}

FLOAT32 ptoject2line_can_be_outside(float ps[2],float p1[2],float p2[2],float pp[2]) {
	float D_min, xs, ys, x1, y1, x2, y2, xp, yp, norm12, norm12_2, D, D1, D2, min_x, min_y, max_x, max_y;
	float TH_D = 1;
	float TH_D2 = 0.1;
	xs = ps[0]; ys = ps[1];
	x1 = p1[0]; y1 = p1[1];
	x2 = p2[0]; y2 = p2[1];

	norm12_2 = (x1-x2)*(x1-x2)+(y1-y2)*(y1-y2);
	norm12 = sqrt(norm12_2);

	D = fabs((xs*(y1-y2)-ys*(x1-x2)+(x1*y2-x2*y1)))/norm12;
	xp = ((x2-x1)*(xs*(x2-x1)+ys*(y2-y1))+(y2-y1)*(x1*y2-x2*y1))/norm12_2;
	yp = ((y2-y1)*(xs*(x2-x1)+ys*(y2-y1))-(x2-x1)*(x1*y2-x2*y1))/norm12_2;

	D1 = sqrt((xs-x1)*(xs-x1)+(ys-y1)*(ys-y1));
	D2 = sqrt((xs-x2)*(xs-x2)+(ys-y2)*(ys-y2));

	if (x1>x2) {
		min_x = x2;
		max_x = x1;
	}
	else {
		min_x = x1;
		max_x = x2;
	}
	if (y1>y2) {
		min_y = y2;
		max_y = y1;
	}
	else {
		min_y = y1;
		max_y = y2;
	}

	//if (((fabs(yp-y1)<TH_D && fabs(yp-y2)<TH_D) && (xp >= min_x && xp <= max_x)) ||
	//	((fabs(xp-x1)<TH_D && fabs(xp-x2)<TH_D) && (yp >= min_y && yp <= max_y)) ||
	//	(xp >= min_x && xp <= max_x && yp >= min_y && yp <= max_y))
	//{
		pp[0] = xp;
		pp[1] = yp;
		D_min = D;
	//}
	//else {
	//	if (D1 <= D2) {
	//		pp[0] = x1;
	//		pp[1] = y1;
	//		D_min = D1;
	//	} else {
	//		pp[0] = x2;
	//		pp[1] = y2;
	//		D_min = D2;
	//	}
	//}

	if (D_min < TH_D2) {
		D_min = TH_D2;
	}

	return D_min;
}

// Measure angular distance from ang1 to ang2
FLOAT32 dist_ang_rad(float ang1, float ang2) {
	float ang = ang2 - ang1;

	if (ang > PI) {
		ang = ang - 2*PI;
	}
	else if (ang < -PI) {
		ang = ang + 2*PI;
	}
	return ang;
}

float cal_dist_2D_NE(float x1, float y1, float x2, float y2){
	float dx, dy;
	dx = x2 - x1;
	dy = y2 - y1;

	return (sqrt(dx*dx + dy*dy));
}


FLOAT32 cal_height_baro(FLOAT32 p)
{
	float p0 = 101325.0;
	float p_p0, y, h;

	p_p0 = p*100.0/p0;
	y = 1.0/5.255;

	h = 44330*(1.0-pow(p_p0, y));

	return h;
}


void indoor_navigation_withWiFi(Meas meas, Sol_Pkt*  sol){
	static UINT8 fitst_time_flag = 0;
	static NavIndoor nav;
	static MemoIndoor memo;

	static FLOAT32 gyro_bias[3], accel_bias[3];
	int i, j, i_mid_in_memo, i_heading_in_memo;
	double min_t_heading_in_memo[2];
	double max_min_qs_detect[2], diff_max_min_gz;
	//float norma1, roll1, pitch1, f_b1[3], sr1, cr1, sp1, cp1, wd1;
	float ma_mid1, min_t1;
	float heading;
	double t_mid1, lat1, ts_gyro;
	double t_diff_g_a[LENGTH_MEMO_SENSOR_DATA];

	int if_detect_step, if_update_step, if_qs;
	int if_update_pos = 0;
	static double t0_gyro = 0.00;
	static double t0_accel = 0.00;
	static double t_pre_gyro = 0.00;
	static double t_pre_accel = 0.00;
	static int if_first_t_accel = 0;
	static int if_first_t_gyro = 0;

	int availability_wifi[1] = {0};
	int num_floor[1] = {1};    // Tempory
	double pos_wifi[3] = {0.00};   // The height should be set here because it is not dealed with in the wifi function

	// Set indicators in sol to "0"
	sol->avail_sol_pdr = 0;
	sol->avail_step = 0;
	sol->avail_heading = 0;
	sol->avail_sol_wifi = 0;


	if (meas.avail_gyro) {
		if (if_first_t_gyro) {
			meas.t_gyro -= t0_gyro;  // t0_gyro is used for time_system for gyro
			if (fabs(meas.t_gyro-t_pre_gyro) < 10e-6) {  // t_pre_gyro is used for gyro time in the previous epoch
				meas.avail_gyro = 0;
			}
		}
		else {
			t0_gyro = meas.t_gyro;
			meas.t_gyro = 0;
			if_first_t_gyro = 1;
		}
		t_pre_gyro = meas.t_gyro;
	}

	if (meas.avail_accel) {
		if (if_first_t_accel) {
			meas.t_accel -= t0_accel;
			if (fabs(meas.t_accel-t_pre_accel) < 10e-6) {
				meas.avail_accel = 0;
			}
		}
		else {
			t0_accel = meas.t_accel;
			meas.t_accel = 0;
			if_first_t_accel = 1;
		}
		t_pre_accel = meas.t_accel;
	}


	//first time
	if (!fitst_time_flag){
		fitst_time_flag = 1;

		// for the first wifi data
		meas.t_gyro = 0;
		meas.t_accel = 0;

		//
		gyro_bias[0] = ini_gyro_bias_att[0];
		gyro_bias[1] = ini_gyro_bias_att[1];
		gyro_bias[2] = ini_gyro_bias_att[2];   // Later we can save and read gBias from a file

		accel_bias[0] = ini_accel_bias_att[0];
		accel_bias[1] = ini_accel_bias_att[1];
		accel_bias[2] = ini_accel_bias_att[2];   // Later we can save and read gBias from a file


		// Nav.P
		// Nav.Q

		// -- Initialization PDR
		nav.pos[0] = INI_POS[0];
		nav.pos[1] = INI_POS[1];
		nav.pos[2] = INI_POS[2];

		if (TYPE_INI_HEADING == 1) {
			nav.heading_platform = INI_HEADING;
		} else {    // Other strategies

		}
		nav.heading_step = nav.heading_platform;
		nav.t_step1 = 0;


		//-- End Initialization PDR



		// ---- INITIALIZAION_MEMO
		memo.i_accel = 0;
		memo.i_gyro = 0;
		memo.i_ma = 0;
		memo.i_t_step = 0;
		memo.i_pos_pdr = 0;

		for(i=0; i<LENGTH_MEMO_SENSOR_DATA; i++){
			memo.t_accel[i] = 0.00;
			memo.a_x[i] = 0.0;
			memo.a_y[i] = 0.0;
			memo.a_z[i] = 0.0;
			memo.t_gyro[i] = 0.00;
			memo.g_x[i] = 0.0;
			memo.g_y[i] = 0.0;
			memo.g_z[i] = 0.0;
			memo.gz_level[i] = 0.0;
			memo.heading_platform[i] = 0.0;
		}
		for(i=0; i<LENGTH_MEMO_MA; i++){
			memo.ma[i] = 0.0;
		}
		for(i=0; i<LENGTH_MEMO_T_STEP; i++) {
			memo.t_step[i] = 0.00;
		}
		for(i=0; i<LENGTH_MEMO_POS_PDR; i++) {
			memo.t_pos_pdr[i] = 0.00;
			memo.pos_pdr_lat[i] = 0.00;
			memo.pos_pdr_lon[i] = 0.00;
			memo.pos_pdr_hei[i] = 0.00;
		}
		// ---- End INITIALIZAION_MEMO


		memo.i_pos_pdr ++;
		memo.pos_pdr_lat[memo.i_pos_pdr-1] = nav.pos[0];
		memo.pos_pdr_lon[memo.i_pos_pdr-1] = nav.pos[1];
		memo.pos_pdr_hei[memo.i_pos_pdr-1] = nav.pos[2];


		/*if (meas.avail_accel){
			memo.t_pos_pdr[memo.i_pos_pdr-1] = 0.00;
		} else if (meas.avail_gyro) {
			memo.t_pos_pdr[memo.i_pos_pdr-1] = 0.00;
		} else {
			memo.t_pos_pdr[memo.i_pos_pdr-1] = 0.00;
		}*/

		// Output
		//sol.count_rec_pos ++;
		sol->avail_sol_pdr = 1;
		sol->sol_t_sol_pdr = memo.t_pos_pdr[memo.i_pos_pdr-1];
		sol->sol_pdr[0] = memo.pos_pdr_lat[memo.i_pos_pdr-1]/D2R;
		sol->sol_pdr[1] = memo.pos_pdr_lon[memo.i_pos_pdr-1]/D2R;
		sol->sol_pdr[2] = memo.pos_pdr_hei[memo.i_pos_pdr-1];

		//fprintf(sol->f_out_rec_pos, "%.4f %.8f %.8f %.3f\n",
		//		memo.t_pos_pdr[memo.i_pos_pdr-1], memo.pos_pdr_lat[memo.i_pos_pdr-1]/D2R,
		//		memo.pos_pdr_lon[memo.i_pos_pdr-1]/D2R, memo.pos_pdr_hei[memo.i_pos_pdr-1]);

	}  // End (!fitst_time_flag)


	// Start
	if (meas.avail_gyro) {
		meas.da_gyro[0] = meas.da_gyro[0] * D2R - gyro_bias[0];
		meas.da_gyro[1] = meas.da_gyro[1] * D2R - gyro_bias[1];
		meas.da_gyro[2] = meas.da_gyro[2] * D2R - gyro_bias[2];
	}

	if (meas.avail_accel) {
		meas.da_accel[0]  -= accel_bias[0];
		meas.da_accel[1]  -= accel_bias[1];
		meas.da_accel[2]  -= accel_bias[2];
	}


	// -- Put into memo
	//real_time_put_into_memo(&memo, meas);
	float norma1, roll1, pitch1, f_b1[3], sr1, cr1, sp1, cp1, wd1;
	if (meas.avail_accel){
		memo.i_accel ++;
		if (memo.i_accel <= LENGTH_MEMO_SENSOR_DATA) {
			memo.t_accel[memo.i_accel-1] = meas.t_accel;
			memo.a_x[memo.i_accel-1] = meas.da_accel[0];
			memo.a_y[memo.i_accel-1] = meas.da_accel[1];
			memo.a_z[memo.i_accel-1] = meas.da_accel[2];
		}else {
			memo.i_accel = LENGTH_MEMO_SENSOR_DATA;
			for (i=0; i<LENGTH_MEMO_SENSOR_DATA-1; i++) {
				memo.t_accel[i] = memo.t_accel[i+1];
				memo.a_x[i] = memo.a_x[i+1];
				memo.a_y[i] = memo.a_y[i+1];
				memo.a_z[i] = memo.a_z[i+1];
			}
			memo.t_accel[LENGTH_MEMO_SENSOR_DATA-1] = meas.t_accel;
			memo.a_x[LENGTH_MEMO_SENSOR_DATA-1] = meas.da_accel[0];
			memo.a_y[LENGTH_MEMO_SENSOR_DATA-1] = meas.da_accel[1];
			memo.a_z[LENGTH_MEMO_SENSOR_DATA-1] = meas.da_accel[2];
		}

		norma1 = sqrt(meas.da_accel[0]*meas.da_accel[0]+meas.da_accel[1]*meas.da_accel[1]+meas.da_accel[2]*meas.da_accel[2]);
		memo.i_ma ++;
		if(memo.i_ma <= LENGTH_MEMO_MA){
			memo.ma[memo.i_ma-1] = norma1;
		} else {
			memo.i_ma = LENGTH_MEMO_MA;
			for (i=0; i<LENGTH_MEMO_MA-1; i++) {
				memo.ma[i] = memo.ma[i+1];
			}
			memo.ma[LENGTH_MEMO_MA-1] = norma1;

		}
	}

	if (meas.avail_gyro) {
		if(!memo.i_accel) {
			roll1 = 0.0;
			pitch1 = 0.0;
		} else{
			f_b1[0] = memo.a_x[memo.i_accel-1];
			f_b1[1] = memo.a_y[memo.i_accel-1];
			f_b1[2] = memo.a_z[memo.i_accel-1];
			roll1 = cal_roll(f_b1);
			pitch1 = cal_pitch(f_b1);
		}
		sr1 = sin(roll1); cr1 = cos(roll1);
		sp1 = sin(pitch1); cp1 = cos(pitch1);
		wd1 = -sp1*meas.da_gyro[0] + sr1*cp1*meas.da_gyro[1] + cr1*cp1*meas.da_gyro[2];

		memo.i_gyro ++;
		if (memo.i_gyro <= LENGTH_MEMO_SENSOR_DATA) {
			memo.t_gyro[memo.i_gyro-1] = meas.t_gyro;
			memo.g_x[memo.i_gyro-1] = meas.da_gyro[0];
			memo.g_y[memo.i_gyro-1] = meas.da_gyro[1];
			memo.g_z[memo.i_gyro-1] = meas.da_gyro[2];
			memo.gz_level[memo.i_gyro-1] = wd1;
		}else {
			memo.i_gyro = LENGTH_MEMO_SENSOR_DATA;
			for (i=0; i<LENGTH_MEMO_SENSOR_DATA-1; i++) {
				memo.t_gyro[i] = memo.t_gyro[i+1];
				memo.g_x[i] = memo.g_x[i+1];
				memo.g_y[i] = memo.g_y[i+1];
				memo.g_z[i] = memo.g_z[i+1];
				memo.gz_level[i] = memo.gz_level[i+1];
			}
			memo.t_gyro[LENGTH_MEMO_SENSOR_DATA-1] = meas.t_gyro;
			memo.g_x[LENGTH_MEMO_SENSOR_DATA-1] = meas.da_gyro[0];
			memo.g_y[LENGTH_MEMO_SENSOR_DATA-1] = meas.da_gyro[1];
			memo.g_z[LENGTH_MEMO_SENSOR_DATA-1] = meas.da_gyro[2];
			memo.gz_level[LENGTH_MEMO_SENSOR_DATA-1] = wd1;
		}

	}


	// -- End Put into memo

	// -- Heading
	if (meas.avail_gyro) {
		ts_gyro = 0.00;
		if (memo.i_gyro>1) {
			ts_gyro = memo.t_gyro[memo.i_gyro-1]-memo.t_gyro[memo.i_gyro-2];
			//printf("ts_gyro = %.4f\n", ts_gyro);
		}

		if (memo.i_gyro < LENGTH_MEMO_SENSOR_DATA) {
			memo.heading_platform[memo.i_gyro-1] = nav.heading_platform;
		} else {

			for (i=0; i<LENGTH_MEMO_SENSOR_DATA-1; i++) {
				memo.heading_platform[i] = memo.heading_platform[i+1];
			}
			memo.heading_platform[LENGTH_MEMO_SENSOR_DATA-1] = nav.heading_platform;
		}

		// QS
		if (memo.i_gyro < EPH_DETECT_QS) {
			find_max_min_double(memo.gz_level, max_min_qs_detect, memo.i_gyro);
		} else {
			find_max_min_double((memo.gz_level)+memo.i_gyro-EPH_DETECT_QS, max_min_qs_detect, EPH_DETECT_QS);
		}
		diff_max_min_gz = max_min_qs_detect[0] - max_min_qs_detect[1];

		if_qs = 0;
		if (diff_max_min_gz < TH_DETECT_QS_GYRO) {
			if_qs = 1;
		}

		if (!if_qs) {
			nav.heading_platform += memo.gz_level[memo.i_gyro-1]*ts_gyro;
			nav.heading_platform = center_heading(nav.heading_platform);
		}

		//sol->count_rec_heading ++;
		sol->avail_heading = 1;
		sol->sol_t_heading = memo.t_gyro[memo.i_gyro-1];
		sol->sol_heading = memo.heading_platform[memo.i_gyro-1]/D2R;
		//fprintf(sol.f_out_rec_heading, "%.4f %.4f\n", memo.t_gyro[memo.i_gyro-1], memo.heading_platform[memo.i_gyro-1]);

	}
	// -- End Heading

	// -- Step detection and PDR
	if (meas.avail_accel) {
		if_detect_step = 0;
		if_update_step = 0;

		if (memo.i_ma == LENGTH_MEMO_MA) {
			if_detect_step = 1;
		}
		if (if_detect_step) {
			ma_mid1 = memo.ma[EPH_COMPARE_SIDE];
			min_t1 = min_f(memo.ma, LENGTH_MEMO_MA);
			if (fabs(ma_mid1-min_t1)<10e-6 && ma_mid1<= TH_A_STEP2){

				i_mid_in_memo = memo.i_accel - EPH_COMPARE_SIDE;  //
				t_mid1 = memo.t_accel[i_mid_in_memo-1];

				if_update_step = 1;
				if (memo.i_t_step) {
					if (t_mid1-memo.t_step[memo.i_t_step-1]<TH_MIN_T_DIFF_STEP){
						if_update_step = 0;
					}
				}

				if (if_update_step) {
					//sol->count_rec_step ++;
					sol->avail_step = 1;
					sol->sol_t_step = t_mid1;
					sol->sol_step = ma_mid1;
					//fprintf(sol.f_out_rec_step, "%.4f %.4f\n", t_mid1, ma_mid1);

					memo.i_t_step ++;
					if(memo.i_t_step <=LENGTH_MEMO_T_STEP){
						memo.t_step[memo.i_t_step-1] = t_mid1;
					} else {
						memo.i_t_step = LENGTH_MEMO_T_STEP;
						for (i=0; i<LENGTH_MEMO_T_STEP-1; i++) {
							memo.t_step[i] = memo.t_step[i+1];
						}
						memo.t_step[LENGTH_MEMO_T_STEP-1] = t_mid1;
					}

					nav.t_step0 = nav.t_step1;
					nav.t_step1 = t_mid1;
					nav.t_s_step = nav.t_step1 - nav.t_step0;

					if_update_pos = 1;
				}
			}

			if (if_update_pos) {    // Update PDR POS
				nav.pos[0] += SL*cos(nav.heading_step)/RM;
				nav.pos[1] += SL*sin(nav.heading_step)/RN_COS_LON;
				nav.pos[2] = nav.pos[2];

				// nav.pos_NEH
				// ....

				//

				// KF prediction
				// ...

				//
				for (i=0; i<LENGTH_MEMO_SENSOR_DATA; i++){
					t_diff_g_a[i] = fabs(t_mid1 - memo.t_gyro[i]);
				}
				min_double(t_diff_g_a, min_t_heading_in_memo, LENGTH_MEMO_SENSOR_DATA);
				i_heading_in_memo = floor(min_t_heading_in_memo[1]+0.001);
				nav.heading_step = memo.heading_platform[i_heading_in_memo];

				memo.i_pos_pdr ++;
				if(memo.i_pos_pdr <= LENGTH_MEMO_POS_PDR){
					memo.t_pos_pdr[memo.i_pos_pdr-1] = t_mid1;
					memo.pos_pdr_lat[memo.i_pos_pdr-1] = nav.pos[0];
					memo.pos_pdr_lon[memo.i_pos_pdr-1] = nav.pos[1];
					memo.pos_pdr_hei[memo.i_pos_pdr-1] = nav.pos[2];
				} else {
					memo.i_pos_pdr = LENGTH_MEMO_POS_PDR;
					for (i=0; i<LENGTH_MEMO_POS_PDR-1; i++) {
						memo.t_pos_pdr[i] = memo.t_pos_pdr[i+1];
						memo.pos_pdr_lat[i] = memo.pos_pdr_lat[i+1];
						memo.pos_pdr_lon[i] = memo.pos_pdr_lon[i+1];
						memo.pos_pdr_hei[i] = memo.pos_pdr_hei[i+1];
					}
					memo.t_pos_pdr[LENGTH_MEMO_POS_PDR-1] = t_mid1;
					memo.pos_pdr_lat[LENGTH_MEMO_POS_PDR-1] = nav.pos[0];
					memo.pos_pdr_lon[LENGTH_MEMO_POS_PDR-1] = nav.pos[1];
					memo.pos_pdr_hei[LENGTH_MEMO_POS_PDR-1] = nav.pos[2];
				}

				// POS!
				//sol.count_rec_pos ++;
				sol->avail_sol_pdr = 1;
				sol->sol_t_sol_pdr = memo.t_pos_pdr[memo.i_pos_pdr-1];
				sol->sol_pdr[0] = memo.pos_pdr_lat[memo.i_pos_pdr-1]/D2R;
				sol->sol_pdr[1] = memo.pos_pdr_lon[memo.i_pos_pdr-1]/D2R;
				sol->sol_pdr[2] = memo.pos_pdr_hei[memo.i_pos_pdr-1];

				//fprintf(sol.f_out_rec_pos, "%.4f %.8f %.8f %.3f\n",
				//		memo.t_pos_pdr[memo.i_pos_pdr-1], memo.pos_pdr_lat[memo.i_pos_pdr-1]/D2R,
				//		memo.pos_pdr_lon[memo.i_pos_pdr-1]/D2R, memo.pos_pdr_hei[memo.i_pos_pdr-1]);

			} // end if_update_pos


		} // end if_detect_step

	} // end if avail_accel
	// -- End  Step detection and PDR


	// WiFi
	if (IF_USE_WIFI && meas.avail_wifi)
	{
		num_floor[0] = 1;
        wifi_position_tracker_smooth(meas.name_ap, meas.rss_ap, meas.N_wifi, num_floor, pos_wifi, availability_wifi);

		if(availability_wifi[0]) {
			sol->avail_sol_wifi = 1;
			sol->sol_t_sol_wifi = meas.t_wifi;
			sol->sol_wifi[0] = pos_wifi[0]/D2R;
			sol->sol_wifi[1] = pos_wifi[1]/D2R;
		}
	}


	// Set as "0"
	//meas.avail_gyro = 0;
	//meas.avail_accel = 0;
}


void indoor_navigation(Meas meas, Sol_Pkt*  sol){
	static UINT8 fitst_time_flag = 0;
	static NavIndoor nav;
	static MemoIndoor memo;

	static FLOAT32 gyro_bias[3], accel_bias[3];
	int i, j, i_mid_in_memo, i_heading_in_memo;
	double min_t_heading_in_memo[2];
	double max_min_qs_detect[2], diff_max_min_gz;
	//float norma1, roll1, pitch1, f_b1[3], sr1, cr1, sp1, cp1, wd1;
	float ma_mid1, min_t1;
	float heading;
	double t_mid1, lat1, ts_gyro;
	double t_diff_g_a[LENGTH_MEMO_SENSOR_DATA];

	int if_detect_step, if_update_step, if_qs;
	int if_update_pos = 0;
	static double t0_gyro = 0.00;
	static double t0_accel = 0.00;
	static double t_pre_gyro = 0.00;
	static double t_pre_accel = 0.00;
	static int if_first_t_accel = 0;
	static int if_first_t_gyro = 0;

	// Set indicators in sol to "0"
	sol->avail_sol_pdr = 0;
	sol->avail_step = 0;
	sol->avail_heading = 0;


	if (meas.avail_gyro) {
		if (if_first_t_gyro) {
			meas.t_gyro -= t0_gyro;  // t0_gyro is used for time_system for gyro
			if (fabs(meas.t_gyro-t_pre_gyro) < 10e-6) {  // t_pre_gyro is used for gyro time in the previous epoch
				meas.avail_gyro = 0;
			}
		}
		else {
			t0_gyro = meas.t_gyro;
			meas.t_gyro = 0;
			if_first_t_gyro = 1;
		}
		t_pre_gyro = meas.t_gyro;
	}

	if (meas.avail_accel) {
		if (if_first_t_accel) {
			meas.t_accel -= t0_accel;
			if (fabs(meas.t_accel-t_pre_accel) < 10e-6) {
				meas.avail_accel = 0;
			}
		}
		else {
			t0_accel = meas.t_accel;
			meas.t_accel = 0;
			if_first_t_accel = 1;
		}
		t_pre_accel = meas.t_accel;
	}


	//first time
	if (!fitst_time_flag){
		fitst_time_flag = 1;

		// for the first wifi data
		meas.t_gyro = 0;
		meas.t_accel = 0;

		//
		gyro_bias[0] = ini_gyro_bias_att[0];
		gyro_bias[1] = ini_gyro_bias_att[1];
		gyro_bias[2] = ini_gyro_bias_att[2];   // Later we can save and read gBias from a file

		accel_bias[0] = ini_accel_bias_att[0];
		accel_bias[1] = ini_accel_bias_att[1];
		accel_bias[2] = ini_accel_bias_att[2];   // Later we can save and read gBias from a file


		// Nav.P
		// Nav.Q

		// -- Initialization PDR
		nav.pos[0] = INI_POS[0];
		nav.pos[1] = INI_POS[1];
		nav.pos[2] = INI_POS[2];

		if (TYPE_INI_HEADING == 1) {
			nav.heading_platform = INI_HEADING;
		} else {    // Other strategies

		}
		nav.heading_step = nav.heading_platform;
		nav.t_step1 = 0;


		//-- End Initialization PDR



		// ---- INITIALIZAION_MEMO
		memo.i_accel = 0;
		memo.i_gyro = 0;
		memo.i_ma = 0;
		memo.i_t_step = 0;
		memo.i_pos_pdr = 0;

		for(i=0; i<LENGTH_MEMO_SENSOR_DATA; i++){
			memo.t_accel[i] = 0.00;
			memo.a_x[i] = 0.0;
			memo.a_y[i] = 0.0;
			memo.a_z[i] = 0.0;
			memo.t_gyro[i] = 0.00;
			memo.g_x[i] = 0.0;
			memo.g_y[i] = 0.0;
			memo.g_z[i] = 0.0;
			memo.gz_level[i] = 0.0;
			memo.heading_platform[i] = 0.0;
		}
		for(i=0; i<LENGTH_MEMO_MA; i++){
			memo.ma[i] = 0.0;
		}
		for(i=0; i<LENGTH_MEMO_T_STEP; i++) {
			memo.t_step[i] = 0.00;
		}
		for(i=0; i<LENGTH_MEMO_POS_PDR; i++) {
			memo.t_pos_pdr[i] = 0.00;
			memo.pos_pdr_lat[i] = 0.00;
			memo.pos_pdr_lon[i] = 0.00;
			memo.pos_pdr_hei[i] = 0.00;
		}
		// ---- End INITIALIZAION_MEMO


		memo.i_pos_pdr ++;
		memo.pos_pdr_lat[memo.i_pos_pdr-1] = nav.pos[0];
		memo.pos_pdr_lon[memo.i_pos_pdr-1] = nav.pos[1];
		memo.pos_pdr_hei[memo.i_pos_pdr-1] = nav.pos[2];


		/*if (meas.avail_accel){
			memo.t_pos_pdr[memo.i_pos_pdr-1] = 0.00;
		} else if (meas.avail_gyro) {
			memo.t_pos_pdr[memo.i_pos_pdr-1] = 0.00;
		} else {
			memo.t_pos_pdr[memo.i_pos_pdr-1] = 0.00;
		}*/

		// Output
		//sol.count_rec_pos ++;
		sol->avail_sol_pdr = 1;
		sol->sol_t_sol_pdr = memo.t_pos_pdr[memo.i_pos_pdr-1];
		sol->sol_pdr[0] = memo.pos_pdr_lat[memo.i_pos_pdr-1]/D2R;
		sol->sol_pdr[1] = memo.pos_pdr_lon[memo.i_pos_pdr-1]/D2R;
		sol->sol_pdr[2] = memo.pos_pdr_hei[memo.i_pos_pdr-1];

		//fprintf(sol->f_out_rec_pos, "%.4f %.8f %.8f %.3f\n",
		//		memo.t_pos_pdr[memo.i_pos_pdr-1], memo.pos_pdr_lat[memo.i_pos_pdr-1]/D2R,
		//		memo.pos_pdr_lon[memo.i_pos_pdr-1]/D2R, memo.pos_pdr_hei[memo.i_pos_pdr-1]);

	}  // End (!fitst_time_flag)


	// Start
	if (meas.avail_gyro) {
		meas.da_gyro[0] = meas.da_gyro[0] * D2R - gyro_bias[0];
		meas.da_gyro[1] = meas.da_gyro[1] * D2R - gyro_bias[1];
		meas.da_gyro[2] = meas.da_gyro[2] * D2R - gyro_bias[2];
	}

	if (meas.avail_accel) {
		meas.da_accel[0]  -= accel_bias[0];
		meas.da_accel[1]  -= accel_bias[1];
		meas.da_accel[2]  -= accel_bias[2];
	}


	// -- Put into memo
	//real_time_put_into_memo(&memo, meas);
	float norma1, roll1, pitch1, f_b1[3], sr1, cr1, sp1, cp1, wd1;
	if (meas.avail_accel){
		memo.i_accel ++;
		if (memo.i_accel <= LENGTH_MEMO_SENSOR_DATA) {
			memo.t_accel[memo.i_accel-1] = meas.t_accel;
			memo.a_x[memo.i_accel-1] = meas.da_accel[0];
			memo.a_y[memo.i_accel-1] = meas.da_accel[1];
			memo.a_z[memo.i_accel-1] = meas.da_accel[2];
		}else {
			memo.i_accel = LENGTH_MEMO_SENSOR_DATA;
			for (i=0; i<LENGTH_MEMO_SENSOR_DATA-1; i++) {
				memo.t_accel[i] = memo.t_accel[i+1];
				memo.a_x[i] = memo.a_x[i+1];
				memo.a_y[i] = memo.a_y[i+1];
				memo.a_z[i] = memo.a_z[i+1];
			}
			memo.t_accel[LENGTH_MEMO_SENSOR_DATA-1] = meas.t_accel;
			memo.a_x[LENGTH_MEMO_SENSOR_DATA-1] = meas.da_accel[0];
			memo.a_y[LENGTH_MEMO_SENSOR_DATA-1] = meas.da_accel[1];
			memo.a_z[LENGTH_MEMO_SENSOR_DATA-1] = meas.da_accel[2];
		}

		norma1 = sqrt(meas.da_accel[0]*meas.da_accel[0]+meas.da_accel[1]*meas.da_accel[1]+meas.da_accel[2]*meas.da_accel[2]);
		memo.i_ma ++;
		if(memo.i_ma <= LENGTH_MEMO_MA){
			memo.ma[memo.i_ma-1] = norma1;
		} else {
			memo.i_ma = LENGTH_MEMO_MA;
			for (i=0; i<LENGTH_MEMO_MA-1; i++) {
				memo.ma[i] = memo.ma[i+1];
			}
			memo.ma[LENGTH_MEMO_MA-1] = norma1;

		}
	}

	if (meas.avail_gyro) {
		if(!memo.i_accel) {
			roll1 = 0.0;
			pitch1 = 0.0;
		} else{
			f_b1[0] = memo.a_x[memo.i_accel-1];
			f_b1[1] = memo.a_y[memo.i_accel-1];
			f_b1[2] = memo.a_z[memo.i_accel-1];
			roll1 = cal_roll(f_b1);
			pitch1 = cal_pitch(f_b1);
		}
		sr1 = sin(roll1); cr1 = cos(roll1);
		sp1 = sin(pitch1); cp1 = cos(pitch1);
		wd1 = -sp1*meas.da_gyro[0] + sr1*cp1*meas.da_gyro[1] + cr1*cp1*meas.da_gyro[2];

		memo.i_gyro ++;
		if (memo.i_gyro <= LENGTH_MEMO_SENSOR_DATA) {
			memo.t_gyro[memo.i_gyro-1] = meas.t_gyro;
			memo.g_x[memo.i_gyro-1] = meas.da_gyro[0];
			memo.g_y[memo.i_gyro-1] = meas.da_gyro[1];
			memo.g_z[memo.i_gyro-1] = meas.da_gyro[2];
			memo.gz_level[memo.i_gyro-1] = wd1;
		}else {
			memo.i_gyro = LENGTH_MEMO_SENSOR_DATA;
			for (i=0; i<LENGTH_MEMO_SENSOR_DATA-1; i++) {
				memo.t_gyro[i] = memo.t_gyro[i+1];
				memo.g_x[i] = memo.g_x[i+1];
				memo.g_y[i] = memo.g_y[i+1];
				memo.g_z[i] = memo.g_z[i+1];
				memo.gz_level[i] = memo.gz_level[i+1];
			}
			memo.t_gyro[LENGTH_MEMO_SENSOR_DATA-1] = meas.t_gyro;
			memo.g_x[LENGTH_MEMO_SENSOR_DATA-1] = meas.da_gyro[0];
			memo.g_y[LENGTH_MEMO_SENSOR_DATA-1] = meas.da_gyro[1];
			memo.g_z[LENGTH_MEMO_SENSOR_DATA-1] = meas.da_gyro[2];
			memo.gz_level[LENGTH_MEMO_SENSOR_DATA-1] = wd1;
		}

	}


	// -- End Put into memo

	// -- Heading
	if (meas.avail_gyro) {
		ts_gyro = 0.00;
		if (memo.i_gyro>1) {
			ts_gyro = memo.t_gyro[memo.i_gyro-1]-memo.t_gyro[memo.i_gyro-2];
			//printf("ts_gyro = %.4f\n", ts_gyro);
		}

		if (memo.i_gyro < LENGTH_MEMO_SENSOR_DATA) {
			memo.heading_platform[memo.i_gyro-1] = nav.heading_platform;
		} else {

			for (i=0; i<LENGTH_MEMO_SENSOR_DATA-1; i++) {
				memo.heading_platform[i] = memo.heading_platform[i+1];
			}
			memo.heading_platform[LENGTH_MEMO_SENSOR_DATA-1] = nav.heading_platform;
		}

		// QS
		if (memo.i_gyro < EPH_DETECT_QS) {
			find_max_min_double(memo.gz_level, max_min_qs_detect, memo.i_gyro);
		} else {
			find_max_min_double((memo.gz_level)+memo.i_gyro-EPH_DETECT_QS, max_min_qs_detect, EPH_DETECT_QS);
		}
		diff_max_min_gz = max_min_qs_detect[0] - max_min_qs_detect[1];

		if_qs = 0;
		if (diff_max_min_gz < TH_DETECT_QS_GYRO) {
			if_qs = 1;
		}

		if (!if_qs) {
			nav.heading_platform += memo.gz_level[memo.i_gyro-1]*ts_gyro;
			nav.heading_platform = center_heading(nav.heading_platform);
		}

		//sol->count_rec_heading ++;
		sol->avail_heading = 1;
		sol->sol_t_heading = memo.t_gyro[memo.i_gyro-1];
		sol->sol_heading = memo.heading_platform[memo.i_gyro-1]/D2R;
		//fprintf(sol.f_out_rec_heading, "%.4f %.4f\n", memo.t_gyro[memo.i_gyro-1], memo.heading_platform[memo.i_gyro-1]);

	}
	// -- End Heading

	// -- Step detection and PDR
	if (meas.avail_accel) {
		if_detect_step = 0;
		if_update_step = 0;

		if (memo.i_ma == LENGTH_MEMO_MA) {
			if_detect_step = 1;
		}
		if (if_detect_step) {


			ma_mid1 = memo.ma[EPH_COMPARE_SIDE];
			min_t1 = min_f(memo.ma, LENGTH_MEMO_MA);
			if (fabs(ma_mid1-min_t1)<10e-6 && ma_mid1<= TH_A_STEP2){

				i_mid_in_memo = memo.i_accel - EPH_COMPARE_SIDE;  //
				t_mid1 = memo.t_accel[i_mid_in_memo-1];

				if_update_step = 1;
				if (memo.i_t_step) {
					if (t_mid1-memo.t_step[memo.i_t_step-1]<TH_MIN_T_DIFF_STEP){
						if_update_step = 0;
					}
				}

				if (if_update_step) {
					//sol->count_rec_step ++;
					sol->avail_step = 1;
					sol->sol_t_step = t_mid1;
					sol->sol_step = ma_mid1;
					//fprintf(sol.f_out_rec_step, "%.4f %.4f\n", t_mid1, ma_mid1);

					memo.i_t_step ++;
					if(memo.i_t_step <=LENGTH_MEMO_T_STEP){
						memo.t_step[memo.i_t_step-1] = t_mid1;
					} else {
						memo.i_t_step = LENGTH_MEMO_T_STEP;
						for (i=0; i<LENGTH_MEMO_T_STEP-1; i++) {
							memo.t_step[i] = memo.t_step[i+1];
						}
						memo.t_step[LENGTH_MEMO_T_STEP-1] = t_mid1;
					}

					nav.t_step0 = nav.t_step1;
					nav.t_step1 = t_mid1;
					nav.t_s_step = nav.t_step1 - nav.t_step0;

					if_update_pos = 1;
				}
			}

			if (if_update_pos) {    // Update PDR POS
				nav.pos[0] += SL*cos(nav.heading_step)/RM;
				nav.pos[1] += SL*sin(nav.heading_step)/RN_COS_LON;
				nav.pos[2] = nav.pos[2];

				// nav.pos_NEH
				// ....

				//

				// KF prediction
				// ...

				//
				for (i=0; i<LENGTH_MEMO_SENSOR_DATA; i++){
					t_diff_g_a[i] = fabs(t_mid1 - memo.t_gyro[i]);
				}
				min_double(t_diff_g_a, min_t_heading_in_memo, LENGTH_MEMO_SENSOR_DATA);
				i_heading_in_memo = floor(min_t_heading_in_memo[1]+0.001);
				nav.heading_step = memo.heading_platform[i_heading_in_memo];

				memo.i_pos_pdr ++;
				if(memo.i_pos_pdr <= LENGTH_MEMO_POS_PDR){
					memo.t_pos_pdr[memo.i_pos_pdr-1] = t_mid1;
					memo.pos_pdr_lat[memo.i_pos_pdr-1] = nav.pos[0];
					memo.pos_pdr_lon[memo.i_pos_pdr-1] = nav.pos[1];
					memo.pos_pdr_hei[memo.i_pos_pdr-1] = nav.pos[2];
				} else {
					memo.i_pos_pdr = LENGTH_MEMO_POS_PDR;
					for (i=0; i<LENGTH_MEMO_POS_PDR-1; i++) {
						memo.t_pos_pdr[i] = memo.t_pos_pdr[i+1];
						memo.pos_pdr_lat[i] = memo.pos_pdr_lat[i+1];
						memo.pos_pdr_lon[i] = memo.pos_pdr_lon[i+1];
						memo.pos_pdr_hei[i] = memo.pos_pdr_hei[i+1];
					}
					memo.t_pos_pdr[LENGTH_MEMO_POS_PDR-1] = t_mid1;
					memo.pos_pdr_lat[LENGTH_MEMO_POS_PDR-1] = nav.pos[0];
					memo.pos_pdr_lon[LENGTH_MEMO_POS_PDR-1] = nav.pos[1];
					memo.pos_pdr_hei[LENGTH_MEMO_POS_PDR-1] = nav.pos[2];
				}

				// POS!
				//sol.count_rec_pos ++;
				sol->avail_sol_pdr = 1;
				sol->sol_t_sol_pdr = memo.t_pos_pdr[memo.i_pos_pdr-1];
				sol->sol_pdr[0] = memo.pos_pdr_lat[memo.i_pos_pdr-1]/D2R;
				sol->sol_pdr[1] = memo.pos_pdr_lon[memo.i_pos_pdr-1]/D2R;
				sol->sol_pdr[2] = memo.pos_pdr_hei[memo.i_pos_pdr-1];

				//fprintf(sol.f_out_rec_pos, "%.4f %.8f %.8f %.3f\n",
				//		memo.t_pos_pdr[memo.i_pos_pdr-1], memo.pos_pdr_lat[memo.i_pos_pdr-1]/D2R,
				//		memo.pos_pdr_lon[memo.i_pos_pdr-1]/D2R, memo.pos_pdr_hei[memo.i_pos_pdr-1]);

			} // end if_update_pos


		} // end if_detect_step

	} // end if avail_accel
	// -- End  Step detection and PDR


	//int  iiii = 0 ;

	// Set as "0"
	//meas.avail_gyro = 0;
	//meas.avail_accel = 0;
}



// -------------------------------- Function Attitude and Misalignment -----------------------------------

/////////////////////////////////  Start of Code ///////////////////////////////////////////////////
/**
* @brief This function provides the high and low peaks for a 1-D vector.
*
* @details The function takes a pointer to vector of 2 seconds data and return the indices
* 		   and values of the low and high peaks. The functions sets the fileds of a pointer to
* 		   structure provided as outputs (peaks).
*
* @param[in] data pointer to a vector of data
* @param[in] delta variable to the signal slope threshold
* @param[in] min_peaks_num variable to the minimum number of peaks to be targeted
* @param[in] peaks_vector_size varable to the size of the vector that store the peaks
* @param[in] data_size varable to the size of the data vector
* @param[in] protection_use_flag a flag to decide using protection of peak at the first sample or not
* @param[out] peaks pointer to a structure of type \c PEAKS
*
* @return returns 1 if successful otherwise, returns 0.
*/
INT8 peaks_detection_mis(FLOAT32 *data, FLOAT32 delta, INT8 min_peaks_num, PEAKS_Ptr peaks, UINT8 peaks_vector_size, UINT8 data_size, UINT8 protection_use_flag)
{
	INT8 i;			// loop counter
	INT8 start_ind;
	INT8 high_peaks_counter = 0, low_peaks_counter = 0;
	INT8 min_val_index = 0, max_val_index = 0, lookformax = 1;
	FLOAT32 min_val =  999.0, max_val = -999.0;

	for(i = 0; i < peaks_vector_size; i++)
	{
		peaks->low_ind[i]  = -100;
		peaks->high_ind[i] = -100;
		peaks->low_val[i]  = 1000.0;
		peaks->high_val[i] = 1000.0;
	}

	// detect the change in the data slope
	if((data[1]-data[0]) > delta)
		start_ind = 1;
	else
		start_ind = 2;

	if((data[start_ind]-data[0]) > 0.0)
		lookformax = 1;			// look for high peak
	else
		lookformax = 0;			// look for low peak

	for(i = 0; i < data_size; i++)
	{
		// get maximum value and its index
		if(data[i] > max_val)
		{
			max_val = data[i];
			max_val_index = i;
		}
		// get minimum value and its index
		if(data[i] < min_val)
		{
			min_val = data[i];
			min_val_index = i;
		}

		if(lookformax)   // high peak
		{
			if(data[i] < (max_val - delta) && high_peaks_counter < peaks_vector_size)
			{
				peaks->high_ind[high_peaks_counter] = max_val_index;
				peaks->high_val[high_peaks_counter] = max_val;
				min_val = data[i];
				min_val_index = i;
				high_peaks_counter++;
				lookformax = 0;
			}
		}
		else	// low peak
		{
			if(data[i] > (min_val + delta) && low_peaks_counter < peaks_vector_size)
			{
				peaks->low_ind[low_peaks_counter] = min_val_index;
				peaks->low_val[low_peaks_counter] = min_val;
				max_val = data[i];
				max_val_index = i;
				low_peaks_counter++;
				lookformax = 1;
			}
		}
	}

	if(protection_use_flag == 1)
	{
		if(low_peaks_counter > 0 && high_peaks_counter > 0)
		{
			if(peaks->low_ind[0] < peaks->high_ind[0])
			{
				if(data[0]-peaks->low_val[0] < delta || peaks->low_ind[0] < 2)
				{
					for(i = 1; i < low_peaks_counter; i++)
					{
						peaks->low_ind[i-1] = peaks->low_ind[i];
						peaks->low_val[i-1] = peaks->low_val[i];
					}
					low_peaks_counter--;
				}
			}
			else
			{
				if(peaks->high_val[0] - data[0] < delta || peaks->high_ind[0] < 2)
				{
					for(i = 1; i < high_peaks_counter; i++)
					{
						peaks->high_ind[i-1] = peaks->high_ind[i];
						peaks->high_val[i-1] = peaks->high_val[i];
					}
					high_peaks_counter--;
				}
			}
		}
	}

	peaks->num[0] = low_peaks_counter;
	peaks->num[1] = high_peaks_counter;

	if(low_peaks_counter >= min_peaks_num || high_peaks_counter >= min_peaks_num)
		return 1;		// function detected the required number of peaks
	else
		return 0;		// function couldn't detect the required number of peaks
}

/**
* @brief This function provides information about motion stability.
*
* @details The function takes 1 pointer to structure of type PCA. it takes 2 pointers to
* 		   vector of 2 seconds for acceleration variance data and vector of 2 second history data of
* 		   step frequency values. the function returns the value of the motion unstability flag
*
* @param[in] pca pointer to a structure of type \c PCA
* @param[in] acc_var_vec pointer to vector of 2 seconds of variable \c FLOAT32
* @param[in] stp_frq_vec pointer to vector of the last 2 seconds of variable \c FLOAT32
* @param[out] Step_Frequency pointer to a variable of type \c FLOAT32
*
* @return motion stability status 0 or 1
*/
#ifdef PRINT_RESULTS_MISALIGNMENT
UINT8 peak_frequency_calculation_mis(PCA_Ptr pca, FLOAT32 *acc_var_vec, FLOAT32 *acc_var, FLOAT32 *stp_frq_vec, FLOAT32 *Step_Frequency, FLOAT32 *Par_Values)
#else
UINT8 peak_frequency_calculation_mis(PCA_Ptr pca, FLOAT32 *acc_var_vec, FLOAT32 *acc_var, FLOAT32 *stp_frq_vec, FLOAT32 *Step_Frequency)
#endif
{
	INT8 i;
	INT8 peaks_count = 0, v_peaks_index[DATA_SIZE_MIS] = {100};

	FLOAT32 acc_var_mean = 0.0;
	FLOAT32 step_freq = 0.0, step_freq_range = 0.0, step_ranges_thr = 0.0;
	FLOAT32 min_step_freq = 99.0, max_step_freq = -99.0;
	FLOAT32 min_v = 99.0, max_v = -99.0;

	PCA_C pca_c;
	memset(&pca_c, 0, sizeof(PCA_C));

	pca->vertical_parametrs[0] = pca->vertical_parametrs[1] = 0.0;

	for(i = 0; i < DATA_SIZE_MIS; i++)
	{
		pca->vertical_parametrs[0] += pca->data_v[i];
		acc_var_mean += acc_var_vec[i];
		if(pca->data_v[i] > max_v)
			max_v = pca->data_v[i];
		if(pca->data_v[i] < min_v)
			min_v = pca->data_v[i];
	}

	pca->vertical_parametrs[0] *= ONE_BY_DATA_SIZE_MIS_MEAN;
	pca->vertical_parametrs[2]  = fabs(max_v - min_v);
	acc_var_mean  *= ONE_BY_DATA_SIZE_MIS_MEAN;
	*acc_var = acc_var_mean;
	for(i = 0; i < DATA_SIZE_MIS; i++)
		pca->vertical_parametrs[1] += SQR(pca->data_v[i]  - pca->vertical_parametrs[0]);

	pca->vertical_parametrs[1] = sqrt(pca->vertical_parametrs[1] * ONE_BY_DATA_SIZE_MIS_STD);

	// Vertical Peaks Detection
	i = peaks_detection_mis(pca->data_v, pca->vertical_parametrs[2]*0.125F, 4, &pca_c.vertical_peaks, 20, 36, 0);
	if((pca_c.vertical_peaks.num[0] < 2 || pca_c.vertical_peaks.num[1] < 2) && pca->vertical_parametrs[2]*0.125F > pca->vertical_parametrs[1])
		i = peaks_detection_mis(pca->data_v, pca->vertical_parametrs[1], 4, &pca_c.vertical_peaks, 20, 36, 0);

	//Crash protection, ZUPT
	if(pca->vertical_parametrs[2] < 0.8)
	{
		*Step_Frequency = 0;
		return 0;
	}
	else
	{
		/////////////////// Step frequency
		for(i = 0; i < pca_c.vertical_peaks.num[1]; i++)
			v_peaks_index[peaks_count++] = pca_c.vertical_peaks.high_ind[i];

		for(i = 0; i < pca_c.vertical_peaks.num[0]; i++)
			v_peaks_index[peaks_count++] = pca_c.vertical_peaks.low_ind[i];

		BubbleSort_mis(v_peaks_index, peaks_count, 1);

		step_freq = 0.0;
		for(i = 1; i < peaks_count; i++)
			step_freq = step_freq + 1.0F/(v_peaks_index[i]- v_peaks_index[i-1]);

		step_freq = 20*(step_freq/(peaks_count - 1));
		*Step_Frequency = step_freq;

		/////////////////// Calculate the Step Frequency range
		for(i = 1; i < DATA_SIZE_MIS; i++)
		{
			if(stp_frq_vec[i] > max_step_freq)
				max_step_freq = stp_frq_vec[i];
			if(stp_frq_vec[i] < min_step_freq)
				min_step_freq = stp_frq_vec[i];
		}
		max_step_freq = MAX(max_step_freq, step_freq);
		min_step_freq = MIN(min_step_freq, step_freq);
		step_freq_range = fabs(max_step_freq - min_step_freq);

		/////////////////// Step range threshold
		if(acc_var_mean <= 10)
			step_ranges_thr = 1;
		else if(acc_var_mean <= 20)
			step_ranges_thr = 2;
		else if(acc_var_mean <= 30)
			step_ranges_thr = 3;
		else if(acc_var_mean > 30)
			step_ranges_thr = 4;

		/////////////////////////////////////// Print Results  /////////////////////////////
#ifdef PRINT_RESULTS_MISALIGNMENT
		//Par_Values[30] =  acc_var_mean;						// Acceleration Variance			(31)
		//Par_Values[31] =  step_freq;					// Step Frequency					(32)
		//Par_Values[32] =  step_freq_range;				// Step Frequency range				(33)
		//Par_Values[33] =  step_ranges_thr;				// Step Frequency range thresholds	(34)
		//Par_Values[34] =  (FLOAT32)peaks_count;			// No. of Peaks						(35)
#endif
		/////////////////////////////////////// End of Print Results  /////////////////////////////

		// Return the value
		if(floorf(step_freq_range * 10000) / 10000 > floorf(step_ranges_thr * 10000) / 10000)
			return 1;
		else
			return 0;
	}
}

#ifndef COM_WALK_SIMPLIFIED
/**
* @brief This function rmove noise from the data.
*
* @details The function takes x and y acceleration data and roll and pitch data.
*          it takes vector of 2 second history data of x and y frequencies. also,
*          it receives the device flag and the motion stability falg.
*          the function returns the acceleration data without noise.
*
* @param[in, out] xy_data            pointer to matrix 2D for x and y data
* @param[in] data_r		            pointer to roll data vector.
* @param[in] data_p		            pointer to pitch data vector.
* @param[in] device flag            variable for the device type.
* @param[in] motion_unstability_flag    variable for the motion stability flag
* @param[in] x_history_vec			pointer to 1 second of data x frequency histor
* @param[in] y_history_vec			pointer to 1 second of data y frequency histor
* @param[in] mode2_fft_mis				variable for device use case classification
* @param[out] x_current				pointer to the current x ferquencies values
* @param[out] y_current				pointer to the current y ferquencies values
*
*/
void fft_mis_calculation_mis(FLOAT32 **xy_data, FLOAT32 *data_r, FLOAT32 *data_p, FLOAT32 acc_var, INT8 device_flag, UINT8 motion_unstability_flag, FLOAT32 **x_history_vec, FLOAT32 *x_current, FLOAT32 **y_history_vec, FLOAT32 *y_current, UINT8 mode2_fft_mis)
{
	INT8 i, j;

	FLOAT32 min_r = 99.0, max_r = -99.0;
	FLOAT32 min_p = 99.0, max_p = -99.0;
	FLOAT32 range_r,range_p, max_range_rp;

	FLOAT32 new_ma[NFFT_FFT_MIS];
	FLOAT32 PS_x[NFFT_FFT_MIS/2+1], PS_y[NFFT_FFT_MIS/2+1];
	FLOAT32 Freqs_M[NFFT_FFT_MIS][2], *pFreqs_M[NFFT_FFT_MIS];
	FLOAT32 tempFreqs_M[NFFT_FFT_MIS][2], *ptempFreqs_M[NFFT_FFT_MIS];
	FLOAT32 x_data[WINDOW_SIZE_MIS], y_data[WINDOW_SIZE_MIS];
	FLOAT32 x_data_avg =0, y_data_avg =0;

	FLOAT32 x_history_avg[10] = {0.0};
	FLOAT32 y_history_avg[10] = {0.0};

	FLOAT32 fft_mis_avg_factor = 0.05F;
	INT8 start_index = 0, end_index = 0;

	FLOAT32 x_peak_val = 0, y_peak_val = 0;
	INT8 x_peak_index = 0, y_peak_index = 0, peak_index = 0;

	FLOAT32 x_cos_coef[NFFT_FFT_MIS/2+1], x_sin_coef[NFFT_FFT_MIS/2+1], y_cos_coef[NFFT_FFT_MIS/2+1], y_sin_coef[NFFT_FFT_MIS/2+1];
	FLOAT32 f_FT[NFFT_FFT_MIS/2+1];

	for(i = 0; i < NFFT_FFT_MIS/2+1; i++)
		f_FT[i] = 10.0F*i/(NFFT_FFT_MIS/2.0F);

	for(i = 0; i < DATA_SIZE_MIS; i++)
	{
		if(data_r[i] > max_r)
			max_r = data_r[i];
		if(data_r[i] < min_r)
			min_r = data_r[i];

		if(data_p[i] > max_p)
			max_p = data_p[i];
		if(data_p[i] < min_p)
			min_p = data_p[i];
	}

	range_r = fabs(max_r - min_r);
	range_p = fabs(max_p - min_p);

	max_range_rp = MAX(range_r, range_p)*R2Df;

	for(i=0;i<WINDOW_SIZE_MIS;i++)
	{
		x_data[i] = xy_data[0][i];
		y_data[i] = xy_data[1][i];
	}

	ChangeArrayMemoryLayout_float_att_float_mis(NFFT_FFT_MIS, 2, pFreqs_M, (FLOAT32 *)Freqs_M, 1);
	ChangeArrayMemoryLayout_float_att_float_mis(NFFT_FFT_MIS, 2, ptempFreqs_M, (FLOAT32 *)tempFreqs_M, 1);

	zero_matrix_entries_nxn_float_mis(pFreqs_M, NFFT_FFT_MIS, 2);
	zero_matrix_entries_nxn_float_mis(ptempFreqs_M, NFFT_FFT_MIS, 2);

	zero_padding_float_float_mis(NFFT_FFT_MIS, WINDOW_SIZE_MIS, x_data, new_ma);
	fft_mis_float_float_mis(NFFT_FFT_MIS, 0, 1, new_ma, pFreqs_M, ptempFreqs_M);

	for(i=0; i<(NFFT_FFT_MIS/2+1); i++)
	{
		PS_x[i] = (2.0F/WINDOW_SIZE_MIS)*sqrt(Freqs_M[i][0]*Freqs_M[i][0]+ Freqs_M[i][1]*Freqs_M[i][1]);
		x_cos_coef[i] = (2.0F/WINDOW_SIZE_MIS)*Freqs_M[i][0];
		x_sin_coef[i] = (2.0F/WINDOW_SIZE_MIS)*Freqs_M[i][1];
	}

	zero_matrix_entries_nxn_float_mis(pFreqs_M, NFFT_FFT_MIS, 2);
	zero_matrix_entries_nxn_float_mis(ptempFreqs_M, NFFT_FFT_MIS, 2);

	zero_padding_float_float_mis(NFFT_FFT_MIS, WINDOW_SIZE_MIS, y_data, new_ma);
	fft_mis_float_float_mis(NFFT_FFT_MIS, 0, 1, new_ma, pFreqs_M, ptempFreqs_M);

	for(i=0; i<(NFFT_FFT_MIS/2+1); i++)
	{
		PS_y[i] = (2.0F/WINDOW_SIZE_MIS)*sqrt(Freqs_M[i][0]*Freqs_M[i][0]+ Freqs_M[i][1]*Freqs_M[i][1]);
		y_cos_coef[i] = (2.0F/WINDOW_SIZE_MIS)*Freqs_M[i][0];
		y_sin_coef[i] = (2.0F/WINDOW_SIZE_MIS)*Freqs_M[i][1];
	}

	for(j = 0; j < 10; j++)
	{
		x_current[j] = PS_x[j];
		y_current[j] = PS_y[j];

		for(i = 0; i < 19; i++)
		{
			x_history_avg[j] += x_history_vec[i][j];
			y_history_avg[j] += y_history_vec[i][j];
		}
		x_history_avg[j] += x_current[j];
		y_history_avg[j] += y_current[j];

		x_history_avg[j] *= fft_mis_avg_factor;
		y_history_avg[j] *= fft_mis_avg_factor;
	}

	if(device_flag == 0)
	{
		if(max_range_rp >= 35.0 && acc_var >= 25.0)
			start_index = 1;
		else if(max_range_rp > 15.0 && max_range_rp < 35.0 && acc_var >= 25.0)
		{
			if(mode2_fft_mis == 1)
				start_index = 3;
			else
				start_index = 1;
		}
		else if(max_range_rp >= 45.0)
			start_index = 1;
		else if(max_range_rp > 15.0 && max_range_rp < 45.0)
		{
			if(mode2_fft_mis == 1)
				start_index = 3;
			else
				start_index = 1;
		}
		else
		{
			if(acc_var >= 6.0 && motion_unstability_flag == 0)
				start_index = 4;
			else if(acc_var < 6.0)					// slow motion
			{
				if(motion_unstability_flag == 0)

					start_index = 3;
				else
					start_index = 2;
			}
		}
	}
	else if(device_flag==1)
	{
		if(max_range_rp >= 10)
			start_index = 1;
		else
		{
			if(motion_unstability_flag == 1)
				start_index = 2;
			else
				start_index = 3;
		}
	}
	else if(device_flag==2)
	{
		if(motion_unstability_flag == 1)
			start_index = 2;
		else
			start_index = 3;
	}
	else if(device_flag == 3)
	{
		if(max_range_rp >= 45.0)
			start_index = 1;
		else if(max_range_rp >= 25.0 && max_range_rp < 45.0)
		{
			if(mode2_fft_mis == 1)
				start_index = 3;
			else
				start_index = 1;
		}
		else if(max_range_rp > 15.0 && max_range_rp < 25.0)
		{
			if(mode2_fft_mis == 1)
				start_index = 2;
			else
				start_index = 1;
		}
		else
		{
			if(fabs(data_r[DATA_SIZE_MIS-1])*R2Df > 80.0 && fabs(data_r[DATA_SIZE_MIS-1])*R2Df < 100.0 && max_range_rp > 10.0)
				start_index = 1;
			else if(fabs(data_r[DATA_SIZE_MIS-1])*R2Df > 80.0 && fabs(data_r[DATA_SIZE_MIS-1])*R2Df < 100.0 && max_range_rp < 10.0)
				start_index = 0;
			else if(motion_unstability_flag == 0)
				start_index = 3;
			else
				start_index = 2;
		}
	}

	end_index = 8;
	if(start_index > 0)
	{
		for(j = start_index+1; j < end_index; j++)
		{
			if (x_history_avg[j] > x_history_avg[j-1] && x_history_avg[j] > x_history_avg[j+1] && x_history_avg[j] > x_peak_val)
			{
				x_peak_val = x_history_avg[j];
				x_peak_index = j;

			}
			if (y_history_avg[j] > y_history_avg[j-1] && y_history_avg[j] > y_history_avg[j+1] && y_history_avg[j] > y_peak_val)
			{
				y_peak_val = y_history_avg[j];
				y_peak_index = j;
			}
		}

		if(x_peak_index>0 || y_peak_index>0)
		{
			if(x_peak_val > y_peak_val)
				peak_index = x_peak_index;
			else
				peak_index = y_peak_index;

			for(i = 0; i < WINDOW_SIZE_MIS; i++)
			{
				x_data_avg += x_data[i];
				y_data_avg += y_data[i];
			}
			x_data_avg /= WINDOW_SIZE_MIS;
			y_data_avg /= WINDOW_SIZE_MIS;

			for(i = 0; i < WINDOW_SIZE_MIS; i++)
			{
				xy_data[0][i] = x_data_avg + x_cos_coef[peak_index] * cos(2*PIf*(i*0.05F)*f_FT[peak_index]) + x_sin_coef[peak_index] * sin(2*PIf*(i*0.05F)*f_FT[peak_index]);
				xy_data[1][i] = y_data_avg + y_cos_coef[peak_index] * cos(2*PIf*(i*0.05F)*f_FT[peak_index]) + y_sin_coef[peak_index] * sin(2*PIf*(i*0.05F)*f_FT[peak_index]);
			}
		}
	}
}
#endif


int fft_mis_calculation_mis_constSize(FLOAT32 **xy_data, FLOAT32 *data_r, FLOAT32 *data_p, FLOAT32 acc_var, INT8 device_flag, UINT8 motion_unstability_flag, FLOAT32 x_history_vec[19][10], FLOAT32 *x_current, FLOAT32 y_history_vec[19][10], FLOAT32 *y_current, UINT8 mode2_fft_mis)
{
	INT8 i, j;

	FLOAT32 min_r = 99.0, max_r = -99.0;
	FLOAT32 min_p = 99.0, max_p = -99.0;
	FLOAT32 range_r,range_p, max_range_rp;

	FLOAT32 new_ma[NFFT_FFT_MIS];
	FLOAT32 PS_x[NFFT_FFT_MIS/2+1], PS_y[NFFT_FFT_MIS/2+1];
	FLOAT32 Freqs_M[NFFT_FFT_MIS][2], *pFreqs_M[NFFT_FFT_MIS];
	FLOAT32 tempFreqs_M[NFFT_FFT_MIS][2], *ptempFreqs_M[NFFT_FFT_MIS];
	FLOAT32 x_data[WINDOW_SIZE_MIS], y_data[WINDOW_SIZE_MIS];
	FLOAT32 x_data_avg =0, y_data_avg =0;

	FLOAT32 x_history_avg[10] = {0.0};
	FLOAT32 y_history_avg[10] = {0.0};

	FLOAT32 fft_mis_avg_factor = 0.05F;
	INT8 start_index = 0, end_index = 0;

	FLOAT32 x_peak_val = 0, y_peak_val = 0;
	INT8 x_peak_index = 0, y_peak_index = 0, peak_index = 0;

	FLOAT32 x_cos_coef[NFFT_FFT_MIS/2+1], x_sin_coef[NFFT_FFT_MIS/2+1], y_cos_coef[NFFT_FFT_MIS/2+1], y_sin_coef[NFFT_FFT_MIS/2+1];
	FLOAT32 f_FT[NFFT_FFT_MIS/2+1];

	for(i = 0; i < NFFT_FFT_MIS/2+1; i++)
		f_FT[i] = 10.0F*i/(NFFT_FFT_MIS/2.0F);

	for(i = 0; i < DATA_SIZE_MIS; i++)
	{
		if(data_r[i] > max_r)
			max_r = data_r[i];
		if(data_r[i] < min_r)
			min_r = data_r[i];

		if(data_p[i] > max_p)
			max_p = data_p[i];
		if(data_p[i] < min_p)
			min_p = data_p[i];
	}

	range_r = fabs(max_r - min_r);
	range_p = fabs(max_p - min_p);

	max_range_rp = MAX(range_r, range_p)*R2Df;

	for(i=0;i<WINDOW_SIZE_MIS;i++)
	{
		x_data[i] = xy_data[0][i];
		y_data[i] = xy_data[1][i];
	}

	ChangeArrayMemoryLayout_float_att_float_mis(NFFT_FFT_MIS, 2, pFreqs_M, (FLOAT32 *)Freqs_M, 1);
	ChangeArrayMemoryLayout_float_att_float_mis(NFFT_FFT_MIS, 2, ptempFreqs_M, (FLOAT32 *)tempFreqs_M, 1);

	zero_matrix_entries_nxn_float_mis(pFreqs_M, NFFT_FFT_MIS, 2);
	zero_matrix_entries_nxn_float_mis(ptempFreqs_M, NFFT_FFT_MIS, 2);

	zero_padding_float_float_mis(NFFT_FFT_MIS, WINDOW_SIZE_MIS, x_data, new_ma);
	fft_mis_float_float_mis(NFFT_FFT_MIS, 0, 1, new_ma, pFreqs_M, ptempFreqs_M);

	for(i=0; i<(NFFT_FFT_MIS/2+1); i++)
	{
		PS_x[i] = (2.0F/WINDOW_SIZE_MIS)*sqrt(Freqs_M[i][0]*Freqs_M[i][0]+ Freqs_M[i][1]*Freqs_M[i][1]);
		x_cos_coef[i] = (2.0F/WINDOW_SIZE_MIS)*Freqs_M[i][0];
		x_sin_coef[i] = (2.0F/WINDOW_SIZE_MIS)*Freqs_M[i][1];
	}

	zero_matrix_entries_nxn_float_mis(pFreqs_M, NFFT_FFT_MIS, 2);
	zero_matrix_entries_nxn_float_mis(ptempFreqs_M, NFFT_FFT_MIS, 2);

	zero_padding_float_float_mis(NFFT_FFT_MIS, WINDOW_SIZE_MIS, y_data, new_ma);
	fft_mis_float_float_mis(NFFT_FFT_MIS, 0, 1, new_ma, pFreqs_M, ptempFreqs_M);

	for(i=0; i<(NFFT_FFT_MIS/2+1); i++)
	{
		PS_y[i] = (2.0F/WINDOW_SIZE_MIS)*sqrt(Freqs_M[i][0]*Freqs_M[i][0]+ Freqs_M[i][1]*Freqs_M[i][1]);
		y_cos_coef[i] = (2.0F/WINDOW_SIZE_MIS)*Freqs_M[i][0];
		y_sin_coef[i] = (2.0F/WINDOW_SIZE_MIS)*Freqs_M[i][1];
	}

	for(j = 0; j < 10; j++)
	{
		x_current[j] = PS_x[j];
		y_current[j] = PS_y[j];

		for(i = 0; i < 19; i++)
		{
			x_history_avg[j] += x_history_vec[i][j];
			y_history_avg[j] += y_history_vec[i][j];
		}
		x_history_avg[j] += x_current[j];
		y_history_avg[j] += y_current[j];

		x_history_avg[j] *= fft_mis_avg_factor;
		y_history_avg[j] *= fft_mis_avg_factor;
	}

	if(device_flag == 0)
	{
		if(max_range_rp >= 35.0 && acc_var >= 25.0)
			start_index = 1;
		else if(max_range_rp > 15.0 && max_range_rp < 35.0 && acc_var >= 25.0)
		{
			if(mode2_fft_mis == 1)
				start_index = 3;
			else
				start_index = 1;
		}
		else if(max_range_rp >= 45.0)
			start_index = 1;
		else if(max_range_rp > 15.0 && max_range_rp < 45.0)
		{
			if(mode2_fft_mis == 1)
				start_index = 3;
			else
				start_index = 1;
		}
		else
		{
			if(acc_var >= 6.0 && motion_unstability_flag == 0)
				start_index = 4;
			else if(acc_var < 6.0)					// slow motion
			{
				if(motion_unstability_flag == 0)

					start_index = 3;
				else
					start_index = 2;
			}
		}
	}
	else if(device_flag==1)
	{
		if(max_range_rp >= 10)
			start_index = 1;
		else
		{
			if(motion_unstability_flag == 1)
				start_index = 2;
			else
				start_index = 3;
		}
	}
	else if(device_flag==2)
	{
		if(motion_unstability_flag == 1)
			start_index = 2;
		else
			start_index = 3;
	}
	else if(device_flag == 3)
	{
		if(max_range_rp >= 45.0)
			start_index = 1;
		else if(max_range_rp >= 25.0 && max_range_rp < 45.0)
		{
			if(mode2_fft_mis == 1)
				start_index = 3;
			else
				start_index = 1;
		}
		else if(max_range_rp > 15.0 && max_range_rp < 25.0)
		{
			if(mode2_fft_mis == 1)
				start_index = 2;
			else
				start_index = 1;
		}
		else
		{
			if(fabs(data_r[DATA_SIZE_MIS-1])*R2Df > 80.0 && fabs(data_r[DATA_SIZE_MIS-1])*R2Df < 100.0 && max_range_rp > 10.0)
				start_index = 1;
			else if(fabs(data_r[DATA_SIZE_MIS-1])*R2Df > 80.0 && fabs(data_r[DATA_SIZE_MIS-1])*R2Df < 100.0 && max_range_rp < 10.0)
				start_index = 0;
			else if(motion_unstability_flag == 0)
				start_index = 3;
			else
				start_index = 2;
		}
	}

	end_index = 8;
	if(start_index > 0)
	{
		for(j = start_index+1; j < end_index; j++)
		{
			if (x_history_avg[j] > x_history_avg[j-1] && x_history_avg[j] > x_history_avg[j+1] && x_history_avg[j] > x_peak_val)
			{
				x_peak_val = x_history_avg[j];
				x_peak_index = j;

			}
			if (y_history_avg[j] > y_history_avg[j-1] && y_history_avg[j] > y_history_avg[j+1] && y_history_avg[j] > y_peak_val)
			{
				y_peak_val = y_history_avg[j];
				y_peak_index = j;
			}
		}

		if(x_peak_index>0 || y_peak_index>0)
		{
			if(x_peak_val > y_peak_val)
				peak_index = x_peak_index;
			else
				peak_index = y_peak_index;

			for(i = 0; i < WINDOW_SIZE_MIS; i++)
			{
				x_data_avg += x_data[i];
				y_data_avg += y_data[i];
			}
			x_data_avg /= WINDOW_SIZE_MIS;
			y_data_avg /= WINDOW_SIZE_MIS;

			for(i = 0; i < WINDOW_SIZE_MIS; i++)
			{
				xy_data[0][i] = x_data_avg + x_cos_coef[peak_index] * cos(2*PIf*(i*0.05F)*f_FT[peak_index]) + x_sin_coef[peak_index] * sin(2*PIf*(i*0.05F)*f_FT[peak_index]);
				xy_data[1][i] = y_data_avg + y_cos_coef[peak_index] * cos(2*PIf*(i*0.05F)*f_FT[peak_index]) + y_sin_coef[peak_index] * sin(2*PIf*(i*0.05F)*f_FT[peak_index]);
			}
		}
	}

	return 1;
}


/**
* @brief This function calculates phase one misalignment angle.
*
* @details This function calculates phase one misalignment angle.
*
* @param[in] data pointer to a vector of leveled accelerometer data
* @param[in] mean_x variable to the average of leveled accelerometer x data
* @param[in] mean_y variable to the average of leveled accelerometer y data
*
* @return returns the value of phase 1 misalignment angle as FLOAT32.
*/
FLOAT32 calculate_phase_one_angle_mis(FLOAT32 **data, FLOAT32 mean_x, FLOAT32 mean_y)
{
	INT16 i, j;
	FLOAT32 point_line_distance[4] = {0}, P_L_dist[4] = {0};
	FLOAT32 sum_of_distance[4] = {0};
	FLOAT32 st_range = 0, end_range = 0;
	FLOAT32 phase_one_angle = 0.0;
	FLOAT32 tan_phase_one_angle = 0.0, one_by_tan_angle = 0;

	for(i = 0; i < WINDOW_SIZE_MIS; i++)
	{
		data[i][0] -= mean_x;
		data[i][1] -= mean_y;
		point_line_distance[0] +=  data[i][1]*data[i][1];
		point_line_distance[1] +=  (data[i][0] - data[i][1])*(data[i][0] - data[i][1])*0.5f;
		point_line_distance[2] +=  data[i][0]*data[i][0];
		point_line_distance[3] +=  (-data[i][0] - data[i][1])*(-data[i][0] - data[i][1])*0.5f;
	}

	sum_of_distance[0] = point_line_distance[0] + point_line_distance[1];
	sum_of_distance[1] = point_line_distance[1] + point_line_distance[2];
	sum_of_distance[2] = point_line_distance[2] + point_line_distance[3];
	sum_of_distance[3] = point_line_distance[3] + point_line_distance[0];

	if(sum_of_distance[0] < sum_of_distance[1] && sum_of_distance[0] < sum_of_distance[2] && sum_of_distance[0] < sum_of_distance[3])
	{
		st_range = 0.0f;
		end_range = 45.0f*D2Rf;
		P_L_dist[0] = point_line_distance[0];
		P_L_dist[1] = point_line_distance[1];
	}
	else if(sum_of_distance[1] < sum_of_distance[0] && sum_of_distance[1] < sum_of_distance[2] && sum_of_distance[1] < sum_of_distance[3])
	{
		st_range = 45.0f*D2Rf;
		end_range = 90.0f*D2Rf;
		P_L_dist[0] = point_line_distance[1];
		P_L_dist[1] = point_line_distance[2];
	}
	else if(sum_of_distance[2] < sum_of_distance[0] && sum_of_distance[2] < sum_of_distance[1] && sum_of_distance[2] < sum_of_distance[3])
	{
		st_range = 90.0f*D2Rf;
		end_range = 135.0f*D2Rf;
		P_L_dist[0] = point_line_distance[2];
		P_L_dist[1] = point_line_distance[3];
	}
	else
	{
		st_range = 135.0f*D2Rf;
		end_range = 180.0f*D2Rf;
		P_L_dist[0] = point_line_distance[3];
		P_L_dist[1] = point_line_distance[0];
	}

	for(i = 0; i < 6; i++)
	{
		P_L_dist[2] = 0.0f;
		phase_one_angle = (st_range + end_range)*0.5f;
		tan_phase_one_angle = tan(phase_one_angle);
		one_by_tan_angle = 1.0f/(tan_phase_one_angle*tan_phase_one_angle + 1);
		for(j = 0; j < WINDOW_SIZE_MIS; j++)
			P_L_dist[2] += ((tan_phase_one_angle*data[j][0] - data[j][1])*(tan_phase_one_angle*data[j][0] - data[j][1]))*one_by_tan_angle;

		if(P_L_dist[0] < P_L_dist[1])
		{
			end_range = phase_one_angle;
			P_L_dist[1] = P_L_dist[2];
		}
		else
		{
			st_range = phase_one_angle;
			P_L_dist[0] = P_L_dist[2];
		}
	}
	return -phase_one_angle;
}
/**
* @brief This function calculates phase one misalignment angle analytically.
*
* @details This function calculates phase one misalignment angle.
*
* @param[in] data pointer to a vector of leveled accelerometer data
* @param[in] mean_x variable to the average of leveled accelerometer x data
* @param[in] mean_y variable to the average of leveled accelerometer y data
* @param[in] angle_slope_flag flag to return angle or slope
*
* @return returns the value of phase 1 misalignment angle or the slope as FLOAT32.
*/
FLOAT32 calculate_phase_one_angle_mis_analytical(FLOAT32 XX_sum, FLOAT32 XY_sum, FLOAT32 YY_sum, UINT8 angle_slope_flag)
{
	FLOAT32 angle_slope = 0.0f;
	if(XY_sum != 0.0f)
		angle_slope = (-(XX_sum - YY_sum) + sqrt(SQR(XX_sum - YY_sum)+(4.0f*XY_sum*XY_sum)))/(2.0f*XY_sum);
	if(angle_slope_flag == 1)
		return (-(FLOAT32)(atan(angle_slope)));
	else if(angle_slope_flag == 2)
		return angle_slope;
	else
		return 0;
}
/**
* @brief This function provides the required parameters for the sign methods.
*
* @details The function takes 2 pointers to structures of type PCA_M and PCA. it takes 2 pointers to
* 		   vectors of 2 seconds history of of data for vertical_motion_phase data and
* 		   motion_effective_coefficient data. The function sets the fileds of a pointer to structure
* 		   provided as outputs (pca_m)
*
* @param[in] pca pointer to a structure of type \c PCA
* @param[in] motion_effective_coefficient_vec pointer to vector of the last 2 seconds of variable \c motion_effective_coefficient
* @param[in] vertical_motion_phase_vec pointer to vector of the last 2 seconds of variable \c vertical_motion_phase
* @param[out] pca_m pointer to a structure of type \c PCA_M
*
* @see for mor see Phase_Shift_Angle_mis_float_float_mis() and Calculate_PCA_float_float_mis()
*/
#ifndef COM_WALK_SIMPLIFIED
void sign_method_parameters_calculation_mis(PCA_M_Ptr pca_m, PCA_Ptr pca, FLOAT32 *motion_effective_coefficient_vec, FLOAT32 *vertical_motion_phase_vec)
#else
void sign_method_parameters_calculation_mis(PCA_M_Ptr pca_m, PCA_Ptr pca)
#endif
{
	INT8 i;    // loop counter
	FLOAT32 vertical_motion_angle_1 = 0.0, vertical_motion_angle_2 = 0.0;
	FLOAT32 data_m_lag_lead[DATA_SIZE_MIS-TIME_SHIFT_MIS] = {0.0};
	FLOAT32 data_v_lead_lag[DATA_SIZE_MIS-TIME_SHIFT_MIS] = {0.0};
#ifndef COM_WALK_SIMPLIFIED
	FLOAT32 mean_m = 0.0f, mean_v = 0.0f;
	FLOAT32 mm_sum = 0.0f, vv_sum = 0.0f, mv_sum = 0.0f;
	FLOAT32 m_v_slope = 0.0f;
	pca_m->motion_effective_coefficient_mean = 0.0;
	pca_m->vertical_motion_phase_mean = 0.0;
#endif
	// Lag Motion Lead Vertical phase shift angle
	for(i = 0; i < DATA_SIZE_MIS-TIME_SHIFT_MIS; i++)
	{
		data_m_lag_lead[i] = pca->data_m[i+TIME_SHIFT_MIS];
		data_v_lead_lag[i] = pca->data_v[i];
	}
	vertical_motion_angle_1 = Phase_Shift_Angle_mis_float_float_mis(data_m_lag_lead,  data_v_lead_lag, DATA_SIZE_MIS-TIME_SHIFT_MIS);

	// Lead Motion Lag Vertical phase shift angle
	for(i = 0; i < DATA_SIZE_MIS-TIME_SHIFT_MIS; i++)
	{
		data_m_lag_lead[i] = pca->data_m[i];
		data_v_lead_lag[i] = pca->data_v[i+TIME_SHIFT_MIS];
	}
	vertical_motion_angle_2 = Phase_Shift_Angle_mis_float_float_mis(data_m_lag_lead, data_v_lead_lag,  DATA_SIZE_MIS-TIME_SHIFT_MIS);

	// The Coprrelated Angle between Vertical and Motion
	pca_m->vertical_motion_angle = (vertical_motion_angle_2 - vertical_motion_angle_1)*R2Df;

#ifndef COM_WALK_SIMPLIFIED
	// Angle between Motion and Vertical
	pca_m->vertical_motion_phase = Phase_Shift_Angle_mis_float_float_mis(pca->data_m, pca->data_v, DATA_SIZE_MIS)*R2Df;

	for(i = 0; i < DATA_SIZE_MIS; i++)
	{
		mean_m += pca->data_m[i];
		mean_v += pca->data_v[i];
	}
	mean_m *= ONE_BY_DATA_SIZE_MIS_MEAN;
	mean_v *= ONE_BY_DATA_SIZE_MIS_MEAN;

	for(i = 0; i < DATA_SIZE_MIS; i++)
	{
		mm_sum +=  SQR(pca->data_m[i] - mean_m);
		mv_sum +=  (pca->data_m[i] - mean_m)*(pca->data_v[i] - mean_v);
		vv_sum +=  SQR(pca->data_v[i] - mean_v);
	}

	m_v_slope = calculate_phase_one_angle_mis_analytical(mm_sum, mv_sum, vv_sum, 2);

	if (fabs(m_v_slope)<1)
	{
		if (m_v_slope>0.0)
			pca_m->motion_effective_coefficient = sqrt((m_v_slope*m_v_slope)/(m_v_slope*m_v_slope+1));
		else
			pca_m->motion_effective_coefficient = -sqrt((m_v_slope*m_v_slope)/(m_v_slope*m_v_slope+1));
	}
	else
	{
		if (m_v_slope>0.0)
			pca_m->motion_effective_coefficient = sqrt(1/(m_v_slope*m_v_slope+1));
		else
			pca_m->motion_effective_coefficient = -sqrt(1/(m_v_slope*m_v_slope+1));
	}

	// Average Value of the parameters
	for(i = 1; i < DATA_SIZE_MIS; i++)
	{
		pca_m->motion_effective_coefficient_mean += motion_effective_coefficient_vec[i];
		pca_m->vertical_motion_phase_mean += vertical_motion_phase_vec[i];
	}
	pca_m->motion_effective_coefficient_mean = (pca_m->motion_effective_coefficient_mean + pca_m->motion_effective_coefficient)*ONE_BY_DATA_SIZE_MIS_MEAN;
	pca_m->vertical_motion_phase_mean = (pca_m->vertical_motion_phase_mean + pca_m->vertical_motion_phase)*ONE_BY_DATA_SIZE_MIS_MEAN;
#endif
}

/**
* @brief	This function classifies the different device use cases
*
* @details The function takes a pointer to structure of type PCA, roll and pitch data, data of other parameters such as
* 		   acceleration variance, radius of rotation, and gyroscope norm. Also, it takes a history for the step frequency.
* 		   The function provides the device use case with the step frequency. Also, it sets the fileds of a pointer to
* 		   structure provided as outputs (pca).
*
* @param[in]	data_r		  				pointer to roll data vector.
* @param[in]	data_p		  				pointer to pitch data vector.
* @param[in]	acc_var						variable to acceleration variance value.
* @param[in]	rad_rot_vec   				pointer to radius of rotation vector.
* @param[in]	gyr_nor_vec   				pointer to gyroscope norm vector.
* @param[in]	motion_unstability_flag   	variable to motion stability status.
* @param[out]	mode1		  				pointer to mode1 variable.
* @param[out]	mode2		  				pointer to mode2 variable.
* @param[in,out]	pca						pointer to a structure of type \c PCA
*
*@see classification_by_peaks_mis() and pocket_pattern_detection_mis()
*/
#ifdef PRINT_RESULTS_MISALIGNMENT
void device_use_cases_classification_mis(FLOAT32 *data_r, FLOAT32 *data_p, PCA_Ptr pca, FLOAT32 acc_var, FLOAT32 *rad_rot_vec, FLOAT32 *gyr_nor_vec, UINT8 motion_unstability_flag, INT8 device_flag, UINT8 *mode1, UINT8 *mode2, FLOAT32 *Par_Values)
#else
void device_use_cases_classification_mis(FLOAT32 *data_r, FLOAT32 *data_p, PCA_Ptr pca, FLOAT32 acc_var, FLOAT32 *rad_rot_vec, FLOAT32 *gyr_nor_vec, UINT8 motion_unstability_flag, INT8 device_flag, UINT8 *mode1, UINT8 *mode2)
#endif
{
	INT8 i;
	INT8 count1 = 0, count2 = 0,/* peaks_count = 0,*/ R_counter = 0, R_thr = 0, pocket_counter = 0;
	UINT8 pocket_flag = 0, R_flag = 0, R_slow_flag = 0, H_flag = 0, dangling_flag = 0;
	//INT8 v_peaks_index[DATA_SIZE_MIS] = {100};

	FLOAT32 /*acc_var = 0.0,*/ mean_gyro = 0.0, std_gyro = 0.0;
	FLOAT32 max_std_rp = 0.0, min_std_rp = 0.0, diff_std_rp = 0.0;
	FLOAT32 max_range_rp = 0.0, min_range_rp = 0.0, diff_range_rp = 0.0;
	FLOAT32 dif_sum_r = 0.0, dif_sum_p = 0.0;
	//FLOAT32 step_freq = 0.0, step_freq_range = 0.0, step_ranges_thr = 0.0;

	//FLOAT32 min_step_freq = 99.0, max_step_freq = -99.0;
	FLOAT32 min_r = 99.0, max_r = -99.0;
	FLOAT32 min_p = 99.0, max_p = -99.0;
	FLOAT32 min_m = 99.0, max_m = -99.0;
	//FLOAT32 min_v = 99.0, max_v = -99.0;
	FLOAT32 min_acc = 99.0, max_acc = -99.0;

#ifdef PRINT_RESULTS_MISALIGNMENT
	INT8 k;
	INT8 st_ind = -1, end_ind = -1;
	FLOAT32 range_gyro = 0.0, max_gyro = -99.0, min_gyro = 99.0;
	FLOAT32 mean_l = 0.0, std_l = 0.0, lateral_range = 0.0, max_l = -99.0, min_l = 99.0;
	FLOAT32 mean_h = 0.0, std_h = 0.0, horizontal_acceleration_range = 0.0, max_h = -99.0, min_h = 99.0;
	FLOAT32 dangling_counter = 0.0, dangling2_flag = 0.0, R_mean = 0.0;
	FLOAT32 change_a = 0, change_v = 0;
	FLOAT32 Old_New_Flag = 0.0;
#endif

	PCA_C pca_c;
	memset(&pca_c, 0, sizeof(PCA_C));

	pca->roll_parametrs[0]     = pca->roll_parametrs[1]     = 0.0;
	pca->pitch_parametrs[0]    = pca->pitch_parametrs[1]    = 0.0;
	pca->motion_parametrs[0]   = pca->motion_parametrs[1]   = 0.0;
	//pca->vertical_parametrs[0] = pca->vertical_parametrs[1] = 0.0;
	pca->acceleration_norm_parametrs[0] = pca->acceleration_norm_parametrs[1] = 0.0;

	*mode1 = 0;
	*mode2 = 0;

	for(i = 0; i < DATA_SIZE_MIS; i++)
	{
		pca->roll_parametrs[0]     += data_r[i];
		pca->pitch_parametrs[0]    += data_p[i];
		pca->motion_parametrs[0]   += pca->data_m[i];
		//pca->vertical_parametrs[0] += pca->data_v[i];
		pca->acceleration_norm_parametrs[0] += pca->Acc_Mag[i];

		if(data_r[i] > max_r)
			max_r = data_r[i];
		if(data_r[i] < min_r)
			min_r = data_r[i];

		if(data_p[i] > max_p)
			max_p = data_p[i];
		if(data_p[i] < min_p)
			min_p = data_p[i];

		if(pca->data_m[i] > max_m)
			max_m = pca->data_m[i];
		if(pca->data_m[i] < min_m)
			min_m = pca->data_m[i];

		/*if(pca->data_v[i] > max_v)
		max_v = pca->data_v[i];
		if(pca->data_v[i] < min_v)
		min_v = pca->data_v[i];*/

		if(pca->Acc_Mag[i] > max_acc)
			max_acc = pca->Acc_Mag[i];
		if(pca->Acc_Mag[i] < min_acc)
			min_acc = pca->Acc_Mag[i];
	}

	pca->roll_parametrs[0]     *= ONE_BY_DATA_SIZE_MIS_MEAN;
	pca->pitch_parametrs[0]    *= ONE_BY_DATA_SIZE_MIS_MEAN;
	pca->motion_parametrs[0]   *= ONE_BY_DATA_SIZE_MIS_MEAN;
	//pca->vertical_parametrs[0] *= ONE_BY_DATA_SIZE_MIS_MEAN;
	pca->acceleration_norm_parametrs[0] *= ONE_BY_DATA_SIZE_MIS_MEAN;

	pca->roll_parametrs[2]     = fabs(max_r   - min_r)*R2Df;
	pca->pitch_parametrs[2]    = fabs(max_p   - min_p)*R2Df;
	//pca->vertical_parametrs[2] = fabs(max_v   - min_v);
	pca->motion_parametrs[2]   = fabs(max_m   - min_m);
	pca->acceleration_norm_parametrs[2] = fabs(max_acc - min_acc);

	// get the std
	for(i = 0; i < DATA_SIZE_MIS; i++)
	{
		pca->roll_parametrs[1]     += SQR(data_r[i]       - pca->roll_parametrs[0]);
		pca->pitch_parametrs[1]    += SQR(data_p[i]       - pca->pitch_parametrs[0]);
		pca->motion_parametrs[1]   += SQR(pca->data_m[i]  - pca->motion_parametrs[0]);
		//pca->vertical_parametrs[1] += SQR(pca->data_v[i]  - pca->vertical_parametrs[0]);
		pca->acceleration_norm_parametrs[1] += SQR(pca->Acc_Mag[i] - pca->acceleration_norm_parametrs[0]);
	}
	pca->roll_parametrs[1]     = sqrt(pca->roll_parametrs[1]     * ONE_BY_DATA_SIZE_MIS_STD);
	pca->pitch_parametrs[1]    = sqrt(pca->pitch_parametrs[1]    * ONE_BY_DATA_SIZE_MIS_STD);
	pca->motion_parametrs[1]   = sqrt(pca->motion_parametrs[1]   * ONE_BY_DATA_SIZE_MIS_STD);
	//pca->vertical_parametrs[1] = sqrt(pca->vertical_parametrs[1] * ONE_BY_DATA_SIZE_MIS_STD);
	pca->acceleration_norm_parametrs[1] = sqrt(pca->acceleration_norm_parametrs[1] * ONE_BY_DATA_SIZE_MIS_STD);

	max_range_rp  = MAX(pca->roll_parametrs[2], pca->pitch_parametrs[2]);
	min_range_rp  = MIN(pca->roll_parametrs[2], pca->pitch_parametrs[2]);
	diff_range_rp = fabs(max_range_rp - min_range_rp);

	max_std_rp  = MAX(pca->roll_parametrs[1], pca->pitch_parametrs[1])*R2Df;
	min_std_rp  = MIN(pca->roll_parametrs[1], pca->pitch_parametrs[1])*R2Df;
	diff_std_rp = fabs(max_std_rp - min_std_rp);

	// Motion Peaks Detection Module
	i = peaks_detection_mis(pca->data_m, pca->motion_parametrs[2]*0.125F, 4, &pca_c.motion_peaks, 20, 36, 0);
	if((pca_c.motion_peaks.num[0] < 2 || pca_c.motion_peaks.num[1] < 2) && pca->motion_parametrs[2]*0.125F > pca->motion_parametrs[1])
		i = peaks_detection_mis(pca->data_m, pca->motion_parametrs[1], 4, &pca_c.motion_peaks, 20, 36, 0);
	if(pca_c.motion_peaks.num[0] == 0 || pca_c.motion_peaks.num[1] == 0)
		i = peaks_detection_mis(pca->data_m, pca->motion_parametrs[2]*0.1F, 4, &pca_c.motion_peaks, 20, 36, 0);
	if((pca_c.motion_peaks.num[0] < 2 || pca_c.motion_peaks.num[1] < 2) && pca->motion_parametrs[2]*0.1F > pca->motion_parametrs[1])
		i = peaks_detection_mis(pca->data_m, pca->motion_parametrs[1], 4, &pca_c.motion_peaks, 20, 36, 0);

	// Vertical Peaks Detection
	i = peaks_detection_mis(pca->data_v, pca->vertical_parametrs[2]*0.125F, 4, &pca_c.vertical_peaks, 20, 36, 0);
	if((pca_c.vertical_peaks.num[0] < 2 || pca_c.vertical_peaks.num[1] < 2) && pca->vertical_parametrs[2]*0.125F > pca->vertical_parametrs[1])
		i = peaks_detection_mis(pca->data_v, pca->vertical_parametrs[1], 4, &pca_c.vertical_peaks, 20, 36, 0);

	// get the mean of Gyro magnetiude and acceleration variance
	mean_gyro = 0;
	//acc_var = 0;
	for(i = 0; i < DATA_SIZE_MIS; i++)
	{
		mean_gyro += gyr_nor_vec[i];
		//acc_var   += acc_var_vec[i];
	}
	mean_gyro *= ONE_BY_DATA_SIZE_MIS_MEAN;
	//acc_var   *= ONE_BY_DATA_SIZE_MIS_MEAN;
	/*if(acc_var == 0)
	acc_var = std_v*std_v;*/

	// get the std of Gyro magnetiude
	std_gyro = 0.0;
	for(i = 0; i < DATA_SIZE_MIS; i++)
		std_gyro += SQR(gyr_nor_vec[i] - mean_gyro);

	mean_gyro *= R2Df;
	std_gyro   = sqrt(std_gyro * ONE_BY_DATA_SIZE_MIS_STD)*R2Df;

	//The change in motion signal.
	pca_c.change_m = 0;
	for(i = 1; i < DATA_SIZE_MIS-1; i++)
	{
		if((pca->data_m[i-1] - pca->data_m[i])*(pca->data_m[i] - pca->data_m[i+1]) < 0.0)			// is a slope change
			pca_c.change_m++;
	}

	/////////////////// Not Dangling Flag
	if(pca_c.vertical_peaks.num[1] > 0 && pca_c.vertical_peaks.num[0] > 0 && pca->vertical_parametrs[2] > 5.6)
	{
		if(pca_c.vertical_peaks.num[1] > 2)
		{
			for(i = 0; i < pca_c.vertical_peaks.num[1]-2; i++)
			{
				count1 = pca_c.vertical_peaks.high_ind[i+2]- pca_c.vertical_peaks.high_ind[i+1];
				count2 = pca_c.vertical_peaks.high_ind[i+1]- pca_c.vertical_peaks.high_ind[i];
				if(abs(count1-count2) > 7)		//peaks series not good, can not be dangling
					pca_c.not_dangling_flag = 1;
			}
		}
		if(pca_c.vertical_peaks.num[0] > 2)
		{
			for(i = 0; i < pca_c.vertical_peaks.num[0]-2; i++)
			{
				count1 = pca_c.vertical_peaks.low_ind[i+2]- pca_c.vertical_peaks.low_ind[i+1];
				count2 = pca_c.vertical_peaks.low_ind[i+1]- pca_c.vertical_peaks.low_ind[i];
				if(abs(count1-count2) > 7)	// Peaks series not good, can not be dangling
					pca_c.not_dangling_flag = 1;
			}
		}

		if(MIN(pca_c.vertical_peaks.high_ind[0], pca_c.vertical_peaks.low_ind[0]) > 9 || MAX(pca_c.vertical_peaks.high_ind[pca_c.vertical_peaks.num[1]-1], pca_c.vertical_peaks.low_ind[pca_c.vertical_peaks.num[0]-1]) < 25)
			pca_c.not_dangling_flag = 1;
	}

	if(pca->roll_parametrs[2] > 340.0)	// Roll has flipping issue (180 to -180), then give message to use pitch. Added, 2012.06.25
	{
		pca_c.att_vec[0] = pca->pitch_parametrs[0];
		pca_c.att_vec[1] = pca->pitch_parametrs[1];
		pca_c.att_vec[2] = pca->pitch_parametrs[2];
		pca_c.b = 1;
		for(i = 0; i < DATA_SIZE_MIS; i++)
			pca_c.atti_vec[i] = data_p[i]-pca->pitch_parametrs[0];
	}
	else
	{
		if(fabs(pca->pitch_parametrs[2]-pca->roll_parametrs[2]) < 1.5)  // roll and pitch move like each other, 2012.06.29
		{
			if(pca->roll_parametrs[1] > pca->pitch_parametrs[1])
			{
				pca_c.att_vec[0] = pca->roll_parametrs[0];
				pca_c.att_vec[1] = pca->roll_parametrs[1];
				pca_c.att_vec[2] = pca->roll_parametrs[2];
				pca_c.b = 0;
				for(i = 0; i < DATA_SIZE_MIS; i++)
					pca_c.atti_vec[i] = data_r[i]-pca->roll_parametrs[0];
			}
			else
			{
				pca_c.att_vec[0] = pca->pitch_parametrs[0];
				pca_c.att_vec[1] = pca->pitch_parametrs[1];
				pca_c.att_vec[2] = pca->pitch_parametrs[2];
				pca_c.b = 1;
				for(i = 0; i < DATA_SIZE_MIS; i++)
					pca_c.atti_vec[i] = data_p[i]-pca->pitch_parametrs[0];
			}
		}
		else
		{
			if(pca->roll_parametrs[2] > pca->pitch_parametrs[2])			//roll has bigger motion
			{
				pca_c.att_vec[0] = pca->roll_parametrs[0];
				pca_c.att_vec[1] = pca->roll_parametrs[1];
				pca_c.att_vec[2] = pca->roll_parametrs[2];
				pca_c.b = 0;
				for(i = 0; i < DATA_SIZE_MIS; i++)
					pca_c.atti_vec[i] = data_r[i]-pca->roll_parametrs[0];
			}
			else										//pitch has bigger motion
			{
				pca_c.att_vec[0] = pca->pitch_parametrs[0];
				pca_c.att_vec[1] = pca->pitch_parametrs[1];
				pca_c.att_vec[2] = pca->pitch_parametrs[2];
				pca_c.b = 1;
				for(i = 0; i < DATA_SIZE_MIS; i++)
					pca_c.atti_vec[i] = data_p[i]-pca->pitch_parametrs[0];
			}
		}
	}
	//Added on July 3.
	dif_sum_r = 0.0;
	dif_sum_p = 0.0;
	for(i = 0; i < DATA_SIZE_MIS-1; i++)
	{
		dif_sum_r += fabs(data_r[i] - data_r[i+1]);
		dif_sum_p += fabs(data_p[i] - data_p[i+1]);
	}

	/////////////////// Attitude Peaks Detection
	if(pca->vertical_parametrs[2] > 10.0 && MAX(dif_sum_r, dif_sum_p) > 2.7 && pca->motion_parametrs[2] < 10.0)
		i = peaks_detection_mis(pca_c.atti_vec, 1.5F*D2Rf, 4, &pca_c.attitude_peaks, 20, 36, 0);
	else
		i = peaks_detection_mis(pca_c.atti_vec, (pca_c.att_vec[2]*0.25F)*D2Rf, 4, &pca_c.attitude_peaks, 20, 36, 0);

	pca_c.atth_thr1 = pca_c.att_vec[0] + pca_c.att_vec[1];
	pca_c.attl_thr1 = pca_c.att_vec[0] - pca_c.att_vec[1];
	pca_c.vh_thr1 = pca->vertical_parametrs[0] + 1.7F*pca->vertical_parametrs[1];
	pca_c.vh_thr2 = pca->vertical_parametrs[0] + 0.7F*pca->vertical_parametrs[1];
	pca_c.mh_thr1 = pca->motion_parametrs[0] + 0.8F*pca->motion_parametrs[1];

	//Crash protection, ZUPT
	if(!(pca->vertical_parametrs[2] < 0.8 || (pca_c.motion_peaks.num[0] + pca_c.motion_peaks.num[1]) > 20))
		//*Step_Frequency = 0;
		//else
	{
		/////////////////////// Start to do classification  ///////////////////////

		////////////////////////////	Classification 1	///////////////////////////
		if(max_range_rp > 70.0 && ((pca->vertical_parametrs[2] > 30.0 && pca->motion_parametrs[2] > 30.0) || (pca->motion_parametrs[2] > 20.0 && pca->vertical_parametrs[2] > 20.0 && pca->vertical_parametrs[2] < pca->motion_parametrs[2])) && (pca_c.attitude_peaks.num[1] + pca_c.attitude_peaks.num[0]) < 6 && (pca_c.motion_peaks.num[1] > 0 || pca_c.motion_peaks.num[0] > 0))
			*mode1 = 2;
		else if(pca_c.att_vec[2] > 10.0 && pca_c.att_vec[1] >= 0.05 && (pca_c.attitude_peaks.num[0] + pca_c.attitude_peaks.num[1]) < 6 && (pca_c.motion_peaks.num[1] > 0 || pca_c.motion_peaks.num[0] > 0))
			*mode1 = classification_by_peaks_mis(&pca_c, pca);
		//////////////////////////	End of Classification 1	///////////////////////////

		/////////////////////////////// Classification 2  ////////////////////////////////
		/////////////////// Dangling Flag
		if((pca->motion_parametrs[2] - pca->vertical_parametrs[2]) > 4 && (pca->motion_parametrs[1] - pca->vertical_parametrs[1]) > 3)
			dangling_flag= 1;
		else
			dangling_flag= 0;

		/////////////////// pocket flag

		pocket_counter  = pocket_pattern_detection_mis(&pca_c, pca, 1);
		pocket_counter += pocket_pattern_detection_mis(&pca_c, pca, 0);

		for(i=0; i<pca_c.vertical_peaks.num[1]-1; i++)
		{
			if((pca_c.vertical_peaks.high_val[i] - pca->vertical_parametrs[0]) > (pca_c.mh_thr1 - pca->motion_parametrs[0]) && (pca_c.vertical_peaks.high_val[i+1] - pca->vertical_parametrs[0]) > (pca_c.mh_thr1 - pca->motion_parametrs[0]) && pca_c.vertical_peaks.high_val[i] > pca_c.vertical_peaks.high_val[i+1] && abs(pca_c.vertical_peaks.high_ind[i+1] - pca_c.vertical_peaks.high_ind[i]) <= 9)
				pocket_counter++;
		}
		if(pocket_counter > 0)
			pocket_flag = 1;
		else
			pocket_flag = 0;

		pocket_flag *=(!dangling_flag);

		/////////////////// Radius of rotation threshold
		if(acc_var <= 10)
			R_thr = 10;
		else if(acc_var <= 20)
			R_thr = 11;
		else if(acc_var <= 30)
			R_thr = 12;
		else if(acc_var > 30)
			R_thr = 13;

		/////////////////// Radius of rotation flag
		for(i = 0; i < DATA_SIZE_MIS; i++)
		{
			if(rad_rot_vec[i] < 1.5)
				R_counter++;
		}
		if(R_counter < R_thr)
			R_flag = 1;
		else
			R_flag = 0;
		/////////////////// Radius of rotation slow flag
		if(R_counter <= 13 && max_range_rp <= 15 && max_std_rp <= 5 && std_gyro <= 30 && acc_var <= 5)
			R_slow_flag = 1;
		else
			R_slow_flag = 0;
		/////////////////// Handheld flag
		if(acc_var <= 5)
			H_flag = R_slow_flag;
		else
			H_flag = R_flag;
		/////////////////// to protect belt from going to pocket
		if((R_counter > 10 && max_range_rp <= 10 && max_std_rp <=  4) ||
			(R_counter > 15 && max_range_rp <= 14 && max_std_rp <=  5) ||
			(R_counter > 20 && max_range_rp <= 20 && max_std_rp <= 10))
			H_flag = 1;

		//if(floorf(step_freq_range * 10000) / 10000 > floorf(step_ranges_thr * 10000) / 10000)
		if(motion_unstability_flag == 1)
		{
			if(H_flag == 1)
				*mode2 = 0;
			else if(H_flag == 0)
			{
				if(pocket_flag == 1)
					*mode2 = 1;
				else if(pocket_flag == 0)
					*mode2 = 2;
			}
		}
		else
		{
			if(max_std_rp < 6 && std_gyro < 35 && (mean_gyro < 30 || (mean_gyro < 80 && max_range_rp < 20)))
				*mode2 = 0;
			else if((max_std_rp > 17 && max_range_rp > 55 && std_gyro > 35 && mean_gyro > 90) || (pca->motion_parametrs[2] > 10 && (pca_c.motion_peaks.num[1] + pca_c.motion_peaks.num[0]) < 5 && pca->motion_parametrs[2] > 1.2*pca->vertical_parametrs[2] && max_std_rp > 10 && max_range_rp > 40 && std_gyro > 25 && mean_gyro > 50))
				*mode2 = 2;
			else if(max_std_rp > 8 && max_std_rp < 17 && std_gyro > 35 && mean_gyro > 90 && max_range_rp > 20 && max_range_rp < 55)
				*mode2 = 1;
			else
				*mode2 = *mode1;

			if(*mode2 == 0)
			{
				if(max_range_rp > 20.0 && max_range_rp <= 45.0 && max_std_rp > 8.0 && max_std_rp < 18.0)
					*mode2 = 1;
				else if(max_range_rp > 35.0 && max_std_rp > 15.0 && pca->motion_parametrs[2] > 10.0)
					*mode2 = 2;
			}
		}

		// Modification to avoid using FFT to be used in watch in pocket datasets
		if(device_flag == 3 && *mode2 == 0)
		{

			if((max_range_rp > 15.0 && max_std_rp > 5.0 && pca->vertical_parametrs[2] > pca->motion_parametrs[2]
			&& mean_gyro > 35.0) || (pocket_flag == 1 && H_flag == 0 && mean_gyro > 30.0))
				*mode2 = 1;
		}

		/////////////////////////////// End of Classification 2  ////////////////////////////////


		/////////////////////////////////////// Print Results  /////////////////////////////

#ifdef PRINT_RESULTS_MISALIGNMENT
		for(i = 0; i < DATA_SIZE_MIS; i++)
		{
			mean_l += pca->data_l[i];
			mean_h += pca->data_h[i];

			if(pca->data_l[i] > max_l)
				max_l = pca->data_l[i];
			if(pca->data_l[i] < min_l)
				min_l = pca->data_l[i];

			if(pca->data_h[i] > max_h)
				max_h = pca->data_h[i];
			if(pca->data_h[i] < min_h)
				min_h = pca->data_h[i];

			if(gyr_nor_vec[i] > max_gyro)
				max_gyro = gyr_nor_vec[i];
			if(gyr_nor_vec[i] < min_gyro)
				min_gyro = gyr_nor_vec[i];
		}

		mean_l *= ONE_BY_DATA_SIZE_MIS_MEAN;
		mean_h *= ONE_BY_DATA_SIZE_MIS_MEAN;

		lateral_range = fabs(max_l-min_l);
		horizontal_acceleration_range = fabs(max_h-min_h);
		range_gyro = fabs(max_gyro - min_gyro)*R2Df;

		// get the std
		for(i = 0; i < DATA_SIZE_MIS; i++)
		{
			std_l += SQR(pca->data_l[i]-mean_l);
			std_h += SQR(pca->data_h[i]-mean_h);
		}
		std_l = sqrt(std_l * ONE_BY_DATA_SIZE_MIS_STD);
		std_h = sqrt(std_h * ONE_BY_DATA_SIZE_MIS_STD);
		for(i = 0; i < DATA_SIZE_MIS; i++)

			//change happens in vertical signal.
			change_v = 0;
		for(i = 1; i < DATA_SIZE_MIS-1; i++)
		{
			if((pca->data_v[i-1]-pca->data_v[i])*(pca->data_v[i]-pca->data_v[i+1]) < 0.0)
				change_v++;
		}

		// Change in attitude signal slope, 2012.06.27
		change_a = 0;
		for (i = 1; i < DATA_SIZE_MIS-1; i++)
		{
			if((pca_c.atti_vec[i-1]-pca_c.atti_vec[i])*(pca_c.atti_vec[i]-pca_c.atti_vec[i+1]) < 0.0)	// is a slope change
				change_a++;
		}

		// Dangling 2 Flag
		for(i = 0; i < pca_c.vertical_peaks.num[1]; i++)
		{
			if(pca_c.vertical_peaks.high_val[i] > (pca->motion_parametrs[0] + 0.6*pca->motion_parametrs[1]) && (pca_c.motion_peaks.num[0] + pca_c.motion_peaks.num[1]) < 5)
			{
				st_ind  = MAX(pca_c.vertical_peaks.high_ind[i]-3, 0);
				end_ind = MIN(pca_c.vertical_peaks.high_ind[i], DATA_SIZE_MIS-1);
				{
					for(k=0; k<pca_c.motion_peaks.num[0]; k++)
					{
						if(pca_c.motion_peaks.low_ind[k] >= st_ind && pca_c.motion_peaks.low_ind[k] <= end_ind && pca_c.motion_peaks.low_val[k] < (pca->motion_parametrs[0] - 0.6*pca->motion_parametrs[1]))
							dangling_counter++;
					}
				}
			}
		}
		if(dangling_counter > 0)
			dangling2_flag= 1;
		else
			dangling2_flag= 0;

		for(i = 0; i < DATA_SIZE_MIS; i++)
			R_mean += rad_rot_vec[i];

		R_mean /=DATA_SIZE_MIS;

		/////////////////////////////////////// Old New Flag  ///////////////////////////////////

		if(motion_unstability_flag == 1)
			Old_New_Flag = 1;
		else
			Old_New_Flag = 0;

		//Par_Values[10] =  pca->motion_parametrs[2];				// Motion range						(11)
		//Par_Values[11] =  pca->motion_parametrs[1];				// Motion std						(12)
		//Par_Values[12] =  lateral_range;						// Lateral range					(13)
		//Par_Values[13] =  std_l;								// Lateral std						(14)
		//Par_Values[14] =  pca->vertical_parametrs[2];			// Vertical range					(15)
		//Par_Values[15] =  pca->vertical_parametrs[1];			// Vertical std						(16)
		//Par_Values[16] =  horizontal_acceleration_range;		// Horizontal range					(17)
		//Par_Values[17] =  std_h;								// Horizontal std					(18)
		//Par_Values[18] =  mean_gyro;							// Gyro Magnitude mean				(19)
		//Par_Values[19] =  std_gyro;								// Gyro Magnitude std				(20)
		//Par_Values[20] =  range_gyro;							// Gyro Magnitude range				(21)
		//Par_Values[21] =  min_range_rp;							// Min or r_p ranges				(22)
		//Par_Values[22] =  max_range_rp;							// Max or r_p ranges				(23)
		//Par_Values[23] =  diff_range_rp;						// Diff	between max and min			(24)
		//Par_Values[24] =  min_std_rp;							// Min or r_p std					(25)
		//Par_Values[25] =  max_std_rp;							// Max or r_p std					(26)
		//Par_Values[26] =  diff_std_rp;							// Diff	between max and min			(27)
		//Par_Values[27] =  pca_c.change_m;						// Change in Motion					(28)
		//Par_Values[28] =  change_v;								// Change in Vertical				(29)
		//Par_Values[29] =  change_a;								// Change in Attitude				(30)
		////Par_Values[30] =  acc_var;							// Acceleration Variance			(31)
		////Par_Values[31] =  step_freq;							// Step Frequency					(32)
		////Par_Values[32] =  step_freq_range;					// Step Frequency range				(33)
		////Par_Values[33] =  step_ranges_thr;					// Step Frequency range thresholds	(34)
		////Par_Values[34] =  (FLOAT32)peaks_count;				// No. of Peaks						(35)
		//Par_Values[35] =  R_mean;								// Raidus of Rotaion Mean			(36)
		//Par_Values[36] =  R_flag;								// Raidus of Rotaion Flag			(37)
		//Par_Values[37] =  R_counter;							// Raidus of Rotaion Counter		(38)
		//Par_Values[38] =  R_thr;								// Raidus of Rotaion Counter thr	(39)
		//Par_Values[39] =  pocket_flag;							// Pocket Flag						(40)
		//Par_Values[40] =  dangling_flag;						// Dangling Flag					(41)
		//Par_Values[41] =  Old_New_Flag;							// Old_New_Flag						(42)
		//Par_Values[42] =  R_slow_flag;							// Raidus of slow Rotaion Flag		(43)
		//Par_Values[43] =  H_flag;								// Handheld flag					(44)
		//Par_Values[44] =  dangling2_flag;						// Dangling 2 Flag					(45)
#endif
	}
}

/**
* @brief	This function is a part of the motion classification funcrion.
*
* @details	The function takes two pointers to structures of type PCA and PCA_C.
* 			it returns a partial results for the device use cases.
*
* @param[in]	pca_c	pointer to a structure of type \c PCA_C.
* @param[in]	pca  	pointer to a structure of type \c PCA.
*
* @return	returns 0 in case of general, 1 in case of pocket, and 2 in case of dangling.
*
*/
INT8 classification_by_peaks_mis(PCA_C_Ptr pca_c, PCA_Ptr pca)
{
	UINT8 mode = 0;
	INT8 i, st_ind = -1, end_ind = -1;
	INT8 k1 = 0, k2 = 0, count1 = 0, count3 = 0;
	INT8 index_ver_h[PEAKS_VECTOR_SIZE_MIS] = {0};
	FLOAT32 max_h_p = 0.0;

	if((pca->motion_parametrs[2]-pca->vertical_parametrs[2]) > 5.0) // motion relatively big, this only happen in dangling. Added on 2012.06.22
		mode = 2;
	else
	{
		if(((pca->horizontal_vertical_flag == 0 && pca_c->b == 1 && pca->pitch_parametrs[1] > pca->roll_parametrs[1] && pca->motion_parametrs[1] >  1.6)  ||  /* Horizontal*/
			(pca->horizontal_vertical_flag != 0 && pca_c->b == 0 && pca->roll_parametrs[1] > pca->pitch_parametrs[1] && pca->pitch_parametrs[2] < 22.0)) &&  /* Vertical  */
			pca_c->not_dangling_flag == 0 && pca_c->change_m < 10 && ((pca->vertical_parametrs[2] < pca->motion_parametrs[2]) || (pca->vertical_parametrs[2] > 9.0 && pca->motion_parametrs[2] > 9.0 && pca->vertical_parametrs[2] < 1.2*pca->motion_parametrs[2])))
			mode = 2;
		else
		{
			if(pca_c->att_vec[2] > 18.0 && pca_c->motion_peaks.num[0] != 0 && pca_c->motion_peaks.num[1] != 0 && pca_c->vertical_peaks.num[0] != 0 && pca_c->vertical_peaks.num[1] != 0 && pca_c->attitude_peaks.num[0] != 0 && pca_c->attitude_peaks.num[1] != 0) //revised, June 22 2012
			{
				count3 = 0;


				if(((pca->horizontal_vertical_flag == 0 && pca->motion_parametrs[1] > 2.5)  ||  /* Horizontal*/
					(pca->horizontal_vertical_flag != 0 && pca->motion_parametrs[1] > 1.7)) &&  /* Vertical  */
					pca->vertical_parametrs[2] > 8.0 && pca->motion_parametrs[1]/pca->motion_parametrs[2] > 0.2)
				{
					// determining peaks that are higher or lower than a threshold
					for(i = 0; i < pca_c->vertical_peaks.num[1]; i++)
					{
						if(pca_c->vertical_peaks.high_ind[i] != -1 && pca_c->vertical_peaks.high_val[i] > floor(pca_c->vh_thr1))
							index_ver_h[count3++] = i;
					}
				}
				else
				{
					if(pca->vertical_parametrs[2] < 7.3 && pca->motion_parametrs[2] > 3.0 && pca->motion_parametrs[1] > 0.8)
					{
						// use highest peak directly
						index_ver_h[0] = -99;
						max_h_p = -99.0;
						for(i = 0; i < pca_c->vertical_peaks.num[1]; i++)
						{
							if(pca_c->vertical_peaks.high_val[i] > max_h_p)
							{
								max_h_p = pca_c->vertical_peaks.high_val[i];
								index_ver_h[0] = i;
							}
						}
						count3 = 1;
					}
				}

				count1 = 0;
				if(count3 > 0)  // Sharp peaks in vertical
				{
					k1 = pca_c->vertical_peaks.high_ind[index_ver_h[0]];
					if(pca_c->atti_vec[k1] > 0)
					{
						k1 = -1;
						k2 = -1;
						for(i = 0; i < pca_c->attitude_peaks.num[0]; i++)
						{
							if(pca_c->attitude_peaks.low_ind[i] > pca_c->vertical_peaks.high_ind[index_ver_h[0]])		//find the low roll peak after this v peak
							{
								k1 = i;
								break;
							}
							if(pca_c->attitude_peaks.low_ind[i] < pca_c->vertical_peaks.high_ind[index_ver_h[0]])		//find the low roll peak before this v peak
								k2 = i;
						}

						if(k1 != -1)
						{
							st_ind  = MAX(pca_c->attitude_peaks.low_ind[k1], 0);					//revised, April 14 2012
							end_ind = MIN(pca_c->attitude_peaks.low_ind[k1]+10, DATA_SIZE_MIS-1);				//revised, April 14 2012
							for(i = 0; i < pca_c->motion_peaks.num[0]; i++)
							{
								if(pca_c->motion_peaks.low_ind[i] < end_ind && pca_c->motion_peaks.low_ind[i] > st_ind)
									count1++;
							}
							for(i = 0; i < pca_c->motion_peaks.num[1]; i++)
							{
								if(pca_c->motion_peaks.high_ind[i] < end_ind && pca_c->motion_peaks.high_ind[i] > st_ind)
									count1++;
							}
						}
						if(k2 !=-1)
						{
							st_ind  = MAX(pca_c->attitude_peaks.low_ind[k2], 0);					//revised, April 14 2012
							end_ind = MIN(pca_c->attitude_peaks.low_ind[k2]+10, DATA_SIZE_MIS-1);				//revised, April 14 2012
							for(i = 0; i < pca_c->motion_peaks.num[0]; i++)
							{
								if(pca_c->motion_peaks.low_ind[i] < end_ind && pca_c->motion_peaks.low_ind[i] > st_ind)
									count1++;
							}
							for(i = 0; i < pca_c->motion_peaks.num[1]; i++)
							{
								if(pca_c->motion_peaks.high_ind[i] < end_ind && pca_c->motion_peaks.high_ind[i] > st_ind)
									count1++;
							}
						}
					}
					else  // sharp v peak locate at low peak roll
					{
						k1 = -1;
						k2 = -1;
						for(i = 0; i < pca_c->attitude_peaks.num[1]; i++)
						{
							if(pca_c->attitude_peaks.high_ind[i] > pca_c->vertical_peaks.high_ind[index_ver_h[0]])	//find the low roll peak after this v peak
							{
								k1 = i;
								break;
							}
							if(pca_c->attitude_peaks.high_ind[i] < pca_c->vertical_peaks.high_ind[index_ver_h[0]])	//find the low roll peak before this v peak
								k2 = i;
						}

						if(k1 != -1)
						{
							st_ind  = MAX(pca_c->attitude_peaks.high_ind[k1], 0);				//revised, April 14 2012
							end_ind = MIN(pca_c->attitude_peaks.high_ind[k1]+10, DATA_SIZE_MIS-1);			//revised, April 14 2012
							for(i = 0; i < pca_c->motion_peaks.num[1]; i++)
							{
								if(pca_c->motion_peaks.high_ind[i] < end_ind && pca_c->motion_peaks.high_ind[i] > st_ind)
									count1++;
							}
							for(i = 0; i < pca_c->motion_peaks.num[0]; i++)
							{
								if(pca_c->motion_peaks.low_ind[i] < end_ind && pca_c->motion_peaks.low_ind[i] > st_ind)
									count1++;
							}
						}
						if(k2 !=-1)
						{
							st_ind  = MAX(pca_c->attitude_peaks.high_ind[k2], 0);				//revised, April 14 2012
							end_ind = MIN(pca_c->attitude_peaks.high_ind[k2]+10, DATA_SIZE_MIS-1);			//revised, April 14 2012
							for(i = 0; i < pca_c->motion_peaks.num[1]; i++)
							{
								if(pca_c->motion_peaks.high_ind[i] < end_ind && pca_c->motion_peaks.high_ind[i] > st_ind)
									count1++;
							}
							for(i = 0; i < pca_c->motion_peaks.num[0]; i++)
							{
								if(pca_c->motion_peaks.low_ind[i] < end_ind && pca_c->motion_peaks.low_ind[i] > st_ind)
									count1++;
							}
						}
					}
					if(count1 > 0)
						mode = 1;
				}
			}
		}
	}
	return mode;
}
/**
* @brief	This function detects pocket pattern in the signal
*
* @details The function takes two pointers to structures of type PCA and PCA_C with a flag for
* 		   using low or high attitude peaks. it returns the number of detected pocket pattern.
*
* @param[in]	pca_c   	pointer to a structure of type \c PCA_C.
* @param[in]	pca			pointer to a structure of type \c PCA.
* @param[in]	h_l_flag	flag for low or high attitude peaks.
*
* @return	number of occurance of the pocket pattern.
*/
INT8 pocket_pattern_detection_mis(PCA_C_Ptr pca_c, PCA_Ptr pca, UINT8 h_l_flag)
{
	INT8 i, j, k, f, n;
	INT8 st_ind = -1, end_ind = -1, st_ind1 = -1, end_ind1 = -1;
	INT8 pocket_counter = 0;

	// Check for the peaks combination of high or low attiude peak, high vertical peak, and both motion high and/or low peaks
	for(i = 0; i < (pca_c->attitude_peaks.num[1]*h_l_flag + pca_c->attitude_peaks.num[0]*(!h_l_flag)); i++)
	{
		if((pca_c->attitude_peaks.high_val[i] > (pca_c->atth_thr1 - pca_c->att_vec[0]) && h_l_flag) ||		/* check for the attitude high peaks that greater than certain threshold*/
			(pca_c->attitude_peaks.low_val[i] < (pca_c->attl_thr1 - pca_c->att_vec[0]) && !h_l_flag))		/* check for the attitude low  peaks that less    than certain threshold*/
		{
			st_ind  = MAX((pca_c->attitude_peaks.high_ind[i]*h_l_flag + pca_c->attitude_peaks.low_ind[i]*(!h_l_flag))-2, 0);
			end_ind = MIN((pca_c->attitude_peaks.high_ind[i]*h_l_flag + pca_c->attitude_peaks.low_ind[i]*(!h_l_flag))+4, DATA_SIZE_MIS-1);
			// Get the closer vertical high peak
			for(j = 0; j < pca_c->vertical_peaks.num[1]; j++)
			{
				if(pca_c->vertical_peaks.high_ind[j] >= st_ind && pca_c->vertical_peaks.high_ind[j] <= end_ind && pca_c->vertical_peaks.high_val[j] > pca_c->vh_thr2 &&
					(pca_c->vertical_peaks.high_val[j] - pca->vertical_parametrs[0]) > (pca_c->mh_thr1 - pca->motion_parametrs[0]) &&
					(pca_c->vertical_peaks.high_val[j] - pca->vertical_parametrs[0]) < (pca_c->attitude_peaks.high_val[i]*h_l_flag - pca_c->attitude_peaks.low_val[i]*(!h_l_flag))*R2Df)
				{
					st_ind1  = MAX(pca_c->vertical_peaks.high_ind[j]-6, 0);
					end_ind1 = MIN(pca_c->vertical_peaks.high_ind[j]+6, DATA_SIZE_MIS-1);
					if((pca_c->attitude_peaks.high_ind[i]*h_l_flag + pca_c->attitude_peaks.low_ind[i]*(!h_l_flag)) >= 4 && (pca_c->attitude_peaks.high_ind[i]*h_l_flag + pca_c->attitude_peaks.low_ind[i]*(!h_l_flag)) <= 26)  // check for both motion high and low
					{
						// check if there is a motion high peak
						for(k = 0; k < pca_c->motion_peaks.num[1]; k++)
						{
							if(pca_c->motion_peaks.high_ind[k] >= st_ind1 && pca_c->motion_peaks.high_ind[k] <= end_ind1)
							{
								// if there is a motion high peak, check for motion low peak in the closer range
								for(f = 0; f < pca_c->motion_peaks.num[0]; f++)
								{
									if(pca_c->motion_peaks.low_ind[f] >= st_ind1 && pca_c->motion_peaks.low_ind[f] <= end_ind1)
									{
										pocket_counter++;
										for(n = 0; n < (pca_c->attitude_peaks.num[0]*h_l_flag + pca_c->attitude_peaks.num[1]*(!h_l_flag)); n++)
										{
											if((pca_c->attitude_peaks.low_ind[n]*h_l_flag + pca_c->attitude_peaks.high_ind[n]*(!h_l_flag)) >= st_ind1 && (pca_c->attitude_peaks.low_ind[n]*h_l_flag + pca_c->attitude_peaks.high_ind[n]*(!h_l_flag)) <= end_ind1)
												pocket_counter--;
										}
									}
								}
							}
						}
					}
					else // peaks are close to either start or end so check for high or low motion peaks
					{
						if((pca_c->motion_peaks.num[0] + pca_c->motion_peaks.num[1]) > 3) // to avoid dangling signal
						{
							// check if there is any motion high peak
							for(k = 0; k < pca_c->motion_peaks.num[1]; k++)
							{
								if(pca_c->motion_peaks.high_ind[k] >= st_ind1 && pca_c->motion_peaks.high_ind[k] <= end_ind1)
								{
									pocket_counter++;
									for(n = 0; n < (pca_c->attitude_peaks.num[0]*h_l_flag + pca_c->attitude_peaks.num[1]*(!h_l_flag)); n++)
									{
										if((pca_c->attitude_peaks.low_ind[n]*h_l_flag + pca_c->attitude_peaks.high_ind[n]*(!h_l_flag)) >= st_ind1 && (pca_c->attitude_peaks.low_ind[n]*h_l_flag + pca_c->attitude_peaks.high_ind[n]*(!h_l_flag)) <= end_ind1)
											pocket_counter--;
									}
								}
							}
							// check if there is any motion high peak
							for(k = 0; k < pca_c->motion_peaks.num[0]; k++)
							{
								if(pca_c->motion_peaks.low_ind[k] >= st_ind1 && pca_c->motion_peaks.low_ind[k] <= end_ind1)
								{
									pocket_counter++;
									for(n = 0; n < (pca_c->attitude_peaks.num[0]*h_l_flag + pca_c->attitude_peaks.num[1]*(!h_l_flag)); n++)
									{
										if((pca_c->attitude_peaks.low_ind[n]*h_l_flag + pca_c->attitude_peaks.high_ind[n]*(!h_l_flag)) >= st_ind1 && (pca_c->attitude_peaks.low_ind[n]*h_l_flag + pca_c->attitude_peaks.high_ind[n]*(!h_l_flag)) <= end_ind1)
											pocket_counter--;
									}
								}
							}
						}
						else
						{
							st_ind1  = MAX(pca_c->vertical_peaks.high_ind[j]-6, 0);
							end_ind1 = MIN(pca_c->vertical_peaks.high_ind[j]+6, DATA_SIZE_MIS-1);
							// check if there is a motion high peak
							for(k = 0; k < pca_c->motion_peaks.num[1]; k++)
							{
								if(pca_c->motion_peaks.high_ind[k] >= st_ind1 && pca_c->motion_peaks.high_ind[k] <= end_ind1)
								{
									// if there is a motion high peak, check for motion low peak in the closer range
									for(f = 0; f < pca_c->motion_peaks.num[0]; f++)
									{
										if(pca_c->motion_peaks.low_ind[f] >= st_ind1 && pca_c->motion_peaks.low_ind[f] <= end_ind1)
										{
											pocket_counter++;
											for(n = 0; n < (pca_c->attitude_peaks.num[0]*h_l_flag + pca_c->attitude_peaks.num[1]*(!h_l_flag)); n++)
											{
												if((pca_c->attitude_peaks.low_ind[n]*h_l_flag + pca_c->attitude_peaks.high_ind[n]*(!h_l_flag)) >= st_ind1 && (pca_c->attitude_peaks.low_ind[n]*h_l_flag + pca_c->attitude_peaks.high_ind[n]*(!h_l_flag)) <= end_ind1)
													pocket_counter--;
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}
	return pocket_counter;
}
/**
* @brief	This is peaks minimum distance function (repeated code).
*
* @details	This function takes a pointer to structure of type PCA_G, a variable for the
* 			method number, and flag to tell if local or normal motion peaks will be used. It fills
* 			the results of backward/forward in vector step. Also, it fills the minimum distances
* 			between peaks in vector min_peaks_dis. It returns the number of detected belt patterns.
*
* @param[in]	pca_g			pointer to a structure of type \c PCA_G.
* @param[in]	method_num		variable for method number.
* @param[in]	m_local			a flag for local peaks use.
* @param[out]	step		 	pointer to step vector.
* @param[out]	min_peaks_dis	pointer to min_peaks_dis vector.
*
* @return	belt check variable value.
*/
INT8 peaks_min_distance_mis(PCA_G_Ptr pca_g, UINT8 *min_peaks_dis, UINT8 method_num, UINT8 m_local, INT8 *step)
{
	INT8 i,j;
	INT8 min_h_dis = 0, min_l_dis = 0;
	INT8 id_h =- 1, id_l = -1, belt_check = 0;
	min_peaks_dis[0] = 0;
	min_peaks_dis[1] = 0;
	min_peaks_dis[2] = 0;
	min_peaks_dis[3] = 0;

	for(i = 0; i < pca_g->vertical.peaks.num[1]; i++)
	{
		min_h_dis = 99;		//min_vh_mh_dis
		min_l_dis = 99;		//min_vh_ml_dis
		for(j = 0; j < (pca_g->motion.peaks.num[1]*!(m_local) + pca_g->motion.local_peaks.num[1]*m_local); j++)
		{
			if(abs(pca_g->vertical.peaks.high_ind[i]-(pca_g->motion.peaks.high_ind[j]*!(m_local)+pca_g->motion.local_peaks.high_ind[j]*m_local)) < min_h_dis)
			{
				min_h_dis = (INT8)(abs(pca_g->vertical.peaks.high_ind[i]-(pca_g->motion.peaks.high_ind[j]*!(m_local)+pca_g->motion.local_peaks.high_ind[j]*m_local)));   // min_h_dis
				id_h = pca_g->motion.peaks.high_ind[j];
			}
		}
		for(j = 0; j < (pca_g->motion.peaks.num[0]*!(m_local) + pca_g->motion.local_peaks.num[0]*m_local); j++)
		{
			if(abs(pca_g->vertical.peaks.high_ind[i]-(pca_g->motion.peaks.low_ind[j]*!(m_local)+pca_g->motion.local_peaks.low_ind[j]*m_local)) < min_l_dis)
			{
				min_l_dis = (INT8)(abs(pca_g->vertical.peaks.high_ind[i]-(pca_g->motion.peaks.low_ind[j]*!(m_local)+pca_g->motion.local_peaks.low_ind[j]*m_local)));   // min_l_dis
				id_l = pca_g->motion.peaks.low_ind[j];
			}
		}

		if(abs(min_h_dis-min_l_dis) < 2 && MIN(min_h_dis, min_l_dis) < 3 && method_num == 2)
		{
			belt_check++;
			if(id_h > id_l)
				step[0]++;
			else
				step[1]++;
		}
		else
		{
			if((min_h_dis < min_l_dis && min_h_dis < 2 && min_l_dis > 2 && (method_num == 0 || method_num == 7)) ||
				(min_h_dis < min_l_dis && min_h_dis < 3 && (method_num == 1 ||method_num == 2)) ||
				(min_h_dis < min_l_dis && min_h_dis <= 3 && (method_num == 3 || method_num == 5 || method_num == 6)))
				min_peaks_dis[2]++;		//method_sh++;   v_h
			if((min_h_dis > min_l_dis && min_h_dis > 2 && min_l_dis < 2 && (method_num == 0 || method_num == 7)) ||
				(min_h_dis > min_l_dis && min_l_dis < 3 && (method_num == 1 ||method_num == 2)) ||
				(min_h_dis > min_l_dis && min_l_dis <= 3 && (method_num == 3 || method_num == 5 || method_num == 6)))
				min_peaks_dis[3]++;		//method_sl++;   v_h
		}
	}

	for(i = 0; i < pca_g->vertical.peaks.num[0]; i++)
	{
		min_h_dis = 99;		//min_vl_mh_dis
		min_l_dis = 99;		//min_vl_ml_dis
		for(j = 0; j < (pca_g->motion.peaks.num[1]*!(m_local) + pca_g->motion.local_peaks.num[1]*m_local); j++)
		{
			if(abs(pca_g->vertical.peaks.low_ind[i]-(pca_g->motion.peaks.high_ind[j]*!(m_local)+pca_g->motion.local_peaks.high_ind[j]*m_local)) < min_h_dis)
			{
				min_h_dis = (INT8)(abs(pca_g->vertical.peaks.low_ind[i]-(pca_g->motion.peaks.high_ind[j]*!(m_local)+pca_g->motion.local_peaks.high_ind[j]*m_local)));   // min_h_dis
				id_h = pca_g->motion.peaks.high_ind[j];
			}
		}
		for(j = 0; j < (pca_g->motion.peaks.num[0]*!(m_local) + pca_g->motion.local_peaks.num[0]*m_local); j++)
		{
			if(abs(pca_g->vertical.peaks.low_ind[i]-(pca_g->motion.peaks.low_ind[j]*!(m_local)+pca_g->motion.local_peaks.low_ind[j]*m_local)) < min_l_dis)
			{
				min_l_dis = (INT8)(abs(pca_g->vertical.peaks.low_ind[i]-(pca_g->motion.peaks.low_ind[j]*!(m_local)+pca_g->motion.local_peaks.low_ind[j]*m_local)));   // min_l_dis
				id_l = pca_g->motion.peaks.low_ind[j];
			}
		}
		if(abs(min_h_dis-min_l_dis) < 2 && MIN(min_h_dis, min_l_dis) < 3 && method_num == 2)
		{
			if(pca_g->i_chest == 0)
			{
				belt_check++;
				if(id_h < id_l)
					step[0]++;
				else
					step[1]++;
			}
			else
			{
				if(pca_g->vertical.peaks.low_ind[i] < id_h && pca_g->vertical.peaks.low_ind[i] > id_l)
					step[1]++;
				if(pca_g->vertical.peaks.low_ind[i] < id_l && pca_g->vertical.peaks.low_ind[i] > id_h)
					step[0]++;
			}
		}
		else
		{
			if((min_h_dis < min_l_dis && min_h_dis < 2 && min_l_dis > 2 && (method_num == 0 || method_num == 7)) ||
				(min_h_dis > min_l_dis && min_l_dis < 3 && (method_num == 1 ||method_num == 2)) ||
				(min_h_dis < min_l_dis && min_h_dis <= 3 && method_num == 3) ||
				(min_h_dis > min_l_dis && min_l_dis <= 3 && (method_num == 5 || method_num == 6)))
				min_peaks_dis[1]++;			//method_sh++;  v_l
			if((min_h_dis > min_l_dis && min_h_dis > 2 && min_l_dis < 2 && (method_num == 0 || method_num == 7)) ||
				(min_h_dis < min_l_dis && min_h_dis < 3 && (method_num == 1 ||method_num == 2)) ||
				(min_h_dis > min_l_dis && min_l_dis <= 3 && method_num == 3) ||
				(min_h_dis < min_l_dis && min_h_dis <= 3 && (method_num == 5 || method_num == 6)))
				min_peaks_dis[0]++;			//method_sl++;  v_l
		}
	}

	if(method_num == 3 || method_num == 5 || method_num == 6)
	{
		if((min_peaks_dis[1]+min_peaks_dis[2]) > (min_peaks_dis[0]+min_peaks_dis[3]))
			step[1]++;
		if((min_peaks_dis[0]+min_peaks_dis[3]) > (min_peaks_dis[1]+min_peaks_dis[2]))
			step[0]++;
	}

	if(method_num == 7)
	{
		if(min_peaks_dis[3] >= (pca_g->vertical.peaks.num[1]-1) && min_peaks_dis[3] != 0)
			step[1]++;
		if(min_peaks_dis[2] >= (pca_g->vertical.peaks.num[1]-1) && min_peaks_dis[2] != 0)
			step[0]++;
		if(min_peaks_dis[0] >= (pca_g->vertical.peaks.num[0]-1) && min_peaks_dis[0] != 0)
			step[0]++;
		if(min_peaks_dis[1] >= (pca_g->vertical.peaks.num[0]-1) && min_peaks_dis[1] != 0)
			step[1]++;
	}

	return belt_check;
}
/**
* @brief	This is slope segment function (repeated code).
*
* @details	This function takes peaks for certain data. It uses the data slope to give
* 			a decision for Forward/Backward based on the interval width around the peaks and
* 			certain threshold. it fills in a pointer to vector as output (step).
*
* @param[in]	peaks_num	The peaks number.
* @param[in]	peaks_ind	pointer to peaks indices.
* @param[in]	peaks_val	pointer to peaks values.
* @param[in]	data	 	pointer to the 1-D vector of data.
* @param[in]	be_ind		variable for number of peaks before the operating peak.
* @param[in]	af_ind		variable for number of peaks after the operating peak.
* @param[in]	method_num	variable for method number.
* @param[in]	threshold	variable for threshold.
* @param[in]	vh_vl		flag to use vertical high or low peaks.
* @param[out]	step	 	pointer to step vector.
*/
void slope_segment_mis(INT8 peaks_num, INT8 *peaks_ind, FLOAT32 *peaks_val, FLOAT32 *data, UINT8 be_ind, UINT8 af_ind, UINT8 method_num, FLOAT32 threshold, INT8 *step, UINT8 vh_vl)
{
	INT8 i;
	INT8 st_ind = -1, en_ind = -1;
	FLOAT32 slope_m = 0.0;
	for(i=0; i<peaks_num; i++)
	{
		if((((peaks_val[i] > threshold && vh_vl == 1)|| (peaks_val[i] < threshold && vh_vl == 0)) && method_num == 0) || method_num == 1)
		{
			st_ind = MAX(peaks_ind[i]-be_ind, 0);
			en_ind = MIN(peaks_ind[i]+af_ind, DATA_SIZE_MIS-1);
			slope_m = data[en_ind] - data[st_ind];
			(vh_vl == 1)?((slope_m >= 0)?step[0]++:step[1]++):((slope_m <= 0)?step[0]++:step[1]++);
		}
	}
}
/**
* @brief	This is Motion peaks segment function (repeated code).
*
* @details	This function takes motion data peaks and uses the relative relation between
* 			the motion and vertical peaks to give decision for Forward/Backward based on the
* 			interval width around the peaks and certain threshold. it fills in a pointer to
* 			vector as output (step).
*
* @param[in]	peaks		pointer for structure of type \c PEAKS.
* @param[in]	m_thrs		variable for thresholds.
* @param[in]	st_ind		variable for start index around the operating peak.
* @param[in]	en_ind		variable for end index around the operating peak.
* @param[in]	Method_Num	variable for method number.
* @param[in]	vh_vl		flag to use vertical high or low peaks.
* @param[out]	step  		pointer to step vector.
*/
void motion_peaks_segment_mis(PEAKS_Ptr peaks, FLOAT32 *m_thrs, INT8 st_ind, INT8 en_ind, UINT8 Method_Num, UINT8 vh_vl, INT8 *step)
{
	INT8 j = 0;

	for(j=0; j<peaks->num[1]; j++)
	{
		if(peaks->high_ind[j] >= st_ind && peaks->high_ind[j] <= en_ind && ((peaks->high_val[j] > m_thrs[1] && Method_Num == 2) || Method_Num == 1))
			(vh_vl == 1)?step[1]++:step[0]++;
	}
	for(j=0; j<peaks->num[0]; j++)
	{
		if(peaks->low_ind[j] >= st_ind && peaks->low_ind[j] <= en_ind && ((peaks->low_val[j] < m_thrs[0] && Method_Num == 2) || Method_Num == 1))
			(vh_vl == 1)?step[0]++:step[1]++;
	}
}
/**
* @brief	This is pocket backward forward function to solve the 180 amiguity problem in case of pocket data.
*
* @details	This function takes the roll and pitch data with other data parameters stored in
* 			2 pointers of structures of type PCA and PCA_M. it returns the decision of Forward/Backward.
*
* @param[in]	data_r	pointer to roll data vector.
* @param[in]	data_p	pointer to pitch data vector.
* @param[in]	pca		pointer to a structure of type \c PCA.
* @param[in]	pca_m  	pointer to a structure of type \c PCA_M.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 pocket_backward_forward_mis(FLOAT32 *data_r, FLOAT32 *data_p, PCA_Ptr pca, PCA_M_Ptr pca_m)
{
	INT8 i;
	INT8 st_ind = -1, en_ind = -1;
	INT8 k1 = 0, k2 = 0, k3 = 0, b = 0, a1 = 0, a2 = 0;
	INT8 count1 = 0, count3 = 0;
	INT16 step_f = 0, step_b = 0;
	INT8 temp_res = 2;
	FLOAT32 temp = 0;
	INT8 index_ver_h[PEAKS_VECTOR_SIZE_MIS] = {0};

	PCA_C pca_c;
	memset(&pca_c, 0, sizeof(PCA_C));

	// Motion Peaks Detection
	i = peaks_detection_mis(pca->data_m, pca->motion_parametrs[1]/3, 4, &pca_c.motion_peaks, 20, 36, 0);

	// Vertical Peaks Detection
	i = peaks_detection_mis(pca->data_v, pca->vertical_parametrs[1]/3, 4, &pca_c.vertical_peaks, 20, 36, 0);
	if(pca_c.vertical_peaks.num[0] == 0 || pca_c.vertical_peaks.num[1] == 0)
		i = peaks_detection_mis(pca->data_v, pca->vertical_parametrs[1]/5, 4, &pca_c.vertical_peaks, 20, 36, 0);

	if(pca->vertical_parametrs[2] > 8)
	{
		if(pca_c.motion_peaks.num[0] == 0 || pca_c.motion_peaks.num[1] == 0) //obviously not a slow motion. HWC. 2012.06.27
			i = peaks_detection_mis(pca->data_m, pca->motion_parametrs[1]/5, 4, &pca_c.motion_peaks, 20, 36, 0);
		else
			i = peaks_detection_mis(pca->data_m, pca->motion_parametrs[1]/3, 4, &pca_c.motion_peaks, 20, 36, 0);
	}
	else
	{
		i = peaks_detection_mis(pca->data_m, pca->motion_parametrs[2]/8, 4, &pca_c.motion_peaks, 20, 36, 0);
		if(pca_c.motion_peaks.num[0] == 0 || pca_c.motion_peaks.num[1] == 0)
			i = peaks_detection_mis(pca->data_m, pca->motion_parametrs[2]/10, 4, &pca_c.motion_peaks, 20, 36, 0);
	}

	//delete unreasonable motion low peaks
	count1 = 0;
	for(i = 0; i < pca_c.motion_peaks.num[0]; i++)
	{
		if(pca_c.motion_peaks.low_val[i] < pca->motion_parametrs[0])
		{
			pca_c.motion_peaks.low_ind[count1]   = pca_c.motion_peaks.low_ind[i];
			pca_c.motion_peaks.low_val[count1++] = pca_c.motion_peaks.low_val[i];
		}
	}
	pca_c.motion_peaks.num[0] = count1;

	//delete unreasonable motion high peaks
	count1 = 0;
	for(i = 0; i < pca_c.motion_peaks.num[1]; i++)
	{
		if(pca_c.motion_peaks.high_val[i] > pca->motion_parametrs[0])
		{
			pca_c.motion_peaks.high_ind[count1]   = pca_c.motion_peaks.high_ind[i];
			pca_c.motion_peaks.high_val[count1++] = pca_c.motion_peaks.high_val[i];
		}
	}
	pca_c.motion_peaks.num[1] = count1;

	// Revised and modified by Hsiu-Wen Chang, june 22, 2012
	//if(pca->horizontal_vertical_flag == 0)
	if(pca->roll_parametrs[2] > pca->pitch_parametrs[2]) //roll has bigger motion
	{
		pca_c.att_vec[1] = pca->roll_parametrs[1];
		pca_c.att_vec[2] = pca->roll_parametrs[2];
		b = 0;
		for(i = 0;  i < DATA_SIZE_MIS; i++)
			pca_c.atti_vec[i] = data_r[i] - pca->roll_parametrs[0];
	}
	else  //pitch has bigger motion
	{
		pca_c.att_vec[1] = pca->pitch_parametrs[1];
		pca_c.att_vec[2] = pca->pitch_parametrs[2];
		b = 1;
		for(i = 0;  i < DATA_SIZE_MIS; i++)
			pca_c.atti_vec[i] = data_p[i] - pca->pitch_parametrs[0];
	}

	//i = peaks_detection_mis(pca_c.atti_vec, pca_c.att_vec[2]*D2Rf/6, 4, pca_c.attitude_peaks.low_ind, pca_c.attitude_peaks.low_val, pca_c.attitude_peaks.high_ind, pca_c.attitude_peaks.high_val, pca_c.attitude_peaks.num, 20, 36, 0);
	i = peaks_detection_mis(pca_c.atti_vec, pca_c.att_vec[2]*D2Rf*0.1667F, 4, &pca_c.attitude_peaks, 20, 36, 0);

	//start to detect forward and backward
	count3 = 0;
	// determining peaks that are higher or lower than a threshold
	for(i = 0; i < pca_c.vertical_peaks.num[1]; i++)
	{
		if(pca_c.vertical_peaks.high_ind[i] != -1 && pca_c.vertical_peaks.high_val[i] > floor(pca->vertical_parametrs[0]+1.7*pca->vertical_parametrs[1]))
			index_ver_h[count3++] = i; //keep peak higher than 2*std
	}

	if(count3 == 0) //this will happen with the very slow motion so use highest vh_peak.
	{
		temp = -99;
		for(i = 0; i < pca_c.vertical_peaks.num[1]; i++)
		{
			if(pca_c.vertical_peaks.high_val[i] > temp)
			{
				index_ver_h[0] = i; //keep peak higher than 2*std
				temp = pca_c.vertical_peaks.high_val[i];
			}
		}
		count3 = 1;
	}
	//Crash protection in motion
#ifdef METHOD_RESULTS_MISALIGNMENT
	if(pca_c.motion_peaks.num[0] == 0 || pca_c.motion_peaks.num[1] == 0 || pca_c.vertical_peaks.num[0] == 0 || pca_c.vertical_peaks.num[1] == 0)
	{
		//My_Method_Worked[1] = 1;
		//My_Method_Num = 1;
		return 2;
	}
#else
	if(pca_c.motion_peaks.num[0] == 0 || pca_c.motion_peaks.num[1] == 0 || pca_c.vertical_peaks.num[0] == 0 || pca_c.vertical_peaks.num[1] == 0)
		return 2;
#endif
	// Method 1
	//Added on March 31, 2014:
#ifdef COM_WALK_SIMPLIFIED
	if(pca_m->height_change_flag == 0 && fabs(pca_m->vertical_motion_angle) > 40.0)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[2] = 1;
		if(pca_m->vertical_motion_angle > 40.0)
		{
			//My_Method_Num = 2;
			return 0;
		}
		else if(pca_m->vertical_motion_angle < -40.0)
		{
			//My_Method_Num = 2;
			return 1;
		}
#else
		if(pca_m->vertical_motion_angle > 40.0)
			return 0;
		else if(pca_m->vertical_motion_angle < -40.0)
			return 1;
#endif
	}
#else
	if(pca_m->height_change_flag == 0)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		if(fabs(pca_m->vertical_motion_angle) > 40.0)
		{
			//My_Method_Worked[2] = 1;
			if(pca_m->vertical_motion_angle > 40.0)
			{
				//My_Method_Num = 2;
				return 0;
			}
			else if(pca_m->vertical_motion_angle < -40.0)
			{
				//My_Method_Num = 2;
				return 1;
			}
		}
		else if(fabs(pca_m->motion_effective_coefficient) > 0.2)
		{
			//My_Method_Worked[2] = 1;
			if(pca_m->motion_effective_coefficient > 0.2)
			{
				//My_Method_Num = 2;
				return 1;
			}
			else if(pca_m->motion_effective_coefficient < -0.2)
			{
				//My_Method_Num = 2;
				return 0;
			}
		}
#else
		if(fabs(pca_m->vertical_motion_angle) > 40.0)
		{
			if(pca_m->vertical_motion_angle > 40.0)
				return 0;
			else if(pca_m->vertical_motion_angle < -40.0)
				return 1;
		}
		else if(fabs(pca_m->motion_effective_coefficient) > 0.2)
		{
			if(pca_m->motion_effective_coefficient > 0.2)
				return 1;
			else if(pca_m->motion_effective_coefficient < -0.2)
				return 0;
		}
#endif
	}
#endif
	// End of Method 1

	// Method 2
	if(pca->motion_parametrs[2] > 6 && pca->motion_parametrs[2] > pca->vertical_parametrs[2] && pca->roll_parametrs[1] < 0.4 && pca->pitch_parametrs[1] < 0.4 && (pca_c.vertical_peaks.num[0] + pca_c.vertical_peaks.num[1]) >= 5 && (pca_c.motion_peaks.num[0] + pca_c.motion_peaks.num[1]) >= 5) //relatively stable, should be purse
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[3] = 1;
		temp_res = xcorr_backward_forward_mis(pca);
		if(temp_res != 2)
		{
			//My_Method_Num = 3;
			return temp_res;
		}
#else
		temp_res = xcorr_backward_forward_mis(pca);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 2

	// Method 3
	if(pca_c.attitude_peaks.num[0] != 0 && pca_c.attitude_peaks.num[1] != 0 && count3 != 0)
	{
		count1 = 0;
		k1 = pca_c.vertical_peaks.high_ind[index_ver_h[0]];
		if(pca_c.atti_vec[k1] > 0)//sharp v peak locate at high peak roll
		{
			k1 = -1; k2 = -1; k3 = -1;
			a1 = -1; a2 = -1;
			for(i = 0; i < pca_c.attitude_peaks.num[0]; i++)
			{
				if(pca_c.attitude_peaks.low_ind[i] > pca_c.vertical_peaks.high_ind[index_ver_h[0]])
				{
					k1 = i;
					break;
				}				//find the low roll peak after this v peak
				if(pca_c.attitude_peaks.low_ind[i] < pca_c.vertical_peaks.high_ind[index_ver_h[0]])
					k2 = i;
			}

			if(k1 != -1)
			{
				k3 = k1;
				st_ind = MAX(pca_c.attitude_peaks.low_ind[k1], 0);
				en_ind = MIN(pca_c.attitude_peaks.low_ind[k1]+10, DATA_SIZE_MIS-1);
				for(i = 0; i < pca_c.motion_peaks.num[0]; i++)
				{
					if(pca_c.motion_peaks.low_ind[i] <= en_ind && pca_c.motion_peaks.low_ind[i] >= st_ind)
					{
						a1 = pca_c.motion_peaks.low_ind[i];
						break;
					}
				}
				for(i = 0; i < pca_c.motion_peaks.num[1]; i++)
				{
					if(pca_c.motion_peaks.high_ind[i] <= en_ind && pca_c.motion_peaks.high_ind[i] >= st_ind)
					{
						a2 = pca_c.motion_peaks.high_ind[i];
						break;
					}
				}
			}
			if(a1 == -1 && a2 == -1 && k2 != -1)	//find the low roll peak before this v peak, revised, April 14
			{
				k3 = k2;
				st_ind = MAX(pca_c.attitude_peaks.low_ind[k2], 0);					//revised, April 14
				en_ind = MIN(pca_c.attitude_peaks.low_ind[k2]+10, DATA_SIZE_MIS-1);		//revised, April 14
				for(i = 0; i < pca_c.motion_peaks.num[0]; i++)
				{
					if(pca_c.motion_peaks.low_ind[i] <= en_ind && pca_c.motion_peaks.low_ind[i] >= st_ind)
					{
						a1 = pca_c.motion_peaks.low_ind[i];
						break;
					}
				}
				for(i = 0; i < pca_c.motion_peaks.num[1]; i++)
				{
					if(pca_c.motion_peaks.high_ind[i] <= en_ind && pca_c.motion_peaks.high_ind[i] >= st_ind)
					{
						a2 = pca_c.motion_peaks.high_ind[i];
						break;
					}
				}
			}
			if(a1 != -1 && a2 != -1 && k3 != -1)
			{
				if(abs(a1 - pca_c.attitude_peaks.low_ind[k3]) < abs(a2 - pca_c.attitude_peaks.low_ind[k3]))
					step_b++;
				else
					step_f++;
			}
			if(a1 != -1 && a2 == -1 && k3 != -1)
				step_b++;
			if(a1 == -1 && a2 != -1 && k3 != -1)
				step_f++;
		} //if(atti_vec[  vh_peaks[index_ver_h[0]][0] ] > 0
		else  //sharp v peak locate at low peak roll
		{
			k1 = -1; k2 = -1; k3 = -1;
			a1 = -1; a2 = -1;
			for(i = 0; i < pca_c.attitude_peaks.num[1]; i++)
			{
				if(pca_c.attitude_peaks.high_ind[i] > pca_c.vertical_peaks.high_ind[index_ver_h[0]])
				{
					k1 = i;
					break;
				}		//find the low roll peak after this v peak
				if(pca_c.attitude_peaks.high_ind[i] < pca_c.vertical_peaks.high_ind[index_ver_h[0]])
					k2 = i;
			}
			if(k1 != -1)
			{
				k3= k1;
				st_ind = MAX(pca_c.attitude_peaks.high_ind[k1], 0);				//revised, April 14
				en_ind = MIN(pca_c.attitude_peaks.high_ind[k1]+10, DATA_SIZE_MIS-1); //revised, April 14
				for(i = 0; i < pca_c.motion_peaks.num[1]; i++)
				{
					if(pca_c.motion_peaks.high_ind[i] <= en_ind && pca_c.motion_peaks.high_ind[i] >= st_ind)
					{
						a1 = pca_c.motion_peaks.high_ind[i];
						break;
					}
				}
				for(i = 0; i < pca_c.motion_peaks.num[0]; i++)
				{
					if(pca_c.motion_peaks.low_ind[i] <= en_ind && pca_c.motion_peaks.low_ind[i] >= st_ind)
					{
						a2 = pca_c.motion_peaks.low_ind[i];
						break;
					}
				}
			}
			if(a1 == -1 && a2 == -1 && k2 != -1)		//find the low roll peak before this v peak, revised, April 14
			{
				k3= k2;
				st_ind = MAX(pca_c.attitude_peaks.high_ind[k2], 0);						//revised, April 14
				en_ind = MIN(pca_c.attitude_peaks.high_ind[k2]+10, DATA_SIZE_MIS-1);			//revised, April 14
				for(i = 0; i < pca_c.motion_peaks.num[1]; i++)
				{
					if(pca_c.motion_peaks.high_ind[i] <= en_ind && pca_c.motion_peaks.high_ind[i] >= st_ind)
					{
						a1 = pca_c.motion_peaks.high_ind[i];
						break;
					}
				}
				for(i = 0; i < pca_c.motion_peaks.num[0]; i++)
				{
					if(pca_c.motion_peaks.low_ind[i] <= en_ind && pca_c.motion_peaks.low_ind[i] >= st_ind)
					{
						a2 = pca_c.motion_peaks.low_ind[i];
						break;
					}
				}
			}
			if(a1 != -1 && a2 != -1 && k3 != -1)
			{
				if(abs(a1 - pca_c.attitude_peaks.high_ind[k3]) > abs(a2 - pca_c.attitude_peaks.high_ind[k3]))
					step_b++;
				else
					step_f++;
			}
			if(a1 != -1 && a2 == -1 && k3 != -1)
				step_f++;
			if(a1 == -1 && a2 != -1 && k3 != -1)
				step_b++;
		}
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[4] = 1;
		if(step_f > step_b)
		{
			//My_Method_Num = 4;
			return 0 ;
		}
		else if(step_b > step_f)
		{
			//My_Method_Num = 4;
			return 1 ;
		}
#else
		if(step_f > step_b)
			return 0 ;
		else if(step_b > step_f)
			return 1 ;
#endif
	}
	// End of Method 3
#ifdef METHOD_RESULTS_MISALIGNMENT
	//My_Method_Worked[1] = 1;
	//My_Method_Num = 1;
#endif
	return 2 ;
}
/**
* @brief	This is dangling backward forward function to solve the 180 amiguity problem in case of dangling data.
*
* @details	This function takes the motion, vertical, roll and pitch data with other data parameters stored in
* 			2 pointers of structures of type PCA and PCA_M. it calls 2 different functions to get the decision
* 			of Forward/Backward in the case of dangling data.
*
* @param[in]	pca		pointer to a structure of type \c PCA.
* @param[in]	pca_m  	pointer to a structure of type \c PCA_M.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*
* @see dangling_phone_backward_forward_mis() and dangling_tablet_watch_backward_forward_mis()
*/
INT8 dangling_backward_forward_mis(PCA_Ptr pca, PCA_M_Ptr pca_m)
{
	INT8 i;
	INT8 temp_res = 2;
	PCA_D pca_d;
	memset(&pca_d, 0, sizeof(PCA_D));

	// Motion Peaks Detection
	i = peaks_detection_mis(pca->data_m, pca->motion_parametrs[1], 4, &pca_d.motion.peaks, 20, 36, 0);
	if(pca_d.motion.peaks.num[0] == 0.0 || pca_d.motion.peaks.num[1] == 0.0)
		i = peaks_detection_mis(pca->data_m, pca->motion_parametrs[1]/2, 4, &pca_d.motion.peaks, 20, 36, 0);
	if(pca_d.motion.peaks.num[0] == 0.0 || pca_d.motion.peaks.num[1] == 0.0)
		i = peaks_detection_mis(pca->data_m, pca->motion_parametrs[2]/8, 4, &pca_d.motion.peaks, 20, 36, 0);
	if((pca_d.motion.peaks.num[0] < 2 || pca_d.motion.peaks.num[1] < 2) && pca->motion_parametrs[2]/8 > pca->motion_parametrs[1])
		i = peaks_detection_mis(pca->data_m, pca->motion_parametrs[1], 4, &pca_d.motion.peaks, 20, 36, 0);

	// Vertical Peaks Detection
	i = peaks_detection_mis(pca->data_v, pca->vertical_parametrs[1]/2.0F, 4, &pca_d.vertical.peaks, 20, 36, 0);

	// define the necessary thresholds
	pca_d.motion.thresholds[0] = pca->motion_parametrs[0] -     pca->motion_parametrs[1];
	pca_d.motion.thresholds[1] = pca->motion_parametrs[0] +     pca->motion_parametrs[1];
	pca_d.vertical.thresholds[0] = pca->vertical_parametrs[0] - 1.2F*pca->vertical_parametrs[1];
	pca_d.vertical.thresholds[1] = pca->vertical_parametrs[0] + 1.2F*pca->vertical_parametrs[1];
	pca_d.vertical.thresholds[2] = pca->vertical_parametrs[0] - 1.4F*pca->vertical_parametrs[1];
	pca_d.vertical.thresholds[3] = pca->vertical_parametrs[0] + 1.4F*pca->vertical_parametrs[1];
	pca_d.vertical.thresholds[4] = pca->vertical_parametrs[0] - 1.5F*pca->vertical_parametrs[1];
	pca_d.vertical.thresholds[5] = pca->vertical_parametrs[0] + 1.5F*pca->vertical_parametrs[1];

	//Crash protection in motion
#ifdef METHOD_RESULTS_MISALIGNMENT
	if((pca_d.motion.peaks.num[0] + pca_d.motion.peaks.num[1]) > 20 || pca->vertical_parametrs[2] < 0.8)   	//Crash protection in motion
	{
		//My_Method_Worked[10] = 1;
		//My_Method_Num = 10;
		return 0;
	}
	else if(pca_d.motion.peaks.num[0] == 0 || pca_d.motion.peaks.num[1] == 0 || pca_d.vertical.peaks.num[0] == 0 || pca_d.vertical.peaks.num[1] == 0)
	{
		//My_Method_Worked[10] = 1;
		//My_Method_Num = 10;
		return 2;
	}
#else
	if((pca_d.motion.peaks.num[0] + pca_d.motion.peaks.num[1]) > 20 || pca->vertical_parametrs[2] < 0.8)   	//Crash protection in motion
		return 0;
	else if(pca_d.motion.peaks.num[0] == 0 || pca_d.motion.peaks.num[1] == 0 || pca_d.vertical.peaks.num[0] == 0 || pca_d.vertical.peaks.num[1] == 0)
		return 2;
#endif

	if(pca_m->device_flag == 0 || pca_m->device_flag == 2)
	{
# ifndef COM_WALK_SIMPLIFIED
		temp_res = dangling_phone_backward_forward_mis(pca, pca_m, &pca_d);
#else
		temp_res = dangling_phone_backward_forward_mis(pca, &pca_d);
#endif
	}
	else
		temp_res = dangling_tablet_watch_backward_forward_mis(pca, pca_m, &pca_d);
	return temp_res;
}
#ifndef COM_WALK_SIMPLIFIED
/**
* @brief	This is phone dangling backward forward function to solve the 180 amiguity problem in
* 			case of phone dangling data.
*
* @details	This function takes 3 pointers of structures of type PCA, PCA_M, and PCA_D. it calls different
* 			sub-functions to get the decision of Forward/Backward in the case of phone dangling data.
*
* @param[in]	pca		pointer to a structure of type \c PCA.
* @param[in]	pca_m  	pointer to a structure of type \c PCA_M.
* @param[in]	pca_d  	pointer to a structure of type \c PCA_D.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*
* @see dangling_phone_1_mis(), dangling_phone_2_mis(), ....
*/
INT8 dangling_phone_backward_forward_mis(PCA_Ptr pca, PCA_M_Ptr pca_m, PCA_D_Ptr pca_d)
#else
/**
* @brief	This is phone dangling backward forward function to solve the 180 amiguity problem in
* 			case of phone dangling data.
*
* @details	This function takes 3 pointers of structures of type PCA, PCA_M, and PCA_D. it calls different
* 			sub-functions to get the decision of Forward/Backward in the case of phone dangling data.
*
* @param[in]	pca		pointer to a structure of type \c PCA.
* @param[in]	pca_d  	pointer to a structure of type \c PCA_D.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*
* @see dangling_phone_1_mis(), dangling_phone_2_mis(), ....
*/
INT8 dangling_phone_backward_forward_mis(PCA_Ptr pca, PCA_D_Ptr pca_d)
#endif
{
	INT8 i;
	INT8 temp_res = 2;
	UINT8 flag_h = 1;
	FLOAT32 diff_v  = 0.0, min_vh_val = 99.0, max_vl_val	= -99.0;

#ifndef COM_WALK_SIMPLIFIED
	// Method 1
	if(pca_m->device_flag == 0 && pca_m->height_change_flag == 0 && fabs(pca_m->vertical_motion_phase-90) > 13 && fabs(pca_m->motion_effective_coefficient) > 0.01)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[11] = 1;
		if(pca_m->motion_effective_coefficient > 0)
		{
			//My_Method_Num = 11;
			return 1;
		}
		else
		{
			//My_Method_Num = 11;
			return 0;
		}
#else
		if(pca_m->motion_effective_coefficient > 0)
			return 1;
		else
			return 0;
#endif
	}
	// End of Method 1
#endif

	// Method 2
	if(pca->motion_parametrs[2] > 6 && pca->motion_parametrs[2] > pca->vertical_parametrs[2] && pca->roll_parametrs[1] < 0.4 && pca->pitch_parametrs[1] < 0.4 && (pca_d->vertical.peaks.num[0] + pca_d->vertical.peaks.num[1]) >= 5 && (pca_d->motion.peaks.num[1] + pca_d->motion.peaks.num[0]) >= 5)	//relatively stable, should be purse
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[12] = 1;
		temp_res = xcorr_backward_forward_mis(pca);
		if(temp_res != 2)
		{
			//My_Method_Num = 12;
			return temp_res;
		}
#else
		temp_res = xcorr_backward_forward_mis(pca);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 2

	// Method 3			Dangling_1()
	// Added June 25th, 2012: deal with the unstable vertical signal
	if(pca->motion_parametrs[2] > 20 && pca->vertical_parametrs[2] > 10 && pca->motion_parametrs[2] > pca->vertical_parametrs[2] && (pca_d->motion.peaks.num[1] + pca_d->motion.peaks.num[0]) < 4 && (pca_d->vertical.peaks.num[0] + pca_d->vertical.peaks.num[1]) > 5)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[13] = 1;
		temp_res = dangling_phone_1_mis(pca_d, 4, 4, 1);
		if(temp_res != 2)
		{
			//My_Method_Num = 13;
			return temp_res;
		}
#else
		temp_res = dangling_phone_1_mis(pca_d, 4, 4, 1);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 3

	// Method 4					Dangling_2()
	// Added May 29th, 2012: deal with the light dangling
	for(i=1; i<pca_d->vertical.peaks.num[1]; i++)
	{
		if((pca_d->vertical.peaks.high_ind[i] - pca_d->vertical.peaks.high_ind[i-1]) <= 6)
			flag_h = 0;
	}

	if((pca_d->motion.peaks.num[1]+pca_d->motion.peaks.num[0]) <= 4 && pca->motion_parametrs[1] > 5 && pca->vertical_parametrs[1] > 5 && (pca->vertical_parametrs[1]/pca->motion_parametrs[1]) > 0.5 && flag_h)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[14] = 1;
		temp_res = dangling_phone_1_mis(pca_d, 2, 2, 2);
		if(temp_res != 2)
		{
			//My_Method_Num = 14;
			return temp_res;
		}
#else
		temp_res = dangling_phone_1_mis(pca_d, 2, 2, 2);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 4

	// Method 5            Dangling_3()
	for(i=0; i<pca_d->vertical.peaks.num[0]; i++)
	{
		if(pca_d->vertical.peaks.low_val[i] > max_vl_val)
			max_vl_val = pca_d->vertical.peaks.low_val[i];
	}
	for(i=0; i<pca_d->vertical.peaks.num[1]; i++)
	{
		if(pca_d->vertical.peaks.high_val[i] < min_vh_val)
			min_vh_val = pca_d->vertical.peaks.high_val[i];
	}
	diff_v = fabs(min_vh_val - max_vl_val);

	if(fabs(pca->roll_parametrs[0]*R2Df) > 60 && fabs(pca->roll_parametrs[0]*R2Df) < 120 && pca->motion_parametrs[1] < 5 && MIN(pca->roll_parametrs[2], pca->pitch_parametrs[2]) < 10 && diff_v > 1.5 && flag_h && (pca->vertical_parametrs[2]/pca->motion_parametrs[2]) < 0.8 && pca->motion_parametrs[2] < 16 && pca->motion_parametrs[2] > 9 && pca->vertical_parametrs[2] < 10 && (pca_d->motion.peaks.num[1]+pca_d->motion.peaks.num[0]) <= 3 && (pca_d->vertical.peaks.num[1]+pca_d->vertical.peaks.num[0]) >= 4)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[15] = 1;
		temp_res = dangling_phone_2_mis(pca_d, pca);
		if(temp_res != 2)
		{
			//My_Method_Num = 15;
			return temp_res;
		}
#else
		temp_res = dangling_phone_2_mis(pca_d, pca);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 5

	// Method 6			Dangling_4()
	// Developed Sep. 7, 2013
	if(pca->horizontal_vertical_flag != 0 && (pca_d->motion.peaks.num[1]+pca_d->motion.peaks.num[0]) <= 3 && (pca_d->vertical.peaks.num[1]+pca_d->vertical.peaks.num[0]) >= 4 && pca->motion_parametrs[2] < 10 && (pca->vertical_parametrs[2]/pca->motion_parametrs[2]) > 0.7 && (pca->vertical_parametrs[2]/pca->motion_parametrs[2]) < 1.2)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[16] = 1;
		temp_res = dangling_phone_3_mis(pca_d);
		if(temp_res != 2)
		{
			//My_Method_Num = 16;
			return temp_res;
		}
#else
		temp_res = dangling_phone_3_mis(pca_d);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 6

	// Method 7				Dangling_5_7()
#ifdef METHOD_RESULTS_MISALIGNMENT
	//My_Method_Worked[17] = 1;
	temp_res = dangling_phone_4_mis(pca_d, 3, 3, 5);
	if(temp_res != 2)
	{
		//My_Method_Num = 17;
		return temp_res;
	}
#else
	temp_res = dangling_phone_4_mis(pca_d, 3, 3, 5);
	if(temp_res != 2)
		return temp_res;
#endif
	// End of Method 7

#ifndef COM_WALK_SIMPLIFIED
	// Method 8
	if(pca_m->height_change_flag == 0 && fabs(pca_m->vertical_motion_phase- 90) > 5 && fabs(pca_m->motion_effective_coefficient) > 0.01)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[18] = 1;
		if(pca_m->motion_effective_coefficient > 0)
		{
			//My_Method_Num = 18;
			return 1;
		}
		else
		{
			//My_Method_Num = 18;
			return 0;
		}
#else
		if(pca_m->motion_effective_coefficient > 0)
			return 1;
		else
			return 0;
#endif
	}
	// End of Method 8
#endif

#ifndef COM_WALK_SIMPLIFIED
	// Method 9				Dangling_6()
#ifdef METHOD_RESULTS_MISALIGNMENT
	//My_Method_Worked[19] = 1;
	temp_res = dangling_phone_5_mis(pca_d, pca);
	if(temp_res != 2)
	{
		//My_Method_Num = 19;
		return temp_res;
	}
#else
	temp_res = dangling_phone_5_mis(pca_d, pca);
	if(temp_res != 2)
		return temp_res;
#endif
	// End of Method 9
#endif
	// Method 10				Dangling_5_7()
#ifdef METHOD_RESULTS_MISALIGNMENT
	//My_Method_Worked[20] = 1;
	temp_res = dangling_phone_4_mis(pca_d, 2, 3, 7);
	if(temp_res != 2)
	{
		//My_Method_Num = 20;
		return temp_res;
	}
#else
	temp_res = dangling_phone_4_mis(pca_d, 2, 3, 7);
	if(temp_res != 2)
		return temp_res;
#endif
	// End of Method 10

#ifndef COM_WALK_SIMPLIFIED
	// Method 11			Dangling_8()
#ifdef METHOD_RESULTS_MISALIGNMENT
	//My_Method_Worked[21] = 1;
	temp_res = dangling_phone_6_mis(pca_d);
	if(temp_res != 2)
	{
		//My_Method_Num = 21;
		return temp_res;
	}
#else
	temp_res = dangling_phone_6_mis(pca_d);
	if(temp_res != 2)
		return temp_res;
#endif
	// End of Method 11
#endif

	// Method 12				Dangling_9()
#ifdef METHOD_RESULTS_MISALIGNMENT
	//My_Method_Worked[22] = 1;
	temp_res = dangling_phone_7_mis(pca_d, pca);
	if(temp_res != 2)
	{
		//My_Method_Num = 22;
		return temp_res;
	}
#else
	temp_res = dangling_phone_7_mis(pca_d, pca);
	if(temp_res != 2)
		return temp_res;
#endif
	// End of Method 12
#ifdef METHOD_RESULTS_MISALIGNMENT
	//My_Method_Worked[10] = 1;
	//My_Method_Num = 10;
#endif
	return 2;
}
/**
* @brief	This is phone dangling method 1 to solve the 180 amiguity problem in case of phone dangling data.
*
* @details	This function takes a pointer of structure of type PCA_D. It uses the relation between the
* 			vertical and motion peaks to provide the decision of Forward/Backward.
*
* @param[in]	pca_d		pointer to a structure of type \c PCA_D.
* @param[in]	bef_val   	variable for number of peaks before the operating peak.
* @param[in]	aft_val   	variable for number of peaks after the operating peak.
* @param[in]	Method_Num	variable for the method number.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_phone_1_mis(PCA_D_Ptr pca_d, UINT8 bef_val, UINT8 aft_val, UINT8 Method_Num)
{
	INT8 i, j;
	INT8 st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;

	for(i=0; i<pca_d->vertical.peaks.num[1]; i++)
	{
		st_ind = MAX(pca_d->vertical.peaks.high_ind[i] - bef_val, 0);
		en_ind = MIN(pca_d->vertical.peaks.high_ind[i] + aft_val, DATA_SIZE_MIS-1);

		if((pca_d->vertical.peaks.high_val[i] > pca_d->vertical.thresholds[3] && Method_Num == 1) ||
			(pca_d->vertical.peaks.high_val[i] > pca_d->vertical.thresholds[1] && Method_Num == 2))
		{
			for(j=0; j<pca_d->motion.peaks.num[1]; j++)
			{
				if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind && ((pca_d->motion.peaks.high_val[j] > pca_d->motion.thresholds[1] && Method_Num == 2) || Method_Num == 1))
					step_b++;
			}
			for(j=0; j<pca_d->motion.peaks.num[0]; j++)
			{
				if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind && ((pca_d->motion.peaks.low_val[j] < pca_d->motion.thresholds[0] && Method_Num == 2) || Method_Num == 1))
					step_f++;
			}
		}
		else if((Method_Num == 2 && pca_d->vertical.peaks.high_val[i] > 0 && pca_d->vertical.peaks.high_val[i] <= pca_d->vertical.thresholds[1]) || Method_Num == 1)
		{
			for(j=0; j<pca_d->motion.peaks.num[1]; j++)
			{
				if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind && ((pca_d->motion.peaks.high_val[j] > pca_d->motion.thresholds[1] && Method_Num == 2) || Method_Num == 1))
					step_f++;
			}
			for(j=0; j<pca_d->motion.peaks.num[0]; j++)
			{
				if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind && ((pca_d->motion.peaks.low_val[j] < pca_d->motion.thresholds[0] && Method_Num == 2) || Method_Num == 1))
					step_b++;
			}
		}
	}

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is phone dangling method 2 to solve the 180 amiguity problem in case of phone dangling data.
*
* @details	This function takes 2 pointers of structures of type PCA_D and PCA. It uses the relation between the
* 			vertical and motion peaks and motion data slopes to provide the decision of Forward/Backward.
*
* @param[in]	pca_d	pointer to a structure of type \c PCA_D.
* @param[in]	pca		pointer to a structure of type \c PCA.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_phone_2_mis(PCA_D_Ptr pca_d, PCA_Ptr pca)
{
	INT8 i, j;
	INT8 min_vl_ind = -1;
	INT8 st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;

	FLOAT32 range_vl = 0;
	FLOAT32 max_vl_peaks = -99.0, min_vl_peaks = 99.0;
	FLOAT32 slope_m = 0.0;

	for(i=0; i< pca_d->vertical.peaks.num[0]; i++)
	{
		if(pca_d->vertical.peaks.low_val[i] > max_vl_peaks)
			max_vl_peaks = pca_d->vertical.peaks.low_val[i];

		if(pca_d->vertical.peaks.low_val[i] < min_vl_peaks)
		{
			min_vl_peaks = pca_d->vertical.peaks.low_val[i];
			min_vl_ind = i;
		}
	}
	range_vl = fabs(max_vl_peaks - min_vl_peaks);

	// 3-1
	if(range_vl > 1)
	{
		st_ind = MAX(pca_d->vertical.peaks.low_ind[min_vl_ind]-2, 0);
		en_ind = MIN(pca_d->vertical.peaks.low_ind[min_vl_ind]+2, DATA_SIZE_MIS-1);

		for(j=0; j<pca_d->motion.peaks.num[1]; j++)
		{
			if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
				step_f++;
		}
		for(j=0; j<pca_d->motion.peaks.num[0]; j++)
		{
			if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
				step_b++;
		}
	}
	// End of 3-1

	// 3-2
	if(step_f == step_b)
	{
		for(i=0; i<pca_d->vertical.peaks.num[1]; i++)
		{
			for(j=0; j<pca_d->vertical.peaks.num[0]; j++)
			{
				if(abs(pca_d->vertical.peaks.high_ind[i] - pca_d->vertical.peaks.low_ind[j]) <= 4 &&
					pca_d->vertical.peaks.low_ind[j] < pca_d->vertical.peaks.high_ind[i])
				{
					slope_m = pca->data_m[pca_d->vertical.peaks.high_ind[i]-1] - pca->data_m[pca_d->vertical.peaks.low_ind[j]-1];
					if(slope_m >= 0)
						step_f++;
					else
						step_b++;
				}
			}
		}
	}
	// End of 3-2

	// 3-3
	if(step_f == step_b)
	{
		for(i=0; i<pca_d->vertical.peaks.num[1]; i++)
		{
			st_ind = MAX(pca_d->vertical.peaks.high_ind[i]-2, 0);
			en_ind = MIN(pca_d->vertical.peaks.high_ind[i]+2, DATA_SIZE_MIS-1);

			for(j=0; j<pca_d->motion.peaks.num[1]; j++)
			{
				if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
					step_b++;
			}
			for(j=0; j<pca_d->motion.peaks.num[0]; j++)
			{
				if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
					step_f++;
			}
		}
	}
	// End of 3-3

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is phone dangling method 3 to solve the 180 amiguity problem in case of phone dangling data.
*
* @details	This function takes a pointer of structure of type PCA_D. It uses the relation between the
* 			vertical and motion peaks to provide the decision of Forward/Backward.
*
* @param[in]	pca_d	pointer to a structure of type \c PCA_D.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_phone_3_mis(PCA_D_Ptr pca_d)
{
	INT8 i, j;
	INT8 max_vl_ind = -1, min_vh_ind = -1;
	INT8 st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;

	FLOAT32 max_vl_val = -99.0, min_vh_val = 99.0, a = 99.0;

	// 4-1
	for(i = 0; i < pca_d->vertical.peaks.num[1]; i++)
	{
		if(pca_d->vertical.peaks.high_val[i] < a)
		{
			a = pca_d->vertical.peaks.high_val[i];
			min_vh_ind = i;
		}
	}
	if(pca_d->vertical.peaks.high_ind[min_vh_ind] > 26) //too close to the end, will have not enough motion peaks after
	{
		min_vh_ind = -1;
		for(i = 0; i < pca_d->vertical.peaks.num[1]; i++)
		{
			if(pca_d->vertical.peaks.high_val[i] < min_vh_val && pca_d->vertical.peaks.high_val[i] != a)
			{
				min_vh_val = pca_d->vertical.peaks.high_val[i];
				min_vh_ind = i;
			}
		}
	}

	st_ind = MAX(pca_d->vertical.peaks.high_ind[min_vh_ind], 0);
	en_ind = MIN(pca_d->vertical.peaks.high_ind[min_vh_ind]+2, DATA_SIZE_MIS-1);

	for(j=0; j<pca_d->motion.peaks.num[1]; j++)
	{
		if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
			step_f++;
	}
	for(j=0; j<pca_d->motion.peaks.num[0]; j++)
	{
		if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
			step_b++;
	}
	// End of 4-1

	// 4-2
	if(step_f == step_b)
	{
		for(i = 0; i < pca_d->vertical.peaks.num[0]; i++)
		{
			if(pca_d->vertical.peaks.low_val[i] > max_vl_val)
			{
				max_vl_val = pca_d->vertical.peaks.low_val[i];
				max_vl_ind = i;
			}
		}

		st_ind = MAX(pca_d->vertical.peaks.low_ind[max_vl_ind], 0);
		en_ind = MIN(pca_d->vertical.peaks.low_ind[max_vl_ind]+1, DATA_SIZE_MIS-1);

		for(j=0; j<pca_d->motion.peaks.num[1]; j++)
		{
			if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
				step_b++;
		}
		for(j=0; j<pca_d->motion.peaks.num[0]; j++)
		{
			if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
				step_f++;
		}
	}
	// End of 4-2

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is phone dangling method 4 to solve the 180 amiguity problem in case of phone dangling data.
*
* @details	This function takes a pointer of structure of type PCA_D. It uses the relation between the
* 			vertical and motion peaks to provide the decision of Forward/Backward.
*
* @param[in]	pca_d		pointer to a structure of type \c PCA_D.
* @param[in]	bef_val   	variable for number of peaks before the operating peak.
* @param[in]	aft_val   	variable for number of peaks after the operating peak.
* @param[in]	Method_Num	variable for the method number.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_phone_4_mis(PCA_D_Ptr pca_d, UINT8 bef_val, UINT8 aft_val, UINT8 Method_Num)
{
	INT8 i, j;
	INT8 st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;

	// check  for  the vertical low peaks
	for(i=0; i<pca_d->vertical.peaks.num[0]; i++)
	{
		if((Method_Num == 5 && pca_d->vertical.peaks.low_val[i]  < pca_d->vertical.thresholds[0]) || ((Method_Num == 7 && pca_d->vertical.peaks.low_val[i] <= pca_d->vertical.thresholds[0])))
		{
			st_ind = MAX(pca_d->vertical.peaks.low_ind[i] - bef_val, 0);
			en_ind = MIN(pca_d->vertical.peaks.low_ind[i] + aft_val, DATA_SIZE_MIS-1);

			for(j=0; j<pca_d->motion.peaks.num[1]; j++)
			{
				if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind && ((Method_Num == 5 && pca_d->motion.peaks.high_val[j] > pca_d->motion.thresholds[1]) || Method_Num == 7))
					step_f++;
			}
			for(j=0; j<pca_d->motion.peaks.num[0]; j++)
			{
				if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind && ((Method_Num == 5 && pca_d->motion.peaks.low_val[j] < pca_d->motion.thresholds[0]) || Method_Num == 7))
					step_b++;
			}
		}
	}

	// check for the vertical high peaks
	for(i=0; i<pca_d->vertical.peaks.num[1]; i++)
	{
		if((Method_Num == 5 && pca_d->vertical.peaks.high_val[i] >  pca_d->vertical.thresholds[1]) || ((Method_Num == 7 && pca_d->vertical.peaks.high_val[i] >= pca_d->vertical.thresholds[1])))
		{
			st_ind = MAX(pca_d->vertical.peaks.high_ind[i] - bef_val, 0);
			en_ind = MIN(pca_d->vertical.peaks.high_ind[i] + aft_val, DATA_SIZE_MIS-1);

			for(j=0; j<pca_d->motion.peaks.num[1]; j++)
			{
				if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind && ((Method_Num == 5 && pca_d->motion.peaks.high_val[j] > pca_d->motion.thresholds[1]) || Method_Num == 7))
					step_b++;
			}
			for(j=0; j<pca_d->motion.peaks.num[0]; j++)
			{
				if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind && ((Method_Num == 5 && pca_d->motion.peaks.low_val[j] < pca_d->motion.thresholds[0]) || Method_Num == 7))
					step_f++;
			}
		}
	}

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is phone dangling method 5 to solve the 180 amiguity problem in case of phone dangling data.
*
* @details	This function takes 2 pointers of structures of type PCA_D and PCA. It uses the relation between the
* 			vertical and motion peaks to provide the decision of Forward/Backward.
*
* @param[in]	pca_d	pointer to a structure of type \c PCA_D.
* @param[in]	pca		pointer to a structure of type \c PCA.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_phone_5_mis(PCA_D_Ptr pca_d, PCA_Ptr pca)
{
	INT8 i, j;
	INT8 st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;

	if(pca->vertical_parametrs[2] >= 0.8*pca->motion_parametrs[2])
	{
		for(i=0; i<pca_d->vertical.peaks.num[1]; i++)
		{
			if(pca_d->vertical.peaks.high_val[i] >= 0.0 && pca_d->vertical.peaks.high_val[i] <= pca_d->vertical.thresholds[1])
			{
				st_ind = MAX(pca_d->vertical.peaks.high_ind[i] - 2, 0);
				en_ind = MIN(pca_d->vertical.peaks.high_ind[i] + 2, DATA_SIZE_MIS-1);

				for(j=0; j<pca_d->motion.peaks.num[1]; j++)
				{
					if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
						step_f++;
				}
				for(j=0; j<pca_d->motion.peaks.num[0]; j++)
				{
					if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
						step_b++;
				}
			}
		}
	}

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is phone dangling method 6 to solve the 180 amiguity problem in case of phone dangling data.
*
* @details	This function takes a pointer of structure of type PCA_D. It uses the relation between the
* 			vertical and motion peaks to provide the decision of Forward/Backward. it use only the paris
* 			of close peaks not all peaks.
*
* @param[in]	pca_d	pointer to a structure of type \c PCA_D.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_phone_6_mis(PCA_D_Ptr pca_d)
{
	INT8 i, j, k;
	INT8 st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;
	INT8 vh_close[PEAKS_VECTOR_SIZE_MIS][2] = {-1};

	k = 0;
	for(i=1; i<pca_d->vertical.peaks.num[1]; i++)
	{
		if((pca_d->vertical.peaks.high_ind[i]-pca_d->vertical.peaks.high_ind[i-1] <= 11) &&	pca_d->vertical.peaks.high_val[i] > pca_d->vertical.thresholds[1] && pca_d->vertical.peaks.high_val[i-1] > pca_d->vertical.thresholds[1])
		{
			for(j=0; j<pca_d->vertical.peaks.num[0]; j++)
			{
				if(pca_d->vertical.peaks.low_ind[j] >= pca_d->vertical.peaks.high_ind[i-1] && pca_d->vertical.peaks.low_ind[j] <= pca_d->vertical.peaks.high_ind[i])
				{
					vh_close[k][0] = i-1;
					vh_close[k][1] = j;
					k++;
				}
			}
		}
	}
	for(i=0; i<k; i++)
	{
		st_ind = MAX(pca_d->vertical.peaks.high_ind[vh_close[i][0]]-2, 0);
		en_ind = MIN(pca_d->vertical.peaks.high_ind[vh_close[i][0]]+3, DATA_SIZE_MIS-1);

		for(j=0; j<pca_d->motion.peaks.num[1]; j++)
		{
			if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
				step_b++;
		}
		for(j=0; j<pca_d->motion.peaks.num[0]; j++)
		{
			if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
				step_f++;
		}
	}

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is phone dangling method 7 to solve the 180 amiguity problem in case of phone dangling data.
*
* @details	This function takes 2 pointers of structures of type PCA_D and PCA. It uses the sign of the slope of motion
* 			data at the minimum vertical low peak to provide the decision of Forward/Backward.
*
* @param[in]	pca_d	pointer to a structure of type \c PCA_D.
* @param[in]	pca		pointer to a structure of type \c PCA.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_phone_7_mis(PCA_D_Ptr pca_d, PCA_Ptr pca)
{
	INT8 i, j;
	INT8 count1 = 0, temp_ind = -1;
	INT8 XI1 = 0, XI2 = 0;
	INT8 low_true = 0;
	INT8 st_ind = -1, en_ind = -1;
	INT8 k_index[PEAKS_VECTOR_SIZE_MIS] = {-1};

	FLOAT32 temp_val = 0.0;
	FLOAT32 max_f = 0.0;
	FLOAT32 slope_m = 0.0;

	INT8 XI_ind[PEAKS_VECTOR_SIZE_MIS] = {-1};
	FLOAT32 XI_val[PEAKS_VECTOR_SIZE_MIS] = {0.0};

	// Vertical Peaks Detection
	i = peaks_detection_mis(pca->data_v, pca->vertical_parametrs[1], 4, &pca_d->vertical.peaks, 20, 36, 0);
	if(pca_d->vertical.peaks.num[0] == 0.0 || pca_d->vertical.peaks.num[1] == 0.0)
		i = peaks_detection_mis(pca->data_v, pca->vertical_parametrs[1]/3, 4, &pca_d->vertical.peaks, 20, 36, 0);

	// Remove all high peaks before the first low peak
	if(pca_d->vertical.peaks.num[1] > 1)
	{
		count1 = 0;
		for(i = 0; i < pca_d->vertical.peaks.num[1]; i++)
		{
			if(pca_d->vertical.peaks.high_ind[i] < pca_d->vertical.peaks.low_ind[0])
				count1++;
		}
		if(count1 > 0)
		{
			j = 0;
			for(i = count1; i < pca_d->vertical.peaks.num[1]; i++)
			{
				pca_d->vertical.peaks.high_ind[j] = pca_d->vertical.peaks.high_ind[i];
				pca_d->vertical.peaks.high_val[j] = pca_d->vertical.peaks.high_val[i];
				j++;
			}
		}
		pca_d->vertical.peaks.num[1] = pca_d->vertical.peaks.num[1]-count1;
	}

	// Remove all low peaks after the last high peak

	if(pca_d->vertical.peaks.num[0] > 1)
	{
		count1 = 0;
		for(i = 0; i < pca_d->vertical.peaks.num[0]; i++)
		{
			if(pca_d->vertical.peaks.low_ind[i] > pca_d->vertical.peaks.high_ind[pca_d->vertical.peaks.num[1]-1])
				count1++;
		}
		pca_d->vertical.peaks.num[0] = pca_d->vertical.peaks.num[0] - count1;
	}

	if(pca_d->vertical.peaks.num[0] == 0 || pca_d->vertical.peaks.num[1] == 0)
		return 2;

	for(i=0; i< pca_d->vertical.peaks.num[0]; i++)
	{
		XI_ind[i] = i;
		XI_val[i] = pca_d->vertical.peaks.low_val[i];
	}

	// sort the peak values to get the minimum peak at the beginning
	for(i=0; i<pca_d->vertical.peaks.num[0]-1; i++)
	{
		for(j=0; j <(pca_d->vertical.peaks.num[0]-i-1); j++)
		{
			if(XI_val[j] > XI_val[j+1])
			{
				temp_ind    = XI_ind[j];
				temp_val    = XI_val[j];
				XI_ind[j]   = XI_ind[j+1];
				XI_val[j]   = XI_val[j+1];
				XI_ind[j+1] = temp_ind;
				XI_val[j+1] = temp_val;
			}
		}
	}
	// Ambiguity detection start!!
	low_true = XI_ind[0];
	// Get the maximum high peak
	max_f = pca_d->vertical.peaks.high_val[0];
	for(i=1; i<pca_d->vertical.peaks.num[1]; i++)
	{
		if(pca_d->vertical.peaks.high_val[i] > max_f)
			max_f = pca_d->vertical.peaks.high_val[i];
	}
	// get the closest high peak
	XI1 = XI_ind[0];
	for(i = 0; i< pca_d->vertical.peaks.num[1]; i++)
	{
		if((pca_d->vertical.peaks.high_ind[i] - pca_d->vertical.peaks.low_ind[XI1]) > 0)
		{	k_index[0] = i;	break;	}
	}

	if(pca_d->vertical.peaks.num[0] > 1)
	{
		XI2 = XI_ind[1];
		if(pca_d->vertical.peaks.high_val[k_index[0]] == max_f)
			low_true = XI_ind[0];     // lowest low peak has highest high peak
		else
		{
			// lowest low peak doesn't has highest high peak
			if(fabs(pca_d->vertical.peaks.low_val[XI1] - pca_d->vertical.peaks.low_val[XI2]) < 0.095)  // two low peaks are too close in value
				low_true = XI_ind[1];
		}
	}
	else
		low_true = XI_ind[0];      // only one low peak

	st_ind = pca_d->vertical.peaks.low_ind[low_true];
	en_ind = MIN(pca_d->vertical.peaks.low_ind[low_true] + 3, DATA_SIZE_MIS-1);
	slope_m = pca->data_m[en_ind] - pca->data_m[st_ind];
	// Changed: removing the part to detect a  motion low peak close to the vertical low peak
	// for some reaseon the motion data is inverted and the low becmoes high and high becomes low which casues a problem in dangling detection
	if(slope_m > 0)
		return 1;
	else if(slope_m < 0.0)
		return 0;
	else
		return 2;
}
/**
* @brief	This is table and watch dangling backward forward function to solve the 180 amiguity problem in
* 			case of tablet or watch dangling data.
*
* @details	This function takes 3 pointers of structures of type PCA, PCA_M, and PCA_D. it calls different
* 			sub-functions to get the decision of Forward/Backward in the case of tablet or watch dangling data.
*
* @param[in]	pca		pointer to a structure of type \c PCA.
* @param[in]	pca_m  	pointer to a structure of type \c PCA_M.
* @param[in]	pca_d  	pointer to a structure of type \c PCA_D.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*
* @see dangling_tablet_watch_1_mis(), dangling_tablet_watch_2_mis(), ....
*/
INT8 dangling_tablet_watch_backward_forward_mis(PCA_Ptr pca, PCA_M_Ptr pca_m, PCA_D_Ptr pca_d)
{
	INT8 i, j;
	INT8 temp_res = 2;
	INT8 close_flage_counter = 0, min_vh_vl = 0;
	UINT8 flag_h = 1;
	UINT8 close_flage = 0;

	FLOAT32 diff_v = 0.0, diff_v2 = 0.0;
	FLOAT32 diff_min_vh_vl = 0, diff_max_vh_vl = 0;
	FLOAT32 ratio_v = 0;
	FLOAT32 range_vl = 0, range_vh = 0, range_mh_vh = 0, range_ml_vl = 0;
	FLOAT32 diff_max_mh_vh = 0, diff_min_ml_vl = 0;
	FLOAT32 min_vh = 99.0, max_vh = -99.0, min_vl = 99.0, max_vl = -99.0;
	FLOAT32 min_mh = 99.0, max_mh = -99.0, min_ml = 99.0, max_ml = -99.0;
	FLOAT32 min_vh_val = 99.0, max_vl_val = -99.0;

	for(i = 0; i < pca_d->vertical.peaks.num[1]; i++)
	{
		if(pca_d->vertical.peaks.high_val[i] < min_vh)
			min_vh = pca_d->vertical.peaks.high_val[i];
		if(pca_d->vertical.peaks.high_val[i] > max_vh)
			max_vh = pca_d->vertical.peaks.high_val[i];
	}

	for(i = 0; i < pca_d->vertical.peaks.num[0]; i++)
	{
		if(pca_d->vertical.peaks.low_val[i] < min_vl)
			min_vl = pca_d->vertical.peaks.low_val[i];
		if(pca_d->vertical.peaks.low_val[i] > max_vl)
			max_vl = pca_d->vertical.peaks.low_val[i];
	}

	for(i = 0; i < pca_d->motion.peaks.num[1]; i++)
	{
		if(pca_d->motion.peaks.high_val[i] < min_mh)
			min_mh = pca_d->motion.peaks.high_val[i];
		if(pca_d->motion.peaks.high_val[i] > max_mh)
			max_mh = pca_d->motion.peaks.high_val[i];
	}

	for(i = 0; i < pca_d->motion.peaks.num[0]; i++)
	{
		if(pca_d->motion.peaks.low_val[i] < min_ml)
			min_ml = pca_d->motion.peaks.low_val[i];
		if(pca_d->motion.peaks.low_val[i] > max_ml)
			max_ml = pca_d->motion.peaks.low_val[i];
	}

	diff_min_vh_vl = fabs(min_vh - max_vl);
	diff_max_vh_vl = fabs(max_vh - min_vl);

	diff_v2 = fabs(diff_min_vh_vl - diff_max_vh_vl);
	ratio_v = fabs(diff_min_vh_vl/diff_max_vh_vl);

	range_vl = fabs(max_vl - min_vl);
	range_vh = fabs(max_vh - min_vh);

	range_mh_vh = fabs(max_mh - min_vh);
	range_ml_vl = fabs(min_ml - max_vl);

	diff_max_mh_vh = fabs(max_mh - max_vh);
	diff_min_ml_vl = fabs(min_ml - min_vl);

# ifndef COM_WALK_SIMPLIFIED
	// Method 1
	if(pca_m->device_flag == 3 && pca_m->height_change_flag == 0)
	{
		if(MAX(pca->roll_parametrs[2], pca->pitch_parametrs[2]) > 40 && MAX(pca->roll_parametrs[2], pca->pitch_parametrs[2]) < 80) // normal dangling
		{
#ifdef METHOD_RESULTS_MISALIGNMENT
			//My_Method_Worked[51] = 1;
			if(pca_m->motion_effective_coefficient > 0.1)
			{
				//My_Method_Num = 51;
				return 1;
			}
			else if(pca_m->motion_effective_coefficient < -0.1)
			{
				//My_Method_Num = 51;
				return 0;
			}
#else
			if(pca_m->motion_effective_coefficient > 0.1)
				return 1;
			else if(pca_m->motion_effective_coefficient < -0.1)
				return 0;
#endif
		}
		else if(MAX(pca->roll_parametrs[2], pca->pitch_parametrs[2]) < 40)  // light dangling
		{
#ifdef METHOD_RESULTS_MISALIGNMENT
			//My_Method_Worked[51] = 1;
			if(fabs(pca_m->vertical_motion_phase-90) > 10)
			{
				if(pca_m->motion_effective_coefficient > 0)
				{
					//My_Method_Num = 51;
					return 1;
				}
				else
				{
					//My_Method_Num = 51;
					return 0;
				}
			}
#else
			if(fabs(pca_m->vertical_motion_phase-90) > 10)
			{
				if(pca_m->motion_effective_coefficient > 0)
					return 1;
				else
					return 0;
			}
#endif
		}
	}
	// End of Method 1

	// Method 2
	// Developed Oct 1st, 2013
	for(i=1; i<pca_d->vertical.peaks.num[1]; i++)
	{
		if((pca_d->vertical.peaks.high_ind[i] - pca_d->vertical.peaks.high_ind[i-1]) <= 6)
		{
			for(j=0; j<pca_d->vertical.peaks.num[0]; j++)
			{
				if(pca_d->vertical.peaks.low_ind[j] >= pca_d->vertical.peaks.high_ind[i-1] && pca_d->vertical.peaks.low_ind[j] <= pca_d->vertical.peaks.high_ind[i])
					close_flage = 1;
			}
		}
	}
	if(pca->horizontal_vertical_flag != 0 && !close_flage && (pca_d->vertical.peaks.num[1] + pca_d->vertical.peaks.num[0]) < 7 && pca->vertical_parametrs[1] < 1.4 && pca_m->horizontal_acceleration_range > 4)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[52] = 1;
		temp_res = dangling_tablet_watch_13_mis(pca_d, pca);
		if(temp_res != 2)
		{
			//My_Method_Num = 52;
			return temp_res;
		}
#else
		temp_res = dangling_tablet_watch_13_mis(pca_d, pca);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 2

	// Method 3
	if(pca_m->device_flag == 1 && fabs(pca_m->vertical_motion_phase_mean-90)>18 && fabs(pca_m->motion_effective_coefficient_mean) > 0.05)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[53] = 1;
		if(pca_m->motion_effective_coefficient_mean>0)
		{
			//My_Method_Num = 53;
			return 1;
		}
		else
		{
			//My_Method_Num = 53;
			return 0;
		}
#else
		if(pca_m->motion_effective_coefficient_mean>0)
			return 1;
		else
			return 0;
#endif
	}
	//     End of Method 3
# endif

	// Method 4
	if(pca->motion_parametrs[2] > 6 && pca->motion_parametrs[2] > pca->vertical_parametrs[2] && pca->roll_parametrs[1] < 0.4 && pca->pitch_parametrs[1] < 0.4 && (pca_d->vertical.peaks.num[1]+pca_d->vertical.peaks.num[0]) >= 5 && (pca_d->motion.peaks.num[1]+pca_d->motion.peaks.num[0]) >= 5)  //relative static data which will belong to purse
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[54] = 1;
		temp_res = xcorr_backward_forward_mis(pca);
		if(temp_res != 2)
		{
			//My_Method_Num = 54;
			return temp_res;
		}
#else
		temp_res = xcorr_backward_forward_mis(pca);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 4

	// Method 5
	// Developed Oct 11th, 2013
	if(pca->horizontal_vertical_flag != 0 && pca->motion_parametrs[2] <= 13 && pca->vertical_parametrs[2] > 4 && (pca_d->vertical.peaks.num[1]+pca_d->vertical.peaks.num[0]) >= 4 && (pca_d->vertical.peaks.num[1]+pca_d->vertical.peaks.num[0]) <= 7) //relative static data which will belong to purse
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[55] = 1;
		temp_res = dangling_tablet_watch_1_mis(pca_d);
		if(temp_res != 2)
		{
			//My_Method_Num = 55;
			return temp_res;
		}
#else
		temp_res = dangling_tablet_watch_1_mis(pca_d);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 5

	// Method 6
	// Developed Oct 8th, 2013
	if(pca->motion_parametrs[2] < 10 && pca->vertical_parametrs[2] < 6 && (pca_d->vertical.peaks.num[0] + pca_d->vertical.peaks.num[1]) > 5 && diff_v2 < 2 && (MAX(pca->roll_parametrs[2], pca->pitch_parametrs[2]) > 22 && MAX(pca->roll_parametrs[2], pca->pitch_parametrs[2]) < 27))
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[56] = 1;
		temp_res = dangling_tablet_watch_2_mis(pca_d, range_ml_vl, range_mh_vh);
		if(temp_res != 2)
		{
			//My_Method_Num = 56;
			return temp_res;
		}
#else
		temp_res = dangling_tablet_watch_2_mis(pca_d, range_ml_vl, range_mh_vh);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	//End of Method 6

	// Method 7
	// Developed Oct 8th, 2013
	if(pca->motion_parametrs[2] < 14 && (pca->vertical_parametrs[2] > 5 && pca->vertical_parametrs[2] < 8) && (diff_v2/pca->vertical_parametrs[2]) > 0.5 && pca_m->horizontal_acceleration_range > 4 && pca_m->lateral_range > 2)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[57] = 1;
		temp_res = dangling_tablet_watch_3_mis(pca_d);
		if(temp_res != 2)
		{
			//My_Method_Num = 57;
			return temp_res;
		}
#else
		temp_res = dangling_tablet_watch_3_mis(pca_d);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 7

	// Method 8
	// Added June 25th, 2012: deal with the unstable vertical signal
	if(pca->motion_parametrs[2] > 20 &&  pca->vertical_parametrs[2] > 10 && pca->motion_parametrs[2] > pca->vertical_parametrs[2] && (pca_d->motion.peaks.num[0] + pca_d->motion.peaks.num[1]) < 4 && (pca_d->vertical.peaks.num[0]+pca_d->vertical.peaks.num[1]) > 5)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[58] = 1;
		temp_res = dangling_tablet_watch_4_mis(pca_d);
		if(temp_res != 2)
		{
			//My_Method_Num = 58;
			return temp_res;
		}
#else
		temp_res = dangling_tablet_watch_4_mis(pca_d);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 8

	// Method 9
	// Added May 29th, 2012: deal with the light dangling
	for(i=1; i<pca_d->vertical.peaks.num[1]; i++)
	{
		if((pca_d->vertical.peaks.high_ind[i] - pca_d->vertical.peaks.high_ind[i-1]) <= 6)
			flag_h = 0;
	}
	if(flag_h && pca->motion_parametrs[1] > 5 && pca->vertical_parametrs[1] > 5 && (pca->vertical_parametrs[1]/pca->motion_parametrs[1]) > 0.5 && (pca_d->motion.peaks.num[0] + pca_d->motion.peaks.num[1]) <= 4)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[59] = 1;
		temp_res = dangling_tablet_watch_5_mis(pca_d);
		if(temp_res != 2)
		{
			//My_Method_Num = 59;
			return temp_res;
		}
#else
		temp_res = dangling_tablet_watch_5_mis(pca_d);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 9

	// Method 10
	// Developed Oct 9th, 2013
	if(pca->horizontal_vertical_flag == 0 && (pca->motion_parametrs[2] > 9 && pca->motion_parametrs[2] < 15) && pca->vertical_parametrs[2] < 10 && (pca->vertical_parametrs[2]/pca->motion_parametrs[2]) > 0.6 && (diff_min_vh_vl/pca->vertical_parametrs[2]) > 0.7 && (pca_d->motion.peaks.num[0] + pca_d->motion.peaks.num[1]) <= 3 && (pca_d->vertical.peaks.num[0] + pca_d->vertical.peaks.num[1]) >= 4 && pca_m->horizontal_acceleration_range > 4 && pca_m->lateral_range > 2 && MAX(pca->roll_parametrs[2], pca->pitch_parametrs[2]) < 35.0)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[60] = 1;
		temp_res = dangling_tablet_watch_6_mis(pca_d, pca);
		if(temp_res != 2)
		{
			//My_Method_Num = 60;
			return temp_res;
		}
#else
		temp_res = dangling_tablet_watch_6_mis(pca_d, pca);
		if(temp_res != 2)

			return temp_res;
#endif
	}
	// End of Method 10

	// Method 11
	for(i=0; i<pca_d->vertical.peaks.num[0]; i++)
	{
		if(pca_d->vertical.peaks.low_val[i] > max_vl_val)
			max_vl_val = pca_d->vertical.peaks.low_val[i];
	}
	for(i=0; i<pca_d->vertical.peaks.num[1]; i++)
	{
		if(pca_d->vertical.peaks.high_val[i] < min_vh_val)
			min_vh_val = pca_d->vertical.peaks.high_val[i];
	}
	diff_v = fabs(min_vh_val - max_vl_val);

	if(flag_h && pca->motion_parametrs[2] > 9 && pca->motion_parametrs[2] < 16 && pca->vertical_parametrs[2] < 10 && (pca->vertical_parametrs[2]/pca->motion_parametrs[2]) < 0.8 && pca->motion_parametrs[1] < 5 && diff_v > 1.5 && (pca_d->motion.peaks.num[0] + pca_d->motion.peaks.num[1]) <= 3 && (pca_d->vertical.peaks.num[0] + pca_d->vertical.peaks.num[1]) >= 4 && abs(pca->roll_parametrs[0]*R2Df) > 60.0 && abs(pca->roll_parametrs[0]*R2Df) < 120.0 && MIN(pca->roll_parametrs[2], pca->pitch_parametrs[2]) < 10.0)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[61] = 1;
		temp_res = dangling_tablet_watch_7_mis(pca_d, pca, range_vl);
		if(temp_res != 2)
		{
			//My_Method_Num = 61;
			return temp_res;
		}
#else
		temp_res = dangling_tablet_watch_7_mis(pca_d, pca, range_vl);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 11

	// Method 12
	// Developed Sep. 7, 2013, mainly for Tablet vertical dangling
	if(pca->horizontal_vertical_flag != 0 && pca->motion_parametrs[2] < 10 && (pca->vertical_parametrs[2]/pca->motion_parametrs[2]) > 0.7 && (pca->vertical_parametrs[2]/pca->motion_parametrs[2]) < 1.2 && (pca_d->motion.peaks.num[0] + pca_d->motion.peaks.num[1]) <= 3 && (pca_d->vertical.peaks.num[0] + pca_d->vertical.peaks.num[1]) >= 4)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[62] = 1;
		temp_res = dangling_tablet_watch_8_mis(pca_d);
		if(temp_res != 2)
		{
			//My_Method_Num = 62;
			return temp_res;
		}
#else
		temp_res = dangling_tablet_watch_8_mis(pca_d);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 12

	// Method 13
	// Developed Oct 9th, 2013
	if(pca->horizontal_vertical_flag != 0 && pca_m->horizontal_acceleration_range < 7 && (pca->vertical_parametrs[1]/pca->motion_parametrs[1]) < 0.8 && (pca->vertical_parametrs[1]/pca->motion_parametrs[1]) > 0.4)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[63] = 1;
		temp_res = dangling_tablet_watch_9_mis(pca_d, pca);
		if(temp_res != 2)
		{
			//My_Method_Num = 63;
			return temp_res;
		}
#else
		temp_res = dangling_tablet_watch_9_mis(pca_d, pca);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 13

# ifndef COM_WALK_SIMPLIFIED
	// Method 14
	if(pca_m->device_flag == 1 && fabs(pca_m->vertical_motion_phase_mean-90) > 13 && fabs(pca_m->motion_effective_coefficient_mean) > 0.05)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[64] = 1;
		if(pca_m->motion_effective_coefficient_mean > 0)
		{
			//My_Method_Num = 64;
			return 1;
		}
		else
		{
			//My_Method_Num = 64;
			return 0;
		}
#else
		if(pca_m->motion_effective_coefficient_mean > 0)
			return 1;
		else
			return 0;
#endif
	}
	// End of Method 14
# endif

	// Method 15
	// Developed Oct 9th, 2013
	if(pca->horizontal_vertical_flag != 0 && pca->motion_parametrs[2] > 10 && pca->motion_parametrs[2] < 20 && pca->vertical_parametrs[2] > 5 && (pca->vertical_parametrs[2]/pca->motion_parametrs[2]) > 0.5 && (diff_min_vh_vl/pca->vertical_parametrs[2]) > 0.5 && (pca_d->vertical.peaks.num[0] + pca_d->vertical.peaks.num[1]) > 4 && pca_m->horizontal_acceleration_range > 5 && MAX(pca->roll_parametrs[2], pca->pitch_parametrs[2]) < 60.0)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[65] = 1;
		temp_res = dangling_tablet_watch_10_mis(pca_d, range_vl, range_vh);
		if(temp_res != 2)
		{
			//My_Method_Num = 65;
			return temp_res;
		}
#else
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 15

	// Method 16
	// Developed Oct 2nd, 2013
	if(pca->horizontal_vertical_flag != 0 && (pca_d->vertical.peaks.num[1] + pca_d->vertical.peaks.num[0]) <= 7 && pca->vertical_parametrs[2] > 4 && (pca->vertical_parametrs[2]/pca->motion_parametrs[2]) > 0.5 && ((pca->motion_parametrs[2] < 14 && diff_v2 < 3.5) || (pca->motion_parametrs[2] > 14 && (diff_v2 > 3.5 &&  diff_v2 < 4))) && diff_min_vh_vl/pca->vertical_parametrs[2] > 0.5 && MAX(pca->roll_parametrs[2], pca->pitch_parametrs[2]) > 30.0)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[66] = 1;
		temp_res = dangling_tablet_watch_11_mis(pca_d, pca, pca_m);
		if(temp_res != 2)
		{
			//My_Method_Num = 66;
			return temp_res;
		}
#else
		temp_res = dangling_tablet_watch_11_mis(pca_d, pca, pca_m);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 16

	// Method 17
	// Developed Sep 26th, 2013
	min_vh_vl = 99;
	for(i=0; i<pca_d->vertical.peaks.num[1]; i++)
	{
		for(j=0; j<pca_d->vertical.peaks.num[0]; j++)
		{
			if(abs(pca_d->vertical.peaks.high_ind[i] - pca_d->vertical.peaks.low_ind[j]) <= min_vh_vl)
				min_vh_vl = (INT8)(abs(pca_d->vertical.peaks.high_ind[i] - pca_d->vertical.peaks.low_ind[j]));
		}
	}
	for(i=1; i<pca_d->vertical.peaks.num[0]; i++)
	{
		if(abs(pca_d->vertical.peaks.low_ind[i] - pca_d->vertical.peaks.low_ind[i-1]) <= 7)
			close_flage_counter++;
	}
	for(i=1; i<pca_d->vertical.peaks.num[1]; i++)
	{
		if(abs(pca_d->vertical.peaks.high_ind[i] - pca_d->vertical.peaks.high_ind[i-1]) <= 7)
			close_flage_counter++;
	}
	if(pca->horizontal_vertical_flag != 0 && pca->motion_parametrs[2] < 11 && pca->vertical_parametrs[2] < 6 && min_vh_vl < 5 && pca_m->horizontal_acceleration_range > 4 && close_flage_counter < 2 && MIN(pca->roll_parametrs[2], pca->pitch_parametrs[2]) < 8)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[67] = 1;
		temp_res = dangling_tablet_watch_12_mis(pca_d);
		if(temp_res != 2)
		{
			//My_Method_Num = 67;
			return temp_res;
		}
#else
		temp_res = dangling_tablet_watch_12_mis(pca_d);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 17

#ifdef COM_WALK_SIMPLIFIED
	// Method 18
	// Developed Oct 1st, 2013
	for(i=1; i<pca_d->vertical.peaks.num[1]; i++)
	{
		if((pca_d->vertical.peaks.high_ind[i] - pca_d->vertical.peaks.high_ind[i-1]) <= 6)
		{
			for(j=0; j<pca_d->vertical.peaks.num[0]; j++)
			{
				if(pca_d->vertical.peaks.low_ind[j] >= pca_d->vertical.peaks.high_ind[i-1] && pca_d->vertical.peaks.low_ind[j] <= pca_d->vertical.peaks.high_ind[i])
					close_flage = 1;
			}
		}
	}
	if(pca->horizontal_vertical_flag != 0 && !close_flage && (pca_d->vertical.peaks.num[1] + pca_d->vertical.peaks.num[0]) < 7 && pca->vertical_parametrs[1] < 1.4 && pca_m->horizontal_acceleration_range > 4)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[68] = 1;
		temp_res = dangling_tablet_watch_13_mis(pca_d, pca);
		if(temp_res != 2)
		{
			//My_Method_Num = 68;
			return temp_res;
		}
#else
		temp_res = dangling_tablet_watch_13_mis(pca_d, pca);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 18
#endif

	// Method 19
	// Developed Oct 1st, 2013
	if(pca->horizontal_vertical_flag != 0 && !close_flage && (pca->motion_parametrs[2] > 9 && pca->motion_parametrs[2] < 15) && pca->vertical_parametrs[2] < 8 && pca->motion_parametrs[1] < 4.5 && (pca_d->motion.peaks.num[0] + pca_d->motion.peaks.num[1]) > 2)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[69] = 1;
		temp_res = dangling_tablet_watch_14_mis(pca_d, pca);
		if(temp_res != 2)
		{
			//My_Method_Num = 69;
			return temp_res;
		}
#else
		temp_res = dangling_tablet_watch_14_mis(pca_d, pca);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 19

	// Method 20
	// Developed Oct 9th, 2013
	if(pca->horizontal_vertical_flag != 0 && pca->motion_parametrs[2] > 14.0 && pca->motion_parametrs[2] < 20.0 && (pca->vertical_parametrs[2]/pca->motion_parametrs[2]) < 0.6 && pca_m->horizontal_acceleration_range > 7.0 && ((diff_min_vh_vl/pca->vertical_parametrs[2] >= 0.45) || (((diff_min_vh_vl/pca->vertical_parametrs[2]) > 0.3 && (diff_min_vh_vl/pca->vertical_parametrs[2]) < 0.45) && pca_m->lateral_range > 5)) && ((MIN(pca->roll_parametrs[2], pca->pitch_parametrs[2]) > 9.0) || (MIN(pca->roll_parametrs[2], pca->pitch_parametrs[2]) > 6.0 && pca_m->horizontal_acceleration_range > 8.0)))
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[70] = 1;
		temp_res = dangling_tablet_watch_15_mis(pca_d);
		if(temp_res != 2)
		{
			//My_Method_Num = 70;
			return temp_res;
		}
#else
		temp_res = dangling_tablet_watch_15_mis(pca_d);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 20

# ifndef COM_WALK_SIMPLIFIED
	// Method 21
	if(pca_m->device_flag == 1 && fabs(pca_m->vertical_motion_phase_mean-90) > 6.5 && fabs(pca_m->motion_effective_coefficient_mean) > 0.05)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[71] = 1;
		if(pca_m->motion_effective_coefficient_mean > 0)
		{
			//My_Method_Num = 71;
			return 1;
		}
		else
		{
			//My_Method_Num = 71;
			return 0;
		}
#else
		if(pca_m->motion_effective_coefficient_mean > 0)
			return 1;
		else
			return 0;
#endif
	}
	// End of Method 21
# endif

	// Method 22
	// Developed Oct 9th, 2013
	if(pca->horizontal_vertical_flag == 0 && pca->motion_parametrs[2] < 6.0 && pca->vertical_parametrs[2] < 6.0 && pca_m->horizontal_acceleration_range < 2.5 && (diff_min_vh_vl/pca->vertical_parametrs[2]) > 0.7 && pca_m->lateral_range < 2.0 && MAX(pca->roll_parametrs[2], pca->pitch_parametrs[2]) < 20.0)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[72] = 1;
		temp_res = dangling_tablet_watch_16_mis(pca_d);
		if(temp_res != 2)
		{
			//My_Method_Num = 72;
			return temp_res;
		}
#else
		temp_res = dangling_tablet_watch_16_mis(pca_d);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 22

	// Method 23 and 24
	// Developed Oct 8th, 2013
	if(pca->motion_parametrs[2] < 9.0 && pca->vertical_parametrs[2] < 6.0 && diff_v2 < 2.0 && MAX(pca->roll_parametrs[2], pca->pitch_parametrs[2]) < 20.0)
	{
		// Method 23
		if(pca->motion_parametrs[2] < 7.0 && pca->vertical_parametrs[2] < 6.0 && (diff_min_vh_vl/pca->vertical_parametrs[2]) > 0.6 && MIN(pca->roll_parametrs[2], pca->pitch_parametrs[2]) > 6.0)
		{
#ifdef METHOD_RESULTS_MISALIGNMENT
			//My_Method_Worked[73] = 1;
			temp_res = dangling_tablet_watch_17_mis(pca_d);
			if(temp_res != 2)
			{
				//My_Method_Num = 73;
				return temp_res;
			}
#else
			temp_res = dangling_tablet_watch_17_mis(pca_d);
			return temp_res;
#endif
		}
		// End of Method 23

		// Method 24
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[74] = 1;
		temp_res = dangling_tablet_watch_17_2_mis(pca_d);
		if(temp_res != 2)
		{
			//My_Method_Num = 74;
			return temp_res;
		}
#else
		temp_res = dangling_tablet_watch_17_2_mis(pca_d);
		if(temp_res != 2)
			return temp_res;
#endif
		// End of Method 24
	}
	// End of Method 23 and 24

	// Method 25
	// Developed Oct 9th, 2013
	if(pca->motion_parametrs[2] > 15 && pca->motion_parametrs[2] <20 && pca->vertical_parametrs[2] < 6 && (pca->vertical_parametrs[2]/pca->motion_parametrs[2]) < 0.35 && (pca->vertical_parametrs[1]/pca->motion_parametrs[1]) < 0.3 && pca_m->horizontal_acceleration_range > 7 && pca_m->lateral_range < 3 && MAX(pca->roll_parametrs[2], pca->pitch_parametrs[2]) > 45 && MIN(pca->roll_parametrs[2], pca->pitch_parametrs[2]) < 15)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[75] = 1;
		temp_res = dangling_tablet_watch_18_mis(pca_d, pca);
		if(temp_res != 2)
		{
			//My_Method_Num = 75;
			return temp_res;
		}
#else
		temp_res = dangling_tablet_watch_18_mis(pca_d, pca);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 25

	// Method 26
	// Developed Oct 9th, 2013
	if(pca->horizontal_vertical_flag == 0 && pca->motion_parametrs[2] > 15 && pca->motion_parametrs[2] < 20 && pca->vertical_parametrs[2] < 6 && (pca->vertical_parametrs[2]/pca->motion_parametrs[2]) < 0.32 && (pca->vertical_parametrs[1]/pca->motion_parametrs[1]) < 0.3 && (diff_min_vh_vl/pca->vertical_parametrs[2]) > 0.27 && pca_m->horizontal_acceleration_range > 7.5 && pca_m->lateral_range < 3 && MAX(pca->roll_parametrs[2], pca->pitch_parametrs[2]) > 45 && MIN(pca->roll_parametrs[2], pca->pitch_parametrs[2]) < 15)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[76] = 1;
		temp_res = dangling_tablet_watch_19_mis(pca_d);
		if(temp_res != 2)
		{
			//My_Method_Num = 76;
			return temp_res;
		}
#else
		temp_res = dangling_tablet_watch_19_mis(pca_d);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 26

	// Method 27
	// Developed Oct 9th, 2013
	if((pca->vertical_parametrs[2]/pca->motion_parametrs[2]) > 0.5 && (ratio_v > 0.5 && ratio_v < 0.85) && (pca_d->vertical.peaks.num[0] + pca_d->vertical.peaks.num[1]) >= 5)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[77] = 1;
		temp_res = dangling_tablet_watch_20_mis(pca_d, pca, pca_m);
		if(temp_res != 2)
		{
			//My_Method_Num = 77;
			return temp_res;
		}
#else
		temp_res = dangling_tablet_watch_20_mis(pca_d, pca, pca_m);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 27

	// Method 28
	if(pca->horizontal_vertical_flag == 0 && (pca->motion_parametrs[2] > 8 && pca->motion_parametrs[2] < 14) && (pca->vertical_parametrs[2] > 3 && pca->vertical_parametrs[2] < 5) && (pca->vertical_parametrs[2]/pca->motion_parametrs[2]) < 0.5 && ratio_v > 0.6 && (pca_d->vertical.peaks.num[0] + pca_d->vertical.peaks.num[1]) >= 5 && pca_m->horizontal_acceleration_range > 4 && pca_m->lateral_range > 3)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[78] = 1;
		temp_res = dangling_tablet_watch_21_mis(pca_d, pca);
		if(temp_res != 2)
		{
			//My_Method_Num = 78;
			return temp_res;
		}
#else
		temp_res = dangling_tablet_watch_21_mis(pca_d, pca);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 28

	// Method 29
	if(pca->horizontal_vertical_flag == 0 && pca->motion_parametrs[2] < 14 && pca->vertical_parametrs[2] < 7 && (pca->vertical_parametrs[2]/pca->motion_parametrs[2]) > 0.4	&& ratio_v > 0.7 && (pca_d->vertical.peaks.num[0] + pca_d->vertical.peaks.num[1]) > 4)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[79] = 1;
		temp_res = dangling_tablet_watch_22_mis(pca_d);
		if(temp_res != 2)
		{
			//My_Method_Num = 79;
			return temp_res;
		}
#else
		temp_res = dangling_tablet_watch_22_mis(pca_d);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 29

	// Method 30
	if(pca->horizontal_vertical_flag != 0 && pca->vertical_parametrs[2] < 8 && (pca_d->vertical.peaks.num[0] + pca_d->vertical.peaks.num[1]) > 5)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[80] = 1;
		temp_res = dangling_tablet_watch_26_mis(pca_d, pca);
		if(temp_res != 2)
		{
			//My_Method_Num = 80;
			return temp_res;
		}
#else
		temp_res = dangling_tablet_watch_26_mis(pca_d, pca);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 30

	// Method 31
#ifdef METHOD_RESULTS_MISALIGNMENT
	//My_Method_Worked[81] = 1;
	temp_res = dangling_tablet_watch_23_mis(pca_d);
	if(temp_res != 2)
	{
		//My_Method_Num = 81;
		return temp_res;
	}
#else
	temp_res = dangling_tablet_watch_23_mis(pca_d);
	if(temp_res != 2)
		return temp_res;
#endif
	// End of Method 31

# ifndef COM_WALK_SIMPLIFIED
	// Method 32
	if(pca_m->device_flag == 1 && fabs(pca_m->vertical_motion_phase_mean-90) > 3 && fabs(pca_m->motion_effective_coefficient_mean) > 0.05)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[82] = 1;
		if(pca_m->motion_effective_coefficient_mean > 0)
		{
			//My_Method_Num = 82;
			return 1;
		}
		else
		{
			//My_Method_Num = 82;
			return 0;
		}
#else
		if(pca_m->motion_effective_coefficient_mean > 0)
			return 1;
		else
			return 0;
#endif
	}
	// End of Method 32
# endif

	// Method 33
#ifdef METHOD_RESULTS_MISALIGNMENT
	//My_Method_Worked[83] = 1;
	temp_res = dangling_tablet_watch_24_mis(pca_d, pca);
	if(temp_res != 2)
	{
		//My_Method_Num = 83;
		return temp_res;
	}
#else
	temp_res = dangling_tablet_watch_24_mis(pca_d, pca);
	if(temp_res != 2)
		return temp_res;
#endif
	// End of Method 33

	// Method 34
#ifdef METHOD_RESULTS_MISALIGNMENT
	//My_Method_Worked[84] = 1;
	temp_res = dangling_tablet_watch_25_mis(pca_d);
	if(temp_res != 2)
	{
		//My_Method_Num = 84;
		return temp_res;
	}
#else
	temp_res = dangling_tablet_watch_25_mis(pca_d);
	if(temp_res != 2)
		return temp_res;
#endif
	// End of Method 34

	// Method 35
#ifdef METHOD_RESULTS_MISALIGNMENT
	//My_Method_Worked[85] = 1;
	temp_res = dangling_phone_7_mis(pca_d, pca);
	if(temp_res != 2)
	{
		//My_Method_Num = 85;
		return temp_res;
	}
#else
	temp_res = dangling_phone_7_mis(pca_d, pca);
	if(temp_res != 2)
		return temp_res;
#endif
	// End of Method 35
#ifdef METHOD_RESULTS_MISALIGNMENT
	//My_Method_Worked[50] = 1;
	//My_Method_Num = 50;
#endif
	return 2;
}
/**
* @brief	This is Tablet dangling method 1 to solve the 180 amiguity problem in case of Tablet
* 			or Watch dangling data.
*
* @details	This function takes a pointer of structure of type PCA_D. It uses the relation between the
* 			vertical peaks that are greater than certain threshold and motion peaks to provide the
* 			decision of Forward/Backward.
*
* @param[in]	pca_d	pointer to a structure of type \c PCA_D.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_tablet_watch_1_mis(PCA_D_Ptr pca_d)
{
	INT8 i, j;
	INT8 st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;
	// check  for  the vertical low peaks
	for(i=0; i<pca_d->vertical.peaks.num[0]; i++)
	{
		if(pca_d->vertical.peaks.low_val[i] < pca_d->vertical.thresholds[4])
		{
			st_ind = MAX(pca_d->vertical.peaks.low_ind[i]-2, 0);
			en_ind = MIN(pca_d->vertical.peaks.low_ind[i]+2, DATA_SIZE_MIS-1);
			for(j=0; j<pca_d->motion.peaks.num[1]; j++)
			{
				if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
					step_f++;
			}
			for(j=0; j<pca_d->motion.peaks.num[0]; j++)
			{
				if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
					step_b++;
			}
		}
	}
	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is Tablet dangling method 2 to solve the 180 amiguity problem in case of Tablet
* 			or Watch dangling data.
*
* @details	This function takes a pointer of structure of type PCA_D. It uses the relation between the
* 			vertical and motion peaks to provide the decision of Forward/Backward. Two ranges are used
* 			to switch between the vertical high and low peaks. It uses only the vertical low or high
* 			peaks that that are less or greater than certain threshold
*
* @param[in]	pca_d		pointer to a structure of type \c PCA_D.
* @param[in]	range_ml_vl	variable for the range between motion and vertical low peaks.
* @param[in]	range_mh_vh	variable for the range between motion and vertical high peaks.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_tablet_watch_2_mis(PCA_D_Ptr pca_d, FLOAT32 range_ml_vl, FLOAT32 range_mh_vh)
{
	INT8 i, j;
	INT8 st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;

	if(range_ml_vl < range_mh_vh && fabs(range_ml_vl) < 2)
	{
		for(i=0; i<pca_d->vertical.peaks.num[0]; i++)
		{
			if(pca_d->vertical.peaks.low_val[i] < pca_d->vertical.thresholds[0])
			{
				st_ind = MAX(pca_d->vertical.peaks.low_ind[i]-2, 0);
				en_ind = MIN(pca_d->vertical.peaks.low_ind[i]+2, DATA_SIZE_MIS-1);
				for(j=0; j<pca_d->motion.peaks.num[1]; j++)
				{
					if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
						step_b++;
				}
				for(j=0; j<pca_d->motion.peaks.num[0]; j++)
				{
					if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
						step_f++;
				}
			}
		}
	}
	else if(range_mh_vh < range_ml_vl && abs(range_mh_vh) < 2)
	{
		for(i=0; i<pca_d->vertical.peaks.num[1]; i++)
		{
			if(pca_d->vertical.peaks.high_val[i] > pca_d->vertical.thresholds[1])
			{
				st_ind = MAX(pca_d->vertical.peaks.high_ind[i]-2, 0);
				en_ind = MIN(pca_d->vertical.peaks.high_ind[i]+2, DATA_SIZE_MIS-1);
				for(j=0; j<pca_d->motion.peaks.num[1]; j++)
				{
					if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
						step_f++;
				}
				for(j=0; j<pca_d->motion.peaks.num[0]; j++)
				{
					if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
						step_b++;
				}
			}
		}
	}

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is Tablet dangling method 3 to solve the 180 amiguity problem in case of Tablet
* 			or Watch dangling data.
*
* @details	This function takes a pointer of structure of type PCA_D. It uses the relation between the
* 			vertical peaks and motion peaks to provide the decision of Forward/Backward. It uses only
* 			the vertical low peaks that are less thancertain threshold
*
* @param[in]	pca_d	pointer to a structure of type \c PCA_D.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_tablet_watch_3_mis(PCA_D_Ptr pca_d)
{
	INT8 i, j;
	INT8 min_vl_ind = -1, st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;

	FLOAT32 min_vl_val = 99.0;

	for(i = 0; i < pca_d->vertical.peaks.num[0]; i++)
	{
		if(pca_d->vertical.peaks.low_val[i] < min_vl_val)
		{
			min_vl_val = pca_d->vertical.peaks.low_val[i];
			min_vl_ind = i;
		}
	}

	st_ind = MAX(pca_d->vertical.peaks.low_ind[min_vl_ind]-2, 0);
	en_ind = MIN(pca_d->vertical.peaks.low_ind[min_vl_ind]+2, DATA_SIZE_MIS-1);
	if(pca_d->vertical.peaks.low_val[min_vl_ind] < pca_d->vertical.thresholds[0])
	{
		for(j=0; j<pca_d->motion.peaks.num[1]; j++)
		{
			if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
				step_f++;
		}
		for(j=0; j<pca_d->motion.peaks.num[0]; j++)
		{
			if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
				step_b++;
		}
	}
	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is Tablet dangling method 4 to solve the 180 amiguity problem in case of Tablet
* 			or Watch dangling data.
*
* @details	This function takes a pointer of structure of type PCA_D. It uses the relation between the
* 			vertical peaks and motion peaks to provide the decision of Forward/Backward.
*
* @param[in]	pca_d	pointer to a structure of type \c PCA_D.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_tablet_watch_4_mis(PCA_D_Ptr pca_d)
{
	INT8 i, j;
	INT8 st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;

	for(i=0; i<pca_d->vertical.peaks.num[1]; i++)
	{
		st_ind = MAX(pca_d->vertical.peaks.high_ind[i] - 4, 0);
		en_ind = MIN(pca_d->vertical.peaks.high_ind[i] + 4, DATA_SIZE_MIS-1);
		if(pca_d->vertical.peaks.high_val[i] > pca_d->vertical.thresholds[3])
		{
			for(j = 0; j < pca_d->motion.peaks.num[1]; j++)
			{
				if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
					step_b++;
			}
			for(j=0; j<pca_d->motion.peaks.num[0]; j++)
			{
				if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
					step_f++;
			}
		}
		else
		{
			for(j = 0; j < pca_d->motion.peaks.num[1]; j++)
			{
				if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
					step_f++;
			}
			for(j = 0; j < pca_d->motion.peaks.num[0]; j++)
			{
				if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
					step_b++;
			}
		}
	}

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is Tablet dangling method 5 to solve the 180 amiguity problem in case of Tablet
* 			or Watch dangling data.
*
* @details	This function takes a pointer of structure of type PCA_D. It uses the relation between the
* 			vertical peaks and motion peaks to provide the decision of Forward/Backward.
*
* @param[in]	pca_d	pointer to a structure of type \c PCA_D.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_tablet_watch_5_mis(PCA_D_Ptr pca_d)
{
	INT8 i, j;
	INT8 st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;

	for(i=0; i<pca_d->vertical.peaks.num[1]; i++)
	{
		st_ind = MAX(pca_d->vertical.peaks.high_ind[i]-2, 0);
		en_ind = MIN(pca_d->vertical.peaks.high_ind[i]+2, DATA_SIZE_MIS-1);

		if(pca_d->vertical.peaks.high_val[i] > pca_d->vertical.thresholds[1])
		{
			for(j=0; j<pca_d->motion.peaks.num[1]; j++)
			{
				if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind && pca_d->motion.peaks.high_val[j] > pca_d->motion.thresholds[1])
					step_b++;
			}
			for(j=0; j<pca_d->motion.peaks.num[0]; j++)
			{
				if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind && pca_d->motion.peaks.low_val[j] < pca_d->motion.thresholds[0])
					step_f++;
			}
		}
		else if(pca_d->vertical.peaks.high_val[i] > 0.0 && pca_d->vertical.peaks.high_val[i] <= pca_d->vertical.thresholds[1])
		{
			for(j=0; j<pca_d->motion.peaks.num[1]; j++)
			{
				if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind && pca_d->motion.peaks.high_val[j] > pca_d->motion.thresholds[1])
					step_f++;
			}
			for(j=0; j<pca_d->motion.peaks.num[0]; j++)
			{
				if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind && pca_d->motion.peaks.low_val[j] < pca_d->motion.thresholds[0])
					step_b++;
			}
		}
	}
	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is Tablet dangling method 6 to solve the 180 amiguity problem in case of Tablet
* 			or Watch dangling data.
*
* @details	This function takes 2 pointers of structures of type PCA_D and PCA. It uses the
* 			relation between the vertical peaks and motion peaks to provide the decision of Forward/Backward.
*
* @param[in]	pca_d	pointer to a structure of type \c PCA_D.
* @param[in]	pca		pointer to a structure of type \c PCA.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_tablet_watch_6_mis(PCA_D_Ptr pca_d, PCA_Ptr pca)
{
	INT8 i, j;
	INT8 st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;
	INT8 max_vh_ind = -1, min_vh_ind = -1, max_vl_ind = -1, min_vl_ind = -1;
	INT8 max_vh_max_vl_dis = 0, min_vh_max_vl_dis = 0, min_vl_max_vh_dis = 0, min_vl_min_vh_dis = 0;
	FLOAT32 range_vl = 0;
	FLOAT32 min_vh = 99, max_vh = -99, min_vl = 99, max_vl = -99;
	FLOAT32 slope_m;

	for(i = 0; i < pca_d->vertical.peaks.num[1]; i++)
	{
		if(pca_d->vertical.peaks.high_val[i] < min_vh)
		{
			min_vh = pca_d->vertical.peaks.high_val[i];
			min_vh_ind = i;
		}
		if(pca_d->vertical.peaks.high_val[i] > max_vh)
		{
			max_vh = pca_d->vertical.peaks.high_val[i];
			max_vh_ind = i;
		}
	}

	for(i = 0; i < pca_d->vertical.peaks.num[0]; i++)
	{
		if(pca_d->vertical.peaks.low_val[i] < min_vl)
		{
			min_vl = pca_d->vertical.peaks.low_val[i];
			min_vl_ind = i;
		}
		if(pca_d->vertical.peaks.low_val[i] > max_vl)
		{
			max_vl = pca_d->vertical.peaks.low_val[i];
			max_vl_ind = i;
		}
	}
	range_vl = fabs(max_vl - min_vl);

	max_vh_max_vl_dis = pca_d->vertical.peaks.low_ind[max_vl_ind] - pca_d->vertical.peaks.high_ind[max_vh_ind];
	min_vh_max_vl_dis = pca_d->vertical.peaks.low_ind[max_vl_ind] - pca_d->vertical.peaks.high_ind[min_vh_ind];
	min_vl_max_vh_dis = pca_d->vertical.peaks.high_ind[max_vh_ind] - pca_d->vertical.peaks.low_ind[min_vl_ind];
	min_vl_min_vh_dis = pca_d->vertical.peaks.high_ind[min_vh_ind] - pca_d->vertical.peaks.low_ind[min_vl_ind];

	if(range_vl < 1)
	{
		for(i=0; i<pca_d->vertical.peaks.num[1]; i++)
		{
			st_ind = MAX(pca_d->vertical.peaks.high_ind[i], 0);
			en_ind = MIN(pca_d->vertical.peaks.high_ind[i]+3, DATA_SIZE_MIS-1);
			for(j=0; j<pca_d->motion.peaks.num[1]; j++)
			{
				if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
					step_b++;
			}
			for(j=0; j<pca_d->motion.peaks.num[0]; j++)
			{
				if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
					step_f++;
			}
		}
	}

	if(max_vh_max_vl_dis > 0 && max_vh_max_vl_dis <= 5)
	{
		st_ind = MAX(pca_d->vertical.peaks.high_ind[max_vh_ind], 0);
		en_ind = MIN(pca_d->vertical.peaks.low_ind[max_vl_ind], DATA_SIZE_MIS-1);
		for(j=0; j<pca_d->motion.peaks.num[1]; j++)
		{
			if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
				step_b++;
		}
		for(j=0; j<pca_d->motion.peaks.num[0]; j++)
		{
			if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
				step_f++;
		}
	}

	if(min_vh_max_vl_dis > 0 && min_vh_max_vl_dis <= 6)
	{
		st_ind = MAX(pca_d->vertical.peaks.high_ind[min_vh_ind], 0);
		en_ind = MIN(pca_d->vertical.peaks.low_ind[max_vl_ind], DATA_SIZE_MIS-1);
		for(j=0; j<pca_d->motion.peaks.num[1]; j++)
		{
			if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
				step_b++;
		}
		for(j=0; j<pca_d->motion.peaks.num[0]; j++)
		{
			if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
				step_f++;
		}
	}

	if(min_vl_max_vh_dis > 0 && min_vl_max_vh_dis <= 4)
	{
		st_ind = MAX(pca_d->vertical.peaks.low_ind[min_vl_ind], 0);
		en_ind = MIN(pca_d->vertical.peaks.high_ind[max_vh_ind], DATA_SIZE_MIS-1);
		slope_m = pca->data_m[en_ind] - pca->data_m[st_ind];
		if(slope_m >= 0)
			step_b++;
		else
			step_f++;
	}

	if(min_vl_min_vh_dis > 0 && min_vl_min_vh_dis <= 4)
	{
		st_ind = MAX(pca_d->vertical.peaks.low_ind[min_vl_ind], 0);
		en_ind = MIN(pca_d->vertical.peaks.high_ind[max_vh_ind], DATA_SIZE_MIS-1);
		slope_m = pca->data_m[en_ind] - pca->data_m[st_ind];
		if(slope_m >= 0)
			step_b++;
		else
			step_f++;
	}
	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is Tablet dangling method 7 to solve the 180 amiguity problem in case of Tablet
* 			or Watch dangling data.
*
* @details	This function takes 2 pointers of structures of type PCA_D and PCA with the range
* 			of the vertical low peaks. It uses the relation between the vertical peaks and
* 			motion peaks to provide the decision of Forward/Backward.
*
* @param[in]	pca_d		pointer to a structure of type \c PCA_D.
* @param[in]	pca			pointer to a structure of type \c PCA.
* @param[in]	range_vl	variable for the vertical low peaks range.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_tablet_watch_7_mis(PCA_D_Ptr pca_d, PCA_Ptr pca, FLOAT32 range_vl)
{
	INT8 i, j;
	INT8 min_vl_ind = -1, st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;
	FLOAT32 slope_m = 0.0, min_vl = 99.0;

	for(i=0; i< pca_d->vertical.peaks.num[0]; i++)
	{
		if(pca_d->vertical.peaks.low_val[i] < min_vl)
		{
			min_vl = pca_d->vertical.peaks.low_val[i];
			min_vl_ind = i;
		}
	}

	// 7-1
	if(range_vl > 1)
	{
		st_ind = MAX(pca_d->vertical.peaks.low_ind[min_vl_ind]-2, 0);
		en_ind = MIN(pca_d->vertical.peaks.low_ind[min_vl_ind]+2, DATA_SIZE_MIS-1);

		for(j=0; j<pca_d->motion.peaks.num[1]; j++)
		{
			if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
				step_f++;
		}
		for(j=0; j<pca_d->motion.peaks.num[0]; j++)
		{
			if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
				step_b++;
		}
	}
	// End of 7-1

	// 7-2
	if(step_f == step_b)
	{
		for(i=0; i<pca_d->vertical.peaks.num[1]; i++)
		{
			for(j=0; j<pca_d->vertical.peaks.num[0]; j++)
			{
				if(abs(pca_d->vertical.peaks.high_ind[i] - pca_d->vertical.peaks.low_ind[j]) <= 4 && pca_d->vertical.peaks.low_ind[j] < pca_d->vertical.peaks.high_ind[i])
				{
					slope_m = pca->data_m[pca_d->vertical.peaks.high_ind[i]-1] - pca->data_m[pca_d->vertical.peaks.low_ind[j]-1];
					if(slope_m >= 0)
						step_f++;
					else
						step_b++;
				}
			}
		}
	}
	// End of 7-2

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is Tablet dangling method 8 to solve the 180 amiguity problem in case of Tablet
* 			or Watch dangling data.
*
* @details	This function takes a pointer of structure of type PCA_D. It uses the relation between the
* 			vertical peaks and motion peaks to provide the decision of Forward/Backward.
*
* @param[in]	pca_d	pointer to a structure of type \c PCA_D.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_tablet_watch_8_mis(PCA_D_Ptr pca_d)
{
	INT8 i, j;
	INT8 max_vl_ind = -1, min_vh_ind = -1, st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;
	FLOAT32 max_vl_val = -99.0, min_vh_val = 99.0, a = 99.0;

	// 8-1
	for(i = 0; i < pca_d->vertical.peaks.num[1]; i++)
	{
		if(pca_d->vertical.peaks.high_val[i] < a)
		{
			a = pca_d->vertical.peaks.high_val[i];
			min_vh_ind = i;
		}
	}
	if(pca_d->vertical.peaks.high_ind[min_vh_ind] > 26) //too close to the end, will have not enough motion peaks after
	{
		min_vh_ind = -1;
		for(i = 0; i < pca_d->vertical.peaks.num[1]; i++)
		{
			if(pca_d->vertical.peaks.high_val[i] < min_vh_val && pca_d->vertical.peaks.high_val[i] != a)
			{
				min_vh_val = pca_d->vertical.peaks.high_val[i];
				min_vh_ind = i;
			}
		}
	}

	st_ind = MAX(pca_d->vertical.peaks.high_ind[min_vh_ind], 0);
	en_ind = MIN(pca_d->vertical.peaks.high_ind[min_vh_ind]+2, DATA_SIZE_MIS-1);

	for(j=0; j<pca_d->motion.peaks.num[1]; j++)
	{
		if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
			step_f++;
	}
	for(j=0; j<pca_d->motion.peaks.num[0]; j++)
	{
		if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
			step_b++;
	}
	// End of 8-1

	// 8-2
	if(step_f == step_b)
	{
		for(i = 0; i < pca_d->vertical.peaks.num[0]; i++)
		{
			if(pca_d->vertical.peaks.low_val[i] > max_vl_val)
			{
				max_vl_val = pca_d->vertical.peaks.low_val[i];
				max_vl_ind = i;
			}
		}

		st_ind = MAX(pca_d->vertical.peaks.low_ind[max_vl_ind], 0);
		en_ind = MIN(pca_d->vertical.peaks.low_ind[max_vl_ind]+1, DATA_SIZE_MIS-1);

		for(j=0; j<pca_d->motion.peaks.num[1]; j++)
		{
			if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
				step_b++;
		}
		for(j=0; j<pca_d->motion.peaks.num[0]; j++)
		{
			if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
				step_f++;
		}
	}
	// End of 8-2

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is Tablet dangling method 9 to solve the 180 amiguity problem in case of Tablet
* 			or Watch dangling data.
*
* @details	This function takes 2 pointers of structures of type PCA_D and PCA. It uses the
* 			relation between the vertical peaks and motion peaks with the slope of the
* 			motion data to provide the decision of Forward/Backward.
*
* @param[in]	pca_d	pointer to a structure of type \c PCA_D.
* @param[in]	pca		pointer to a structure of type \c PCA.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_tablet_watch_9_mis(PCA_D_Ptr pca_d, PCA_Ptr pca)
{
	INT8 i, j;
	INT8 st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;
	INT8 diff_vh_vl = 0;
	FLOAT32 slope_m;

	for(i = 0; i < pca_d->vertical.peaks.num[1]; i++)
	{
		for(j = 0; j < pca_d->vertical.peaks.num[0]; j++)
		{
			diff_vh_vl = pca_d->vertical.peaks.low_ind[j] - pca_d->vertical.peaks.high_ind[i];
			if(diff_vh_vl > 3 && diff_vh_vl <= 6 && pca_d->vertical.peaks.high_val[i] > pca_d->vertical.thresholds[1] && pca_d->vertical.peaks.low_val[j] < pca_d->vertical.thresholds[0])
			{
				st_ind = pca_d->vertical.peaks.high_ind[i];
				en_ind = pca_d->vertical.peaks.low_ind[j];
				slope_m = pca->data_m[en_ind] - pca->data_m[st_ind];
				if(slope_m <= 0)
					step_b++;
				else
					step_f++;
			}
		}
	}

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is Tablet dangling method 10 to solve the 180 amiguity problem in case of Tablet
* 			or Watch dangling data.
*
* @details	This function takes a pointer of structure of type PCA_D. It uses the relation between the
* 			vertical and motion peaks to provide the decision of Forward/Backward.
*
* @param[in]	pca_d		pointer to a structure of type \c PCA_D.
* @param[in]	range_vl	variable for the vertical low peaks range.
* @param[in]	range_vh	variable for the vertical high peaks range.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_tablet_watch_10_mis(PCA_D_Ptr pca_d, FLOAT32 range_vl, FLOAT32 range_vh)
{
	INT8 i, j;
	INT8 st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;

	if(range_vl > range_vh)
	{
		for(i=0; i<pca_d->vertical.peaks.num[0]; i++)
		{
			if(pca_d->vertical.peaks.low_val[i] < pca_d->vertical.thresholds[2])
			{
				st_ind = MAX(pca_d->vertical.peaks.low_ind[i]-4, 0);
				en_ind = MIN(pca_d->vertical.peaks.low_ind[i]+4, DATA_SIZE_MIS-1);

				for(j=0; j<pca_d->motion.peaks.num[1]; j++)
				{
					if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
						step_f++;
				}

				for(j=0; j<pca_d->motion.peaks.num[0]; j++)
				{
					if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
						step_b++;
				}
			}
		}
	}
	else if(range_vh > range_vl)
	{
		for(i=0; i<pca_d->vertical.peaks.num[1]; i++)
		{
			if(pca_d->vertical.peaks.high_val[i] < pca_d->vertical.thresholds[1])
			{
				st_ind = MAX(pca_d->vertical.peaks.high_ind[i]-3, 0);
				en_ind = MIN(pca_d->vertical.peaks.high_ind[i]+3, DATA_SIZE_MIS-1);
				for(j=0; j<pca_d->motion.peaks.num[1]; j++)
				{
					if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
						step_f++;
				}
				for(j=0; j<pca_d->motion.peaks.num[0]; j++)
				{
					if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
						step_b++;
				}
			}
		}
	}

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is Tablet dangling method 11 to solve the 180 amiguity problem in case of Tablet
* 			or Watch dangling data.
*
* @details	This function takes 3 pointers of structures of type PCA_D, PCA, and PCA_M. It uses the
* 			relation between the vertical peaks and motion peaks with the distance between the
* 			vertical peaks to provide the decision of Forward/Backward.
*
* @param[in]	pca_d	pointer to a structure of type \c PCA_D.
* @param[in]	pca		pointer to a structure of type \c PCA.
* @param[in]	pca_m	pointer to a structure of type \c PCA_M.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_tablet_watch_11_mis(PCA_D_Ptr pca_d, PCA_Ptr pca, PCA_M_Ptr pca_m)
{
	INT8 i, j;
	INT8  min_vl_ind = -1, min_vl_vh = 99, st_ind = -1, en_ind = -1, vl_ind = -1, vh_ind = -1;
	INT8 step_f = 0, step_b = 0;
	FLOAT32 min_vl_val_1 = 99.0;

	for(i = 0; i < pca_d->vertical.peaks.num[0]; i++)
	{
		min_vl_vh = 99;
		vl_ind = -1;
		vh_ind = -1;
		for(j = 0; j < pca_d->vertical.peaks.num[1]; j++)
		{
			if(abs(pca_d->vertical.peaks.low_ind[i] - pca_d->vertical.peaks.high_ind[j]) < min_vl_vh && pca_d->vertical.peaks.high_ind[j] > pca_d->vertical.peaks.low_ind[i])
			{
				min_vl_vh = (INT8)(abs(pca_d->vertical.peaks.low_ind[i] - pca_d->vertical.peaks.high_ind[j]));
				vl_ind = i;
				vh_ind = j;
			}
		}

		if(min_vl_vh <= 5 && vl_ind > -1 && vh_ind > -1)
		{
			for(j=0; j<pca_d->motion.peaks.num[1]; j++)
			{
				if(pca_d->motion.peaks.high_ind[j] >= pca_d->vertical.peaks.low_ind[vl_ind] && pca_d->motion.peaks.high_ind[j] <= pca_d->vertical.peaks.high_ind[vh_ind])
					step_f++;
			}
			for(j=0; j<pca_d->motion.peaks.num[0]; j++)
			{
				if(pca_d->motion.peaks.low_ind[j] >= pca_d->vertical.peaks.low_ind[vl_ind] && pca_d->motion.peaks.low_ind[j] <= pca_d->vertical.peaks.high_ind[vh_ind])
					step_b++;
			}
		}
	}
	if(step_f == step_b && pca->motion_parametrs[2] > 14 && pca_m->horizontal_acceleration_range > 6)
	{
		for(i = 0; i < pca_d->vertical.peaks.num[0]; i++)
		{
			if(pca_d->vertical.peaks.low_val[i] < min_vl_val_1)
			{
				min_vl_val_1 = pca_d->vertical.peaks.low_val[i];
				min_vl_ind = i;
			}
		}

		st_ind = MAX(pca_d->vertical.peaks.low_ind[min_vl_ind]-2, 0);
		en_ind = MIN(pca_d->vertical.peaks.low_ind[min_vl_ind]+2, DATA_SIZE_MIS-1);

		if(pca_d->vertical.peaks.low_val[min_vl_ind] < pca_d->vertical.thresholds[0])
		{
			for(j=0; j<pca_d->motion.peaks.num[1]; j++)
			{
				if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
					step_f++;
			}
			for(j=0; j<pca_d->motion.peaks.num[0]; j++)
			{
				if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
					step_b++;
			}
		}
	}

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is Tablet dangling method 12 to solve the 180 amiguity problem in case of Tablet
* 			or Watch dangling data.
*
* @details	This function takes a pointer of structure of type PCA_D. It uses the relation between the
* 			vertical peaks and motion peaks to provide the decision of Forward/Backward.
*
* @param[in]	pca_d	pointer to a structure of type \c PCA_D.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_tablet_watch_12_mis(PCA_D_Ptr pca_d)
{
	INT8 i, j;
	INT8 min_vl_ind = -1, st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;
	FLOAT32  min_vl_val = 99;

	for(i = 0; i < pca_d->vertical.peaks.num[0]; i++)
	{
		if(pca_d->vertical.peaks.low_val[i] < min_vl_val)
		{
			min_vl_val = pca_d->vertical.peaks.low_val[i];
			min_vl_ind = i;
		}
	}

	st_ind = MAX(pca_d->vertical.peaks.low_ind[min_vl_ind]-1, 0);
	en_ind = MIN(pca_d->vertical.peaks.low_ind[min_vl_ind]+1, DATA_SIZE_MIS-1);
	for(j=0; j<pca_d->motion.peaks.num[1]; j++)
	{
		if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
			step_f++;
	}
	for(j=0; j<pca_d->motion.peaks.num[0]; j++)
	{
		if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
			step_b++;
	}

	if(step_f == step_b)
	{
		for(i = 0; i < pca_d->vertical.peaks.num[0]; i++)
		{
			st_ind = MAX(pca_d->vertical.peaks.low_ind[i]-1, 0);
			en_ind = MIN(pca_d->vertical.peaks.low_ind[i]+1, DATA_SIZE_MIS-1);
			for(j=0; j<pca_d->motion.peaks.num[1]; j++)
			{
				if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
					step_b++;
			}
			for(j=0; j<pca_d->motion.peaks.num[0]; j++)
			{
				if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
					step_f++;
			}
		}
	}

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is Tablet dangling method 13 to solve the 180 amiguity problem in case of Tablet
* 			or Watch dangling data.
*
* @details	This function takes 2 pointers of structures of type PCA_D and PCA. It uses the
* 			relation between pairs of vertical peaks and motion data slope to provide the
* 			decision of Forward/Backward.
*
* @param[in]	pca_d	pointer to a structure of type \c PCA_D.
* @param[in]	pca		pointer to a structure of type \c PCA.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_tablet_watch_13_mis(PCA_D_Ptr pca_d, PCA_Ptr pca)
{
	INT8 i, j;
	INT8 count1 = -1;
	INT8 v_peaks1_diff_ind = 0;
	INT8 step_f = 0, step_b = 0;
	FLOAT32 slope_m;

	PEAKS vertical_peaks;
	memset(&vertical_peaks, 0, sizeof(PEAKS));

	// Remove all high peaks before the first low peak
	if(pca_d->vertical.peaks.num[0] > 1 && pca_d->vertical.peaks.num[1] > 0)
	{
		count1 = 0;
		for(i = 0; i < pca_d->vertical.peaks.num[1]; i++)
		{
			if(pca_d->vertical.peaks.high_ind[i] > pca_d->vertical.peaks.low_ind[0])
			{
				vertical_peaks.high_ind[count1] = pca_d->vertical.peaks.high_ind[i];
				vertical_peaks.high_val[count1] = pca_d->vertical.peaks.high_val[i];
				count1++;
			}
		}
		vertical_peaks.num[1] = count1;
	}

	else
	{
		for(i = 0; i < pca_d->vertical.peaks.num[1]; i++)
		{
			vertical_peaks.high_ind[i] = pca_d->vertical.peaks.high_ind[i];
			vertical_peaks.high_val[i] = pca_d->vertical.peaks.high_val[i];
		}
		vertical_peaks.num[1] = pca_d->vertical.peaks.num[1];
	}
	// Remove all low peaks after the last high peak
	if(pca_d->vertical.peaks.num[0] > 1 && pca_d->vertical.peaks.num[1] > 0)
	{
		count1 = 0;
		for(i = 0; i < pca_d->vertical.peaks.num[0]; i++)
		{
			if(pca_d->vertical.peaks.low_ind[i] < pca_d->vertical.peaks.high_ind[pca_d->vertical.peaks.num[1]-1])
			{
				vertical_peaks.low_ind[count1] = pca_d->vertical.peaks.low_ind[i];
				vertical_peaks.low_val[count1] = pca_d->vertical.peaks.low_val[i];
				count1++;
			}
		}
		vertical_peaks.num[0] = count1;
	}
	else
	{
		for(i = 0; i < pca_d->vertical.peaks.num[0]; i++)
		{
			vertical_peaks.low_ind[i] = pca_d->vertical.peaks.low_ind[i];
			vertical_peaks.low_val[i] = pca_d->vertical.peaks.low_val[i];
		}
		vertical_peaks.num[0] = pca_d->vertical.peaks.num[0];
	}

	for(i = 0; i < vertical_peaks.num[1]; i++)
	{
		for(j = 0; j < vertical_peaks.num[0]; j++)
		{
			v_peaks1_diff_ind = vertical_peaks.high_ind[i] - vertical_peaks.low_ind[j];
			if(v_peaks1_diff_ind >= 3 && v_peaks1_diff_ind <= 5 && vertical_peaks.high_val[i] > pca_d->vertical.thresholds[1] && vertical_peaks.low_val[j] < pca_d->vertical.thresholds[0])
			{
				slope_m = pca->data_m[vertical_peaks.high_ind[i]] - pca->data_m[vertical_peaks.low_ind[j]];
				if(slope_m <= 0)
					step_b++;
				else
					step_f++;
			}
		}
	}

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is Tablet dangling method 14 to solve the 180 amiguity problem in case of Tablet
* 			or Watch dangling data.
*
* @details	This function takes 2 pointers of structures of type PCA_D and PCA. It uses the
* 			distance between the vertical peaks and the motion data slope to provide the
* 			decision of Forward/Backward.
*
* @param[in]	pca_d	pointer to a structure of type \c PCA_D.
* @param[in]	pca		pointer to a structure of type \c PCA.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_tablet_watch_14_mis(PCA_D_Ptr pca_d, PCA_Ptr pca)
{
	INT8 i, j;
	INT8 v_peaks_diff_ind = 0;
	INT8 step_f = 0, step_b = 0;
	FLOAT32 slope_m;

	for(i = 0; i < pca_d->vertical.peaks.num[0]; i++)
	{
		for(j = 0; j < pca_d->vertical.peaks.num[1]; j++)
		{
			v_peaks_diff_ind = pca_d->vertical.peaks.high_ind[j] - pca_d->vertical.peaks.low_ind[i];
			if(v_peaks_diff_ind >= 3 && v_peaks_diff_ind <= 5 && pca_d->vertical.peaks.high_val[j] > pca_d->vertical.thresholds[1] && pca_d->vertical.peaks.low_val[i] < pca_d->vertical.thresholds[0])
			{
				slope_m = pca->data_m[pca_d->vertical.peaks.high_ind[j]] - pca->data_m[pca_d->vertical.peaks.low_ind[i]];
				if(slope_m <= 0)
					step_b++;
				else
					step_f++;
			}
		}
	}

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is Tablet dangling method 15 to solve the 180 amiguity problem in case of Tablet
* 			or Watch dangling data.
*
* @details	This function takes a pointer of structure of type PCA_D. It uses the relation between the
* 			vertical low peaks and motion peaks to provide the decision of Forward/Backward. It uses only
* 			the vertical peaks that are less than a certain threshold.
*
* @param[in]	pca_d	pointer to a structure of type \c PCA_D.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_tablet_watch_15_mis(PCA_D_Ptr pca_d)
{
	INT8 i, j;
	INT8 st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;

	for(i=0; i<pca_d->vertical.peaks.num[1]; i++)
	{
		if(pca_d->vertical.peaks.high_val[i] > pca_d->vertical.thresholds[1])
		{
			st_ind = MAX(pca_d->vertical.peaks.high_ind[i]-2, 0);
			en_ind = MIN(pca_d->vertical.peaks.high_ind[i]+2, DATA_SIZE_MIS-1);
		}
		else if(pca_d->vertical.peaks.high_val[i] < pca_d->vertical.thresholds[1])
		{
			st_ind = MAX(pca_d->vertical.peaks.high_ind[i]-3, 0);
			en_ind = MIN(pca_d->vertical.peaks.high_ind[i]+3, DATA_SIZE_MIS-1);
		}
		for(j=0; j<pca_d->motion.peaks.num[1]; j++)
		{
			if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
				step_b++;
		}
		for(j=0; j<pca_d->motion.peaks.num[0]; j++)
		{
			if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
				step_f++;
		}
	}

	for(i=0; i<pca_d->vertical.peaks.num[0]; i++)
	{
		if(pca_d->vertical.peaks.low_val[i] < pca_d->vertical.thresholds[4])
		{
			st_ind = MAX(pca_d->vertical.peaks.low_ind[i]-3, 0);
			en_ind = MIN(pca_d->vertical.peaks.low_ind[i]+1, DATA_SIZE_MIS-1);
			for(j=0; j<pca_d->motion.peaks.num[1]; j++)
			{
				if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
					step_f++;
			}
			for(j=0; j<pca_d->motion.peaks.num[0]; j++)
			{
				if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
					step_b++;
			}
		}
	}

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is Tablet dangling method 16 to solve the 180 amiguity problem in case of Tablet
* 			or Watch dangling data.
*
* @details	This function takes a pointer of structure of type PCA_D. It uses the relation between the
* 			vertical peaks and motion peaks to provide the decision of Forward/Backward. It uses only
* 			the vertical low peaks that are less than a certain threshold the vertical high peaks that are
* 			greater than a certain threshold.
*
* @param[in]	pca_d	pointer to a structure of type \c PCA_D.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_tablet_watch_16_mis(PCA_D_Ptr pca_d)
{
	INT8 i, j;
	INT8 st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;

	for(i=0; i<pca_d->vertical.peaks.num[0]; i++)
	{
		if(pca_d->vertical.peaks.low_val[i] < pca_d->vertical.thresholds[2])
		{
			st_ind = MAX(pca_d->vertical.peaks.low_ind[i]-2, 0);
			en_ind = MIN(pca_d->vertical.peaks.low_ind[i]+2, DATA_SIZE_MIS-1);
			for(j=0; j<pca_d->motion.peaks.num[1]; j++)
			{
				if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
					step_f++;
			}
			for(j=0; j<pca_d->motion.peaks.num[0]; j++)
			{
				if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
					step_b++;
			}
		}
	}

	for(i=0; i<pca_d->vertical.peaks.num[1]; i++)
	{
		if(pca_d->vertical.peaks.high_val[i] > pca_d->vertical.thresholds[1])
		{
			st_ind = MAX(pca_d->vertical.peaks.high_ind[i]-1, 0);
			en_ind = MIN(pca_d->vertical.peaks.high_ind[i]+1, DATA_SIZE_MIS-1);
			for(j=0; j<pca_d->motion.peaks.num[1]; j++)
			{
				if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
					step_b++;
			}
			for(j=0; j<pca_d->motion.peaks.num[0]; j++)
			{
				if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
					step_f++;
			}
		}
	}
	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is Tablet dangling method 17_1 to solve the 180 amiguity problem in case of Tablet
* 			or Watch dangling data.
*
* @details	This function takes a pointer of structure of type PCA_D. It uses the relation between the
* 			vertical low peaks and motion peaks to provide the decision of Forward/Backward.
*
* @param[in]	pca_d	pointer to a structure of type \c PCA_D.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_tablet_watch_17_mis(PCA_D_Ptr pca_d)
{
	INT8 i, j;
	INT8 st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;

	for(i=0; i<pca_d->vertical.peaks.num[0]; i++)
	{
		if(pca_d->vertical.peaks.low_val[i] < pca_d->vertical.thresholds[2])
		{
			st_ind = MAX(pca_d->vertical.peaks.low_ind[i]-2, 0);
			en_ind = MIN(pca_d->vertical.peaks.low_ind[i]+2, DATA_SIZE_MIS-1);
			for(j=0; j<pca_d->motion.peaks.num[1]; j++)
			{
				if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
					step_f++;
			}
			for(j=0; j<pca_d->motion.peaks.num[0]; j++)
			{
				if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
					step_b++;
			}
		}
	}
	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is Tablet dangling method 17_2 to solve the 180 amiguity problem in case of Tablet
* 			or Watch dangling data.
*
* @details	This function takes a pointer of structure of type PCA_D. It uses the relation between the
* 			vertical low peaks and motion peaks to provide the decision of Forward/Backward.
*
* @param[in]	pca_d	pointer to a structure of type \c PCA_D.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_tablet_watch_17_2_mis(PCA_D_Ptr pca_d)
{
	INT8 i, j;
	INT8 st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;

	for(i=0; i<pca_d->vertical.peaks.num[0]; i++)
	{
		if(pca_d->vertical.peaks.low_val[i] < pca_d->motion.thresholds[0])
		{
			st_ind = MAX(pca_d->vertical.peaks.low_ind[i]-2, 0);
			en_ind = MIN(pca_d->vertical.peaks.low_ind[i]+2, DATA_SIZE_MIS-1);

			for(j=0; j<pca_d->motion.peaks.num[1]; j++)
			{
				if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
					step_b++;
			}
			for(j=0; j<pca_d->motion.peaks.num[0]; j++)
			{
				if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
					step_f++;
			}
		}
	}
	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is Tablet dangling method 18 to solve the 180 amiguity problem in case of Tablet
* 			or Watch dangling data.
*
* @details	This function takes 2 pointers of structures of type PCA_D and PCA. It uses the
* 			slope of the motion data during the intrval of minimum distance between high-low
* 			pair of vertical peaks to provide the decision of Forward/Backward.
*
* @param[in]	pca_d	pointer to a structure of type \c PCA_D.
* @param[in]	pca		pointer to a structure of type \c PCA.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_tablet_watch_18_mis(PCA_D_Ptr pca_d, PCA_Ptr pca)
{
	INT8 i, j;
	INT8 min_vl_vh = 99, min_vl_ind = -1, min_vh_ind = -1;
	INT8 step_f = 0, step_b = 0;

	FLOAT32 slope_m;

	for(i = 0; i < pca_d->vertical.peaks.num[0]; i++)
	{
		for(j = 0; j < pca_d->vertical.peaks.num[1]; j++)
		{
			if(abs(pca_d->vertical.peaks.low_ind[i] - pca_d->vertical.peaks.high_ind[j]) < min_vl_vh)
			{
				min_vl_vh = (INT8)(abs(pca_d->vertical.peaks.low_ind[i] - pca_d->vertical.peaks.high_ind[j]));
				min_vl_ind = i;
				min_vh_ind = j;
			}
		}
	}

	if(min_vl_vh <= 3 && min_vl_ind > -1 && min_vh_ind > -1 && pca_d->vertical.peaks.low_val[min_vl_ind] < pca_d->vertical.thresholds[0] && pca_d->vertical.peaks.high_val[min_vh_ind] > pca_d->vertical.thresholds[1])
	{
		slope_m = pca->data_m[pca_d->vertical.peaks.high_ind[min_vh_ind]] - pca->data_m[pca_d->vertical.peaks.low_ind[min_vl_ind]] ;
		if(slope_m >= 0)
			step_b++;
		else
			step_f++;
	}
	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is Tablet dangling method 19 to solve the 180 amiguity problem in case of Tablet
* 			or Watch dangling data.
*
* @details	This function takes a pointer of structure of type PCA_D. It uses the relation between the
* 			vertical low peaks and motion peaks to provide the decision of Forward/Backward.
*
* @param[in]	pca_d	pointer to a structure of type \c PCA_D.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_tablet_watch_19_mis(PCA_D_Ptr pca_d)
{
	INT8 i, j;
	INT8 min_vl_ind = -1, st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;
	FLOAT32 min_vl_val = 99.0;

	for(i = 0; i < pca_d->vertical.peaks.num[0]; i++)
	{
		if(pca_d->vertical.peaks.low_val[i] < min_vl_val)
		{
			min_vl_val = pca_d->vertical.peaks.low_val[i];
			min_vl_ind = i;
		}
	}

	st_ind = MAX(pca_d->vertical.peaks.low_ind[min_vl_ind]-1, 0);
	en_ind = MIN(pca_d->vertical.peaks.low_ind[min_vl_ind]+1, DATA_SIZE_MIS-1);

	for(j=0; j<pca_d->motion.peaks.num[1]; j++)
	{
		if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
			step_b++;
	}

	for(j=0; j<pca_d->motion.peaks.num[0]; j++)
	{
		if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
			step_f++;
	}

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is Tablet dangling method 20 to solve the 180 amiguity problem in case of Tablet
* 			or Watch dangling data.
*
* @details	This function takes 3 pointers of structures of type PCA_D, PCA, and PCA_M. It uses the
* 			relation between the vertical peaks and motion peaks to provide the decision of Forward/Backward.
*
* @param[in]	pca_d	pointer to a structure of type \c PCA_D.
* @param[in]	pca		pointer to a structure of type \c PCA.
* @param[in]	pca_m	pointer to a structure of type \c PCA_M.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_tablet_watch_20_mis(PCA_D_Ptr pca_d, PCA_Ptr pca, PCA_M_Ptr pca_m)
{
	INT8 i, j, k;
	INT8 st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;
	INT8 max_vh_ind = -1, min_vh_ind = -1, max_vl_ind = -1, min_vl_ind = -1;
	INT8 max_vh_max_vl_dis = 0, max_vl_min_vh_dis = 0;

	FLOAT32 min_vh = 99, max_vh = -99, max_vl = -99;
	FLOAT32 slope_m, min_vl_val = 99;

	// 20-1
	if(pca_m->lateral_range < 4)
	{
		for(i = 0; i < pca_d->vertical.peaks.num[0]; i++)
		{
			if(pca_d->vertical.peaks.low_val[i] < min_vl_val)
			{
				min_vl_val = pca_d->vertical.peaks.low_val[i];
				min_vl_ind = i;
			}
		}
		st_ind = MAX(pca_d->vertical.peaks.low_ind[min_vl_ind]-2, 0);
		en_ind = MIN(pca_d->vertical.peaks.low_ind[min_vl_ind]+2, DATA_SIZE_MIS-1);
		for(j=0; j<pca_d->motion.peaks.num[1]; j++)
		{
			if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
				step_f++;
		}
		for(j=0; j<pca_d->motion.peaks.num[0]; j++)
		{
			if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
				step_b++;
		}
	}
	// End of 20-1

	// 20-2
	if(step_f == step_b && pca->horizontal_vertical_flag == 1)
	{
		for(i = 0; i < pca_d->vertical.peaks.num[1]; i++)
		{
			for(j = 0; j < pca_d->vertical.peaks.num[0]; j++)
			{
				if(abs(pca_d->vertical.peaks.high_ind[i] - pca_d->vertical.peaks.low_ind[j]) <= 6 &&
					pca_d->vertical.peaks.high_ind[i] < pca_d->vertical.peaks.low_ind[j])
				{
					st_ind = pca_d->vertical.peaks.high_ind[i];
					en_ind = pca_d->vertical.peaks.low_ind[j];

					for(k=0; k<pca_d->motion.peaks.num[1]; k++)
					{
						if(pca_d->motion.peaks.high_ind[k] >= st_ind && pca_d->motion.peaks.high_ind[k] <= en_ind)
							step_b++;
					}
					for(k=0; k<pca_d->motion.peaks.num[0]; k++)
					{
						if(pca_d->motion.peaks.low_ind[k] >= st_ind && pca_d->motion.peaks.low_ind[k] <= en_ind)
							step_f++;
					}

				}
			}
		}

		for(i = 0; i < pca_d->vertical.peaks.num[1]; i++)
		{
			if(pca_d->vertical.peaks.high_val[i] < min_vh)
			{
				min_vh = pca_d->vertical.peaks.high_val[i];
				min_vh_ind = i;
			}
			if(pca_d->vertical.peaks.high_val[i] > max_vh)
			{
				max_vh = pca_d->vertical.peaks.high_val[i];
				max_vh_ind = i;
			}
		}

		for(i = 0; i < pca_d->vertical.peaks.num[0]; i++)
		{
			if(pca_d->vertical.peaks.low_val[i] > max_vl)
			{
				max_vl = pca_d->vertical.peaks.low_val[i];
				max_vl_ind = i;
			}
		}
		max_vh_max_vl_dis = pca_d->vertical.peaks.low_ind[max_vl_ind] - pca_d->vertical.peaks.high_ind[max_vh_ind];
		max_vl_min_vh_dis = pca_d->vertical.peaks.high_ind[min_vh_ind] - pca_d->vertical.peaks.low_ind[max_vl_ind];

		if(max_vh_max_vl_dis > 0 && max_vh_max_vl_dis <= 7)
		{
			st_ind = pca_d->vertical.peaks.high_ind[max_vh_ind];
			en_ind = pca_d->vertical.peaks.low_ind[max_vl_ind];

			for(j=0; j<pca_d->motion.peaks.num[1]; j++)
			{
				if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
					step_b++;
			}
			for(j=0; j<pca_d->motion.peaks.num[0]; j++)
			{
				if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
					step_f++;
			}
		}

		if(max_vl_min_vh_dis > 0 && max_vl_min_vh_dis <= 5)
		{
			st_ind = pca_d->vertical.peaks.low_ind[max_vl_ind];
			en_ind = pca_d->vertical.peaks.high_ind[min_vh_ind];
			slope_m = pca->data_m[en_ind] - pca->data_m[st_ind];
			if(slope_m >= 0)
				step_f++;
			else
				step_b++;
		}
	}
	// End of 20-2

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is Tablet dangling method 21 to solve the 180 amiguity problem in case of Tablet
* 			or Watch dangling data.
*
* @details	This function takes 2 pointers of structures of type PCA_D and PCA. It uses the
* 			slope of the motion and the distance between vertical high and low peaks
* 			to provide the decision of Forward/Backward.
*
* @param[in]	pca_d	pointer to a structure of type \c PCA_D.
* @param[in]	pca		pointer to a structure of type \c PCA.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_tablet_watch_21_mis(PCA_D_Ptr pca_d, PCA_Ptr pca)
{
	INT8 i, j;
	INT8 step_f = 0, step_b = 0;
	INT8 v_peaks_diff_dis = 0;
	FLOAT32 slope_m;

	for(i = 0; i < pca_d->vertical.peaks.num[0]; i++)
	{
		for(j = 0; j < pca_d->vertical.peaks.num[1]; j++)
		{
			v_peaks_diff_dis = pca_d->vertical.peaks.high_ind[j] - pca_d->vertical.peaks.low_ind[i];
			if(v_peaks_diff_dis > 3 && v_peaks_diff_dis <= 6)
			{
				slope_m = pca->data_m[pca_d->vertical.peaks.high_ind[j]] - pca->data_m[pca_d->vertical.peaks.low_ind[i]];
				if(slope_m >= 0)
					step_b++;
				else
					step_f++;
			}
		}
	}

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is Tablet dangling method 22 to solve the 180 amiguity problem in case
* 			of Tablet or Watch dangling data.
*
* @details	This function takes a pointer of structure of type PCA_D. It uses the
* 			relation between the vertical low peaks and motion peaks to provide
* 			the decision of Forward/Backward. It uses only the vertical low peaks
* 			that are less than a certain threshold the vertical high peaks that are
* 			greater than a certain threshold.
*
* @param[in]	pca_d	pointer to a structure of type \c PCA_D.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_tablet_watch_22_mis(PCA_D_Ptr pca_d)
{
	INT8 i, j;
	INT8 st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;

	// check  for  the vertical high peaks
	for(i=0; i<pca_d->vertical.peaks.num[1]; i++)
	{
		if(pca_d->vertical.peaks.high_val[i] > pca_d->vertical.thresholds[1])
		{
			st_ind = MAX(pca_d->vertical.peaks.high_ind[i]-3, 0);
			en_ind = MIN(pca_d->vertical.peaks.high_ind[i]+3, DATA_SIZE_MIS-1);

			for(j=0; j<pca_d->motion.peaks.num[1]; j++)
			{
				if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind && pca_d->motion.peaks.high_val[j] > pca_d->motion.thresholds[1])
					step_f++;
			}
			for(j=0; j<pca_d->motion.peaks.num[0]; j++)
			{
				if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind && pca_d->motion.peaks.low_val[j] < pca_d->motion.thresholds[0])
					step_b++;
			}
		}
	}

	// check  for  the vertical low peaks
	for(i=0; i<pca_d->vertical.peaks.num[0]; i++)
	{
		if(pca_d->vertical.peaks.low_val[i] < pca_d->vertical.thresholds[0])
		{
			st_ind = MAX(pca_d->vertical.peaks.low_ind[i]-3, 0);
			en_ind = MIN(pca_d->vertical.peaks.low_ind[i]+3, DATA_SIZE_MIS-1);

			for(j=0; j<pca_d->motion.peaks.num[1]; j++)
			{
				if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind && pca_d->motion.peaks.high_val[j] > pca_d->motion.thresholds[1])
					step_b++;
			}
			for(j=0; j<pca_d->motion.peaks.num[0]; j++)
			{
				if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind && pca_d->motion.peaks.low_val[j] < pca_d->motion.thresholds[0])
					step_f++;
			}
		}
	}

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is Tablet dangling method 23 to solve the 180 amiguity problem in case
* 			of Tablet or Watch dangling data.
*
* @details	This function takes a pointer of structure of type PCA_D. It uses the
* 			relation between the vertical low peaks and motion peaks to provide
* 			the decision of Forward/Backward. It uses only the vertical low peaks
* 			that are less than a certain threshold and the vertical high peaks that are
* 			greater than a certain threshold.
*
* @param[in]	pca_d	pointer to a structure of type \c PCA_D.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_tablet_watch_23_mis(PCA_D_Ptr pca_d)
{
	INT8 i, j;
	INT8 st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;

	// check  for  the vertical high peaks
	for(i=0; i<pca_d->vertical.peaks.num[1]; i++)
	{
		if(pca_d->vertical.peaks.high_val[i] > pca_d->vertical.thresholds[1])
		{
			st_ind = MAX(pca_d->vertical.peaks.high_ind[i]-3, 0);
			en_ind = MIN(pca_d->vertical.peaks.high_ind[i]+3, DATA_SIZE_MIS-1);

			for(j=0; j<pca_d->motion.peaks.num[1]; j++)
			{
				if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind && pca_d->motion.peaks.high_val[j] > pca_d->motion.thresholds[1])
					step_b++;
			}
			for(j=0; j<pca_d->motion.peaks.num[0]; j++)
			{
				if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind && pca_d->motion.peaks.low_val[j] < pca_d->motion.thresholds[0])
					step_f++;
			}
		}
	}
	// check  for  the vertical low peaks
	for(i=0; i<pca_d->vertical.peaks.num[0]; i++)
	{
		if(pca_d->vertical.peaks.low_val[i] < pca_d->vertical.thresholds[0])
		{
			st_ind = MAX(pca_d->vertical.peaks.low_ind[i]-3, 0);
			en_ind = MIN(pca_d->vertical.peaks.low_ind[i]+3, DATA_SIZE_MIS-1);

			for(j=0; j<pca_d->motion.peaks.num[1]; j++)
			{
				if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind && pca_d->motion.peaks.high_val[j] > pca_d->motion.thresholds[1])
					step_f++;
			}

			for(j=0; j<pca_d->motion.peaks.num[0]; j++)
			{
				if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind && pca_d->motion.peaks.low_val[j] < pca_d->motion.thresholds[0])
					step_b++;
			}
		}
	}

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is Tablet dangling method 24 to solve the 180 amiguity problem in case of Tablet
* 			or Watch dangling data.
*
* @details	This function takes 2 pointers of structures of type PCA_D and PCA. It uses the
* 			relation between the motion and the vertical high peaks to provide the decision
* 			of Forward/Backward. It uses only the vertical high peaks that are positive and
* 			greater than a certain threshold
*
* @param[in]	pca_d	pointer to a structure of type \c PCA_D.
* @param[in]	pca		pointer to a structure of type \c PCA.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_tablet_watch_24_mis(PCA_D_Ptr pca_d, PCA_Ptr pca)
{
	INT8 i, j;
	INT8 st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;
	//INT8 count_h = 0;
	//INT8 vh_peaks_low_ind[PEAKS_VECTOR_SIZE_MIS] = {-1};
	//FLOAT32 vh_peaks_low_val[PEAKS_VECTOR_SIZE_MIS] = {0.0};

	/*if(pca->vertical_parametrs[2] >= 0.8*pca->motion_parametrs[2])
	{
	for(i=0; i<pca_d->vertical.peaks.num[1]; i++)
	{
	if(pca_d->vertical.peaks.high_val[i] <= pca_d->vertical.thresholds[1] && pca_d->vertical.peaks.high_val[i] >= 0.0)
	{
	vh_peaks_low_ind[count_h] = pca_d->vertical.peaks.high_ind[i];
	vh_peaks_low_val[count_h] = pca_d->vertical.peaks.high_val[i];
	count_h++;
	}
	}
	for(i=0; i<count_h; i++)
	{
	st_ind = MAX(vh_peaks_low_ind[i]-2, 0);
	en_ind = MIN(vh_peaks_low_ind[i]+2, DATA_SIZE_MIS-1);

	for(j=0; j<pca_d->motion.peaks.num[1]; j++)
	{
	if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
	step_f++;
	}
	for(j=0; j<pca_d->motion.peaks.num[0]; j++)
	{
	if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
	step_b++;
	}
	}
	} */
	if(pca->vertical_parametrs[2] >= 0.8*pca->motion_parametrs[2])
	{
		for(i=0; i<pca_d->vertical.peaks.num[1]; i++)
		{
			if(pca_d->vertical.peaks.high_val[i] <= pca_d->vertical.thresholds[1] && pca_d->vertical.peaks.high_val[i] >= 0.0)
			{
				st_ind = MAX(pca_d->vertical.peaks.high_ind[i]-2, 0);
				en_ind = MIN(pca_d->vertical.peaks.high_ind[i]+2, DATA_SIZE_MIS-1);

				for(j=0; j<pca_d->motion.peaks.num[1]; j++)
				{
					if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
						step_f++;
				}
				for(j=0; j<pca_d->motion.peaks.num[0]; j++)
				{
					if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
						step_b++;
				}
			}
		}
	}

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is Tablet dangling method 25 to solve the 180 amiguity problem in case
* 			of Tablet or Watch dangling data.
*
* @details	This function takes a pointer of structure of type PCA_D. It uses the
* 			relation between the vertical low peaks and motion peaks to provide
* 			the decision of Forward/Backward. It uses only the vertical high peaks
* 			that are close to each other within a disatnce of 11 samples.
*
* @param[in]	pca_d	pointer to a structure of type \c PCA_D.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_tablet_watch_25_mis(PCA_D_Ptr pca_d)
{
	INT8 i, j, k;
	INT8 st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;
	INT8 vh_close[PEAKS_VECTOR_SIZE_MIS][2] = {0};

	k = 0;
	for(i=1; i<pca_d->vertical.peaks.num[1]; i++)
	{
		if((pca_d->vertical.peaks.high_ind[i]-pca_d->vertical.peaks.high_ind[i-1]) <= 11 && pca_d->vertical.peaks.high_val[i] > pca_d->vertical.thresholds[1] && pca_d->vertical.peaks.high_val[i-1] > pca_d->vertical.thresholds[1])
		{
			for(j=0; j<pca_d->vertical.peaks.num[0]; j++)
			{
				if(pca_d->vertical.peaks.low_ind[j] >= pca_d->vertical.peaks.high_ind[i-1] && pca_d->vertical.peaks.low_ind[j] <= pca_d->vertical.peaks.high_ind[i])
				{
					vh_close[k][0] = i-1;
					vh_close[k][1] = j;
					k++;
				}
			}
		}
	}
	for(i=0; i<k; i++)
	{
		st_ind = MAX(pca_d->vertical.peaks.high_ind[vh_close[i][0]]-2, 0);
		en_ind = MIN(pca_d->vertical.peaks.high_ind[vh_close[i][0]]+3, DATA_SIZE_MIS-1);

		for(j=0; j<pca_d->motion.peaks.num[1]; j++)
		{
			if(pca_d->motion.peaks.high_ind[j] >= st_ind && pca_d->motion.peaks.high_ind[j] <= en_ind)
				step_b++;
		}
		for(j=0; j<pca_d->motion.peaks.num[0]; j++)
		{
			if(pca_d->motion.peaks.low_ind[j] >= st_ind && pca_d->motion.peaks.low_ind[j] <= en_ind)
				step_f++;
		}
	}
	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is Tablet dangling method 26 to solve the 180 amiguity problem in case of Tablet
* 			or Watch dangling data.
*
* @details	This function takes 2 pointers of structures of type PCA_D and PCA. It uses the
* 			slope of the motion and the distance between vertical high and low peaks
* 			to provide the decision of Forward/Backward.
*
* @param[in]	pca_d	pointer to a structure of type \c PCA_D.
* @param[in]	pca		pointer to a structure of type \c PCA.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 dangling_tablet_watch_26_mis(PCA_D_Ptr pca_d, PCA_Ptr pca)
{
	INT8 i, j, k;
	INT8 st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;
	INT8 vh_vl_dis = 0, vl_vh_dis = 0;
	FLOAT32 slope_m = 0;
	UINT8 low_peak_flag = 0;
	UINT8 high_peak_flag = 0;

	for(i = 0; i < pca_d->vertical.peaks.num[1]; i++)
	{
		for(j = 0; j < pca_d->vertical.peaks.num[0]; j++)
		{
			vh_vl_dis = pca_d->vertical.peaks.low_ind[j] - pca_d->vertical.peaks.high_ind[i];
			if(vh_vl_dis >= 5 && vh_vl_dis <= 7 && pca_d->vertical.peaks.high_val[i] > pca_d->vertical.thresholds[1] && pca_d->vertical.peaks.low_val[j] < pca_d->vertical.thresholds[0])
			{
				st_ind = pca_d->vertical.peaks.high_ind[i];
				en_ind = pca_d->vertical.peaks.low_ind[j];
				for(k=0; k<pca_d->motion.peaks.num[1]; k++)
				{
					if(pca_d->motion.peaks.high_ind[k] >= st_ind && pca_d->motion.peaks.high_ind[k] <= en_ind)
						step_f++;
				}
				for(k=0; k<pca_d->motion.peaks.num[0]; k++)
				{
					if(pca_d->motion.peaks.low_ind[k] >= st_ind && pca_d->motion.peaks.low_ind[k] <= en_ind)
						step_b++;
				}
			}

			if(vh_vl_dis > 0 && vh_vl_dis < 5 && pca_d->vertical.peaks.high_val[i] > pca_d->vertical.thresholds[1] && pca_d->vertical.peaks.low_val[j] < pca_d->vertical.thresholds[0])
			{
				st_ind = pca_d->vertical.peaks.high_ind[i];
				en_ind = pca_d->vertical.peaks.low_ind[j];
				for(k=0; k<pca_d->motion.peaks.num[1]; k++)
				{
					if(pca_d->motion.peaks.high_ind[k] >= st_ind && pca_d->motion.peaks.high_ind[k] <= en_ind)
						step_b++;
				}
				for(k=0; k<pca_d->motion.peaks.num[0]; k++)
				{
					if(pca_d->motion.peaks.low_ind[k] >= st_ind && pca_d->motion.peaks.low_ind[k] <= en_ind)
						step_f++;
				}
			}
		}
	}

	if(step_f > step_b)
	{
		step_f = 1;
		step_b = 0;
	}
	else if(step_f < step_b)
	{
		step_f = 0;
		step_b = 1;
	}

	for(i = 0; i < pca_d->vertical.peaks.num[0]; i++)
	{
		for(j = 0; j < pca_d->vertical.peaks.num[1]; j++)
		{
			vl_vh_dis = pca_d->vertical.peaks.high_ind[j] - pca_d->vertical.peaks.low_ind[i];
			if(vl_vh_dis >= 4 && vl_vh_dis <= 6 && pca_d->vertical.peaks.high_val[j] > pca_d->vertical.thresholds[1] && pca_d->vertical.peaks.low_val[i] < pca_d->vertical.thresholds[0])
			{
				st_ind = pca_d->vertical.peaks.low_ind[i];
				en_ind = pca_d->vertical.peaks.high_ind[j];
				slope_m = pca->data_m[en_ind] - pca->data_m[st_ind];
				if(slope_m < 0)
				{
					step_f++;
					// check for low peak
					for(k=0; k<pca_d->motion.peaks.num[0]; k++)
					{
						if(pca_d->motion.peaks.low_ind[k] >= st_ind && pca_d->motion.peaks.low_ind[k] <= en_ind)
						{	step_f--;	low_peak_flag = 1;	}
					}
					// if no low peak
					// check for a bump
					if(!low_peak_flag)
					{
						for(k = st_ind+1; k < en_ind-1; k++)
						{
							if((pca->data_m[k] - pca->data_m[k-1])*(pca->data_m[k+1] - pca->data_m[k]) < 0) //slope change
							{	step_f--;	break;	}
						}
					}
				}
				else
				{
					step_b++;
					// check for high peak
					for(k = 0; k < pca_d->motion.peaks.num[1]; k++)
					{
						if(pca_d->motion.peaks.high_ind[k] >= st_ind && pca_d->motion.peaks.high_ind[k] <= en_ind)
						{	step_b--;	high_peak_flag = 1;	}
					}
					// if no high peak
					// check for a bump
					if(!high_peak_flag)
					{
						for(k = st_ind+1; k < en_ind-1; k++)
						{
							if((pca->data_m[k] - pca->data_m[k-1])*(pca->data_m[k+1] - pca->data_m[k]) < 0) //slope change
							{	step_b--;	break;	}
						}
					}
				}
			}
		}
	}

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is general backward forward function to solve the 180 amiguity problem in general use cases.
*
* @details	This function takes the motion, vertical, roll and pitch data with other data parameters stored in
* 			2 pointers of structures of type PCA and PCA_M with the misalignment angle. it calls 2 different
* 			functions to get the decision of Forward/Backward. This function deals with all cases other than
* 			pocket and dangling such as handheld, belt, ear taking, etc.... . Also, it can deal with the
* 			dangling or pocket data if theya are classified as general.
*
* @param[in]	data_r	pointer to roll data vector.
* @param[in]	data_p	pointer to pitch data vector.
* @param[in]	pca		pointer to a structure of type \c PCA.
* @param[in]	pca_m  	pointer to a structure of type \c PCA_M.
* @param[in]	misa  	variable to mislaignment angle in float.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*
* @see general_phone_tablet_backward_forward_mis() and general_watch_backward_forward_mis().
*/
INT8 general_backward_forward_mis(FLOAT32 *data_r, FLOAT32 *data_p, PCA_Ptr pca, PCA_M_Ptr pca_m, FLOAT32 misa)
{
	INT8 i, j, k;
	INT8 count1 = 0, count2 = 0;
	INT8 trigger = 1, effective = 0, change_m = 0;
	INT8 max_dis_vl = -99, max_dis_vh = -99;
	INT8 be_array[10] = {0};
	INT8 temp_res = 2;
	INT8 min_samp[PEAKS_VECTOR_SIZE_MIS] = {0};
	INT8 dis_vl[PEAKS_VECTOR_SIZE_MIS] = {0}, dis_vh[PEAKS_VECTOR_SIZE_MIS] = {0};
	FLOAT32 range_att = 0.0;
	FLOAT32 a = 0.0, c = 0.0, delta = 1.0;
	FLOAT32 max_v = -99.0;
	FLOAT32 zero_cro_fv[PEAKS_VECTOR_SIZE_MIS] = {0.0}, zero_cro_rv[PEAKS_VECTOR_SIZE_MIS] = {0.0};
	FLOAT32 max_min[2] = {0.0};
	FLOAT32 atti_vec[DATA_SIZE_MIS] = {0.0}, min_diff[PEAKS_VECTOR_SIZE_MIS] = {0.0};

	PCA_G pca_g;PEAKS motion_peaks1;PEAKS vertical_peaks1;
	memset(&pca_g, 0, sizeof(PCA_G));


	memset(&motion_peaks1, 0, sizeof(PEAKS));


	memset(&vertical_peaks1, 0, sizeof(PEAKS));

	pca_g.switch_me = 0;
	pca_g.i_chest = 0;
	pca_g.misa = misa;

	// define the thresholds
	pca_g.vertical.thresholds[0] = pca->vertical_parametrs[0] - 1.2F*pca->vertical_parametrs[1];		// vl_thr  = mean_v - 1.2*std_v;
	pca_g.vertical.thresholds[1] = pca->vertical_parametrs[0] + 1.2F*pca->vertical_parametrs[1];		// vh_thr  = mean_v + 1.2*std_v;
	pca_g.vertical.thresholds[2] = pca->vertical_parametrs[0] - 1.5F*pca->vertical_parametrs[1];		// vl_thr2 = mean_v - 1.5*std_v;
	pca_g.vertical.thresholds[3] = pca->vertical_parametrs[0] + 1.5F*pca->vertical_parametrs[1];		// vh_thr2 = mean_v + 1.5*std_v;
	pca_g.motion.thresholds[0] = pca->motion_parametrs[0] -      pca->motion_parametrs[1];			// ml_thr  = mean_m - std_m;
	pca_g.motion.thresholds[1] = pca->motion_parametrs[0] +      pca->motion_parametrs[1];			// mh_thr  = mean_m + std_m;
	pca_g.motion.thresholds[2] = pca->motion_parametrs[0] - 1.6F*pca->motion_parametrs[1];			// mh_thr  = mean_m + std_m;
	pca_g.motion.thresholds[3] = pca->motion_parametrs[0] + 1.6F*pca->motion_parametrs[1];			// mh_thr  = mean_m + std_m;

	// peaks detection
	delta = pca->motion_parametrs[1]/4;
	if(delta < 0.2)
		delta = pca->motion_parametrs[1]/2;

	i = peaks_detection_mis(pca->data_m, delta, 4, &motion_peaks1, 20, 36, 0);

	i = peaks_detection_mis(pca->data_v, pca->vertical_parametrs[2]/10, 4, &vertical_peaks1, 20, 36, 0);
	if(vertical_peaks1.num[0] == 0 || vertical_peaks1.num[1] == 0)
		i= peaks_detection_mis(pca->data_v, pca->vertical_parametrs[1]/3, 4, &vertical_peaks1, 20, 36, 0);

	//delete unreasonable vertical low peaks
	for(i = 0; i < DATA_SIZE_MIS; i++)
	{
		if(pca->data_v[i] > max_v)
			max_v = pca->data_v[i];
	}
	count1 = 0;
	if(vertical_peaks1.num[0] > 0)
	{
		for(i = 0; i < vertical_peaks1.num[0]; i++)
		{
			if(vertical_peaks1.low_val[i] < (max_v-pca->vertical_parametrs[2]/2))
			{
				pca_g.vertical.peaks.low_ind[count1]   = vertical_peaks1.low_ind[i];
				pca_g.vertical.peaks.low_val[count1++] = vertical_peaks1.low_val[i];
			}
		}
		vertical_peaks1.num[0] = count1;
	}
	pca_g.vertical.peaks.num[0] = vertical_peaks1.num[0];

	//delete unreasonable vertical high peaks
	count1 = 0;
	if(vertical_peaks1.num[1] > 0)
	{
		for(i = 0; i < vertical_peaks1.num[1]; i++)
		{
			if(vertical_peaks1.high_val[i] > (max_v-pca->vertical_parametrs[2]/2))
			{
				pca_g.vertical.peaks.high_ind[count1]   = vertical_peaks1.high_ind[i];
				pca_g.vertical.peaks.high_val[count1++] = vertical_peaks1.high_val[i];
			}
		}
		vertical_peaks1.num[1] = count1;
	}
	pca_g.vertical.peaks.num[1] = vertical_peaks1.num[1];

	//Enhancement for vertical peaks consistence. Added 2012.06.25
	while(trigger == 1)
	{
		trigger = 0;
		for(i = 0; i < pca_g.vertical.peaks.num[1]-1; i++)
		{
			effective = 0;
			be_array[0] = 0;
			be_array[1] = 0;
			for(j = 0; j < pca_g.vertical.peaks.num[0]; j++)
			{
				if(pca_g.vertical.peaks.low_ind[j] > pca_g.vertical.peaks.high_ind[i] && pca_g.vertical.peaks.low_ind[j] < pca_g.vertical.peaks.high_ind[i+1])
				{
					be_array[effective] = j;
					effective = effective +1;
				}
			}
			if(effective == 0) //no low point between high points
			{
				if(pca_g.vertical.peaks.high_val[i] < pca_g.vertical.peaks.high_val[i+1])
					count1 = i;
				else
					count1 = i+1;

				count2 = 0;
				for(k = 0; k < pca_g.vertical.peaks.num[1]; k++)
				{
					if(k != count1) //not the peak that need to be deleted
					{
						pca_g.vertical.peaks.high_ind[count2] = pca_g.vertical.peaks.high_ind[k];
						pca_g.vertical.peaks.high_val[count2] = pca_g.vertical.peaks.high_val[k];
						count2++;
					}
				}
				trigger = 1; //restart checking loop
				pca_g.vertical.peaks.num[1]--;
				break;
			}
			if(effective == 2) // two low points between high points
			{
				if(pca_g.vertical.peaks.low_val[be_array[0]] < pca_g.vertical.peaks.low_val[be_array[1]])
					count1 = be_array[1];
				else
					count1 = be_array[0];

				count2 = 0; //start to delete this peak
				for(k = 0; k < pca_g.vertical.peaks.num[0]; k++)
				{
					if(k != count1) //not the peak that need to be deleted
					{
						pca_g.vertical.peaks.low_ind[count2] = pca_g.vertical.peaks.low_ind[k];
						pca_g.vertical.peaks.low_val[count2] = pca_g.vertical.peaks.low_val[k];
						count2++;
					}
				}
				pca_g.vertical.peaks.num[0]--;
			}
		}//for(i = 0; i < v_h-1; i++)
	}//while(trigger == 1)

	i = peaks_detection_mis(pca->data_m, pca->motion_parametrs[2]/6, 4, &pca_g.motion.local_peaks, 20, 36, 0);

	change_m = 0;
	for(i = 1; i < DATA_SIZE_MIS-1; i++)
	{
		if((pca->data_m[i-1]-pca->data_m[i])*(pca->data_m[i]-pca->data_m[i+1]) < 0)	// Slope change in motion
			change_m++;
	}
	if(change_m > 7 && fabs(pca->roll_parametrs[0])*R2Df > 60.0 && abs(motion_peaks1.num[0] + motion_peaks1.num[1] - pca_g.vertical.peaks.num[0] - pca_g.vertical.peaks.num[1]) > 2)
		pca_g.switch_me = 1;
	if(motion_peaks1.num[0] > 2 && motion_peaks1.num[1] > 2)
	{
		for(i = 0; i < motion_peaks1.num[0];  i++)
		{
			min_samp[i] = 99;
			min_diff[i] = 99;
			for(j = 0 ; j < motion_peaks1.num[1]; j++)
			{
				if(abs(motion_peaks1.high_ind[j]-motion_peaks1.low_ind[i]) < min_samp[i])
				{
					min_samp[i] = (INT8)(abs(motion_peaks1.high_ind[j]-motion_peaks1.low_ind[i]));
					min_diff[i] = fabs(motion_peaks1.high_val[j]-motion_peaks1.low_val[i]);
				}
				if(j < (motion_peaks1.num[1]-1))
				{
					if(abs(motion_peaks1.high_ind[j] - motion_peaks1.high_ind[j+1]) < 5)
						pca_g.switch_me = 1;
				}
			}
			if(i < (motion_peaks1.num[0]-1))
			{
				if(abs(motion_peaks1.low_ind[i] - motion_peaks1.low_ind[i+1]) < 5)
					pca_g.switch_me = 1;
			}
		}
		if((motion_peaks1.num[0] + motion_peaks1.num[1]) > (pca_g.vertical.peaks.num[0]+pca_g.vertical.peaks.num[1]+2))
			pca_g.switch_me = 1;

		a = 99.0;
		for(i = 0; i < motion_peaks1.num[0]; i++)	// Comp_mc August 2013 Modified from i = 1; to i = 0;
		{
			if(min_diff[i] < a)
			{
				a = min_diff[i];				//[count1 b]=min(min_diff);
				count2 = min_samp[i];			// count2 = min_samp(b)
			}
		}
		if(a < 0.4 && count2 < 4 && fabs(pca->roll_parametrs[0])*R2Df > 60.0)
			pca_g.switch_me = 1;
	}

	if(motion_peaks1.num[0] == 0 || motion_peaks1.num[1] == 0)
		i= peaks_detection_mis(pca->data_m, delta/3, 4, &motion_peaks1, 20, 36, 0);

	// delete the unreasonable motion low peaks
	count1 = 0;
	count2 = 0;
	a = 99;
	c = -99;
	for(i = 0; i < motion_peaks1.num[0]; i++)
	{
		pca_g.motion.old_peaks.low_ind[i] = motion_peaks1.low_ind[i];
		pca_g.motion.old_peaks.low_val[i] = motion_peaks1.low_val[i];
		if(motion_peaks1.low_val[i] < pca->motion_parametrs[0])
		{
			pca_g.motion.peaks.low_ind[count1]   = motion_peaks1.low_ind[i];
			pca_g.motion.peaks.low_val[count1++] = motion_peaks1.low_val[i];
			if(motion_peaks1.low_val[i] < a)
				a = motion_peaks1.low_val[i];
			if(motion_peaks1.low_val[i] > c)
				c = motion_peaks1.low_val[i];
		}
	}
	pca_g.motion.motion_low_peaks_range = c-a;
	pca_g.motion.old_peaks.num[0] = motion_peaks1.num[0];
	//motion_peaks1.num[0] = count1;
	pca_g.motion.peaks.num[0] = count1;

	// delete the unreasonable motion high peaks
	a =  99;
	c = -99;
	for(i = 0; i < motion_peaks1.num[1]; i++)
	{
		pca_g.motion.old_peaks.high_ind[i] = motion_peaks1.high_ind[i];
		pca_g.motion.old_peaks.high_val[i] = motion_peaks1.high_val[i];
		if(motion_peaks1.high_val[i] > pca->motion_parametrs[0])
		{
			pca_g.motion.peaks.high_ind[count2]   = motion_peaks1.high_ind[i];
			pca_g.motion.peaks.high_val[count2++] = motion_peaks1.high_val[i];
			if(motion_peaks1.high_val[i] < a)
				a = motion_peaks1.high_val[i];
			if(motion_peaks1.high_val[i] > c)
				c = motion_peaks1.high_val[i];
		}
	}
	pca_g.motion.motion_high_peaks_range = c-a;
	pca_g.motion.old_peaks.num[1] = motion_peaks1.num[1];
	//motion_peaks1.num[1] = count2;
	pca_g.motion.peaks.num[1] = count2;

	//Crash protection, ZUPT
#ifdef METHOD_RESULTS_MISALIGNMENT
	if(pca->vertical_parametrs[2] < 0.8 || (pca_g.motion.peaks.num[0] + pca_g.motion.peaks.num[1]) > 20)
	{
		//My_Method_Worked[100] = 1;
		//My_Method_Num = 100;
		return 0;
	}
	else if(pca_g.vertical.peaks.num[0] == 0 || pca_g.vertical.peaks.num[1] == 0 || pca_g.motion.peaks.num[0] == 0 || pca_g.motion.peaks.num[1] == 0)
	{
		//My_Method_Worked[100] = 1;
		//My_Method_Num = 100;
		return 2;
	}
#else
	if(pca->vertical_parametrs[2] < 0.8 || (pca_g.motion.peaks.num[0] + pca_g.motion.peaks.num[1]) > 20)
		return 0;
	else if(pca_g.vertical.peaks.num[0] == 0 || pca_g.vertical.peaks.num[1] == 0 || pca_g.motion.peaks.num[0] == 0 || pca_g.motion.peaks.num[1] == 0)
		return 2;
#endif

	/*pca_g.motion.count[1] = 0;
	pca_g.motion.count[0] = 0;
	for(i = 0; i < pca_g.motion.peaks.num[1]; i++)
	{
	if(pca_g.motion.peaks.high_val[i] > (pca->motion_parametrs[0]+1.6*pca->motion_parametrs[1]))
	pca_g.motion.index_m_h[pca_g.motion.count[1]++] = i;
	}
	for(i = 0; i < pca_g.motion.peaks.num[0]; i++)
	{
	if(pca_g.motion.peaks.low_val[i] < (pca->motion_parametrs[0]-1.6*pca->motion_parametrs[1]))
	pca_g.motion.index_m_l[pca_g.motion.count[0]++] = i;
	}*/
	///////////////////////////////////
	i = peaks_detection_mis(pca->data_m, pca->motion_parametrs[2]/3, 4, &pca_g.motion.slow_peaks, 20, 36, 0);
	if(pca_g.motion.slow_peaks.num[0] == 0 || pca_g.motion.slow_peaks.num[1] == 0)
		i = peaks_detection_mis(pca->data_m, pca->motion_parametrs[2]/6, 4, &pca_g.motion.slow_peaks, 20, 36, 0);

	i = peaks_detection_mis(pca->data_v, pca->vertical_parametrs[2]/3, 4, &pca_g.vertical.slow_peaks, 20, 36, 0);
	if(pca_g.vertical.slow_peaks.num[0] == 0 || pca_g.vertical.slow_peaks.num[1] == 0)
		i = peaks_detection_mis(pca->data_v, pca->vertical_parametrs[2]/6, 4, &pca_g.vertical.slow_peaks, 20, 36, 0);

	/////////////////////////////////////
	//New code added on July 18, this is for Method 3.
	if(pca->horizontal_vertical_flag == 1)
	{
		range_att = pca->roll_parametrs[2]*D2Rf;
		for(i = 0; i < DATA_SIZE_MIS; i++)
			atti_vec[i] = data_r[i]-pca->roll_parametrs[0];
	}
	else  //pitch has bigger motion
	{
		range_att = pca->pitch_parametrs[2]*D2Rf;
		for(i = 0; i < DATA_SIZE_MIS; i++)
			atti_vec[i] = data_p[i]-pca->pitch_parametrs[0];
	}

	i = peaks_detection_mis(atti_vec, range_att/6, 4, &pca_g.attitude.peaks, 20, 36, 0);
	if(pca_g.attitude.peaks.num[0] == 0 || pca_g.attitude.peaks.num[1] == 0)
		i = peaks_detection_mis(atti_vec, range_att/8, 4, &pca_g.attitude.peaks, 20, 36, 0);

	// Average cycle needed by Xcorr_phase method
	i = peaks_detection_mis(pca->Acc_Mag, pca->acceleration_norm_parametrs[2]/4, 4, &pca_g.xcorr.peaks, 20, 36, 0);

	pca_g.xcorr.avg_cycle = 0.0;
	if(pca_g.vertical.peaks.num[0] > 1 && pca_g.vertical.peaks.num[1] > 1 && pca_g.xcorr.peaks.num[0] > 1 && pca_g.xcorr.peaks.num[1] > 1)
	{
		for(i = 0; i < pca_g.xcorr.peaks.num[1]-1; i++)
		{
			min_diff[i] =  (FLOAT32)(pca_g.xcorr.peaks.high_ind[i+1]- pca_g.xcorr.peaks.high_ind[i]);
			pca_g.xcorr.avg_cycle += min_diff[i];
		}
		for(i = 0; i < pca_g.xcorr.peaks.num[0]-1; i++)
		{
			if((i+pca_g.xcorr.peaks.num[1]-1) < 20)
			{
				min_diff[i+pca_g.xcorr.peaks.num[1]-1] = (FLOAT32)(pca_g.xcorr.peaks.low_ind[i+1]-pca_g.xcorr.peaks.low_ind[i]);
				pca_g.xcorr.avg_cycle += min_diff[i+pca_g.xcorr.peaks.num[1]-1];
			}
		}

		find_max_min_float_mis(min_diff, max_min,pca_g.xcorr.peaks.num[1]+pca_g.xcorr.peaks.num[0]-2);
		if((pca_g.xcorr.peaks.num[1]+pca_g.xcorr.peaks.num[0]-2) > 3 && (max_min[0]-max_min[1]) > 4)
		{
			//delete most unreasonable cycles estimation
			pca_g.xcorr.avg_cycle += - max_min[1]-max_min[0];
			pca_g.xcorr.avg_cycle /= (pca_g.xcorr.peaks.num[1]+pca_g.xcorr.peaks.num[0]-4);
		}
		else
			pca_g.xcorr.avg_cycle /= (pca_g.xcorr.peaks.num[1]+pca_g.xcorr.peaks.num[0]-2);

		//cross validate MAG signal with vertical signal
		//do not use Xcorr_2 when MAG and Vertical signal are obvious different
		if(abs(pca_g.xcorr.peaks.num[1]+pca_g.xcorr.peaks.num[0]-pca_g.vertical.peaks.num[1]-pca_g.vertical.peaks.num[0]) > 0)
		{
			if(pca_g.motion.peaks.num[1] > 1 && pca_g.motion.peaks.num[0] > 1)
			{
				pca_g.xcorr.avg_cycle = 0.0;
				for(i = 0; i < pca_g.motion.peaks.num[1]-1; i++)
				{
					min_diff[i] = (FLOAT32)(pca_g.motion.peaks.high_ind[i+1]-pca_g.motion.peaks.high_ind[i]);
					pca_g.xcorr.avg_cycle += min_diff[i];
				}
				for(i = 0; i < pca_g.motion.peaks.num[0]-1; i++)
				{
					if((i+pca_g.motion.peaks.num[1]-1) < 20)
					{
						min_diff[i+pca_g.motion.peaks.num[1]-1] = (FLOAT32)(pca_g.motion.peaks.low_ind[i+1]-pca_g.motion.peaks.low_ind[i]);
						pca_g.xcorr.avg_cycle += min_diff[i+pca_g.motion.peaks.num[1]-1];
					}
				}
				pca_g.xcorr.avg_cycle /= (pca_g.motion.peaks.num[1]+pca_g.motion.peaks.num[0]-2);
			}
		}
	}
	else   //use zero-crossing to estimate cycle
	{
		pca_g.xcorr.avg_cycle = 0;
		count1 = 0;
		count2 = 0;
		for(i = 0; i < DATA_SIZE_MIS-1; i++)
		{
			if(pca->data_v[i] > 0 && pca->data_v[i+1] < 0) //falling zero-crossing
			{
				zero_cro_fv[count1] = (fabs(pca->data_v[i])*(i+2)+fabs(pca->data_v[i+1])*(i+1))/(fabs(pca->data_v[i])+fabs(pca->data_v[i+1]));
				count1++;
			}

			if(pca->data_v[i] < 0 && pca->data_v[i+1] > 0) //rsing zero-crossing
			{
				zero_cro_rv[count2] = (fabs(pca->data_v[i])*(i+2)+fabs(pca->data_v[i+1])*(i+1))/(fabs(pca->data_v[i])+fabs(pca->data_v[i+1]));
				count2++;
			}
		}

		if((count1 < 2 && count2 == 0) || (count2 < 2 && count1 == 0))
			pca_g.xcorr.avg_cycle = 20;
		else
		{
			if(count1 == 1 && count2 == 1)
				pca_g.xcorr.avg_cycle = 2*fabs(zero_cro_fv[0]-zero_cro_rv[0]);
			else
			{
				for(i = 0; i < count1-1; i++)
					pca_g.xcorr.avg_cycle += zero_cro_fv[i+1]-zero_cro_fv[i];
				for(i = 0; i < count2-1; i++)
					pca_g.xcorr.avg_cycle += zero_cro_rv[i+1]-zero_cro_rv[i];

				pca_g.xcorr.avg_cycle /= (count1+count2-2);
			}
		}
	}

	if(fmod(pca_g.xcorr.avg_cycle, 1) >= 0.5)
		pca_g.xcorr.avg_cycle = ceil(pca_g.xcorr.avg_cycle);
	else
		pca_g.xcorr.avg_cycle = floor(pca_g.xcorr.avg_cycle);

	if(pca_g.xcorr.avg_cycle*2 <= DATA_SIZE_MIS) //data sample enough to one stride
		pca_g.xcorr.stride_size = (INT8)(pca_g.xcorr.avg_cycle*2);
	else
		pca_g.xcorr.stride_size = DATA_SIZE_MIS;

	for(i = 0; i < pca_g.xcorr.stride_size; i++)
	{
		pca_g.xcorr.x_drift[i] = pca->data_m[i]-pca->motion_parametrs[0];
		pca_g.xcorr.z_drift[i] = pca->data_v[i]-pca->vertical_parametrs[0];
	}

	for(i = 0; i < pca_g.vertical.peaks.num[1]-1; i++)
	{
		dis_vh[i] = pca_g.vertical.peaks.high_ind[i+1] - pca_g.vertical.peaks.high_ind[i];
		if(dis_vh[i] > max_dis_vh)
			max_dis_vh = (INT8)(abs(dis_vh[i]));
	}

	for(i = 0; i < pca_g.vertical.peaks.num[0]-1; i++)
	{
		dis_vl[i] = pca_g.vertical.peaks.low_ind[i+1] - pca_g.vertical.peaks.low_ind[i];
		if(dis_vl[i] > max_dis_vl)
			max_dis_vl = (INT8)(abs(dis_vl[i]));
	}
	if(abs(max_dis_vh - max_dis_vl) < 3 && max_dis_vh != -99 && max_dis_vl != -99)
		pca_g.i_chest = 1;

	if(pca_m->device_flag != 3)
		temp_res = general_phone_tablet_backward_forward_mis(&pca_g, pca, pca_m);
	else
		temp_res = general_watch_backward_forward_mis(&pca_g, pca, pca_m);
	return temp_res;
}
/**
* @brief	This is phone and tablet general backward forward function to solve the 180
* 			amiguity problem in case of phone and tablet general data.
*
* @details	This function takes 3 pointers of structures of type PCA, PCA_M, and PCA_D.
* 			It calls different sub-functions to get the decision of Forward/Backward in
* 			the case of phone and tablet general data.
*
* @param[in]	pca_g	pointer to a structure of type \c PCA_G.
* @param[in]	pca  	pointer to a structure of type \c PCA.
* @param[in]	pca_m  	pointer to a structure of type \c PCA_M.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 general_phone_tablet_backward_forward_mis(PCA_G_Ptr pca_g, PCA_Ptr pca, PCA_M_Ptr pca_m)
{
	INT8 i, j;
	INT8 temp_res = 2;
	INT8 slow_undetect_tag = 0;
	INT8 min_vsp = 20, max_vsp = -20;
	INT8 m_peaks_count = 0;
	INT8 ml_close_counter = 0, mh_close_counter = 0;
	INT8 mh_high_count = 0, ml_low_count  = 0;

	FLOAT32 tem_mean = 0;
	FLOAT32 std_att = 0.0, range_att = 0.0;
	FLOAT32 amp_peak_vh = 0.0;
	FLOAT32 min_vh = 99.0, max_vh = -99.0, max_vh_val = -99.0, min_vh_val = 99.0;
	FLOAT32 diff_ml = 0.0, diff_mh = 0.0;

	if(pca->horizontal_vertical_flag == 0)
		tem_mean = floorf((FLOAT32)(fabs(pca->roll_parametrs[0]*R2Df)));
	else
		tem_mean = floorf((FLOAT32)(fabs(pca->roll_parametrs[0]*R2Df)+90.0));

	for(i = 0; i < pca_g->motion.peaks.num[1]; i++)
	{
		if(pca_g->motion.peaks.high_val[i] > pca_g->motion.thresholds[3])
			mh_high_count++;
	}
	for(i = 0; i < pca_g->motion.peaks.num[0]; i++)
	{
		if(pca_g->motion.peaks.low_val[i] < pca_g->motion.thresholds[2])
			ml_low_count++;
	}

	// Method 1
	if(pca_m->height_change_flag != 0)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		if(pca_m->vertical_motion_phase_shift > 110.0 && (pca_m->vertical_motion_phase_shift - pca_m->vertical_lateral_phase_shift) > 30.0)
		{
			//My_Method_Worked[101] = 1;
			//My_Method_Num = 101;
			return 1;
		}
		else if(pca_m->vertical_motion_phase_shift < 70.0 && (pca_m->vertical_lateral_phase_shift - pca_m->vertical_motion_phase_shift) > 30.0)
		{
			//My_Method_Worked[101] = 1;
			//My_Method_Num = 101;
			return 0;
		}
#else
		if(pca_m->vertical_motion_phase_shift > 110.0 && (pca_m->vertical_motion_phase_shift - pca_m->vertical_lateral_phase_shift) > 30.0)
			return 1;
		else if(pca_m->vertical_motion_phase_shift < 70.0 && (pca_m->vertical_lateral_phase_shift - pca_m->vertical_motion_phase_shift) > 30.0)
			return 0;
#endif
	}
	// End of Method 1

	// Method 2		general_purse()
	/***Xcorr_phase method with strict condition to process purse only***/
	if(pca->motion_parametrs[2] > 6 && pca->motion_parametrs[2] > pca->vertical_parametrs[2] && pca_g->xcorr.avg_cycle > 3.0 && pca->roll_parametrs[1] < 0.4 && pca->pitch_parametrs[1] < 0.4 && pca_m->lateral_range < 6)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[102] = 1;
		temp_res = general_phone_tablet_1_mis(pca_g, pca, 155.0, 25.0, -20.0, -160.0);
		if(temp_res != 2)
		{
			//My_Method_Num = 102;
			return temp_res;
		}
#else
		temp_res = general_phone_tablet_1_mis(pca_g, pca, 155.0, 25.0, -20.0, -160.0);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 2

	// Method 3
	//Added on July 20th, 2012 deal with light dangling

	if(fabs(pca->roll_parametrs[0]*R2Df) > 60.0 && fabs(pca->roll_parametrs[0]*R2Df) < 120.0 && pca->pitch_parametrs[2] > 30.0)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[103] = 1;
		temp_res = general_phone_tablet_2_mis(pca_g, pca);
		if(temp_res != 2)
		{
			//My_Method_Num = 103;
			return temp_res;
		}
#else
		temp_res = general_phone_tablet_2_mis(pca_g, pca);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 3

	// Method 4
	//Added on June 26th, 2012 deal with the slow motion
	if(pca->vertical_parametrs[2] < 4 && pca->motion_parametrs[2] < 3 && (pca_g->motion.slow_peaks.num[0] + pca_g->motion.slow_peaks.num[1]) < 8 && (pca_g->vertical.slow_peaks.num[0] + pca_g->vertical.slow_peaks.num[1]) < 8)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[104] = 1;
		temp_res = general_phone_tablet_3_mis(pca_g, pca);
		if(temp_res != 2)
		{
			//My_Method_Num = 104;
			return temp_res;
		}
#else
		temp_res = general_phone_tablet_3_mis(pca_g, pca);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 4

	// Method 5
	//Added on March 31, 2014
	if(pca_m->height_change_flag == 0)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[105] = 1;
		if(pca_m->vertical_motion_angle > 45.0)
		{
			//My_Method_Num = 105;
			return 0;
		}
		else if(pca_m->vertical_motion_angle < -45.0)
		{
			//My_Method_Num = 105;
			return 1;
		}
#else
		if(pca_m->vertical_motion_angle > 45.0)
			return 0;
		else if(pca_m->vertical_motion_angle < -45.0)
			return 1;
#endif
	}
	// End of Method 5

#ifndef COM_WALK_SIMPLIFIED
	// Method 6
	//Added on April 10th, 2014 solve the problem with texting portrait or landscape with little tilt
	if((fabs(pca->roll_parametrs[0]*R2Df) < 40 || fabs(pca->roll_parametrs[0]*R2Df) > 140) && fabs(pca->pitch_parametrs[0]*R2Df) < 50 && pca->horizontal_vertical_flag == 0 && pca_m->height_change_flag == 0)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[106] = 1;
		if((pca_m->vertical_lateral_phase_shift - pca_m->vertical_motion_phase_shift) > 30.0)
		{
			//My_Method_Num = 106;
			return 0;
		}
		else if((pca_m->vertical_motion_phase_shift- pca_m->vertical_lateral_phase_shift) > 30.0)
		{
			//My_Method_Num = 106;
			return 1;
		}
#else
		if((pca_m->vertical_lateral_phase_shift - pca_m->vertical_motion_phase_shift) > 30.0)
			return 0;
		else if((pca_m->vertical_motion_phase_shift- pca_m->vertical_lateral_phase_shift) > 30.0)
			return 1;
#endif
	}
	// End of Method 6
#endif
	// Methods 7 and 8
	//Start with Simple Method I: strictly matching method, vh-mh, vl-ml within one sample, Revised condition on July 3.
	// Condition is modified on Augus 2nd make it only for compass
	max_vh = -99.0;
	min_vh =  99.0;
	for(i = 0; i < pca_g->vertical.peaks.num[1]; i++)
	{
		if(pca_g->vertical.peaks.high_val[i] < min_vh)
			min_vh = pca_g->vertical.peaks.high_val[i];
		if(pca_g->vertical.peaks.high_val[i] > max_vh)
			max_vh = pca_g->vertical.peaks.high_val[i];
	}
	if(fabs(max_vh - min_vh) < 0.8 && pca->vertical_parametrs[2] < 5.5 && pca->motion_parametrs[2] < pca->vertical_parametrs[2] && MAX(pca->roll_parametrs[1], pca->pitch_parametrs[1]) < 2.5 && pca_m->horizontal_acceleration_range < 3)
	{
		// Method 7			general_purse()
		if(pca_g->xcorr.avg_cycle > 0)
		{
#ifdef METHOD_RESULTS_MISALIGNMENT
			//My_Method_Worked[107] = 1;
			temp_res = general_phone_tablet_1_mis(pca_g, pca, 95.0, 85.0, -85.0, -95.0);
			if(temp_res != 2)
			{
				//My_Method_Num = 107;
				return temp_res;
			}
#else
			temp_res = general_phone_tablet_1_mis(pca_g, pca, 95.0, 85.0, -85.0, -95.0);
			if(temp_res != 2)
				return temp_res;
#endif
		}
		// End of Method 7

		// Method 8			general_compass_simple_method_1()
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[108] = 1;
		temp_res = general_phone_tablet_14_mis(pca_g, pca);
		if(temp_res != 2)
		{
			//My_Method_Num = 108;
			return temp_res;
		}
#else
		temp_res = general_phone_tablet_14_mis(pca_g, pca);
		if(temp_res != 2)
			return temp_res;
#endif
		// End of Method 8
	}
	// End of Methods 7 and 8

#ifndef COM_WALK_SIMPLIFIED
	// Method 9				general_compass_motion_big_amplitude()
	//Added on May 29th, 2012 New version to deal with Compass special when motion signal have big amplitude  2012.07.06
	if((fabs(pca->roll_parametrs[0]*R2Df) <= 50.0 || fabs(pca->roll_parametrs[0]*R2Df) >= 130.0) && fabs(pca->pitch_parametrs[0]*R2Df) <= 50.0 && pca->horizontal_vertical_flag == 0)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[109] = 1;
		temp_res = general_phone_tablet_4_mis(pca_g, pca);
		if(temp_res != 2)
		{
			//My_Method_Num = 109;
			return temp_res;
		}
#else
		temp_res = general_phone_tablet_4_mis(pca_g, pca);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 9
#endif

	// Method 10					general_light_dangling_2()
	// Added August 2nd, 2012 deal with the light dangling
	for(i = 0; i < pca_g->vertical.peaks.num[1]; i++)
	{
		if(pca_g->vertical.peaks.high_val[i] > max_vh_val)
			max_vh_val = pca_g->vertical.peaks.high_val[i];
		if(pca_g->vertical.peaks.high_val[i] < min_vh_val)
			min_vh_val = pca_g->vertical.peaks.high_val[i];
	}
	amp_peak_vh = max_vh_val - min_vh_val;

	//***New index for distinguish light dangling and slow belt
	if(pca_g->vertical.peaks.num[1] > 1 && pca_g->vertical.peaks.num[0] > 1)
	{
		if(pca_g->vertical.peaks.high_ind[0] < pca_g->vertical.peaks.low_ind[0])
		{
			for(i = 0; i < pca_g->vertical.peaks.num[1]-1; i++)
			{
				for(j = 0; j < pca_g->vertical.peaks.num[0]; j++)
				{
					if(pca_g->vertical.peaks.low_ind[j] > pca_g->vertical.peaks.high_ind[i] && pca_g->vertical.peaks.low_ind[j] < pca_g->vertical.peaks.high_ind[i+1])
					{
						if(min_vsp  > MIN(pca_g->vertical.peaks.low_ind[j] - pca_g->vertical.peaks.high_ind[i], pca_g->vertical.peaks.high_ind[i+1]-pca_g->vertical.peaks.low_ind[j]))
							min_vsp = MIN(pca_g->vertical.peaks.low_ind[j] - pca_g->vertical.peaks.high_ind[i], pca_g->vertical.peaks.high_ind[i+1]-pca_g->vertical.peaks.low_ind[j]);
						if(max_vsp  < MAX(pca_g->vertical.peaks.low_ind[j] - pca_g->vertical.peaks.high_ind[i], pca_g->vertical.peaks.high_ind[i+1]-pca_g->vertical.peaks.low_ind[j]))
							max_vsp = MAX(pca_g->vertical.peaks.low_ind[j] - pca_g->vertical.peaks.high_ind[i], pca_g->vertical.peaks.high_ind[i+1]-pca_g->vertical.peaks.low_ind[j]);
						// Comp_mc August 2013 Modified: braces adjust to include the below lines in j loope
						if(pca_g->vertical.peaks.low_ind[j] > pca_g->vertical.peaks.high_ind[pca_g->vertical.peaks.num[1]-1] && (pca_g->vertical.peaks.low_ind[j] - pca_g->vertical.peaks.high_ind[pca_g->vertical.peaks.num[1]-1]) < min_vsp)
							min_vsp = pca_g->vertical.peaks.low_ind[j] - pca_g->vertical.peaks.high_ind[pca_g->vertical.peaks.num[1]-1];
						if(pca_g->vertical.peaks.low_ind[j] > pca_g->vertical.peaks.high_ind[pca_g->vertical.peaks.num[1]-1] && (pca_g->vertical.peaks.low_ind[j] - pca_g->vertical.peaks.high_ind[pca_g->vertical.peaks.num[1]-1]) > max_vsp)
							max_vsp = pca_g->vertical.peaks.low_ind[j] - pca_g->vertical.peaks.high_ind[pca_g->vertical.peaks.num[1]-1];
					}
				}
			}
		}
		else
		{
			for(i = 0; i < pca_g->vertical.peaks.num[0]-1; i++)
			{
				for(j = 0; j < pca_g->vertical.peaks.num[1]; j++)
				{
					if(pca_g->vertical.peaks.high_ind[j] > pca_g->vertical.peaks.low_ind[i] && pca_g->vertical.peaks.high_ind[j] < pca_g->vertical.peaks.low_ind[i+1])
					{
						if(min_vsp  > MIN(pca_g->vertical.peaks.high_ind[j] - pca_g->vertical.peaks.low_ind[i],pca_g->vertical.peaks.low_ind[i+1]-pca_g->vertical.peaks.high_ind[j]))
							min_vsp = MIN(pca_g->vertical.peaks.high_ind[j] - pca_g->vertical.peaks.low_ind[i],pca_g->vertical.peaks.low_ind[i+1]-pca_g->vertical.peaks.high_ind[j]);
						if(max_vsp  < MAX(pca_g->vertical.peaks.high_ind[j] - pca_g->vertical.peaks.low_ind[i],pca_g->vertical.peaks.low_ind[i+1]-pca_g->vertical.peaks.high_ind[j]))
							max_vsp = MAX(pca_g->vertical.peaks.high_ind[j] - pca_g->vertical.peaks.low_ind[i],pca_g->vertical.peaks.low_ind[i+1]-pca_g->vertical.peaks.high_ind[j]);
						// Comp_mc August 2013 Modified: braces adjust to include the below lines in j loope
						if(pca_g->vertical.peaks.high_ind[j] > pca_g->vertical.peaks.low_ind[pca_g->vertical.peaks.num[0]-1] && (pca_g->vertical.peaks.high_ind[j] - pca_g->vertical.peaks.low_ind[pca_g->vertical.peaks.num[0]-1]) < min_vsp)
							min_vsp = pca_g->vertical.peaks.high_ind[j] - pca_g->vertical.peaks.low_ind[pca_g->vertical.peaks.num[0]-1];
						if(pca_g->vertical.peaks.high_ind[j] < pca_g->vertical.peaks.low_ind[pca_g->vertical.peaks.num[0]-1] && (pca_g->vertical.peaks.high_ind[j] - pca_g->vertical.peaks.low_ind[pca_g->vertical.peaks.num[0]-1]) > max_vsp)
							max_vsp = pca_g->vertical.peaks.high_ind[j] - pca_g->vertical.peaks.low_ind[pca_g->vertical.peaks.num[0]-1];
					}
				}
			}
		}
	}

	if(pca_g->i_chest == 1 && fabs(pca->roll_parametrs[0]*R2Df) > 60.0 && fabs(pca->roll_parametrs[0]*R2Df) < 120.0 && fabs(pca->pitch_parametrs[0]*R2Df) < 25.0 && (fabs(pca->pitch_parametrs[0]*R2Df) > 10.0 || (pca->pitch_parametrs[2] > 6.2 && pca->roll_parametrs[2] > 5)) && (min_vsp >= 5 || max_vsp < 8) && amp_peak_vh < 1.2 && pca->vertical_parametrs[2] < 7.8 && pca->motion_parametrs[2] < 5.5)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[110] = 1;
		temp_res = general_phone_tablet_5_mis(pca_g, pca);
		if(temp_res != 2)
		{
			//My_Method_Num = 110;
			return temp_res;
		}
#else
		temp_res = general_phone_tablet_5_mis(pca_g, pca);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 10

	// Method 11           general_slow_belt()
	//New algorithm developed on June 28, 2012. Especially for Belt data under slow motion, Revised on July 18, 2012.
	//Deal with slow belt and light belt Revised on Aug 8th, 2012. Deal with ligth dangling and slow belt
	// Condition is modified by Abdelrahman : August 2nd.
	if(pca_g->vertical.peaks.num[0] < 5 && pca->motion_parametrs[1] > 0.8 && MAX(pca->roll_parametrs[2], pca->pitch_parametrs[2]) > 4.5 && pca->vertical_parametrs[2] < 8 && ((pca->horizontal_vertical_flag == 0 && fabs(pca->roll_parametrs[0]*R2Df) > 70.0  && fabs(pca->roll_parametrs[0]*R2Df) < 110.0 && pca_g->vertical.slow_peaks.num[0] != 0) || (pca->horizontal_vertical_flag != 0 && pca_g->i_chest != 1 && pca_g->vertical.slow_peaks.num[0] != 0)))
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[111] = 1;
		temp_res = general_phone_tablet_6_mis(pca_g, pca);
		if(temp_res != 2)
		{
			//My_Method_Num = 111;
			return temp_res;
		}
#else
		temp_res = general_phone_tablet_6_mis(pca_g, pca);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 11

	// Method 12              general_slow_compass()
	//Enhancement part for slow motion algorithm. Added 2012.06.12
	if(pca->vertical_parametrs[2] < 3.5 && (pca_g->vertical.peaks.num[1]+pca_g->vertical.peaks.num[0]) <= 8) // Condition for slow motion detection
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[112] = 1;
		temp_res = general_phone_tablet_7_mis(pca_g);
		if(temp_res != 2)
		{
			//My_Method_Num = 112;
			return temp_res;
		}
		else
			slow_undetect_tag = 1;  //slow motion mode still cannot been detected, them use normal code but reverse theory
#else
		temp_res = general_phone_tablet_7_mis(pca_g);
		if(temp_res != 2)
			return temp_res;
		else
			slow_undetect_tag = 1;  //slow motion mode still cannot been detected, them use normal code but reverse theory
#endif
	}
	// End of Method 12

	// Method 13     general_light_dangling_3()
	//Added, May 29th, 2012: deal with the light dangling
	if(pca->roll_parametrs[2] > pca->pitch_parametrs[2] && pca->roll_parametrs[2] < 340.0) //roll has bigger motion
	{
		range_att = pca->roll_parametrs[2];
		std_att   = pca->roll_parametrs[1];
	}
	else  //pitch has bigger motion
	{
		range_att = pca->pitch_parametrs[2];
		std_att   = pca->pitch_parametrs[1];
	}
	if((pca_g->motion.peaks.num[1]+pca_g->motion.peaks.num[0]) <= 3 && range_att > 8.0 && std_att >= 0.04 && fabs(pca->roll_parametrs[0]*R2Df) > 60.0 && fabs(pca->roll_parametrs[0]*R2Df) < 120.0)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[113] = 1;
		temp_res = general_phone_tablet_8_mis(pca_g);
		if(temp_res == 0)
		{
			//My_Method_Num = 113;
			return slow_undetect_tag;
		}
		else if(temp_res == 1)
		{
			//My_Method_Num = 113;
			return !(slow_undetect_tag);
		}
#else
		temp_res = general_phone_tablet_8_mis(pca_g);
		if(temp_res == 0)
			return slow_undetect_tag;
		else if(temp_res == 1)
			return !(slow_undetect_tag);
#endif
	}
	// End of Method 13

	// Method 14, 15, 16, and 17
	//Added on 2012.05.18 New version using Ear special Method I and Method II. This is when motion signal have big amplitude.
	//This new version also combined with Simple Method I.
	if(fabs(pca->roll_parametrs[0]*R2Df) > 60.0 && fabs(pca->pitch_parametrs[0]*R2Df) > 30.0 && pca->motion_parametrs[2] > 4.5)
	{
		// Method 14          general_special_ear_1()
		//Start with Simple Method I: strictly matching method, vh-mh, vl-ml within one sample
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[114] = 1;
		temp_res = general_phone_tablet_9_mis(pca_g);
		if(temp_res != 2 && slow_undetect_tag == 1)
		{
			//My_Method_Num = 114;
			return !(temp_res);
		}
		else if(temp_res != 2 && slow_undetect_tag == 0)
		{
			//My_Method_Num = 114;
			return temp_res;
		}
#else
		temp_res = general_phone_tablet_9_mis(pca_g);
		if(temp_res != 2 && slow_undetect_tag == 1)
			return !(temp_res);
		else if(temp_res != 2 && slow_undetect_tag == 0)
			return temp_res;
#endif
		// End of Method 14

		// Method 15, 16, and 17
		if(mh_high_count != 0 && ml_low_count != 0)
		{
			// Method 15             general_special_ear_2()
			//Start to use Ear Special Method II if it is still undetect
#ifdef METHOD_RESULTS_MISALIGNMENT
			//My_Method_Worked[115] = 1;
			temp_res = general_phone_tablet_10_mis(pca_g, pca);
			if(temp_res == 0)
			{
				//My_Method_Num = 115;
				return slow_undetect_tag;
			}
			else if(temp_res == 1)
			{
				//My_Method_Num = 115;
				return !(slow_undetect_tag);
			}
#else
			temp_res = general_phone_tablet_10_mis(pca_g, pca);
			if(temp_res == 0)
				return slow_undetect_tag;
			else if(temp_res == 1)
				return !(slow_undetect_tag);
#endif
			// End of Method 15

			// Method 16             general_special_ear_3()
			//Specail case Method I: use local bump area to determin direction, bigger positive bump corresponding to backward, vice versa.
#ifdef METHOD_RESULTS_MISALIGNMENT
			//My_Method_Worked[116] = 1;
			temp_res = general_phone_tablet_11_mis(pca_g, pca);
			if(temp_res == 0)
			{
				//My_Method_Num = 116;
				return slow_undetect_tag;
			}
			else if(temp_res == 1)
			{
				//My_Method_Num = 116;
				return !(slow_undetect_tag);
			}
#else
			temp_res = general_phone_tablet_11_mis(pca_g, pca);
			if(temp_res == 0)
				return slow_undetect_tag;
			else if(temp_res == 1)
				return !(slow_undetect_tag);
#endif
			// End of Method 16
		}
		// Method 17
#ifdef METHOD_RESULTS_MISALIGNMENT
		else if(mh_high_count > 0 && ml_low_count == 0)
		{
			//My_Method_Worked[117] = 1;
			//My_Method_Num = 117;
			return slow_undetect_tag;
		}
		else if(mh_high_count == 0 && ml_low_count > 0)
		{
			//My_Method_Worked[117] = 1;
			//My_Method_Num = 117;
			return !(slow_undetect_tag);
		}
#else
		else if(mh_high_count > 0 && ml_low_count == 0)
			return slow_undetect_tag;
		else if(mh_high_count == 0 && ml_low_count > 0)
			return !(slow_undetect_tag);
#endif
		// End of Method 17
		// End of Method 15, 16, and 17
	}
	// End of Method 14, 15, 16, and 17

	// Method 18          general_belt_reverse_theory()
	//special dealing Belt data when it has pocket patterm but reverse theory 2012.05.05.
	//Revised from (max_vh - min_vh) > 1.4 || MAX(pca_mis.motion.vec[6], pca_mis.motion.vec[5]) > 1.6
	//      	to (max_vh - min_vh) > 1.4 && MAX(pca_mis.motion.vec[6], pca_mis.motion.vec[5]) > 1.6 on May 31th
	//    	reason: avoid ear data set in dragon go to this one, Condition is modified on Augus 2nd 2012
	if(pca_g->vertical.peaks.num[1] > 1 && amp_peak_vh > 1.4 && MAX(pca_g->motion.motion_low_peaks_range, pca_g->motion.motion_high_peaks_range) > 1.6 && pca->motion_parametrs[2] > 4 && pca->motion_parametrs[1] > 1 &&  pca_g->motion.local_peaks.num[1] != 0 && pca_g->motion.local_peaks.num[0] != 0 && fabs(pca->roll_parametrs[0]*R2Df) > 60 && fabs(pca->roll_parametrs[0]*R2Df) < 120.0)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[118] = 1;
		temp_res = general_phone_tablet_12_mis(pca_g);
		if(temp_res == 0)
		{
			//My_Method_Num = 118;
			return slow_undetect_tag;
		}
		else if(temp_res == 1)
		{
			//My_Method_Num = 118;
			return !(slow_undetect_tag);
		}
#else
		temp_res = general_phone_tablet_12_mis(pca_g);
		if(temp_res == 0)
			return slow_undetect_tag;
		else if(temp_res == 1)
			return !(slow_undetect_tag);
#endif
	}
	// End Method 18

#ifdef COM_WALK_SIMPLIFIED
	// sign Method  19
	//Added on March 31, 2014
	if(pca_m->device_flag == 1)	// for Tablet
	{
		if(fabs(pca_m->vertical_motion_angle) > 20.0)
		{
#ifdef METHOD_RESULTS_MISALIGNMENT
			//My_Method_Worked[119] = 1;
			if(pca_m->vertical_motion_angle > 20)
			{
				//My_Method_Num = 119;
				return slow_undetect_tag;
			}
			else if(pca_m->vertical_motion_angle < -20)
			{
				//My_Method_Num = 119;
				return !(slow_undetect_tag);
			}
#else
			if(pca_m->vertical_motion_angle > 20)
				return slow_undetect_tag;
			else if(pca_m->vertical_motion_angle < -20)
				return !(slow_undetect_tag);
#endif
		}
	}
	// End of sign Method  19
#else
	//////////////////////////////////////////////////
	// Method 19
	//Added on March 31, 2014
	if(pca_m->height_change_flag == 0)
	{
		if(pca_m->device_flag == 0)		// for Phone
		{
			if(fabs(pca_m->vertical_motion_phase- 90.0) > 30.0 && fabs(pca_m->vertical_motion_angle) < 20.0)
			{
#ifdef METHOD_RESULTS_MISALIGNMENT
				//My_Method_Worked[119] = 1;
				if(pca_m->motion_effective_coefficient > 0.4 && (pca_m->vertical_motion_phase- 90.0) < -30.0)
				{
					//My_Method_Num = 119;
					return !(slow_undetect_tag);
				}
				else if(pca_m->motion_effective_coefficient < -0.4 && (pca_m->vertical_motion_phase- 90.0) > 30.0)
				{
					//My_Method_Num = 119;
					return slow_undetect_tag;
				}
#else
				if(pca_m->motion_effective_coefficient > 0.4 && (pca_m->vertical_motion_phase- 90.0) < -30.0)
					return !(slow_undetect_tag);
				else if(pca_m->motion_effective_coefficient < -0.4 && (pca_m->vertical_motion_phase- 90.0) > 30.0)
					return slow_undetect_tag;
#endif
			}
		}
		else if(pca_m->device_flag == 1)	// for Tablet
		{
			if(fabs(pca_m->vertical_motion_phase- 90.0) > 30.0 && fabs(pca_m->vertical_motion_angle) < 35.0)
			{
#ifdef METHOD_RESULTS_MISALIGNMENT
				//My_Method_Worked[119] = 1;
				if(pca_m->motion_effective_coefficient > 0.0)
				{
					//My_Method_Num = 119;
					return !(slow_undetect_tag);
				}
				else
				{
					//My_Method_Num = 119;
					return slow_undetect_tag;
				}
#else
				if(pca_m->motion_effective_coefficient > 0.0)
					return !(slow_undetect_tag);
				else
					return slow_undetect_tag;
#endif
			}
			else if(fabs(pca_m->vertical_motion_angle) > 20.0)
			{
#ifdef METHOD_RESULTS_MISALIGNMENT
				//My_Method_Worked[119] = 1;
				if(pca_m->vertical_motion_angle > 20)
				{
					//My_Method_Num = 119;
					return slow_undetect_tag;
				}
				else if(pca_m->vertical_motion_angle < -20)
				{
					//My_Method_Num = 119;
					return !(slow_undetect_tag);
				}
#else
				if(pca_m->vertical_motion_angle > 20)
					return slow_undetect_tag;
				else if(pca_m->vertical_motion_angle < -20)
					return !(slow_undetect_tag);
#endif
			}
			else if(fabs(pca_m->vertical_motion_phase- 90.0) > 5.0)
			{
#ifdef METHOD_RESULTS_MISALIGNMENT
				//My_Method_Worked[119] = 1;
				if(pca_m->motion_effective_coefficient > 0.0)
				{
					//My_Method_Num = 119;
					return !(slow_undetect_tag);
				}
				else
				{
					//My_Method_Num = 119;
					return slow_undetect_tag;
				}
#else
				if(pca_m->motion_effective_coefficient > 0.0)
					return !(slow_undetect_tag);
				else
					return slow_undetect_tag;
#endif
			}
		}
	}
	// End of Method 19
# endif

#ifndef COM_WALK_SIMPLIFIED
	// Method 20			general_belt_reverse_pocket()
#ifdef METHOD_RESULTS_MISALIGNMENT
	//My_Method_Worked[120] = 1;
	temp_res = general_phone_tablet_13_mis(pca_g, pca);
	if(temp_res == 0)
	{
		//My_Method_Num = 120;
		return slow_undetect_tag;
	}
	else if(temp_res == 1)
	{
		//My_Method_Num = 120;
		return !(slow_undetect_tag);
	}
#else
	temp_res = general_phone_tablet_13_mis(pca_g, pca);
	if(temp_res == 0)
		return slow_undetect_tag;
	else if(temp_res == 1)
		return !(slow_undetect_tag);
#endif
	// End of Method 20
#endif

	// Method 21				general_purse()
	/***Xcorr_phase method, #9***/
	if(pca_g->xcorr.avg_cycle > 3 && pca->motion_parametrs[2] < 5 && pca->vertical_parametrs[2] < 6 && !(fabs(pca->roll_parametrs[0]*R2Df) > 40.0 && fabs(pca->roll_parametrs[0]*R2Df) < 140 && pca_m->horizontal_acceleration_range > 2))
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[121] = 1;
		temp_res = general_phone_tablet_1_mis(pca_g, pca, 135.0, 45.0, -45.0, -135.0);
		if(temp_res == 0)
		{
			//My_Method_Num = 121;
			return slow_undetect_tag;
		}
		else if(temp_res == 1)
		{
			//My_Method_Num = 121;
			return !(slow_undetect_tag);
		}
#else
		temp_res = general_phone_tablet_1_mis(pca_g, pca, 135.0, 45.0, -45.0, -135.0);
		if(temp_res == 0)
			return slow_undetect_tag;
		else if(temp_res == 1)
			return !(slow_undetect_tag);
#endif
	}
	// END of Method 21

	// Method 22                general_compass_simple_method_1()
	//Start with Simple Method I: strictly matching method, vh-mh, vl-ml within one sample, Revised condition on July 3.
	// Condition is modified on Augus 2nd, make it only for compass
	if(amp_peak_vh < 0.8 && !(tem_mean > 60.0 && tem_mean < 120.0) && pca->motion_parametrs[2] < pca->vertical_parametrs[2] && (pca->vertical_parametrs[2]/pca->motion_parametrs[2]) > 1.4)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[122] = 1;
		temp_res = general_phone_tablet_14_mis(pca_g, pca);
		if(temp_res == 0)
		{
			//My_Method_Num = 122;
			return slow_undetect_tag;
		}
		else if(temp_res == 1)
		{
			//My_Method_Num = 122;
			return !(slow_undetect_tag);
		}
#else
		temp_res = general_phone_tablet_14_mis(pca_g, pca);
		if(temp_res == 0)
			return slow_undetect_tag;
		else if(temp_res == 1)
			return !(slow_undetect_tag);
#endif
	}
	// End of Method 22

	// Method 23				general_special_belt()
	// Added May 31th, 2012 deal with Amir Belt
	if(fabs(pca->roll_parametrs[0]*R2Df) > 40.0 && fabs(pca->roll_parametrs[0]*R2Df) < 140.0 && pca->vertical_parametrs[2] < 5.0)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[123] = 1;
		temp_res = general_phone_tablet_15_mis(pca_g, pca);
		if(temp_res == 0)
		{
			//My_Method_Num = 123;
			return slow_undetect_tag;
		}
		else if(temp_res == 1)
		{
			//My_Method_Num = 123;
			return !(slow_undetect_tag);
		}
#else
		temp_res = general_phone_tablet_15_mis(pca_g, pca);
		if(temp_res == 0)
			return slow_undetect_tag;
		else if(temp_res == 1)
			return !(slow_undetect_tag);
#endif
	}
	// End of Method 23

	// Method 24               general_unstable_mixed_method()
	//Unstable Mixed method,  May 18 2012
	//Revised: judgement became adding results from both part. Original use priority
	if(pca_g->switch_me == 1 && fabs(pca->roll_parametrs[0]*R2Df) > 60.0 && pca->motion_parametrs[2] < 6.0)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[124] = 1;
		temp_res = general_phone_tablet_16_mis(pca_g, pca);
		if(temp_res == 0)
		{
			//My_Method_Num = 124;
			return slow_undetect_tag;
		}
		else if(temp_res == 1)
		{
			//My_Method_Num = 124;
			return !(slow_undetect_tag);
		}
#else
		temp_res = general_phone_tablet_16_mis(pca_g, pca);
		if(temp_res == 0)
			return slow_undetect_tag;
		else if(temp_res == 1)
			return !(slow_undetect_tag);
#endif
	}
	// End of Method 24

	// Method 25 and 26
	if(pca_g->motion.peaks.num[1] > 1 && pca_g->motion.peaks.num[0] > 1 && pca->motion_parametrs[2] < 5.0)
	{
		// Method 25			general_compass_simple_method_jacket()
		// Added August 2013 Motion signal is small compared to vertical signal
		if((pca->vertical_parametrs[2]/pca->motion_parametrs[2]) > 1.5 && pca_m->lateral_range > 2.0)
		{
#ifdef METHOD_RESULTS_MISALIGNMENT
			//My_Method_Worked[125] = 1;
			temp_res = general_phone_tablet_17_mis(pca_g, pca);
			if(temp_res == 0)
			{
				//My_Method_Num = 125;
				return slow_undetect_tag;
			}
			else if(temp_res == 1)
			{
				//My_Method_Num = 125;
				return !(slow_undetect_tag);
			}
#else
			temp_res = general_phone_tablet_17_mis(pca_g, pca);
			if(temp_res == 0)
				return slow_undetect_tag;
			else if(temp_res == 1)
				return !(slow_undetect_tag);
#endif
		}
		// End of Method 25

		// Method 26				general_compass_simple_method_2()
		// Simple method I increase possibility then peaks are perfectly match(usually in compass mode) 2012.05.08
		diff_ml = -99.0;
		diff_mh = -99.0;
		//calculate difference summation for both high peaks and low peaks
		for(i = 0; i < pca_g->motion.peaks.num[0]-1; i++)
		{
			if(fabs(pca_g->motion.peaks.low_val[i]-pca_g->motion.peaks.low_val[i+1]) > diff_ml)
				diff_ml = fabs(pca_g->motion.peaks.low_val[i]-pca_g->motion.peaks.low_val[i+1]);
		}

		for(i = 0; i < pca_g->motion.peaks.num[1]-1; i++)
		{
			if(fabs(pca_g->motion.peaks.high_val[i]-pca_g->motion.peaks.high_val[i+1]) > diff_mh)
				diff_mh = fabs(pca_g->motion.peaks.high_val[i]-pca_g->motion.peaks.high_val[i+1]);
		}
		if(diff_ml < (pca->motion_parametrs[2]/4) && diff_mh < (pca->motion_parametrs[2]/4))
		{
#ifdef METHOD_RESULTS_MISALIGNMENT
			//My_Method_Worked[126] = 1;
			temp_res = general_phone_tablet_18_mis(pca_g);
			if(temp_res == 0)
			{
				//My_Method_Num = 126;
				return slow_undetect_tag;
			}
			else if(temp_res == 1)
			{
				//My_Method_Num = 126;
				return !(slow_undetect_tag);
			}
#else
			temp_res = general_phone_tablet_18_mis(pca_g);
			if(temp_res == 0)
				return slow_undetect_tag;
			else if(temp_res == 1)
				return !(slow_undetect_tag);
#endif
		}
		// End of Method 26
	}
	// End of Method 25 and 26

	// Method 27				general_belt_reverse_pocket()
	//special dealing Belt data when it has pocket patterm but reverse 2012.05.05.
	//Revised from (max_vh - min_vh) > 1.4 || MAX(pca_mis.motion.vec[6], pca_mis.motion.vec[5]) > 1.6
	//      	to (max_vh - min_vh) > 1.4 && MAX(pca_mis.motion.vec[6], pca_mis.motion.vec[5]) > 1.6 on May 31th 2012
	//    	reason: avoid ear data set in dragon go to this one, Revised condition and new part when undetection happen. July 3 2012
	if(pca_g->vertical.peaks.num[1] > 1 && (amp_peak_vh > 1.4 || MAX(pca_g->motion.motion_low_peaks_range, pca_g->motion.motion_high_peaks_range) > 1.6) && pca->motion_parametrs[2] > 4 && pca->motion_parametrs[1] > 1 && pca_g->motion.local_peaks.num[0] > 0 && pca_g->motion.local_peaks.num[1] > 0 &&	((pca->horizontal_vertical_flag == 0 && fabs(pca->roll_parametrs[0]*R2Df) > 60.0) || (pca->horizontal_vertical_flag != 0 && MAX(pca->roll_parametrs[2], pca->pitch_parametrs[2]) > 7.0)))
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[127] = 1;
		temp_res = general_phone_tablet_13_mis(pca_g, pca);
		if(temp_res == 0)
		{
			//My_Method_Num = 127;
			return slow_undetect_tag;
		}
		else if(temp_res == 1)
		{
			//My_Method_Num = 127;
			return !(slow_undetect_tag);
		}
#else
		temp_res = general_phone_tablet_13_mis(pca_g, pca);
		if(temp_res == 0)
			return slow_undetect_tag;
		else if(temp_res == 1)
			return !(slow_undetect_tag);
#endif
	}
	// End of Method 27

	// Method 28				general_purse()
	/***Xcorr_phase method***/
	if(pca->motion_parametrs[2] > 6 && pca->motion_parametrs[2] > pca->vertical_parametrs[2] && pca_g->xcorr.avg_cycle > 3.0)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[128] = 1;
		temp_res = general_phone_tablet_1_mis(pca_g, pca, 135.0, 45.0, -45.0, -135.0);
		if(temp_res == 0)
		{
			//My_Method_Num = 128;
			return slow_undetect_tag;
		}
		else if(temp_res == 1)
		{
			//My_Method_Num = 128;
			return !(slow_undetect_tag);
		}
#else
		temp_res = general_phone_tablet_1_mis(pca_g, pca, 135.0, 45.0, -45.0, -135.0);
		if(temp_res == 0)
			return slow_undetect_tag;
		else if(temp_res == 1)
			return !(slow_undetect_tag);
#endif
	}
	// End of Method 28

	// Method 29				 general_light_dangling_4()
	// Added August 2nd deal with the light dangling
	m_peaks_count = 0;
	for(i=0; i<pca_g->motion.peaks.num[1]; i++)
	{
		if(pca_g->motion.peaks.high_val[i] > pca_g->motion.thresholds[1])
			m_peaks_count++;
	}
	for(i=0; i<pca_g->motion.peaks.num[0]; i++)
	{
		if(pca_g->motion.peaks.low_val[i] < pca_g->motion.thresholds[0])
			m_peaks_count++;
	}

	if(fabs(pca->roll_parametrs[0]*R2Df) > 60.0 && fabs(pca->roll_parametrs[0]*R2Df) < 120.0 && m_peaks_count <= 5 && pca->vertical_parametrs[2] < 8.0 && fabs(pca->pitch_parametrs[0]*R2Df) < 25.0 && (fabs(pca->pitch_parametrs[0]*R2Df) > 10.0 || (pca->pitch_parametrs[2] > 6.2 && pca->roll_parametrs[2] > 4.0)))
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[129] = 1;
		temp_res = general_phone_tablet_19_mis(pca_g, pca);
		if(temp_res == 0)
		{
			//My_Method_Num = 129;
			return slow_undetect_tag;
		}
		else if(temp_res == 1)
		{
			//My_Method_Num = 129;
			return !(slow_undetect_tag);
		}
#else
		temp_res = general_phone_tablet_19_mis(pca_g, pca);
		if(temp_res == 0)
			return slow_undetect_tag;
		else if(temp_res == 1)
			return !(slow_undetect_tag);
#endif
	}
	// End of Method 29

	// Method 30 and 31               general_compass_simple_method_3_inverse_logic()
	// Add August 2013: deal with Jacket pockets, Simple method I with special requirement

	// Method 30
	if(pca->horizontal_vertical_flag == 0 && fabs(pca->roll_parametrs[0]*R2Df) > 60.0 && fabs(pca->roll_parametrs[0]*R2Df) < 120.0 && pca->motion_parametrs[1] > 2.0 && pca_m->lateral_range > 2.0 && pca_m->lateral_range < 4.5 && (pca->vertical_parametrs[2]/pca->motion_parametrs[2]) < 1.5 && MIN(pca->roll_parametrs[2], pca->pitch_parametrs[2]) > 5.0)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[130] = 1;
		temp_res = general_phone_tablet_20_mis(pca_g, pca, 1);
		if(temp_res == 0)
		{
			//My_Method_Num = 130;
			return slow_undetect_tag;
		}
		else if(temp_res == 1)
		{
			//My_Method_Num = 130;
			return !(slow_undetect_tag);
		}
#else
		temp_res = general_phone_tablet_20_mis(pca_g, pca, 1);
		if(temp_res == 0)
			return slow_undetect_tag;
		else if(temp_res == 1)
			return !(slow_undetect_tag);
#endif
	}
	// End of Method 30

	// Method 31
	else if(pca->horizontal_vertical_flag != 0 && pca->motion_parametrs[1] > 1.2 && pca_m->horizontal_acceleration_range > 2.0 && pca_m->lateral_range > 3.0 && (pca->vertical_parametrs[2]/pca->motion_parametrs[2]) > 0.8 && (pca->vertical_parametrs[2]/pca->motion_parametrs[2]) < 1.3)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[131] = 1;
		temp_res = general_phone_tablet_20_mis(pca_g, pca, 2);
		if(temp_res == 0)
		{
			//My_Method_Num = 131;
			return slow_undetect_tag;
		}
		else if(temp_res == 1)
		{
			//My_Method_Num = 131;
			return !(slow_undetect_tag);
		}
#else
		temp_res = general_phone_tablet_20_mis(pca_g, pca, 2);
		if(temp_res == 0)
			return slow_undetect_tag;
		else if(temp_res == 1)
			return !(slow_undetect_tag);
#endif
	}
	// End of Method 31
	// End of Method 30 and 31

	// Method 32       General_Unstable_Handheld_Method_3()
	// Add December 2013: deal with unstable handheld
	if(MAX(pca->roll_parametrs[2], pca->pitch_parametrs[2]) < 7.0 && MAX(fabs(pca->roll_parametrs[0]*R2Df), fabs(pca->pitch_parametrs[0]*R2Df)) < 70.0 && pca_m->lateral_range > 1.5 && pca_m->device_flag == 0)
	{
		//temp_res = General_Unstable_Handheld_Method(pca_g, pca, pca_m);
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[132] = 1;
		temp_res = general_phone_tablet_20_mis(pca_g, pca, 3);
		if(temp_res == 0)
		{
			//My_Method_Num = 132;
			return slow_undetect_tag;
		}
		else if(temp_res == 1)
		{
			//My_Method_Num = 132;
			return !(slow_undetect_tag);
		}
#else
		temp_res = general_phone_tablet_20_mis(pca_g, pca, 3);
		if(temp_res == 0)
			return slow_undetect_tag;
		else if(temp_res == 1)
			return !(slow_undetect_tag);
#endif
	}
	// End of Method 32

	// Method 33
	// Add August 2013: deal with Jacket pockets Simple method I with special requirement
#ifdef METHOD_RESULTS_MISALIGNMENT
	//My_Method_Worked[133] = 1;
	temp_res = general_phone_tablet_21_mis(pca_g, pca);
	if(temp_res == 0)
	{
		//My_Method_Num = 133;
		return slow_undetect_tag;
	}
	else if(temp_res == 1)
	{
		//My_Method_Num = 133;
		return !(slow_undetect_tag);
	}
#else
	temp_res = general_phone_tablet_21_mis(pca_g, pca);
	if(temp_res == 0)
		return slow_undetect_tag;
	else if(temp_res == 1)
		return !(slow_undetect_tag);
#endif
	// End of Method 33

	// Method 34				General_Compass_Motion_Signal_1()
	// Add August 2013: deal with Jacket pockets Simple enhanced version II: using peaks pair that do not have mean value restriction
	ml_close_counter = 0;
	mh_close_counter = 0;
	for(i = 1; i < pca_g->motion.peaks.num[1]; i++)
	{
		if(abs(pca_g->motion.peaks.high_ind[i] - pca_g->motion.peaks.high_ind[i-1]) <= 6)
			mh_close_counter++;
	}
	for(i = 1; i < pca_g->motion.peaks.num[0]; i++)
	{
		if(abs(pca_g->motion.peaks.low_ind[i] - pca_g->motion.peaks.low_ind[i-1]) <= 6)
			ml_close_counter++;
	}
	if(pca->vertical_parametrs[2]/pca->motion_parametrs[2] > 1.8 && MAX(pca->roll_parametrs[2], pca->pitch_parametrs[2]) < 3.0 && pca_m->lateral_range > 2.5 && pca_m->horizontal_acceleration_range > 3.0 && pca->vertical_parametrs[2] > 9 && (ml_close_counter > 0 || mh_close_counter > 0))
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[134] = 1;
		temp_res = general_phone_tablet_20_mis(pca_g, pca, 4);
		if(temp_res == 0)
		{
			//My_Method_Num = 134;
			return slow_undetect_tag;
		}
		else if(temp_res == 1)
		{
			//My_Method_Num = 134;
			return !(slow_undetect_tag);
		}
#else
		temp_res = general_phone_tablet_20_mis(pca_g, pca, 4);
		if(temp_res == 0)
			return slow_undetect_tag;
		else if(temp_res == 1)
			return !(slow_undetect_tag);
#endif
	}
	// End of Method 34

	// Method 35			general_compass_motion_signal_2()
	// Add August 2013: deal with Jacket pockets, deals with the Jacket pockets when have Compoass reverse logic
#ifdef METHOD_RESULTS_MISALIGNMENT
	//My_Method_Worked[135] = 1;
	temp_res = general_phone_tablet_22_mis(pca_g, pca, pca_m);
	if(temp_res == 0)
	{
		//My_Method_Num = 135;
		return slow_undetect_tag;
	}
	else if(temp_res == 1)
	{
		//My_Method_Num = 135;
		return !(slow_undetect_tag);
	}
#else
	temp_res = general_phone_tablet_22_mis(pca_g, pca, pca_m);
	if(temp_res == 0)
		return slow_undetect_tag;
	else if(temp_res == 1)
		return !(slow_undetect_tag);
#endif
	// End of Method 35

	// Method 36
	//Simple enhanced version II: using peaks pair that do not have mean value restriction
	if(pca_m->lateral_range < 4.0 && pca_m->horizontal_acceleration_range < 3.0)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[136] = 1;
		temp_res = general_phone_tablet_23_mis(pca_g);
		if(temp_res == 0)
		{
			//My_Method_Num = 136;
			return slow_undetect_tag;
		}
		else if(temp_res == 1)
		{
			//My_Method_Num = 136;
			return !(slow_undetect_tag);
		}
#else
		temp_res = general_phone_tablet_23_mis(pca_g);
		if(temp_res == 0)
			return slow_undetect_tag;
		else if(temp_res == 1)
			return !(slow_undetect_tag);
#endif
	}
	// End of Method 36

	// Method 37                  general_purse()
	if(pca_g->xcorr.avg_cycle > 0)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[137] = 1;
		temp_res = general_phone_tablet_1_mis(pca_g, pca, 100.0, 10.0, -10.0, -100.0);
		if(temp_res == 0)
		{
			//My_Method_Num = 137;
			return slow_undetect_tag;
		}
		else if(temp_res == 1)
		{
			//My_Method_Num = 137;
			return !(slow_undetect_tag);
		}
#else
		temp_res = general_phone_tablet_1_mis(pca_g, pca, 100.0, 10.0, -10.0, -100.0);
		if(temp_res == 0)
			return slow_undetect_tag;
		else if(temp_res == 1)
			return !(slow_undetect_tag);
#endif
	}
	// End of Method 37
#ifdef METHOD_RESULTS_MISALIGNMENT
	//My_Method_Worked[100] = 1;
	//My_Method_Num = 100;
#endif
	return 2;
}
/**
* @brief	This is phone and tablet general_phone_tablet_1_mis function to solve the 180 amiguity problem in
* 			case of phone or tablet general data.
*
* @details	This function takes 2 pointers of structures of type PCA_G and PCA and 4 values.
* 			It uses the auto and cross correlation between the different data to to provide a
* 			decision for Forward/Backward.
*
* @param[in]	pca_g	pointer to a structure of type \c PCA_G.
* @param[in]	pca		pointer to a structure of type \c PCA.
* @param[in]	v1   	variable for the first value in float.
* @param[in]	v2   	variable for the second value in float.
* @param[in]	v3   	variable for the third value in float.
* @param[in]	v4   	variable for the fourth value in float.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*
* @see calculate_correlation_mis_float_float_mis() and xcorr_c_float_float_mis().
*/
INT8 general_phone_tablet_1_mis(PCA_G_Ptr pca_g, PCA_Ptr pca, FLOAT32 v1, FLOAT32 v2, FLOAT32 v3, FLOAT32 v4)
{
	INT8 i, j, m;
	INT8 count1 = 0, count2 = 0, count3 = 0, count4 = 0;
	INT8 n_cycle = 0;
	INT8 step_f = 0, step_b = 0;
	INT16 phase_lag = 0;
	INT16 lags[2*DATA_SIZE_MIS-1] = {0};

	FLOAT32 phase_shift = 0.0, forward_score = 0.0, temp = 0.0;
	FLOAT32 zero_cro_fm[PEAKS_VECTOR_SIZE_MIS] = {0.0}, zero_cro_rm[PEAKS_VECTOR_SIZE_MIS] = {0.0};
	FLOAT32 zero_cro_fv[PEAKS_VECTOR_SIZE_MIS] = {0.0}, zero_cro_rv[PEAKS_VECTOR_SIZE_MIS] = {0.0};
	FLOAT32 x_drift_1[DATA_SIZE_MIS] = {0.0}, z_drift_1[DATA_SIZE_MIS] = {0.0};
	FLOAT32 r[2*DATA_SIZE_MIS-1]     = {0.0}, d_phase[DATA_SIZE_MIS]   = {0.0};

	//Method 1
	phase_lag = (INT16)(ceil(pca_g->xcorr.avg_cycle/4)-1);

	if(phase_lag == 0)
		phase_lag = (INT16)ceil(pca_g->xcorr.avg_cycle/4);  // Comp_mc August 2013 Modified: Adding 'phase_lag =' at the beginning

	for(i = 0; i < pca_g->xcorr.stride_size-phase_lag; i++)
	{
		x_drift_1[i] = pca->data_m[i+phase_lag];
		z_drift_1[i] = pca->data_v[i];
	}
	forward_score = calculate_correlation_mis_float_float_mis(x_drift_1, z_drift_1, pca_g->xcorr.stride_size-phase_lag);

	if(fabs(forward_score) > 0.6)
	{
		if(forward_score > 0.0)
			phase_shift = -90.0;
		else
			phase_shift =  90.0;
	}
	else
	{
		phase_lag = xcorr_c_float_float_mis(pca_g->xcorr.z_drift, pca_g->xcorr.x_drift, pca_g->xcorr.stride_size, r, lags, 0);

		if(abs(phase_lag) > pca_g->xcorr.avg_cycle)
		{
			n_cycle   = (INT8)(floor(abs(phase_lag)/pca_g->xcorr.avg_cycle));
			phase_lag = (INT16)((abs(phase_lag)-n_cycle*pca_g->xcorr.avg_cycle)*(phase_lag > 0 ? 1:-1));
		}

		phase_shift = phase_lag/pca_g->xcorr.avg_cycle*360.0F;
		phase_shift = phase_shift-(phase_shift >  180.0F ?  360.0F:0.0F);
		phase_shift = phase_shift-(phase_shift < -180.0F ? -360.0F:0.0F);

	}////End of Xcorr_phase method 1

	if(phase_shift <= -45.0 &&  phase_shift >= -135.0)
		step_f++;
	else
	{
		if(phase_shift <= 135.0 &&  phase_shift >= 45.0)
			step_b++;
		else
		{
			//Start method 3
			count1 = 0; count2 = 0; count3 = 0; count4 = 0;
			for(i = 0; i < pca_g->xcorr.stride_size-1; i++)
			{
				if(pca_g->xcorr.x_drift[i] > 0.0 && pca_g->xcorr.x_drift[i+1] < 0.0) //falling zero-crossing of motion signal
				{
					zero_cro_fm[count1] = (fabs(pca_g->xcorr.x_drift[i])*(i+2)+fabs(pca_g->xcorr.x_drift[i+1])*(i+1))/(fabs(pca_g->xcorr.x_drift[i])+fabs(pca_g->xcorr.x_drift[i+1]));
					count1++;
				}
				if(pca_g->xcorr.x_drift[i] < 0.0 && pca_g->xcorr.x_drift[i+1] > 0.0) //rsing zero-crossing of motion signal
				{
					zero_cro_rm[count2] = (fabs(pca_g->xcorr.x_drift[i])*(i+2)+fabs(pca_g->xcorr.x_drift[i+1])*(i+1))/(fabs(pca_g->xcorr.x_drift[i])+fabs(pca_g->xcorr.x_drift[i+1]));
					count2++;
				}
			}

			for(i = 0; i < pca_g->xcorr.stride_size-1; i++)
			{
				if(pca_g->xcorr.z_drift[i] > 0.0 && pca_g->xcorr.z_drift[i+1] < 0.0) //falling zero-crossing of vertical signal
				{
					zero_cro_fv[count3] = (fabs(pca_g->xcorr.z_drift[i])*(i+2)+fabs(pca_g->xcorr.z_drift[i+1])*(i+1))/(fabs(pca_g->xcorr.z_drift[i])+fabs(pca_g->xcorr.z_drift[i+1]));
					count3++;
				}

				if(pca_g->xcorr.z_drift[i] < 0.0 && pca_g->xcorr.z_drift[i+1] > 0.0) //rsing zero-crossing of vertical signal
				{
					zero_cro_rv[count4] = (fabs(pca_g->xcorr.z_drift[i])*(i+2)+fabs(pca_g->xcorr.z_drift[i+1])*(i+1))/(fabs(pca_g->xcorr.z_drift[i])+fabs(pca_g->xcorr.z_drift[i+1]));
					count4++;
				}
			}
			m = 0;
			for(i = 0; i < count1; i++)
			{
				temp = 999.0;
				for(j = 0; j < count3; j++)
				{
					if(fabs(zero_cro_fv[j]-zero_cro_fm[i]) < temp)
					{
						temp   = fabs(zero_cro_fv[j]-zero_cro_fm[i]);
						d_phase[m] = (zero_cro_fv[j]-zero_cro_fm[i])/pca_g->xcorr.avg_cycle*360.0F;
					}
				}
				m++;
			}

			for(i = 0; i < count2; i++)
			{
				temp = 999.0;
				for(j = 0; j < count4; j++)
				{
					if(fabs(zero_cro_rv[j]-zero_cro_rm[i]) < temp)
					{
						temp = fabs(zero_cro_rv[j]-zero_cro_rm[i]);
						d_phase[m] = (zero_cro_rv[j]-zero_cro_rm[i])/pca_g->xcorr.avg_cycle*360;
					}
				}
				m++;
			}
			temp = 0.0;
			for(i = 0; i < m; i++)
			{
				d_phase[i] -= d_phase[i] >  180.0F ?  360.0F:0.0F;
				d_phase[i] -= d_phase[i] < -180.0F ? -360.0F:0.0F;
				temp += d_phase[i];
			}

			if(m == 0)
				phase_shift = temp;
			else
				phase_shift = temp/m;

			if(phase_shift <= v1 &&  phase_shift >= v2)
				step_b++;
			if(phase_shift <= v3 &&  phase_shift >= v4)
				step_f++;
		}
	}// END of Xcorr_phase method 1
	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is phone and tablet general_phone_tablet_2_mis function to solve the 180
* 			amiguity problem in case of phone or tablet general data.
*
* @details	This function takes 2 pointers of structures of type PCA_G and PCA. It deals with the
* 			data of light dangling cases that misclassified as general. It uses the realtion
* 			between the vertical and motion slow peaks and the motion data slope to provide a
* 			decision for Forward/Backward.
*
* @param[in]	pca_g	pointer to a structure of type \c PCA_G.
* @param[in]	pca		pointer to a structure of type \c PCA.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 general_phone_tablet_2_mis(PCA_G_Ptr pca_g, PCA_Ptr pca)
{
	INT8 i, j;
	INT8 min_vl_ind = -1, m_high_ind = -1, m_low_ind = -1;
	INT8 st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;
	FLOAT32 vl_peaks_range = 0;
	FLOAT32 max_vl_peaks = 0, min_vl_peaks = 0;
	FLOAT32 slope_m = 0.0;
	UINT8 ff;

	for(i=0; i<pca_g->motion.peaks.num[0]; i++)
	{
		ff = 0;
		for(j=0; j<pca_g->motion.slow_peaks.num[0]; j++)
		{
			if(pca_g->motion.slow_peaks.low_ind[j] == pca_g->motion.peaks.low_ind[i])
			{ff = 1;	break;	}
		}
		if(ff)
			continue;

		if(pca_g->motion.peaks.low_val[i] > pca_g->motion.thresholds[0])
			m_low_ind = pca_g->motion.peaks.low_ind[i];
	}

	for(i=0; i<pca_g->motion.peaks.num[1]; i++)
	{
		ff = 0;
		for(j=0; j<pca_g->motion.slow_peaks.num[1]; j++)
		{
			if(pca_g->motion.slow_peaks.high_ind[j] ==  pca_g->motion.peaks.high_ind[i])
			{ff = 1;   break;  }
		}
		if(ff)
			continue;

		if(pca_g->motion.peaks.high_val[i] < pca_g->motion.thresholds[1])
			m_high_ind = pca_g->motion.peaks.high_ind[i];
	}
	if(m_high_ind > -1 && m_low_ind > -1)
	{
		if(m_high_ind < m_low_ind)
			step_f++;
		else
			step_b++;
	}

	max_vl_peaks = -99.0;
	min_vl_peaks =  99.0;
	min_vl_ind = -1;

	for(i=0; i<pca_g->vertical.peaks.num[0]; i++)
	{
		if(pca_g->vertical.peaks.low_val[i] > max_vl_peaks)
			max_vl_peaks = pca_g->vertical.peaks.low_val[i];
		if(pca_g->vertical.peaks.low_val[i] < min_vl_peaks)
		{
			min_vl_peaks = pca_g->vertical.peaks.low_val[i];
			min_vl_ind = i;
		}
	}

	vl_peaks_range = fabs(max_vl_peaks - min_vl_peaks);

	if(vl_peaks_range > 1)
	{
		st_ind = MAX(pca_g->vertical.peaks.low_ind[min_vl_ind]-2, 0);
		en_ind = MIN(pca_g->vertical.peaks.low_ind[min_vl_ind]+2, DATA_SIZE_MIS-1);

		for(j=0; j<pca_g->motion.peaks.num[1]; j++)
		{
			if(pca_g->motion.peaks.high_ind[j] >= st_ind && pca_g->motion.peaks.high_ind[j] <= en_ind)
				step_f++;
		}
		for(j=0; j<pca_g->motion.peaks.num[0]; j++)
		{
			if(pca_g->motion.peaks.low_ind[j] >= st_ind && pca_g->motion.peaks.low_ind[j] <= en_ind)
				step_b++;
		}
	}

	if(step_f == step_b)
	{
		for(i=0; i<pca_g->vertical.peaks.num[1]; i++)
		{
			for(j=0; j<pca_g->vertical.peaks.num[0]; j++)
			{
				if(abs(pca_g->vertical.peaks.high_ind[i] - pca_g->vertical.peaks.low_ind[j]) <= 4 && pca_g->vertical.peaks.low_ind[j] < pca_g->vertical.peaks.high_ind[i])
				{
					slope_m = pca->data_m[pca_g->vertical.peaks.high_ind[i]-1] - pca->data_m[pca_g->vertical.peaks.low_ind[j]-1];
					if(slope_m >= 0)
						step_f++;
					else
						step_b++;
				}
			}
		}
	}

	if(step_f == step_b)
	{
		for(i=0; i<pca_g->vertical.peaks.num[1]; i++)
		{
			st_ind = MAX(pca_g->vertical.peaks.high_ind[i]-2, 0);
			en_ind = MIN(pca_g->vertical.peaks.high_ind[i]+2, DATA_SIZE_MIS-1);

			for(j=0; j<pca_g->motion.slow_peaks.num[1]; j++)
			{
				if(pca_g->motion.slow_peaks.high_ind[j] >= st_ind && pca_g->motion.slow_peaks.high_ind[j] <= en_ind)
					step_b++;
			}
			for(j=0; j<pca_g->motion.slow_peaks.num[0]; j++)
			{
				if(pca_g->motion.slow_peaks.low_ind[j] >= st_ind && pca_g->motion.slow_peaks.low_ind[j] <= en_ind)
					step_f++;
			}
		}
	}

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is phone and tablet general_phone_tablet_3_mis function to solve the 180
* 			amiguity problem in case of phone or tablet general data.
*
* @details	This function takes 2 pointers of structures of type PCA_G and PCA. It deals with the
* 			data of slow motion. It uses the realtion between the vertical and motion slow peaks
* 			to provide a decision for Forward/Backward.
*
* @param[in]	pca_g	pointer to a structure of type \c PCA_G.
* @param[in]	pca		pointer to a structure of type \c PCA.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*
* @see slope_segment_mis().
*/
INT8 general_phone_tablet_3_mis(PCA_G_Ptr pca_g, PCA_Ptr pca)
{
	INT8 step[2] = {0};

	if((pca_g->vertical.slow_peaks.num[0] + pca_g->vertical.slow_peaks.num[1]) < 6)
		slope_segment_mis(pca_g->vertical.slow_peaks.num[0], pca_g->vertical.slow_peaks.low_ind, pca_g->vertical.slow_peaks.low_val, pca->data_m, 2, 0,1, pca_g->vertical.thresholds[0], step, 0);

	if(step[0] > step[1])
		return 0;
	else if(step[1] > step[0])
		return 1;
	else
		return 2;
}
/**
* @brief	This is phone and tablet general_phone_tablet_4_mis function to solve
* 			the 180 amiguity problem in	case of phone or tablet general data.
*
* @details	This function takes 2 pointers of structures of type PCA_G and PCA. It deals with compass
* 			data that has big motion amplitude signal. It uses the realtion between the vertical and
* 			 motion peaks to provide a decision for Forward/Backward.
*
* @param[in]	pca_g	pointer to a structure of type \c PCA_G.
* @param[in]	pca		pointer to a structure of type \c PCA.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 general_phone_tablet_4_mis(PCA_G_Ptr pca_g, PCA_Ptr pca)
{
	INT8 i, j;
	INT8 k1 = 0, b = -1;
	INT8 st_ind = -1, en_ind = -1;
	INT8 id_h = -1, id_l = -1;
	INT8 min_vh_mh_dis, min_vh_ml_dis, min_vl_mh_dis, min_vl_ml_dis = 99;
	INT8 min_vl_vh = 0, min_vh_vl = 0;
	INT8 step_f = 0, step_b = 0, step[2] = {0};
	UINT8 min_peaks_dis[4] = {0,0,0,0};
	FLOAT32 a = 0.0;

	for(i = 0; i < pca_g->vertical.peaks.num[1]; i++)
	{
		k1 = -1;
		for(j = 0; j < pca_g->vertical.peaks.num[0]; j++)
		{
			if(pca_g->vertical.peaks.low_ind[j] > pca_g->vertical.peaks.high_ind[i])
			{
				k1 = j;
				break;
			}
		}
		if(k1 != -1 && abs(pca_g->vertical.peaks.low_ind[k1]-pca_g->vertical.peaks.high_ind[i]) > min_vh_vl)
			min_vh_vl = pca_g->vertical.peaks.low_ind[k1]-pca_g->vertical.peaks.high_ind[i];
	}

	for(i = 0; i < pca_g->vertical.peaks.num[0]; i++)
	{
		k1 = -1;
		for(j = 0; j < pca_g->vertical.peaks.num[1]; j++)
		{
			if(pca_g->vertical.peaks.high_ind[j] > pca_g->vertical.peaks.low_ind[i])
			{
				k1 = j;
				break;
			}
		}
		if(k1 != -1 && abs(pca_g->vertical.peaks.high_ind[k1] - pca_g->vertical.peaks.low_ind[i]) > min_vl_vh)
			min_vl_vh = pca_g->vertical.peaks.high_ind[k1] - pca_g->vertical.peaks.low_ind[i];
	}

	if(pca->vertical_parametrs[2] <= 3.5)
	{
		// check for the vertical high peaks
		for(i=0; i<pca_g->vertical.peaks.num[1]; i++)
		{
			if(pca_g->vertical.peaks.high_val[i] > pca_g->vertical.thresholds[1])
			{
				st_ind = MAX(pca_g->vertical.peaks.high_ind[i]-3, 0);
				en_ind = MIN(pca_g->vertical.peaks.high_ind[i]+2, DATA_SIZE_MIS-1);
				for(j=0; j<pca_g->motion.peaks.num[1]; j++)
				{
					if(pca_g->motion.peaks.high_ind[j] >= st_ind && pca_g->motion.peaks.high_ind[j] <= en_ind && pca_g->motion.peaks.high_val[j] > pca_g->motion.thresholds[1])
						step_b++;
				}
				for (j=0; j<pca_g->motion.peaks.num[0]; j++)
				{
					if(pca_g->motion.peaks.low_ind[j] >= st_ind && pca_g->motion.peaks.low_ind[j] <= en_ind && pca_g->motion.peaks.low_val[j] < pca_g->motion.thresholds[0])
						step_f++;
				}
			}
		}
		// check  for  the vertical low peaks
		for(i=0; i<pca_g->vertical.peaks.num[0]; i++)
		{
			if(pca_g->vertical.peaks.low_val[i] < pca_g->vertical.thresholds[0])
			{
				st_ind = MAX(pca_g->vertical.peaks.low_ind[i]-3, 0);
				en_ind = MIN(pca_g->vertical.peaks.low_ind[i]+2, DATA_SIZE_MIS-1);
				for(j=0; j<pca_g->motion.peaks.num[1]; j++)
				{
					if(pca_g->motion.peaks.high_ind[j] >= st_ind && pca_g->motion.peaks.high_ind[j] <= en_ind && pca_g->motion.peaks.high_val[j] > pca_g->motion.thresholds[1])
						step_f++;
				}
				for(j=0; j<pca_g->motion.peaks.num[0]; j++)
				{
					if(pca_g->motion.peaks.low_ind[j] >= st_ind && pca_g->motion.peaks.low_ind[j] <= en_ind && pca_g->motion.peaks.low_val[j] < pca_g->motion.thresholds[0])
						step_b++;
				}
			}
		}
	}
	else if(pca->vertical_parametrs[2] > 3.5  && (pca_g->vertical.peaks.num[1]+pca_g->vertical.peaks.num[0]) < 6 && pca->vertical_parametrs[2] < 6.0 && abs(min_vh_vl-min_vl_vh) > 3)
	{
		i = peaks_min_distance_mis(pca_g, min_peaks_dis, 3, 0, step);
		step_f += step[0];
		step_b += step[1];
	} // if((max_v-min_v) <= 3.5)

	//added on July 24. Especailly for solving problem between Husain dataset and girl dataset in compass.
	if(step_f == step_b && (!(pca->vertical_parametrs[2] > 5.5 && pca->motion_parametrs[2] < 2.5)) && pca->vertical_parametrs[2] < 8 &&
		pca->motion_parametrs[2] > 2.9 && (pca->vertical_parametrs[2]/pca->motion_parametrs[2]) < 1.7)
	{
		a = 99.0;
		if(fabs(pca_g->misa)*R2Df < 90.0)
		{
			for(i = 0; i < pca_g->motion.peaks.num[0]; i++)
			{
				if(pca_g->motion.peaks.low_val[i] < a)
				{
					a = pca_g->motion.peaks.low_val[i];
					b = i;
				}
			}

			min_vh_ml_dis = 99;
			min_vl_ml_dis = 99;
			for(i = 0; i < pca_g->vertical.peaks.num[0]; i++)
			{
				if(abs(pca_g->vertical.peaks.low_ind[i] - pca_g->motion.peaks.low_ind[b]) < min_vl_ml_dis)
				{
					min_vl_ml_dis = (INT8)(abs(pca_g->vertical.peaks.low_ind[i] - pca_g->motion.peaks.low_ind[b]));
					id_l = pca_g->vertical.peaks.low_ind[i];
				}
			}

			for(i = 0; i < pca_g->vertical.peaks.num[1]; i++)
			{
				if(abs(pca_g->vertical.peaks.high_ind[i] - pca_g->motion.peaks.low_ind[b]) < min_vh_ml_dis)
				{
					min_vh_ml_dis = (INT8)(abs(pca_g->vertical.peaks.high_ind[i] - pca_g->motion.peaks.low_ind[b]));
					id_h = pca_g->vertical.peaks.high_ind[i];
				}
			}

			if(id_h >= pca_g->motion.peaks.low_ind[b] && id_l <= pca_g->motion.peaks.low_ind[b]) // locate on positive slope
				step_f++;
			else
			{
				if(id_h <= pca_g->motion.peaks.low_ind[b] && id_l >= pca_g->motion.peaks.low_ind[b]) // locate on negative slope
					step_b++;
				else
				{
					if((id_h > id_l && id_h < pca_g->motion.peaks.low_ind[b]) || (id_l < id_h && id_l > pca_g->motion.peaks.low_ind[b]))
						step_b++;
					if((id_l > id_h && id_l < pca_g->motion.peaks.low_ind[b]) || (id_h < id_l && id_h > pca_g->motion.peaks.low_ind[b]))
						step_f++;
				}
			}//if(id_h >= pca_g->motion.ml_peaks[b][0] && id_l <= pca_g->motion.ml_peaks[b][0])
		}
		else
		{
			a = -99.0;
			for(i = 0; i < pca_g->motion.peaks.num[1]; i++)
			{
				if(pca_g->motion.peaks.high_val[i] > a)
				{
					a = pca_g->motion.peaks.high_val[i];
					b = i;
				}
			}

			min_vh_mh_dis = 99;
			min_vl_mh_dis = 99;
			for(i = 0; i < pca_g->vertical.peaks.num[0]; i++)
			{
				if(abs(pca_g->vertical.peaks.low_ind[i] - pca_g->motion.peaks.high_ind[b]) < min_vl_mh_dis)
				{
					min_vl_mh_dis = (INT8)(abs(pca_g->vertical.peaks.low_ind[i] - pca_g->motion.peaks.high_ind[b]));
					id_l = pca_g->vertical.peaks.low_ind[i];
				}
			}

			for(i = 0; i < pca_g->vertical.peaks.num[1]; i++)
			{
				if(abs(pca_g->vertical.peaks.high_ind[i] - pca_g->motion.peaks.high_ind[b]) < min_vh_mh_dis)
				{
					min_vh_mh_dis = (INT8)(abs(pca_g->vertical.peaks.high_ind[i] - pca_g->motion.peaks.high_ind[b]));
					id_h = pca_g->vertical.peaks.high_ind[i];
				}
			}

			if(id_h >= pca_g->motion.peaks.high_ind[b] && id_l <= pca_g->motion.peaks.high_ind[b]) // locate on positive slope
				step_b++;
			else
			{
				if(id_h <= pca_g->motion.peaks.high_ind[b] && id_l >= pca_g->motion.peaks.high_ind[b]) // locate on negative slope
					step_f++;
				else
				{
					if((id_h > id_l && id_h < pca_g->motion.peaks.high_ind[b]) || (id_l < id_h && id_l > pca_g->motion.peaks.high_ind[b]))
						step_f++;
					if((id_l > id_h && id_l < pca_g->motion.peaks.high_ind[b]) || (id_h < id_l && id_h > pca_g->motion.peaks.high_ind[b]))
						step_b++;
				}
			}//if(id_h >= pca_g->motion.peaks.low_ind[b] && id_l <= pca_g->motion.peaks.low_ind[b])
		}
	}//if(step_f == step_b && (!(range_v > 5.5 && pca->motion_parametrs[2] < 2.5)))

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is phone and tablet general_phone_tablet_5_mis function to solve the 180
* 			amiguity problem in case of phone or tablet general data.
*
* @details	This function takes 2 pointers of structures of type PCA_G and PCA. It deals with the
* 			data of light dangling cases that misclassified as general. It uses the realtion
* 			between the vertical and motion slow peaks and the motion data slope to provide a
* 			decision for Forward/Backward. It only uses the motion low and high peaks that
* 			are less or greater than certain thresholds.
*
* @param[in]	pca_g	pointer to a structure of type \c PCA_G.
* @param[in]	pca		pointer to a structure of type \c PCA.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 general_phone_tablet_5_mis(PCA_G_Ptr pca_g, PCA_Ptr pca)
{
	INT8 i;
	INT8 st_ind = -1, en_ind = -1;
	INT8 m_peaks_count = 0;
	INT8 max_vl_peaks_ind = -1;
	INT8 step_f = 0, step_b = 0;

	FLOAT32 max_vl_peaks = -99.0;
	FLOAT32 slope_m = 0.0;

	for(i=0; i<pca_g->motion.peaks.num[1]; i++)
	{
		if(pca_g->motion.peaks.high_val[i] > pca_g->motion.thresholds[1])
			m_peaks_count++;
	}
	for(i=0; i<pca_g->motion.peaks.num[0]; i++)
	{
		if(pca_g->motion.peaks.low_val[i] < pca_g->motion.thresholds[0])
			m_peaks_count++;
	}
	if(m_peaks_count <= 5)
	{
		// to get the index of the maximum low peak
		for(i=0; i<pca_g->vertical.peaks.num[0]; i++)
		{
			if(pca_g->vertical.peaks.low_val[i] > max_vl_peaks)
			{
				max_vl_peaks = pca_g->vertical.peaks.low_val[i];
				max_vl_peaks_ind = i;
			}
		}
		st_ind = MAX(pca_g->vertical.peaks.low_ind[max_vl_peaks_ind], 0);
		en_ind = MIN(pca_g->vertical.peaks.low_ind[max_vl_peaks_ind]+2, DATA_SIZE_MIS-1);
		slope_m = pca->data_m[en_ind] - pca->data_m[st_ind];
		if(slope_m >= 0)
			step_f++;
		else
			step_b++;
	}
	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is phone and tablet general_phone_tablet_6_mis function to solve the 180
* 			amiguity problem in case of phone or tablet general data.
*
* @details	This function takes 2 pointers of structures of type PCA_G and PCA. It deals with the
* 			belt data that has slow motion. It uses the realtion between the vertical and motion peaks
* 			to provide a decision for Forward/Backward.
*
* @param[in]	pca_g	pointer to a structure of type \c PCA_G.
* @param[in]	pca		pointer to a structure of type \c PCA.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 general_phone_tablet_6_mis(PCA_G_Ptr pca_g, PCA_Ptr pca)
{
	INT8 i, j;
	INT8 light_tag1 = 0, light_tag2 = 0;
	INT8 count1 = 0;
	INT8 step_f = 0, step_b = 0;
	INT8 min_h_ind = -99, min_l_ind = 99;
	INT8 index_ver_l[PEAKS_VECTOR_SIZE_MIS] = {0};
	INT8 dis_vh[PEAKS_VECTOR_SIZE_MIS] = {0};
	INT8  min_dis_vl_slow = 99;        // Comp_mc August 2013 Modified from a = -99.0; to a = 99.0 to catch the min value
	FLOAT32 max_l_val = -99.0, min_l_val = 99.0;

	for(i = 0; i < pca_g->vertical.slow_peaks.num[0]; i++)
	{
		if(i < (pca_g->vertical.slow_peaks.num[0]-1))
		{
			dis_vh[i] = pca_g->vertical.slow_peaks.low_ind[i+1]- pca_g->vertical.slow_peaks.low_ind[i];
			if(dis_vh[i] < min_dis_vl_slow)
				min_dis_vl_slow = dis_vh[i];
		}
		if(pca_g->vertical.slow_peaks.low_val[i] < min_l_val)
			min_l_val = pca_g->vertical.slow_peaks.low_val[i];  // min(vl_peaks_slow)
		if(pca_g->vertical.slow_peaks.low_val[i] > max_l_val)
			max_l_val = pca_g->vertical.slow_peaks.low_val[i];  // max(vl_peaks_slow)
	}

	if(pca_g->vertical.slow_peaks.num[0] > 1)
	{
		if(min_dis_vl_slow < 6 && pca_g->vertical.slow_peaks.num[0] > 2)
		{
			count1 = 0;
			for(i = 0; i < pca_g->vertical.slow_peaks.num[0]; i++)
			{
				if(pca_g->vertical.slow_peaks.low_val[i] < (min_l_val + 0.9))
					index_ver_l[count1++] = i;
			}
			if(count1 < 2) // will delete too many peaks, then we only delete the biggest one
			{
				count1 = 0;
				for(i = 0; i < pca_g->vertical.slow_peaks.num[0]; i++)
				{
					if(pca_g->vertical.slow_peaks.low_val[i] < max_l_val)
						index_ver_l[count1++] = i;
				}
			}

			max_l_val = -99.0;
			min_l_val =  99.0;
			for(i = 0; i < count1; i++)
			{
				pca_g->vertical.slow_peaks.low_ind[i] = pca_g->vertical.slow_peaks.low_ind[index_ver_l[i]];
				pca_g->vertical.slow_peaks.low_val[i] = pca_g->vertical.slow_peaks.low_val[index_ver_l[i]];
				if(pca_g->vertical.peaks.low_val[i] < min_l_val)
					min_l_val = pca_g->vertical.peaks.low_val[i];
				if(pca_g->vertical.peaks.low_val[i] > max_l_val)
					max_l_val = pca_g->vertical.peaks.low_val[i];
			}
			pca_g->vertical.slow_peaks.num[0] = count1;
		}			//if(a < 6 && pca_g->vertical.slow_peaks.num[0] > 2)

		if((max_l_val-min_l_val) > 1.5 && pca_g->vertical.slow_peaks.num[0] > 2)
		{
			count1 = 0;
			for(i = 0; i < pca_g->vertical.slow_peaks.num[0]; i++)
			{
				if(pca_g->vertical.slow_peaks.low_val[i] < max_l_val)
					index_ver_l[count1++] = i;
			}
			for(i = 0; i < count1; i++)
			{
				pca_g->vertical.slow_peaks.low_ind[i] = pca_g->vertical.slow_peaks.low_ind[index_ver_l[i]];
				pca_g->vertical.slow_peaks.low_val[i] = pca_g->vertical.slow_peaks.low_val[index_ver_l[i]];
			}
			pca_g->vertical.slow_peaks.num[0] = count1;
		}
	}		//if(pca_g->vertical.slow_peaks.num[0] > 1)

	if(pca->vertical_parametrs[2] < 5.6 && pca_g->vertical.slow_peaks.num[0] > 0)
	{
		for(i = 0; i < pca_g->vertical.slow_peaks.num[0]-1; i++)
		{
			light_tag1 = 0;
			light_tag2 = 0;
			min_h_ind = 99;
			min_l_ind = 99;
			for(j = 0; j < pca_g->attitude.peaks.num[1]; j++)
			{
				if(abs(pca_g->attitude.peaks.high_ind[j] - pca_g->vertical.slow_peaks.low_ind[i]) < min_h_ind)
					min_h_ind = (INT8)(abs(pca_g->attitude.peaks.high_ind[j] - pca_g->vertical.slow_peaks.low_ind[i]));
			}
			for(j = 0; j < pca_g->attitude.peaks.num[0]; j++)
			{
				if(abs(pca_g->attitude.peaks.low_ind[j] - pca_g->vertical.slow_peaks.low_ind[i]) < min_l_ind)
					min_l_ind = (INT8)(abs(pca_g->attitude.peaks.low_ind[j] - pca_g->vertical.slow_peaks.low_ind[i]));
			}
			if(min_h_ind < min_l_ind && min_h_ind < 4)
				light_tag1++;
			if(min_h_ind > min_l_ind && min_l_ind < 4)
				light_tag2++;

			if(light_tag1 == 1)
			{
				min_h_ind = 99;
				min_l_ind = 99;
				for(j = 0; j < pca_g->attitude.peaks.num[1]; j++)
				{
					if(abs(pca_g->attitude.peaks.high_ind[j] - pca_g->vertical.slow_peaks.low_ind[i+1]) < min_h_ind)
						min_h_ind = (INT8)(abs(pca_g->attitude.peaks.high_ind[j] - pca_g->vertical.slow_peaks.low_ind[i+1]));
				}
				for(j = 0; j < pca_g->attitude.peaks.num[0]; j++)
				{
					if(abs(pca_g->attitude.peaks.low_ind[j] - pca_g->vertical.slow_peaks.low_ind[i+1]) < min_l_ind)
						min_l_ind = (INT8)(abs(pca_g->attitude.peaks.low_ind[j] - pca_g->vertical.slow_peaks.low_ind[i+1]));
				}
				if(min_h_ind > min_l_ind && min_l_ind < 4)
					light_tag1++;
			}

			if(light_tag2 == 1)
			{
				min_h_ind = 99;
				min_l_ind = 99;
				for(j = 0; j < pca_g->attitude.peaks.num[1]; j++)
				{
					if(abs(pca_g->attitude.peaks.high_ind[j] - pca_g->vertical.slow_peaks.low_ind[i+1]) < min_h_ind)
						min_h_ind = (INT8)(abs(pca_g->attitude.peaks.high_ind[j] - pca_g->vertical.slow_peaks.low_ind[i+1]));
				}
				for(j = 0; j < pca_g->attitude.peaks.num[0]; j++)
				{
					if(abs(pca_g->attitude.peaks.low_ind[j] - pca_g->vertical.slow_peaks.low_ind[i+1]) < min_l_ind)
						min_l_ind = (INT8)(abs(pca_g->attitude.peaks.low_ind[j] - pca_g->vertical.slow_peaks.low_ind[i+1]));
				}
				if(min_h_ind < min_l_ind && min_h_ind < 4)
					light_tag2++;
			}
			if(light_tag1 == 2 || light_tag2 == 2)
				break;

		}//for(i = 0; i < pca_g->vertical.slow_peaks.num[0]-1; i++)
	}

	for(i = 0; i < pca_g->vertical.peaks.num[0]; i++)
	{
		min_l_ind = 99;
		min_h_ind = 99;
		for(j = 0; j < pca_g->motion.slow_peaks.num[1]; j++)
		{
			if(abs(pca_g->vertical.peaks.low_ind[i]-pca_g->motion.slow_peaks.high_ind[j]) < min_h_ind && (pca_g->motion.slow_peaks.high_ind[j]-pca_g->vertical.peaks.low_ind[i]) > 0)
				min_h_ind = pca_g->motion.slow_peaks.high_ind[j]-pca_g->vertical.peaks.low_ind[i];
		}
		for(j = 0; j < pca_g->motion.slow_peaks.num[0]; j++)
		{
			if(abs(pca_g->vertical.peaks.low_ind[i]-pca_g->motion.slow_peaks.low_ind[j]) < min_l_ind && (pca_g->motion.slow_peaks.low_ind[j]-pca_g->vertical.peaks.low_ind[i]) > 0)
				min_l_ind = pca_g->motion.slow_peaks.low_ind[j]-pca_g->vertical.peaks.low_ind[i];
		}
		if(min_h_ind < min_l_ind && min_h_ind < 4)    //&& min_l < 20) //revised on 2012.08.08, "min_l < 20" is added to avoid min_l == 99
			step_b++;

		if(min_l_ind < min_h_ind && min_l_ind < 4)   //&& min_h < 20) //revised on 2012.08.08, "min_l < 20" is added to avoid min_h == 99
			step_f++;
	}

	if(light_tag1 == 2 || light_tag2 == 2)
	{
		count1 = step_f;
		step_f = step_b;
		step_b = count1;
	}

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is phone and tablet general_phone_tablet_7_mis function to solve the 180
* 			amiguity problem in case of phone or tablet general data.
*
* @details	This function takes a pointer of structure of type PCA_G. It deals with the
* 			compass data that has slow motion. It uses the realtion between the vertical and
* 			motion peaks to provide a decision for Forward/Backward.
*
* @param[in]	pca_g	pointer to a structure of type \c PCA_G.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*
* @see peaks_min_distance_mis().
*/
INT8 general_phone_tablet_7_mis(PCA_G_Ptr pca_g)
{
	INT8 i;
	INT8 step[2] = {0};
	UINT8 min_peaks_dis[4] = {0, 0, 0, 0};
	i = peaks_min_distance_mis(pca_g, min_peaks_dis, 5, 0, step);
	if(step[0] > step[1])
		return 0;
	else if(step[1] > step[0])
		return 1;
	else
		return 2;
}
/**
* @brief	This is phone and tablet general_phone_tablet_8_mis function to solve the 180
* 			amiguity problem in case of phone or tablet general data.
*
* @details	This function takes a pointer of structure of type PCA_G. It deals with the
* 			data of light dangling cases that misclassified as general. It uses the realtion
* 			between the vertical and motion peaks and the motion data slope to provide a
* 			decision for Forward/Backward. It only uses the motion low and high peaks that
* 			are less or greater than certain thresholds.
*
* @param[in]	pca_g	pointer to a structure of type \c PCA_G.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*
* @see motion_peaks_segment_mis().
*/
INT8 general_phone_tablet_8_mis(PCA_G_Ptr pca_g)
{
	INT8 i;
	INT8 st_ind = -1, en_ind = -1;
	INT8 step[2] = {0};

	for(i=0; i<pca_g->vertical.peaks.num[1]; i++)
	{
		if(pca_g->vertical.peaks.high_val[i] > pca_g->vertical.thresholds[1])
		{
			st_ind = MAX(pca_g->vertical.peaks.high_ind[i]-3, 0);
			en_ind = MIN(pca_g->vertical.peaks.high_ind[i]+3, DATA_SIZE_MIS-1);
			motion_peaks_segment_mis(&pca_g->motion.peaks, pca_g->motion.thresholds, st_ind, en_ind, 2, 1, step);
		}
	}
	// check  for  the vertical low peaks
	for(i=0; i<pca_g->vertical.peaks.num[0]; i++)
	{
		if(pca_g->vertical.peaks.low_val[i] < pca_g->vertical.thresholds[0])
		{
			st_ind = MAX(pca_g->vertical.peaks.low_ind[i]-3, 0);
			en_ind = MIN(pca_g->vertical.peaks.low_ind[i]+3, DATA_SIZE_MIS-1);
			motion_peaks_segment_mis(&pca_g->motion.peaks, pca_g->motion.thresholds, st_ind, en_ind, 2, 0, step);
		}
	}
	if(step[0] > step[1])
		return 0;
	else if(step[1] > step[0])
		return 1;
	else
		return 2;
}
/**
* @brief	This is phone and tablet general_phone_tablet_9_mis function to solve the 180
* 			amiguity problem in case of phone or tablet general data.
*
* @details	This function takes a pointer of structure of type PCA_G. It deals with the
* 			data of ear talking cases. It uses the realtion between the vertical and motion
* 			peaks to provide a decision for Forward/Backward.
*
* @param[in]	pca_g	pointer to a structure of type \c PCA_G.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*
* @see peaks_min_distance_mis().
*/
INT8 general_phone_tablet_9_mis(PCA_G_Ptr pca_g)
{
	INT8 i;
	INT8 step[2] = {0};
	UINT8 min_peaks_dis[4] = {0,0,0,0};

	i = peaks_min_distance_mis(pca_g, min_peaks_dis, 0, 0, step);

	if(min_peaks_dis[3] >= (pca_g->vertical.peaks.num[1]-1))
		step[1]++;
	if(min_peaks_dis[2] >= (pca_g->vertical.peaks.num[1]-1))
		step[0]++;

	if(min_peaks_dis[0] >= (pca_g->vertical.peaks.num[0]-1))
		step[0]++;
	if(min_peaks_dis[1] >= (pca_g->vertical.peaks.num[0]-1))
		step[1]++;

	if(step[0] > step[1])
		return 0;
	else if(step[1] > step[0])
		return 1;
	else
		return 2;
}
/**
* @brief	This is phone and tablet general_phone_tablet_10_mis function to solve the 180
* 			amiguity problem in	case of phone or tablet general data.
*
* @details	This function takes a pointer of structure of type PCA_G. It deals with the
* 			data of ear talking cases. It uses the realtion between the vertical and motion
* 			peaks to provide a decision for Forward/Backward.
*
* @param[in]	pca_g	pointer to a structure of type \c PCA_G.
* @param[in]	pca		pointer to a structure of type \c PCA.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 general_phone_tablet_10_mis(PCA_G_Ptr pca_g, PCA_Ptr pca)
{
	INT8 i;
	INT8 st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;

	if((pca_g->vertical.peaks.high_val[1]-pca_g->vertical.peaks.high_val[0])> 0.35 && (pca_g->vertical.peaks.high_val[1]-pca_g->vertical.peaks.high_val[2])> 0.35 && fabs(pca_g->vertical.peaks.high_val[2]-pca_g->vertical.peaks.high_val[0]) < 1.1 && pca_g->vertical.peaks.num[1] == 3)
	{
		st_ind = 0;
		en_ind = DATA_SIZE_MIS-1;
		for(i = 0; i < pca_g->vertical.peaks.num[0]; i++)
		{
			if(pca_g->vertical.peaks.low_ind[i] < pca_g->vertical.peaks.high_ind[1])
				st_ind = pca_g->vertical.peaks.low_ind[i];
		}
		for(i = 0; i < pca_g->vertical.peaks.num[0]; i++)
		{
			if(pca_g->vertical.peaks.low_ind[i] > pca_g->vertical.peaks.high_ind[1])
			{
				en_ind = pca_g->vertical.peaks.low_ind[i];
				break;
			}
		}

		for(i = 0; i < pca_g->motion.peaks.num[0]; i++)
		{
			if(pca_g->motion.peaks.low_ind[i] < en_ind && pca_g->motion.peaks.low_ind[i] > st_ind && pca_g->motion.peaks.low_val[i] < (pca->motion_parametrs[0]-1.6*pca->motion_parametrs[1]))
				step_b++;
		}
		for(i = 0; i < pca_g->motion.peaks.num[1]; i++)
		{
			if(pca_g->motion.peaks.high_ind[i] < en_ind && pca_g->motion.peaks.high_ind[i] > st_ind && pca_g->motion.peaks.high_val[i] > (pca->motion_parametrs[0]+1.6*pca->motion_parametrs[1]))
				step_f++;
		}
	}

	if((pca_g->vertical.peaks.low_val[0]-pca_g->vertical.peaks.low_val[1])> 0.35 && (pca_g->vertical.peaks.low_val[2]-pca_g->vertical.peaks.low_val[1])> 0.35 && fabs(pca_g->vertical.peaks.low_val[2]-pca_g->vertical.peaks.low_val[0]) < 1.1 && pca_g->vertical.peaks.num[0] == 3)
	{
		st_ind = 0;
		en_ind = DATA_SIZE_MIS-1;
		for(i = 0; i < pca_g->vertical.peaks.num[1]; i++)
		{
			if(pca_g->vertical.peaks.high_ind[i] < pca_g->vertical.peaks.low_ind[1])
				st_ind = pca_g->vertical.peaks.high_ind[i];
		}

		for(i = 0; i < pca_g->vertical.peaks.num[1]; i++)
		{
			if(pca_g->vertical.peaks.high_ind[i] > pca_g->vertical.peaks.low_ind[1])
			{
				en_ind = pca_g->vertical.peaks.high_ind[i];
				break;
			}
		}

		for(i = 0; i < pca_g->motion.peaks.num[0]; i++)
		{
			if(pca_g->motion.peaks.low_ind[i] < en_ind && pca_g->motion.peaks.low_ind[i] > st_ind && pca_g->motion.peaks.low_val[i] < (pca->motion_parametrs[0]-1.6*pca->motion_parametrs[1]))
				step_f++;
		}
		for(i = 0; i < pca_g->motion.peaks.num[1]; i++)
		{
			if(pca_g->motion.peaks.high_ind[i] < en_ind && pca_g->motion.peaks.high_ind[i] > st_ind && pca_g->motion.peaks.high_val[i] > (pca->motion_parametrs[0]+1.6*pca->motion_parametrs[1]))
				step_b++;
		}
	}

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is phone and tablet general_phone_tablet_11_mis function to solve the
* 			180 amiguity problem in	case of phone or tablet general data.
*
* @details	This function takes a pointer of structure of type PCA_G. It deals with the
* 			data of ear talking cases. It uses the realtion between the vertical and motion
* 			peaks and zero crossing technique to provide a decision for Forward/Backward.
*
* @param[in]	pca_g	pointer to a structure of type \c PCA_G.
* @param[in]	pca		pointer to a structure of type \c PCA.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 general_phone_tablet_11_mis(PCA_G_Ptr pca_g, PCA_Ptr pca)
{
	INT8 i, j;
	INT8 tem_ind = -1, b = -1, d = -1;
	INT8 curl_check_h = -1, curl_check_l = -1;
	INT8 step_f = 0, step_b = 0;
	FLOAT32 a = -99.0, c = 99.0;
	FLOAT32 area_up = -1, area_down = -1, tem_tag = 0;

	for(i = 0; i < pca_g->motion.peaks.num[1]; i++)
	{
		if(pca_g->motion.peaks.high_val[i] > a)
		{
			a = pca_g->motion.peaks.high_val[i];
			b = pca_g->motion.peaks.high_ind[i];
		}
	}
	for(i = 0; i < pca_g->motion.peaks.num[0]; i++)
	{
		if(pca_g->motion.peaks.low_val[i] < c)
		{
			c = pca_g->motion.peaks.low_val[i];
			d = pca_g->motion.peaks.low_ind[i];
		}
	}

	for(i = b+1; i <= MIN(b+10, DATA_SIZE_MIS-2); i++)
	{
		if((pca->data_m[i]-pca->data_m[i-1])*(pca->data_m[i+1]-pca->data_m[i]) < 0 && pca->data_m[i] > pca->motion_parametrs[0] && pca->data_m[i] < pca->data_m[b])		//slope change
		{
			curl_check_h = i;
			break;
		}
	}

	for(i = d+1; i <= MIN(d+10,DATA_SIZE_MIS-2); i++)
	{
		if((pca->data_m[i]-pca->data_m[i-1])*(pca->data_m[i+1]-pca->data_m[i]) < 0 && pca->data_m[i] < pca->motion_parametrs[0] && pca->data_m[i] > pca->data_m[d])		//slope change
		{
			curl_check_l = i;
			break;
		}

	}
	if(curl_check_h == -1) //this peak doesn't have local bump after it
	{
		tem_tag = -99;
		tem_ind = -1;
		for(i = 0; i < pca_g->motion.peaks.num[1]; i++)
		{
			if(pca_g->motion.peaks.high_val[i] > tem_tag && pca_g->motion.peaks.high_val[i] != a)
			{
				tem_tag = pca_g->motion.peaks.high_val[i];
				tem_ind = pca_g->motion.peaks.high_ind[i];
			}
		}
		if(tem_ind != -1)
		{
			a = tem_tag;
			b = tem_ind;
			for(i = b+1; i <= MIN(b+10,DATA_SIZE_MIS-2); i++)
			{
				if((pca->data_m[i]-pca->data_m[i-1])*(pca->data_m[i+1]-pca->data_m[i]) < 0 && pca->data_m[i] > pca->motion_parametrs[0] && pca->data_m[i] < pca->data_m[b])   //slope change
				{
					curl_check_h = i;
					break;
				}
			}
		}
	}

	if(curl_check_l == -1) //this peak doesn't have local bump after it
	{
		tem_tag = 99;
		tem_ind = -1;
		for(i = 0; i < pca_g->motion.peaks.num[0]; i++)
			if(pca_g->motion.peaks.low_val[i] < tem_tag && pca_g->motion.peaks.low_val[i] != c)
			{
				tem_tag = pca_g->motion.peaks.low_val[i];
				tem_ind = pca_g->motion.peaks.low_ind[i];
			}

			if(tem_ind != -1)
			{
				c = tem_tag;
				d = tem_ind;
				for(i = d+1; i <= MIN(d+10,DATA_SIZE_MIS-2); i++)
				{
					if((pca->data_m[i]-pca->data_m[i-1])*(pca->data_m[i+1]-pca->data_m[i]) < 0 && pca->data_m[i] < pca->motion_parametrs[0] && pca->data_m[i] > pca->data_m[d])         //slope change
					{
						curl_check_l = i;
						break;
					}
				}
			}
	}
	if(curl_check_l != -1 && curl_check_h != -1)
	{
		area_up = 0;
		area_down = 0;
		//find intersection after this tag
		for(i = curl_check_h; i < DATA_SIZE_MIS-1; i++) //zero crossing
		{
			if(pca->data_m[i] > 0.0 && pca->data_m[i+1] < 0)
			{
				for(j = curl_check_h; j <= i; j++)
					area_up += pca->data_m[j];
				break;
			}
		}
		for(i = curl_check_l; i < DATA_SIZE_MIS-1; i++) //zero crossing
		{
			if(pca->data_m[i] < 0.0 && pca->data_m[i+1] > 0)
			{
				for(j = curl_check_l; j <= i; j++)
					area_down += pca->data_m[j];
				break;
			}
		}
		if(area_up == 0) //highly because of no zero crossing
		{
			tem_tag = -99;
			tem_ind = -1;
			for(i = 0; i < pca_g->motion.peaks.num[1]; i++)
			{
				if(pca_g->motion.peaks.high_val[i] > tem_tag && pca_g->motion.peaks.high_val[i] != a)
				{
					tem_tag = pca_g->motion.peaks.high_val[i];
					tem_ind = pca_g->motion.peaks.high_ind[i];
				}
			}

			if(tem_ind != -1)
			{
				a = tem_tag;
				b = tem_ind;
				for(i = b+1; i <= MIN(b+10, DATA_SIZE_MIS-2); i++)
				{
					if((pca->data_m[i]-pca->data_m[i-1])*(pca->data_m[i+1]-pca->data_m[i]) < 0 && pca->data_m[i] > pca->motion_parametrs[0] && pca->data_m[i] < pca->data_m[b])//slope change
					{
						curl_check_h = i;
						break;
					}
				}

				area_up = 0;
				for(i = curl_check_h; i < DATA_SIZE_MIS-1; i++) //zero crossing
				{
					if(pca->data_m[i] > 0.0 && pca->data_m[i+1]<0)
					{
						for(j = curl_check_h; j <= i; j++)
							area_up += pca->data_m[j];
						break;
					}
				}
			}
		}//if(area_up == 0) //highly because of no zero crossing

		if(area_down == 0) //highly because of no zero crossing in negative space
		{
			tem_tag = 99;
			tem_ind = -1;
			for(i = 0; i < pca_g->motion.peaks.num[0]; i++)
			{
				if(pca_g->motion.peaks.low_val[i] < tem_tag && pca_g->motion.peaks.low_val[i] != c)
				{
					tem_tag = pca_g->motion.peaks.low_val[i];
					tem_ind = pca_g->motion.peaks.low_ind[i];
				}
			}
			if(tem_ind != -1)
			{
				c = tem_tag;
				d = tem_ind;
				for(i = d+1; i <= MIN(d+10, DATA_SIZE_MIS-2); i++)
				{
					if((pca->data_m[i]-pca->data_m[i-1])*(pca->data_m[i+1]-pca->data_m[i]) < 0 && pca->data_m[i] < pca->motion_parametrs[0] && pca->data_m[i] > pca->data_m[d])//slope change
					{
						curl_check_l = i;
						break;
					}
				}
				area_down = 0;
				for(i = curl_check_l; i < DATA_SIZE_MIS-1; i++) //zero crossing
				{
					if(pca->data_m[i] < 0.0 && pca->data_m[i+1] > 0)
					{
						for(j = curl_check_l; j <= i; j++)
							area_down += pca->data_m[j];
						break;
					}
				}
			}
		}//if(area_down == 0) //highly because of no zero crossing
		if(area_up != 0 && area_down != 0)
		{
			if(fabs(area_up) > fabs(area_down))
				step_b++;
			if(fabs(area_up) < fabs(area_down))
				step_f++;
		}
	}//if(curl_check_l != -1 && curl_check_h != -1)

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is phone and tablet general_phone_tablet_12_mis function to solve the
* 			180 amiguity problem in	case of phone or tablet general data.
*
* @details	This function takes a pointer of structure of type PCA_G. It deals with the
* 			data of belt that has reverse logic than the normal one . It uses the realtion
* 			between the vertical and motion	peaks to provide a decision for Forward/Backward.
*
* @param[in]	pca_g	pointer to a structure of type \c PCA_G.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 general_phone_tablet_12_mis(PCA_G_Ptr pca_g)
{
	INT8 i, j;
	INT8 tem_ind = -1, b = -1;
	INT8 k1 = 0, k2 = 0, k3 = 0, k4 = 0, k5 = 0;
	INT8 step_f = 0, step_b = 0;
	FLOAT32 a = -99.0;
	FLOAT32 tem_tag = 0;

	a = -99.0;
	b = -1;
	for(i = 0; i < pca_g->vertical.peaks.num[1]; i++)
	{
		if(pca_g->vertical.peaks.high_val[i] > a)
		{
			a = pca_g->vertical.peaks.high_val[i];
			b = i;
		}
	}
	if(pca_g->vertical.peaks.high_ind[b] > 26) //too close to the end, will have not enough motion peaks after
	{
		tem_tag = -99;
		tem_ind = -1;
		for(i = 0; i < pca_g->vertical.peaks.num[1]; i++)
		{
			if(pca_g->vertical.peaks.high_val[i] > tem_tag && pca_g->vertical.peaks.high_val[i] != a)
			{
				tem_tag = pca_g->vertical.peaks.high_val[i];
				tem_ind = i;
			}
		}
		a = tem_tag;
		b = tem_ind;
	}

	k1 = -1;
	k2 = -1;
	k3 = -1;
	for(j=0; j<pca_g->motion.local_peaks.num[0]; j++) //k1 =find(pca_g->motion.ml_peaks_local(:,1) > pca_g->vertical.vh_peaks(b,1));
	{
		if(pca_g->motion.local_peaks.low_ind[j] > (pca_g->vertical.peaks.high_ind[b]+1))
		{
			k1 = j;
			break;
		}
	}

	for(j=0; j<pca_g->motion.local_peaks.num[1]; j++) //k2 =find(pca_g->motion.mh_peaks_local(:,1) > pca_g->vertical.vh_peaks(b,1));
	{
		if(pca_g->motion.local_peaks.high_ind[j] > (pca_g->vertical.peaks.high_ind[b]+1))
		{
			k2 = j;
			break;
		}
	}
	for(j = 0; j < pca_g->vertical.peaks.num[0]; j++)//k3 = find(pca_g->vertical.vl_peaks(:,1) > pca_g->vertical.vh_peaks(b,1));
	{
		if(pca_g->vertical.peaks.low_ind[j] > (pca_g->vertical.peaks.high_ind[b]+1))
		{
			k3 = j;
			break;
		}
	}

	if(k1 != -1 && k2 != -1 && k3 != -1)
	{
		k4 = -1;
		k5 = -1;
		//k4 = find(pca_g->motion.mh_peaks_local(:,1) > pca_g->motion.ml_peaks_local(k1(1),1));
		for(j=0; j<pca_g->motion.local_peaks.num[1]; j++)
		{
			if(pca_g->motion.local_peaks.high_ind[j] > pca_g->motion.local_peaks.low_ind[k1])
			{
				k4 = j;
				break;
			}
		}
		//k5 = find(pca_g->motion.ml_peaks_local(:,1) > pca_g->motion.mh_peaks_local(k2(1),1));
		for(j=0; j<pca_g->motion.local_peaks.num[0]; j++)
		{
			if(pca_g->motion.local_peaks.low_ind[j] > pca_g->motion.local_peaks.high_ind[k2])
			{
				k5 = j;
				break;
			}
		}

		if(k4 != -1)
		{
			if(k4 != k2)
			{
				if(pca_g->motion.local_peaks.low_ind[k1] <= (pca_g->vertical.peaks.low_ind[k3] + 1) && abs(pca_g->motion.local_peaks.high_ind[k4] - pca_g->vertical.peaks.low_ind[k3]) < 6 && pca_g->motion.local_peaks.high_ind[k4] > pca_g->vertical.peaks.low_ind[k3])
				{
					step_b++;
					if(b < (pca_g->vertical.peaks.num[1]-1))
					{
						if(pca_g->motion.local_peaks.high_ind[k4] > pca_g->vertical.peaks.high_ind[b+1])
							step_b--;
					}
				}
			}
			else
			{
				if(pca_g->motion.local_peaks.low_ind[k1] > pca_g->vertical.peaks.low_ind[k3] && (pca_g->motion.local_peaks.high_ind[k4] - pca_g->motion.local_peaks.low_ind[k1]) > 3)
					step_f++;
				else
					step_b++;
			}
		}
		if(k5 != -1 && step_f == step_b)
		{
			if(k5 != k1)
			{
				if(pca_g->motion.local_peaks.high_ind[k2] <= (pca_g->vertical.peaks.low_ind[k3]+1) && abs(pca_g->motion.local_peaks.low_ind[k5]-pca_g->vertical.peaks.low_ind[k3]) < 6 && pca_g->motion.local_peaks.low_ind[k5] > pca_g->vertical.peaks.low_ind[k3])
				{
					step_f++;
					if(b < (pca_g->vertical.peaks.num[1]-1))
					{
						if(pca_g->motion.local_peaks.low_ind[k5] > pca_g->vertical.peaks.high_ind[b+1])
							step_f--;
					}
				}
			}
			else
			{
				if(pca_g->motion.local_peaks.high_ind[k2] > pca_g->vertical.peaks.low_ind[k3] && (pca_g->motion.local_peaks.low_ind[k5] - pca_g->motion.local_peaks.high_ind[k2]) > 3)
					step_b++;
				else
					step_f++;
			}
		}
	}//if(k1 != -1 && k2 != -1 && k3 != -1)
	else  //change to another peaks, because this peak might be too close to the end
	{
		tem_tag = -99;
		tem_ind = -1;
		for(i = 0; i < pca_g->vertical.peaks.num[1]; i++)
		{
			if(pca_g->vertical.peaks.high_val[i] > tem_tag && pca_g->vertical.peaks.high_val[i] != a)
			{
				tem_tag = pca_g->vertical.peaks.high_val[i];
				tem_ind = i;
			}
		}
		if(abs(tem_ind - b) > 1)
		{
			a = tem_tag;
			b= tem_ind;
			k1 = -1;
			k2 = -1;
			k3 = -1;
			for(j=0; j<pca_g->motion.local_peaks.num[0]; j++) //k1 =find(pca_g->motion.ml_peaks_local(:,1) > pca_g->vertical.vh_peaks(b,1));
			{
				if(pca_g->motion.local_peaks.low_ind[j] > (pca_g->vertical.peaks.high_ind[b]+1))
				{
					k1 = j;
					break;
				}
			}

			for(j=0; j<pca_g->motion.local_peaks.num[1]; j++) //k2 =find(pca_g->motion.mh_peaks_local(:,1) > pca_g->vertical.vh_peaks(b,1));
			{
				if(pca_g->motion.local_peaks.high_ind[j] > (pca_g->vertical.peaks.high_ind[b]+1))
				{
					k2 = j;
					break;
				}
			}
			for(j = 0; j < pca_g->vertical.peaks.num[0]; j++)//k3 = find(pca_g->vertical.vl_peaks(:,1) > pca_g->vertical.vh_peaks(b,1));
			{
				if(pca_g->vertical.peaks.low_ind[j] > (pca_g->vertical.peaks.high_ind[b]+1))
				{
					k3 = j;
					break;
				}
			}

			if(k1 != -1 && k2 != -1 && k3 != -1)
			{
				k4 = -1;
				k5 = -1;
				//k4 = find(pca_g->motion.mh_peaks_local(:,1) > pca_g->motion.ml_peaks_local(k1(1),1));
				for(j=0; j<pca_g->motion.local_peaks.num[1]; j++)
				{
					if(pca_g->motion.local_peaks.high_ind[j] > pca_g->motion.local_peaks.low_ind[k1])
					{
						k4 = j;
						break;
					}
				}
				//k5 = find(pca_g->motion.ml_peaks_local(:,1) > pca_g->motion.mh_peaks_local(k2(1),1));
				for(j=0; j<pca_g->motion.local_peaks.num[0]; j++)
				{
					if(pca_g->motion.local_peaks.low_ind[j] > pca_g->motion.local_peaks.high_ind[k2])
					{
						k5 = j;
						break;
					}
				}

				if(k4 != -1)
				{
					if(k4 != k2)
					{
						if(pca_g->motion.local_peaks.low_ind[k1] <= (pca_g->vertical.peaks.low_ind[k3]+1) && abs(pca_g->motion.local_peaks.high_ind[k4]-pca_g->vertical.peaks.low_ind[k3]) < 6 && pca_g->motion.local_peaks.high_ind[k4] > pca_g->vertical.peaks.low_ind[k3])
						{
							step_b++;
							if(b < (pca_g->vertical.peaks.num[1]-1))
							{
								if(pca_g->motion.local_peaks.high_ind[k4] > pca_g->vertical.peaks.high_ind[b+1])
									step_b--;
							}
						}
					}
					else
					{
						if(pca_g->motion.local_peaks.low_ind[k1] > pca_g->vertical.peaks.low_ind[k3] && (pca_g->motion.local_peaks.high_ind[k4]-pca_g->motion.local_peaks.low_ind[k1]) > 3)
							step_f++;
						else
							step_b++;
					}
				}
				if(step_f == step_b && k5 != -1)
				{
					if(k5 != k1)
					{
						if(pca_g->motion.local_peaks.high_ind[k2] <= (pca_g->vertical.peaks.low_ind[k3]+1) && abs(pca_g->motion.local_peaks.low_ind[k5]-pca_g->vertical.peaks.low_ind[k3]) < 6 && pca_g->motion.local_peaks.low_ind[k5] > pca_g->vertical.peaks.low_ind[k3])
						{
							step_f++;
							if(b < (pca_g->vertical.peaks.num[1]-1))
							{
								if(pca_g->motion.local_peaks.low_ind[k5] > pca_g->vertical.peaks.high_ind[b+1])
									step_f--;
							}
						}
					}
					else
					{
						if(pca_g->motion.local_peaks.high_ind[k2] > pca_g->vertical.peaks.low_ind[k3] && (pca_g->motion.local_peaks.low_ind[k5]-pca_g->motion.local_peaks.high_ind[k2]) > 3)
							step_b++;
						else
							step_f++;
					}
				}
			}//if(k1 != -1 && k2 != -1 && k3 != -1)
		}
	}//if(k1 != -1 && k2 != -1 && k3 != -1)
	if(step_f == step_b) //Second new part added on July 4, 2012.
	{
		a = -99.0;
		b = -1;
		for(i = 0; i < pca_g->vertical.peaks.num[1]; i++)
		{
			if(pca_g->vertical.peaks.high_val[i] > a)
			{
				a = pca_g->vertical.peaks.high_val[i];
				b = i;
			}
		}
		if(pca_g->vertical.peaks.high_ind[b] > 26) //too close to the end, will have not enough motion peaks after
		{
			tem_tag = -99;
			tem_ind = -1;
			for(i = 0; i < pca_g->vertical.peaks.num[1]; i++)
			{
				if(pca_g->vertical.peaks.high_val[i] > tem_tag && pca_g->vertical.peaks.high_val[i] != a)
				{
					tem_tag = pca_g->vertical.peaks.high_val[i];
					tem_ind = i;
				}
			}
			a = tem_tag;
			b = tem_ind;
		}
		k3 = -1;
		for(j = 0; j < pca_g->vertical.peaks.num[0]; j++)//k3 = find(pca_g->vertical.vl_peaks(:,1) > pca_g->vertical.vh_peaks(b,1));
		{
			if(pca_g->vertical.peaks.low_ind[j] > (pca_g->vertical.peaks.high_ind[b]+1))
			{
				k3 = j;
				break;
			}
		}
		if(k3 != -1)
		{
			for(j=0; j<pca_g->motion.peaks.num[1]; j++) //k2 =find(pca_g->motion.mh_peaks_local(:,1) > pca_g->vertical.vh_peaks(b,1));
			{
				if(abs(pca_g->motion.peaks.high_ind[j] - pca_g->vertical.peaks.low_ind[k3]) < 6 && pca_g->motion.peaks.high_ind[j] > pca_g->vertical.peaks.low_ind[k3] && pca_g->motion.peaks.high_val[j] > pca_g->motion.thresholds[1])
					step_b++;
			}
			for(j=0; j<pca_g->motion.peaks.num[0]; j++) //k1 =find(pca_g->motion.ml_peaks_local(:,1) > pca_g->vertical.vh_peaks(b,1));
			{
				if(abs(pca_g->motion.peaks.low_ind[j] - pca_g->vertical.peaks.low_ind[k3]) < 6 && pca_g->motion.peaks.low_ind[j] > pca_g->vertical.peaks.low_ind[k3] && pca_g->motion.peaks.low_val[j] < pca_g->motion.thresholds[0])
					step_f++;
			}
		}
	}//End second new part added on July 3. HWC

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is phone and tablet general_phone_tablet_13_mis function to solve the
* 			180 amiguity problem in	case of phone or tablet general data.
*
* @details	This function takes 2 pointers of structures of type PCA_G and PCA. It deals with the
* 			data of misclassified pocket data that has reverse logic than the normal one.
* 			It uses the realtion between the vertical and motion peaks and motion data slope
* 			to provide a decision for Forward/Backward.
*
* @param[in]	pca_g	pointer to a structure of type \c PCA_G.
* @param[in]	pca		pointer to a structure of type \c PCA.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*
* @see slope_segment_mis().
*/
INT8 general_phone_tablet_13_mis(PCA_G_Ptr pca_g, PCA_Ptr pca)
{
	INT8 i, j;
	INT8 tem_ind = -1, b = -1;
	INT8 k1=0, k2=0, k3=0, k4=0, k5=0;
	INT8 step_f = 0, step_b = 0, step[2] = {0};

	FLOAT32 a= -99.0, tem_tag=0;
	FLOAT32 max_vh = -99, min_vh = 99;

	for(i =0; i < pca_g->vertical.peaks.num[1]; i++)
	{
		if(pca_g->vertical.peaks.high_val[i] < min_vh)
			min_vh = pca_g->vertical.peaks.high_val[i];
		if(pca_g->vertical.peaks.high_val[i] > max_vh)
			max_vh = pca_g->vertical.peaks.high_val[i];
	}

	for(i = 0; i < pca_g->vertical.peaks.num[1]; i++)
	{
		if(pca_g->vertical.peaks.high_val[i] > a)
		{
			a = pca_g->vertical.peaks.high_val[i];
			b = i;
		}
	}

	if(pca_g->vertical.peaks.high_ind[b] > 26) //too close to the end, will have not enough motion peaks after
	{
		tem_tag = -99.0;
		tem_ind = -1;
		for(i = 0; i < pca_g->vertical.peaks.num[1]; i++)
		{
			if(pca_g->vertical.peaks.high_val[i] > tem_tag && pca_g->vertical.peaks.high_val[i] != a)
			{
				tem_tag = pca_g->vertical.peaks.high_val[i];
				tem_ind = i;
			}
		}
		a = tem_tag;
		b = tem_ind;
	}
	// Method 7      #10-2-1
	if(pca_g->vertical.peaks.num[1] > 1 && ((max_vh - min_vh) > 1.4 || MAX(pca_g->motion.motion_low_peaks_range, pca_g->motion.motion_high_peaks_range) > 1.6) &&
		pca->motion_parametrs[2] > 4 && pca->motion_parametrs[1] > 1 && pca_g->motion.local_peaks.num[0] != 0 && pca_g->motion.local_peaks.num[1] != 0 &&
		((pca->horizontal_vertical_flag == 0 && fabs(pca->roll_parametrs[0])*R2Df > 60.0) || (pca->horizontal_vertical_flag != 0 &&
		MAX(pca->pitch_parametrs[2], pca->roll_parametrs[2]) > 7.0))) //&& pca_mis.attitude.vec_p[2] > 30)
	{
		k1 = -1;
		k2 = -1;
		k3 = -1;
		for(j=0; j<pca_g->motion.local_peaks.num[0]; j++) //k1 =find(pca_g->motion.ml_peaks_local(:,1) > pca_g->vertical.vh_peaks(b,1));
		{
			if(pca_g->motion.local_peaks.low_ind[j] > (pca_g->vertical.peaks.high_ind[b]+1))
			{
				k1 = j;
				break;
			}
		}

		for(j=0; j<pca_g->motion.local_peaks.num[1]; j++) //k2 =find(pca_g->motion.mh_peaks_local(:,1) > pca_g->vertical.vh_peaks(b,1));
		{
			if(pca_g->motion.local_peaks.high_ind[j] > (pca_g->vertical.peaks.high_ind[b]+1))
			{
				k2 = j;
				break;
			}
		}
		for(j = 0; j < pca_g->vertical.peaks.num[0]; j++)
		{
			if(pca_g->vertical.peaks.low_ind[j] > (pca_g->vertical.peaks.high_ind[b]+1))
			{
				k3 = j;
				break;
			}
		}

		if(k1 != -1 && k2 != -1 && k3 != -1)
		{
			k4 = -1;
			k5 = -1;
			for(j=0; j<pca_g->motion.local_peaks.num[1]; j++)
			{
				if(pca_g->motion.local_peaks.high_ind[j] > pca_g->motion.local_peaks.low_ind[k1])
				{
					k4 = j;
					break;
				}
			}
			for(j=0; j<pca_g->motion.local_peaks.num[0]; j++)
			{
				if(pca_g->motion.local_peaks.low_ind[j] > pca_g->motion.local_peaks.high_ind[k2])
				{
					k5 = j;
					break;
				}
			}

			if(k4 != -1)
			{
				if(pca_g->motion.local_peaks.low_ind[k1] <= (pca_g->vertical.peaks.low_ind[k3]+1) && abs(pca_g->motion.local_peaks.high_ind[k4]-pca_g->vertical.peaks.low_ind[k3]) < 6 && pca_g->motion.local_peaks.high_ind[k4] > pca_g->vertical.peaks.low_ind[k3])
				{
					step_b++;
					if(b < (pca_g->vertical.peaks.num[1]-1) && pca_g->motion.local_peaks.high_ind[k4] > pca_g->vertical.peaks.high_ind[b+1])
						step_b--;
				}
			}
			if(k5 != -1)
			{
				if(pca_g->motion.local_peaks.high_ind[k2] <= (pca_g->vertical.peaks.low_ind[k3]+1) && abs(pca_g->motion.local_peaks.low_ind[k5]-pca_g->vertical.peaks.low_ind[k3]) < 6 && pca_g->motion.local_peaks.low_ind[k5] > pca_g->vertical.peaks.low_ind[k3])
				{
					step_f++;
					if(b < (pca_g->vertical.peaks.num[1]-1) && pca_g->motion.local_peaks.low_ind[k5] > pca_g->vertical.peaks.high_ind[b+1])
						step_f--;
				}
			}
		} //if(k1 != -1 && k2 != -1 && k3 != -1)
		else  //change to another peaks, because this peak might be too close to the end
		{
			tem_tag = -99;
			tem_ind = -1;
			for(i = 0; i < pca_g->vertical.peaks.num[1]; i++)
			{
				if(pca_g->vertical.peaks.high_val[i] > tem_tag && pca_g->vertical.peaks.high_val[i] != a)
				{
					tem_tag = pca_g->vertical.peaks.high_val[i];
					tem_ind = i;
				}
			}
			a = tem_tag;
			b = tem_ind;

			k1 = -1;
			k2 = -1;
			k3 = -1;
			for(j=0; j<pca_g->motion.local_peaks.num[0]; j++) //k1 =find(pca_g->motion.ml_peaks_local(:,1) > pca_g->vertical.vh_peaks(b,1));
			{
				if(pca_g->motion.local_peaks.low_ind[j] > (pca_g->vertical.peaks.high_ind[b]+1))
				{
					k1 = j;
					break;
				}
			}

			for(j=0; j<pca_g->motion.local_peaks.num[1]; j++) //k2 =find(pca_g->motion.mh_peaks_local(:,1) > pca_g->vertical.vh_peaks(b,1));
			{
				if(pca_g->motion.local_peaks.high_ind[j] > (pca_g->vertical.peaks.high_ind[b]+1))
				{
					k2 = j;
					break;
				}
			}
			for(j = 0; j < pca_g->vertical.peaks.num[0]; j++)//k3 = find(pca_g->vertical.vl_peaks(:,1) > pca_g->vertical.vh_peaks(b,1));
			{
				if(pca_g->vertical.peaks.low_ind[j] > (pca_g->vertical.peaks.high_ind[b]+1))
				{
					k3 = j;
					break;
				}
			}

			if(k1 != -1 && k2 != -1 && k3 != -1)
			{
				k4 = -1;
				k5 = -1;
				for(j=0; j<pca_g->motion.local_peaks.num[1]; j++)
				{
					if(pca_g->motion.local_peaks.high_ind[j] > pca_g->motion.local_peaks.low_ind[k1])
					{
						k4 = j;
						break;
					}
				}
				for(j=0; j<pca_g->motion.local_peaks.num[0]; j++)
				{
					if(pca_g->motion.local_peaks.low_ind[j] > pca_g->motion.local_peaks.high_ind[k2])
					{
						k5 = j;
						break;
					}
				}

				if(k4 != -1)
				{
					if(pca_g->motion.local_peaks.low_ind[k1] <= (pca_g->vertical.peaks.low_ind[k3]+1) && abs(pca_g->motion.local_peaks.high_ind[k4]-pca_g->vertical.peaks.low_ind[k3]) < 6 && pca_g->motion.local_peaks.high_ind[k4] > pca_g->vertical.peaks.low_ind[k3])
					{
						step_b++;
						if(b < (pca_g->vertical.peaks.num[1]-1) && pca_g->motion.local_peaks.high_ind[k4] > pca_g->vertical.peaks.high_ind[b+1])
							step_b--;
					}
				}
				if(k5 != -1)
				{
					if(pca_g->motion.local_peaks.high_ind[k2] <= (pca_g->vertical.peaks.low_ind[k3] + 1) && abs(pca_g->motion.local_peaks.low_ind[k5]-pca_g->vertical.peaks.low_ind[k3]) < 6 && pca_g->motion.local_peaks.low_ind[k5] > pca_g->vertical.peaks.low_ind[k3])
					{
						step_f++;
						if(b < (pca_g->vertical.peaks.num[1]-1) && pca_g->motion.local_peaks.low_ind[k5] > pca_g->vertical.peaks.high_ind[b+1])
							step_f--;
					}
				}
			}//if(k1 != -1 && k2 != -1 && k3 != -1)
		}//if(k1 != -1 && k2 != -1 && k3 != -1)
		// End of Method 7      #10-2-1

		// Method 7      #10-2-2
		if(step_f == step_b) //Second new part added on July 3.
		{
			a = -99.0;
			b = -1;
			for(i = 0; i < pca_g->vertical.peaks.num[1]; i++)
			{
				if(pca_g->vertical.peaks.high_val[i] > a)
				{
					a = pca_g->vertical.peaks.high_val[i];
					b = i;
				}
			}
			if(pca_g->vertical.peaks.high_ind[b] > 26) //too close to the end, will have not enough motion peaks after
			{
				tem_tag = -99;
				tem_ind = -1;
				for(i = 0; i < pca_g->vertical.peaks.num[1]; i++)
				{
					if(pca_g->vertical.peaks.high_val[i] > tem_tag && pca_g->vertical.peaks.high_val[i] != a)
					{
						tem_tag = pca_g->vertical.peaks.high_val[i];
						tem_ind = i;
					}
				}
				a = tem_tag;
				b = tem_ind;
			}
			k3 = -1;
			for(j = 0; j < pca_g->vertical.peaks.num[0]; j++)//k3 = find(pca_g->vertical.vl_peaks(:,1) > pca_g->vertical.vh_peaks(b,1));
			{
				if(pca_g->vertical.peaks.low_ind[j] > pca_g->vertical.peaks.high_ind[b])
				{
					k3 = j;
					break;
				}
			}
			if(k3 != -1)
			{
				for(i=0; i<pca_g->motion.peaks.num[1]; i++) //k2 =find(pca_g->motion.mh_peaks_local(:,1) > pca_g->vertical.vh_peaks(b,1));
				{
					if(abs(pca_g->motion.peaks.high_ind[i] - pca_g->vertical.peaks.low_ind[k3]) < 6.0 && pca_g->motion.peaks.high_val[i] > pca_g->motion.thresholds[1] && pca_g->motion.peaks.high_ind[i] > pca_g->vertical.peaks.low_ind[k3])
						step_b++;
				}
				for(i=0; i<pca_g->motion.peaks.num[0]; i++) //k1 =find(pca_g->motion.ml_peaks_local(:,1) > pca_g->vertical.vh_peaks(b,1));
				{
					if(abs(pca_g->motion.peaks.low_ind[i] - pca_g->vertical.peaks.low_ind[k3]) < 6 && pca_g->motion.peaks.low_val[i] < pca_g->motion.thresholds[0] && pca_g->motion.peaks.low_ind[i] > pca_g->vertical.peaks.low_ind[k3])
						step_f++;
				}
			}
		}//End second new part added on July 3. HWC
	}
	// End of Method 7      #10-2-2

	// Method 7      #10-2-3
	else if(pca_g->vertical.peaks.num[1] > 1 && ((max_vh - min_vh) < 1.3 || MAX(pca_g->motion.motion_low_peaks_range, pca_g->motion.motion_high_peaks_range) > 1.6) && pca->motion_parametrs[2] > 4.0 && pca->motion_parametrs[1] > 2.5 && (pca->vertical_parametrs[2]/pca->motion_parametrs[2]) > 0.8 && (pca->vertical_parametrs[2]/pca->motion_parametrs[2]) < 1.4 && ((pca->horizontal_vertical_flag == 0 && fabs(pca->roll_parametrs[0])*R2Df > 60.0) || (pca->horizontal_vertical_flag != 0  && MAX(pca->roll_parametrs[2], pca->pitch_parametrs[2]) > 7.0)))
	{
		slope_segment_mis(pca_g->vertical.peaks.num[1],pca_g->vertical.peaks.high_ind, pca_g->vertical.peaks.high_val, pca->data_m, 1, 1, 0, pca_g->vertical.thresholds[1], step, 1);
		step_f += step[0];
		step_b += step[1];
	}
	// End of Method 7      #10-2-3

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is phone and tablet general_phone_tablet_14_mis function to solve
* 			the 180 amiguity problem in	case of phone or tablet general data.
*
* @details	This function takes 2 pointers of structures of type PCA_G and PCA. It is a simplified version
* 			of algorithm to deal with compass data. It uses the realtion between the vertical and
* 			motion peaks to provide a decision for Forward/Backward.
*
* @param[in]	pca_g	pointer to a structure of type \c PCA_G.
* @param[in]	pca		pointer to a structure of type \c PCA.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*
* @see peaks_min_distance_mis() and slope_segment_mis().
*/
INT8 general_phone_tablet_14_mis(PCA_G_Ptr pca_g, PCA_Ptr pca)
{
	INT8 i;
	INT8 step[2]={0};
	UINT8 min_peaks_dis[4] = {0, 0, 0, 0};

	i = peaks_min_distance_mis(pca_g, min_peaks_dis, 0, 0, step);

	if(min_peaks_dis[0] >= (pca_g->vertical.peaks.num[0]-1) && min_peaks_dis[0] != 0) // method_sl   v_l
		step[0]++;
	if(min_peaks_dis[1] >= (pca_g->vertical.peaks.num[0]-1) && min_peaks_dis[1] != 0) // method_sh   v_l
		step[1]++;
	if(min_peaks_dis[2] >= (pca_g->vertical.peaks.num[1]-1) && min_peaks_dis[2] != 0) // method_sh   v_h
		step[0]++;
	if(min_peaks_dis[3] >= (pca_g->vertical.peaks.num[1]-1) && min_peaks_dis[3] != 0) // method_sl   v_h
		step[1]++;

	// 14_1_new     December 2013   Unstable Handheld
	if(MAX(pca->roll_parametrs[2], pca->pitch_parametrs[2]) < 7.0 && MAX(fabs(pca->roll_parametrs[0]*R2Df), fabs(pca->pitch_parametrs[0]*R2Df)) < 70.0)
	{
		slope_segment_mis(pca_g->vertical.peaks.num[1], pca_g->vertical.peaks.high_ind, pca_g->vertical.peaks.high_val, pca->data_m, 1, 1, 0, pca_g->vertical.thresholds[1], step, 1);
		slope_segment_mis(pca_g->vertical.peaks.num[0], pca_g->vertical.peaks.low_ind, pca_g->vertical.peaks.low_val, pca->data_m, 1, 1, 0, pca_g->vertical.thresholds[0], step, 0);
	}
	// End of 14_1_new     December 2013

	if(step[0] > step[1])
		return 0;
	else if(step[1] > step[0])
		return 1;
	else
		return 2;
}
/**
* @brief	This is phone and tablet general_phone_tablet_15_mis function to solve the 180
* 			amiguity problem in	case of phone or tablet general data.
*
* @details	This function takes 2 pointers of structures of type PCA_G and PCA. It deals with belt noisy data
* 			with . It uses the realtion between the vertical and motion peaks to provide a decision for
* 			Forward/Backward.
*
* @param[in]	pca_g	pointer to a structure of type \c PCA_G.
* @param[in]	pca		pointer to a structure of type \c PCA.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 general_phone_tablet_15_mis(PCA_G_Ptr pca_g, PCA_Ptr pca)
{
	INT8 i, j, k;
	INT8 st_ind=-1, en_ind=-1;
	INT8 step_f = 0, step_b = 0;

	if((pca->vertical_parametrs[2]/pca->motion_parametrs[2]) > 2)
	{
		for(i=0; i<pca_g->vertical.peaks.num[1]; i++)
		{
			if(pca_g->vertical.peaks.high_val[i] > pca_g->vertical.thresholds[3])
			{
				st_ind = MAX(pca_g->vertical.peaks.high_ind[i]-2, 0);
				en_ind = MIN(pca_g->vertical.peaks.high_ind[i]+2, DATA_SIZE_MIS-1);

				for(j=0; j<pca_g->motion.peaks.num[1]; j++)
				{
					if(pca_g->motion.peaks.high_ind[j] >= st_ind && pca_g->motion.peaks.high_ind[j] <= en_ind && pca_g->motion.peaks.high_val[j] > pca_g->motion.thresholds[1])
						step_b++;
				}
				for(j=0; j<pca_g->motion.peaks.num[0]; j++)
				{
					if(pca_g->motion.peaks.low_ind[j] >= st_ind && pca_g->motion.peaks.low_ind[j] <= en_ind && pca_g->motion.peaks.low_val[j] < pca_g->motion.thresholds[0])
						step_f++;
				}
			}
		}
		// check  for  the vertical low peaks
		for(i=0; i<pca_g->vertical.peaks.num[0]; i++)
		{
			if(pca_g->vertical.peaks.low_val[i] < pca_g->vertical.thresholds[2])
			{
				st_ind = MAX(pca_g->vertical.peaks.low_ind[i]-2, 0);
				en_ind = MIN(pca_g->vertical.peaks.low_ind[i]+2, DATA_SIZE_MIS-1);

				for(j=0; j<pca_g->motion.peaks.num[1]; j++)
				{
					if(pca_g->motion.peaks.high_ind[j] >= st_ind && pca_g->motion.peaks.high_ind[j] <= en_ind && pca_g->motion.peaks.high_val[j] > pca_g->motion.thresholds[1])
						step_f++;
				}
				for(j=0; j<pca_g->motion.peaks.num[0]; j++)
				{
					if(pca_g->motion.peaks.low_ind[j] >= st_ind && pca_g->motion.peaks.low_ind[j] <= en_ind && pca_g->motion.peaks.low_val[j] < pca_g->motion.thresholds[0])
						step_b++;
				}
			}
		}
	}
	else
	{
		for(i=0; i<pca_g->vertical.peaks.num[0]; i++)
		{
			if(pca_g->vertical.peaks.low_val[i] < pca_g->vertical.thresholds[2])
			{
				st_ind = MAX(pca_g->vertical.peaks.low_ind[i]-3, 0);
				en_ind = MIN(pca_g->vertical.peaks.low_ind[i]+3, DATA_SIZE_MIS-1);

				for(j=0; j<pca_g->motion.peaks.num[1]; j++)
				{
					if(pca_g->motion.peaks.high_ind[j] >= st_ind && pca_g->motion.peaks.high_ind[j] <= pca_g->vertical.peaks.low_ind[i] && pca_g->motion.peaks.high_val[j] > 0)
					{
						for(k = 0; k < pca_g->motion.peaks.num[0]; k++)
						{
							if(pca_g->motion.peaks.low_ind[k] >= pca_g->vertical.peaks.low_ind[i] && pca_g->motion.peaks.low_ind[k] <= en_ind && pca_g->motion.peaks.low_val[k] < 0)
								step_b++;
						}
					}
				}
				for(j=0; j<pca_g->motion.peaks.num[0]; j++)
				{
					if(pca_g->motion.peaks.low_ind[j] >= st_ind && pca_g->motion.peaks.low_ind[j] <= pca_g->vertical.peaks.low_ind[i] && pca_g->motion.peaks.low_val[j] < 0)
					{
						for(k = 0; k < pca_g->motion.peaks.num[1]; k++)
						{
							if(pca_g->motion.peaks.high_ind[k] >= pca_g->vertical.peaks.low_ind[i] && pca_g->motion.peaks.high_ind[k] <= en_ind && pca_g->motion.peaks.high_val[k] > 0)
								step_f++;
						}
					}
				}
			}
		}//for(i=0; i<pca_g->vertical.peaks.num[1]; i++)
	}

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is phone and tablet general_phone_tablet_16_mis function to solve the
* 			180 amiguity problem in	case of phone or tablet general data.
*
* @details	This function takes 2 pointers of structures of type PCA_G and PCA. It uses the realtion
* 			between the vertical and motion peaks to provide a decision for Forward/Backward.
*
* @param[in]	pca_g	pointer to a structure of type \c PCA_G.
* @param[in]	pca		pointer to a structure of type \c PCA.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 general_phone_tablet_16_mis(PCA_G_Ptr pca_g, PCA_Ptr pca)
{
	INT8 i, j;
	INT8 st_ind = -1, en_ind = -1;
	INT8 count1 = 0, count2 = 0;
	INT8 id_h = -1, id_l = -1;
	INT8 step_f = 0, step_b = 0;
	INT8 min_l_dis = 0, min_h_dis = 0;
	FLOAT32 amp_l = 0.0, amp_h = 0.0;

	for(i=0; i<pca_g->vertical.peaks.num[1]; i++)
	{
		st_ind = MAX(pca_g->vertical.peaks.high_ind[i], 0);
		en_ind = MIN(pca_g->vertical.peaks.high_ind[i]+2, DATA_SIZE_MIS-1);

		/*for(j=0; j<pca_g->motion.count[1]; j++)
		{
		if(pca_g->motion.peaks.high_ind[pca_g->motion.index_m_h[j]] >= st_ind && pca_g->motion.peaks.high_ind[pca_g->motion.index_m_h[j]] <= en_ind)
		step_f++;
		}
		for(j=0; j<pca_g->motion.count[0]; j++)
		{
		if(pca_g->motion.peaks.low_ind[pca_g->motion.index_m_l[j]] >= st_ind && pca_g->motion.peaks.low_ind[pca_g->motion.index_m_l[j]] <= en_ind)
		step_b++;
		}*/

		for(j=0; j<pca_g->motion.peaks.num[1]; j++)
		{
			if(pca_g->motion.peaks.high_ind[j] >= st_ind && pca_g->motion.peaks.high_ind[j] <= en_ind && pca_g->motion.peaks.high_val[j] > pca_g->motion.thresholds[3])
				step_f++;
		}
		for(j=0; j<pca_g->motion.peaks.num[0]; j++)
		{
			if(pca_g->motion.peaks.low_ind[j] >= st_ind && pca_g->motion.peaks.low_ind[j] <= en_ind && pca_g->motion.peaks.low_val[j] < pca_g->motion.thresholds[2])
				step_b++;
		}

		st_ind = MAX(pca_g->vertical.peaks.high_ind[i]-4, 0);
		en_ind = MIN(pca_g->vertical.peaks.high_ind[i], DATA_SIZE_MIS-1);

		/*for(j=0; j<pca_g->motion.count[1]; j++)
		{
		if(pca_g->motion.peaks.high_ind[pca_g->motion.index_m_h[j]] >= st_ind && pca_g->motion.peaks.high_ind[pca_g->motion.index_m_h[j]] <= en_ind)
		step_b++;
		}
		for(j=0; j<pca_g->motion.count[0]; j++)
		{
		if(pca_g->motion.peaks.low_ind[pca_g->motion.index_m_l[j]] >= st_ind && pca_g->motion.peaks.low_ind[pca_g->motion.index_m_l[j]] <= en_ind)
		step_f++;
		}*/
		for(j=0; j<pca_g->motion.peaks.num[1]; j++)
		{
			if(pca_g->motion.peaks.high_ind[j] >= st_ind && pca_g->motion.peaks.high_ind[j] <= en_ind && pca_g->motion.peaks.high_val[j] > pca_g->motion.thresholds[3])
				step_b++;
		}
		for(j=0; j<pca_g->motion.peaks.num[0]; j++)
		{
			if(pca_g->motion.peaks.low_ind[j] >= st_ind && pca_g->motion.peaks.low_ind[j] <= en_ind && pca_g->motion.peaks.low_val[j] < pca_g->motion.thresholds[2])
				step_f++;
		}
	} //for(j=0; j<v_h; j++)
	//}

	// Special case for unstable ear dataset Method III, Hsiu-Wen Chang,
	// 2012.05.17
	for(i=0; i<pca_g->vertical.peaks.num[1]; i++)
	{
		count1 = 0;
		count2 = 0;
		min_l_dis = 99;
		min_h_dis = 99;

		for(j=0; j<pca_g->motion.old_peaks.num[1]; j++)
		{
			if(abs(pca_g->vertical.peaks.high_ind[i]-pca_g->motion.old_peaks.high_ind[j]) < min_h_dis)
			{
				min_h_dis = (INT8)(abs(pca_g->vertical.peaks.high_ind[i] - pca_g->motion.old_peaks.high_ind[j]));
				amp_h = pca_g->motion.old_peaks.high_val[j];
				id_h  = pca_g->motion.old_peaks.high_ind[j];
			}
		}
		for(j=0; j<pca_g->motion.old_peaks.num[1]; j++)
		{
			if(abs(pca_g->vertical.peaks.high_ind[i]-pca_g->motion.old_peaks.high_ind[j]) == min_h_dis)
				count1++;
		}
		for(j=0; j<pca_g->motion.old_peaks.num[0]; j++)
		{
			if(abs(pca_g->vertical.peaks.high_ind[i]-pca_g->motion.old_peaks.low_ind[j]) < min_l_dis)
			{
				min_l_dis = (INT8)(abs(pca_g->vertical.peaks.high_ind[i] - pca_g->motion.old_peaks.low_ind[j]));
				amp_l = pca_g->motion.old_peaks.low_val[j];
				id_l  = pca_g->motion.old_peaks.low_ind[j];
			}
		}
		for(j=0; j<pca_g->motion.old_peaks.num[0]; j++)
		{
			if(abs(pca_g->vertical.peaks.high_ind[i]-pca_g->motion.old_peaks.low_ind[j]) == min_l_dis)
				count2++;
		}
		if(min_h_dis == 0 || min_l_dis == 0)  // exactly coincide
		{
			if(min_h_dis == 0 && pca->data_m[id_h] > (pca->motion_parametrs[0]+1.6*pca->motion_parametrs[1]))
				step_f++;
			if(min_l_dis == 0 && pca->data_m[id_l] < (pca->motion_parametrs[0]-1.6*pca->motion_parametrs[1]))
				step_b++;
		}
		else
		{
			if(min_h_dis < 5 && min_l_dis < 5 && fabs(amp_h-amp_l) > 0.4)  // small negative slope
			{
				if(id_h < id_l)
				{
					if(id_h < pca_g->vertical.peaks.high_ind[i] && id_l > pca_g->vertical.peaks.high_ind[i])
					{
						if(count2 != 2)
							step_b++;
						else
							step_f++;
					}
					else //v_h not at the middle
					{
						if(count1 != 2 && count2 != 2)
						{
							if(id_h < pca_g->vertical.peaks.high_ind[i] && id_l < pca_g->vertical.peaks.high_ind[i])
								step_f++;    	// with positive slop around v_h
							else
								step_b++;    	// with negative slop around v_h
						}
						else
						{
							if(min_l_dis < min_h_dis && count1 == 2)
								step_b++;
							else
								step_f++;
						}
					}
				}
				else if(id_h > id_l)
				{
					if(id_l < pca_g->vertical.peaks.high_ind[i] && id_h > pca_g->vertical.peaks.high_ind[i])
					{
						if(count1 != 2)
							step_f++;
						else
							step_b++;
					}
					else
					{
						if(count1 != 2 && count2 != 2)
						{
							if(id_l < pca_g->vertical.peaks.high_ind[i] && id_h < pca_g->vertical.peaks.high_ind[i])
								step_b++;    	// with negative slop around v_h
							else
								step_f++;    	// with positive slop around v_h
						}
						else
						{
							if(min_l_dis < min_h_dis && count1 == 2)
								step_b++;
							else
								step_f++;
						}
					}
				}
			}   // small negative slope
			if(min_h_dis >= 5 && min_l_dis < 5)   // long negative slope
			{
				if(id_l > pca_g->vertical.peaks.high_ind[i])
				{
					st_ind = MAX(id_l-5, 0);
					en_ind = id_l;
				}
				else
				{
					st_ind = id_l;
					en_ind = MIN(id_l+5, DATA_SIZE_MIS-1);
				}

				if(pca->data_m[st_ind] > pca->data_m[en_ind])  // negative slope
					step_b++;
				else
					step_f++;
			}// long negative slope

			if(min_h_dis < 5 && min_l_dis >= 5)
			{
				if(id_h > pca_g->vertical.peaks.high_ind[i])
				{
					st_ind = MAX(id_h-5, 0);
					en_ind = id_h;
				}
				else
				{
					st_ind = id_h;
					en_ind = MIN(id_h+5, DATA_SIZE_MIS-1);
				}

				if(pca->data_m[st_ind] < pca->data_m[en_ind])  // Positive slope
					step_f++;
				else
					step_b++;
			}
		} // else of // if(min_h_dis == 0 || min_l_dis == 0)  // exactly coincide
	} // for(j=0; j<v_h; j++)
	//} //if(step_f == step_b && fabs(mean_p > 30.0*PI/180.0))
	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is phone and tablet general_phone_tablet_17_mis function to solve the
* 			180 amiguity problem in	case of phone or tablet general data.
*
* @details	This function takes 2 pointers of structures of type PCA_G and PCA. It deals with the
* 			Jacket pocket data. It uses the vertical peaks and motion data slope to provide a
* 			decision for Forward/Backward.
*
* @param[in]	pca_g	pointer to a structure of type \c PCA_G.
* @param[in]	pca		pointer to a structure of type \c PCA.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 general_phone_tablet_17_mis(PCA_G_Ptr pca_g, PCA_Ptr pca)
{
	INT8 i;
	INT8 slope_counter_f = 0, slope_counter_b = 0;
	INT8 st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;
	FLOAT32 slope_m = 0;

	for(i = 0; i < pca_g->vertical.peaks.num[0]; i++)
	{
		st_ind = MAX(pca_g->vertical.peaks.low_ind[i]-1, 0);
		en_ind = MIN(pca_g->vertical.peaks.low_ind[i]+1, DATA_SIZE_MIS-1);
		slope_m = pca->data_m[en_ind] - pca->data_m[st_ind];
		if(slope_m < 0)
			slope_counter_f++;
		else
			slope_counter_b++;
	}
	for(i = 0; i < pca_g->vertical.peaks.num[1]; i++)
	{
		st_ind = MAX(pca_g->vertical.peaks.high_ind[i]-1, 0);
		en_ind = MIN(pca_g->vertical.peaks.high_ind[i]+1, DATA_SIZE_MIS-1);
		slope_m = pca->data_m[en_ind] - pca->data_m[st_ind];
		if(slope_m >= 0)
			slope_counter_f++;
		else
			slope_counter_b++;
	}
	if(slope_counter_f == (pca_g->vertical.peaks.num[0] + pca_g->vertical.peaks.num[1]))
		step_f++;
	if(slope_counter_b == (pca_g->vertical.peaks.num[0] + pca_g->vertical.peaks.num[1]))
		step_b++;

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is phone and tablet general_phone_tablet_18_mis function to solve the
* 			180 amiguity problem in case of phone or tablet general data.
*
* @details	This function takes a pointer of structure of type PCA_G. It is a version
* 			of algorithm to deal with compass data that has good shape of motion signal.
* 			It uses the realtion between the vertical and motion peaks to provide a
* 			decision for Forward/Backward.
*
* @param[in]	pca_g	pointer to a structure of type \c PCA_G.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*
* @see peaks_min_distance_mis().
*/
INT8 general_phone_tablet_18_mis(PCA_G_Ptr pca_g)
{
	INT8 i;
	INT8 method_sl = 0, method_sh = 0, s_thr=0;
	INT8 step[2] = {0};
	INT8 ml_vl_first_dis = 0, mh_vl_first_dis = 0;
	UINT8 min_peaks_dis[4] = {0,0,0,0};

	i = peaks_min_distance_mis(pca_g, min_peaks_dis, 0, 0, step);

	if(min_peaks_dis[3] >= (pca_g->vertical.peaks.num[1]-1))
		step[1]++;
	if(min_peaks_dis[2] >= (pca_g->vertical.peaks.num[1]-1))
		step[0]++;

	if(min_peaks_dis[1] >= (pca_g->vertical.peaks.num[0]-1))
		step[1]++;
	if(min_peaks_dis[0] >= (pca_g->vertical.peaks.num[0]-1))
		step[0]++;

	//Simple method II: good shape in motion and vertical
	//Great motion signal with vertical signal series, then priori using
	//method simple method and finish. Hsiu-Wen, Chang, 2012.05.07
	if(step[0] == step[1] && abs(pca_g->motion.peaks.num[0]+pca_g->motion.peaks.num[1]-pca_g->vertical.peaks.num[0]-pca_g->vertical.peaks.num[1]) < 2)
	{
		method_sh = 0;
		ml_vl_first_dis = (INT8)(abs(pca_g->motion.peaks.low_ind[0] - pca_g->vertical.peaks.low_ind[0]));
		mh_vl_first_dis = (INT8)(abs(pca_g->motion.peaks.high_ind[0] - pca_g->vertical.peaks.low_ind[0]));
		if(ml_vl_first_dis < mh_vl_first_dis && ml_vl_first_dis < 4)
		{
			//lowe peak in v comes with low peak in motion and high peak in
			//v comes with high peak in motion
			if(pca_g->vertical.peaks.num[0] > 1)
				s_thr = (INT8)(ceil((FLOAT32)(pca_g->vertical.peaks.low_ind[1]-pca_g->vertical.peaks.low_ind[0]))*0.25F);
			else
				s_thr = 3;

			for(i = 0; i < pca_g->motion.peaks.num[0]; i++)
			{
				if(i <= pca_g->vertical.peaks.num[0] && abs(pca_g->motion.peaks.low_ind[i]-pca_g->vertical.peaks.low_ind[i]) < s_thr)
					method_sh++;
			}
			if(pca_g->vertical.peaks.num[1] > 1)
				s_thr = (INT8)(ceil((FLOAT32)(pca_g->vertical.peaks.high_ind[1]-pca_g->vertical.peaks.high_ind[0]))*0.25F);
			else
				s_thr = 3;
			for(i = 0; i < pca_g->motion.peaks.num[1]; i++)
			{
				if(i <= pca_g->vertical.peaks.num[1] && abs(pca_g->motion.peaks.high_ind[i]-pca_g->vertical.peaks.high_ind[i]) < s_thr)
					method_sh++;
			}
			if(method_sl >= (pca_g->vertical.peaks.num[1]+pca_g->vertical.peaks.num[0]-1))
				step[0]++;
		}
		else if(ml_vl_first_dis > mh_vl_first_dis && mh_vl_first_dis < 4)
		{
			//lowe peak in v comes with high peak in motion and high peak in
			//v comes with low peak in motion
			if(pca_g->vertical.peaks.num[0] > 1)
				s_thr =(INT8)(ceil((FLOAT32)(pca_g->vertical.peaks.low_ind[1]-pca_g->vertical.peaks.low_ind[0]))*0.25F);
			else
				s_thr = 3;

			for(i = 0; i < pca_g->motion.peaks.num[0]; i++)
			{
				if(i <= pca_g->vertical.peaks.num[1] && abs(pca_g->motion.peaks.low_ind[i]-pca_g->vertical.peaks.high_ind[i]) < s_thr)
					method_sl++;
			}
			if(pca_g->vertical.peaks.num[1] > 1)
				s_thr = (INT8)(ceil((FLOAT32)(pca_g->vertical.peaks.high_ind[1]-pca_g->vertical.peaks.high_ind[0]))*0.25F);
			else
				s_thr = 3;

			for(i = 0; i < pca_g->motion.peaks.num[1]; i++)
			{
				if(i <= pca_g->vertical.peaks.num[0] && abs(pca_g->motion.peaks.high_ind[i]-pca_g->vertical.peaks.low_ind[i]) < s_thr)
					method_sl++;
			}
			if(method_sl >= (pca_g->vertical.peaks.num[1]+pca_g->vertical.peaks.num[0]-1))
				step[1]++;
		}//if(a > c && c < 4)
	}//if(abs(m_l+pca_g->motion.peaks.num[1]-pca_g->vertical.peaks.num[0]-pca_g->vertical.peaks.num[1]) < 2 && step_f ==step_b)

	if(step[0] > step[1])
		return 0;
	else if(step[1] > step[0])
		return 1;
	else
		return 2;
}
/**
* @brief	This is phone and tablet general_phone_tablet_19_mis function to solve the 180
* 			amiguity problem in case of phone or tablet general data.
*
* @details	This function takes 2 pointers of structures of type PCA_G and PCA. It deals with the
* 			data of light dangling cases that misclassified as general. It uses the
* 			motion data slope at the maximum vertical low peak to provide a decision for
* 			Forward/Backward.
*
* @param[in]	pca_g	pointer to a structure of type \c PCA_G.
* @param[in]	pca		pointer to a structure of type \c PCA.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 general_phone_tablet_19_mis(PCA_G_Ptr pca_g, PCA_Ptr pca)
{
	INT8 i;
	INT8 st_ind=-1, en_ind=-1;
	INT8 max_vl_peaks_ind = -1;
	INT8 step_f = 0, step_b = 0;

	FLOAT32 slope_m=0;
	FLOAT32 max_vl_peaks=0;

	// to get the index of the maximum low peak
	max_vl_peaks_ind = -1;
	max_vl_peaks = -99.0;
	for(i=0; i<pca_g->vertical.peaks.num[0]; i++)
	{
		if(pca_g->vertical.peaks.low_val[i] > max_vl_peaks)
		{
			max_vl_peaks = pca_g->vertical.peaks.low_val[i];
			max_vl_peaks_ind = i;
		}
	}
	st_ind = MAX(pca_g->vertical.peaks.low_ind[max_vl_peaks_ind], 0);
	en_ind = MIN(pca_g->vertical.peaks.low_ind[max_vl_peaks_ind]+2, DATA_SIZE_MIS-1);
	slope_m = pca->data_m[en_ind] - pca->data_m[st_ind];
	if(slope_m >= 0)
		step_f++;
	else
		step_b++;

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is phone and tablet general_phone_tablet_20_mis function to
* 			solve the 180 amiguity problem in case of phone or tablet general data.
*
* @details	This function takes 2 pointers of structures of type PCA_G and PCA and method number.
* 			It deals with general data that has inverse logic than the normal one. It uses the
* 			motion data slope at the vertical peaks to provide a decision for Forward/Backward.
* 			Method number variable is used to switch bewteen the vertical high and low peaks.
*
* @param[in]	pca_g			pointer to a structure of type \c PCA_G.
* @param[in]	pca				pointer to a structure of type \c PCA.
* @param[in]	Method_Num		variable for method number.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 general_phone_tablet_20_mis(PCA_G_Ptr pca_g, PCA_Ptr pca, UINT8 Method_Num)
{
	INT8 i;
	INT8 st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;
	FLOAT32 slope_m = 0.0;
	UINT8 vh_flag = 0, vl_flag = 1;
	if(Method_Num == 1)
	{
		vh_flag = 1;
		vl_flag = 0;
	}
	for(i = 0; i < (vh_flag*pca_g->vertical.peaks.num[1] + vl_flag*pca_g->vertical.peaks.num[0]); i++)
	{
		if(vh_flag*(pca_g->vertical.peaks.high_val[i] > pca_g->vertical.thresholds[1]) ||
			vl_flag*((Method_Num == 2 && pca_g->vertical.peaks.low_val[i] < pca_g->vertical.thresholds[0]) ||
			(Method_Num == 3 && pca_g->vertical.peaks.low_val[i] < pca_g->vertical.thresholds[0] && pca_g->vertical.peaks.low_val[i] < pca_g->motion.thresholds[0]) ||
			(Method_Num == 4 && pca_g->vertical.peaks.low_val[i] > pca_g->vertical.thresholds[2])))
		{
			st_ind = MAX((pca_g->vertical.peaks.high_ind[i]*vh_flag + pca_g->vertical.peaks.low_ind[i]*vl_flag) - 1 - 1*(Method_Num == 3), 0);
			en_ind = MIN((pca_g->vertical.peaks.high_ind[i]*vh_flag + pca_g->vertical.peaks.low_ind[i]*vl_flag) + 1 + 1*(Method_Num == 3), DATA_SIZE_MIS-1);

			slope_m = pca->data_m[en_ind] - pca->data_m[st_ind];
			if(vh_flag*(slope_m >= 0) || vl_flag*(slope_m < 0))
				step_f++;
			else
				step_b++;
		}
	}
	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is phone and tablet general_phone_tablet_21_mis function to solve
* 			the 180 amiguity problem in	case of phone or tablet general data.
*
* @details	This function takes 2 pointers of structures of type PCA_G and PCA. It is a version
* 			of algorithm to deal with compass data. It uses the realtion between the vertical and
* 			motion peaks to provide a decision for Forward/Backward.
*
* @param[in]	pca_g	pointer to a structure of type \c PCA_G.
* @param[in]	pca		pointer to a structure of type \c PCA.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*
* @see peaks_min_distance_mis().
*/
INT8 general_phone_tablet_21_mis(PCA_G_Ptr pca_g, PCA_Ptr pca)
{
	INT8 i, j;
	INT8 k1=0, k2=0, k3=0, k4=0, k5=0;
	INT8 belt_check = 0, b = -1;
	INT8 step[2] = {0};
	UINT8 min_peaks_dis[4] = {0,0,0,0};
	FLOAT32 min_h_val = 0, max_h_val = 0;
	FLOAT32 a = 0, tem_tag = 0;
	FLOAT32 max_v = -99.0;

	for(i = 0; i < DATA_SIZE_MIS; i++)
	{
		if(pca->data_v[i] > max_v)
			max_v = pca->data_v[i];
	}
	belt_check = peaks_min_distance_mis(pca_g, min_peaks_dis, 2, 0, step);

	//Enhanced code 2012.05.12
	if(pca_g->vertical.peaks.num[1] > 2)
	{
		max_h_val = -99.0;
		min_h_val =  99.0;
		for(i =0; i < pca_g->vertical.peaks.num[1]; i++)
		{
			if(pca_g->vertical.peaks.high_val[i] < min_h_val)
				min_h_val = pca_g->vertical.peaks.high_val[i];
			if(pca_g->vertical.peaks.high_val[i] > max_h_val)
				max_h_val = pca_g->vertical.peaks.high_val[i];
		}

		if(belt_check > 0 && pca->motion_parametrs[2] > 3 && pca->motion_parametrs[1] > 1 && max_v > 4 && (pca_g->vertical.peaks.high_val[0] - pca_g->vertical.peaks.high_val[1])*
			(pca_g->vertical.peaks.high_val[1] - pca_g->vertical.peaks.high_val[2]) < 0 && fabs(max_h_val-min_h_val) > 1.4 && pca_g->motion.local_peaks.num[1] != 0 && pca_g->motion.local_peaks.num[0] != 0)
		{
			//change method to special belt data
			a = -99.0;
			b = -1;

			for(i = 0; i < pca_g->vertical.peaks.num[1]; i++)
			{
				if(pca_g->vertical.peaks.high_val[i] > a)
				{
					a = pca_g->vertical.peaks.high_val[i];
					b = i;
				}
			}
			if(pca_g->vertical.peaks.high_ind[b] > 25) //too close to the end, will have not enough motion peaks after
			{
				tem_tag = -99.0;
				for(i = 0; i < pca_g->vertical.peaks.num[1]; i++)
				{
					if(pca_g->vertical.peaks.high_val[i] > tem_tag && pca_g->vertical.peaks.high_val[i] != a)
					{
						tem_tag = pca_g->vertical.peaks.high_val[i];
						b = i;
					}
				}
				a = tem_tag;
			}

			k1 = -1;
			k2 = -1;
			// Comp_mc August 2013 Modified from for(j=0; j<m_l; j++)  to for(j=0; j<m_ll; j++)
			for(j=0; j<pca_g->motion.local_peaks.num[0]; j++) //k1 =find(pca_g->motion.ml_peaks_local(:,1) > pca_g->vertical.vh_peaks(b,1));
			{
				if(pca_g->motion.local_peaks.low_ind[j] > pca_g->vertical.peaks.high_ind[b])
				{ k1 = j;	break;	}
			}
			// Comp_mc August 2013 Modified from for(j=0; j<m_h; j++)  to for(j=0; j<m_hh; j++)
			for(j=0; j<pca_g->motion.local_peaks.num[1]; j++) //k2 =find(pca_g->motion.mh_peaks_local(:,1) > pca_g->vertical.vh_peaks(b,1));
			{
				if(pca_g->motion.local_peaks.high_ind[j] > pca_g->vertical.peaks.high_ind[b])
				{  	k2 = j;	break;	}
			}
			for(j = 0; j < pca_g->vertical.peaks.num[0]; j++)//k3 = find(pca_g->vertical.vl_peaks(:,1) > pca_g->vertical.vh_peaks(b,1));
			{
				if(pca_g->vertical.peaks.low_ind[j] > pca_g->vertical.peaks.high_ind[b])
				{ 	k3 = j;	break;	}
			}

			if(k1 != -1 && k2 != -1 && k3 != -1)
			{
				k4 = -1;
				k5 = -1;
				//k4 = find(pca_g->motion.mh_peaks_local(:,1) > pca_g->motion.ml_peaks_local(k1(1),1));
				// Comp_mc August 2013 Modified from for(j=0; j<m_h; j++)  to for(j=0; j<pca_g->motion.local_peaks.num[1]; j++)
				for(j=0; j<pca_g->motion.local_peaks.num[1]; j++)
				{
					if(pca_g->motion.local_peaks.high_ind[j] > pca_g->motion.local_peaks.low_ind[k1])
					{ 	k4 = j;	break;	}
				}
				//k5 = find(pca_g->motion.ml_peaks_local(:,1) > pca_g->motion.mh_peaks_local(k2(1),1));
				// Comp_mc August 2013 Modified from for(j=0; j<m_l; j++)  to for(j=0; j<m_ll; j++)
				for(j=0; j<pca_g->motion.local_peaks.num[0]; j++)
				{
					if(pca_g->motion.local_peaks.low_ind[j] > pca_g->motion.local_peaks.high_ind[k2])
					{	k5 = j; break;	}
				}

				if(k4 != -1)
				{
					if(pca_g->motion.local_peaks.low_ind[k1] <= pca_g->vertical.peaks.low_ind[k3] && abs(pca_g->motion.local_peaks.high_ind[k4]-pca_g->vertical.peaks.low_ind[k3]) < 6 && pca_g->motion.local_peaks.high_ind[k4] > pca_g->vertical.peaks.low_ind[k3])
					{
						step[1]++;
						if(b < (pca_g->vertical.peaks.num[1]-1) && pca_g->motion.local_peaks.high_ind[k4] > pca_g->vertical.peaks.high_ind[b+1])
							step[1]--;
					}
				}
				if(k5 != -1)
				{
					if(pca_g->motion.local_peaks.high_ind[k2] <= pca_g->vertical.peaks.low_ind[k3] && abs(pca_g->motion.local_peaks.low_ind[k5]-pca_g->vertical.peaks.low_ind[k3]) < 6 && pca_g->motion.local_peaks.low_ind[k5] > pca_g->vertical.peaks.low_ind[k3])
					{
						step[0]++;
						if(b < (pca_g->vertical.peaks.num[1]-1) && pca_g->motion.local_peaks.low_ind[k5] > pca_g->vertical.peaks.high_ind[b+1])
							step[0]--;
					}
				}
			}//if(k1 != -1 && k2 != -1 && k3 != -1
		}
		else
		{
			if((min_peaks_dis[1]+min_peaks_dis[2]) > (min_peaks_dis[0]+min_peaks_dis[3]))
				step[0]++;
			else if((min_peaks_dis[0]+min_peaks_dis[3]) > (min_peaks_dis[1]+min_peaks_dis[2]))
				step[1]++;
		}
	}
	else
	{
		if((min_peaks_dis[1]+min_peaks_dis[2]) > (min_peaks_dis[0]+min_peaks_dis[3]))
			step[0]++;
		else if((min_peaks_dis[0]+min_peaks_dis[3]) > (min_peaks_dis[1]+min_peaks_dis[2]))
			step[1]++;
	}
	if(step[0] > step[1])
		return 0;
	else if(step[1] > step[0])
		return 1;
	else
		return 2;
}
/**
* @brief	This is phone and tablet general_phone_tablet_22_mis function to solve
* 			the 180 amiguity problem in	case of phone or tablet general data.
*
* @details	This function takes 3 pointers of structures of type PCA_G, PCA, and PCA_M. It is a version
* 			of algorithm to deal with compass data. It uses the realtion between the vertical and
* 			motion peaks to provide a decision for Forward/Backward.
*
* @param[in]	pca_g	pointer to a structure of type \c PCA_G.
* @param[in]	pca		pointer to a structure of type \c PCA.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*
* @see peaks_min_distance_mis() and motion_peaks_segment_mis().
*/
INT8 general_phone_tablet_22_mis(PCA_G_Ptr pca_g, PCA_Ptr pca, PCA_M_Ptr pca_m)
{
	INT8 i;
	INT8 st_ind = -1, en_ind = -1;
	INT8 step[2] = {0};
	UINT8 min_peaks_dis[4] ={0, 0, 0, 0};

	if(pca->motion_parametrs[2] > 4.0 && pca->vertical_parametrs[2] > 7.0 && (pca->vertical_parametrs[2]/pca->motion_parametrs[2]) > 1.2F)
	{	// #14-1-2-1
		i = peaks_min_distance_mis(pca_g, min_peaks_dis, 1, 1, step);

		if((min_peaks_dis[1]+min_peaks_dis[2]) > (min_peaks_dis[0]+min_peaks_dis[3]))
			step[1]++;
		if((min_peaks_dis[0]+min_peaks_dis[3]) > (min_peaks_dis[1]+min_peaks_dis[2]))
			step[0]++;
	} // End of #14-1-2-1
	else if(pca->motion_parametrs[2] > 5 && pca->vertical_parametrs[2] > 5 && (pca->vertical_parametrs[2]/pca->motion_parametrs[2]) < 1.5F && pca->horizontal_vertical_flag != 0)
	{
		if(pca_m->lateral_range >= 2.5 && pca_m->lateral_range < 4)
		{ // #14-1-2-2
			for(i = 0; i < pca_g->vertical.peaks.num[0]; i++)
			{
				if(pca_g->vertical.peaks.low_val[i] < pca_g->vertical.thresholds[0])
				{
					st_ind = MAX(pca_g->vertical.peaks.low_ind[i]-1, 0);
					en_ind = MIN(pca_g->vertical.peaks.low_ind[i]+1, DATA_SIZE_MIS-1);
					motion_peaks_segment_mis(&pca_g->motion.peaks, pca_g->motion.thresholds, st_ind, en_ind, 2, 0, step);
				}
			}

			for(i = 0; i < pca_g->vertical.peaks.num[1]; i++)
			{
				if(pca_g->vertical.peaks.high_val[i] > pca_g->vertical.thresholds[1])
				{
					st_ind = MAX(pca_g->vertical.peaks.high_ind[i] - 2, 0);
					en_ind = MIN(pca_g->vertical.peaks.high_ind[i] + 2, DATA_SIZE_MIS-1);
					motion_peaks_segment_mis(&pca_g->motion.peaks, pca_g->motion.thresholds, st_ind, en_ind, 2, 1, step);
				}
			}
		} // End of #14-1-2-2
		else if(pca_m->lateral_range >= 4.0)
		{ // #14-1-2-3
			// big motion signal
			for(i = 0; i < pca_g->vertical.peaks.num[0]; i++)
			{
				st_ind = MAX(pca_g->vertical.peaks.low_ind[i]-1, 0);
				en_ind = MIN(pca_g->vertical.peaks.low_ind[i]+1, DATA_SIZE_MIS-1);
				motion_peaks_segment_mis(&pca_g->motion.peaks, pca_g->motion.thresholds, st_ind, en_ind, 2, 0, step);
			}
		} // End of #14-1-2-3
	}
	if(step[0] > step[1])
		return 0;
	else if(step[1] > step[0])
		return 1;
	else
		return 2;
}
/**
* @brief	This is phone and tablet general_phone_tablet_23_mis function to solve the
* 			180 amiguity problem in case of phone or tablet general data.
*
* @details	This function takes a pointer of structure of type PCA_G. It is an improved version
* 			of algorithm to deal with compass data that has good shape of motion signal.
* 			It uses the realtion between the vertical and motion peaks to provide a
* 			decision for Forward/Backward.
*
* @param[in]	pca_g	pointer to a structure of type \c PCA_G.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*
* @see peaks_min_distance_mis().
*/
INT8 general_phone_tablet_23_mis(PCA_G_Ptr pca_g)
{
	INT8 i;
	INT8 step[2] = {0};
	UINT8 min_peaks_dis[4] ={0,0,0,0};

	i = peaks_min_distance_mis(pca_g, min_peaks_dis, 1, 1, step);

	if((min_peaks_dis[1]+min_peaks_dis[2]) > (min_peaks_dis[0]+min_peaks_dis[3]))
		step[0]++;
	if((min_peaks_dis[0]+min_peaks_dis[3]) > (min_peaks_dis[1]+min_peaks_dis[2]))
		step[1]++;

	if (step[0] > step[1])
		return 0;
	else if (step[1] > step[0])
		return 1;
	else
		return 2;
}
/**
* @brief	This is watch general backward forward function to solve the 180 amiguity problem in
* 			case of watch general data.
*
* @details	This function takes 3 pointers of structures of type PCA, PCA_M, and PCA_D. It calls different
* 			sub-functions to get the decision of Forward/Backward in the case of watch general data.
*
* @param[in]	pca_g	pointer to a structure of type \c PCA_G.
* @param[in]	pca  	pointer to a structure of type \c PCA.
* @param[in]	pca_m  	pointer to a structure of type \c PCA_M.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 general_watch_backward_forward_mis(PCA_G_Ptr pca_g, PCA_Ptr pca, PCA_M_Ptr pca_m)
{
	INT8 i;
	INT8 temp_res = 0;
	FLOAT32 tem_mean = 0.0;
	FLOAT32 min_vh = 99, max_vh = -99;
	FLOAT32 max_diff_ml = -99.0, max_diff_mh = -99.0;

	if(pca->horizontal_vertical_flag == 0)
		tem_mean = floorf((FLOAT32)(fabs(pca->roll_parametrs[0]*R2Df)));
	else
		tem_mean = floorf((FLOAT32)(fabs(pca->roll_parametrs[0]*R2Df)+90.0));

	for(i=0; i< pca_g->vertical.peaks.num[1]; i++)
	{
		if(pca_g->vertical.peaks.high_val[i] < min_vh)
			min_vh = pca_g->vertical.peaks.high_val[i];

		if(pca_g->vertical.peaks.high_val[i] > max_vh)
			max_vh = pca_g->vertical.peaks.high_val[i];
	}

	// Method 1
#ifdef METHOD_RESULTS_MISALIGNMENT
	if(pca_m->vertical_motion_phase_shift > 100.0 && (pca_m->vertical_motion_phase_shift - pca_m->vertical_lateral_phase_shift) > 30.0)
	{
		//My_Method_Worked[151] = 1;
		//My_Method_Num = 151;
		return 1;
	}
	else if(pca_m->vertical_motion_phase_shift < 80.0 && (pca_m->vertical_lateral_phase_shift - pca_m->vertical_motion_phase_shift) > 30.0)
	{
		//My_Method_Worked[151] = 1;
		//My_Method_Num = 151;
		return 0;
	}
#else
	if(pca_m->vertical_motion_phase_shift > 100.0 && (pca_m->vertical_motion_phase_shift - pca_m->vertical_lateral_phase_shift) > 30.0)
		return 1;
	else if(pca_m->vertical_motion_phase_shift < 80.0 && (pca_m->vertical_lateral_phase_shift - pca_m->vertical_motion_phase_shift) > 30.0)
		return 0;
#endif
	// End of Method 1

#ifdef COM_WALK_SIMPLIFIED
	// Methods 2
	if(pca_m->height_change_flag == 0 && MAX(pca->roll_parametrs[2], pca->pitch_parametrs[2]) < 50)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[152] = 1;
		if(fabs(pca_m->vertical_motion_angle) > 30)
		{
			if(pca_m->vertical_motion_angle > 30)
			{
				//My_Method_Num = 152;
				return 0;
			}
			else if(pca_m->vertical_motion_angle<-30)
			{
				//My_Method_Num = 152;
				return 1;
			}
		}
#else
		if(fabs(pca_m->vertical_motion_angle) > 30)
		{
			if(pca_m->vertical_motion_angle > 30)
				return 0;
			else if(pca_m->vertical_motion_angle<-30)
				return 1;
		}
#endif
	}
	// End of sign method 2
# else
	// Methods 2, 3, 4, and 5
	if(pca_m->height_change_flag == 0)  // No chnage in height
	{
		// Method 2
		if(MAX(pca->roll_parametrs[2], pca->pitch_parametrs[2]) < 20.0F && fabs(pca_m->vertical_motion_angle) > 40)   // not pocket
		{
#ifdef METHOD_RESULTS_MISALIGNMENT
			//My_Method_Worked[152] = 1;
			if(pca_m->vertical_motion_angle > 40)
			{
				//My_Method_Num = 152;
				return 0;
			}
			else if(pca_m->vertical_motion_angle < -40)
			{
				//My_Method_Num = 152;
				return 1;
			}
#else
			if(pca_m->vertical_motion_angle > 40)
				return 0;
			else if(pca_m->vertical_motion_angle < -40)
				return 1;
#endif
		}
		// End of Method 2

		// Method 3
		if(MAX(pca->roll_parametrs[2], pca->pitch_parametrs[2]) > 40 || (MAX(pca->roll_parametrs[2], pca->pitch_parametrs[2]) > 7 && MAX(pca->roll_parametrs[2], pca->pitch_parametrs[2]) < 20))//Not in pocket
		{
#ifdef METHOD_RESULTS_MISALIGNMENT
			//My_Method_Worked[153] = 1;
			if(fabs(pca_m->vertical_motion_phase-90) > 10)
			{
				if(pca_m->motion_effective_coefficient>0)
				{
					//My_Method_Num = 153;
					return 0;
				}
				else
				{
					//My_Method_Num = 153;
					return 1;
				}
			}
			else if(fabs(pca_m->motion_effective_coefficient) > 0.3)
			{
				if(pca_m->motion_effective_coefficient > 0.3)
				{
					//My_Method_Num = 153;
					return 0;
				}
				else if(pca_m->motion_effective_coefficient < -0.3)
				{
					//My_Method_Num = 153;
					return 1;
				}
			}
#else
			if(fabs(pca_m->vertical_motion_phase-90) > 10)
			{
				if(pca_m->motion_effective_coefficient>0)
					return 0;
				else
					return 1;
			}
			else if(fabs(pca_m->motion_effective_coefficient) > 0.3)
			{
				if(pca_m->motion_effective_coefficient > 0.3)
					return 0;
				else if(pca_m->motion_effective_coefficient < -0.3)
					return 1;
			}
#endif
		}
		// End of Method 3

		// Method 4
		if(MAX(pca->roll_parametrs[2], pca->pitch_parametrs[2]) < 50 && fabs(pca_m->vertical_motion_angle) > 30)
		{
#ifdef METHOD_RESULTS_MISALIGNMENT
			//My_Method_Worked[154] = 1;
			if(pca_m->vertical_motion_angle > 30)
			{
				//My_Method_Num = 154;
				return 0;
			}
			else if(pca_m->vertical_motion_angle<-30)
			{
				//My_Method_Num = 154;
				return 1;
			}
#else
			if(pca_m->vertical_motion_angle > 30)
				return 0;
			else if(pca_m->vertical_motion_angle<-30)
				return 1;
#endif
		}
		// End of Method 4

		// Method 5
		if(MAX(pca->roll_parametrs[2], pca->pitch_parametrs[2]) > 24 && MAX(pca->roll_parametrs[2], pca->pitch_parametrs[2]) < 35) // Hand inside pocket
		{
#ifdef METHOD_RESULTS_MISALIGNMENT
			//My_Method_Worked[155] = 1;
			if(fabs(pca_m->vertical_motion_phase - 90.0F) > 15.0F)
			{
				if(pca_m->motion_effective_coefficient>0)
				{
					//My_Method_Num = 155;
					return 1;
				}
				else
				{
					//My_Method_Num = 155;
					return 0;
				}
			}
			else if(fabs(pca_m->motion_effective_coefficient)>0.2)
			{
				if(pca_m->motion_effective_coefficient>0.2)
				{
					//My_Method_Num = 155;
					return 1;
				}
				else if(pca_m->motion_effective_coefficient<-0.2)
				{
					//My_Method_Num = 155;
					return 0;
				}
			}
#else
			if(fabs(pca_m->vertical_motion_phase - 90.0F) > 15.0F)
			{
				if(pca_m->motion_effective_coefficient>0)
					return 1;
				else
					return 0;
			}
			else if(fabs(pca_m->motion_effective_coefficient)>0.2)
			{
				if(pca_m->motion_effective_coefficient>0.2)
					return 1;
				else if(pca_m->motion_effective_coefficient<-0.2)
					return 0;
			}
#endif
		}
		// End of Method 5
	}
# endif
	// End of Methods 2, 3, 4, and 5

	// Method 6
	// general_watch_method1_mis New 1 December 2013
	if(MAX(pca->roll_parametrs[2], pca->pitch_parametrs[2]) > 10.0)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[156] = 1;
		temp_res = general_watch_method1_mis(pca_g, pca, pca_m);
		if(temp_res != 2)
		{
			//My_Method_Num = 156;
			return temp_res;
		}
#else
		temp_res = general_watch_method1_mis(pca_g, pca, pca_m);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 6

	// Method 7			general_phone_tablet_4_mis()
	//Added on May 29th, 2012, New version to deal with Compass special when motion signal have big amplitude
	if((fabs(pca->roll_parametrs[0]*R2Df) <= 50.0 || fabs(pca->roll_parametrs[0]*R2Df) >= 130.0) && fabs(pca->pitch_parametrs[0]*R2Df) <= 50.0 && pca->horizontal_vertical_flag == 0)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[157] = 1;
		temp_res = general_watch_method2_mis(pca_g, pca);
		if(temp_res != 2)
		{
			//My_Method_Num = 157;
			return temp_res;
		}
#else
		temp_res = general_watch_method2_mis(pca_g, pca);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 7

	// Method 8           general_compass_simple_method_1()
	// Start with Simple Method I: strictly matching method, vh-mh, vl-ml within one sample
	// Revised condition on July 3 Condition is modified on Augus 2nd make it only for compass
	if((max_vh - min_vh) < 0.8 && !(tem_mean > 60.0 && tem_mean < 120.0) && pca->motion_parametrs[2] < pca->vertical_parametrs[2] && (pca->vertical_parametrs[2]/pca->motion_parametrs[2]) > 1.4)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[158] = 1;
		temp_res = general_watch_method3_mis(pca_g, pca);
		if(temp_res != 2)
		{
			//My_Method_Num = 158;
			return temp_res;
		}
#else
		temp_res = general_watch_method3_mis(pca_g, pca);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 8

	// Methods 9 and 10       Simple method I
	if(pca_g->motion.peaks.num[1] > 1 && pca_g->motion.peaks.num[0] > 1 && pca->motion_parametrs[2] < 5)
	{
		// Method 9			general_compass_simple_method_jacket()
		// Added August 2013 Motion signal is small compared to vertical signal
		if((pca->vertical_parametrs[2]/pca->motion_parametrs[2]) > 1.5 && pca_m->lateral_range > 2.0)
		{
			//temp_res = general_watch_method4_mis_1(pca_g, pca);
#ifdef METHOD_RESULTS_MISALIGNMENT
			//My_Method_Worked[159] = 1;
			temp_res = general_phone_tablet_17_mis(pca_g, pca);
			if(temp_res != 2)
			{
				//My_Method_Num = 159;
				return temp_res;
			}
#else
			temp_res = general_phone_tablet_17_mis(pca_g, pca);
			if(temp_res != 2)
				return temp_res;
#endif
		}
		// End of Method 9

		// Method 10			general_compass_simple_method_2()
		// Simple method I increase possibility then peaks are perfectly match(usually in compass mode) 2012.05.08
		//calculate difference summation for both high peaks and low peaks
		for(i = 0; i < pca_g->motion.peaks.num[0]-1; i++)
		{
			if(fabs(pca_g->motion.peaks.low_val[i]-pca_g->motion.peaks.low_val[i+1]) > max_diff_ml)
				max_diff_ml = fabs(pca_g->motion.peaks.low_val[i]-pca_g->motion.peaks.low_val[i+1]);
		}
		for(i = 0; i < pca_g->motion.peaks.num[1]-1; i++)
		{
			if(fabs(pca_g->motion.peaks.high_val[i]-pca_g->motion.peaks.high_val[i+1]) > max_diff_mh)
				max_diff_mh = fabs(pca_g->motion.peaks.high_val[i]-pca_g->motion.peaks.high_val[i+1]);
		}
		if(max_diff_ml < pca->motion_parametrs[2]*0.25F && max_diff_mh < pca->motion_parametrs[2]*0.25F)
		{
#ifdef METHOD_RESULTS_MISALIGNMENT
			//My_Method_Worked[160] = 1;
			temp_res = general_watch_method4_mis(pca_g);
			if(temp_res != 2)
			{
				//My_Method_Num = 160;
				return temp_res;
			}
#else
			temp_res = general_watch_method4_mis(pca_g);
			if(temp_res != 2)
				return temp_res;
#endif
		}
		// End of Method 10
	}
	// End of Methods 9 and 10

	// Method 11         General_Unstable_Handheld_Method_3()
	// Add December 2013: deal with unstable handheld
	if(MAX(pca->roll_parametrs[2], pca->pitch_parametrs[2]) < 7.0 && MAX(fabs(pca->roll_parametrs[0]*R2Df), fabs(pca->pitch_parametrs[0]*R2Df)) < 70.0 && pca_m->lateral_range > 1.5)
	{
#ifdef METHOD_RESULTS_MISALIGNMENT
		//My_Method_Worked[161] = 1;
		temp_res = general_phone_tablet_20_mis(pca_g, pca, 3);
		if(temp_res != 2)
		{
			//My_Method_Num = 161;
			return temp_res;
		}
#else
		temp_res = general_phone_tablet_20_mis(pca_g, pca, 3);
		if(temp_res != 2)
			return temp_res;
#endif
	}
	// End of Method 11

	// Method 12	general_compass_simple_method_3()
	// Add August 2013: deal with Jacket pockets Simple method I with special requirement
#ifdef METHOD_RESULTS_MISALIGNMENT
	//My_Method_Worked[162] = 1;
	temp_res = general_phone_tablet_21_mis(pca_g, pca);
	if(temp_res != 2)
	{
		//My_Method_Num = 162;
		return temp_res;
	}
#else
	temp_res = general_phone_tablet_21_mis(pca_g, pca);
	if(temp_res != 2)
		return temp_res;
#endif
	// End of Method 12

	// Method13
#ifdef METHOD_RESULTS_MISALIGNMENT
	//My_Method_Worked[163] = 1;
	temp_res = general_watch_method5_mis(pca_g);
	if(temp_res != 2)
	{
		//My_Method_Num = 163;
		return temp_res;
	}
#else
	temp_res = general_watch_method5_mis(pca_g);
	if(temp_res != 2)
		return temp_res;
#endif
	// End of Mthod 13
#ifdef METHOD_RESULTS_MISALIGNMENT
	//My_Method_Worked[150] = 1;
	//My_Method_Num = 150;
#endif
	return 2;
}
/**
* @brief	This is Watch general method 1 to solve the 180 amiguity problem in case
* 			of Watch general data.
*
* @details	This function takes 3 pointers of structures of type PCA_G, PCA, and PCA_M.
* 			It uses the relation between the vertical and motion peaks
* 			to provide the decision of Forward/Backward.
*
* @param[in]	pca_g	pointer to a structure of type \c PCA_G.
* @param[in]	pca		pointer to a structure of type \c PCA.
* @param[in]	pca_m		pointer to a structure of type \c PCA_M.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 general_watch_method1_mis(PCA_G_Ptr pca_g, PCA_Ptr pca, PCA_M_Ptr pca_m)
{
	INT8 i, j;
	INT8 st_ind = -1, en_ind = -1;
	INT8 step_f = 0, step_b = 0;

	if((pca_m->vertical_motion_phase_shift - pca_m->vertical_lateral_phase_shift) > 10.0)
		step_b++;
	else if((pca_m->vertical_lateral_phase_shift - pca_m->vertical_motion_phase_shift) > 10.0)
		step_f++;

	for(i=0; i<pca_g->vertical.peaks.num[1]; i++)
	{
		st_ind = MAX(pca_g->vertical.peaks.high_ind[i], 0);
		en_ind = MIN(pca_g->vertical.peaks.high_ind[i] + 2, DATA_SIZE_MIS-1);

		for(j=0; j<pca_g->motion.peaks.num[1]; j++)
		{
			if(pca_g->motion.peaks.high_ind[j] >= st_ind && pca_g->motion.peaks.high_ind[j] <= en_ind && pca_g->motion.peaks.high_val[j] > (pca->motion_parametrs[0] + 1.6*pca->motion_parametrs[1]))
				step_f++;
		}
		for(j=0; j<pca_g->motion.peaks.num[0]; j++)
		{
			if(pca_g->motion.peaks.low_ind[j] >= st_ind && pca_g->motion.peaks.low_ind[j] <= en_ind && pca_g->motion.peaks.low_val[j] < (pca->motion_parametrs[0] - 1.6*pca->motion_parametrs[1]))
				step_b++;
		}
		st_ind = MAX(pca_g->vertical.peaks.high_ind[i] - 4, 0);
		en_ind = MIN(pca_g->vertical.peaks.high_ind[i], DATA_SIZE_MIS-1);

		for(j=0; j<pca_g->motion.peaks.num[1]; j++)
		{
			if(pca_g->motion.peaks.high_ind[j] >= st_ind && pca_g->motion.peaks.high_ind[j] <= en_ind && pca_g->motion.peaks.high_val[j] > (pca->motion_parametrs[0] + 1.6*pca->motion_parametrs[1]))
				step_b++;
		}
		for(j=0; j<pca_g->motion.peaks.num[0]; j++)
		{
			if(pca_g->motion.peaks.low_ind[j] >= st_ind && pca_g->motion.peaks.low_ind[j] <= en_ind && pca_g->motion.peaks.low_val[j] < (pca->motion_parametrs[0] - 1.6*pca->motion_parametrs[1]))
				step_f++;
		}
	}

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is Watch general method 2 to solve the 180 amiguity problem in case
* 			of Watch general data.
*
* @details	This function takes 2 pointers of structures of type PCA_G and PCA.
* 			It uses the relation between the vertical and motion peaks
* 			to provide the decision of Forward/Backward.
*
* @param[in]	pca_g	pointer to a structure of type \c PCA_G.
* @param[in]	pca		pointer to a structure of type \c PCA.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*
* @see motion_peaks_segment_mis().
*/
INT8 general_watch_method2_mis(PCA_G_Ptr pca_g, PCA_Ptr pca)
{
	INT8 i, j;
	INT8 k1 = 0, b = -1;
	INT8 st_ind = -1, en_ind = -1;
	INT8 id_h = -1, id_l = -1;
	INT8 step[2] = {0};
	INT8 min_l_dis = 0, min_h_dis = 0;
	FLOAT32 a = 0.0;
	UINT8 min_peaks_dis[4] = {0,0,0,0};

	for(i = 0; i < pca_g->vertical.peaks.num[1]; i++)
	{
		k1 = -1;
		for(j = 0; j < pca_g->vertical.peaks.num[0]; j++)
		{
			if(pca_g->vertical.peaks.low_ind[j] > pca_g->vertical.peaks.high_ind[i])
			{	k1 = j;	break;	}
		}
		if(k1 != -1 && (pca_g->vertical.peaks.low_ind[k1]-pca_g->vertical.peaks.high_ind[i]) > min_h_dis)
			min_h_dis = pca_g->vertical.peaks.low_ind[k1]-pca_g->vertical.peaks.high_ind[i];
	}

	for(i = 0; i < pca_g->vertical.peaks.num[0]; i++)
	{
		k1 = -1;
		for(j = 0; j < pca_g->vertical.peaks.num[1]; j++)
		{
			if(pca_g->vertical.peaks.high_ind[j] > pca_g->vertical.peaks.low_ind[i])
			{	k1 = j;	break;	}
		}
		if(k1 != -1 && (pca_g->vertical.peaks.high_ind[k1] - pca_g->vertical.peaks.low_ind[i]) > min_l_dis)
			min_l_dis = pca_g->vertical.peaks.high_ind[k1] - pca_g->vertical.peaks.low_ind[i];
	}

	if(pca->vertical_parametrs[2] <= 3.5)
	{
		// check  for  the vertical high peaks
		for(i=0; i<pca_g->vertical.peaks.num[1]; i++)
		{
			if(pca_g->vertical.peaks.high_val[i] > pca_g->vertical.thresholds[1])
			{
				st_ind = MAX(pca_g->vertical.peaks.high_ind[i]-3, 0);
				en_ind = MIN(pca_g->vertical.peaks.high_ind[i]+2, DATA_SIZE_MIS-1);
				motion_peaks_segment_mis(&pca_g->motion.peaks, pca_g->motion.thresholds, st_ind, en_ind, 2, 1, step);
			}
		}
		// check  for  the vertical low peaks
		for(i=0; i<pca_g->vertical.peaks.num[0]; i++)
		{
			if(pca_g->vertical.peaks.low_val[i] < pca_g->vertical.thresholds[0])
			{
				st_ind = MAX(pca_g->vertical.peaks.low_ind[i]-3, 0);
				en_ind = MIN(pca_g->vertical.peaks.low_ind[i]+2, DATA_SIZE_MIS-1);
				motion_peaks_segment_mis(&pca_g->motion.peaks, pca_g->motion.thresholds, st_ind, en_ind, 2, 0, step);
			}
		}
	}
	else if(pca->vertical_parametrs[2] > 3.5 && (pca_g->vertical.peaks.num[1]+pca_g->vertical.peaks.num[0]) < 6 && pca->vertical_parametrs[2] < 6.0 && abs(min_h_dis-min_l_dis) > 3)
	{
		i = peaks_min_distance_mis(pca_g, min_peaks_dis, 6, 0, step);
	} // if((max_v-min_v) <= 3.5)

	//added on July 24. Especailly for solving problem between Husain dataset and girl dataset in compass.
	if(step[0] == step[1] && !(pca->vertical_parametrs[2] > 5.5 && pca->motion_parametrs[2] < 2.5) && pca->vertical_parametrs[2] < 8.0 && pca->motion_parametrs[2] > 2.9 && (pca->vertical_parametrs[2]/pca->motion_parametrs[2]) < 1.7)
	{
		a = 99.0;
		if(fabs(pca_g->misa)*R2Df < 90)
		{
			for(i = 0; i < pca_g->motion.peaks.num[0]; i++)
			{
				if(pca_g->motion.peaks.low_val[i] < a)
				{
					a = pca_g->motion.peaks.low_val[i];
					b = i;
				}
			}

			min_l_dis = 99;
			min_h_dis = 99;
			for(i = 0; i < pca_g->vertical.peaks.num[0]; i++)
			{
				if(abs(pca_g->vertical.peaks.low_ind[i] - pca_g->motion.peaks.low_ind[b]) < min_l_dis)
				{
					min_l_dis = (INT8)(abs(pca_g->vertical.peaks.low_ind[i] - pca_g->motion.peaks.low_ind[b]));
					id_l = pca_g->vertical.peaks.low_ind[i];
				}
			}

			for(i = 0; i < pca_g->vertical.peaks.num[1]; i++)
			{
				if(abs(pca_g->vertical.peaks.high_ind[i] - pca_g->motion.peaks.low_ind[b]) < min_h_dis)
				{
					min_h_dis = (INT8)(abs(pca_g->vertical.peaks.high_ind[i] - pca_g->motion.peaks.low_ind[b]));
					id_h = pca_g->vertical.peaks.high_ind[i];
				}
			}

			if(id_h >= pca_g->motion.peaks.low_ind[b] && id_l <= pca_g->motion.peaks.low_ind[b]) // locate on positive slope
				step[0]++;
			else
			{
				if(id_h <= pca_g->motion.peaks.low_ind[b] && id_l >= pca_g->motion.peaks.low_ind[b]) // locate on negative slope
					step[1]++;
				else
				{
					if((id_h > id_l && id_h < pca_g->motion.peaks.low_ind[b]) || (id_l < id_h && id_l > pca_g->motion.peaks.low_ind[b]))
						step[1]++;
					if((id_l > id_h && id_l < pca_g->motion.peaks.low_ind[b]) || (id_h < id_l && id_h > pca_g->motion.peaks.low_ind[b]))
						step[0]++;
				}
			}//if(id_h >= pca_g->motion.peaks.low_ind[b] && id_l <= pca_g->motion.peaks.low_ind[b])
		}
		else
		{
			a = -99.0;
			for(i = 0; i < pca_g->motion.peaks.num[1]; i++)
			{
				if(pca_g->motion.peaks.high_val[i] > a)
				{
					a = pca_g->motion.peaks.high_val[i];
					b = i;
				}
			}

			min_l_dis = 99;
			min_h_dis = 99;
			for(i = 0; i < pca_g->vertical.peaks.num[0]; i++)
			{
				if(abs(pca_g->vertical.peaks.low_ind[i] - pca_g->motion.peaks.high_ind[b]) < min_l_dis)
				{
					min_l_dis = (INT8)(abs(pca_g->vertical.peaks.low_ind[i] - pca_g->motion.peaks.high_ind[b]));
					id_l = pca_g->vertical.peaks.low_ind[i];
				}
			}

			for(i = 0; i < pca_g->vertical.peaks.num[1]; i++)
			{
				if(abs(pca_g->vertical.peaks.high_ind[i] - pca_g->motion.peaks.high_ind[b]) < min_h_dis)
				{
					min_h_dis = (INT8)(abs(pca_g->vertical.peaks.high_ind[i] - pca_g->motion.peaks.high_ind[b]));
					id_h = pca_g->vertical.peaks.high_ind[i];
				}
			}

			if(id_h >= pca_g->motion.peaks.high_ind[b] && id_l <= pca_g->motion.peaks.high_ind[b]) // locate on positive slope
				step[1]++;
			else
			{
				if(id_h <= pca_g->motion.peaks.high_ind[b] && id_l >= pca_g->motion.peaks.high_ind[b]) // locate on negative slope
					step[0]++;
				else
				{
					if((id_h > id_l && id_h < pca_g->motion.peaks.high_ind[b]) || (id_l < id_h && id_l > pca_g->motion.peaks.high_ind[b]))
						step[0]++;
					if((id_l > id_h && id_l < pca_g->motion.peaks.high_ind[b]) || (id_h < id_l && id_h > pca_g->motion.peaks.high_ind[b]))
						step[1]++;
				}
			}//if(id_h >= pca_g->motion.peaks.low_ind[b] && id_l <= pca_g->motion.peaks.low_ind[b])
		}
	}//if(step_f == step_b && (!(range_v > 5.5 && range_m < 2.5)))

	if(step[0] > step[1])
		return 0;
	else if(step[1] > step[0])
		return 1;
	else
		return 2;
}
/**
* @brief	This is Watch general method 3 to solve the 180 amiguity problem in case
* 			of Watch general data.
*
* @details	This function takes 2 pointers of structures of type PCA_G and PCA.
* 			It uses the vertical peaks and the motion data slope to provide
* 			the decision of Forward/Backward.
*
* @param[in]	pca_g	pointer to a structure of type \c PCA_G.
* @param[in]	pca		pointer to a structure of type \c PCA.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*
* @see slope_segment_mis().
*/
INT8 general_watch_method3_mis(PCA_G_Ptr pca_g, PCA_Ptr pca)
{
	INT8 i;
	INT8 step[2] = {0};
	UINT8 min_peaks_dis[4] = {0,0,0,0};

	i = peaks_min_distance_mis(pca_g, min_peaks_dis, 7, 0, step);

	// 14_1_new     December 2013   Unstable Handheld
	if(MAX(pca->roll_parametrs[2], pca->pitch_parametrs[2]) < 7.0 && MAX(fabs(pca->roll_parametrs[0]), fabs(pca->pitch_parametrs[0]))*R2Df < 70.0)
	{
		//check for the vertical High peaks
		slope_segment_mis(pca_g->vertical.peaks.num[1], pca_g->vertical.peaks.high_ind, pca_g->vertical.peaks.high_val, pca->data_m, 1, 1,0, pca_g->vertical.thresholds[1], step, 1);

		//check for the vertical low peaks
		slope_segment_mis(pca_g->vertical.peaks.num[0], pca_g->vertical.peaks.low_ind, pca_g->vertical.peaks.low_val, pca->data_m, 1, 1,0, pca_g->vertical.thresholds[0], step, 0);
	}
	// End of 14_1_new     December 2013

	if(step[0] > step[1])
		return 0;
	else if(step[1] > step[0])
		return 1;
	else
		return 2;
}
/**
* @brief	This is Watch general method 4 to solve the 180 amiguity problem in case
* 			of Watch general data.
*
* @details	This function takes a pointer of structure of type PCA_G. It uses the
* 			relation between the vertical and motion peaks to provide
* 			the decision of Forward/Backward.
*
* @param[in]	pca_g	pointer to a structure of type \c PCA_G.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 general_watch_method4_mis(PCA_G_Ptr pca_g)
{
	INT8 i;
	INT8 method_sl = 0, s_thr=0;
	INT8 step[2] = {0};
	INT8 a = 0, c = 0;
	UINT8 min_peaks_dis[4] = {0,0,0,0};

	i = peaks_min_distance_mis(pca_g, min_peaks_dis, 7, 0, step);

	//Simple method II: good shape in motion and vertical
	//Great motion signal with vertical signal series, then priori using method simple method and finish 2012.05.07
	if(step[0] == step[1] && abs((pca_g->motion.peaks.num[0]+pca_g->motion.peaks.num[1])-(pca_g->vertical.peaks.num[0]+pca_g->vertical.peaks.num[1])) < 2)
	{
		method_sl = 0;
		a = (INT8)(abs(pca_g->motion.peaks.low_ind[0] - pca_g->vertical.peaks.low_ind[0]));
		c = (INT8)(abs(pca_g->motion.peaks.high_ind[0] - pca_g->vertical.peaks.low_ind[0]));
		if(a < c && a < 4)
		{
			//lowe peak in v comes with low peak in motion and high peak in v comes with high peak in motion
			if(pca_g->vertical.peaks.num[0] > 1)
				s_thr = (INT8)(ceil((FLOAT32)(pca_g->vertical.peaks.low_ind[1]-pca_g->vertical.peaks.low_ind[0]))*0.25F);
			else
				s_thr = 3;

			for(i = 0; i < pca_g->motion.peaks.num[0]; i++)
			{
				if(i <= pca_g->vertical.peaks.num[0] && abs(pca_g->motion.peaks.low_ind[i]-pca_g->vertical.peaks.low_ind[i]) < s_thr)
					method_sl++;
			}
			if(pca_g->vertical.peaks.num[1] > 1)
				s_thr = (INT8)(ceil((FLOAT32)(pca_g->vertical.peaks.high_ind[1]-pca_g->vertical.peaks.high_ind[0]))*0.25F);
			else
				s_thr = 3;

			for(i = 0; i < pca_g->motion.peaks.num[1]; i++)
			{
				if(i <= pca_g->vertical.peaks.num[1] && abs(pca_g->motion.peaks.high_ind[i]-pca_g->vertical.peaks.high_ind[i]) < s_thr)
					method_sl++;
			}
			if(method_sl >= (pca_g->vertical.peaks.num[1]+pca_g->vertical.peaks.num[0]-1))
				step[0]++;
		}
		else if(a > c && c < 4)
		{
			//lowe peak in v comes with high peak in motion and high peak in v comes with low peak in motion
			if(pca_g->vertical.peaks.num[0] > 1)
				s_thr = (INT8)(ceil((FLOAT32)(pca_g->vertical.peaks.low_ind[1]-pca_g->vertical.peaks.low_ind[0]))*0.25F);
			else
				s_thr = 3;

			for(i = 0; i < pca_g->motion.peaks.num[0]; i++)
			{
				if(i <= pca_g->vertical.peaks.num[1] && abs(pca_g->motion.peaks.low_ind[i]-pca_g->vertical.peaks.high_ind[i]) < s_thr)
					method_sl++;
			}
			if(pca_g->vertical.peaks.num[1] > 1)
				s_thr = (INT8)(ceil((FLOAT32)(pca_g->vertical.peaks.high_ind[1]-pca_g->vertical.peaks.high_ind[0]))*0.25F);
			else
				s_thr = 3;

			for(i = 0; i < pca_g->motion.peaks.num[1]; i++)
			{
				if(i <= pca_g->vertical.peaks.num[0] && abs(pca_g->motion.peaks.high_ind[i]-pca_g->vertical.peaks.low_ind[i]) < s_thr)
					method_sl++;
			}
			if(method_sl >= (pca_g->vertical.peaks.num[1]+pca_g->vertical.peaks.num[0]-1))
				step[1]++;
		}//if(a > c && c < 4)
	}//if(abs(m_l+pca_g->motion.peaks.num[1]-pca_g->vertical.peaks.num[0]-pca_g->vertical.peaks.num[1]) < 2 && step_f ==step_b)

	if(step[0] > step[1])
		return 0;
	else if(step[1] > step[0])
		return 1;
	else
		return 2;
}
/**
* @brief	This is Watch general method 5 to solve the 180 amiguity problem in case
* 			of Watch general data.
*
* @details	This function takes a pointer of structure of type PCA_G. It uses the
* 			relation between the vertical and motion peaks to provide
* 			the decision of Forward/Backward.
*
* @param[in]	pca_g		pointer to a structure of type \c PCA_G.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 general_watch_method5_mis(PCA_G_Ptr pca_g)
{
	INT8 i, j;
	INT8 min_vh_ind = -1, max_vl_ind= -1, st_ind = -1, en_ind= -1;
	INT8 step_f = 0, step_b = 0;
	FLOAT32 a = -99.0, min_vh_val = 99,  max_vl_val = -99;

	a = 99.0;
	for(i = 0; i < pca_g->vertical.peaks.num[1]; i++)
	{
		if(pca_g->vertical.peaks.high_val[i] < a)
		{
			a = pca_g->vertical.peaks.high_val[i];
			min_vh_ind = i;
		}
	}
	if(pca_g->vertical.peaks.high_ind[min_vh_ind] > 26) //too close to the end, will have not enough motion peaks after
	{
		min_vh_ind = -1;
		for(i = 0; i < pca_g->vertical.peaks.num[1]; i++)
		{
			if(pca_g->vertical.peaks.high_val[i] < min_vh_val && pca_g->vertical.peaks.high_val[i] != a)
			{
				min_vh_val = pca_g->vertical.peaks.high_val[i];
				min_vh_ind = i;
			}
		}
	}

	if(min_vh_ind >= 0)
	{
		st_ind = MAX(pca_g->vertical.peaks.high_ind[min_vh_ind], 0);
		en_ind = MIN(pca_g->vertical.peaks.high_ind[min_vh_ind] + 2, DATA_SIZE_MIS-1);

		for(j=0; j<pca_g->motion.peaks.num[1]; j++)
		{
			if(pca_g->motion.peaks.high_ind[j] >= st_ind && pca_g->motion.peaks.high_ind[j] <= en_ind)
				step_f++;
		}
		for(j=0; j<pca_g->motion.peaks.num[0]; j++)
		{
			if(pca_g->motion.peaks.low_ind[j] >= st_ind && pca_g->motion.peaks.low_ind[j] <= en_ind)
				step_b++;
		}
	}
	if(step_f == step_b)
	{
		for(i = 0; i < pca_g->vertical.peaks.num[0]; i++)
		{
			if(pca_g->vertical.peaks.low_val[i] > max_vl_val)
			{
				max_vl_val = pca_g->vertical.peaks.low_val[i];
				max_vl_ind = i;
			}
		}

		st_ind = MAX(pca_g->vertical.peaks.low_ind[max_vl_ind], 0);
		en_ind = MIN(pca_g->vertical.peaks.low_ind[max_vl_ind] + 1, DATA_SIZE_MIS-1);

		for(j=0; j<pca_g->motion.peaks.num[1]; j++)
		{
			if(pca_g->motion.peaks.high_ind[j] >= st_ind && pca_g->motion.peaks.high_ind[j] <= en_ind)
				step_b++;
		}
		for(j=0; j<pca_g->motion.peaks.num[0]; j++)
		{
			if(pca_g->motion.peaks.low_ind[j] >= st_ind && pca_g->motion.peaks.low_ind[j] <= en_ind)
				step_f++;
		}
	}

	if(step_f > step_b)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}
/**
* @brief	This is Watch general method 7 to solve the 180 amiguity problem in case
* 			of Watch general data.
*
* @details	This function takes a pointer of structure of type PCA_G. It uses the
* 			relation between the vertical and motion peaks to provide
* 			the decision of Forward/Backward.
*
* @param[in]	pca		pointer to a structure of type \c PCA.
*
* @return	returns 0 in case of Forward, 1 in case of Backward, and 2 if no decision is made.
*/
INT8 xcorr_backward_forward_mis(PCA_Ptr pca)
{
	INT8 i, j;
	INT8 count1 = 0, count2 = 0, count3 = 0,count4 = 0;
	INT8 n_cycle = 0, stride_size = 0, m = 0;
	INT8 no_me_2 = 0;
	INT8 step_b = 0, step_f = 0;
	INT16 phase_lag = 0;
	INT16 lags[2*DATA_SIZE_MIS-1] = {0};
	//INT8 v_peaks_num[2] ={-1}, acc_peaks_num[2] ={-1}, m_peaks_num[2] ={-1};

	FLOAT32 avg_cycle = 0, phase_shift, phase_shift1, forward_score;
	FLOAT32 temp = 99.0;

	FLOAT32 x_drift[DATA_SIZE_MIS]   = {0}, z_drift[DATA_SIZE_MIS]   = {0};
	FLOAT32 x_drift_1[DATA_SIZE_MIS] = {0}, z_drift_1[DATA_SIZE_MIS] = {0};
	FLOAT32 m_drift[DATA_SIZE_MIS] = {0};
	FLOAT32 r[2*DATA_SIZE_MIS-1] = {0}, d_phase[DATA_SIZE_MIS] = {0};
	FLOAT32 zero_cro_fm[PEAKS_VECTOR_SIZE_MIS] = {0}, zero_cro_rm[PEAKS_VECTOR_SIZE_MIS] = {0};
	FLOAT32 zero_cro_fv[PEAKS_VECTOR_SIZE_MIS] = {0}, zero_cro_rv[PEAKS_VECTOR_SIZE_MIS] = {0};
	FLOAT32 max_min[2] = {0}, min_diff[PEAKS_VECTOR_SIZE_MIS] = {0};

	PEAKS vertical_peaks;
	PEAKS motion_peaks;
	PEAKS acceleration_norm_peaks;

	memset(&motion_peaks, 0, sizeof(PEAKS));
	memset(&vertical_peaks, 0, sizeof(PEAKS));
	memset(&acceleration_norm_peaks, 0, sizeof(PEAKS));


	// Vertica Peaks Detection
	i = peaks_detection_mis(pca->data_v, pca->vertical_parametrs[2]*0.25F, 4, &vertical_peaks, 20, 36, 0);

	// Motion Peaks Detection
	i = peaks_detection_mis(pca->data_m, pca->motion_parametrs[2]*0.25F, 4, &motion_peaks, 20, 36, 0);

	// Average cycle needed by Xcorr_phase method
	i = peaks_detection_mis(pca->Acc_Mag, pca->acceleration_norm_parametrs[2]*0.25F, 4, &acceleration_norm_peaks, 20, 36, 0);

	if(vertical_peaks.num[1] > 1 && vertical_peaks.num[0] > 1 && acceleration_norm_peaks.num[1] > 1 && acceleration_norm_peaks.num[0] > 1)
	{
		for(i = 0; i < acceleration_norm_peaks.num[1]-1; i++)
		{
			min_diff[i] = (FLOAT32)(acceleration_norm_peaks.high_ind[i+1]-acceleration_norm_peaks.high_ind[i]);
			avg_cycle += min_diff[i];
		}
		for(i = 0; i < acceleration_norm_peaks.num[0]-1; i++)
		{
			if((i+acceleration_norm_peaks.num[1]-1) < 20)
			{
				min_diff[i+acceleration_norm_peaks.num[1]-1] = (FLOAT32)(acceleration_norm_peaks.low_ind[i+1]-acceleration_norm_peaks.low_ind[i]);
				avg_cycle += min_diff[i+acceleration_norm_peaks.num[1]-1];
			}
		}

		find_max_min_float_mis(min_diff, max_min, acceleration_norm_peaks.num[1]+acceleration_norm_peaks.num[0]-2);
		if((acceleration_norm_peaks.num[1]+acceleration_norm_peaks.num[0]-2) > 3 && (max_min[0]-max_min[1]) > 4)
		{
			//delete most unreasonable cycles estimation
			avg_cycle  = avg_cycle-max_min[1]-max_min[0];
			avg_cycle /= (acceleration_norm_peaks.num[1]+acceleration_norm_peaks.num[0]-4);
		}
		else
		{
			avg_cycle /= (acceleration_norm_peaks.num[1]+acceleration_norm_peaks.num[0]-2);
		}

		//cross validate MAG signal with vertical signal
		//do not use Xcorr_2 when MAG and Vertical signal are obvious different
		if(abs((acceleration_norm_peaks.num[1]+acceleration_norm_peaks.num[0]) - (vertical_peaks.num[1]+vertical_peaks.num[0])) > 0)
		{
			if(motion_peaks.num[1] > 1 && motion_peaks.num[0] > 1)
			{
				no_me_2 = 1;
				avg_cycle = 0;
				for(i = 0; i < motion_peaks.num[1]-1; i++)
				{
					min_diff[i] = (FLOAT32)(motion_peaks.high_ind[i+1]-motion_peaks.high_ind[i]);
					avg_cycle += min_diff[i];
				}
				for(i = 0; i < motion_peaks.num[0]-1; i++)
				{
					if((i+motion_peaks.num[1]-1) < 20)
					{
						min_diff[i+motion_peaks.num[1]-1] = (FLOAT32)(motion_peaks.low_ind[i+1]-motion_peaks.low_ind[i]);
						avg_cycle += min_diff[i+motion_peaks.num[1]-1];
					}
				}
				avg_cycle /= (motion_peaks.num[1]+motion_peaks.num[0]-2);
			}
		}
	}
	else   //use zero-crossing to estimate cycle
	{
		avg_cycle = 0;
		count1 = 0;
		count2 = 0;

		for(i = 0; i < DATA_SIZE_MIS-1; i++)
		{
			if(pca->data_v[i] > 0 && pca->data_v[i+1] < 0) //falling zero-crossing
			{
				zero_cro_fv[count1] = (fabs(pca->data_v[i])*(i+1)+fabs(pca->data_v[i+1])*i)/(fabs(pca->data_v[i])+fabs(pca->data_v[i+1]));
				count1++;
			}
		}
		for(i = 0; i < count1-1; i++)
			avg_cycle += zero_cro_fv[i+1]-zero_cro_fv[i];

		for(i = 0; i < DATA_SIZE_MIS-1; i++)
		{
			if(pca->data_v[i] < 0 && pca->data_v[i+1] > 0) //rsing zero-crossing
			{
				zero_cro_rv[count2] = (fabs(pca->data_v[i])*(i+1)+fabs(pca->data_v[i+1])*i)/(fabs(pca->data_v[i])+fabs(pca->data_v[i+1]));
				count2++;
			}
		}
		for(i = 0; i < count2-1; i++)
			avg_cycle += zero_cro_rv[i+1]-zero_cro_rv[i];

		avg_cycle /= (count1+count2-2);
	}

	if(fmod(avg_cycle, 1) >= 0.5)
		avg_cycle = ceil(avg_cycle);
	else
		avg_cycle = floor(avg_cycle);

	//  Crash protection in motion
	if(vertical_peaks.num[0] == 0 || vertical_peaks.num[1] == 0 || motion_peaks.num[0] == 0 || motion_peaks.num[1] == 0)
		return 2;

	if(pca->motion_parametrs[2] > 6 && pca->motion_parametrs[2] > pca->vertical_parametrs[2] && avg_cycle > 3)
	{
		if(avg_cycle*2 <= DATA_SIZE_MIS)		//data sample enough to one stride
			stride_size = (INT8)(avg_cycle*2);
		else
			stride_size = DATA_SIZE_MIS;

		for(i = 0; i < stride_size; i++)
		{
			x_drift[i] = pca->data_m[i] - pca->motion_parametrs[0];
			z_drift[i] = pca->data_v[i] - pca->vertical_parametrs[0];
			m_drift[i] = pca->Acc_Mag[i] - pca->acceleration_norm_parametrs[0];
		}

		//Xcorr_phase method 1
		phase_lag = (INT8)(ceil(avg_cycle*0.25F)-1);
		if(phase_lag == 0)
			phase_lag = (INT16)(ceil(avg_cycle*0.25F));
		for(i = 0; i < stride_size-phase_lag; i++)
		{
			x_drift_1[i] = pca->data_m[i+phase_lag];
			z_drift_1[i] = pca->data_v[i];
		}
		forward_score = calculate_correlation_mis_float_float_mis(x_drift_1, z_drift_1, stride_size-phase_lag);
		if(fabs(forward_score) > 0.6)
		{
			if(forward_score > 0)
				phase_shift = -90;
			else
				phase_shift = 90;
		}
		else
		{
			phase_lag = xcorr_c_float_float_mis(z_drift, x_drift, stride_size, r, lags, 0);

			if(fabs((FLOAT32)(phase_lag)) > avg_cycle)
			{
				n_cycle = (INT8)(floor(fabs((FLOAT32)(phase_lag))/avg_cycle));
				phase_lag = (INT16)((fabs((FLOAT32)(phase_lag))-n_cycle*avg_cycle)*(phase_lag > 0 ? 1:-1));
			}
			phase_shift = phase_lag/avg_cycle*360.0F;
			phase_shift = phase_shift-(phase_shift >  180.0F ?  360.0F:0.0F);
			phase_shift = phase_shift-(phase_shift < -180.0F ? -360.0F:0.0F);
		}	//End of Xcorr_phase method 1

		if(phase_shift <= -45.0 &&  phase_shift >= -135.0)
			step_f++;
		else
		{
			if(phase_shift <= 135.0 &&  phase_shift >= 45.0)
				step_b++;
			else
			{
				//Start Xcorr_phase method 2
				phase_lag = xcorr_c_float_float_mis(x_drift, m_drift, stride_size, r, lags, 0);
				if(fabs((FLOAT32)(phase_lag)) > avg_cycle)
				{
					n_cycle = (INT8)(floor(fabs((FLOAT32)(phase_lag))/avg_cycle));
					phase_lag = (INT16)((fabs((FLOAT32)(phase_lag))-n_cycle*avg_cycle)*(phase_lag > 0 ? 1:-1));
				}
				phase_shift = phase_lag/avg_cycle*360;

				phase_lag = xcorr_c_float_float_mis(z_drift, m_drift, stride_size,r,lags,0);
				if(fabs((FLOAT32)(phase_lag)) > avg_cycle)
				{
					n_cycle = (INT8)(floor(fabs((FLOAT32)(phase_lag))/avg_cycle));
					phase_lag = (INT16)((fabs((FLOAT32)(phase_lag))-n_cycle*avg_cycle)*(phase_lag > 0 ? 1:-1));
				}
				phase_shift1 = phase_lag/avg_cycle*360;

				phase_shift = phase_shift1-phase_shift;
				phase_shift = phase_shift-(phase_shift >  180.0F ?  360.0F:0.0F);
				phase_shift = phase_shift-(phase_shift < -180.0F ? -360.0F:0.0F);
				if(phase_shift <= -45.0 && phase_shift >= -135.0)
					step_f++;
				else
				{
					if(phase_shift <= 135.0 && phase_shift >= 45.0)
						step_b++;
					else
					{
						//Start Xcorr_phase method 3
						count1 = 0; count2 = 0; count3 = 0; count4 = 0;
						for(i = 0; i < stride_size-1; i++)
						{
							if(x_drift[i] > 0 && x_drift[i+1] < 0) //falling zero-crossing of motion signal
							{
								zero_cro_fm[count1] = (fabs(x_drift[i])*(i+2)+fabs(x_drift[i+1])*(i+1))/(fabs(x_drift[i])+fabs(x_drift[i+1]));
								count1++;
							}

							if(x_drift[i] < 0 && x_drift[i+1] > 0) //rsing zero-crossing of motion signal
							{
								zero_cro_rm[count2] = (fabs(x_drift[i])*(i+2)+fabs(x_drift[i+1])*(i+1))/(fabs(x_drift[i])+fabs(x_drift[i+1]));
								count2++;
							}
						}

						for(i = 0; i < stride_size-1; i++)
						{
							if(z_drift[i] > 0 && z_drift[i+1] < 0) //falling zero-crossing of vertical signal
							{
								zero_cro_fv[count3] = (fabs(z_drift[i])*(i+2)+fabs(z_drift[i+1])*(i+1))/(fabs(z_drift[i])+fabs(z_drift[i+1]));
								count3++;
							}

							if(z_drift[i] < 0 && z_drift[i+1] > 0) //rsing zero-crossing of vertical signal
							{
								zero_cro_rv[count4] = (fabs(z_drift[i])*(i+2)+fabs(z_drift[i+1])*(i+1))/(fabs(z_drift[i])+fabs(z_drift[i+1]));
								count4++;
							}
						}
						m = 0;
						for(i = 0; i < count1; i++)
						{
							temp = 999;
							for(j = 0; j < count3; j++)
							{
								if(fabs(zero_cro_fv[j]-zero_cro_fm[i]) < temp)
								{
									temp = fabs(zero_cro_fv[j]-zero_cro_fm[i]);
									d_phase[m] = (zero_cro_fv[j]-zero_cro_fm[i])/avg_cycle*360;
								}
							}
							m++;
						}
						for(i = 0; i < count2; i++)
						{
							temp = 999;
							for(j = 0; j < count4; j++)
							{
								if(fabs(zero_cro_rv[j]-zero_cro_rm[i]) < temp)
								{
									temp = fabs(zero_cro_rv[j]-zero_cro_rm[i]);
									d_phase[m] = (zero_cro_rv[j]-zero_cro_rm[i])/avg_cycle*360.0F;
								}
							}
							m++;
						}

						temp = 0;
						for(i = 0; i < m; i++)
						{
							d_phase[i] -= d_phase[i] >  180.0F ?  360.0F:0.0F;
							d_phase[i] -= d_phase[i] < -180.0F ? -360.0F:0.0F;
							temp += d_phase[i];
						}
						phase_shift = temp/m;
						if(phase_shift <= -45.0 &&  phase_shift >= -135.0)
							step_f++;
						else if(phase_shift <= 135.0 &&  phase_shift >= 45.0)
							step_b++;
					}
				}//END of method 2
			}
		}// END of method 1
	}// END of method

	if(step_b < step_f)
		return 0;
	else if(step_b > step_f)
		return 1;
	else
		return 2;
}

INT16 xcorr_c(DOUBLE64 *X, DOUBLE64 *Y, INT16 N, DOUBLE64 *r, INT16 * lags, INT16 control)
{
	//size of r and lags should be 2*N-1
	DOUBLE64 max_Rxy = -999;
	INT16 i, j,ind_max_Rxy = 0;
	DOUBLE64 factor = 0;

	for( i = 1; i <= 2*N-1; i++)
	{
		lags[i-1] = i-N;
		r[i-1] = 0;
		if( lags[i-1] >= 0)
		{
			for( j = 0; j <= N-lags[i-1]-1; j++)
				r[i-1] += (X[j+lags[i-1]]*Y[j]);
		}
		else
		{
			for( j = 0; j <= N+lags[i-1]-1; j++)
				r[i-1] += (Y[j-lags[i-1]]*X[j]);
		}
		if(r[i-1] > max_Rxy)
		{
			max_Rxy = r[i-1];
			ind_max_Rxy = lags[i-1];
		}
	}

	//normalization
	factor = r[N-1];
	if( control ==1 && r[N-1] != 0.0)
	{
		for( i = 0; i < 2*N-1; i++)
		{
			r[i] /= factor;
		}
	}
	return ind_max_Rxy;
}
DOUBLE64 running_avg_mis(DOUBLE64 prev, DOUBLE64 newdata,INT32 newcount)
{
	DOUBLE64 avgdata=0.0;
	INT32 count;
	DOUBLE64 temp1,temp2;
	temp1 = temp2 = 0.0;
	count = newcount;
	count++;
	temp1 = (count - 1.0)/(DOUBLE64)count;
	temp2 = 1.0/(DOUBLE64)count;
	avgdata = (temp1 * prev) + (temp2 * newdata);
	return avgdata;
}
DOUBLE64 Average_Data_mis(DOUBLE64 *data,INT16 num_elements)
{
	DOUBLE64 prev = data[0];
	INT16 i;
	for (i = 1; i < num_elements;i++)
	{
		prev = running_avg_mis(prev,data[i],num_elements);
	}
	return prev;
}
void find_max_min_float_mis(FLOAT32 *vector, FLOAT32 *maxmin, UINT16 arraysize)
{
	UINT16 i;
	maxmin[0] = vector[0];
	maxmin[1] = vector[0];
	for (i = 1;i < arraysize;i++)
	{
		if (maxmin[0] < vector[i])
			maxmin[0] = vector[i];
		if (maxmin[1] > vector[i])
			maxmin[1] = vector[i];
	}
}
void Mtrx_Mul_Vctr_mis22_mis( DOUBLE64 Matrix[2][2], DOUBLE64 vector[2], DOUBLE64 vectorR[2] )
{
	INT16 i, j;
	for(i=0; i<2; i++)
	{
		vectorR[i] = 0.0;

		for(j=0; j<2; j++)
			vectorR[i] += Matrix[i][j] * vector[j];
	}
}
void Vctr22_Mul_Vctr22_mis( DOUBLE64 vector1[2], DOUBLE64 vector2[2], DOUBLE64 MatrixR[2][2] )
{
	MatrixR[0][0] = vector1[0]*vector2[0];
	MatrixR[0][1] = vector1[0]*vector2[1];
	MatrixR[1][0] = vector1[1]*vector2[0];
	MatrixR[1][1] = vector1[1]*vector2[1];
}
void Zeros_22_mis(DOUBLE64 Matrix0[2][2])
{
	INT16 i, j;
	for(i=0; i<2; i++)
		for(j=0; j<2; j++)
			Matrix0[i][j] = 0.0;
}
void Mtrx_Mul_Mtrx_mis22_mis( DOUBLE64 Matrix1[2][2], DOUBLE64 Matrix2[2][2], DOUBLE64 MatrixR[2][2] )
{
	INT16 i, j,k;
	Zeros_22_mis( MatrixR );
	for(i=0; i<2; i++)
		for(j=0; j<2; j++)
		{
			for(k=0; k<2; k++) MatrixR[i][j] += Matrix1[i][k] * Matrix2[k][j];
		}
}
void Zeros_33_mis(DOUBLE64 Matrix0[3][3])
{
	INT16 i, j;
	for(i=0; i<3; i++)
		for(j=0; j<3; j++)
			Matrix0[i][j] = 0.0;
}
void Mtrx_Mul_Vctr_mis( DOUBLE64 Matrix[3][3], DOUBLE64 vector[3], DOUBLE64 vectorR[3] )
{
	INT16 i, j;
	for(i=0; i<3; i++)
	{
		vectorR[i] = 0.0;

		for(j=0; j<3; j++)
			vectorR[i] += Matrix[i][j] * vector[j];
	}
}
void VectorCrossProduct_mis( DOUBLE64 vector1[3], DOUBLE64 vector2[3], DOUBLE64 vectorR[3] )
{

	vectorR[0] = vector1[1] * vector2[2] - vector2[1] * vector1[2];
	vectorR[1] = vector2[0] * vector1[2] - vector1[0] * vector2[2];
	vectorR[2] = vector1[0] * vector2[1] - vector2[0] * vector1[1];

}
void DiagMatrix_mis( DOUBLE64 vector[3], DOUBLE64 Matrix[3][3] )
{
	Zeros_33_mis(Matrix);
	Matrix[0][0] = vector[0];
	Matrix[1][1] = vector[1];
	Matrix[2][2] = vector[2];
}
void SkewMatrix_mis( DOUBLE64 vector[3], DOUBLE64 Matrix[3][3] )
{
	Matrix[0][0] = 0.0;
	Matrix[0][1] = -vector[2];
	Matrix[0][2] = vector[1];

	Matrix[1][0] = vector[2];
	Matrix[1][1] = 0.0;
	Matrix[1][2] = -vector[0];

	Matrix[2][0] = -vector[1];
	Matrix[2][1] = vector[0];
	Matrix[2][2] = 0.0;

}
void MsubM_mis( DOUBLE64 Matrix1[3][3], DOUBLE64 Matrix2[3][3], DOUBLE64 MatrixR[3][3] )
{
	INT16 i, j;

	for(i=0; i<3; i++)
		for(j=0; j<3; j++)
			MatrixR[i][j] = Matrix1[i][j] - Matrix2[i][j];
}
void Mtrx_Mul_Mtrx_mis( DOUBLE64 Matrix1[3][3], DOUBLE64 Matrix2[3][3], DOUBLE64 MatrixR[3][3] )
{
	INT16 i, j,k;

	Zeros_33_mis( MatrixR );

	for(i=0; i<3; i++)
		for(j=0; j<3; j++)
		{
			for(k=0; k<3; k++) MatrixR[i][j] += Matrix1[i][k] * Matrix2[k][j];
		}
}
void MMT_mis( DOUBLE64 Matrix[3][3], DOUBLE64 MatrixT[3][3] )
{
	INT16 i, j;
	for(i=0; i<3; i++)
		for(j=0; j<3; j++)
			MatrixT[i][j] = Matrix[j][i];
}
void MaddM_mis( DOUBLE64 Matrix1[3][3], DOUBLE64 Matrix2[3][3], DOUBLE64 AddedMatrix[3][3] )
{
	INT16 i, j;

	for(i=0; i<3; i++)
		for(j=0; j<3; j++)
			AddedMatrix[i][j] = Matrix1[i][j] + Matrix2[i][j];
}
DOUBLE64 Vec_Mul_Vec_mis(DOUBLE64 vector1[3], DOUBLE64 vector2[3])
{
	DOUBLE64 value = 0.0;
	value = vector1[0] * vector2[0] + vector1[1] * vector2[1] + vector1[2] * vector2[2];
	return value;
}
void ScalarWithMtrx_mis(DOUBLE64 Scalar, DOUBLE64 Matrix[3][3], DOUBLE64 MatrixR[3][3])
{
	INT16 i, j;

	for(i=0; i<3; i++)
		for(j=0; j<3; j++)
			MatrixR[i][j] = Scalar * Matrix[i][j];
}
void MEM_mis(DOUBLE64 Matrix[3][3], DOUBLE64 CopyMatrix[3][3])
{
	INT16 i, j;
	for(i=0; i<3; i++)
		for(j=0; j<3; j++)
			CopyMatrix[i][j] = Matrix[i][j];
}
void Unitmatrix_D_mis(DOUBLE64 **A, INT16 size)
{
	INT16 i, j;
	for (i=0; i<size; i++)
		for (j=0; j<size; j++)
		{
			A[i][j] = 0.0;
			if(i==j) A[i][j]=1.0;
		}

}
void MV_D_mis(DOUBLE64 **Matrix, INT16 ar, INT16 ac, DOUBLE64 *vector, DOUBLE64 *vectorR)
{
	INT16 i, j;
	for (i=0; i<ar; i++)
	{
		vectorR[i] = 0.0;
		for (j=0; j<ac; j++)
			vectorR[i] = vectorR[i] + Matrix[i][j] * vector[j];
	}

}
void MMM_D_mis(DOUBLE64 **Matrix1, DOUBLE64 **Matrix2, INT16 ar, INT16 ac, INT16 bc, DOUBLE64 **MatrixR)
{
	INT16 i, j,k;
	DOUBLE64 Matrix2T[40][40];
	DOUBLE64 sigmax;

	for(i=0;i<bc;i++)
		for(j=0;j<ac;j++)
			Matrix2T[i][j] = Matrix2[j][i];

	for ( i = 0; i < ar; i++ )
	  for ( j = 0; j < bc; j++ )
	  {
		  sigmax = 0.0;

		 for ( k = 0; k < ac; k++ )
			sigmax += Matrix1[i][k] * Matrix2T[j][k];
		 MatrixR[i][j] = sigmax;
	  }

}
DOUBLE64 *Mem_Alloc_1d_mis(INT16 size)
{
	DOUBLE64 *pd;
	INT16 i;

	pd = (DOUBLE64 *) malloc ((size_t) size * sizeof(DOUBLE64));

	for(i=0; i<size; i++)
		pd[i] = 0.0;

	return pd;
}
void Free_2d(DOUBLE64 **ppd)
{
	free(ppd[0]);
	free(ppd);
}
void MMT_D_mis(DOUBLE64 **Matrix, INT16 m, INT16 n, DOUBLE64 **MatrixT)
{
	INT16 i, j;

	for(i = 0; i < n ; i++)
		for(j = 0; j < m; j++)
			MatrixT[i][j] = Matrix[j][i];
}
void MaddM_mis_D_mis(DOUBLE64 **Matrix1, DOUBLE64 **Matrix2, INT16 nrow, INT16 ncol, DOUBLE64 **MatrixR)
{
	INT16 i, j;

	for (i=0; i<nrow; i++)
		for(j=0; j<ncol; j++)
			MatrixR[i][j] = Matrix1[i][j] + Matrix2[i][j];
}
void MsubM_D_mis(DOUBLE64 **Matrix1, DOUBLE64 **Matrix2, INT16 nrow, INT16 ncol, DOUBLE64 **MatrixR)
{
	INT16 i, j;

	for (i=0; i<nrow; i++)
		for(j=0; j<ncol; j++)
			MatrixR[i][j] = Matrix1[i][j] - Matrix2[i][j];
}
void dM2d_mis(DOUBLE64 **Matrix, INT16 ar, INT16 ac, DOUBLE64 scalar, DOUBLE64 **MatrixR)
{
	INT16 i, j;
	for (i=0; i<ar; i++)
		for (j=0; j<ac; j++)
			MatrixR[i][j] = Matrix[i][j]*scalar;
}
STATUS InvertMatrix_mis( DOUBLE64 **Matrix, INT16 n)
{
	INT16 i,j,k;
	if( n==0 )
		return(FAILURE);



	for(i=0; i<n; i++)
	{
		if( Matrix[i][i] < 0.0 )
			return(FAILURE);
		if( fabs(Matrix[i][i]) < 1.0e-15 )
			return(FAILURE);
	}


	for (j = 0; j < n; j++)
	{
		for(k = 0; k < j; k++)
			Matrix[j][j] -= Matrix[j][k] * Matrix[j][k];

		if( Matrix[j][j]<0.0 )
			return(FAILURE);

		Matrix[j][j] = sqrt( Matrix[j][j] );

		for(i = j + 1; i < n; i++)
		{
			for (k = 0; k < j; k++)
				Matrix[i][j] -= Matrix[i][k] * Matrix[j][k];

			if( fabs(Matrix[j][j]) < 1.0e-12 )
				return(FAILURE);

			Matrix[i][j] /= Matrix[j][j];
		}
	}


	for (j = 0; j < n; j++)
	{
		Matrix[j][j] = 1.0 / Matrix[j][j];

		for (i = j + 1; i < n; i++)
		{
			Matrix[i][j] *= -Matrix[j][j] / Matrix[i][i];

			for (k = j + 1; k < i; k++)
				Matrix[i][j] -= Matrix[i][k] * Matrix[k][j] / Matrix[i][i];
		}
	}


	for (j = 0; j < n; j++)
	{
		for (i = j; i < n; i++)
		{
			Matrix[i][j] *= Matrix[i][i];

			for (k = i + 1; k < n; k++)
				Matrix[i][j] += Matrix[k][i] * Matrix[k][j];
		}
	}


	for (i = 1; i < n; i++)
		for (j = 0; j < i; j++)
			Matrix[j][i] = Matrix[i][j];

	return(SUCCESS);

}
void fft_mis(INT16 Index, INT16 start_index, INT16 diff, DOUBLE64 *time, DOUBLE64 **Freqs_M, DOUBLE64 **tempFreqs_M)
{
	INT16 k,a, b, c, d,m;
	DOUBLE64 A, cosA, sinA,temp1, temp2;
	m = Index/2;

	if (Index != 2)
	{
		fft_mis(m, start_index, 2*diff, time, tempFreqs_M, Freqs_M);
		fft_mis(m, start_index+diff, 2*diff, time, tempFreqs_M, Freqs_M);

		for(k=0; k<m; k++)
		{
			a = start_index + k * diff;
			b = a + m * diff;
			c = start_index + 2 * k * diff;
			d = c + diff;

			A = 2.0 * PIf * k / Index;
			cosA = cos(A);
			sinA = sin(A);

			temp1 = cosA * tempFreqs_M[d][0] + sinA * tempFreqs_M[d][1];
			temp2 = cosA * tempFreqs_M[d][1] - sinA * tempFreqs_M[d][0];

			Freqs_M[a][0] = tempFreqs_M[c][0] + temp1;
			Freqs_M[a][1] = tempFreqs_M[c][1] + temp2;
			Freqs_M[b][0] = tempFreqs_M[c][0] - temp1;
			Freqs_M[b][1] = tempFreqs_M[c][1] - temp2;

		}
	}
	else
	{
		a = start_index;
		b = a + diff;

		Freqs_M[a][0] = time[a] + time[b];
		Freqs_M[b][0] = time[a] - time[b];

		Freqs_M[a][1] = Freqs_M[b][1] = 0.0;
	}
}
void zero_padding_mis(INT16 I,INT16 N,DOUBLE64 *x, DOUBLE64 *newx){
	INT16 j;
	for (j=0;j<I;j++){
		if (N>j)
			newx[j]=x[j];
		else
			newx[j]=0.0;
	}
}
void make_freq_axis_mis(INT16 Fs,INT16 temp1,DOUBLE64 *f)
{
	INT16 i;
	DOUBLE64 lim,cnst;
	lim = Fs/2;
	temp1/=2;
	cnst = 1/(lim-1);
	f[0] = 0;
	for (i = 1;i<lim;i++)
		f[i]=(f[i-1]+cnst);
	for (i = 0;i<lim;i++)
		f[i]*=temp1;
}
DOUBLE64 vectors_dot_product_mis(DOUBLE64 *vec1, DOUBLE64 *vec2, INT16 wind_size)
{ // return the dot product for two vectors
	INT16 i;
	DOUBLE64 sum = 0.0;
	for(i=0; i<wind_size; i++)
		sum += vec1[i]*vec2[i];
	return sum;
}
////////////// Start of FLOAT32 /////////////////////////////////////////////////////////////////////
void BubbleSort_mis(INT8 *data_sort, INT8 num, INT contl)
{
	UINT8 swapped = 1;
	INT8 i = 0, j = 0;
	INT8 temp = 0;

	while(swapped)
	{
		swapped = 0;
		i++;
		for(j = 0; j < (num - i); j++)
		{
			if(contl == 0 && data_sort[j+1] > data_sort[j])  // descent
			{
				temp   = data_sort[j];
				data_sort[j]   = data_sort[j+1];
				data_sort[j+1] = temp;
				swapped = 1;
			}
			if(contl == 1 && data_sort[j+1] < data_sort[j]) // ascent
			{
				temp   = data_sort[j];
				data_sort[j]   = data_sort[j+1];
				data_sort[j+1] = temp;
				swapped = 1;
			}
		}
	}
}
void ChangeArrayMemoryLayout_float_att_float_mis(INT32 r, INT32 c, FLOAT32 **ptr, FLOAT32 *Arr, UINT8 InitToZero)
{
	INT32 i;
	for(i=0; i < r; i++)
		ptr[i] = (FLOAT32 *)Arr+i*c;
	if(InitToZero == 1)
		Zeros2D_float_float_mis(ptr, r, c);
}
void Zeros2D_float_float_mis(FLOAT32 **Matrix, INT32 row, INT32 col)
{
	INT16 i, j;
	for(i=0; i<row; i++)
		for(j=0; j<col; j++)
			Matrix[i][j] = 0.0;
}
void Zeros1D_float_float_mis(FLOAT32 *vector, INT16 size)
{
	INT16 i;
	for(i=0; i<size; i++)
		vector[i] = 0.0;
}
FLOAT32 **Mem_Alloc_2d_mis_float_float_mis(INT16 row, INT16 col)
{
	FLOAT32 **ppd;
	INT16 u;

	ppd = (FLOAT32 **) malloc( (size_t)row * sizeof(double *) );

	ppd[0] = (FLOAT32 *) malloc( (size_t)row * col * sizeof(double) );

	for(u=1; u<row; u++)
		ppd[u] = ppd[0] + u * col;

	Zeros2D_float_float_mis(ppd, row, col);

	return ppd;
}
FLOAT32 *Mem_Alloc_1d_mis_float_float_mis(INT16 size)
{
	FLOAT32 *pd;
	INT16 i;
	pd = (FLOAT32 *) malloc ((size_t) size * sizeof(FLOAT32));
	for(i=0; i<size; i++)
		pd[i] = 0.0;

	return pd;
}
/**
 * @brief	This is vectors dot product calculation function.
 *
 * @details This function recieves 2 pointers for 2 1-D vectors of data with the length
 * 			of the vectors. it returns the dot product of the two vectors.
 *
 * @param[in]	vec1		pointer to the first data vector in float.
 * @param[in]	vec2		pointer to the second data vector in float.
 * @param[in]	wind_size   variable for the data vector length.
 *
 * @return	returns the dot product value.
 */
FLOAT32 vectors_dot_product_mis_float_float_mis(FLOAT32 *vec1, FLOAT32 *vec2, INT16 wind_size)
{
	INT16 i;
	FLOAT32 sum = 0.0;
	for(i=0; i<wind_size; i++)
		sum += vec1[i]*vec2[i];
	return sum;
}
/**
 * @brief	This is a phase shift angle calculation function.
 *
 * @details This function recieves 2 pointers for 2 1-D vectors of data with the length
 * 			of the vectors. it returns the phase shift angle between the two vectors
 * 			in rad.
 *
 * @param[in]	vec1		pointer to the first data vector in float.
 * @param[in]	vec2		pointer to the second data vector in float.
 * @param[in]	wind_size   variable for the data vector length.
 *
 * @return	return the value of the phase shift angle in rad.
 *
 * @see vectors_dot_product_mis_float_float_mis().
 */
FLOAT32 Phase_Shift_Angle_mis_float_float_mis(FLOAT32 *vec1, FLOAT32 *vec2, INT16 wind_size)
{
	return (FLOAT32)(acos(vectors_dot_product_mis_float_float_mis(vec1, vec2, wind_size)/(sqrt(vectors_dot_product_mis_float_float_mis(vec1, vec1, wind_size))*sqrt(vectors_dot_product_mis_float_float_mis(vec2, vec2, wind_size)))));
}
void LowPassFilter_float_float_mis(FLOAT32 *InputSignal, UINT8 fc,UINT8 fs,UINT8 FilterOrder, FLOAT32 *FilteredOutPut)
{
	//Construct the Impulse Response of the Filter
	FLOAT32 h[21];
	UINT8 i;
	FLOAT32 n1, n2, n;
	FLOAT32 num;
	FLOAT32 num1 = ((FLOAT32)2.0)*fc/fs;
	FLOAT32 n3;
	n1 = -0.5F * (FilterOrder-1);
	n2 =  0.5F * (FilterOrder-1);

	num = PIf*num1;
	for(n = n1, i = 0; n <= n2 ; n++, i++)
	{
		if(n != 0.0)
		{
			n3   = num*n;
			h[i] = num1*(FLOAT32)(sin(n3))/n3;
		}
		else//Lim of sinc(x)/x = 1 when x approaches zero
			h[i] = 1;
	}

	//Construct the filtered output by convolution
	*FilteredOutPut = 0;
	for(i = 0; i < FilterOrder; i++)
	{
		*FilteredOutPut += InputSignal[i] * h[FilterOrder-i-1];
	}
}
void LowPassFilter2_mis(FLOAT32 *InputSignal, UINT8 fc, UINT8 fs, UINT8 FilterOrder, FLOAT32 *FilteredOutPut, UINT8 N)
{
	//Construct the Impulse Response of the Filter
	FLOAT32 h[SENSORS_RATE_MOBILE + 1];
	UINT8 i, k;
	FLOAT32 n1, n2, n;
	FLOAT32 const_num, const_num1 = 2.0f*fc/fs, n3, m_l = 0;
	n1 = -0.5F * (FilterOrder-1);
	n2 =  0.5F * (FilterOrder-1);
	const_num = PIf*const_num1;
	for(n = n1,i = 0; n <= n2 ; n++, i++)
	{
		if(n != 0)
		{
			n3   = const_num*n;
			h[i] = const_num1*((FLOAT32)(sin(n3)))/n3;
		}
		else	//Lim of sinc(x)/x = 1 when x approaches zero
			h[i] = 1;
	}
	//Construct the filtered output by convolution
	for(k = 0; k < N-FilterOrder; k++)
	{
		m_l = 0;
		for(i = 0; i < FilterOrder; i++)
			m_l += InputSignal[k+i] * h[FilterOrder-i-1];

		FilteredOutPut[k] = m_l;
	}
}
void Free_2d_float_float_mis(FLOAT32 **ppd)
{
	free(ppd[0]);
	free(ppd);
}
void Free_1d_float_float_mis(FLOAT32 *pd)
{
	free(pd);
}
FLOAT32 calculate_correlation_mis_float_float_mis(FLOAT32 *X, FLOAT32 *Y, INT16 dimension)
{
	INT16 i;
	FLOAT32 sigmaX, sigmaY, sigmaX2, sigmaY2, sigmaXY, num, den;
	sigmaX = sigmaY = sigmaX2 = sigmaY2 = sigmaXY = 0.0;
	for (i = 0; i < dimension; i++)
	{
		sigmaX += X[i];
		sigmaY += Y[i];
		sigmaXY += (X[i] * Y[i]);
		sigmaX2 += SQR(X[i]);
		sigmaY2 += SQR(Y[i]);
	}
	num = (dimension * sigmaXY) - (sigmaX * sigmaY);
	den = (FLOAT32)(sqrt((dimension * sigmaX2 - SQR(sigmaX))* (dimension * sigmaY2 - SQR(sigmaY))));
	return (num/den);
}
INT16 xcorr_c_float_float_mis(FLOAT32 *X, FLOAT32 *Y, INT16 N, FLOAT32 *r, INT16 * lags, INT16 control)
{
	//size of r and lags should be 2*N-1
	FLOAT32 max_Rxy = -999;
	INT16 i, j, ind_max_Rxy = 0;
	FLOAT32 factor = 0;

	for( i = 1; i <= 2*N-1; i++)
	{
		lags[i-1] = i-N;
		r[i-1] = 0;
		if( lags[i-1] >= 0)
		{
			for( j = 0; j <= N-lags[i-1]-1; j++)
				r[i-1] += (X[j+lags[i-1]]*Y[j]);
		}
		else
		{
			for( j = 0; j <= N+lags[i-1]-1; j++)
				r[i-1] += (Y[j-lags[i-1]]*X[j]);
		}
		if(r[i-1] > max_Rxy)
		{
			max_Rxy = r[i-1];
			ind_max_Rxy = lags[i-1];
		}
	}

	//normalization
	factor = r[N-1];
	if( control ==1 && r[N-1] != 0.0)
	{
		for( i = 0; i < 2*N-1; i++)
		{
			r[i] /= factor;
		}
	}
	return ind_max_Rxy;
}
INT16 Calculate_PCA_float_float_mis(FLOAT32 **a1, UINT8 m, FLOAT32 **u, FLOAT32 v[2][2], FLOAT32 eps, FLOAT32 mean_x, FLOAT32 mean_y)
{
	UINT8 i;
	UINT8 n = 2;
	//FLOAT32 mean_m[2];
	FLOAT32 *pv[2];
	INT16 check_i = 0;
	ChangeArrayMemoryLayout_float_att_float_mis(n, n, pv, (FLOAT32 *)v, 1);

	for(i = 0; i < m; i++)
	{
		a1[i][0] = a1[i][0]-mean_x;
		a1[i][1] = a1[i][1]-mean_y;
	}
    check_i = SVD_float_float_mis(a1, m, n, u, a1, pv, eps);
	//Free_2d(a);
    return check_i;
}
INT16 SVD_float_float_mis(FLOAT32 **a1, INT16 m,INT16 n, FLOAT32 **u, FLOAT32 **a, FLOAT32 **v, DOUBLE64 eps)
{
	//Singular value decomposition. Feb, 2013
	//		a1 = U*a*V';
	//Input variables:
	//		a1 = target matrix
	//		m = number of row
	//		n = number of column
	//		eps = required accuracy
	//Output variables:
	//		a = diagonal matrix
	//		u = unitary matrix
	//		v = unitary matrix

	//INT16 ka = MAX(m, n)+1;
	INT16 k = MIN(n,m-1);
	INT16 l = MIN(m,n-2);
	INT16 i,j,it,ll,kk,mm,nn,m1,ks;
	FLOAT32 d,dd,t,sm,sm1,em1,sk,ek,b,c,shh,fg[2],cs[2];
	FLOAT32 s[41],e[41],w[41];  //maximum size: m=40 or n = 40;

	for( i = 0; i < m; i++)
		for( j = 0; j < n; j++)
			a[i][j] = a1[i][j];
	it=60;
	ll=MAX(k,l);

	if (ll>=1)
	{
		for (kk=1; kk<=ll; kk++)
		{
			if (kk<=k)
			{
				d=0.0;
				for (i=kk; i<=m; i++)
				{
					d=d+a[i-1][kk-1]*a[i-1][kk-1];
				}
				s[kk-1]=(FLOAT32)(sqrt(d));
				if (s[kk-1]!=0.0)
				{
					if (a[kk-1][kk-1] !=0.0)
					{
						s[kk-1]=(FLOAT32)(fabs(s[kk-1]));
						if (a[kk-1][kk-1]<0.0)
						{	s[kk-1]=-s[kk-1];}
					}
					for (i=kk; i<=m; i++)
					{	a[i-1][kk-1]=a[i-1][kk-1]/s[kk-1];}

					a[kk-1][kk-1]=(FLOAT32)(1.0+a[kk-1][kk-1]);
				  }
				  s[kk-1]=-s[kk-1];
			}
			if (n>=kk+1)
			{
				for (j=kk+1; j<=n; j++)
				{
					if ((kk<=k)&&(s[kk-1]!=0.0))
					{
						d=0.0;
						for (i=kk; i<=m; i++)
						{
							d=d+a[i-1][kk-1]*a[i-1][j-1];
						}

						d=-d/a[kk-1][kk-1];
						for (i=kk; i<=m; i++)
						{
							a[i-1][j-1]=a[i-1][j-1]+d*a[i-1][kk-1];
						}
					}
					e[j-1]=a[kk-1][j-1];
				}
			}
			if (kk<=k)
			{
				for (i=kk; i<=m; i++)
				{    u[i-1][kk-1]=a[i-1][kk-1];}
			}
			if (kk<=l)
			{
				d=0.0;
				for (i=kk+1; i<=n; i++)
				{	d=d+e[i-1]*e[i-1];}
				e[kk-1]=(FLOAT32)(sqrt(d));
				if (e[kk-1]!=0.0)
				{
					if (e[kk]!=0.0)
					{
						e[kk-1]=(FLOAT32)(fabs(e[kk-1]));
						if (e[kk]<0.0)
							e[kk-1]=-e[kk-1];
					}
					for (i=kk+1; i<=n; i++)
					{	e[i-1]=e[i-1]/e[kk-1];}
					e[kk]=(FLOAT32)(1.0+e[kk]);
				}
				e[kk-1]=-e[kk-1];
				if ((kk+1<=m)&&(e[kk-1]!=0.0))
				{
					for (i=kk+1; i<=m; i++)
					{	w[i-1]=0.0;}
					for (j=kk+1; j<=n; j++)
					{
						for (i=kk+1; i<=m; i++)
						{
							w[i-1]=w[i-1]+e[j-1]*a[i-1][j-1];
						}
					}
					for (j=kk+1; j<=n; j++)
					{
						for (i=kk+1; i<=m; i++)
						{
						  a[i-1][j-1]=a[i-1][j-1]-w[i-1]*e[j-1]/e[kk];
						}
					}

				}
				for (i=kk+1; i<=n; i++)
				{
					v[i-1][kk-1]=e[i-1];
				}
			}
		}
	}
	mm = MIN(m+1,n);

	if (k<n)
		s[k]=a[k][k];
	if (m<mm)
		s[mm-1]=0.0;
	if (l+1<mm)
		e[l]=a[l][mm-1];

	e[mm-1]=0.0;
	nn = MIN(m,n);

	if (nn>=k+1)
	{
		for (j=k+1; j<=nn; j++)
		{
			for (i=1; i<=m; i++)
			{	u[i-1][j-1]=0.0;}
			u[j-1][j-1]=1.0;
		}
	}
	if (k>=1)
	{
		for (ll=1; ll<=k; ll++)
		{
			kk=k-ll+1;
			if (s[kk-1]!=0.0)
			{
				if (nn>=kk+1)
				{
					for (j=kk+1; j<=nn; j++)
					{
						d=0.0;
						for (i=kk; i<=m; i++)
						{	d=d+u[i-1][kk-1]*u[i-1][j-1]/u[kk-1][kk-1];}

						d=-d;
						for (i=kk; i<=m; i++)
						{	u[i-1][j-1]=u[i-1][j-1]+d*u[i-1][kk-1];}
					}
				}
				for (i=kk; i<=m; i++)
				{
					u[i-1][kk-1]=-u[i-1][kk-1];
				}
				u[kk-1][kk-1]=(FLOAT32)(1.0+u[kk-1][kk-1]);
				if (kk-1>=1)
				{
					for (i=1; i<=kk-1; i++)
					{	u[i-1][kk-1]=0.0;}
				}
			}
			else
			{
				for (i=1; i<=m; i++)
				{	u[i-1][kk-1]=0.0;}
				u[kk-1][kk-1]=1.0;
			}
		}
	}
	for (ll=1; ll<=n; ll++)
	{
		kk=n-ll+1;
		if ((kk<=l)&&(e[kk-1]!=0.0))
		{
			for (j=kk+1; j<=n; j++)
			{
				d=0.0;
				for (i=kk+1; i<=n; i++)
				{
					d=d+v[i-1][kk-1]*v[i-1][j-1]/v[kk][kk-1];
				}
				d=-d;
				for (i=kk+1; i<=n; i++)
				{    v[i-1][j-1]=v[i-1][j-1]+d*v[i-1][kk-1];}
			}
		}
		for (i=1; i<=n; i++)
		{	v[i-1][kk-1]=0.0;}
		v[kk-1][kk-1]=1.0;
	}
	for (i=1; i<=m; i++)
	{
		for (j=1; j<=n; j++)
		{	a[i-1][j-1]=0.0;}
	}
	m1 = mm;
	it = 60;
	while(1==1)
	{
		if (mm==0)
		{
			SVD_1_float_float_mis(a,e,s,v,m,n);
			//free(s);free(e);free(w);
			return(1);
		}
		if (it==0)
		{
			SVD_1_float_float_mis(a, e, s, v, m, n);
			//free(s); free(e); free(w);
			return(-1);
		}

		kk=mm-1;
		while ((kk!=0)&&(fabs(e[kk-1])!=0.0))
		{
			d=(FLOAT32)(fabs(s[kk-1])+fabs(s[kk]));
			dd=(FLOAT32)(fabs(e[kk-1]));
			if (dd>eps*d)
			{
				kk=kk-1;
			}
			else
			{
				e[kk-1]=0.0;
			}
		}
		if (kk==mm-1)
		{
			kk=kk+1;
			if (s[kk-1]<0.0)
			{
				s[kk-1]=-s[kk-1];
				for (i=1; i<=n; i++)
				{
					v[i-1][kk-1]=-v[i-1][kk-1];
				}
			}

			while ((kk!=m1)&&(s[kk-1]<s[kk]))
			{
				d=s[kk-1]; s[kk-1]=s[kk]; s[kk]=d;
				if (kk<n)
				{
					for (i=1; i<=n; i++)
					{
						d=v[i-1][kk-1];
						v[i-1][kk-1]=v[i-1][kk];
						v[i-1][kk]=d;
					}
				}
				if (kk<m)
				{
					for (i=1; i<=m; i++)
					{
						d=u[i-1][kk-1];
						u[i-1][kk-1]=u[i-1][kk];
						u[i-1][kk]=d;
					}
				}
				kk=kk+1;
			}
			it=60;
			mm=mm-1;
		}
		else
		{
			ks=mm;
			while ((ks>kk)&&(fabs(s[ks-1])!=0.0))
			{
				d=0.0;
				if(ks!=mm)
					d=(FLOAT32)(d+fabs(e[ks-1]));
				if(ks!=kk+1)
					d=(FLOAT32)(d+fabs(e[ks-2]));
				dd=(FLOAT32)(fabs(s[ks-1]));
				if (dd>eps*d)
					ks=ks-1;
				else
					s[ks-1]=0.0;
			}
			if (ks==kk)
			{
				kk=kk+1;
				d=(FLOAT32)(fabs(s[mm-1]));
				t=(FLOAT32)(fabs(s[mm-2]));
				if (t>d) d=t;
				t=(FLOAT32)(fabs(e[mm-2]));
				if (t>d) d=t;
				t=(FLOAT32)(fabs(s[kk-1]));
				if (t>d) d=t;
				t=(FLOAT32)(fabs(e[kk-1]));
				if (t>d) d=t;
				sm=s[mm-1]/d; sm1=s[mm-2]/d;
				em1=e[mm-2]/d;
				sk=s[kk-1]/d; ek=e[kk-1]/d;
				b=(FLOAT32)(((sm1+sm)*(sm1-sm)+em1*em1)/2.0);
				c=sm*em1; c=c*c; shh=0.0;
				if ((b!=0.0)||(c!=0.0))
				{
					shh=(FLOAT32)(sqrt(b*b+c));
					if (b<0.0) shh=-shh;
						shh=c/(b+shh);
				}
				fg[0]=(sk+sm)*(sk-sm)-shh;
				fg[1]=sk*ek;
				for (i=kk; i<=mm-1; i++)
				{
					SVD_2_float_float_mis(fg,cs);
					if (i!=kk)
						e[i-2]=fg[0];
					fg[0]=cs[0]*s[i-1]+cs[1]*e[i-1];
					e[i-1]=cs[0]*e[i-1]-cs[1]*s[i-1];
					fg[1]=cs[1]*s[i];
					s[i]=cs[0]*s[i];
					if ((cs[0]!=1.0)||(cs[1]!=0.0))
					{
						for (j=1; j<=n; j++)
						{
							d=cs[0]*v[j-1][i-1]+cs[1]*v[j-1][i];
							v[j-1][i]=-cs[1]*v[j-1][i-1]+cs[0]*v[j-1][i];
							v[j-1][i-1]=d;
						}
					}
					SVD_2_float_float_mis(fg,cs);
					s[i-1]=fg[0];
					fg[0]=cs[0]*e[i-1]+cs[1]*s[i];
					s[i]=-cs[1]*e[i-1]+cs[0]*s[i];
					fg[1]=cs[1]*e[i];
					e[i]=cs[0]*e[i];
					if (i<m)
					{
						if ((cs[0]!=1.0)||(cs[1]!=0.0))
						{
							for (j=1; j<=m; j++)
							{
								d=cs[0]*u[j-1][i-1]+cs[1]*u[j-1][i];
								u[j-1][i]=-cs[1]*u[j-1][i-1]+cs[0]*u[j-1][i];
								u[j-1][i-1]=d;
							}
						}
					}
				}
				e[mm-2]=fg[0];
				it=it-1;
			}
			else
			{
				if (ks==mm)
				{
					kk=kk+1;
					fg[1]=e[mm-2]; e[mm-2]=0.0;
					for (ll=kk; ll<=mm-1; ll++)
					{
						i=mm+kk-ll-1;
						fg[0]=s[i-1];
						SVD_2_float_float_mis(fg,cs);
						s[i-1]=fg[0];
						if (i!=kk)
						{
							fg[1]=-cs[1]*e[i-2];
							e[i-2]=cs[0]*e[i-2];
						}
						if ((cs[0]!=1.0)||(cs[1]!=0.0))
							for (j=1; j<=n; j++)
							{
								d=cs[0]*v[j-1][i-1]+cs[1]*v[j-1][mm-1];
								v[j-1][mm-1]=-cs[1]*v[j-1][i-1]+cs[0]*v[j-1][mm-1];
								v[j-1][i-1]=d;
							}
					}
				}
				else
				{
					kk=ks+1;
					fg[1]=e[kk-2];
					e[kk-2]=0.0;
					for (i=kk; i<=mm; i++)
					{
						fg[0]=s[i-1];
						SVD_2_float_float_mis(fg,cs);
						s[i-1]=fg[0];
						fg[1]=-cs[1]*e[i-1];
						e[i-1]=cs[0]*e[i-1];
						if ((cs[0]!=1.0)||(cs[1]!=0.0))
							for (j=1; j<=m; j++)
							{
								d=cs[0]*u[j-1][i-1]+cs[1]*u[j-1][kk-2];
								u[j-1][kk-2]=-cs[1]*u[j-1][i-1]+cs[0]*u[j-1][kk-2];
								u[j-1][i-1]=d;
							}
					}
				}
			 }
		 }
	  }
	//free(s);free(e);free(w);
	return(1);
}
void SVD_1_float_float_mis(FLOAT32 **a, FLOAT32 *e, FLOAT32 *s, FLOAT32 **v, INT16 m, INT16 n)
{
	INT16 i,j;
	FLOAT32 d;
	if (m>=n) i=n;
	else i=m;
	for (j=1; j<=i-1; j++)
	{
		a[j-1][j-1]=s[j-1];
		a[j-1][j]=e[j-1];
	}
	a[i-1][i-1]=s[i-1];
	if (m<n)
		a[i-1][i]=e[i-1];
	for (i=1; i<=n-1; i++)
		for (j=i+1; j<n; j++)
		{
		  //p=(i-1)*n+j-1; q=(j-1)*n+i-1;//
			d=v[i-1][j-1]; v[i-1][j-1]=v[j-1][i-1]; v[j-1][i-1]=d;
		}
	return;
}
void SVD_2_float_float_mis(FLOAT32 *fg, FLOAT32 *cs)
{
	FLOAT32 r,d;
	if((fabs(fg[0])+fabs(fg[1]))==0.0)
	{ cs[0]=1.0; cs[1]=0.0; d=0.0;}
	else
	{ d=(FLOAT32)(sqrt(fg[0]*fg[0]+fg[1]*fg[1]));
	if (fabs(fg[0])>fabs(fg[1]))
	{ d=(FLOAT32)(fabs(d));
	if (fg[0]<0.0) d=-d;
	}
	if (fabs(fg[1])>=fabs(fg[0]))
	{ d=(FLOAT32)(fabs(d));
	if (fg[1]<0.0) d=-d;
	}
	cs[0]=fg[0]/d; cs[1]=fg[1]/d;
	}
	r=1.0;
	if (fabs(fg[0])>fabs(fg[1])) r=cs[1];
	else
		if(cs[0]!=0.0) r=(FLOAT32)(1.0/cs[0]);
	fg[0]=d; fg[1]=r;
	return;
}
void MEM_mis_D_float_float_mis(FLOAT32 **Matrix, INT16 rsi, INT16 rei, INT16 csi, INT16 cei, FLOAT32 **CopyMatrix)
{
	INT16 i, j;

	for(i=rsi;i<=rei;i++)
		for(j=csi;j<=cei;j++)
			CopyMatrix[i-rsi][j-csi] = Matrix[i][j];
}

void Euler2Dcm_att_float_float_mis(FLOAT32 roll, FLOAT32 pitch, FLOAT32 heading, FLOAT32 **R)
{
	FLOAT32 cx,cy,cz, sx,sy,sz;
	cx = (FLOAT32)(cos(roll)); cy = (FLOAT32)(cos(pitch)); cz = (FLOAT32)(cos(heading));
	sx = (FLOAT32)(sin(roll)); sy = (FLOAT32)(sin(pitch)); sz = (FLOAT32)(sin(heading));
	R[0][0] = cy*cz;
	R[1][0] = cy*sz;
	R[2][0] = -sy;

	R[0][1] = sx*sy*cz+cx*(-sz);
	R[1][1] = sx*sy*sz+cx*cz;
	R[2][1] = sx*cy;

	R[0][2] = cx*sy*cz+(-sx)*(-sz);
	R[1][2] = cx*sy*sz+(-sx)*cz;
	R[2][2] = cx*cy;
}


void Euler2Dcm_att_float_float_3_3_mis(FLOAT32 roll, FLOAT32 pitch, FLOAT32 heading, FLOAT32 R[3][3])
{
	FLOAT32 cx,cy,cz, sx,sy,sz;
	cx = (FLOAT32)(cos(roll)); cy = (FLOAT32)(cos(pitch)); cz = (FLOAT32)(cos(heading));
	sx = (FLOAT32)(sin(roll)); sy = (FLOAT32)(sin(pitch)); sz = (FLOAT32)(sin(heading));
	R[0][0] = cy*cz;
	R[1][0] = cy*sz;
	R[2][0] = -sy;

	R[0][1] = sx*sy*cz+cx*(-sz);
	R[1][1] = sx*sy*sz+cx*cz;
	R[2][1] = sx*cy;

	R[0][2] = cx*sy*cz+(-sx)*(-sz);
	R[1][2] = cx*sy*sz+(-sx)*cz;
	R[2][2] = cx*cy;
}

 /**
 * @brief		Function to decompose matrix to Q and R matrix
 * @details
 * @param[in] 	A pointer to an nxn matrix in float that needs to decomposed
 * @param[in]	m number of rows in int
 * @param[in]	n number of columns in int
 * @param[in] 	Q pointer to an mxm matrix in float
 * @param[in] 	R pointer to an nxn matrix in float
 * @param[out]	CopyMatrix pointer to the matrix which will have the copy elements of
 * 				'Matrix' in float
 * @return 		NONE
 */
STATUS qr_decomposition_mis(FLOAT32 **A, INT16 m, INT16 n, FLOAT32 **Q, FLOAT32 **R)
{
	int i, j,jj, k;
	FLOAT32 u=0, t=0,alpha = 0;

	if( m < n)
	{
		return FAILURE;
	}
	else
	{
		convert_to_identity_matrix_nxn_mis(Q, m);
		for( k = 0; k <= MIN(m-1,n); k++)
		{
			u = 0.0;
			for( i = k; i < m;i++)
			{
				if( fabs(A[i][k]) > u)
					u = fabs(A[i][k]);
			}

			alpha = 0.0;
			for( i = k; i < m; i++)
				alpha += pow(A[i][k]/u,2);

			if( A[k][k] > 0)
				u = -u;
			alpha = u*sqrt(alpha);

			if( fabs(alpha) == 0)
				return FAILURE;

			u = sqrt(2.0*alpha*(alpha-A[k][k]));

			if( u > 0)
			{
				A[k][k] = (A[k][k]-alpha)/u;
				for( i = k+1; i < m; i++)
				{
					A[i][k] = A[i][k]/u;
				}

				for( j = 0; j < m; j++)
				{
					t = 0.0;
					for( jj = k; jj < m; jj++)
					{
						t += A[jj][k]*Q[jj][j];
					}

					for( i = k; i < m; i++)
					{
						Q[i][j] = Q[i][j]-2*t*A[i][k];
					}
				}

				for( j = k+1; j < n; j++)
				{
					t =0;
					for( jj = k; jj < m; jj++)
					{
						t += A[jj][k]*A[jj][j];
					}
					for( i = k; i < m; i++)
					{
						A[i][j] = A[i][j]-2*t*A[i][k];
					}
				}

				A[k][k] = alpha;
				for( i = k+1; i < m; i++)
					A[i][k] = 0;
			}
		}

		for( i = 0; i < m-1; i++)
		{
			for( j = i+1; j < m; j++)
			{
				t = Q[i][j];
				Q[i][j] = Q[k][k];
				Q[k][k] = t;
			}
		}
		return SUCCESS;
	}
}
/**
 * @brief		Function to make an nxn matrix into identity matrix
 * @details
 * @param[in] 	A pointer to an nxn matrix of type float
 * @param[in]	size number of columns of the matrix in uint
 * @param[out]	MatrixR nxn matrix of type float
 * @return 		NONE
 */
void convert_to_identity_matrix_nxn_mis(FLOAT32 **A, UINT8 size)
{
	UINT8 i, j;
	for (i=0; i<size; i++)
		for (j=0; j<size; j++)
		{
			A[i][j] = 0.0;
			if(i==j) A[i][j]=1.0;
		}

}
INT16 Calculate_PCA_float_float_2_mis(FLOAT32 **a1, UINT8 m, FLOAT32 **u, FLOAT32 v[2][2], FLOAT32 eps, FLOAT32 mean_x, FLOAT32 mean_y)
{
	UINT8 i;
	UINT8 n = 2;
	//FLOAT32 mean_m[2];
	FLOAT32 *pv[2];
	INT16 check_i = 0;

	ChangeArrayMemoryLayout_float_att_float_mis(n, n, pv, (FLOAT32 *)v, 1);

	//mean_m[0] = 0;
	//mean_m[1] = 0;


	//before calculate SVD, we need to substrate mean value in order to stable eigen vector
	/*for(i = 0; i < m; i++)
		for(j = 0; j < n; j++)
			mean_m[j] += a1[i][j];

	mean_m[0] = mean_m[0]/m;
	mean_m[1] = mean_m[1]/m;*/

	/*for(i = 0; i < m; i++)
		for(j = 0; j < n; j++)
			a1[i][j] = a1[i][j]-mean_m[j];*/

	for(i = 0; i < m; i++)
	{
		a1[i][0] = a1[i][0]-mean_x;
		a1[i][1] = a1[i][1]-mean_y;
	}
	//check_i = SVD_float_float_mis(a1, m, n, u, a1, pv, eps);
	check_i = qr_decomposition_mis(a1, m, n, u, a1);
	check_i = SVD_float_float_mis(a1, n, n, u, a1, pv, eps);
	//intentional added to make it same direction with original PCA method 1
	v[0][0] = -v[0][0]; v[0][1] = -v[0][1];v[1][0] = -v[1][0];v[1][1] = -v[1][1];
	//Free_2d(a);
	return check_i;
}

void zero_matrix_entries_nxn_float_mis(FLOAT32 **Matrix, INT16 row, INT16 col)
{
	INT16 i, j;
	for(i=0; i<row; i++)
		for(j=0; j<col; j++)
			Matrix[i][j] = 0.0;
}
void zero_padding_float_float_mis(INT16 I,INT16 N,FLOAT32 *x, FLOAT32 *newx){
	INT16 j;
	for (j=0;j<I;j++){
		if (N>j)
			newx[j]=x[j];
		else
			newx[j]=0.0f;
	}
}
void fft_mis_float_float_mis(INT16 Index, INT16 start_index, INT16 diff, FLOAT32 *time, FLOAT32 **Freqs_M, FLOAT32 **tempFreqs_M)
{
	INT16 k,a, b, c, d,m;
	FLOAT32 A, cosA, sinA,temp1, temp2;
	m = (INT16)(Index*0.5f);

	if (Index != 2)
    {
		fft_mis_float_float_mis(m, start_index, 2*diff, time, tempFreqs_M, Freqs_M);
		fft_mis_float_float_mis(m, start_index+diff, 2*diff, time, tempFreqs_M, Freqs_M);

		for(k=0; k<m; k++)
		{
			a = start_index + k * diff;
			b = a + m * diff;
			c = start_index + 2 * k * diff;
			d = c + diff;

			A = TWO_PIf * k / Index;
			cosA = cos(A);
			sinA = sin(A);

			temp1 = cosA * tempFreqs_M[d][0] + sinA * tempFreqs_M[d][1];
			temp2 = cosA * tempFreqs_M[d][1] - sinA * tempFreqs_M[d][0];

			Freqs_M[a][0] = tempFreqs_M[c][0] + temp1;
			Freqs_M[a][1] = tempFreqs_M[c][1] + temp2;
			Freqs_M[b][0] = tempFreqs_M[c][0] - temp1;
			Freqs_M[b][1] = tempFreqs_M[c][1] - temp2;

		}
	}
	else
    {
		a = start_index;
		b = a + diff;

		Freqs_M[a][0] = time[a] + time[b];
		Freqs_M[b][0] = time[a] - time[b];

		Freqs_M[a][1] = Freqs_M[b][1] = 0.0;
    }
}



FLOAT32 cal_mean_float_att(FLOAT32* a, UINT16 n) {
	UINT16 i;
	FLOAT32 mean1 = 0.0;

	for(i=0;i<n;i++){
		mean1 += a[i];
	}
	return (mean1 / n);
}


FLOAT32 cal_variance_float(FLOAT32* a, float mean1, UINT16 n) {
	UINT16 i;
	float temp1 = 0.0;
	float std1 = 0.0;

	for (i=0;i<n;i++)
	{
		temp1 = a[i] - mean1;
		std1 += temp1 * temp1;
	}
	return temp1;
}

FLOAT32 cal_roll_att(FLOAT32* f_b){
	FLOAT32 roll = atan2(-f_b[1],-f_b[2]);
	return roll;
}
FLOAT32 cal_pitch_att(FLOAT32* f_b){
	FLOAT32 pitch = atan2(f_b[0],sqrt(f_b[1]*f_b[1]+f_b[2]*f_b[2]));
	return pitch;
}


void Euler2Dcm_att(FLOAT32 att[3], FLOAT32 Cbn[3][3])
{
	FLOAT32 cr, cp, ch, sr, sp, sh;

	cr = cos(att[0]); 	cp = cos(att[1]);	ch = cos(att[2]);
	sr = sin(att[0]); 	sp = sin(att[1]);	sh = sin(att[2]);

	Cbn[0][0] = cp * ch ;
	Cbn[0][1] = -cr*sh + sr*sp*ch;
	Cbn[0][2] = sr*sh + cr*sp*ch ;

	Cbn[1][0] = cp * sh;
	Cbn[1][1] = cr*ch + sr*sp*sh;
	Cbn[1][2] = -sr * ch + cr * sp * sh;

	Cbn[2][0] = - sp;
	Cbn[2][1] = sr * cp;
	Cbn[2][2] = cr * cp;
}

/**
 * @brief		Function to calculate quaternions from direction cosines matrix
 * @details
 * @param[in]	C: A 3x3 direction cosines matrix of type float containing attitude information
 * @param[out]	q: quaternion vector related to C in float
 * @return 		NONE
 */
void Dcm2Quat_att(FLOAT32 C[3][3], FLOAT32 q[4])
{
	FLOAT32 Tr = 0, Pq[4],a;
	UINT8 i, max_id;

	Tr = C[0][0] + C[1][1] + C[2][2];

	Pq[0] = 1 + Tr;
	Pq[1] = 1 + 2.0f * C[0][0] - Tr;
	Pq[2] = 1 + 2.0f * C[1][1] - Tr;
	Pq[3] = 1 + 2.0f * C[2][2] - Tr;

	max_id = 0;
	for(i=0; i<4; i++)
	{
		if (Pq[i] > Pq[max_id])	max_id = i;
	}

	switch (max_id)
	{
	  case 0:
		q[0] = 0.5f*sqrt(Pq[0]);
		a = 0.25f/q[0];
		q[1] = (C[2][1] - C[1][2])*a;
		q[2] = (C[0][2] - C[2][0])*a;
		q[3] = (C[1][0] - C[0][1])*a;
		break;
	  case 1:
		q[1] = 0.5f*sqrt(Pq[1]);
		a = 0.25f/q[1];
		q[0] = (C[2][1] - C[1][2])*a;
		q[2] = (C[1][0] + C[0][1])*a;
		q[3] = (C[0][2] + C[2][0])*a;
		break;
	  case 2:
		q[2] = 0.5f*sqrt(Pq[2]);
		a = 0.25f/q[2];
		q[0] = (C[0][2] - C[2][0])*a;
		q[1] = (C[1][0] + C[0][1])*a;
		q[3] = (C[2][1] + C[1][2])*a;
		break;
	  case 3:
		q[3] = 0.5f*sqrt(Pq[3]);
		a = 0.25f/q[3];
		q[0] = (C[1][0] - C[0][1])*a;
		q[1] = (C[0][2] + C[2][0])*a;
		q[2] = (C[2][1] + C[1][2])*a;
		break;
	}

	if (q[0] < 0)
	{
		for(i=0; i<4; i++) q[i] = -q[i];
	}
}

void Rvec2Quat_att(FLOAT32 r_vec[3], FLOAT32 q[4]){
	FLOAT32 mag2 = r_vec[0]*r_vec[0] + r_vec[1]*r_vec[1] + r_vec[2]*r_vec[2];
	FLOAT32 mag = 0.0;
	FLOAT32 s_mag = 0.0;
	FLOAT32 c, s;

	if(mag2 < PI*PI) {
		mag2 *= 0.25;
		c = 1.0 - mag2/2.0 * (1.0 - mag2/12.0 * (1.0 - mag2/30.0 ));
		s = 1.0 - mag2/6.0 * (1.0 - mag2/20.0 * (1.0 - mag2/42.0 ));

		q[0] = c;
		q[1] = s * 0.5 * r_vec[0];
		q[2] = s * 0.5 * r_vec[1];
		q[3] = s * 0.5 * r_vec[2];
	}
	else {
		mag = sqrt(mag2);
		s_mag = sin(mag/2);

		q[0] = cos(mag/2);
		q[1] = r_vec[0] * s_mag/mag;
		q[2] = r_vec[1] * s_mag/mag;
		q[3] = r_vec[2] * s_mag/mag;

		if (q[0] < 0) {
			q[0] *= -1;
			q[1] *= -1;
			q[2] *= -1;
			q[3] *= -1;
		}
	}
}

void QuatPrdct_att( FLOAT32 q[4], FLOAT32 p[4], FLOAT32 qnew[4] )
{
	UINT8 i;
	FLOAT32 q_temp[4];

	qnew[0] = q[0]*p[0] - q[1]*p[1] - q[2]*p[2] -q[3]*p[3];
	qnew[1] = q[0]*p[1] + q[1]*p[0] + q[2]*p[3] -q[3]*p[2];
	qnew[2] = q[0]*p[2] + q[2]*p[0] + q[3]*p[1] -q[1]*p[3];
	qnew[3] = q[0]*p[3] + q[3]*p[0] + q[1]*p[2] -q[2]*p[1];

	if( qnew[0] < 0.0f )
	{
		for(i=0; i<4; i++) q_temp[i] = -qnew[i];
		for(i=0; i<4; i++) qnew[i] = q_temp[i];
	}
}


void NormQuat_att(FLOAT32 q[4], FLOAT32 q_norm[4])
{
	UINT8 i;
	FLOAT32 value;

	value = ( SQR(q[0]) + SQR(q[1]) + SQR(q[2]) + SQR(q[3]) - 1.0f ) *0.5f;
	for(i=0; i<4; i++) q_norm[i] = ( 1.0f - value ) * q[i];
}

void Quat2Dcm_att( FLOAT32 quar[4], FLOAT32 C[3][3] )
{
	C[0][0] = quar[0] * quar[0] + quar[1] * quar[1] - quar[2] * quar[2] - quar[3] * quar[3];
	C[0][1] = 2.0f * ( quar[1] * quar[2] - quar[0] * quar[3] );
	C[0][2] = 2.0f * ( quar[1] * quar[3] + quar[0] * quar[2] );

	C[1][0] = 2.0f * ( quar[1] * quar[2] + quar[0] * quar[3] );
	C[1][1] = quar[0] * quar[0] - quar[1] * quar[1] + quar[2] * quar[2] - quar[3] * quar[3];
	C[1][2] = 2.0f * ( quar[2] * quar[3] - quar[0] * quar[1] );

	C[2][0] = 2.0f * ( quar[1] * quar[3] - quar[0] * quar[2] );
	C[2][1] = 2.0f * ( quar[2] * quar[3] + quar[0] * quar[1] );
	C[2][2] = quar[0] * quar[0] - quar[1] * quar[1] - quar[2] * quar[2] + quar[3] * quar[3];
}

void Dcm2Euler_att(FLOAT32 Cbn[3][3], FLOAT32 attitude[3])
{
	// between +/-pi
	attitude[0] = atan2(Cbn[2][1], Cbn[2][2]);
	// between +/-pi/2
	attitude[1] = atan( -Cbn[2][0]/sqrt(SQR(Cbn[2][1])+SQR(Cbn[2][2])) );
	// between +/-pi
	attitude[2] = atan2(Cbn[1][0], Cbn[0][0]);
}


void compensate_b_att(FLOAT32* obs, FLOAT32* bias){
	obs[0] -= bias[0];
	obs[1] -= bias[1];
	obs[2] -= bias[2];
}

void Attitude_Mechanization_simple_float_att(FLOAT32 curG[3], FLOAT32 nav_prev_q_bn[4], FLOAT32 nav_cur_q_bn[4])
{

	FLOAT32 beta1[3], q[4];

	beta1[0] = curG[0];
	beta1[1] = curG[1];
	beta1[2] = curG[2];

	Rvec2Quat_att(beta1, q);
	QuatPrdct_att(nav_prev_q_bn, q, nav_cur_q_bn);

	NormQuat_att(nav_cur_q_bn, nav_cur_q_bn);
}


void ChangeArrayMemoryLayout_float_att(UINT8 r, UINT8 c , FLOAT32 **ptr, FLOAT32 *Arr, UINT8 InitToZero)
{
	UINT8 i;
	for(i=0;i < r ; i++)
		ptr[i] = (FLOAT32 *)Arr+i*c;
	if(InitToZero == 1)
		zero_matrix_entries_nxn_float_mis(ptr,r,c);
}

void GetSystemModel_6_att(FLOAT32 Cbn[3][3], FLOAT32 tao[3], FLOAT32 F[NSTATE_ATT][NSTATE_ATT], FLOAT32 G[NSTATE_ATT][6])
{
	UINT16 i, j;
	for(i=0; i<NSTATE_ATT; i++){
		for (j=0; j<NSTATE_ATT; j++){
			F[i][j] = 0.0;
			G[i][j] = 0.0;
		}
	}
	//F[0][1] = winn[2];
	//F[1][0] = -winn[2]; F[1][2] = winn[0];
	//F[2][1] = -winn[0];

	F[0][3] = -Cbn[0][0]; F[0][4] = -Cbn[0][1]; F[0][5] = -Cbn[0][2];
	F[1][3] = -Cbn[1][0]; F[1][4] = -Cbn[1][1]; F[1][5] = -Cbn[1][2];
	F[2][3] = -Cbn[2][0]; F[2][4] = -Cbn[2][1]; F[2][5] = -Cbn[2][2];

	F[3][3] = -1.0/ tao[0];
	F[4][4] = -1.0/ tao[1];
	F[5][5] = -1.0/ tao[2];

	G[0][0] = -Cbn[0][0]; G[0][1] = -Cbn[0][1]; G[0][2] = -Cbn[0][2];
	G[1][0] = -Cbn[1][0]; G[1][1] = -Cbn[1][1]; G[1][2] = -Cbn[1][2];
	G[2][0] = -Cbn[2][0]; G[2][1] = -Cbn[2][1]; G[2][2] = -Cbn[2][2];
	G[3][3] = 1.0;
	G[4][4] = 1.0;
	G[5][5] = 1.0;
}

void multiply_scalar_with_matrix_nxn_att(FLOAT32 Scalar, FLOAT32 **Matrix, FLOAT32 **MatrixR,UINT8 cols, UINT8 rows)
{
	UINT8 i, j;

	for(i=0; i<rows; i++)
		for(j=0; j<cols; j++)
			MatrixR[i][j] = Scalar * Matrix[i][j];
}


void add_matrices_nxn_att(FLOAT32 **Matrix1, FLOAT32 **Matrix2, UINT8 nrow, UINT8 ncol, FLOAT32 **MatrixR)
{
	UINT8 i, j;

	for (i=0; i<nrow; i++)
		for(j=0; j<ncol; j++)
			MatrixR[i][j] = Matrix1[i][j] + Matrix2[i][j];
}

void matrix_transpose_nxn_float_att(FLOAT32 **Matrix, UINT8 m, UINT8 n, FLOAT32 **MatrixT)
{
	UINT8 i, j;

	for(i = 0; i < n ; i++)
		for(j = 0; j < m; j++)
			MatrixT[i][j] = Matrix[j][i];
}

void multiply_matrices_nxn_float_att(FLOAT32 **Matrix1, FLOAT32 **Matrix2, UINT8 ar, UINT8 ac, UINT8 bc, FLOAT32 **MatrixR)
{
	UINT8 i, j,k;
#ifndef COM_WALK_SIMPLIFIED
	FLOAT32 Matrix2T[15][15];
#else
	FLOAT32 Matrix2T[9][9];
#endif
	FLOAT32 sigmax;

	for(i=0;i<bc;i++)
		for(j=0;j<ac;j++)
			Matrix2T[i][j] = Matrix2[j][i];

	for ( i = 0; i < ar; i++ )
		for ( j = 0; j < bc; j++ )
		{
			sigmax = 0.0f;

			for ( k = 0; k < ac; k++ )
				sigmax += Matrix1[i][k] * Matrix2T[j][k];
			MatrixR[i][j] = sigmax;
		}

}

void multiply_matrix_with_vector_nxn_att(FLOAT32 **Matrix, UINT8 ar, UINT8 ac, FLOAT32 *vector, FLOAT32 *vectorR)
{
	UINT8 i, j;
	for (i=0; i<ar; i++)
	{
		vectorR[i] = 0;
		for (j=0; j<ac; j++)
			vectorR[i] = vectorR[i] + Matrix[i][j] * vector[j];
	}
}


void make_skew_symmetric_matrix_att( FLOAT32 vector[3], FLOAT32 Matrix[3][3] )
{
	Matrix[0][0] = 0.0f;
	Matrix[0][1] = -vector[2];
	Matrix[0][2] = vector[1];

	Matrix[1][0] = vector[2];
	Matrix[1][1] = 0.0f;
	Matrix[1][2] = -vector[0];

	Matrix[2][0] = -vector[1];
	Matrix[2][1] = vector[0];
	Matrix[2][2] = 0.0f;
}


void diag2_sq_att(FLOAT32 *A, INT16 r, FLOAT32 **X)
{
	INT16 i, j;
	for (i=0; i<r; i++)
		for (j=0; j<r; j++)
		{
			X[i][j] = 0.0;
			if(i==j) X[i][j]=A[i]*A[i];
		}

	/*for(i = 0; i < r; ++i){
		X[i][i] = A[i];
	}*/
}


STATUS MatrixInv_att(FLOAT32 **A, INT16 n) {

	INT16 i,j;
	INT16 indx[MATRIX_INVERSE_SIZE];
	FLOAT32 *pB[MATRIX_INVERSE_SIZE];
	FLOAT32 C[MATRIX_INVERSE_SIZE];
	FLOAT32 B[MATRIX_INVERSE_SIZE][MATRIX_INVERSE_SIZE];

	for(i=0;i < n; i++)
		pB[i] = (FLOAT32 *)B+i*n;

	CopyMatrix_N_Dimension_att(A,n,n,pB);

	if (lu_decomposition_att(pB,n,indx) == FAILURE) {
		return FAILURE;
	}

	for (j=0;j<n;j++) {
		for (i=0;i<n;i++)
			A[i][j]=0.0f;

		A[j][j]=1.0f;

		for (i=0;i<n;i++)
			C[i] = A[i][j];

		lu_back_subsititution_att(pB,n,indx,C);

		for (i=0;i<n;i++)
			A[i][j] = C[i];


	}
	return SUCCESS;


}


void CopyMatrix_N_Dimension_att(FLOAT32 **Matrix, INT16 m, INT16 n, FLOAT32 **CopyMatrix)
{
	INT16 i, j;

	for(i=0;i<m;i++)
		for(j=0;j<n;j++)
			CopyMatrix[i][j] = Matrix[i][j];
}

STATUS lu_decomposition_att(FLOAT32 **A, INT16 n, INT16 *indx)
{
	FLOAT32 big,s,tmp;
	FLOAT32 vv[24];
	INT16 i,imax=0,j,k;

//    *d=1.0f;
	for (i=0;i<n;i++) {
		big=0.0f; for (j=0;j<n;j++) if ((tmp=fabs(A[i][j]))>big) big=tmp;
		if (big>0.0f)
			vv[i]=1.0f/big;
		else {
			return FAILURE;
		}
	}
	for (j=0;j<n;j++) {
		for (i=0;i<j;i++) {
			s=A[i][j];
			for (k=0;k<i;k++)
				s-=A[i][k]*A[k][j];
			A[i][j]=s;
		}
		big=0.0;
		for (i=j;i<n;i++) {
			s=A[i][j];
			for (k=0;k<j;k++)
				s-=A[i][k]*A[k][j];
			A[i][j]=s;
			if ((tmp=vv[i]*fabs(s))>=big) {
				big=tmp; imax=i;
			}
		}
		if (j!=imax) {
			for (k=0;k<n;k++) {
				tmp=A[imax][k];
				A[imax][k]=A[j][k];
				A[j][k]=tmp;
			}
			//*d=-(*d);
			vv[imax]=vv[j];
		}
		indx[j]=imax;
		if (A[j][j]==0.0f) {
			return FAILURE;
		}
		if (j!=n-1) {
			tmp=1.0f/A[j][j];
			for (i=j+1;i<n;i++)
				A[i][j]*=tmp;
		}
	}
	return SUCCESS;
}

static void lu_back_subsititution_att(FLOAT32 **A, INT16 n, INT16 *indx, FLOAT32 *b) {
	FLOAT32 s;
	INT16 i,ii=-1,ip,j;

	for (i=0;i<n;i++) {
		ip=indx[i]; s=b[ip]; b[ip]=b[i];

		if (ii>=0)
			for (j=ii;j<i;j++)
				s-=A[i][j]*b[j];
		else if (s)
			ii=i;

		b[i]=s;
	}
	for (i=n-1;i>=0;i--) {
		s=b[i];
		for (j=i+1;j<n;j++)
			s-=A[i][j]*b[j];

		b[i]=s/A[i][i];
	}
}

void eye_matrix_float_att(FLOAT32 **A, INT16 n)
{
	UINT8 i,j;
	for (i=0; i<n; i++)
	{
		for (j=0; j<n; j++)
		{
			A[i][j] = 0;
			if (i==j)
			{
				A[i][j] = 1.0;
			}
		}
	}
}

void subtract_matrices_nxn_att(FLOAT32 **Matrix1, FLOAT32 **Matrix2, UINT8 nrow, UINT8 ncol, FLOAT32 **MatrixR)
{
	UINT8 i, j;

	for (i=0; i<nrow; i++)
		for(j=0; j<ncol; j++)
			MatrixR[i][j] = Matrix1[i][j] - Matrix2[i][j];
}



void initialization_misalign_pkt_mis(Misalign_pkt * misa){
	int i, j;
	//memset(&misa->pca,   0, sizeof(PCA  ));
	//memset(&misa->pca_m, 0, sizeof(PCA_M));


	if (1)         // ADD detection of devices later!!
	{
		misa->device_flag = 0;
	}



	// ===================================== PCA
	// Data parameters
	for (i=0; i<3; i++)
	{
		misa->pca.motion_parametrs[i] = 0.0;
		misa->pca.vertical_parametrs[i] = 0.0;
		misa->pca.acceleration_norm_parametrs[i] = 0.0;
		misa->pca.roll_parametrs[i] = 0.0;
		misa->pca.pitch_parametrs[i] = 0.0;
	}
	//Data
	for (i=0; i<WINDOW_SIZE_MIS; i++)
	{
		misa->pca.data_m[i] = 0.0;
		misa->pca.data_v[i] = 0.0;
		misa->pca.data_l[i] = 0.0;
		misa->pca.data_h[i] = 0.0;
		misa->pca.Acc_Mag[i] = 0.0;

		for (j=0; j<3; j++)
		{
			misa->pca.PCA_ACCEL[i][j] = 0.0;
		}
	}
	misa->pca.horizontal_vertical_flag = 0;
	misa->pca.counter = 0;
	misa->pca.flag_run_PCA = 0;

	// ======================================= PCA_M
#ifndef COM_WALK_SIMPLIFIED
	misa->pca_m.motion_effective_coefficient = 0.0;
	misa->pca_m.motion_effective_coefficient_mean = 0.0;
	misa->pca_m.vertical_motion_phase = 0.0;
	misa->pca_m.vertical_motion_phase_mean = 0.0;
#endif
	misa->pca_m.vertical_motion_angle = 0.0;
	misa->pca_m.vertical_motion_phase_shift = 0.0;
	misa->pca_m.vertical_lateral_phase_shift = 0.0;
	misa->pca_m.lateral_range = 0.0;
	misa->pca_m.horizontal_acceleration_range = 0.0;
	misa->pca_m.device_flag = 0;
	misa->pca_m.height_change_flag = 0;


	// ============================================
	misa->horizontal_vertical_flag = 0;
	misa->height_change_flag = 0;

	ChangeArrayMemoryLayout_float_att_float_mis(3, WINDOW_SIZE_MIS, misa->pLev_Acc_Window, (FLOAT32 *)misa->Lev_Acc_Window, 1);

	// ================================================

	for (i=0; i<WINDOW_SIZE_MIS; i++)
	{
		misa->data_r[i] = 0.0;
		misa->data_p[i] = 0.0;
		misa->acc_var_vec[i] = 0.0;
		misa->rad_rot_vec[i] = 0.0;
		misa->gyr_nor_vec[i] = 0.0;
		misa->stp_frq_vec[i] = 0.0;

		for (j=0; j<3; j++)
		{
			misa->memo_Lev_Acc_Data[j][i] = 0.0;
			misa->Lev_Acc_Window[j][i] = 0.0;
		}

		misa->memo_vertical_change[i] = 0.0;
		misa->vertical_motion_phase_vec[i] = 0.0;
		misa->motion_effective_coefficient_vec[i] = 0.0;

		misa->memo_mode_s2[i] = 0.0;
	}

	for (i=0; i<10; i++)
	{
		misa->x_current[i] = 0.0;
		misa->y_current[i] = 0.0;
		for (j=0; j<19; j++)
		{
			misa->x_history_vec[j][i] = 0.0;
			misa->y_history_vec[j][i] = 0.0;
		}
		for (j=0; j<40; j++)
		{
			misa->memo_x_history_data[j][i] = 0.0;
			misa->memo_y_history_data[j][i] = 0.0;
		}
	}
	misa->i_memo = 0;
	misa->i_memo_mode_s2 = 0;
	misa->i_memo_history_data = 0;
	misa->horizontal_vertical_flag = 0;
	misa->height_change_flag = 0;

	misa->dir_s1 = 0;
	misa->step_frequency_data = 0.0;
	misa->vertical_motion_phase_Data = 0.0;
	misa->motion_effective_coefficient_Data;

	misa->Misalign_Az = 0.0;
	misa->Misalign_Az_pre = 0.0;
	misa->Misalign_Az1 = 0.0;

	/*for (i=0; i<50; i++)
	{
	misa->Par_Values[i] = 0.0;
	}*/

	// =================================== Input
	misa->in_cur_Lev_Acc_Data[0] = 0.0;
	misa->in_cur_Lev_Acc_Data[1] = 0.0;
	misa->in_cur_Lev_Acc_Data[2] = 0.0;
	misa->in_cur_pca_data_h = 0.0;
	misa->in_cur_pca_Acc_Mag = 0.0;
	misa->in_cur_Lev_Acc_Window[0] = 0.0;
	misa->in_cur_Lev_Acc_Window[1] = 0.0;
	misa->in_cur_Lev_Acc_Window[2] = 0.0;
	misa->in_cur_RP_Data[0] = 0.0;
	misa->in_cur_RP_Data[1] = 0.0;
	misa->in_cur_Acc_Variance_Data = 0.0;
	misa->in_cur_Rad_Rotation_Data = 0.0;
	misa->in_cur_Gyr_Norm_Data = 0.0;
	misa->in_cur_Step_Frequency_Data = 0.0;
	misa->in_cur_vertical_motion_phase_Data = 0.0;
	misa->in_cur_motion_effective_coefficient_Data = 0.0;
	misa->in_cur_Vertical_Change_Data = 0.0;
	misa->in_cur_Height_Change_Data = 0.0;
}


STATUS KF_update1_float_att(INT16 Num, FLOAT32 *x, FLOAT32 **P, FLOAT32 *inno, FLOAT32 **H, FLOAT32 **R,INT16 NUMStates)
{
	INT16 i;
	STATUS A = FAILURE;
	FLOAT32 Ht[NSTATE_ATT][NSTATE_ATT], PHt[NSTATE_ATT][NSTATE_ATT],HPHt[NSTATE_ATT][NSTATE_ATT], U[NSTATE_ATT][NSTATE_ATT],K[NSTATE_ATT][NSTATE_ATT];
	FLOAT32 *pHt[NSTATE_ATT], *pPHt[NSTATE_ATT],*pHPHt[NSTATE_ATT], *pU[NSTATE_ATT],*pK[NSTATE_ATT];

	FLOAT32 dx[NSTATE_ATT],KH[NSTATE_ATT][NSTATE_ATT],I[NSTATE_ATT][NSTATE_ATT], IKH[NSTATE_ATT][NSTATE_ATT], IKHT[NSTATE_ATT][NSTATE_ATT];
	FLOAT32 *pKH[NSTATE_ATT],*pI[NSTATE_ATT],*pIKH[NSTATE_ATT],*pIKHT[NSTATE_ATT];

	FLOAT32 IKHP[NSTATE_ATT][NSTATE_ATT],IPIT[NSTATE_ATT][NSTATE_ATT],KT[NSTATE_ATT][NSTATE_ATT],KR[NSTATE_ATT][NSTATE_ATT],KRKT[NSTATE_ATT][NSTATE_ATT];
	FLOAT32 *pIKHP[NSTATE_ATT],*pIPIT[NSTATE_ATT],*pKT[NSTATE_ATT],*pKR[NSTATE_ATT],*pKRKT[NSTATE_ATT];


	ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pHt,(FLOAT32 *)Ht,1);
	ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pPHt,(FLOAT32 *)PHt,1);
	ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pHPHt,(FLOAT32 *)HPHt,1);
	ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pU,(FLOAT32 *)U,1);
	ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pK,(FLOAT32 *)K,1);
	ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pKH,(FLOAT32 *)KH,1);
	ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pI,(FLOAT32 *)I,1);
	ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pIKH,(FLOAT32 *)IKH,1);
	ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pIKHT,(FLOAT32 *)IKHT,1);
	ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pIKHP,(FLOAT32 *)IKHP,1);
	ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pIPIT,(FLOAT32 *)IPIT,1);
	ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pKT,(FLOAT32 *)KT,1);
	ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pKR,(FLOAT32 *)KR,1);
	ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pKRKT,(FLOAT32 *)KRKT,1);
	matrix_transpose_nxn_float_att(H, Num, NUMStates, pHt);  // Ht


	multiply_matrices_nxn_float_att(P, pHt, NUMStates, NUMStates, Num, pPHt);  // Pk(-) * Ht
	multiply_matrices_nxn_float_att(H, pPHt, Num, NUMStates, Num, pHPHt);    // H * Pk(-) * Ht


	add_matrices_nxn_att(pHPHt, R, Num, Num, pU);     // U = H * Pk(-) * Ht + R
	//A = InvertMatrix_mis(pU, Num);
	A = MatrixInv_att(pU , Num);
	if (A == SUCCESS)         // U = ( H * Pk(-) * Ht + R )^-1
	{
		multiply_matrices_nxn_float_att(pPHt, pU, NUMStates, Num, Num, pK);  // K = Pk(-) * Ht * ( H * Pk(-) * Ht + R )^-1

		multiply_matrix_with_vector_nxn_att(pK, NUMStates, Num, inno, dx);       // K * (Z - H * xk(-) ) = K *inno

		multiply_matrices_nxn_float_att(pK, H, NUMStates, Num, NUMStates, pKH);       // K * H

		eye_matrix_float_att(pI, NUMStates);                      // unit matrix
		subtract_matrices_nxn_att(pI, pKH, NUMStates, NUMStates, pIKH);        // I - K * H

		for(i=0; i<NUMStates; i++) x[i] += dx[i];
		matrix_transpose_nxn_float_att(pIKH,NUMStates, NUMStates, pIKHT);
		multiply_matrices_nxn_float_att(pIKH, P, NUMStates, NUMStates, NUMStates, pIKHP);  //  (I - K * H) * Pk(-)
		multiply_matrices_nxn_float_att(pIKHP, pIKHT, NUMStates, NUMStates, NUMStates, pIPIT); // (I - K * H) * Pk(-) *(I-K*H)_t


		matrix_transpose_nxn_float_att(pK, NUMStates, Num, pKT);                     // Kt
		multiply_matrices_nxn_float_att(pK, R, NUMStates, Num, Num, pKR);              // K * R
		multiply_matrices_nxn_float_att(pKR, pKT, NUMStates, Num, NUMStates, pKRKT);      // K * R *Kt


		add_matrices_nxn_att(pIPIT, pKRKT, NUMStates, NUMStates, P);     // Pk(+) = (I - K * H) * Pk(-) *(I-K*H)_t + K * R * Kt

	}

	return A;
}

void axis_leveling_att(Att_Pkt * att_pkt)
{
	FLOAT32 Cbn_temp[3][3], *pCbn_temp[3], x456_temp[3], v3_temp[3];
	ChangeArrayMemoryLayout_float_att(3,3,pCbn_temp,(FLOAT32 *)Cbn_temp,0);

	x456_temp[0] = att_pkt->xk[3];
	x456_temp[1] = att_pkt->xk[4];
	x456_temp[2] = att_pkt->xk[5];

	if (IF_AXIS_LEVELING_ATT)
	{
		if (fabs(att_pkt->att[1])>TH_AXIS_LEVELING_ATT)
		{
			if (!att_pkt->if_vertical)
			{
				if (att_pkt->att[1] > 0.0)
				{
					att_pkt->if_vertical = 1;
					multiply_matrices_nxn_float_att(att_pkt->pCbn, att_pkt->pCbb_2, 3, 3, 3, pCbn_temp);
					CopyMatrix_N_Dimension_att(pCbn_temp, 3, 3, att_pkt->pCbn);
					multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_1,3, 3, att_pkt->bg_estimated, v3_temp);
					att_pkt->bg_estimated[0] = v3_temp[0];
					att_pkt->bg_estimated[1] = v3_temp[1];
					att_pkt->bg_estimated[2] = v3_temp[2];
					multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_1,3, 3, x456_temp, v3_temp);
					att_pkt->xk[3] = v3_temp[0];
					att_pkt->xk[4] = v3_temp[1];
					att_pkt->xk[5] = v3_temp[2];
				}
				else if (att_pkt->att[1] < 0.0)
				{
					att_pkt->if_vertical = -1;
					multiply_matrices_nxn_float_att(att_pkt->pCbn, att_pkt->pCbb_1, 3, 3, 3, pCbn_temp);
					CopyMatrix_N_Dimension_att(pCbn_temp, 3, 3, att_pkt->pCbn);
					multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_2,3, 3, att_pkt->bg_estimated, v3_temp);
					att_pkt->bg_estimated[0] = v3_temp[0];
					att_pkt->bg_estimated[1] = v3_temp[1];
					att_pkt->bg_estimated[2] = v3_temp[2];
					multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_2,3, 3, x456_temp, v3_temp);
					att_pkt->xk[3] = v3_temp[0];
					att_pkt->xk[4] = v3_temp[1];
					att_pkt->xk[5] = v3_temp[2];
				}
			}
			else
			{
				if (att_pkt->if_vertical == 1)
				{
					att_pkt->if_vertical = 0;
					multiply_matrices_nxn_float_att(att_pkt->pCbn, att_pkt->pCbb_1, 3, 3, 3, pCbn_temp);
					CopyMatrix_N_Dimension_att(pCbn_temp, 3, 3, att_pkt->pCbn);
					multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_2,3, 3, att_pkt->bg_estimated, v3_temp);
					att_pkt->bg_estimated[0] = v3_temp[0];
					att_pkt->bg_estimated[1] = v3_temp[1];
					att_pkt->bg_estimated[2] = v3_temp[2];
					multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_2,3, 3, x456_temp, v3_temp);
					att_pkt->xk[3] = v3_temp[0];
					att_pkt->xk[4] = v3_temp[1];
					att_pkt->xk[5] = v3_temp[2];
				}
				else if (att_pkt->if_vertical == -1)
				{
					att_pkt->if_vertical = 0;
					multiply_matrices_nxn_float_att(att_pkt->pCbn, att_pkt->pCbb_2, 3, 3, 3, pCbn_temp);
					CopyMatrix_N_Dimension_att(pCbn_temp, 3, 3, att_pkt->pCbn);
					multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_1,3, 3, att_pkt->bg_estimated, v3_temp);
					att_pkt->bg_estimated[0] = v3_temp[0];
					att_pkt->bg_estimated[1] = v3_temp[1];
					att_pkt->bg_estimated[2] = v3_temp[2];
					multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_1,3, 3, x456_temp, v3_temp);
					att_pkt->xk[3] = v3_temp[0];
					att_pkt->xk[4] = v3_temp[1];
					att_pkt->xk[5] = v3_temp[2];
				}
			} // End if (fabs(att_pkt->att[1])>TH_AXIS_LEVELING_ATT)

			Dcm2Quat_att(att_pkt->Cbn, att_pkt->qbn);
			Dcm2Euler_att(att_pkt->Cbn, att_pkt->att);
		}
	}

}

void misalign_estimation_mis(Misalign_pkt *misa)
{
	static UINT8 if_not_first_misa = 0;
	UINT8 i, j, k;
	FLOAT32  temp_vertical= 0.0;
	UINT8 motion_unstability_flag = 0;
	FLOAT32 acc_var = 0;
	FLOAT32 Step_Frequency = 0;				// Step Frequency Data  (Store the output from Classification 2)

	if (!if_not_first_misa)
	{
		if_not_first_misa = 1;
		// Initialization
		initialization_misalign_pkt_mis(misa);   // To be completed
	}

	for(j=0; j<3; j++)
	{
		misa->in_cur_Lev_Acc_Window[j] = misa->in_cur_Lev_Acc_Data[j];
		if(j == 2)
			misa->in_cur_Lev_Acc_Window[j] += Constant_Gravity; // Remove Gravity from the vertical axis
		if(j != 2)
			misa->in_cur_pca_data_h += (FLOAT32)(SQR(misa->in_cur_Lev_Acc_Data[j]));	// Horizontal Data
		misa->in_cur_pca_Acc_Mag += (FLOAT32)(SQR(misa->in_cur_Lev_Acc_Data[j]));		// Acceleration magnitude
	}
	misa->in_cur_pca_data_h = sqrt(misa->in_cur_pca_data_h);			// Horizontal magnitude
	misa->in_cur_pca_Acc_Mag = sqrt(misa->in_cur_pca_Acc_Mag);		// Acceleration magnitude


	// Put into memo
	misa->i_memo ++;
	if (misa->i_memo <= WINDOW_SIZE_MIS)
	{

		misa->pca.data_h[misa->i_memo-1] = misa->in_cur_pca_data_h;
		misa->pca.Acc_Mag[misa->i_memo-1] = misa->in_cur_pca_Acc_Mag;

		misa->data_r[misa->i_memo-1] = misa->in_cur_RP_Data[0];
		misa->data_p[misa->i_memo-1] = misa->in_cur_RP_Data[1];
		misa->acc_var_vec[misa->i_memo-1] = misa->in_cur_Acc_Variance_Data;
		misa->rad_rot_vec[misa->i_memo-1] = misa->in_cur_Rad_Rotation_Data;
		misa->gyr_nor_vec[misa->i_memo-1] = misa->in_cur_Gyr_Norm_Data;
		misa->stp_frq_vec[misa->i_memo-1] = misa->in_cur_Step_Frequency_Data;
		misa->vertical_motion_phase_vec[misa->i_memo-1] = misa->in_cur_vertical_motion_phase_Data;
		misa->motion_effective_coefficient_vec[misa->i_memo-1] = misa->in_cur_motion_effective_coefficient_Data;

		misa->memo_vertical_change[misa->i_memo-1] = misa->in_cur_Vertical_Change_Data;

		misa->Lev_Acc_Window[0][misa->i_memo-1] = misa->in_cur_Lev_Acc_Window[0];
		misa->Lev_Acc_Window[1][misa->i_memo-1] = misa->in_cur_Lev_Acc_Window[1];
		misa->Lev_Acc_Window[2][misa->i_memo-1] = misa->in_cur_Lev_Acc_Window[2];


		misa->memo_Lev_Acc_Data[0][misa->i_memo-1] = misa->in_cur_Lev_Acc_Data[0];
		misa->memo_Lev_Acc_Data[1][misa->i_memo-1] = misa->in_cur_Lev_Acc_Data[1];
		misa->memo_Lev_Acc_Data[2][misa->i_memo-1] = misa->in_cur_Lev_Acc_Data[2];

	}
	else
	{
		misa->i_memo = WINDOW_SIZE_MIS;
		for (k=0; k<WINDOW_SIZE_MIS-1; k++)
		{

			misa->pca.data_h[k] = misa->pca.data_h[k+1];
			misa->pca.Acc_Mag[k] = misa->pca.Acc_Mag[k+1];

			misa->data_r[k] = misa->data_r[k+1];
			misa->data_p[k] = misa->data_p[k+1];

			misa->acc_var_vec[k] = misa->acc_var_vec[k+1];
			misa->rad_rot_vec[k] = misa->rad_rot_vec[k+1];
			misa->gyr_nor_vec[k] = misa->gyr_nor_vec[k+1];
			misa->stp_frq_vec[k] = misa->stp_frq_vec[k+1];
			misa->vertical_motion_phase_vec[k] = misa->vertical_motion_phase_vec[k+1];
			misa->motion_effective_coefficient_vec[k] = misa->motion_effective_coefficient_vec[k+1];
			misa->memo_vertical_change[k] = misa->memo_vertical_change[k+1];

			misa->Lev_Acc_Window[0][k] = misa->Lev_Acc_Window[0][k+1];
			misa->Lev_Acc_Window[1][k] = misa->Lev_Acc_Window[1][k+1];
			misa->Lev_Acc_Window[2][k] = misa->Lev_Acc_Window[2][k+1];


			misa->memo_Lev_Acc_Data[0][k] = misa->memo_Lev_Acc_Data[0][k+1];
			misa->memo_Lev_Acc_Data[1][k] = misa->memo_Lev_Acc_Data[1][k+1];
			misa->memo_Lev_Acc_Data[2][k] = misa->memo_Lev_Acc_Data[2][k+1];


		}

			misa->pca.data_h[WINDOW_SIZE_MIS-1] = misa->in_cur_pca_data_h;
			misa->pca.Acc_Mag[WINDOW_SIZE_MIS-1] = misa->in_cur_pca_Acc_Mag;

			misa->data_r[WINDOW_SIZE_MIS-1] = misa->in_cur_RP_Data[0];
			misa->data_p[WINDOW_SIZE_MIS-1] = misa->in_cur_RP_Data[1];
			misa->acc_var_vec[WINDOW_SIZE_MIS-1] = misa->in_cur_Acc_Variance_Data;
			misa->rad_rot_vec[WINDOW_SIZE_MIS-1] = misa->in_cur_Rad_Rotation_Data;
			misa->gyr_nor_vec[WINDOW_SIZE_MIS-1] = misa->in_cur_Gyr_Norm_Data;
			misa->stp_frq_vec[WINDOW_SIZE_MIS-1] = misa->in_cur_Step_Frequency_Data;
			misa->vertical_motion_phase_vec[WINDOW_SIZE_MIS-1] = misa->in_cur_vertical_motion_phase_Data;
			misa->motion_effective_coefficient_vec[WINDOW_SIZE_MIS-1] = misa->in_cur_motion_effective_coefficient_Data;
			misa->memo_vertical_change[WINDOW_SIZE_MIS-1] = misa->in_cur_Vertical_Change_Data;

			misa->Lev_Acc_Window[0][WINDOW_SIZE_MIS-1] = misa->in_cur_Lev_Acc_Window[0];
			misa->Lev_Acc_Window[1][WINDOW_SIZE_MIS-1] = misa->in_cur_Lev_Acc_Window[1];
			misa->Lev_Acc_Window[2][WINDOW_SIZE_MIS-1] = misa->in_cur_Lev_Acc_Window[2];

			misa->memo_Lev_Acc_Data[0][WINDOW_SIZE_MIS-1] = misa->in_cur_Lev_Acc_Data[0];
			misa->memo_Lev_Acc_Data[1][WINDOW_SIZE_MIS-1] = misa->in_cur_Lev_Acc_Data[1];
			misa->memo_Lev_Acc_Data[2][WINDOW_SIZE_MIS-1] = misa->in_cur_Lev_Acc_Data[2];

	}


	if(misa->i_memo >= WINDOW_SIZE_MIS-1)
	{

		//misa->start_1 = mm * OVERLAP_MISA_MIS - WINDOW_SIZE_MIS+1;            // Window starting index

		// Average of flag vertical
		temp_vertical = 0.0;
		for(i = 0; i < WINDOW_SIZE_MIS; i++)
		{
			temp_vertical += misa->memo_vertical_change[i];
		}
		temp_vertical /= WINDOW_SIZE_MIS;



		if(fabs(temp_vertical) > 0.5)
		{
			if(temp_vertical > 0)
				misa->horizontal_vertical_flag =  1;
			else
				misa->horizontal_vertical_flag = -1;
		}
		else
		{
			misa->horizontal_vertical_flag = 0;
		}


		if(misa->in_cur_Height_Change_Data >= 1)
			misa->height_change_flag = 1;
		else
			misa->height_change_flag = 0;



		// Vertical data
		for(i = 0; i < WINDOW_SIZE_MIS; i++)
		{
			misa->pca.data_v[i] = misa->Lev_Acc_Window[2][i];					// Vertical Data
		}



		LowPassFilter2_mis(misa->pca.data_v,  FC_LP_FILTER_MIS, FS_LP_FILTER_MIS, FILTER_ORDER_MIS, misa->pca.data_v,  WINDOW_SIZE_MIS);
		for(i = WINDOW_SIZE_MIS-1; i > 0 ; i--)
			misa->pca.data_v[i]  = misa->pca.data_v[i-1];

		// Walking Frequency
		for(i = DATA_SIZE_MIS-1; i > 0 ; i--)
			misa->stp_frq_vec[i]  = misa->stp_frq_vec[i-1];   // Until here its all zeros LY

#ifdef PRINT_RESULTS_MISALIGNMENT
		motion_unstability_flag = peak_frequency_calculation_mis(&misa->pca, misa->acc_var_vec, &acc_var, misa->stp_frq_vec, &Step_Frequency, misa->Par_Values);
#else
		motion_unstability_flag = peak_frequency_calculation_mis(&misa->pca, misa->acc_var_vec, &acc_var, misa->stp_frq_vec, &Step_Frequency);
#endif
		misa->step_frequency_data	= Step_Frequency;


		cal_misalign_1_eph_mis(misa);

	}
	else  // if(mm < WINDOW_SIZE_MIS-1)
	{
		//for(i=0; i<50; i++)
		//{
		//	misa->Par_Values[i] = 0.0;
		//}
	}

}

void cal_misalign_1_eph_mis(Misalign_pkt *misa)
{
	int i,j;
	UINT8 count_general = 0, count_pocket = 0, count_dangling = 0;
	FLOAT32 Step_Frequency = 0;				// Step Frequency Data  (Store the output from Classification 2)
	FLOAT32 acc_var = 0;
	UINT8 motion_unstability_flag = 0;
	UINT8 mode2_fft_mis = 0;
	FLOAT32 mean_x, mean_y, xx_sum, yy_sum, xy_sum, V[2][2];
	FLOAT32 horizontal_acceleration_range = 0;				// Horizontal range
	FLOAT32 lateral_range = 0;				// Lateral range
	FLOAT32 phase_shift_ML = 0.0;		// Angle between Motion and Lateral
	FLOAT32 R[2][2];			// Misalignment based rotation matrix
	FLOAT32 maxmin[2] = {0.0};
	FLOAT32 range_Acc_Mag = 0.0;
	FLOAT32 static_motion_flag = 0;
	//INT8  device_flag = 0;					// To tell the device type(Phone - Tablet, Watch, ...)
	UINT8 mode1 = 0;						// Mode 1 selection (others = 0, pocket =1, dangling = 2)
	UINT8 mode2 = 0;						// Mode 2 selection (others = 0, pocket =1, dangling = 2)

#ifdef PRINT_RESULTS_MISALIGNMENT
	motion_unstability_flag = peak_frequency_calculation_mis(&misa->pca, misa->acc_var_vec, &acc_var, misa->stp_frq_vec, &Step_Frequency, misa->Par_Values);
#else
	motion_unstability_flag = peak_frequency_calculation_mis(&misa->pca, misa->acc_var_vec, &acc_var, misa->stp_frq_vec, &Step_Frequency);
#endif
	// Step_Frequency_Data[mm]	= Step_Frequency;
	misa->step_frequency_data = Step_Frequency;

	// FFT
#ifndef COM_WALK_SIMPLIFIED
	//if(device_flag != 3)
	//{
	count_general = 0;
	count_pocket = 0;
	count_dangling = 0;
	// Majority code for device use cases mode 2

	if (misa->i_memo_mode_s2 == 40)
	{
		for (i=39; i>=0; i--)
		{
			if(misa->memo_mode_s2[i] == 0)
				count_general++;
			else if(misa->memo_mode_s2[i] == 1)
				count_pocket++;
			else if(misa->memo_mode_s2[i] == 2)
				count_dangling++;
		}
	}


	if(count_dangling >= count_general && count_dangling >= count_pocket)
		mode2_fft_mis = 2;
	else if(count_pocket >= count_general && count_pocket >= count_dangling)
		mode2_fft_mis = 1;
	else if(count_general >= count_pocket && count_general >= count_dangling)
		mode2_fft_mis = 0;
	else
		mode2_fft_mis = 0;

	/*for(i = 0; i < 19; i++)
	{
		for(j = 0; j < 10; j++)
		{
			misa->x_history_vec[i][j] = misa->x_history_data[i+misa->start_1+20][j];
			misa->y_history_vec[i][j] = misa->y_history_data[i+misa->start_1+20][j];
		}
	}*/

	for(i = 0; i < 19; i++)
	{
		for(j = 0; j < 10; j++)
		{
			misa->x_history_vec[i][j] = misa->memo_x_history_data[i+20][j];
			misa->y_history_vec[i][j] = misa->memo_y_history_data[i+20][j];
		}
	}


	fft_mis_calculation_mis_constSize(misa->pLev_Acc_Window, misa->data_r, misa->data_p, acc_var, misa->device_flag, motion_unstability_flag, misa->x_history_vec, misa->x_current, misa->y_history_vec, misa->y_current, mode2_fft_mis);
	/*for(j = 0; j < 10; j++)
	{
		misa->x_history_data[mm][j] =  misa->x_current[j];
		misa->y_history_data[mm][j] =  misa->y_current[j];
	}*/

	misa->i_memo_history_data ++;
	for (j = 0; j < 10; j++)
	{
		if (misa->i_memo_history_data <= 40)
		{
			misa->memo_x_history_data[misa->i_memo_history_data-1][j] = misa->x_current[j];
			misa->memo_y_history_data[misa->i_memo_history_data-1][j] = misa->y_current[j];
		}
		else
		{
			for (i = 0; i < 39; i++)
			{
				misa->memo_x_history_data[i][j] = misa->memo_x_history_data[i+1][j];
				misa->memo_y_history_data[i][j] = misa->memo_y_history_data[i+1][j];
			}
			misa->memo_x_history_data[39][j] = misa->x_current[j];
			misa->memo_y_history_data[39][j] = misa->y_current[j];
		}
	}

	//}
#endif
	// Calculate phase one angle
	V[0][0] = 0.0;
	V[0][1] = 0.0;
	V[1][0] = 0.0;
	V[1][1] = 0.0;
	mean_x = mean_y = 0;
	for(i = 0; i < WINDOW_SIZE_MIS; i++)
	{
		mean_x += misa->Lev_Acc_Window[0][i];
		mean_y += misa->Lev_Acc_Window[1][i];
	}
	mean_x *= ONE_BY_WINDOW_SIZE_MIS_MEAN;
	mean_y *= ONE_BY_WINDOW_SIZE_MIS_MEAN;

	xx_sum = 0.0f; yy_sum = 0.0f; xy_sum = 0.0f;
	for(i = 0; i < WINDOW_SIZE_MIS; i++)
	{
		xx_sum +=  (misa->Lev_Acc_Window[0][i] - mean_x)*(misa->Lev_Acc_Window[0][i] - mean_x);
		xy_sum +=  (misa->Lev_Acc_Window[0][i] - mean_x)*(misa->Lev_Acc_Window[1][i] - mean_y);
		yy_sum +=  (misa->Lev_Acc_Window[1][i] - mean_y)*(misa->Lev_Acc_Window[1][i] - mean_y);
	}

	// Method 1: PCA misalignment
	/*check_i = Calculate_PCA_float_float_mis(pLev_Acc_Window, WINDOW_SIZE_MIS, pU, V, 0.000001, mean_x, mean_y); //princomp(Lev_Acc_Window,eigenvalue);
	if(check_i < 0)
	{
	cout <<"eigen value failed\n";
	return 0;
	}
	misa->Misalign_Az[mm] = (FLOAT32)(atan2(-V[1][0], V[0][0]));*/

	// Method 2: PCA misalignment based on 2x2 SVD
	/*check_i = Calculate_PCA_float_float_2_mis(pLev_Acc_Window, WINDOW_SIZE_MIS, pU, V, 0.000001, mean_x, mean_y); //princomp(Lev_Acc_Window,eigenvalue);
	if(check_i < 0)
	{
	cout <<"eigen value failed\n";
	return 0;
	}
	misa->Misalign_Az[mm] = (FLOAT32)(atan2(-V[1][0], V[0][0]));*/

	// Method 3: Iterative method for misalignment angle calculation
	/*misa->Misalign_Az[mm] = calculate_phase_one_angle_mis(pLev_Acc_Window, mean_x, mean_y);*/

	// Refill the accelerometer data to avoid any change in the vector after PCA
	/*for(i=0;i<WINDOW_SIZE_MIS;i++)
	{
	for(j=0;j<3;j++)
	{
	Lev_Acc_Window[i][j] = Lev_Acc_Data[i+start_1][j];
	if(j == 2)
	Lev_Acc_Window[i][j] += Constant_Gravity;
	}
	}*/

	// Method 4: analytical method for misalignment angle calculation
	misa->Misalign_Az_pre = misa->Misalign_Az;
	misa->Misalign_Az = calculate_phase_one_angle_mis_analytical(xx_sum, xy_sum, yy_sum, 1);

	misa->Misalign_Az = misa->Misalign_Az-(misa->Misalign_Az > PIf/4 ? PIf:0);
	misa->Misalign_Az = misa->Misalign_Az-(misa->Misalign_Az < -3*PIf/4 ? -PIf:0);
	misa->Misalign_Az1    = misa->Misalign_Az;
	//VVDisplay(&misa->Misalign_Az1,1);

	// Refill the accelerometer data to avoid any change in the vector after PCA
	for(i=0;i<WINDOW_SIZE_MIS;i++)
	{
		for(j=0;j<3;j++)
		{
			//misa->Lev_Acc_Window[j][i] = Lev_Acc_Data[j][i+misa->start_1];
			misa->Lev_Acc_Window[j][i] = misa->memo_Lev_Acc_Data[j][i];
			if(j == 2)
				misa->Lev_Acc_Window[j][i] += Constant_Gravity;
		}
	}

	R[0][0] =  cos(misa->Misalign_Az);
	R[0][1] = -sin(misa->Misalign_Az);
	R[1][0] =  sin(misa->Misalign_Az);
	R[1][1] =  cos(misa->Misalign_Az);

	for(i = 0; i < WINDOW_SIZE_MIS; i++)
	{
		misa->pca.data_m[i] = R[0][0]*misa->Lev_Acc_Window[0][i] + R[0][1]*misa->Lev_Acc_Window[1][i];		// Motion data
		misa->pca.data_l[i] = R[1][0]*misa->Lev_Acc_Window[0][i] + R[1][1]*misa->Lev_Acc_Window[1][i];		// Lateral data
	}

	//// Phase Shift, December 2013
	//phase_shift_ML       = Phase_Shift_Angle_mis_float_float_mis(pca.data_m, pca.data_l, WINDOW_SIZE_MIS);
	//misa->pca_m.vertical_motion_phase_shift  = Phase_Shift_Angle_mis_float_float_mis(data_v1,  pca.data_m, WINDOW_SIZE_MIS)*R2Df;
	//misa->pca_m.vertical_lateral_phase_shift = Phase_Shift_Angle_mis_float_float_mis(data_v1, pca.data_l, WINDOW_SIZE_MIS)*R2Df;

	// LPF
	LowPassFilter2_mis(misa->pca.data_m,  FC_LP_FILTER_MIS, FS_LP_FILTER_MIS, FILTER_ORDER_MIS, misa->pca.data_m,  WINDOW_SIZE_MIS);
	LowPassFilter2_mis(misa->pca.data_l,  FC_LP_FILTER_MIS, FS_LP_FILTER_MIS, FILTER_ORDER_MIS, misa->pca.data_l,  WINDOW_SIZE_MIS);
	LowPassFilter2_mis(misa->pca.data_h,  FC_LP_FILTER_MIS, FS_LP_FILTER_MIS, FILTER_ORDER_MIS, misa->pca.data_h,  WINDOW_SIZE_MIS);
	LowPassFilter2_mis(misa->pca.Acc_Mag, FC_LP_FILTER_MIS, FS_LP_FILTER_MIS, FILTER_ORDER_MIS, misa->pca.Acc_Mag, WINDOW_SIZE_MIS);

	// Fill PCA
	for(i = WINDOW_SIZE_MIS-1; i > 0 ; i--)
	{
		misa->pca.data_m[i]  = misa->pca.data_m[i-1];
		misa->pca.data_l[i]  = misa->pca.data_l[i-1];
		misa->pca.data_h[i]  = misa->pca.data_h[i-1];
		misa->pca.Acc_Mag[i] = misa->pca.Acc_Mag[i-1];
	}

	misa->pca.horizontal_vertical_flag = misa->horizontal_vertical_flag;

	// Phase Shift, December 2013
	phase_shift_ML                     = Phase_Shift_Angle_mis_float_float_mis(misa->pca.data_m, misa->pca.data_l, DATA_SIZE_MIS);
	misa->pca_m.vertical_motion_phase_shift  = Phase_Shift_Angle_mis_float_float_mis(misa->pca.data_v,  misa->pca.data_m, DATA_SIZE_MIS)*R2Df;
	misa->pca_m.vertical_lateral_phase_shift = Phase_Shift_Angle_mis_float_float_mis(misa->pca.data_v, misa->pca.data_l, DATA_SIZE_MIS)*R2Df;

	find_max_min_float_mis(misa->pca.data_h, maxmin, DATA_SIZE_MIS);
	horizontal_acceleration_range = maxmin[0]-maxmin[1];
	find_max_min_float_mis(misa->pca.data_l, maxmin, DATA_SIZE_MIS);
	lateral_range =  maxmin[0]-maxmin[1];
	//////////     Sign Method Added  March 31th, 2014 //////////////////////

#ifndef COM_WALK_SIMPLIFIED
	for(i = DATA_SIZE_MIS-1; i > 0 ; i--)
	{
		misa->motion_effective_coefficient_vec[i] = misa->motion_effective_coefficient_vec[i-1];
		misa->vertical_motion_phase_vec[i] = misa->vertical_motion_phase_vec[i-1];
	}
	sign_method_parameters_calculation_mis(&misa->pca_m, &misa->pca, misa->motion_effective_coefficient_vec, misa->vertical_motion_phase_vec);
	misa->vertical_motion_phase_Data = misa->pca_m.vertical_motion_phase;
	misa->motion_effective_coefficient_Data = misa->pca_m.motion_effective_coefficient;
#else
	sign_method_parameters_calculation_mis(&misa->pca_m, &misa->pca);
#endif

	// Fill PCA_M
	misa->pca_m.height_change_flag = misa->height_change_flag;
	misa->pca_m.device_flag = misa->device_flag;
	misa->pca_m.horizontal_acceleration_range = horizontal_acceleration_range;
	misa->pca_m.lateral_range = lateral_range;

	// Motion Classification
#ifdef PRINT_RESULTS_MISALIGNMENT
	device_use_cases_classification_mis(misa->data_r, misa->data_p, &misa->pca, acc_var, misa->rad_rot_vec, misa->gyr_nor_vec, motion_unstability_flag, misa->device_flag, &mode1, &mode2, misa->Par_Values);
#else
	device_use_cases_classification_mis(misa->data_r, misa->data_p, &misa->pca, acc_var, misa->rad_rot_vec, misa->gyr_nor_vec, motion_unstability_flag, misa->device_flag, &mode1, &mode2);
#endif
	//misa->mode_s1[mm] = mode1;
	//misa->mode_s2[mm] = mode2;

	misa->i_memo_mode_s2 ++;
	if (misa->i_memo_mode_s2 <= 40)
	{
		misa->memo_mode_s2[misa->i_memo_mode_s2-1] = mode2;
	}
	else
	{
		misa->i_memo_mode_s2 = 40;
		for (i=0; i<39; i++)
		{
			misa->memo_mode_s2[i] = misa->memo_mode_s2[i+1];
		}
		misa->memo_mode_s2[39] = mode2;
	}

	// 180 ambiguity
	if(mode1 == 1 && misa->device_flag != 3)			// Pocket
		misa->dir_s1 = pocket_backward_forward_mis(misa->data_r, misa->data_p, &misa->pca, &misa->pca_m);
	else if(mode1 == 2 || (mode1 == 1 && misa->device_flag == 3))		// Dangling
		misa->dir_s1 = dangling_backward_forward_mis(&misa->pca, &misa->pca_m);
	else							// General
		misa->dir_s1 = general_backward_forward_mis(misa->data_r, misa->data_p, &misa->pca, &misa->pca_m, misa->Misalign_Az);

	if(misa->dir_s1 == 1)				// 180 disambiguity detected
		misa->Misalign_Az += PIf;
	if(misa->dir_s1 == 2)				// Undetected (keep the previous value)
	{
		//if (mm>0)
		//{
			misa->Misalign_Az = misa->Misalign_Az_pre;
		//}

	}

	find_max_min_float_mis(misa->pca.Acc_Mag, maxmin, DATA_SIZE_MIS);
	range_Acc_Mag =  maxmin[0]-maxmin[1];
	if(range_Acc_Mag > 12)
		static_motion_flag = 1;
	else
		static_motion_flag	= 0;

	// Print the output to the file
	//misa->misalign_az = misa->Misalign_Az;	 // Phase_II Misalignment		(4)
	misa->motion_mode = mode1;


//	misa->Par_Values[0] = V[0][0];									// coefs(1,1)				(1)
//	misa->Par_Values[1] = V[1][0];									// coefs(2,1)				(2)
//	misa->Par_Values[2] = misa->Misalign_Az1;								// Phase_I Misalignment		(3)
//	misa->Par_Values[3] = misa->Misalign_Az;							// Phase_II Misalignmen		(4)
//	misa->Par_Values[4] = (FLOAT32)misa->dir_s1;							// For/Back/Undetect		(5)
//	misa->Par_Values[5] = mode1; //(FLOAT32)misa->mode_s1[mm];						// mode1 Classification		(6)
//	misa->Par_Values[6] = mode2; //(FLOAT32)misa->mode_s2[mm];						// mode2 Classification	 	(7)
//	misa->Par_Values[7] = phase_shift_ML;								// ML phase shift			(8)
//	misa->Par_Values[8] = misa->pca_m.vertical_motion_phase_shift*D2Rf;		// VM phase shift			(9)
//	misa->Par_Values[9] = misa->pca_m.vertical_lateral_phase_shift*D2Rf;	// VL phase shift			(10)
//
//#ifndef COM_WALK_SIMPLIFIED
//	misa->Par_Values[45] = misa->pca_m.motion_effective_coefficient;		// Sign						(46)
//	misa->Par_Values[46] = misa->pca_m.vertical_motion_phase;				// Phase shift				(47)
//#else
//	misa->Par_Values[45] = 0;											// Sign						(46)
//	misa->Par_Values[46] = 0;											// Phase shift				(47)
//#endif
//	misa->Par_Values[47] = misa->pca_m.vertical_motion_angle;				// Phase Angle				(48)
//	misa->Par_Values[48] = static_motion_flag;						// Static Motion Flag		(49)
//	misa->Par_Values[49] = 0; //My_Method_Num;								// Method Number			(50)


}


void play_with_misalign(Misalign_pkt *misa, Att_Pkt * att_pkt, MemoIndoor *memo)
{
	int k;
	FLOAT32 misalign_135, temp_misalign2;
	FLOAT32 temp_misalign = 0.0;

	// Divide Ear from General
	if (misa->motion_mode == 0)
	{
		if (att_pkt->if_vertical == 1 && att_pkt->att[0] > TH_ROLL_EAR)
		//if ((att_pkt->if_vertical == 1 && fabs(att_pkt->att[0]) > 45*D2R) || (fabs(att_pkt->att[0]) > 70 * D2R ))
		//if ((att_pkt->if_vertical == 1 && att_pkt->att[0] > TH_ROLL_EAR)) // || (att_pkt->att[0] > 60 * D2R && att_pkt->att[1] > 15 * D2R ))
		{
			misa->motion_mode = 4;  // Ear
		}
	}

	// Change mis to -225 ~ 135 deg
	if (misa->Misalign_Az > ANG_FOR_SM_MISA_135)
	{
		misalign_135 = misa->Misalign_Az - 2*PI;
	}
	else
	{
		misalign_135 = misa->Misalign_Az;
	}

	// Put into memory
	memo->i_misa ++;
	if (memo->i_misa <= N_SM_MISALIGN)
	{
		memo->misalign[memo->i_misa-1] = misalign_135;
	}
	else
	{
		memo->i_misa = N_SM_MISALIGN;
		for (k=0; k<N_SM_MISALIGN-1; k++)
		{
			memo->misalign[k] = memo->misalign[k+1];
		}
		memo->misalign[N_SM_MISALIGN-1] = misalign_135;
	}

	// Calculate mean of misalignment
	temp_misalign = cal_mean_float(memo->misalign, memo->i_misa);

	// Calculate misalignemnt_used
	//temp_misalign2 = misa->Misalign_Az;
	temp_misalign2 = 0.0;

	// ------------------------------ For Show ------------------------------
	if (misa->motion_mode == 0)  // Handheld
	{
		if (fabs(temp_misalign-ANG_0_RAD) < TH_DIFF_MIS_HANDHELD)
		{
			temp_misalign2 = ANG_0_RAD;
		} else if (fabs(temp_misalign-ANG_90_RAD) < TH_DIFF_MIS_HANDHELD)
		{
			temp_misalign2 = ANG_90_RAD;
		}else if (fabs(temp_misalign-ANG_NEG_90_RAD) < TH_DIFF_MIS_HANDHELD)
		{
			temp_misalign2 = ANG_NEG_90_RAD;
		} else if (fabs(temp_misalign-ANG_180_RAD) < TH_DIFF_MIS_HANDHELD)
		{
			temp_misalign2 = ANG_180_RAD;
		}else if (fabs(temp_misalign-ANG_NEG_180_RAD) < TH_DIFF_MIS_HANDHELD)
		{
			temp_misalign2 = ANG_180_RAD;
		}
	}
	else if (misa->motion_mode == 4) // Ear
	{
		temp_misalign2 = ANG_180_RAD;
	}
	else if (misa->motion_mode == 1) // Pocket
	{
		if (att_pkt->if_vertical == 0)
		{
			if (fabs(temp_misalign-ANG_60_RAD) < TH_DIFF_MIS_HANDHELD)
			{
				temp_misalign2 = ANG_60_RAD;
			} else if (fabs(temp_misalign-ANG_120_RAD) < TH_DIFF_MIS_HANDHELD)
			{
				temp_misalign2 = ANG_120_RAD;
			}else if (fabs(temp_misalign-ANG_NEG_60_RAD) < TH_DIFF_MIS_HANDHELD)
			{
				temp_misalign2 = ANG_NEG_60_RAD;
			} else if (fabs(temp_misalign-ANG_NEG_120_RAD) < TH_DIFF_MIS_HANDHELD)
			{
				temp_misalign2 = ANG_NEG_120_RAD;
			}else
			{
				temp_misalign2 = 0.0;
			}
		}
		else   // if_vertical != 0
		{
			if (fabs(temp_misalign-ANG_180_RAD) < TH_DIFF_MIS_HANDHELD)
			{
				temp_misalign2 = ANG_180_RAD;
			} else if (fabs(temp_misalign-ANG_NEG_180_RAD) < TH_DIFF_MIS_HANDHELD)
			{
				temp_misalign2 = ANG_180_RAD;
			} else
			{
				temp_misalign2 = 0.0;
			}
		}
	}
	else if (misa->motion_mode == 2)  // Dangling
	{
		temp_misalign2 = 0.0;
	}
	else if (misa->motion_mode == 3)  // Others
	{
		temp_misalign2 = 0.0;
	}

	//misa->Misalign_Az_For_Use = temp_misalign2;
	misa->Misalign_Az_For_Show = temp_misalign2;

	temp_misalign2 = 0.0;
	// ------------------------------ For Use ------------------------------
	if (misa->motion_mode == 0 && fabs(att_pkt->att[0]) < 30* D2R && fabs(att_pkt->att[1]) < 40* D2R)  // Handheld
	{
		if (fabs(temp_misalign-ANG_0_RAD) < TH_DIFF_MIS_HANDHELD)
		{
			temp_misalign2 = ANG_0_RAD;
		} else if (fabs(temp_misalign-ANG_90_RAD) < TH_DIFF_MIS_HANDHELD)
		{
			temp_misalign2 = ANG_90_RAD;
		}else if (fabs(temp_misalign-ANG_NEG_90_RAD) < TH_DIFF_MIS_HANDHELD)
		{
			temp_misalign2 = ANG_NEG_90_RAD;
		} else if (fabs(temp_misalign-ANG_180_RAD) < TH_DIFF_MIS_HANDHELD)
		{
			temp_misalign2 = ANG_180_RAD;
		}else if (fabs(temp_misalign-ANG_NEG_180_RAD) < TH_DIFF_MIS_HANDHELD)
		{
			temp_misalign2 = ANG_180_RAD;
		}
	}
	else if (misa->motion_mode == 4) // Ear
	{
		temp_misalign2 = ANG_180_RAD;
	}
	else if (misa->motion_mode == 1) // Pocket
	{
		if (att_pkt->if_vertical == 0)
		{
			temp_misalign2 = 0.0;
			/*if (fabs(temp_misalign-ANG_60_RAD) < TH_DIFF_MIS_HANDHELD)
			{
				temp_misalign2 = ANG_60_RAD;
			} else if (fabs(temp_misalign-ANG_120_RAD) < TH_DIFF_MIS_HANDHELD)
			{
				temp_misalign2 = ANG_120_RAD;
			}else if (fabs(temp_misalign-ANG_NEG_60_RAD) < TH_DIFF_MIS_HANDHELD)
			{
				temp_misalign2 = ANG_NEG_60_RAD;
			} else if (fabs(temp_misalign-ANG_NEG_120_RAD) < TH_DIFF_MIS_HANDHELD)
			{
				temp_misalign2 = ANG_NEG_120_RAD;
			}else
			{
				temp_misalign2 = 0.0;
			} */
		}
		else   // if_vertical != 0
		{
			if (fabs(temp_misalign-ANG_180_RAD) < TH_DIFF_MIS_HANDHELD)
			{
				temp_misalign2 = ANG_180_RAD;
			} else if (fabs(temp_misalign-ANG_NEG_180_RAD) < TH_DIFF_MIS_HANDHELD)
			{
				temp_misalign2 = ANG_180_RAD;
			} else
			{
				temp_misalign2 = 0.0;
			}
		}
	}
	else if (misa->motion_mode == 2)  // Dangling
	{
		temp_misalign2 = 0.0;
	}
	else if (misa->motion_mode == 3)  // Others
	{
		temp_misalign2 = 0.0;
	}

	misa->Misalign_Az_For_Use = temp_misalign2;
}

//void INSNavFeedBack_Attitude_float_att( FLOAT32 x[NSTATE_ATT], FLOAT32 q_bn[4], FLOAT32 C_bn[3][3], FLOAT32 att[3], FLOAT32 EstimatedGyroBias[3])
void INSNavFeedBack_Attitude_float_att(Att_Pkt * att_pkt, FLOAT32 f_b[3])
{

	FLOAT32 phi_ang[3], qe[4], qbn[4];
	INT16 i;
	/*FLOAT32 Cbn_temp[3][3], *pCbn_temp[3], x456_temp[3];
	ChangeArrayMemoryLayout_float_att(3,3,pCbn_temp,(FLOAT32 *)Cbn_temp,0);*/

	// ===== attitude feedback
	for(i=0; i<3; i++)
		phi_ang[i] = att_pkt->xk[i];
	Rvec2Quat_att(phi_ang, qe);
	QuatPrdct_att(qe, att_pkt->qbn, qbn);

	for(i=0; i<4; i++)
		att_pkt->qbn[i] = qbn[i];
	Quat2Dcm_att(att_pkt->qbn, att_pkt->Cbn);
	Dcm2Euler_att(att_pkt->Cbn, att_pkt->att);

	// attitude
	att_pkt->xk[0] = 0.0;
	att_pkt->xk[1] = 0.0;
	att_pkt->xk[2] = 0.0;

	// ==========================   Axis Leveling
	axis_leveling_att(att_pkt);

	//if (IF_AXIS_LEVELING_ATT)
	//{
	//	if (fabs(att_pkt->att[1])>TH_AXIS_LEVELING_ATT)
	//	{
	//		if (att_pkt->att[1] > 0.0)
	//		{
	//			att_pkt->if_vertical = 1;
	//			multiply_matrices_nxn_float_att(att_pkt->pCbn, att_pkt->pCbb_2, 3, 3, 3, pCbn_temp);
	//			CopyMatrix_N_Dimension_att(pCbn_temp, 3, 3, att_pkt->pCbn);
	//			multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_1,3, 3, att_pkt->bg_estimated, v3_temp);
	//			att_pkt->bg_estimated[0] = v3_temp[0];
	//			att_pkt->bg_estimated[1] = v3_temp[1];
	//			att_pkt->bg_estimated[2] = v3_temp[2];
	//			multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_1,3, 3, x456_temp, v3_temp);
	//			att_pkt->xk[3] = v3_temp[0];
	//			att_pkt->xk[4] = v3_temp[1];
	//			att_pkt->xk[5] = v3_temp[2];
	//		}
	//		else if (att_pkt->att[2] < 0.0)
	//		{
	//			att_pkt->if_vertical = -1;
	//			multiply_matrices_nxn_float_att(att_pkt->pCbn, att_pkt->pCbb_1, 3, 3, 3, pCbn_temp);
	//			CopyMatrix_N_Dimension_att(pCbn_temp, 3, 3, att_pkt->pCbn);
	//			multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_2,3, 3, att_pkt->bg_estimated, v3_temp);
	//			att_pkt->bg_estimated[0] = v3_temp[0];
	//			att_pkt->bg_estimated[1] = v3_temp[1];
	//			att_pkt->bg_estimated[2] = v3_temp[2];
	//			multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_2,3, 3, x456_temp, v3_temp);
	//			att_pkt->xk[3] = v3_temp[0];
	//			att_pkt->xk[4] = v3_temp[1];
	//			att_pkt->xk[5] = v3_temp[2];
	//		}
	//	}
	//	else
	//	{
	//		if (att_pkt->if_vertical == 1)
	//		{
	//			att_pkt->if_vertical = 0;
	//			multiply_matrices_nxn_float_att(att_pkt->pCbn, att_pkt->pCbb_1, 3, 3, 3, pCbn_temp);
	//			CopyMatrix_N_Dimension_att(pCbn_temp, 3, 3, att_pkt->pCbn);
	//			multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_2,3, 3, att_pkt->bg_estimated, v3_temp);
	//			att_pkt->bg_estimated[0] = v3_temp[0];
	//			att_pkt->bg_estimated[1] = v3_temp[1];
	//			att_pkt->bg_estimated[2] = v3_temp[2];
	//			multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_2,3, 3, x456_temp, v3_temp);
	//			att_pkt->xk[3] = v3_temp[0];
	//			att_pkt->xk[4] = v3_temp[1];
	//			att_pkt->xk[5] = v3_temp[2];
	//		}
	//		else if (att_pkt->if_vertical == -1)
	//		{
	//			att_pkt->if_vertical = 0;
	//			multiply_matrices_nxn_float_att(att_pkt->pCbn, att_pkt->pCbb_2, 3, 3, 3, pCbn_temp);
	//			CopyMatrix_N_Dimension_att(pCbn_temp, 3, 3, att_pkt->pCbn);
	//			multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_1,3, 3, att_pkt->bg_estimated, v3_temp);
	//			att_pkt->bg_estimated[0] = v3_temp[0];
	//			att_pkt->bg_estimated[1] = v3_temp[1];
	//			att_pkt->bg_estimated[2] = v3_temp[2];
	//			multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_1,3, 3, x456_temp, v3_temp);
	//			att_pkt->xk[3] = v3_temp[0];
	//			att_pkt->xk[4] = v3_temp[1];
	//			att_pkt->xk[5] = v3_temp[2];
	//		}
	//	} // End if (fabs(att_pkt->att[1])>TH_AXIS_LEVELING_ATT)

	//	Dcm2Quat_att(att_pkt->Cbn, att_pkt->qbn);
	//	Dcm2Euler_att(att_pkt->Cbn, att_pkt->att);
	//}

	// ==========================  End Axis Leveling

	if (IF_FEEDBACK_GYRO_BIAS_ATT)     //  IF ONLY ACCEL !!!!!!!!!!!!!!
	{
		if (fabs(f_b[0]) > 8.0)   // update y and z gyros
		{
			//att_pkt->bg_estimated[0]  += att_pkt->xk[3];
			att_pkt->bg_estimated[1]  += att_pkt->xk[4];
			att_pkt->bg_estimated[2]  += att_pkt->xk[5];
		}
		else if (fabs(f_b[1]) > 8.0)
		{
			att_pkt->bg_estimated[0]  += att_pkt->xk[3];
			//att_pkt->bg_estimated[1]  += att_pkt->xk[4];
			att_pkt->bg_estimated[2]  += att_pkt->xk[5];
		}
		else if (fabs(f_b[2]) > 8.0)
		{
			att_pkt->bg_estimated[0]  += att_pkt->xk[3];
			att_pkt->bg_estimated[1]  += att_pkt->xk[4];
			//att_pkt->bg_estimated[2]  += att_pkt->xk[5];
		}
		else
		{
			//att_pkt->bg_estimated[0]  += att_pkt->xk[3];
			//att_pkt->bg_estimated[1]  += att_pkt->xk[4];
			////att_pkt->bg_estimated[2]  += att_pkt->xk[5];
		}


		att_pkt->xk[3] = 0.0;
		att_pkt->xk[4] = 0.0;
		att_pkt->xk[5] = 0.0;
	}
}


/**
 * @brief		Function to calculate gyro biases from sensor readings
 * @details
 * @param[in/out]	Att_Pkt* att_pkt
 * @param[in]	t_cur: time stamp, Unit: secs
 * @param[in]	w_b[3]: gyro readings, Unit: rad/s ============================== RAD
 * @param[in]	f_b[3]: accel readings, Unit: m/s^2
 * @param[in]	m_b[3]: mag readings, Unit: mG
 * @param[in]	flag_mag: flag of mag
 * @param[in]	ini_heading: rad=================================================RAD
 * @param[in]	flag_ini_heading: 0-0.0; 1-set manually, 2-use mag-heading
 * @param[in]	flag_accuracy
 * @param[in]	seed_bias: If seed_availability == 1, then use this array as the initial gyro biases, Unit: RAD/s;  =========
 *                         Otherwise, the initial gyro biases will start from [0; 0; 0] inside the function;
 * @param[in]	seed_availability
 * @return 		NONE
 * @ NOTE: This function also changes the axis of w_b, f_b, and m_b
 */


void attitude_filter_att(Att_Pkt* att_pkt, DOUBLE64 t_cur, FLOAT32 w_b[3], FLOAT32 f_b[3], UINT8 flag_accel, FLOAT32 m_b[3], UINT8 flag_mag, FLOAT32 ini_heading, UINT8 flag_ini_heading, FLOAT32 seed_bias[3], UINT8 seed_availability)
{
    static UINT8 if_not_first_att = 0;
    //static Att_Pkt att_pkt;
    //static FLOAT32 gyro_bias[3];
    //static FLOAT32 Cbb_1[3][3], Cbb_2[3][3], *pCbb_1[3], *pCbb_2[3];
    FLOAT32 w_b_used[3], f_b_used[3], m_b_used[3], temp_b;

    float ts, norma;
    UINT8 tag_accel;
    //UINT8 tag_mag, neph_QS;
    FLOAT32 curG[3],f_n[3], f_n_hat[3];
    FLOAT32 F[NSTATE_ATT][NSTATE_ATT], G[NSTATE_ATT][6], PHI[NSTATE_ATT][NSTATE_ATT], Fts[NSTATE_ATT][NSTATE_ATT],  GT[6][NSTATE_ATT], GQ[NSTATE_ATT][NSTATE_ATT], GQGT[NSTATE_ATT][NSTATE_ATT];
    FLOAT32 PHIGQGT[NSTATE_ATT][NSTATE_ATT],PHIT[NSTATE_ATT][NSTATE_ATT], GQGTPHIT[NSTATE_ATT][NSTATE_ATT], PHIP[NSTATE_ATT][NSTATE_ATT], PHIPPHIT[NSTATE_ATT][NSTATE_ATT], Qk[6][6];
    FLOAT32 Rk3[3][3], CbnRk3[3][3], CbnT[3][3], Zk3[3], Fn1[3][3],inno3[3], Hk3xk[3];
    FLOAT32 EYE6[NSTATE_ATT][NSTATE_ATT], Hk3[3][NSTATE_ATT]; //, Hk2[3][NSTATE_ATT], Hk2xk[3], Rk2[3][3], CbnRk2[3][3];
    // FLOAT32 xk[NSTATE_ATT] = {0.0};
    UINT16 i;
    //UINT8 if_QS = 0;
    FLOAT32 accl_accu[3]; //, mag_acc[3];

    //FLOAT32* pCbn[3],
    FLOAT32* pF[NSTATE_ATT], *pG[NSTATE_ATT], *pPHI[NSTATE_ATT], *pFts[NSTATE_ATT], *pEYE6[NSTATE_ATT], *pGT[NSTATE_ATT];
    FLOAT32* pGQ[NSTATE_ATT], *pGQGT[NSTATE_ATT], *pPHIGQGT[NSTATE_ATT], *pPHIT[NSTATE_ATT], *pGQGTPHIT[NSTATE_ATT], *pQk[6], *pPHIP[NSTATE_ATT], *pPHIPPHIT[NSTATE_ATT];
    FLOAT32* pHk3[3], *pRk3[3], *pCbnRk3[3], *pCbnT[3]; //, *pHk2[3], *pRk2[3], *pCbnRk2[3];
    float qbn_temp[4];

    FLOAT32 xk_temp[NSTATE_ATT];
    FLOAT32* pxk_temp[NSTATE_ATT];

    //FLOAT32 v3_temp[3];

    if (if_not_first_att)
    {
        ts = (FLOAT32)(t_cur - att_pkt->t);

        if (fabs(t_cur-40.0) < 0.05)
        {
            t_cur = t_cur;
        }

        att_pkt->t = t_cur;
        if (att_pkt->t - att_pkt->t0 >= 2){
            att_pkt->availability = 1;
        } else {
            att_pkt->availability = 0;
        }

        if (!att_pkt->if_vertical)
        {
            w_b_used[0] = w_b[0];
            w_b_used[1] = w_b[1];
            w_b_used[2] = w_b[2];

            f_b_used[0] = f_b[0];
            f_b_used[1] = f_b[1];
            f_b_used[2] = f_b[2];

            m_b_used[0] = m_b[0];
            m_b_used[1] = m_b[1];
            m_b_used[2] = m_b[2];
        }
        else if (att_pkt->if_vertical == 1)   // Vertical Axis
        {
            w_b_used[0] =  w_b[2];
            f_b_used[0] =  f_b[2];
            m_b_used[0] =  m_b[2];

            w_b_used[1] =  w_b[1];
            f_b_used[1] =  f_b[1];
            m_b_used[1] =  m_b[1];

            w_b_used[2] =  -w_b[0];
            f_b_used[2] =  -f_b[0];
            m_b_used[2] =  -m_b[0];

            /*multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_1,3, 3, w_b, w_b_used);
             multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_1,3, 3, f_b, f_b_used);
             multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_1,3, 3, m_b, m_b_used);*/
        }
        else if (att_pkt->if_vertical == -1)
        {
            w_b_used[0] =  -w_b[2];
            f_b_used[0] =  -f_b[2];
            m_b_used[0] =  -m_b[2];

            w_b_used[1] =  w_b[1];
            f_b_used[1] =  f_b[1];
            m_b_used[1] =  m_b[1];

            w_b_used[2] =  w_b[0];
            f_b_used[2] =  f_b[0];
            m_b_used[2] =  m_b[0];
            /*multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_2,3, 3, w_b, w_b_used);
             multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_2,3, 3, f_b, f_b_used);
             multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_2,3, 3, m_b, m_b_used);*/
        }

        compensate_b_att(w_b_used, att_pkt->bg_estimated);
        curG[0] = w_b_used[0]*ts;
        curG[1] = w_b_used[1]*ts;
        curG[2] = w_b_used[2]*ts;

        /*printf("\nCbn matrix before Mech:\n");
         printf("%-.6f %-.6f %-.6f \n", att_pkt->Cbn[0][0], att_pkt->Cbn[0][1], att_pkt->Cbn[0][2]);
         printf("%-.6f %-.6f %-.6f \n", att_pkt->Cbn[1][0], att_pkt->Cbn[1][1], att_pkt->Cbn[1][2]);
         printf("%-.6f %-.6f %-.6f \n", att_pkt->Cbn[2][0], att_pkt->Cbn[2][1], att_pkt->Cbn[2][2]);*/


        Attitude_Mechanization_simple_float_att(curG, att_pkt->qbn, qbn_temp);
        att_pkt->qbn[0] = qbn_temp[0]; att_pkt->qbn[1] = qbn_temp[1];
        att_pkt->qbn[2] = qbn_temp[2]; att_pkt->qbn[3] = qbn_temp[3];
        Quat2Dcm_att(att_pkt->qbn, att_pkt->Cbn);


        ChangeArrayMemoryLayout_float_att(3,3,att_pkt->pCbn,(FLOAT32 *)att_pkt->Cbn,0);

        /*printf("\nCbn matrix before Mech:\n");
         printf("%-.6f %-.6f %-.6f \n", pCbn[0][0], pCbn[0][1], pCbn[0][2]);
         printf("%-.6f %-.6f %-.6f \n", pCbn[1][0], pCbn[1][1], pCbn[1][2]);
         printf("%-.6f %-.6f %-.6f \n", pCbn[2][0], pCbn[2][1], pCbn[2][2]);*/


        // Conditions
        if (1)
        {
            accl_accu[0] = AccelAccuracy_att[0];
            accl_accu[1] = AccelAccuracy_att[1];
            accl_accu[2] = AccelAccuracy_att[2];
        }


        // Prediction
        GetSystemModel_6_att(att_pkt->Cbn, (FLOAT32* )tao_bg_att, F, G);

        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,att_pkt->pP,(FLOAT32 *)att_pkt->P,0);   //P
        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,att_pkt->pQ,(FLOAT32 *)att_pkt->Q,0);   //Q
        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pF,(FLOAT32 *)F,0);		//F
        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pG,(FLOAT32 *)G,0);		//G
        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pPHI,(FLOAT32 *)PHI,1);	//Phi
        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pFts,(FLOAT32 *)Fts,0);   //F*ts
        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pEYE6,(FLOAT32 *)EYE6,1);	//I

        for (i=0; i<NSTATE_ATT; i++){
            EYE6[i][i] = 1.0;
        }

        /*printf("\nQ matrix:\n");
         printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pQ[0][0], att_pkt->pQ[0][1], att_pkt->pQ[0][2], att_pkt->pQ[0][3], att_pkt->pQ[0][4], att_pkt->pQ[0][5]);
         printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pQ[1][0], att_pkt->pQ[1][1], att_pkt->pQ[1][2], att_pkt->pQ[1][3], att_pkt->pQ[1][4], att_pkt->pQ[1][5]);
         printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pQ[2][0], att_pkt->pQ[2][1], att_pkt->pQ[2][2], att_pkt->pQ[2][3], att_pkt->pQ[2][4], att_pkt->pQ[2][5]);
         printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pQ[3][0], att_pkt->pQ[3][1], att_pkt->pQ[3][2], att_pkt->pQ[3][3], att_pkt->pQ[3][4], att_pkt->pQ[3][5]);
         printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pQ[4][0], att_pkt->pQ[4][1], att_pkt->pQ[4][2], att_pkt->pQ[4][3], att_pkt->pQ[4][4], att_pkt->pQ[4][5]);
         printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pQ[5][0], att_pkt->pQ[5][1], att_pkt->pQ[5][2], att_pkt->pQ[5][3], att_pkt->pQ[5][4], att_pkt->pQ[5][5]);
         */

        //PHI[0][0] = 0.0; PHI[0][1] = 0.0; PHI[0][2] = 0.0;  PHI[0][3] = -att_pkt->Cbn[0][0]*ts; PHI[0][4] = -att_pkt->Cbn[0][1]*ts; PHI[0][5] = -att_pkt->Cbn[0][2]*ts;
        //PHI[1][0] = 0.0; PHI[1][1] = 0.0; PHI[1][2] = 0.0;  PHI[1][3] = -att_pkt->Cbn[1][0]*ts; PHI[1][4] = -att_pkt->Cbn[1][1]*ts; PHI[1][5] = -att_pkt->Cbn[1][2]*ts;
        //PHI[2][0] = 0.0; PHI[2][1] = 0.0; PHI[2][2] = 0.0;  PHI[2][3] = -att_pkt->Cbn[2][0]*ts; PHI[2][4] = -att_pkt->Cbn[2][1]*ts; PHI[2][5] = -att_pkt->Cbn[2][2]*ts;
        //PHI[3][0] = 0.0; PHI[3][1] = 0.0; PHI[3][2] = 0.0;  PHI[3][3] = 0.0; PHI[3][4] = 0.0; PHI[3][5] = 0.0;
        //PHI[4][0] = 0.0; PHI[4][1] = 0.0; PHI[4][2] = 0.0;  PHI[4][3] = 0.0; PHI[4][4] = 0.0; PHI[4][5] = 0.0;
        //PHI[5][0] = 0.0; PHI[5][1] = 0.0; PHI[5][2] = 0.0;  PHI[5][3] = 0.0; PHI[5][4] = 0.0; PHI[5][5] = 0.0;
        //ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pPHI,(FLOAT32 *)PHI,0);     //G'


        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pGT,(FLOAT32 *)GT,0);     //G'
        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pGQ,(FLOAT32 *)GQ,0);		//GQ
        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pGQGT,(FLOAT32 *)GQGT,0);  //GQG'
        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pPHIGQGT,(FLOAT32 *)PHIGQGT,0);     //Phi*GQG'
        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pPHIT,(FLOAT32 *)PHIT,0);  //Phi'
        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pGQGTPHIT,(FLOAT32 *)GQGTPHIT,0);	  //GQG'*Phi'
        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pQk,(FLOAT32 *)Qk,0);		 //Qk  = 0.5*(Phi*GQGt + GQGt*Phi') * ts;
        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pPHIP,(FLOAT32 *)PHIP,0);			  //Phi*P
        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pPHIPPHIT,(FLOAT32 *)PHIPPHIT,0);	  //Phi*P*Phi'


        multiply_scalar_with_matrix_nxn_att(ts, pF, pFts,NSTATE_ATT, NSTATE_ATT);		//Fts = F*ts
        add_matrices_nxn_att(pEYE6, pFts, NSTATE_ATT, NSTATE_ATT, pPHI);				//Phi = I + F*ts
        matrix_transpose_nxn_float_att(pG, NSTATE_ATT, 6, pGT);						//G' = G'
        multiply_matrices_nxn_float_att(pG, att_pkt->pQ, NSTATE_ATT, 6, 6, pGQ);				//GQ = G*Q
        multiply_matrices_nxn_float_att(pGQ, pGT, NSTATE_ATT, 6, NSTATE_ATT, pGQGT);	//GQG' = G*Q*G'
        multiply_matrices_nxn_float_att(pPHI, pGQGT, NSTATE_ATT, NSTATE_ATT, NSTATE_ATT, pPHIGQGT);	//PhiGQG' = Phi*GQG'
        matrix_transpose_nxn_float_att(pPHI, NSTATE_ATT, NSTATE_ATT, pPHIT);			//PHI'
        multiply_matrices_nxn_float_att(pGQGT, pPHIT, NSTATE_ATT, NSTATE_ATT, NSTATE_ATT, pGQGTPHIT);	//GQG'Phi' = GQG'*Phi';
        add_matrices_nxn_att(pPHIGQGT, pGQGTPHIT, NSTATE_ATT, NSTATE_ATT, pQk);         //Qk = Phi*GQGt + GQGt*Phi'
        multiply_scalar_with_matrix_nxn_att(0.5*ts, pQk, pQk, NSTATE_ATT, NSTATE_ATT);  //Qk = 0.5*ts * Qk

        multiply_matrices_nxn_float_att(pPHI, att_pkt->pP, NSTATE_ATT, NSTATE_ATT, NSTATE_ATT, pPHIP);			//Phi*P
        multiply_matrices_nxn_float_att(pPHIP, pPHIT, NSTATE_ATT, NSTATE_ATT, NSTATE_ATT, pPHIPPHIT);	//Phi*P*Phi'
        add_matrices_nxn_att(pPHIPPHIT, pQk, NSTATE_ATT, NSTATE_ATT, att_pkt->pP);				//P = Phi*P*Phi'+ Qk

        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,1,att_pkt->pxk,(FLOAT32 *)att_pkt->xk,0);   //xk
        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,1,pxk_temp,(FLOAT32 *)xk_temp,0);   //xk
        multiply_matrices_nxn_float_att(pPHI, att_pkt->pxk, NSTATE_ATT, NSTATE_ATT, 1, pxk_temp);	//Phi*xk
        CopyMatrix_N_Dimension_att(pxk_temp, NSTATE_ATT, 1, att_pkt->pxk);

        //printf("\nPHI matrix:\n");
        //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pPHI[0][0], pPHI[0][1], pPHI[0][2], pPHI[0][3], pPHI[0][4], pPHI[0][5]);
        //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pPHI[1][0], pPHI[1][1], pPHI[1][2], pPHI[1][3], pPHI[1][4], pPHI[1][5]);
        //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pPHI[2][0], pPHI[2][1], pPHI[2][2], pPHI[2][3], pPHI[2][4], pPHI[2][5]);
        //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pPHI[3][0], pPHI[3][1], pPHI[3][2], pPHI[3][3], pPHI[3][4], pPHI[3][5]);
        //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pPHI[4][0], pPHI[4][1], pPHI[4][2], pPHI[4][3], pPHI[4][4], pPHI[4][5]);
        //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pPHI[5][0], pPHI[5][1], pPHI[5][2], pPHI[5][3], pPHI[5][4], pPHI[5][5]);

        //printf("\nQk matrix:\n");
        //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pQk[0][0], pQk[0][1], pQk[0][2], pQk[0][3], pQk[0][4], pQk[0][5]);
        //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pQk[1][0], pQk[1][1], pQk[1][2], pQk[1][3], pQk[1][4], pQk[1][5]);
        //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pQk[2][0], pQk[2][1], pQk[2][2], pQk[2][3], pQk[2][4], pQk[2][5]);
        //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pQk[3][0], pQk[3][1], pQk[3][2], pQk[3][3], pQk[3][4], pQk[3][5]);
        //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pQk[4][0], pQk[4][1], pQk[4][2], pQk[4][3], pQk[4][4], pQk[4][5]);
        //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pQk[5][0], pQk[5][1], pQk[5][2], pQk[5][3], pQk[5][4], pQk[5][5]);



        // UPDATE
        norma = sqrt(f_b_used[0]*f_b_used[0]+f_b_used[1]*f_b_used[1]+f_b_used[2]*f_b_used[2]) - Norm_Gravity_ATT;

        tag_accel = 1;
//#define TH_LARGE_ACC 10.0
        if (!flag_accel || fabs(norma) > TH_LARGE_ACC)
        {
            tag_accel = 0;
        }

        // Accelerometer updates
        if (tag_accel)
        {
            ChangeArrayMemoryLayout_float_att(3,NSTATE_ATT,pHk3,(FLOAT32 *)Hk3,1);
            ChangeArrayMemoryLayout_float_att(3,3,pRk3,(FLOAT32 *)Rk3,0);		   //Rk = diag
            ChangeArrayMemoryLayout_float_att(3,3,pCbnRk3,(FLOAT32 *)CbnRk3,0);  //Cbn*Rk
            ChangeArrayMemoryLayout_float_att(3,3,pCbnT,(FLOAT32 *)CbnT,0);  //Cbn'

            /*printf("\nCbn matrix:\n");
             printf("%-.6f %-.6f %-.6f \n", pCbn[0][0], pCbn[0][1], pCbn[0][2]);
             printf("%-.6f %-.6f %-.6f \n", pCbn[1][0], pCbn[1][1], pCbn[1][2]);
             printf("%-.6f %-.6f %-.6f \n", pCbn[2][0], pCbn[2][1], pCbn[2][2]);*/


            multiply_matrix_with_vector_nxn_att(att_pkt->pCbn, 3, 3, f_b_used, f_n);       //fn = Cbn*fb
            f_n_hat[0] = 0.0;
            f_n_hat[1] = 0.0;
            f_n_hat[2] = -Norm_Gravity_ATT;
            make_skew_symmetric_matrix_att(f_n_hat, Fn1);

            Hk3[0][0] = Fn1[0][0]; Hk3[0][1] = Fn1[0][1]; Hk3[0][2] = Fn1[0][2];
            Hk3[1][0] = Fn1[1][0]; Hk3[1][1] = Fn1[1][1]; Hk3[1][2] = Fn1[1][2];
            Hk3[2][0] = Fn1[2][0]; Hk3[2][1] = Fn1[2][1]; Hk3[2][2] = Fn1[2][2];

            Zk3[0] = f_n[0]-f_n_hat[0];
            Zk3[1] = f_n[1]-f_n_hat[1];
            Zk3[2] = f_n[2]-f_n_hat[2];

            diag2_sq_att(accl_accu, 3, pRk3);
            multiply_matrices_nxn_float_att(att_pkt->pCbn, pRk3, 3, 3, 3, pCbnRk3);
            matrix_transpose_nxn_float_att(att_pkt->pCbn, 3, 3, pCbnT);
            multiply_matrices_nxn_float_att(pCbnRk3, pCbnT, 3, 3, 3, pRk3);   //Rk=Cbn*Rk*Cbn'
            multiply_matrix_with_vector_nxn_att(pHk3, 3, 6, att_pkt->xk, Hk3xk);       //Hx = H*x

            inno3[0] = Zk3[0] - Hk3xk[0];
            inno3[1] = Zk3[1] - Hk3xk[1];
            inno3[2] = Zk3[2] - Hk3xk[2];

            /*printf("\nP matrix:\n");
             printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pP[0][0], att_pkt->pP[0][1], att_pkt->pP[0][2], att_pkt->pP[0][3], att_pkt->pP[0][4], att_pkt->pP[0][5]);
             printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pP[1][0], att_pkt->pP[1][1], att_pkt->pP[1][2], att_pkt->pP[1][3], att_pkt->pP[1][4], att_pkt->pP[1][5]);
             printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pP[2][0], att_pkt->pP[2][1], att_pkt->pP[2][2], att_pkt->pP[2][3], att_pkt->pP[2][4], att_pkt->pP[2][5]);
             printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pP[3][0], att_pkt->pP[3][1], att_pkt->pP[3][2], att_pkt->pP[3][3], att_pkt->pP[3][4], att_pkt->pP[3][5]);
             printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pP[4][0], att_pkt->pP[4][1], att_pkt->pP[4][2], att_pkt->pP[4][3], att_pkt->pP[4][4], att_pkt->pP[4][5]);
             printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pP[5][0], att_pkt->pP[5][1], att_pkt->pP[5][2], att_pkt->pP[5][3], att_pkt->pP[5][4], att_pkt->pP[5][5]);
             printf("\nHk3 matrix:\n");
             printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pHk3[0][0], pHk3[0][1], pHk3[0][2], pHk3[0][3], pHk3[0][4], pHk3[0][5]);
             printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pHk3[1][0], pHk3[1][1], pHk3[1][2], pHk3[1][3], pHk3[1][4], pHk3[1][5]);
             printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pHk3[2][0], pHk3[2][1], pHk3[2][2], pHk3[2][3], pHk3[2][4], pHk3[2][5]);
             printf("\nR matrix:\n");
             printf("%-.6f %-.6f %-.6f \n", pRk3[0][0], pRk3[0][1], pRk3[0][2]);
             printf("%-.6f %-.6f %-.6f \n", pRk3[1][0], pRk3[1][1], pRk3[1][2]);
             printf("%-.6f %-.6f %-.6f \n", pRk3[2][0], pRk3[2][1], pRk3[2][2]);
             printf("\nINNO:\n");
             printf("%-.6f %-.6f %-.6f \n", inno3[0], inno3[1], inno3[2]);*/



            KF_update1_float_att(3, att_pkt->xk, att_pkt->pP, inno3, pHk3, pRk3, NSTATE_ATT);

            /*printf("\nXK:\n");
             printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->xk[0], att_pkt->xk[1], att_pkt->xk[2], att_pkt->xk[3], att_pkt->xk[4], att_pkt->xk[5]);*/
        }

        // Feedback
        // INSNavFeedBack_Attitude_float_att(att_pkt->xk, att_pkt->qbn, att_pkt->Cbn, att_pkt->att, att_pkt->bg_estimated);
        INSNavFeedBack_Attitude_float_att(att_pkt, f_b_used);

        /*printf("\nCbn matrix before Mech:\n");
         printf("%-.6f %-.6f %-.6f \n", att_pkt->Cbn[0][0], att_pkt->Cbn[0][1], att_pkt->Cbn[0][2]);
         printf("%-.6f %-.6f %-.6f \n", att_pkt->Cbn[1][0], att_pkt->Cbn[1][1], att_pkt->Cbn[1][2]);
         printf("%-.6f %-.6f %-.6f \n", att_pkt->Cbn[2][0], att_pkt->Cbn[2][1], att_pkt->Cbn[2][2]);

         printf("\nXK:\n");
         printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->xk[0], att_pkt->xk[1], att_pkt->xk[2], att_pkt->xk[3], att_pkt->xk[4], att_pkt->xk[5]);

         printf("\nqbn:\n");
         printf("%-.6f %-.6f %-.6f %-.6f \n", att_pkt->qbn[0], att_pkt->qbn[1], att_pkt->qbn[2], att_pkt->qbn[3]);

         printf("\nATT:\n");
         printf("%-.6f %-.6f %-.6f\n", att_pkt->att[0], att_pkt->att[1], att_pkt->att[2]);

         printf("\ngyro_bias:\n");
         printf("%-.6f %-.6f %-.6f\n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);*/

        /*att_pkt->bg_estimated[0] = gyro_bias[0];
         att_pkt->bg_estimated[1] = gyro_bias[1];
         att_pkt->bg_estimated[2] = gyro_bias[2];*/

        att_pkt->std_att[0] = (FLOAT32) sqrt(att_pkt->pP[0][0]);
        att_pkt->std_att[1] = (FLOAT32) sqrt(att_pkt->pP[1][1]);
        att_pkt->std_att[2] = (FLOAT32) sqrt(att_pkt->pP[2][2]);
        att_pkt->std_bg[0]  = (FLOAT32) sqrt(att_pkt->pP[3][3]);
        att_pkt->std_bg[1]  = (FLOAT32) sqrt(att_pkt->pP[4][4]);
        att_pkt->std_bg[2]  = (FLOAT32) sqrt(att_pkt->pP[5][5]);

    }
    else
    {
        if_not_first_att = 1;

        if (seed_availability)
        {
            att_pkt->bg_estimated[0] = seed_bias[0];
            att_pkt->bg_estimated[1] = seed_bias[1];
            att_pkt->bg_estimated[2] = seed_bias[2];
        }
        else
        {
            att_pkt->bg_estimated[0] = 0.0;
            att_pkt->bg_estimated[1] = 0.0;
            att_pkt->bg_estimated[2] = 0.0;
        }

        att_pkt->P[0][0] = ini_att_var_att[0] * ini_att_var_att[0];
        att_pkt->P[1][1] = ini_att_var_att[1] * ini_att_var_att[1];
        att_pkt->P[2][2] = ini_att_var_att[2] * ini_att_var_att[2];
        att_pkt->P[3][3] = ini_bg_var_att[0] * ini_bg_var_att[0];
        att_pkt->P[4][4] = ini_bg_var_att[1] * ini_bg_var_att[1];
        att_pkt->P[5][5] = ini_bg_var_att[2] * ini_bg_var_att[2];

        att_pkt->Q[0][0] = ARW_q_att[0] * ARW_q_att[0];
        att_pkt->Q[1][1] = ARW_q_att[1] * ARW_q_att[1];
        att_pkt->Q[2][2] = ARW_q_att[2] * ARW_q_att[2];
        att_pkt->Q[3][3] = q_bg_att[0] * q_bg_att[0];
        att_pkt->Q[4][4] = q_bg_att[1] * q_bg_att[1];
        att_pkt->Q[5][5] = q_bg_att[2] * q_bg_att[2];

        att_pkt->xk[0] = 0.0; att_pkt->xk[1] = 0.0; att_pkt->xk[2] = 0.0;
        att_pkt->xk[3] = 0.0; att_pkt->xk[4] = 0.0; att_pkt->xk[5] = 0.0;

        att_pkt->t0 = t_cur;
        att_pkt->t = t_cur;
        ts = 1.0/SENSORS_RATE_MOBILE;


        att_pkt->att[0] = cal_roll_att(f_b);
        att_pkt->att[1] = cal_pitch_att(f_b);
        if (flag_ini_heading == 1)
        {
            att_pkt->att[2] = ini_heading;
        }
        else if (!flag_ini_heading)
        {
            att_pkt->att[2] = 0.0;
        }
        Euler2Dcm_att(att_pkt->att, att_pkt->Cbn);
        Dcm2Quat_att(att_pkt->Cbn, att_pkt->qbn);

        att_pkt->if_vertical = 0;
        att_pkt->Cbb_1[0][0] = 0.0;  att_pkt->Cbb_1[0][1] = 0.0; att_pkt->Cbb_1[0][2] = 1.0;
        att_pkt->Cbb_1[1][0] = 0.0;  att_pkt->Cbb_1[1][1] = 1.0; att_pkt->Cbb_1[1][2] = 0.0;
        att_pkt->Cbb_1[2][0] = -1.0; att_pkt->Cbb_1[2][1] = 0.0; att_pkt->Cbb_1[2][2] = 0.0;
        att_pkt->Cbb_2[0][0] = 0.0;  att_pkt->Cbb_2[0][1] = 0.0; att_pkt->Cbb_2[0][2] = -1.0;
        att_pkt->Cbb_2[1][0] = 0.0;  att_pkt->Cbb_2[1][1] = 1.0; att_pkt->Cbb_2[1][2] = 0.0;
        att_pkt->Cbb_2[2][0] = 1.0;  att_pkt->Cbb_2[2][1] = 0.0; att_pkt->Cbb_2[2][2] = 0.0;
        ChangeArrayMemoryLayout_float_att(3,3,att_pkt->pCbb_1,(FLOAT32 *)att_pkt->Cbb_1,0);
        ChangeArrayMemoryLayout_float_att(3,3,att_pkt->pCbb_2,(FLOAT32 *)att_pkt->Cbb_2,0);

    }

    // Change Axis of w, f, and m_b
    if (att_pkt->if_vertical == 1)   // Vertical Axis     // Can be optimized by using att_pkt_if_vertical_pre
    {
        temp_b = w_b[0];
        w_b[0] =  w_b[2];
        w_b[2] = -temp_b;

        temp_b = f_b[0];
        f_b[0] =  f_b[2];
        f_b[2] = -temp_b;

        temp_b = m_b[0];
        m_b[0] =  m_b[2];
        m_b[2] = -temp_b;

        //multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_1,3, 3, w_b, v3_temp);
        //w_b[0] = v3_temp[0];
        //w_b[1] = v3_temp[1];
        //w_b[2] = v3_temp[2];
        //multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_1,3, 3, f_b, v3_temp);
        //f_b[0] = v3_temp[0];
        //f_b[1] = v3_temp[1];
        //f_b[2] = v3_temp[2];
        //multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_1,3, 3, m_b, v3_temp);
        //m_b[0] = v3_temp[0];
        //m_b[1] = v3_temp[1];
        //m_b[2] = v3_temp[2];
    }
    else if (att_pkt->if_vertical == -1)
    {
        temp_b = w_b[0];
        w_b[0] = -w_b[2];
        w_b[2] = temp_b;

        temp_b = f_b[0];
        f_b[0] = -f_b[2];
        f_b[2] = temp_b;

        temp_b = m_b[0];
        m_b[0] = -m_b[2];
        m_b[2] = temp_b;

        /*multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_2,3, 3, w_b, v3_temp);
         w_b[0] = v3_temp[0];
         w_b[1] = v3_temp[1];
         w_b[2] = v3_temp[2];
         multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_2,3, 3, f_b, v3_temp);
         f_b[0] = v3_temp[0];
         f_b[1] = v3_temp[1];
         f_b[2] = v3_temp[2];
         multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_2,3, 3, m_b, v3_temp);
         m_b[0] = v3_temp[0];
         m_b[1] = v3_temp[1];
         m_b[2] = v3_temp[2];*/
    }

}


void attitude_filter_without_bg_conpensation_att(Att_Pkt* att_pkt, MemoIndoor *memo, DOUBLE64 t_cur, FLOAT32 w_b[3], FLOAT32 f_b[3], UINT8 flag_accel, FLOAT32 m_b[3], UINT8 flag_mag, FLOAT32 ini_heading, UINT8 flag_ini_heading, FLOAT32 seed_bias[3], UINT8 seed_availability)
{
    static UINT8 if_not_first_att = 0;
    //static Att_Pkt att_pkt;
    //static FLOAT32 gyro_bias[3];
    //static FLOAT32 Cbb_1[3][3], Cbb_2[3][3], *pCbb_1[3], *pCbb_2[3];
    FLOAT32 w_b_used[3], f_b_used[3], m_b_used[3], temp_b;
    static FLOAT32 mag_n00[3] = {MAG_N00_N, MAG_N00_E, MAG_N00_D};

    float ts, norma;
    UINT8 tag_accel, tag_mag;
    //UINT8  neph_QS;
    FLOAT32 curG[3],f_n[3], f_n_hat[3], m_n[3], m_n_hat[3];
    FLOAT32 F[NSTATE_ATT][NSTATE_ATT], G[NSTATE_ATT][6], PHI[NSTATE_ATT][NSTATE_ATT], Fts[NSTATE_ATT][NSTATE_ATT],  GT[6][NSTATE_ATT], GQ[NSTATE_ATT][NSTATE_ATT], GQGT[NSTATE_ATT][NSTATE_ATT];
    FLOAT32 PHIGQGT[NSTATE_ATT][NSTATE_ATT],PHIT[NSTATE_ATT][NSTATE_ATT], GQGTPHIT[NSTATE_ATT][NSTATE_ATT], PHIP[NSTATE_ATT][NSTATE_ATT], PHIPPHIT[NSTATE_ATT][NSTATE_ATT], Qk[6][6];
    FLOAT32 Rk3[3][3], CbnRk3[3][3], CbnT[3][3], Zk3[3], Fn1[3][3],inno3[3], Hk3xk[3];
    FLOAT32 EYE6[NSTATE_ATT][NSTATE_ATT], Hk3[3][NSTATE_ATT], Hk2[3][NSTATE_ATT], Hk2xk[3], Rk2[3][3], CbnRk2[3][3]; //, Hk2[3][NSTATE_ATT], Hk2xk[3], Rk2[3][3], CbnRk2[3][3];
    // FLOAT32 xk[NSTATE_ATT] = {0.0};
    UINT16 i;
    //UINT8 if_QS = 0;
    FLOAT32 accl_accu[3], mag_acc[3];

    //FLOAT32* pCbn[3],
    FLOAT32* pF[NSTATE_ATT], *pG[NSTATE_ATT], *pPHI[NSTATE_ATT], *pFts[NSTATE_ATT], *pEYE6[NSTATE_ATT], *pGT[NSTATE_ATT];
    FLOAT32* pGQ[NSTATE_ATT], *pGQGT[NSTATE_ATT], *pPHIGQGT[NSTATE_ATT], *pPHIT[NSTATE_ATT], *pGQGTPHIT[NSTATE_ATT], *pQk[6], *pPHIP[NSTATE_ATT], *pPHIPPHIT[NSTATE_ATT];
    FLOAT32* pHk3[3], *pRk3[3], *pCbnRk3[3], *pCbnT[3], *pHk2[3], *pRk2[3], *pCbnRk2[3]; //, *pHk2[3], *pRk2[3], *pCbnRk2[3];
    float qbn_temp[4];

    FLOAT32 xk_temp[NSTATE_ATT];
    FLOAT32* pxk_temp[NSTATE_ATT];

    //FLOAT32 v3_temp[3];

    if (if_not_first_att)
    {
        ts = (FLOAT32)(t_cur - att_pkt->t);

        //if (fabs(t_cur-40.0) < 0.05)
        //{
        //	t_cur = t_cur;
        //}

        att_pkt->t = t_cur;
        if (att_pkt->t - att_pkt->t0 >= 2){
            att_pkt->availability = 1;
        } else {
            att_pkt->availability = 0;
        }

        w_b_used[0] = w_b[0];
        w_b_used[1] = w_b[1];
        w_b_used[2] = w_b[2];

        f_b_used[0] = f_b[0];
        f_b_used[1] = f_b[1];
        f_b_used[2] = f_b[2];

        m_b_used[0] = m_b[0];
        m_b_used[1] = m_b[1];
        m_b_used[2] = m_b[2];

        //compensate_b_att(w_b_used, att_pkt->bg_estimated);
        curG[0] = w_b_used[0]*ts;
        curG[1] = w_b_used[1]*ts;
        curG[2] = w_b_used[2]*ts;

        /*printf("\nCbn matrix before Mech:\n");
         printf("%-.6f %-.6f %-.6f \n", att_pkt->Cbn[0][0], att_pkt->Cbn[0][1], att_pkt->Cbn[0][2]);
         printf("%-.6f %-.6f %-.6f \n", att_pkt->Cbn[1][0], att_pkt->Cbn[1][1], att_pkt->Cbn[1][2]);
         printf("%-.6f %-.6f %-.6f \n", att_pkt->Cbn[2][0], att_pkt->Cbn[2][1], att_pkt->Cbn[2][2]);*/


        Attitude_Mechanization_simple_float_att(curG, att_pkt->qbn, qbn_temp);
        att_pkt->qbn[0] = qbn_temp[0]; att_pkt->qbn[1] = qbn_temp[1];
        att_pkt->qbn[2] = qbn_temp[2]; att_pkt->qbn[3] = qbn_temp[3];
        Quat2Dcm_att(att_pkt->qbn, att_pkt->Cbn);


        ChangeArrayMemoryLayout_float_att(3,3,att_pkt->pCbn,(FLOAT32 *)att_pkt->Cbn,0);

        /*printf("\nCbn matrix before Mech:\n");
         printf("%-.6f %-.6f %-.6f \n", pCbn[0][0], pCbn[0][1], pCbn[0][2]);
         printf("%-.6f %-.6f %-.6f \n", pCbn[1][0], pCbn[1][1], pCbn[1][2]);
         printf("%-.6f %-.6f %-.6f \n", pCbn[2][0], pCbn[2][1], pCbn[2][2]);*/


        // Conditions
        //if (1)
        //{
        accl_accu[0] = AccelAccuracy_att[0];
        accl_accu[1] = AccelAccuracy_att[1];
        accl_accu[2] = AccelAccuracy_att[2];

        mag_acc[0] = MagAccuracy_att[0];
        mag_acc[1] = MagAccuracy_att[1];
        mag_acc[2] = MagAccuracy_att[2];
        //}


        // Prediction
        GetSystemModel_6_att(att_pkt->Cbn, (FLOAT32* )tao_bg_att, F, G);

        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,att_pkt->pP,(FLOAT32 *)att_pkt->P,0);   //P
        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,att_pkt->pQ,(FLOAT32 *)att_pkt->Q,0);   //Q
        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pF,(FLOAT32 *)F,0);		//F
        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pG,(FLOAT32 *)G,0);		//G
        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pPHI,(FLOAT32 *)PHI,1);	//Phi
        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pFts,(FLOAT32 *)Fts,0);   //F*ts
        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pEYE6,(FLOAT32 *)EYE6,1);	//I

        for (i=0; i<NSTATE_ATT; i++){
            EYE6[i][i] = 1.0;
        }

        /*printf("\nQ matrix:\n");
         printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pQ[0][0], att_pkt->pQ[0][1], att_pkt->pQ[0][2], att_pkt->pQ[0][3], att_pkt->pQ[0][4], att_pkt->pQ[0][5]);
         printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pQ[1][0], att_pkt->pQ[1][1], att_pkt->pQ[1][2], att_pkt->pQ[1][3], att_pkt->pQ[1][4], att_pkt->pQ[1][5]);
         printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pQ[2][0], att_pkt->pQ[2][1], att_pkt->pQ[2][2], att_pkt->pQ[2][3], att_pkt->pQ[2][4], att_pkt->pQ[2][5]);
         printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pQ[3][0], att_pkt->pQ[3][1], att_pkt->pQ[3][2], att_pkt->pQ[3][3], att_pkt->pQ[3][4], att_pkt->pQ[3][5]);
         printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pQ[4][0], att_pkt->pQ[4][1], att_pkt->pQ[4][2], att_pkt->pQ[4][3], att_pkt->pQ[4][4], att_pkt->pQ[4][5]);
         printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pQ[5][0], att_pkt->pQ[5][1], att_pkt->pQ[5][2], att_pkt->pQ[5][3], att_pkt->pQ[5][4], att_pkt->pQ[5][5]);
         */

        //PHI[0][0] = 0.0; PHI[0][1] = 0.0; PHI[0][2] = 0.0;  PHI[0][3] = -att_pkt->Cbn[0][0]*ts; PHI[0][4] = -att_pkt->Cbn[0][1]*ts; PHI[0][5] = -att_pkt->Cbn[0][2]*ts;
        //PHI[1][0] = 0.0; PHI[1][1] = 0.0; PHI[1][2] = 0.0;  PHI[1][3] = -att_pkt->Cbn[1][0]*ts; PHI[1][4] = -att_pkt->Cbn[1][1]*ts; PHI[1][5] = -att_pkt->Cbn[1][2]*ts;
        //PHI[2][0] = 0.0; PHI[2][1] = 0.0; PHI[2][2] = 0.0;  PHI[2][3] = -att_pkt->Cbn[2][0]*ts; PHI[2][4] = -att_pkt->Cbn[2][1]*ts; PHI[2][5] = -att_pkt->Cbn[2][2]*ts;
        //PHI[3][0] = 0.0; PHI[3][1] = 0.0; PHI[3][2] = 0.0;  PHI[3][3] = 0.0; PHI[3][4] = 0.0; PHI[3][5] = 0.0;
        //PHI[4][0] = 0.0; PHI[4][1] = 0.0; PHI[4][2] = 0.0;  PHI[4][3] = 0.0; PHI[4][4] = 0.0; PHI[4][5] = 0.0;
        //PHI[5][0] = 0.0; PHI[5][1] = 0.0; PHI[5][2] = 0.0;  PHI[5][3] = 0.0; PHI[5][4] = 0.0; PHI[5][5] = 0.0;
        //ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pPHI,(FLOAT32 *)PHI,0);     //G'


        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pGT,(FLOAT32 *)GT,0);     //G'
        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pGQ,(FLOAT32 *)GQ,0);		//GQ
        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pGQGT,(FLOAT32 *)GQGT,0);  //GQG'
        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pPHIGQGT,(FLOAT32 *)PHIGQGT,0);     //Phi*GQG'
        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pPHIT,(FLOAT32 *)PHIT,0);  //Phi'
        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pGQGTPHIT,(FLOAT32 *)GQGTPHIT,0);	  //GQG'*Phi'
        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pQk,(FLOAT32 *)Qk,0);		 //Qk  = 0.5*(Phi*GQGt + GQGt*Phi') * ts;
        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pPHIP,(FLOAT32 *)PHIP,0);			  //Phi*P
        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pPHIPPHIT,(FLOAT32 *)PHIPPHIT,0);	  //Phi*P*Phi'


        multiply_scalar_with_matrix_nxn_att(ts, pF, pFts,NSTATE_ATT, NSTATE_ATT);		//Fts = F*ts
        add_matrices_nxn_att(pEYE6, pFts, NSTATE_ATT, NSTATE_ATT, pPHI);				//Phi = I + F*ts
        matrix_transpose_nxn_float_att(pG, NSTATE_ATT, 6, pGT);						//G' = G'
        multiply_matrices_nxn_float_att(pG, att_pkt->pQ, NSTATE_ATT, 6, 6, pGQ);				//GQ = G*Q
        multiply_matrices_nxn_float_att(pGQ, pGT, NSTATE_ATT, 6, NSTATE_ATT, pGQGT);	//GQG' = G*Q*G'
        multiply_matrices_nxn_float_att(pPHI, pGQGT, NSTATE_ATT, NSTATE_ATT, NSTATE_ATT, pPHIGQGT);	//PhiGQG' = Phi*GQG'
        matrix_transpose_nxn_float_att(pPHI, NSTATE_ATT, NSTATE_ATT, pPHIT);			//PHI'
        multiply_matrices_nxn_float_att(pGQGT, pPHIT, NSTATE_ATT, NSTATE_ATT, NSTATE_ATT, pGQGTPHIT);	//GQG'Phi' = GQG'*Phi';
        add_matrices_nxn_att(pPHIGQGT, pGQGTPHIT, NSTATE_ATT, NSTATE_ATT, pQk);         //Qk = Phi*GQGt + GQGt*Phi'
        multiply_scalar_with_matrix_nxn_att(0.5*ts, pQk, pQk, NSTATE_ATT, NSTATE_ATT);  //Qk = 0.5*ts * Qk

        multiply_matrices_nxn_float_att(pPHI, att_pkt->pP, NSTATE_ATT, NSTATE_ATT, NSTATE_ATT, pPHIP);			//Phi*P
        multiply_matrices_nxn_float_att(pPHIP, pPHIT, NSTATE_ATT, NSTATE_ATT, NSTATE_ATT, pPHIPPHIT);	//Phi*P*Phi'
        add_matrices_nxn_att(pPHIPPHIT, pQk, NSTATE_ATT, NSTATE_ATT, att_pkt->pP);				//P = Phi*P*Phi'+ Qk

        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,1,att_pkt->pxk,(FLOAT32 *)att_pkt->xk,0);   //xk
        ChangeArrayMemoryLayout_float_att(NSTATE_ATT,1,pxk_temp,(FLOAT32 *)xk_temp,0);   //xk
        multiply_matrices_nxn_float_att(pPHI, att_pkt->pxk, NSTATE_ATT, NSTATE_ATT, 1, pxk_temp);	//Phi*xk
        CopyMatrix_N_Dimension_att(pxk_temp, NSTATE_ATT, 1, att_pkt->pxk);

        //printf("\nPHI matrix:\n");
        //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pPHI[0][0], pPHI[0][1], pPHI[0][2], pPHI[0][3], pPHI[0][4], pPHI[0][5]);
        //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pPHI[1][0], pPHI[1][1], pPHI[1][2], pPHI[1][3], pPHI[1][4], pPHI[1][5]);
        //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pPHI[2][0], pPHI[2][1], pPHI[2][2], pPHI[2][3], pPHI[2][4], pPHI[2][5]);
        //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pPHI[3][0], pPHI[3][1], pPHI[3][2], pPHI[3][3], pPHI[3][4], pPHI[3][5]);
        //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pPHI[4][0], pPHI[4][1], pPHI[4][2], pPHI[4][3], pPHI[4][4], pPHI[4][5]);
        //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pPHI[5][0], pPHI[5][1], pPHI[5][2], pPHI[5][3], pPHI[5][4], pPHI[5][5]);

        //printf("\nQk matrix:\n");
        //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pQk[0][0], pQk[0][1], pQk[0][2], pQk[0][3], pQk[0][4], pQk[0][5]);
        //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pQk[1][0], pQk[1][1], pQk[1][2], pQk[1][3], pQk[1][4], pQk[1][5]);
        //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pQk[2][0], pQk[2][1], pQk[2][2], pQk[2][3], pQk[2][4], pQk[2][5]);
        //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pQk[3][0], pQk[3][1], pQk[3][2], pQk[3][3], pQk[3][4], pQk[3][5]);
        //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pQk[4][0], pQk[4][1], pQk[4][2], pQk[4][3], pQk[4][4], pQk[4][5]);
        //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pQk[5][0], pQk[5][1], pQk[5][2], pQk[5][3], pQk[5][4], pQk[5][5]);



        // UPDATE
        norma = sqrt(f_b_used[0]*f_b_used[0]+f_b_used[1]*f_b_used[1]+f_b_used[2]*f_b_used[2]) - Norm_Gravity_ATT;

        tag_accel = 0;
//#define TH_LARGE_ACC 20.0
        //if (!flag_accel || fabs(norma) > TH_LARGE_ACC)
        if (flag_accel )
        {
            tag_accel = 1;
        }

        tag_mag = 0;
        if (flag_mag)
        {
            tag_mag = 1;
        }

        // Accelerometer updates
        if (tag_accel)
        {
            ChangeArrayMemoryLayout_float_att(3,NSTATE_ATT,pHk3,(FLOAT32 *)Hk3,1);
            ChangeArrayMemoryLayout_float_att(3,3,pRk3,(FLOAT32 *)Rk3,0);		   //Rk = diag
            ChangeArrayMemoryLayout_float_att(3,3,pCbnRk3,(FLOAT32 *)CbnRk3,0);  //Cbn*Rk
            ChangeArrayMemoryLayout_float_att(3,3,pCbnT,(FLOAT32 *)CbnT,0);  //Cbn'

            /*printf("\nCbn matrix:\n");
             printf("%-.6f %-.6f %-.6f \n", pCbn[0][0], pCbn[0][1], pCbn[0][2]);
             printf("%-.6f %-.6f %-.6f \n", pCbn[1][0], pCbn[1][1], pCbn[1][2]);
             printf("%-.6f %-.6f %-.6f \n", pCbn[2][0], pCbn[2][1], pCbn[2][2]);*/


            multiply_matrix_with_vector_nxn_att(att_pkt->pCbn, 3, 3, f_b_used, f_n);       //fn = Cbn*fb
            f_n_hat[0] = 0.0;
            f_n_hat[1] = 0.0;
            f_n_hat[2] = -Norm_Gravity_ATT;
            make_skew_symmetric_matrix_att(f_n_hat, Fn1);

            Hk3[0][0] = Fn1[0][0]; Hk3[0][1] = Fn1[0][1]; Hk3[0][2] = Fn1[0][2];
            Hk3[1][0] = Fn1[1][0]; Hk3[1][1] = Fn1[1][1]; Hk3[1][2] = Fn1[1][2];
            Hk3[2][0] = Fn1[2][0]; Hk3[2][1] = Fn1[2][1]; Hk3[2][2] = Fn1[2][2];

            Zk3[0] = f_n[0]-f_n_hat[0];
            Zk3[1] = f_n[1]-f_n_hat[1];
            Zk3[2] = f_n[2]-f_n_hat[2];

            diag2_sq_att(accl_accu, 3, pRk3);
            multiply_matrices_nxn_float_att(att_pkt->pCbn, pRk3, 3, 3, 3, pCbnRk3);
            matrix_transpose_nxn_float_att(att_pkt->pCbn, 3, 3, pCbnT);
            multiply_matrices_nxn_float_att(pCbnRk3, pCbnT, 3, 3, 3, pRk3);   //Rk=Cbn*Rk*Cbn'
            multiply_matrix_with_vector_nxn_att(pHk3, 3, 6, att_pkt->xk, Hk3xk);       //Hx = H*x

            inno3[0] = Zk3[0] - Hk3xk[0];
            inno3[1] = Zk3[1] - Hk3xk[1];
            inno3[2] = Zk3[2] - Hk3xk[2];

            /*printf("\nP matrix:\n");
             printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pP[0][0], att_pkt->pP[0][1], att_pkt->pP[0][2], att_pkt->pP[0][3], att_pkt->pP[0][4], att_pkt->pP[0][5]);
             printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pP[1][0], att_pkt->pP[1][1], att_pkt->pP[1][2], att_pkt->pP[1][3], att_pkt->pP[1][4], att_pkt->pP[1][5]);
             printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pP[2][0], att_pkt->pP[2][1], att_pkt->pP[2][2], att_pkt->pP[2][3], att_pkt->pP[2][4], att_pkt->pP[2][5]);
             printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pP[3][0], att_pkt->pP[3][1], att_pkt->pP[3][2], att_pkt->pP[3][3], att_pkt->pP[3][4], att_pkt->pP[3][5]);
             printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pP[4][0], att_pkt->pP[4][1], att_pkt->pP[4][2], att_pkt->pP[4][3], att_pkt->pP[4][4], att_pkt->pP[4][5]);
             printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pP[5][0], att_pkt->pP[5][1], att_pkt->pP[5][2], att_pkt->pP[5][3], att_pkt->pP[5][4], att_pkt->pP[5][5]);
             printf("\nHk3 matrix:\n");
             printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pHk3[0][0], pHk3[0][1], pHk3[0][2], pHk3[0][3], pHk3[0][4], pHk3[0][5]);
             printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pHk3[1][0], pHk3[1][1], pHk3[1][2], pHk3[1][3], pHk3[1][4], pHk3[1][5]);
             printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pHk3[2][0], pHk3[2][1], pHk3[2][2], pHk3[2][3], pHk3[2][4], pHk3[2][5]);
             printf("\nR matrix:\n");
             printf("%-.6f %-.6f %-.6f \n", pRk3[0][0], pRk3[0][1], pRk3[0][2]);
             printf("%-.6f %-.6f %-.6f \n", pRk3[1][0], pRk3[1][1], pRk3[1][2]);
             printf("%-.6f %-.6f %-.6f \n", pRk3[2][0], pRk3[2][1], pRk3[2][2]);
             printf("\nINNO:\n");
             printf("%-.6f %-.6f %-.6f \n", inno3[0], inno3[1], inno3[2]);*/



            KF_update1_float_att(3, att_pkt->xk, att_pkt->pP, inno3, pHk3, pRk3, NSTATE_ATT);

            /*printf("\nXK:\n");
             printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->xk[0], att_pkt->xk[1], att_pkt->xk[2], att_pkt->xk[3], att_pkt->xk[4], att_pkt->xk[5]);*/
        }

        // Magnetometer updates
        if (tag_mag)
        {
            if (MODE_MAG_UPDATE == 0)
            {
                FLOAT32 heading_mag1 = memo->heading_mag[memo->i_memo_heading_mag-1];
                KF_Update_Heading_Attitude(att_pkt->xk, att_pkt->pP,att_pkt->att,att_pkt->Cbn, heading_mag1, COV_ACCURACY_HEADING);
                //float Zk1[1], inno1[1], Hk1xk[1], Rk1[1][1], *pRk1[1];
                //float heading_mag1;

                //heading_mag1 = memo->heading_mag[memo->i_memo_heading_mag-1];
                //Zk1[0] = att_pkt->att[2] - heading_mag1;   // If have misalignment, how to use???????????????????? ATT is the device heading

                //FLOAT32 Hk1[1][6], *pHk1[1];
                //ChangeArrayMemoryLayout_float(1,NSTATE_ATT,pHk1,(FLOAT32 *)Hk1,1);
                //ChangeArrayMemoryLayout_float(1,1,pRk1,(FLOAT32 *)Rk1,0);		   //Rk = diag

                ////Hk1[0] = 0.0; Hk1[1] = 0.0; Hk1[2] = -1.0;
                //float c11, c11_2, c21, c21_2, c31, denom;
                //c11 = att_pkt->Cbn[0][0];
                //c21 = att_pkt->Cbn[1][0];
                //c31 = att_pkt->Cbn[2][0];
                //c11_2 = SQR(c11);
                //c21_2 = SQR(c21);
                //denom = c11_2 + c21_2;

                //Hk1[0][0] = c11*c31/denom;
                //Hk1[0][1] = c21*c31/denom;
                //Hk1[0][2] = -1.0;

                //Rk1[0][0] = ACCURACY_HEADING_MAG * ACCURACY_HEADING_MAG;

                //multiply_matrix_with_vector_nxn(pHk1, 1, 6, att_pkt->xk, Hk1xk);       //Hx = H*x
                //inno1[0] = Zk1[0] - Hk1xk[0];

                //KF_update1_float_att(1, att_pkt->xk, att_pkt->pP, inno1, pHk1, pRk1, NSTATE_ATT);
            }

            else if (MODE_MAG_UPDATE == 1)
            {
                m_n_hat[0] = mag_n00[0];
                m_n_hat[1] = mag_n00[1];
                m_n_hat[2] = mag_n00[2];

                ChangeArrayMemoryLayout_float(3,NSTATE_ATT,pHk2,(FLOAT32 *)Hk2,1);
                ChangeArrayMemoryLayout_float(3,3,pRk2,(FLOAT32 *)Rk2,0);		   //Rk = diag
                ChangeArrayMemoryLayout_float(3,3,pCbnRk2,(FLOAT32 *)CbnRk2,0);  //Cbn*Rk
                ChangeArrayMemoryLayout_float(3,3,pCbnT,(FLOAT32 *)CbnT,0);  //Cbn'

                multiply_matrix_with_vector_nxn_att(att_pkt->pCbn, 3, 3, m_b_used, m_n);       //fn = Cbn*fb
                Zk3[0] = m_n[0]-m_n_hat[0];
                Zk3[1] = m_n[1]-m_n_hat[1];
                Zk3[2] = m_n[2]-m_n_hat[2];

                make_skew_symmetric_matrix(m_n_hat, Fn1);

                Hk2[0][0] = Fn1[0][0]; Hk2[0][1] = Fn1[0][1]; Hk2[0][2] = Fn1[0][2];
                Hk2[1][0] = Fn1[1][0]; Hk2[1][1] = Fn1[1][1]; Hk2[1][2] = Fn1[1][2];
                Hk2[2][0] = Fn1[2][0]; Hk2[2][1] = Fn1[2][1]; Hk2[2][2] = Fn1[2][2];

                diag2_sq(mag_acc, 3, pRk2);
                multiply_matrices_nxn_float(att_pkt->pCbn, pRk2, 3, 3, 3, pCbnRk2);
                matrix_transpose_nxn_float(att_pkt->pCbn, 3, 3, pCbnT);
                multiply_matrices_nxn_float(pCbnRk2, pCbnT, 3, 3, 3, pRk2);   //Rk=Cbn*Rk*Cbn'

                multiply_matrix_with_vector_nxn(pHk2, 3, 6, att_pkt->xk, Hk2xk);       //Hx = H*x

                inno3[0] = Zk3[0] - Hk2xk[0];
                inno3[1] = Zk3[1] - Hk2xk[1];
                inno3[2] = Zk3[2] - Hk2xk[2];

                KF_update1_float_att(3, att_pkt->xk, att_pkt->pP, inno3, pHk2, pRk2, NSTATE_ATT);
            }

            // -------------- Feedback for ONLY mag update ----------------
            //for(i=0; i<3; i++)
            //	phi_ang[i] = att_pkt->xk[i];
            //Rvec2Quat_att(phi_ang, qe);
            //QuatPrdct_att(qe, att_pkt->qbn, qbn);

            //for(i=0; i<4; i++)
            //	att_pkt->qbn[i] = qbn[i];
            //Quat2Dcm_att(att_pkt->qbn, att_pkt->Cbn);
            //Dcm2Euler_att(att_pkt->Cbn, att_pkt->att);

            //// attitude
            //att_pkt->xk[0] = 0.0;
            //att_pkt->xk[1] = 0.0;
            //att_pkt->xk[2] = 0.0;

            //if (IF_FEEDBACK_GYRO_BIAS_ATT)
            //{
            //	att_pkt->xk[3] = 0.0;
            //	att_pkt->xk[4] = 0.0;
            //	att_pkt->xk[5] = 0.0;
            //}

            // ============== Feedback for ONLY mag update =================
        }

        // Feedback
        // INSNavFeedBack_Attitude_float_att(att_pkt->xk, att_pkt->qbn, att_pkt->Cbn, att_pkt->att, att_pkt->bg_estimated);
        INSNavFeedBack_Attitude_float_att(att_pkt, f_b_used);

        /*printf("\nCbn matrix before Mech:\n");
         printf("%-.6f %-.6f %-.6f \n", att_pkt->Cbn[0][0], att_pkt->Cbn[0][1], att_pkt->Cbn[0][2]);
         printf("%-.6f %-.6f %-.6f \n", att_pkt->Cbn[1][0], att_pkt->Cbn[1][1], att_pkt->Cbn[1][2]);
         printf("%-.6f %-.6f %-.6f \n", att_pkt->Cbn[2][0], att_pkt->Cbn[2][1], att_pkt->Cbn[2][2]);

         printf("\nXK:\n");
         printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->xk[0], att_pkt->xk[1], att_pkt->xk[2], att_pkt->xk[3], att_pkt->xk[4], att_pkt->xk[5]);

         printf("\nqbn:\n");
         printf("%-.6f %-.6f %-.6f %-.6f \n", att_pkt->qbn[0], att_pkt->qbn[1], att_pkt->qbn[2], att_pkt->qbn[3]);

         printf("\nATT:\n");
         printf("%-.6f %-.6f %-.6f\n", att_pkt->att[0], att_pkt->att[1], att_pkt->att[2]);

         printf("\ngyro_bias:\n");
         printf("%-.6f %-.6f %-.6f\n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);*/

        /*att_pkt->bg_estimated[0] = gyro_bias[0];
         att_pkt->bg_estimated[1] = gyro_bias[1];
         att_pkt->bg_estimated[2] = gyro_bias[2];*/

        att_pkt->std_att[0] = (FLOAT32) sqrt(att_pkt->pP[0][0]);
        att_pkt->std_att[1] = (FLOAT32) sqrt(att_pkt->pP[1][1]);
        att_pkt->std_att[2] = (FLOAT32) sqrt(att_pkt->pP[2][2]);
        att_pkt->std_bg[0]  = (FLOAT32) sqrt(att_pkt->pP[3][3]);
        att_pkt->std_bg[1]  = (FLOAT32) sqrt(att_pkt->pP[4][4]);
        att_pkt->std_bg[2]  = (FLOAT32) sqrt(att_pkt->pP[5][5]);

    }
    else
    {
        if_not_first_att = 1;

        if (seed_availability)
        {
            att_pkt->bg_estimated[0] = seed_bias[0];
            att_pkt->bg_estimated[1] = seed_bias[1];
            att_pkt->bg_estimated[2] = seed_bias[2];
        }
        else
        {
            att_pkt->bg_estimated[0] = 0.0;
            att_pkt->bg_estimated[1] = 0.0;
            att_pkt->bg_estimated[2] = 0.0;
        }

        att_pkt->P[0][0] = ini_att_var_att[0] * ini_att_var_att[0];
        att_pkt->P[1][1] = ini_att_var_att[1] * ini_att_var_att[1];
        att_pkt->P[2][2] = ini_att_var_att[2] * ini_att_var_att[2];
        att_pkt->P[3][3] = ini_bg_var_att[0] * ini_bg_var_att[0];
        att_pkt->P[4][4] = ini_bg_var_att[1] * ini_bg_var_att[1];
        att_pkt->P[5][5] = ini_bg_var_att[2] * ini_bg_var_att[2];

        att_pkt->Q[0][0] = ARW_q_att[0] * ARW_q_att[0];
        att_pkt->Q[1][1] = ARW_q_att[1] * ARW_q_att[1];
        att_pkt->Q[2][2] = ARW_q_att[2] * ARW_q_att[2];
        att_pkt->Q[3][3] = q_bg_att[0] * q_bg_att[0];
        att_pkt->Q[4][4] = q_bg_att[1] * q_bg_att[1];
        att_pkt->Q[5][5] = q_bg_att[2] * q_bg_att[2];

        att_pkt->xk[0] = 0.0; att_pkt->xk[1] = 0.0; att_pkt->xk[2] = 0.0;
        att_pkt->xk[3] = 0.0; att_pkt->xk[4] = 0.0; att_pkt->xk[5] = 0.0;

        att_pkt->t0 = t_cur;
        att_pkt->t = t_cur;
        ts = 1.0/SENSORS_RATE_MOBILE;


        att_pkt->att[0] = cal_roll_att(f_b);
        att_pkt->att[1] = cal_pitch_att(f_b);
        if (flag_ini_heading == 1)
        {
            att_pkt->att[2] = ini_heading;
        }
        else if (!flag_ini_heading)
        {
            att_pkt->att[2] = 0.0;
        }
        Euler2Dcm_att(att_pkt->att, att_pkt->Cbn);
        Dcm2Quat_att(att_pkt->Cbn, att_pkt->qbn);

        att_pkt->if_vertical = 0;
        att_pkt->Cbb_1[0][0] = 0.0;  att_pkt->Cbb_1[0][1] = 0.0; att_pkt->Cbb_1[0][2] = 1.0;
        att_pkt->Cbb_1[1][0] = 0.0;  att_pkt->Cbb_1[1][1] = 1.0; att_pkt->Cbb_1[1][2] = 0.0;
        att_pkt->Cbb_1[2][0] = -1.0; att_pkt->Cbb_1[2][1] = 0.0; att_pkt->Cbb_1[2][2] = 0.0;
        att_pkt->Cbb_2[0][0] = 0.0;  att_pkt->Cbb_2[0][1] = 0.0; att_pkt->Cbb_2[0][2] = -1.0;
        att_pkt->Cbb_2[1][0] = 0.0;  att_pkt->Cbb_2[1][1] = 1.0; att_pkt->Cbb_2[1][2] = 0.0;
        att_pkt->Cbb_2[2][0] = 1.0;  att_pkt->Cbb_2[2][1] = 0.0; att_pkt->Cbb_2[2][2] = 0.0;
        ChangeArrayMemoryLayout_float_att(3,3,att_pkt->pCbb_1,(FLOAT32 *)att_pkt->Cbb_1,0);
        ChangeArrayMemoryLayout_float_att(3,3,att_pkt->pCbb_2,(FLOAT32 *)att_pkt->Cbb_2,0);

    }

    // Change Axis of w, f, and m_b
    if (att_pkt->if_vertical == 1)   // Vertical Axis     // Can be optimized by using att_pkt_if_vertical_pre
    {
        temp_b = w_b[0];
        w_b[0] =  w_b[2];
        w_b[2] = -temp_b;

        temp_b = f_b[0];
        f_b[0] =  f_b[2];
        f_b[2] = -temp_b;

        temp_b = m_b[0];
        m_b[0] =  m_b[2];
        m_b[2] = -temp_b;

        //multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_1,3, 3, w_b, v3_temp);
        //w_b[0] = v3_temp[0];
        //w_b[1] = v3_temp[1];
        //w_b[2] = v3_temp[2];
        //multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_1,3, 3, f_b, v3_temp);
        //f_b[0] = v3_temp[0];
        //f_b[1] = v3_temp[1];
        //f_b[2] = v3_temp[2];
        //multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_1,3, 3, m_b, v3_temp);
        //m_b[0] = v3_temp[0];
        //m_b[1] = v3_temp[1];
        //m_b[2] = v3_temp[2];
    }
    else if (att_pkt->if_vertical == -1)
    {
        temp_b = w_b[0];
        w_b[0] = -w_b[2];
        w_b[2] = temp_b;

        temp_b = f_b[0];
        f_b[0] = -f_b[2];
        f_b[2] = temp_b;

        temp_b = m_b[0];
        m_b[0] = -m_b[2];
        m_b[2] = temp_b;

        /*multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_2,3, 3, w_b, v3_temp);
         w_b[0] = v3_temp[0];
         w_b[1] = v3_temp[1];
         w_b[2] = v3_temp[2];
         multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_2,3, 3, f_b, v3_temp);
         f_b[0] = v3_temp[0];
         f_b[1] = v3_temp[1];
         f_b[2] = v3_temp[2];
         multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_2,3, 3, m_b, v3_temp);
         m_b[0] = v3_temp[0];
         m_b[1] = v3_temp[1];
         m_b[2] = v3_temp[2];*/
    }

}

STATUS KF_Update_Heading_Attitude(FLOAT32 *x, FLOAT32 **P,FLOAT32 att[3],FLOAT32 C_bn[3][3], FLOAT32 Heading, FLOAT32 covH)
{
    STATUS A = SUCCESS;
    const INT states = 6;
    FLOAT32 R_axis[1][1], inno_axis[1], HX_axis[1],H_RP[1][states],*pH_RP[1];
    FLOAT32 *pR_axis[1];

    FLOAT32 z[1];
    //FLOAT32 nom1, nom2, nom3,denom,denom2;
    float c11, c11_2, c21, c21_2, c31, denom;

    //Convert the 2D Static Array Memory Layout to a DOUBLE Pointer Array Memory Layout
    ChangeArrayMemoryLayout_float_att_float_mis(1,states,pH_RP,(FLOAT32 *)H_RP,1);
    ChangeArrayMemoryLayout_float_att_float_mis(1,1,pR_axis,(FLOAT32 *)R_axis,1);

    z[0] = dist_ang_rad(Heading,att[2]);

    c11 = C_bn[0][0];
    c21 = C_bn[1][0];
    c31 = C_bn[2][0];
    c11_2 = SQR(c11);
    c21_2 = SQR(c21);
    denom = c11_2 + c21_2;

    H_RP[0][0] = c11*c31/denom;
    H_RP[0][1] = c21*c31/denom;
    H_RP[0][2] = -1.0;

    multiply_matrix_with_vector_nxn_att(pH_RP,1,states,x,HX_axis);
    inno_axis[0] = z[0] - HX_axis[0];

    R_axis[0][0] = covH;

    A = KF_update1_float_att(1, x, P, inno_axis, pH_RP, pR_axis,states);

    return (A);
}


void attitude_filter_predict_att(Att_Pkt* att_pkt, DOUBLE64 t_cur, FLOAT32 w_b[3], FLOAT32 f_b[3], UINT8 flag_accel, FLOAT32 m_b[3], UINT8 flag_mag, FLOAT32 ini_heading, UINT8 flag_ini_heading, FLOAT32 seed_bias[3], UINT8 seed_availability)
{
    static UINT8 if_not_first_att = 0;
    //static Att_Pkt att_pkt;
    //static FLOAT32 gyro_bias[3];
    //static FLOAT32 Cbb_1[3][3], Cbb_2[3][3], *pCbb_1[3], *pCbb_2[3];
    FLOAT32 w_b_used[3], f_b_used[3], m_b_used[3], temp_b;

    float ts, norma;
    UINT8 tag_accel;
    //UINT8 tag_mag, neph_QS;
    FLOAT32 curG[3],f_n[3], f_n_hat[3];
    FLOAT32 F[NSTATE_ATT][NSTATE_ATT], G[NSTATE_ATT][6], PHI[NSTATE_ATT][NSTATE_ATT], Fts[NSTATE_ATT][NSTATE_ATT],  GT[6][NSTATE_ATT], GQ[NSTATE_ATT][NSTATE_ATT], GQGT[NSTATE_ATT][NSTATE_ATT];
    FLOAT32 PHIGQGT[NSTATE_ATT][NSTATE_ATT],PHIT[NSTATE_ATT][NSTATE_ATT], GQGTPHIT[NSTATE_ATT][NSTATE_ATT], PHIP[NSTATE_ATT][NSTATE_ATT], PHIPPHIT[NSTATE_ATT][NSTATE_ATT], Qk[6][6];
    FLOAT32 Rk3[3][3], CbnRk3[3][3], CbnT[3][3], Zk3[3], Fn1[3][3],inno3[3], Hk3xk[3];
    FLOAT32 EYE6[NSTATE_ATT][NSTATE_ATT], Hk3[3][NSTATE_ATT]; //, Hk2[3][NSTATE_ATT], Hk2xk[3], Rk2[3][3], CbnRk2[3][3];
    // FLOAT32 xk[NSTATE_ATT] = {0.0};
    UINT16 i;
    //UINT8 if_QS = 0;
    FLOAT32 accl_accu[3]; //, mag_acc[3];

    //FLOAT32* pCbn[3],
    FLOAT32* pF[NSTATE_ATT], *pG[NSTATE_ATT], *pPHI[NSTATE_ATT], *pFts[NSTATE_ATT], *pEYE6[NSTATE_ATT], *pGT[NSTATE_ATT];
    FLOAT32* pGQ[NSTATE_ATT], *pGQGT[NSTATE_ATT], *pPHIGQGT[NSTATE_ATT], *pPHIT[NSTATE_ATT], *pGQGTPHIT[NSTATE_ATT], *pQk[6], *pPHIP[NSTATE_ATT], *pPHIPPHIT[NSTATE_ATT];
    FLOAT32* pHk3[3], *pRk3[3], *pCbnRk3[3], *pCbnT[3]; //, *pHk2[3], *pRk2[3], *pCbnRk2[3];
    float qbn_temp[4];

    FLOAT32 xk_temp[NSTATE_ATT];
    FLOAT32* pxk_temp[NSTATE_ATT];

    //FLOAT32 v3_temp[3];

    if (if_not_first_att)
    {
        ts = (FLOAT32)(t_cur - att_pkt->t);

        if (fabs(t_cur-40.0) < 0.05)
        {
            t_cur = t_cur;
        }

        att_pkt->t = t_cur;
        if (att_pkt->t - att_pkt->t0 >= 2){
            att_pkt->availability = 1;
        } else {
            att_pkt->availability = 0;
        }

        if (!att_pkt->if_vertical)
        {
            w_b_used[0] = w_b[0];
            w_b_used[1] = w_b[1];
            w_b_used[2] = w_b[2];

            f_b_used[0] = f_b[0];
            f_b_used[1] = f_b[1];
            f_b_used[2] = f_b[2];

            m_b_used[0] = m_b[0];
            m_b_used[1] = m_b[1];
            m_b_used[2] = m_b[2];
        }
        else if (att_pkt->if_vertical == 1)   // Vertical Axis
        {
            w_b_used[0] =  w_b[2];
            f_b_used[0] =  f_b[2];
            m_b_used[0] =  m_b[2];

            w_b_used[1] =  w_b[1];
            f_b_used[1] =  f_b[1];
            m_b_used[1] =  m_b[1];

            w_b_used[2] =  -w_b[0];
            f_b_used[2] =  -f_b[0];
            m_b_used[2] =  -m_b[0];

            /*multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_1,3, 3, w_b, w_b_used);
             multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_1,3, 3, f_b, f_b_used);
             multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_1,3, 3, m_b, m_b_used);*/
        }
        else if (att_pkt->if_vertical == -1)
        {
            w_b_used[0] =  -w_b[2];
            f_b_used[0] =  -f_b[2];
            m_b_used[0] =  -m_b[2];

            w_b_used[1] =  w_b[1];
            f_b_used[1] =  f_b[1];
            m_b_used[1] =  m_b[1];

            w_b_used[2] =  w_b[0];
            f_b_used[2] =  f_b[0];
            m_b_used[2] =  m_b[0];
            /*multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_2,3, 3, w_b, w_b_used);
             multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_2,3, 3, f_b, f_b_used);
             multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_2,3, 3, m_b, m_b_used);*/
        }

        compensate_b_att(w_b_used, att_pkt->bg_estimated);
        curG[0] = w_b_used[0]*ts;
        curG[1] = w_b_used[1]*ts;
        curG[2] = w_b_used[2]*ts;

        /*printf("\nCbn matrix before Mech:\n");
         printf("%-.6f %-.6f %-.6f \n", att_pkt->Cbn[0][0], att_pkt->Cbn[0][1], att_pkt->Cbn[0][2]);
         printf("%-.6f %-.6f %-.6f \n", att_pkt->Cbn[1][0], att_pkt->Cbn[1][1], att_pkt->Cbn[1][2]);
         printf("%-.6f %-.6f %-.6f \n", att_pkt->Cbn[2][0], att_pkt->Cbn[2][1], att_pkt->Cbn[2][2]);*/


        Attitude_Mechanization_simple_float_att(curG, att_pkt->qbn, qbn_temp);
        att_pkt->qbn[0] = qbn_temp[0]; att_pkt->qbn[1] = qbn_temp[1];
        att_pkt->qbn[2] = qbn_temp[2]; att_pkt->qbn[3] = qbn_temp[3];
        Quat2Dcm_att(att_pkt->qbn, att_pkt->Cbn);


        ChangeArrayMemoryLayout_float_att(3,3,att_pkt->pCbn,(FLOAT32 *)att_pkt->Cbn,0);

        /*printf("\nCbn matrix before Mech:\n");
         printf("%-.6f %-.6f %-.6f \n", pCbn[0][0], pCbn[0][1], pCbn[0][2]);
         printf("%-.6f %-.6f %-.6f \n", pCbn[1][0], pCbn[1][1], pCbn[1][2]);
         printf("%-.6f %-.6f %-.6f \n", pCbn[2][0], pCbn[2][1], pCbn[2][2]);*/

    }
    else
    {
        if_not_first_att = 1;

        if (seed_availability)
        {
            att_pkt->bg_estimated[0] = seed_bias[0];
            att_pkt->bg_estimated[1] = seed_bias[1];
            att_pkt->bg_estimated[2] = seed_bias[2];
        }
        else
        {
            att_pkt->bg_estimated[0] = 0.0;
            att_pkt->bg_estimated[1] = 0.0;
            att_pkt->bg_estimated[2] = 0.0;
        }

        att_pkt->P[0][0] = ini_att_var_att[0] * ini_att_var_att[0];
        att_pkt->P[1][1] = ini_att_var_att[1] * ini_att_var_att[1];
        att_pkt->P[2][2] = ini_att_var_att[2] * ini_att_var_att[2];
        att_pkt->P[3][3] = ini_bg_var_att[0] * ini_bg_var_att[0];
        att_pkt->P[4][4] = ini_bg_var_att[1] * ini_bg_var_att[1];
        att_pkt->P[5][5] = ini_bg_var_att[2] * ini_bg_var_att[2];

        att_pkt->Q[0][0] = ARW_q_att[0] * ARW_q_att[0];
        att_pkt->Q[1][1] = ARW_q_att[1] * ARW_q_att[1];
        att_pkt->Q[2][2] = ARW_q_att[2] * ARW_q_att[2];
        att_pkt->Q[3][3] = q_bg_att[0] * q_bg_att[0];
        att_pkt->Q[4][4] = q_bg_att[1] * q_bg_att[1];
        att_pkt->Q[5][5] = q_bg_att[2] * q_bg_att[2];

        att_pkt->xk[0] = 0.0; att_pkt->xk[1] = 0.0; att_pkt->xk[2] = 0.0;
        att_pkt->xk[3] = 0.0; att_pkt->xk[4] = 0.0; att_pkt->xk[5] = 0.0;

        att_pkt->t0 = t_cur;
        att_pkt->t = t_cur;
        ts = 1.0/SENSORS_RATE_MOBILE;


        att_pkt->att[0] = cal_roll_att(f_b);
        att_pkt->att[1] = cal_pitch_att(f_b);
        if (flag_ini_heading == 1)
        {
            att_pkt->att[2] = ini_heading;
        }
        else if (!flag_ini_heading)
        {
            att_pkt->att[2] = 0.0;
        }
        Euler2Dcm_att(att_pkt->att, att_pkt->Cbn);
        Dcm2Quat_att(att_pkt->Cbn, att_pkt->qbn);

        att_pkt->if_vertical = 0;
        att_pkt->Cbb_1[0][0] = 0.0;  att_pkt->Cbb_1[0][1] = 0.0; att_pkt->Cbb_1[0][2] = 1.0;
        att_pkt->Cbb_1[1][0] = 0.0;  att_pkt->Cbb_1[1][1] = 1.0; att_pkt->Cbb_1[1][2] = 0.0;
        att_pkt->Cbb_1[2][0] = -1.0; att_pkt->Cbb_1[2][1] = 0.0; att_pkt->Cbb_1[2][2] = 0.0;
        att_pkt->Cbb_2[0][0] = 0.0;  att_pkt->Cbb_2[0][1] = 0.0; att_pkt->Cbb_2[0][2] = -1.0;
        att_pkt->Cbb_2[1][0] = 0.0;  att_pkt->Cbb_2[1][1] = 1.0; att_pkt->Cbb_2[1][2] = 0.0;
        att_pkt->Cbb_2[2][0] = 1.0;  att_pkt->Cbb_2[2][1] = 0.0; att_pkt->Cbb_2[2][2] = 0.0;
        ChangeArrayMemoryLayout_float_att(3,3,att_pkt->pCbb_1,(FLOAT32 *)att_pkt->Cbb_1,0);
        ChangeArrayMemoryLayout_float_att(3,3,att_pkt->pCbb_2,(FLOAT32 *)att_pkt->Cbb_2,0);

    }

}


void attitude_filter_update_accel_att(Att_Pkt* att_pkt, DOUBLE64 t_cur, FLOAT32 w_b[3], FLOAT32 f_b[3], UINT8 flag_accel, FLOAT32 m_b[3], UINT8 flag_mag, FLOAT32 ini_heading, UINT8 flag_ini_heading, FLOAT32 seed_bias[3], UINT8 seed_availability)
{
    //static UINT8 if_not_first_att = 0;
    //static Att_Pkt att_pkt;
    //static FLOAT32 gyro_bias[3];
    //static FLOAT32 Cbb_1[3][3], Cbb_2[3][3], *pCbb_1[3], *pCbb_2[3];
    FLOAT32 w_b_used[3], f_b_used[3], m_b_used[3], temp_b;

    float ts, norma;
    UINT8 tag_accel;
    //UINT8 tag_mag, neph_QS;
    FLOAT32 curG[3],f_n[3], f_n_hat[3];
    FLOAT32 F[NSTATE_ATT][NSTATE_ATT], G[NSTATE_ATT][6], PHI[NSTATE_ATT][NSTATE_ATT], Fts[NSTATE_ATT][NSTATE_ATT],  GT[6][NSTATE_ATT], GQ[NSTATE_ATT][NSTATE_ATT], GQGT[NSTATE_ATT][NSTATE_ATT];
    FLOAT32 PHIGQGT[NSTATE_ATT][NSTATE_ATT],PHIT[NSTATE_ATT][NSTATE_ATT], GQGTPHIT[NSTATE_ATT][NSTATE_ATT], PHIP[NSTATE_ATT][NSTATE_ATT], PHIPPHIT[NSTATE_ATT][NSTATE_ATT], Qk[6][6];
    FLOAT32 Rk3[3][3], CbnRk3[3][3], CbnT[3][3], Zk3[3], Fn1[3][3],inno3[3], Hk3xk[3];
    FLOAT32 EYE6[NSTATE_ATT][NSTATE_ATT], Hk3[3][NSTATE_ATT]; //, Hk2[3][NSTATE_ATT], Hk2xk[3], Rk2[3][3], CbnRk2[3][3];
    // FLOAT32 xk[NSTATE_ATT] = {0.0};
    UINT16 i;
    //UINT8 if_QS = 0;
    FLOAT32 accl_accu[3]; //, mag_acc[3];

    //FLOAT32* pCbn[3],
    FLOAT32* pF[NSTATE_ATT], *pG[NSTATE_ATT], *pPHI[NSTATE_ATT], *pFts[NSTATE_ATT], *pEYE6[NSTATE_ATT], *pGT[NSTATE_ATT];
    FLOAT32* pGQ[NSTATE_ATT], *pGQGT[NSTATE_ATT], *pPHIGQGT[NSTATE_ATT], *pPHIT[NSTATE_ATT], *pGQGTPHIT[NSTATE_ATT], *pQk[6], *pPHIP[NSTATE_ATT], *pPHIPPHIT[NSTATE_ATT];
    FLOAT32* pHk3[3], *pRk3[3], *pCbnRk3[3], *pCbnT[3]; //, *pHk2[3], *pRk2[3], *pCbnRk2[3];
    float qbn_temp[4];

    FLOAT32 xk_temp[NSTATE_ATT];
    FLOAT32* pxk_temp[NSTATE_ATT];

    // Conditions
    if (1)
    {
        accl_accu[0] = AccelAccuracy_att[0];
        accl_accu[1] = AccelAccuracy_att[1];
        accl_accu[2] = AccelAccuracy_att[2];
    }


    // Prediction
    GetSystemModel_6_att(att_pkt->Cbn, (FLOAT32* )tao_bg_att, F, G);

    ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,att_pkt->pP,(FLOAT32 *)att_pkt->P,0);   //P
    ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,att_pkt->pQ,(FLOAT32 *)att_pkt->Q,0);   //Q
    ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pF,(FLOAT32 *)F,0);		//F
    ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pG,(FLOAT32 *)G,0);		//G
    ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pPHI,(FLOAT32 *)PHI,1);	//Phi
    ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pFts,(FLOAT32 *)Fts,0);   //F*ts
    ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pEYE6,(FLOAT32 *)EYE6,1);	//I

    for (i=0; i<NSTATE_ATT; i++){
        EYE6[i][i] = 1.0;
    }

    /*printf("\nQ matrix:\n");
     printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pQ[0][0], att_pkt->pQ[0][1], att_pkt->pQ[0][2], att_pkt->pQ[0][3], att_pkt->pQ[0][4], att_pkt->pQ[0][5]);
     printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pQ[1][0], att_pkt->pQ[1][1], att_pkt->pQ[1][2], att_pkt->pQ[1][3], att_pkt->pQ[1][4], att_pkt->pQ[1][5]);
     printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pQ[2][0], att_pkt->pQ[2][1], att_pkt->pQ[2][2], att_pkt->pQ[2][3], att_pkt->pQ[2][4], att_pkt->pQ[2][5]);
     printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pQ[3][0], att_pkt->pQ[3][1], att_pkt->pQ[3][2], att_pkt->pQ[3][3], att_pkt->pQ[3][4], att_pkt->pQ[3][5]);
     printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pQ[4][0], att_pkt->pQ[4][1], att_pkt->pQ[4][2], att_pkt->pQ[4][3], att_pkt->pQ[4][4], att_pkt->pQ[4][5]);
     printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pQ[5][0], att_pkt->pQ[5][1], att_pkt->pQ[5][2], att_pkt->pQ[5][3], att_pkt->pQ[5][4], att_pkt->pQ[5][5]);
     */

    //PHI[0][0] = 0.0; PHI[0][1] = 0.0; PHI[0][2] = 0.0;  PHI[0][3] = -att_pkt->Cbn[0][0]*ts; PHI[0][4] = -att_pkt->Cbn[0][1]*ts; PHI[0][5] = -att_pkt->Cbn[0][2]*ts;
    //PHI[1][0] = 0.0; PHI[1][1] = 0.0; PHI[1][2] = 0.0;  PHI[1][3] = -att_pkt->Cbn[1][0]*ts; PHI[1][4] = -att_pkt->Cbn[1][1]*ts; PHI[1][5] = -att_pkt->Cbn[1][2]*ts;
    //PHI[2][0] = 0.0; PHI[2][1] = 0.0; PHI[2][2] = 0.0;  PHI[2][3] = -att_pkt->Cbn[2][0]*ts; PHI[2][4] = -att_pkt->Cbn[2][1]*ts; PHI[2][5] = -att_pkt->Cbn[2][2]*ts;
    //PHI[3][0] = 0.0; PHI[3][1] = 0.0; PHI[3][2] = 0.0;  PHI[3][3] = 0.0; PHI[3][4] = 0.0; PHI[3][5] = 0.0;
    //PHI[4][0] = 0.0; PHI[4][1] = 0.0; PHI[4][2] = 0.0;  PHI[4][3] = 0.0; PHI[4][4] = 0.0; PHI[4][5] = 0.0;
    //PHI[5][0] = 0.0; PHI[5][1] = 0.0; PHI[5][2] = 0.0;  PHI[5][3] = 0.0; PHI[5][4] = 0.0; PHI[5][5] = 0.0;
    //ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pPHI,(FLOAT32 *)PHI,0);     //G'


    ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pGT,(FLOAT32 *)GT,0);     //G'
    ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pGQ,(FLOAT32 *)GQ,0);		//GQ
    ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pGQGT,(FLOAT32 *)GQGT,0);  //GQG'
    ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pPHIGQGT,(FLOAT32 *)PHIGQGT,0);     //Phi*GQG'
    ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pPHIT,(FLOAT32 *)PHIT,0);  //Phi'
    ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pGQGTPHIT,(FLOAT32 *)GQGTPHIT,0);	  //GQG'*Phi'
    ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pQk,(FLOAT32 *)Qk,0);		 //Qk  = 0.5*(Phi*GQGt + GQGt*Phi') * ts;
    ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pPHIP,(FLOAT32 *)PHIP,0);			  //Phi*P
    ChangeArrayMemoryLayout_float_att(NSTATE_ATT,NSTATE_ATT,pPHIPPHIT,(FLOAT32 *)PHIPPHIT,0);	  //Phi*P*Phi'


    multiply_scalar_with_matrix_nxn_att(ts, pF, pFts,NSTATE_ATT, NSTATE_ATT);		//Fts = F*ts
    add_matrices_nxn_att(pEYE6, pFts, NSTATE_ATT, NSTATE_ATT, pPHI);				//Phi = I + F*ts
    matrix_transpose_nxn_float_att(pG, NSTATE_ATT, 6, pGT);						//G' = G'
    multiply_matrices_nxn_float_att(pG, att_pkt->pQ, NSTATE_ATT, 6, 6, pGQ);				//GQ = G*Q
    multiply_matrices_nxn_float_att(pGQ, pGT, NSTATE_ATT, 6, NSTATE_ATT, pGQGT);	//GQG' = G*Q*G'
    multiply_matrices_nxn_float_att(pPHI, pGQGT, NSTATE_ATT, NSTATE_ATT, NSTATE_ATT, pPHIGQGT);	//PhiGQG' = Phi*GQG'
    matrix_transpose_nxn_float_att(pPHI, NSTATE_ATT, NSTATE_ATT, pPHIT);			//PHI'
    multiply_matrices_nxn_float_att(pGQGT, pPHIT, NSTATE_ATT, NSTATE_ATT, NSTATE_ATT, pGQGTPHIT);	//GQG'Phi' = GQG'*Phi';
    add_matrices_nxn_att(pPHIGQGT, pGQGTPHIT, NSTATE_ATT, NSTATE_ATT, pQk);         //Qk = Phi*GQGt + GQGt*Phi'
    multiply_scalar_with_matrix_nxn_att(0.5*ts, pQk, pQk, NSTATE_ATT, NSTATE_ATT);  //Qk = 0.5*ts * Qk

    multiply_matrices_nxn_float_att(pPHI, att_pkt->pP, NSTATE_ATT, NSTATE_ATT, NSTATE_ATT, pPHIP);			//Phi*P
    multiply_matrices_nxn_float_att(pPHIP, pPHIT, NSTATE_ATT, NSTATE_ATT, NSTATE_ATT, pPHIPPHIT);	//Phi*P*Phi'
    add_matrices_nxn_att(pPHIPPHIT, pQk, NSTATE_ATT, NSTATE_ATT, att_pkt->pP);				//P = Phi*P*Phi'+ Qk

    ChangeArrayMemoryLayout_float_att(NSTATE_ATT,1,att_pkt->pxk,(FLOAT32 *)att_pkt->xk,0);   //xk
    ChangeArrayMemoryLayout_float_att(NSTATE_ATT,1,pxk_temp,(FLOAT32 *)xk_temp,0);   //xk
    multiply_matrices_nxn_float_att(pPHI, att_pkt->pxk, NSTATE_ATT, NSTATE_ATT, 1, pxk_temp);	//Phi*xk
    CopyMatrix_N_Dimension_att(pxk_temp, NSTATE_ATT, 1, att_pkt->pxk);

    //printf("\nPHI matrix:\n");
    //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pPHI[0][0], pPHI[0][1], pPHI[0][2], pPHI[0][3], pPHI[0][4], pPHI[0][5]);
    //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pPHI[1][0], pPHI[1][1], pPHI[1][2], pPHI[1][3], pPHI[1][4], pPHI[1][5]);
    //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pPHI[2][0], pPHI[2][1], pPHI[2][2], pPHI[2][3], pPHI[2][4], pPHI[2][5]);
    //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pPHI[3][0], pPHI[3][1], pPHI[3][2], pPHI[3][3], pPHI[3][4], pPHI[3][5]);
    //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pPHI[4][0], pPHI[4][1], pPHI[4][2], pPHI[4][3], pPHI[4][4], pPHI[4][5]);
    //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pPHI[5][0], pPHI[5][1], pPHI[5][2], pPHI[5][3], pPHI[5][4], pPHI[5][5]);

    //printf("\nQk matrix:\n");
    //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pQk[0][0], pQk[0][1], pQk[0][2], pQk[0][3], pQk[0][4], pQk[0][5]);
    //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pQk[1][0], pQk[1][1], pQk[1][2], pQk[1][3], pQk[1][4], pQk[1][5]);
    //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pQk[2][0], pQk[2][1], pQk[2][2], pQk[2][3], pQk[2][4], pQk[2][5]);
    //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pQk[3][0], pQk[3][1], pQk[3][2], pQk[3][3], pQk[3][4], pQk[3][5]);
    //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pQk[4][0], pQk[4][1], pQk[4][2], pQk[4][3], pQk[4][4], pQk[4][5]);
    //printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pQk[5][0], pQk[5][1], pQk[5][2], pQk[5][3], pQk[5][4], pQk[5][5]);



    // UPDATE
    norma = sqrt(f_b_used[0]*f_b_used[0]+f_b_used[1]*f_b_used[1]+f_b_used[2]*f_b_used[2]) - Norm_Gravity_ATT;

    tag_accel = 1;
//#define TH_LARGE_ACC 10.0
    if (!flag_accel || fabs(norma) > TH_LARGE_ACC)
    {
        tag_accel = 0;
    }

    // Accelerometer updates
    if (tag_accel)
    {
        ChangeArrayMemoryLayout_float_att(3,NSTATE_ATT,pHk3,(FLOAT32 *)Hk3,1);
        ChangeArrayMemoryLayout_float_att(3,3,pRk3,(FLOAT32 *)Rk3,0);		   //Rk = diag
        ChangeArrayMemoryLayout_float_att(3,3,pCbnRk3,(FLOAT32 *)CbnRk3,0);  //Cbn*Rk
        ChangeArrayMemoryLayout_float_att(3,3,pCbnT,(FLOAT32 *)CbnT,0);  //Cbn'

        /*printf("\nCbn matrix:\n");
         printf("%-.6f %-.6f %-.6f \n", pCbn[0][0], pCbn[0][1], pCbn[0][2]);
         printf("%-.6f %-.6f %-.6f \n", pCbn[1][0], pCbn[1][1], pCbn[1][2]);
         printf("%-.6f %-.6f %-.6f \n", pCbn[2][0], pCbn[2][1], pCbn[2][2]);*/


        multiply_matrix_with_vector_nxn_att(att_pkt->pCbn, 3, 3, f_b_used, f_n);       //fn = Cbn*fb
        f_n_hat[0] = 0.0;
        f_n_hat[1] = 0.0;
        f_n_hat[2] = -Norm_Gravity_ATT;
        make_skew_symmetric_matrix_att(f_n_hat, Fn1);

        Hk3[0][0] = Fn1[0][0]; Hk3[0][1] = Fn1[0][1]; Hk3[0][2] = Fn1[0][2];
        Hk3[1][0] = Fn1[1][0]; Hk3[1][1] = Fn1[1][1]; Hk3[1][2] = Fn1[1][2];
        Hk3[2][0] = Fn1[2][0]; Hk3[2][1] = Fn1[2][1]; Hk3[2][2] = Fn1[2][2];

        Zk3[0] = f_n[0]-f_n_hat[0];
        Zk3[1] = f_n[1]-f_n_hat[1];
        Zk3[2] = f_n[2]-f_n_hat[2];

        diag2_sq_att(accl_accu, 3, pRk3);
        multiply_matrices_nxn_float_att(att_pkt->pCbn, pRk3, 3, 3, 3, pCbnRk3);
        matrix_transpose_nxn_float_att(att_pkt->pCbn, 3, 3, pCbnT);
        multiply_matrices_nxn_float_att(pCbnRk3, pCbnT, 3, 3, 3, pRk3);   //Rk=Cbn*Rk*Cbn'
        multiply_matrix_with_vector_nxn_att(pHk3, 3, 6, att_pkt->xk, Hk3xk);       //Hx = H*x

        inno3[0] = Zk3[0] - Hk3xk[0];
        inno3[1] = Zk3[1] - Hk3xk[1];
        inno3[2] = Zk3[2] - Hk3xk[2];

        /*printf("\nP matrix:\n");
         printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pP[0][0], att_pkt->pP[0][1], att_pkt->pP[0][2], att_pkt->pP[0][3], att_pkt->pP[0][4], att_pkt->pP[0][5]);
         printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pP[1][0], att_pkt->pP[1][1], att_pkt->pP[1][2], att_pkt->pP[1][3], att_pkt->pP[1][4], att_pkt->pP[1][5]);
         printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pP[2][0], att_pkt->pP[2][1], att_pkt->pP[2][2], att_pkt->pP[2][3], att_pkt->pP[2][4], att_pkt->pP[2][5]);
         printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pP[3][0], att_pkt->pP[3][1], att_pkt->pP[3][2], att_pkt->pP[3][3], att_pkt->pP[3][4], att_pkt->pP[3][5]);
         printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pP[4][0], att_pkt->pP[4][1], att_pkt->pP[4][2], att_pkt->pP[4][3], att_pkt->pP[4][4], att_pkt->pP[4][5]);
         printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->pP[5][0], att_pkt->pP[5][1], att_pkt->pP[5][2], att_pkt->pP[5][3], att_pkt->pP[5][4], att_pkt->pP[5][5]);
         printf("\nHk3 matrix:\n");
         printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pHk3[0][0], pHk3[0][1], pHk3[0][2], pHk3[0][3], pHk3[0][4], pHk3[0][5]);
         printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pHk3[1][0], pHk3[1][1], pHk3[1][2], pHk3[1][3], pHk3[1][4], pHk3[1][5]);
         printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", pHk3[2][0], pHk3[2][1], pHk3[2][2], pHk3[2][3], pHk3[2][4], pHk3[2][5]);
         printf("\nR matrix:\n");
         printf("%-.6f %-.6f %-.6f \n", pRk3[0][0], pRk3[0][1], pRk3[0][2]);
         printf("%-.6f %-.6f %-.6f \n", pRk3[1][0], pRk3[1][1], pRk3[1][2]);
         printf("%-.6f %-.6f %-.6f \n", pRk3[2][0], pRk3[2][1], pRk3[2][2]);
         printf("\nINNO:\n");
         printf("%-.6f %-.6f %-.6f \n", inno3[0], inno3[1], inno3[2]);*/



        KF_update1_float_att(3, att_pkt->xk, att_pkt->pP, inno3, pHk3, pRk3, NSTATE_ATT);

        /*printf("\nXK:\n");
         printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->xk[0], att_pkt->xk[1], att_pkt->xk[2], att_pkt->xk[3], att_pkt->xk[4], att_pkt->xk[5]);*/
    }

    // Feedback
    // INSNavFeedBack_Attitude_float_att(att_pkt->xk, att_pkt->qbn, att_pkt->Cbn, att_pkt->att, att_pkt->bg_estimated);
    INSNavFeedBack_Attitude_float_att(att_pkt, f_b);

    /*printf("\nCbn matrix before Mech:\n");
     printf("%-.6f %-.6f %-.6f \n", att_pkt->Cbn[0][0], att_pkt->Cbn[0][1], att_pkt->Cbn[0][2]);
     printf("%-.6f %-.6f %-.6f \n", att_pkt->Cbn[1][0], att_pkt->Cbn[1][1], att_pkt->Cbn[1][2]);
     printf("%-.6f %-.6f %-.6f \n", att_pkt->Cbn[2][0], att_pkt->Cbn[2][1], att_pkt->Cbn[2][2]);

     printf("\nXK:\n");
     printf("%-.6f %-.6f %-.6f %-.6f %-.6f %-.6f\n", att_pkt->xk[0], att_pkt->xk[1], att_pkt->xk[2], att_pkt->xk[3], att_pkt->xk[4], att_pkt->xk[5]);

     printf("\nqbn:\n");
     printf("%-.6f %-.6f %-.6f %-.6f \n", att_pkt->qbn[0], att_pkt->qbn[1], att_pkt->qbn[2], att_pkt->qbn[3]);

     printf("\nATT:\n");
     printf("%-.6f %-.6f %-.6f\n", att_pkt->att[0], att_pkt->att[1], att_pkt->att[2]);

     printf("\ngyro_bias:\n");
     printf("%-.6f %-.6f %-.6f\n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);*/

    /*att_pkt->bg_estimated[0] = gyro_bias[0];
     att_pkt->bg_estimated[1] = gyro_bias[1];
     att_pkt->bg_estimated[2] = gyro_bias[2];*/

    att_pkt->std_att[0] = (FLOAT32) sqrt(att_pkt->pP[0][0]);
    att_pkt->std_att[1] = (FLOAT32) sqrt(att_pkt->pP[1][1]);
    att_pkt->std_att[2] = (FLOAT32) sqrt(att_pkt->pP[2][2]);
    att_pkt->std_bg[0]  = (FLOAT32) sqrt(att_pkt->pP[3][3]);
    att_pkt->std_bg[1]  = (FLOAT32) sqrt(att_pkt->pP[4][4]);
    att_pkt->std_bg[2]  = (FLOAT32) sqrt(att_pkt->pP[5][5]);



    // Change Axis of w, f, and m_b
    if (att_pkt->if_vertical == 1)   // Vertical Axis     // Can be optimized by using att_pkt_if_vertical_pre
    {
        temp_b = w_b[0];
        w_b[0] =  w_b[2];
        w_b[2] = -temp_b;

        temp_b = f_b[0];
        f_b[0] =  f_b[2];
        f_b[2] = -temp_b;

        temp_b = m_b[0];
        m_b[0] =  m_b[2];
        m_b[2] = -temp_b;

        //multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_1,3, 3, w_b, v3_temp);
        //w_b[0] = v3_temp[0];
        //w_b[1] = v3_temp[1];
        //w_b[2] = v3_temp[2];
        //multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_1,3, 3, f_b, v3_temp);
        //f_b[0] = v3_temp[0];
        //f_b[1] = v3_temp[1];
        //f_b[2] = v3_temp[2];
        //multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_1,3, 3, m_b, v3_temp);
        //m_b[0] = v3_temp[0];
        //m_b[1] = v3_temp[1];
        //m_b[2] = v3_temp[2];
    }
    else if (att_pkt->if_vertical == -1)
    {
        temp_b = w_b[0];
        w_b[0] = -w_b[2];
        w_b[2] = temp_b;

        temp_b = f_b[0];
        f_b[0] = -f_b[2];
        f_b[2] = temp_b;

        temp_b = m_b[0];
        m_b[0] = -m_b[2];
        m_b[2] = temp_b;

        /*multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_2,3, 3, w_b, v3_temp);
         w_b[0] = v3_temp[0];
         w_b[1] = v3_temp[1];
         w_b[2] = v3_temp[2];
         multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_2,3, 3, f_b, v3_temp);
         f_b[0] = v3_temp[0];
         f_b[1] = v3_temp[1];
         f_b[2] = v3_temp[2];
         multiply_matrix_with_vector_nxn_att(att_pkt->pCbb_2,3, 3, m_b, v3_temp);
         m_b[0] = v3_temp[0];
         m_b[1] = v3_temp[1];
         m_b[2] = v3_temp[2];*/
    }

}
// ========== End of Function Attitude and Misalignment ===============

// =============================== C SOURCE ===================================
// ============================================================================


