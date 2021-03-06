// Filename:									perception_array.cpp
// Creation Date:							03/25/2022
// Last Revision Date:
// Author(s) [email]:						Shaede Perzanowski [sperzanowski1@lssu.edu]	Taylor Lamorie [tlamorie@lssu]
//													Brad Hacker [bhacker@lssu.edu]
// Revisor(s) [Revision Date]:		Brad Hacker [04/07/22]
// Organization/Institution:			Team AMORE / RobotX Club - Lake Superior State University

//...................................................About perception_array.cpp.....................................................................
// The perception_array.cpp file is used to access image data from the on-board camera sensors of the USV. 
// The program uses OpenCV. 

// Inputs and Outputs
//				Inputs: Camera sensor data from forward facing cameras, state command from mission_control, USV
//							pose from mission_control, and data from OpenCV
//				Outputs: Detected object data to mission_control, target data to weapon_system,
//							report to mission_control, data to OpenCV

//...................................................About the sensors.....................................................................
// Cameras: Two, forward-facing RGB cameras. Update rate is 30 Hz. Our estimated focal length of the camera's
//					used is 845 [pixels]. Image height is ___, and image width is ___. The camera frame is _____
//					relative to the USV frame.
// LiDAR: Single, 16-beam LiDAR sensor. FoV is ____. Update rate is 10 Hz. The LiDAR frame is ___ relative to
//				the USV frame.


//................................................Included Libraries and Message Types..........................................
#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "iostream"
#include "stdio.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "geometry_msgs/PointStamped.h"
#include "geographic_msgs/GeoPoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "time.h"
#include <vector>
#include <string>
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "vrx_gazebo/Task.h"
#include "amore/NED_waypoints.h"
#include "amore/state_msg.h"												// message type used to recieve state of operation from mission_control
#include "amore/usv_pose_msg.h"
#include "amore/NED_objects.h"
//...........................................End of Included Libraries and Message Types....................................


//..............................................................Namespaces................................................................
using namespace cv;
using namespace std;
//........................................................End of Namespaces...........................................................


//.................................................................Constants..................................................................
#define BASELINE 0.2							// baseline between camera sensors [m]
#define Umax 1279.0								// maximum pixel index for u (a pictures's x)
#define Umin 0.0									// minimum pixel index for u (a pictures's x)
#define PHI -33.724223							// latitude of ENU origin
#define LAMBDA 150.679736					// longitude of ENU origin
#define R11 -0.489690849						//_	The rotation matrix from ENU to ECEF
#define	R12 -0.484073338						//  |
#define	R13 -0.725172997						//	|
#define	R21 -0.871896136						//	|
#define	R22 0.271874452						//	|
#define	R23 0.407285416						//	|		
#define	R31 0.0										//	|
#define	R32 0.831719476						//	|
#define	R33 -0.555196104						//_|
#define PI 3.14159265							// Our pi, 8 decimal places
#define MbCMIN 0.01942425					// Marker buoy minimum compactness value
#define MbCMAX 0.03975825					// Marker buoy maximum compactness value
#define MbRMIN 1.50754025					// Marker buoy minimum extreme point ratio
#define MbRMAX 1.74948625					// Marker buoy maximum extreme point ratio
#define MbHead "mb_marker_buoy_"		// Marker buoy ID header
#define RbCMIN 0.065152						// Round buoy minimum compactness value
#define RbCMAX 0.071328						// Round buoy maximum compactness value
#define RbRMIN 1.016995375					// Round buoy minimum extreme point ratio
#define RbRMAX 1.342738375				// Round buoy maximum extreme point ratio
#define RbHead "mb_round_buoy_"			// Round buoy ID header
//............................................................End of Constants.............................................................


//..............................................................Global Variables............................................................
int loop_count = 0;                                    				// loop counter, first 10 loops used to intitialize subscribers
bool system_initialized = false;								// false means the system has not been initialized

//	STATES CONCERNED WITH "navigation_array"
int NA_state = 0;
//	0 = On standby
//	1 = USV NED pose converter
//	2 = Station-Keeping NED goal pose converter
//	3 = Wayfinding NED goal pose converter
//	4 = Wildlife NED animals converter

//	STATES CONCERNED WITH "path_planner"
int PP_state = 0;
//	0 = On standby
//	1 = Task 1: Station-Keeping
//	2 = Task 2: Wayfinding
//	4 = Task 4: Wildlife Encounter and Avoid
//	5 = Task 5: Channel Navigation, Acoustic Beacon Localization and Obstacle Avoidance
//	6 = Task 6: Scan and Dock and Deliver
//	STATES CONCERNED WITH "propulsion_system"
int PS_state = 0;
//	0 = On standby
//	1 = Propulsion system ON

//	STATES CONCERNED WITH "perception_array"
int PA_state = 0;
//	0 = On standby
//	1 = General State
//	3 = Task 3: Landmark Localization and Characterization
//	4 = Task 4: Wildlife Encounter and Avoid
//	5 = Task 5: Channel Navigation, Acoustic Beacon Localization and Obstacle Avoidance
//	6 = Task 6: Scan and Dock and Deliver

geographic_msgs::GeoPoseStamped task3_message; //publisher message type
ros::Publisher task3_pub; //publisher for judges topic

//below is the hsv ranges for each color default lighting
// Color limits used in blob detection
cv::Scalar green_low, green_high; 
cv::Scalar red_low, red_high; 
cv::Scalar red_low1 = cv::Scalar(0, 185, 54); 
cv::Scalar red_high1 = cv::Scalar(1, 255, 255); 
cv::Scalar orange_low, orange_high; 
cv::Scalar white_low, white_high;
cv::Scalar black_low, black_high;

// Disturbance limits 
cv::Scalar fog_background_low, fog_background_high;
cv::Scalar fog_background1_low, fog_background1_high;
cv::Scalar am_background_low, am_background_high;

cv::Mat mask, mask_new, mask_new1;

Mat background, background1;

// Mask images for each colour and camera
cv::Mat red_mask, red_mask1, orange_mask, green_mask, black_mask, white_mask;
cv::Mat red_mask_r, red_mask1_r, orange_mask_r, green_mask_r, black_mask_r, white_mask_r;

// Mask images for visual disturbances
cv::Mat fog_background_mask, fog_background1_mask, am_background_mask;

// Used to apply the colored mask to the original image
cv::Mat red_res, orange_res, green_res, black_res, white_res;
cv::Mat red_res_r, orange_res_r, green_res_r, black_res_r, white_res_r;

// For drawing contrours around the objects
std::vector <std::vector <cv::Point>> contours, contours1, contours2;

// Original images from cameras
cv::Mat org_img, org_img1, org_img2;

// Height and width of image 
float height, width;

// Buoy classification variables
float compactness, area, perimeter;
float compactness1, area1, perimeter1;
float Ry, Ry1;																// Ratio of extreme points of object

// Buoy types
int reg_buoy, circle_buoy, buoy_total_L, buoy_total_R; 

int color_red = 0;
int color_green = 0;
int color_orange = 0;
int color_black = 0;
int color_white = 0;

string color_type_buoy = " ";
string color_type_buoy1 = " ";

// Distances in x and y for each color
float x_red, y_red;
float x_green, y_green;
float x_orange, y_orange;
float x_black, y_black;
float x_white, y_white;

int size_red, size_green, size_orange, size_black, size_white, size_mask_t, size_mask_t1;

// Two vectors to hold the centroids of the BLOBs
vector<Point2f> MC(100);			// For left image
vector<Point2f> MC1(100);		// For right image

// Look-up-table for finding accurate lateral translation of buoy from the USV
float lat_scale_LUT[10][9] =	{// 7m     8m     9m     10m    11m    12m   13m    14m    15m
												{1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000}, // 0m
												{1.838, 1.610, 1.439, 1.290, 1.176, 1.075, 1.000, 0.930, 0.735}, // 1m		columns are forward, longitudinal translation
												{0.947, 0.826, 0.727, 0.658, 0.597, 0.543, 1.000, 0.468, 0.431}, // 2m
												{0.624, 0.544, 0.489, 0.438, 0.397, 0.365, 1.000, 0.313, 0.287}, // 3m		rows are lateral translation, either direction
												{0.472, 0.410, 0.363, 0.325, 0.295, 0.273, 1.000, 0.233, 0.218}, // 4m
												{1.000, 0.331, 0.291, 0.261, 0.236, 0.214, 1.000, 0.186, 0.175}, // 5m
												{1.000, 1.000, 0.245, 0.218, 0.196, 0.181, 1.000, 0.153, 0.143}, // 6m
												{1.000, 1.000, 1.000, 0.188, 0.171, 0.156, 1.000, 0.131, 0.122}, // 7m
												{1.000, 1.000, 1.000, 1.000, 0.147, 0.135, 1.000, 0.114, 0.106}, // 8m
												{1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 0.101, 0.097}  // 9m
};

amore::state_msg state_pa;													// The state command from mission_control

float u_x[100], u_x1[100];														// The x-coordinates of the BLOBs [pixels]
float v_y[100], v_y1[100];														// The y-coordinates of the BLOBs [pixels]
float x_offset[100], y_offset[100];											// Arrays for the centroids of detected objects [m]
int colors[100];																		// Array for buoy color IDs - 1: RED; 2: GREEN; 3: ORANGE; 4: WHITE; 5: BLACK
char typers[100];																	// Array for buoy type IDs - 'm' = mb_marker_buoy_; 'r' = mb_round_buoy_
float N_USV, E_USV, D_USV, PSI_USV;							// USV position and heading in NED
float O_N[100], O_E[100], O_D[100];									// Object global NED position
float new_lat[100], new_long[100];										// Determined lat and long
int right_blob_cnt = 0;																// Holds the count of BLOBs from left camera
bool my_key = true;                           									// If my_key = false, this means according to the current task status, conversion shouldn't be done

float LX, LY, RX, RY;																// Variables for the coordinates of the buoys w.r.t USV

std_msgs::Bool pa_initialization_status;								// "pa_initialization_state" message
ros::Publisher pa_initialization_state_pub;							// "pa_initialization_state" publisher

amore::NED_objects NED_buoys_msg;								// message used to hold and publish buoy locations
ros::Publisher NED_buoys_pub;											// "NED_buoys" publisher

ros::Time current_time, last_time;										// creates time variables
//........................................................End of Global Variables........................................................

//..................................................................Functions.................................................................
// THIS FUNCTION: Updates global current_time, loop_count, and publishes initialization status to "pa_initialization_state"
// ACCEPTS: (VOID)
// RETURNS: (VOID)
// =============================================================================
void PERCEPTION_ARRAY_inspector()
{
	current_time = ros::Time::now();   		// sets current_time to the time it is now
	loop_count += 1;									// increment loop counter
	if (loop_count > 5)
	{
		system_initialized = true;
		//ROS_INFO("navigation_array_initialized -- NA");
	}
	else
	{
		system_initialized = false;
		ROS_INFO("!navigation_array_initialized -- NA");
	}
	pa_initialization_status.data = system_initialized;
	pa_initialization_state_pub.publish(pa_initialization_status);						// publish the initialization status of the perception_array to "pa_initialization_state"
} // END OF PERCEPTION_ARRAY_inspector()

/////////////////////////////////////////////////////////////////		STATE UPDATERS		///////////////////////////////////////////////////////////////////
// THIS FUNCTION: Updates the state of "navigation_array" given by "mission_control"
// ACCEPTS: navigation_array state_msg from "na_state"
// RETURNS: (VOID)		Updates global variables
// =============================================================================
void na_state_update(const amore::state_msg::ConstPtr& msg)
{
	if (system_initialized)
	{
		NA_state = msg->state.data;
	}
} // END OF na_state_update()

// THIS FUNCTION: Updates the state of "path_planner" given by "mission_control"
// ACCEPTS: path_planner state_msg from "pp_state"
// RETURNS: (VOID)		Updates global variables
// =============================================================================
void pp_state_update(const amore::state_msg::ConstPtr& msg)
{
	if (system_initialized)
	{
		PP_state = msg->state.data;
	}
} // END OF pp_state_update()

// THIS FUNCTION: Updates the state of "propulsion_system" given by "mission_control"
// ACCEPTS: propulsion_system state_msg from "ps_state"
// RETURNS: (VOID)
// =============================================================================
void ps_state_update(const amore::state_msg::ConstPtr& msg) 
{
	if (system_initialized)
	{
		PS_state = msg->state.data;
	}
} // END OF ps_state_update()

// THIS FUNCTION: Updates the state of "perception_array" given by "mission_control"
// ACCEPTS: perception_array state_msg from "pa_state"
// RETURNS: (VOID) Updates global variable
// =============================================================================
void pa_state_update(const amore::state_msg::ConstPtr& msg) 
{
	if (system_initialized)
	{
		PA_state = msg->state.data;
	}
} // END OF pa_state_update()
//////////////////////////////////////////////////////////////		STATE UPDATERS END		///////////////////////////////////////////////////////////////////

// THIS FUNCTION: Obtains the USV NED pose
// ACCEPTS: The USV pose
// RETURNS: Nothing. Updates the global variables N_USV, E_USV, D_USV, PSI_USV
// =============================================================================
void pose_update(const nav_msgs::Odometry::ConstPtr& odom)
{
	if (NA_state == 1) // if navigation_array is in USV NED pose converter mode 
	{
		// Update NED USV pose 
		N_USV = odom->pose.pose.position.x;
		E_USV = odom->pose.pose.position.y;
		D_USV = odom->pose.pose.position.z;
		PSI_USV = odom->pose.pose.orientation.z;
	} // if navigation_array is in standard USV Pose Conversion mode
} // end of pose_update()

// THIS FUNCTION: Converts from the relative USV frame to spherical ECEF
// ACCEPTS: The centroid of an object and the pose of the USV
// RETURNS: Nothing. Updates the global variables new_lat and new_long
// =============================================================================
void USVtoECEF(float Bn, float Be, float Un, float Ue, float Ud, float Upsi, int t)
{
	// Variables
	// Could add Bd if the program gets improved. This would allow us to find the object in 3-space
	double r11, r12, r13, r21, r22, r23, r31, r32, r33;								// Transformation from local NED to global NED (Rz)
	float bn, be, bd, bE, bN, bU, x, y, z;													// Variables to hold object position in various frames
	
	// Populate Rz matrix
	r11 = cos(Upsi);
	r12 = -sin(Upsi);
	//r13 = 0.0;
	r21 = sin(Upsi);
	r22 = cos(Upsi);
	//r23 = 0.0;
	//r31 = 0.0;
	//r32 = 0.0;
	//r33 = 1.0;
	
	// Matrix equation to transform local x to global x_hat: x_hat = Rz*x
	bn = (r11*Bn + r12*Be) + Un;
	be = (r21*Bn + r22*Be) + Ue;
	bd = Ud;
	O_N[t] = bn;
	O_E[t] = be;
	O_D[t] = bd;
	// Convert from global NED to global ENU
	bE = bn;
	bN = be;
	bU = -Ud - 1.0;															// u-coordinate - USV's offset
	// Convert from global ENU to rectangular ECEF
	x = (R11*bE + R12*bN + R13*bU) - 4630032.0;
	y = (R21*bE + R22*bN + R23*bU) + 2600407.0;
	z = (R31*bE + R32*bN + R33*bU)  -3521046.0;
	// Convert from rectangular ECEF to spherical ECEF
	new_lat[t] = (atan2(sqrt(pow(x,2)+pow(x,2)),z)*180.0/PI)-151.9952361;
	new_long[t] = atan2(y,x)*180.0/PI;
} // end of USVtoECEF()

// THIS FUNCTION: Uses the disturbance limits and the HSV image to set new color limits and find the blobs
// ACCEPTS: An image in the HSV color space (imgHSV)
// RETURNS: Nothing. Updates global variables of color limits and of contours
// =============================================================================
void HSVFunc(cv::Mat imgHSV)
{
	// Set color limits
	green_low = cv::Scalar(59, 115, 25);  
	green_high = cv::Scalar(100, 255, 255); 
	red_low = cv::Scalar(130, 170, 60);  
	red_high = cv::Scalar(179, 255, 255); 
	red_low1 = cv::Scalar(0, 185, 54); 
	red_high1 = cv::Scalar(1, 255, 255); 
	orange_low = cv::Scalar(2, 170, 50); 
	orange_high = cv::Scalar(15, 255, 255); 
	white_low = cv::Scalar(0, 0, 80); 
	white_high = cv::Scalar(0, 0, 255); 
	black_low = cv::Scalar(0, 0, 0); 
	black_high = cv::Scalar(179, 0, 35); 
	
	// Set disturbance limits
	fog_background_low = cv::Scalar(0, 0, 200); 
	fog_background_high = cv::Scalar(0, 0, 240); 
	fog_background1_low = cv::Scalar(0, 0, 160); 
	fog_background1_high = cv::Scalar(0, 0, 240); 
	/* am_background_low = cv::Scalar(32, 130, 0); 
	am_background_high = cv::Scalar(33, 255, 255);  */
	
	// Fog scenario #1
	cv::inRange(imgHSV, fog_background_low, fog_background_high, fog_background_mask);									// Masks imgHSV based on limits, copies to fog_background_mask
	cv::findContours(fog_background_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);		// Finds contours around coloured objects and draws a rectangle around the object
	if (contours.size() >0)																																					// Set new color limits
	{
		green_low = cv::Scalar(59, 25, 0);  
		green_high = cv::Scalar(100, 255, 255); 
		red_low = cv::Scalar(130, 65, 0);  
		red_high = cv::Scalar(179, 255, 255);  
		orange_low = cv::Scalar(2, 0, 0); 
		orange_high = cv::Scalar(15, 255, 255); 
		white_low = cv::Scalar(0, 0, 95); 
		white_high = cv::Scalar(0, 0, 202); 
		black_low = cv::Scalar(0, 0, 0); 
		black_high = cv::Scalar(179, 0, 125);
	}
	// Fog scenario #2
	cv::inRange(imgHSV, fog_background1_low, fog_background1_high, fog_background1_mask);							// Masks imgHSV based on limits, copies to fog_background1_mask
	cv::findContours(fog_background1_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);	// Finds contours around coloured objects and draws a rectangle around the object
	if (contours.size() >0)																																					// Set new color limits
	{
		green_low = cv::Scalar(59, 50, 0);  
		green_high = cv::Scalar(100, 255, 255); 
		red_low = cv::Scalar(130, 60, 0);  
		red_high = cv::Scalar(179, 255, 255); 
		orange_low = cv::Scalar(2, 0, 0); 
		orange_high = cv::Scalar(15, 255, 255); 
		white_low = cv::Scalar(0, 0, 100); 
		white_high = cv::Scalar(0, 0, 150); 
		black_low = cv::Scalar(0, 0, 0); 
		black_high = cv::Scalar(179, 0, 100);
	}
	} // end of HSVFunc()

// THIS FUNCTION: 	Performs iteritive buoy ID concatination using the colors and typers ID arrays
//									Publishes the buoy IDs with ECEF locations of the buoys to "/vrx/perception/landmark"
//									Publishes the buoy IDs with local NED locations of the buoys to "NED_buoys"
// ACCEPTS: (VOID)
// RETURNS: (VOID)
// =============================================================================
void buoys_publish()
{
	int true_size = 0;
	string buoy_id = " ";
	// VARIABLES FOR SELF CREATED OBJECT ARRAY MESSAGE CONTAINING OBJECTS WITH THEIR RESPECTIVE NED POSITIONS AND IDs
	NED_buoys_msg.objects.clear();
	
	for (int i=0; i<buoy_total_L; i++)
	{
		current_time = ros::Time::now(); 							// time update
		switch(colors[i])
		{
			// 1: RED; 2: GREEN; 3: ORANGE; 4: WHITE; 5: BLACK
			case 1:
				buoy_id = "red";
				break;
			case 2:
				buoy_id = "green";
				break;
			case 3:
				buoy_id = "white";
				break;
			case 4:
				buoy_id = "orange";
				break;
			case 5:
				buoy_id = "black";
				break;
			default:
				break;
		}
		color_type_buoy = buoy_id;
		if (typers[i] == 'm')
		{
			color_type_buoy.insert(0, MbHead);											// Append MbHead to the front of color_type_buoy
		}
		else if (typers[i] == 'r')
		{
			color_type_buoy.insert(0, RbHead);											// Append RbHead to the front of color_type_buoy
		}
		
		if (PA_state == 3)
		{
			task3_message.header.seq +=1;												// sequence number
			task3_message.header.stamp = current_time;						// sets stamp to current time
			task3_message.header.frame_id = color_type_buoy.c_str(); 	// header frame
			task3_message.pose.position.latitude = new_lat[i];
			task3_message.pose.position.longitude = new_long[i];
			task3_pub.publish(task3_message);
		}
		
		
		// LUT made to for buoys within [7 - 15] m in front (NORTH) of the USV and [-9 - 9] m to the side of the USV (WEST OR EAST)
		if ((PA_state == 5) || (PA_state == 3))				// if calculated values are within expected range, put into buoy array  // ((x_offset[i]) < 15.0) && ((x_offset[i]) > 5.0) && ((y_offset[i]) < 10.0) && ((y_offset[i]) > -10.0)
		{
			geometry_msgs::PointStamped buoy;						//publisher message type
			buoy.header.seq +=1;												// sequence number
			buoy.header.stamp = current_time;							// sets stamp to current time
			buoy.header.frame_id = color_type_buoy.c_str(); 	// header frame
			buoy.point.x = x_offset[i];
			buoy.point.y = y_offset[i];
			NED_buoys_msg.objects.push_back(buoy);
			true_size += 1;
		}
		color_type_buoy = " ";
	}
	if ((PA_state == 5) || (PA_state == 3))
	{
		ROS_INFO("Printing array of buoys locations wrt USV -- PA");
		ROS_INFO("true_size: %2i", true_size);
		
		for (int j=0; j<true_size; j++)
		{
			ROS_INFO("Buoy: %s		number: %2i			x: %4.2f			y: %4.2f", NED_buoys_msg.objects[j].header.frame_id.c_str(), j, NED_buoys_msg.objects[j].point.x, NED_buoys_msg.objects[j].point.y);
		}
		NED_buoys_msg.quantity = true_size;				// publish quantity of poses so the path planner knows
		NED_buoys_pub.publish(NED_buoys_msg);		// publish left and right buoy locations, respectively, in array "NED_buoys"
	}
} // end of buoys_publish()

// THIS FUNCTION: Fills out the global buoy camera image centroids, as well as the buoy IDs (color and type)
// ACCEPTS: A counter, a color identifier, and an identifier for which camera is using the function 
// RETURNS: The counter input but updated. Updates global variables
// =============================================================================
int ClassLocFunc(int cunter, int ckey, int keyer)
{
	float y1, y2;																																		// Hold the distance values from centroid to extreme point [pixels]
	int first_counter = 0;
	
	// For LeftCamFunc() vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
	if(keyer == 1)
	{
		switch(ckey)
		{
			// 1: RED; 2: GREEN; 3: ORANGE; 4: WHITE; 5: BLACK
			case 1:
				size_red = contours.size();
				break;
			case 2:
				size_green = contours.size();
				break;
			case 3:
				size_white = contours.size();
				break;
			case 4:
				size_orange = contours.size();
				break;
			case 5:
				size_black = contours.size();
				break;
			default:
				break;
		}
		
		vector<Moments> mu(contours.size());																		// Holds the moments of the blob
		std::vector<cv::Point> pts;																							// Used in finding the extreme points
		vector<Point2f> mc(contours.size());																			// Holds the mass centroid
		
		for (int i=0; i < contours.size(); i++)
		{
			cv::Rect boundRect = cv::boundingRect(contours[i]);													// Finds and saves locations of bounding rectangles
			if ((boundRect.area() > 100) && (boundRect.width < 1000))											// Filter out small noise
			{
				/* // Draws the bounding rectangles on the image
				if (ckey == 1)		{		cv::rectangle(background, boundRect.tl(), boundRect.br(), cv::Scalar(0, 0, 255), 3);			}
				if (ckey == 2)		{		cv::rectangle(background, boundRect.tl(), boundRect.br(), cv::Scalar(0, 255, 0), 3);			}
				if (ckey == 3)		{		cv::rectangle(background, boundRect.tl(), boundRect.br(), cv::Scalar(255, 255, 255), 3);	}
				if (ckey == 4)		{		cv::rectangle(background, boundRect.tl(), boundRect.br(), cv::Scalar(0, 179, 255), 3);		}
				if (ckey == 5)		{		cv::rectangle(background, boundRect.tl(), boundRect.br(), cv::Scalar(0, 0, 0), 3);				} */
				
				mu[i] = cv::moments(contours[i], false);																	// This calculates the moments of the contours, but these change depeding on scaling
				pts = contours[i];
				mc[i] = Point2f(mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00);								// Finds the centroid of each BLOB [pixels]
				u_x[cunter+first_counter] = mc[i].x;																								// x-bar of centroid [pixels]
				v_y[cunter+first_counter] = mc[i].y;																								// y-bar of centroid [pixels]
				colors[cunter+first_counter] = ckey;																								// Orders the colors to the respective u_x, u_y centroids
				first_counter += 1;																												// Counter for the number of BLOBs
			}
		}
		
		// fill out the buoy shape type
		for( int i = 0; i<contours.size(); i++ )
		{
			cv::Rect boundRect = cv::boundingRect(contours[i]);													// Finds and saves locations of bounding rectangles
			if ((boundRect.area() > 100) && (boundRect.width < 1000))											// Filter out small noise
			{
				// Finds extreme points of buoys
				auto val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){return a.x < b.x;});
				val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){return a.y < b.y;});
				// Calculate the distances from the centroid to the extreme points
				y2 = (val.second -> y) - mc[i].y;
				y1 = mc[i].y - (val.first -> y);
				//	Buoy classification based on extreme point ratio, Ry
				Ry = y1 / y2;
				// Classify the buoys	
				area = cv::contourArea(contours[i]);																													// Area of the given BLOB. [pixels]
				perimeter = cv::arcLength(contours[i], true);																									// Perimeter of the given BLOB. [pixels]
				compactness = (area) / (pow(perimeter, 2));																									// Compactness of given BLOB
				if ((compactness >= MbCMIN & compactness <= MbCMAX) && (Ry >= MbRMIN & Ry <= MbRMAX))	// Finds marker/regular buoys
				{
					typers[cunter] = 'm';		// 'm' = mb_marker_buoy_
				}
				else if ((compactness >= RbCMIN & compactness <= RbCMAX)  && (Ry >= RbRMIN & Ry <= RbRMAX))	// Finds the round buoys
				{
					typers[cunter] = 'r';			// 'r' = mb_round_buoy_
				}
				else 
				{
					typers[cunter] = 'm';		// 'm' = mb_marker_buoy_					SET TO mb_marker_buoy_ IF NEITHER TYPE WAS DETECTED
				}
				cunter += 1;																												// Counter for the number of BLOBs
			}
		}
	}
	// For RightCamFunc() vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
	else
	{
		for (int i=0; i < contours1.size(); i++)
		{
			cv::Rect boundRect = cv::boundingRect(contours1[i]);									// Finds and saves locations of bounding rectangles
			if ((boundRect.area() > 100) && (boundRect.width < 1000))							// Filter out small noise
			{			
				vector<Moments> mu1(contours1.size());													// Holds the moments of the blob
				vector<Point2f> mc1(contours1.size());														// Holds the mass centroid
				mu1[i] = cv::moments(contours1[i], false);													// This calculates the moments of the red contours, but these change depeding on scaling
				mc1[i] = Point2f(mu1[i].m10/mu1[i].m00 , mu1[i].m01/mu1[i].m00);			// Finds the centroid of each BLOB [pixels]
				u_x1[cunter] = mc1[i].x;																				// x-bar of centroid [pixels]
				v_y1[cunter] = mc1[i].y;																				// y-bar of centroid [pixels]
				cunter += 1;			
			}
		}
	}
	return cunter;
} // end of ClassLocFunc()

// THIS FUNCTION: Finds the best focal length of the camera sensors, based off a LUT
// ACCEPTS: the estimated longitudinal translation to the object (zz)
// RETURNS: the corrected focal length (focal_length)
// =============================================================================
float fpLUT(float zz)
{
	float focal_length;
	zz = round(zz - 6.0);				// Use longitudinal distance to find entry for LUT
	switch(int(zz))
	{
		case 0:								// <7m
			focal_length = 865.0;
			break;
		case 1:								// 7m
			focal_length = 855.0;
			break;
		case 2:								// 8m
			focal_length = 846.0;
			break;
		case 3:								// 9m
			focal_length = 839.2;
			break;
		case 4:								// 10m
			focal_length = 837.0;
			break;
		case 5:								// 11m
			focal_length = 829.0;
			break;
		case 6:								// 12m
			focal_length = 826.1;
			break;
		case 7:								// 13m
			focal_length = 823.0;
			break;
		case 8:								// 14m
			focal_length = 821.0;
			break;
		case 9:								// 15m
			focal_length = 815.9;
			break;
		default:
			focal_length = 845.0;
	}
	return focal_length;
} // end of fpLUT()

// THIS FUNCTION: Determines the initial lateral gain to multiply the first lateral disparity calculation by, based off a LUT
// ACCEPTS: the approximated lateral translation to the buoy (latzz)
// RETURNS: the lateral gain (gainer)
// =============================================================================
float lateral_gainLUT(float latzz)
{
	float gainer;
	latzz = abs(round(latzz));
	// This LUT is referenced to a longitudinal translation of 13m
	switch(int(latzz))
	{
		case 0:						// in-line
			gainer = 62.5;
			break;
		case 1:						// 1m shift
			gainer = 62.5;
			break;
		case 2:						// 2m shift
			gainer = 62.5;
			break;
		case 3:						// 3m shift
			gainer = 62.5;
			break;
		case 4:						// 4m shift
			gainer = 62.5;
			break;
		case 5:						// 5m shift
			gainer = 62.5;
			break;
		case 6:						// 6m shift
			gainer = 61.86;
			break;
		case 7:						// 7m shift
			gainer = 62.5;
			break;
		case 8:						// 8m shift
			gainer = 63.0;
			break;
		case 9:						// 9m shift
			gainer = 63.38;
			break;
		default:					// in-line
			gainer = 62.5;
	}
	return gainer;
} // end of lateral_gainLUT()

// THIS FUNCTION: Determines the final lateral gain to multiply the lateral disparity calculation by, based off a LUT (lat_scale_LUT)
// ACCEPTS: the approximated lateral translation (lat_z) and longitudinal (long_z) translation to the buoy
// RETURNS: final lateral gain (lats)
// =============================================================================
float lateral_scale(float lat_z, float long_z)
{
	float lats = 1.0, lat_z_prev, error_s;
	int r, c;
	c = int(round(long_z) - 7.0);							// Use the longitudinal translation to determine the column of the LUT to look at
	if(c<0)														//_ Do not let c break the LUT columns
	{																//	|
		c = 0;													//	|
	}																//	|
	if(c>8)														//	|
	{																//	|
		c = 8;													//	|
	}																//_|
	lat_z_prev = lat_z;
	lat_z = 1.0/lat_z;
	if(c==6)													// Ignore lat_scale_LUT because column 6 is filled with ones
	{
		return lats;
	}
	for(r=1;r<10;r++)										// Find and apply lateral gain from lat_scale_LUT
	{
		if(lat_z>=lat_scale_LUT[r][c])
		{
			lats = float(r)*lat_scale_LUT[r][c];
			break;
		}
	}
	return lats;
} // end of lateral_scale()

// THIS FUNCTION: Uses disparity to find the centroids of detected objects wrt local NED
// ACCEPTS: Nothing. Uses the global variables left_blob_cnt and MC
// RETURNS: Nothing. Updates the global variables
// =============================================================================
void DisparityFunc()
{
	float d[buoy_total_L];																// The disparity between camera images [pixels]
	float f_p = 820.0;																	// experimental focal length [pixels]
	float z[buoy_total_L];																// Distance along USV surge to the projection of the point [m]
	float z_0;																				// Used to obtain more accurate estimate of z [m]
	float s_left[buoy_total_L], s_right[buoy_total_L];						// Distances from centroid to each image's interior vertical [pixels]
	float ds_left[buoy_total_L], ds_right[buoy_total_L];					// The "lateral disparity" for each camera image [pixels]
	float ds_lat = -1.0;																// Average of ds_left and ds_right [pixels]
	float lat_z_left[buoy_total_L], lat_z_right[buoy_total_L];			// Distance along USV sway to the projection of the point [m]
	float lat_z_avg[buoy_total_L];												// Average of lat_z_left and lat_z_right [m]
	float g_left = 62.5, g_right = 62.5;										// Initial lateral gains for cameras
	float k_lat = -1.0;																	// Final average lateral gain for cameras
	
	for(int i=0; i<buoy_total_L; i++)
	{
		d[i] = MC[i].x - MC1[i].x;												// Find longitudinal disparity [pixels]
		z_0 = (f_p*BASELINE)/d[i];											// Initial calculation of distance along USV surge to the projection of the point [m]
		f_p = fpLUT(z_0);															// Uses z_0 to tune the focal length with fpLUT() [pixels]
		z[i] = (f_p*BASELINE)/d[i];											// Improved calculation of distance along USV surge to the projection of the point [m]
		s_left[i] = MC[i].x - (Umax/2.0);									// Find distance from centroid to left image's interior vertical [pixels]
		s_right[i] = MC1[i].x - (Umax/2.0);								// Find distance from centroid to right image' interior vertical [pixels]
		ds_left[i] = (z[i]*s_left[i])/f_p;											// Find "lateral disparity" for left camera image [pixels]
		ds_right[i] = (z[i]*s_right[i])/f_p;										// Find "lateral disparity" for right camera image [pixels]
		ds_lat = (ds_left[i]+ds_right[i])/2.0;								// Find the average "lateral disparity" [pixels]
		lat_z_left[i] = g_left*((z[i]*ds_left[i])/f_p - 0.001);			// Initial finding of leftward sway distance to the projection of the point [m]
		lat_z_right[i] = g_right*((z[i]*ds_right[i])/f_p + 0.002);	// Initial finding of rightward sway distance to the projection of the point [m]
		g_left = lateral_gainLUT(lat_z_left[i]);							// Initial lateral gain of left camera using lateral_gainLUT()
		g_right = lateral_gainLUT(lat_z_right[i]);						// Initial lateral gain of right camera using lateral_gainLUT()
		lat_z_left[i] = g_left*((z[i]*ds_left[i])/f_p - 0.001);			// Improved finding of leftward sway distance to the projection of the point [m]
		lat_z_right[i] = g_right*((z[i]*ds_right[i])/f_p + 0.002);	// Improved finding of rightward sway distance to the projection of the point [m]
		lat_z_avg[i] = (lat_z_left[i] + lat_z_right[i])/2.0;				// Initial average sway distance to the projection of the point [m]
		k_lat = lateral_scale(lat_z_avg[i], z[i]);							// Final lateral gain
		lat_z_avg[i] = lat_z_avg[i]*k_lat;									// Improved average sway distance to the projection of the point [m]
		
		// Update global array of buoy centroids wrt USV in NED convention
		x_offset[i] = z[i];										// Surge (North) distance [m]
		y_offset[i] = lat_z_avg[i];						// Sway (East) distance [m]
	}
} // end of DisparityFunc()

// THIS FUNCTION: Finds the BLOBs of interest and their centroids for the left camera sensor
// ACCEPTS: The image from the left camera sensor
// RETURNS: Nothing. Manipulates many global variables
// =============================================================================
void LeftCamFunc(const sensor_msgs::ImageConstPtr& camera_msg)
{
	if (PA_state != 0) // as long as perception_array is not on standby
	{
		Mat imgHSV;																								// Holds the HSV copy of the original image
		
		try 
		{
			int counter_L = 0;
			buoy_total_L = 0;																						// Reset the buoy count
			size_red = 0;
			size_green = 0;
			size_orange = 0;
			size_black = 0;
			size_white = 0;
			size_mask_t = 0;
			
			// Convert original image to bgr8-type and make a copy in HSV color space
			org_img = cv_bridge::toCvShare(camera_msg, "bgr8") -> image;		// Converts message camera_msg into bgr8 type image
			org_img.copyTo(background);																// Copies the original image to background image
			cv::cvtColor(org_img, imgHSV, cv::COLOR_BGR2HSV);					// Copies original image into HSV color space
			HSVFunc(imgHSV);																				// Sets new color limits and finds blobs of imgHSV (contours)
			
			// Find all the BLOBs
			// Find the red BLOBs
			cv::inRange(imgHSV, red_low, red_high, red_mask); 
			cv::inRange(imgHSV, red_low1, red_high1, red_mask1); 
			red_mask = red_mask + red_mask1;													// Image containing only Red BLOBs
			
			// Find the green BLOBs
			cv::inRange(imgHSV, green_low, green_high, green_mask);
			
			// Find the white BLOBs
			cv::inRange(imgHSV, white_low, white_high, white_mask);
			
			// Find the orange BLOBs
			cv::inRange(imgHSV, orange_low, orange_high, orange_mask);
			
			// Find the black BLOBs
			cv::inRange(imgHSV, black_low, black_high, black_mask); 
			
			// Apply median filter with a kernel size of 3 to the new color images to get rid of small noise
			cv::medianBlur(red_mask, red_mask, 3);
			cv::medianBlur(green_mask, green_mask, 3);
			cv::medianBlur(white_mask, white_mask, 3);
			cv::medianBlur(orange_mask, orange_mask, 3);
			cv::medianBlur(black_mask, black_mask, 3);
			
			// Find countours to do object identification calculations with ClassLocFunc which performs the calculations to ID the buoys 
			cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
			counter_L = ClassLocFunc(counter_L, 1, 1);
			cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
			counter_L = ClassLocFunc(counter_L, 2, 1);
			cv::findContours(white_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
			counter_L = ClassLocFunc(counter_L, 3, 1);
			cv::findContours(orange_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
			counter_L = ClassLocFunc(counter_L, 4, 1);
			cv::findContours(black_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
			counter_L = ClassLocFunc(counter_L, 5, 1);
			
			buoy_total_L = counter_L;
			size_mask_t = size_red + size_green + size_white + size_orange + size_black;
			
			ROS_INFO("The buoy_total_L: %i", buoy_total_L);
			ROS_INFO("The size_mask_t: %i\n", size_mask_t);
			
			// Perform the disparity calculations if BLOBs were detected
			if (buoy_total_L == 0) // Nothing was found
			{
				//ROS_INFO("No Object Detected by Left Camera. {perception_array}");
			}
			else 
			{
				if (buoy_total_L == size_mask_t)
				{
					for (int i = 0; i < buoy_total_L; i++)
					{
						MC[i].x = u_x[i];
						MC[i].y = v_y[i];
					}
					DisparityFunc();			// Use disparity to find the centroids of detected objects wrt local NED frame 
					
					if (PA_state == 3)		// If task 3: Landmark Localization and Characterization
					{
						for (int i=0; i<buoy_total_L; i++)
						{
							// Convert centroids from local NED to spherical ECEF using USVtoECEF()
							USVtoECEF(x_offset[i], y_offset[i], N_USV, E_USV, D_USV, PSI_USV, i);
						}
					}
					buoys_publish();			// publish desired identified objects
				}
			}
			
			// NOTE: comment the following line out for the docker image, but uncomment for user display of cameras and detected buoys
			//cv::imshow("Left Camera Updated", background);
			cv::waitKey(30);			// Wait 30 milliseconds
		} // try 
		catch (cv_bridge::Exception& e) // looks for errors 
		{
			ROS_ERROR("Left Camera could not convert from '%s' to 'bgr8'. {perception_array}", camera_msg -> encoding.c_str()); //prints out the encoding string
		}
	} // if (PA_state != 0) 
} // end of LeftCamFunc()

// THIS FUNCTION: Finds the BLOBs of interest and their centroids for the right camera sensor
// ACCEPTS: The image from the left camera sensor
// RETURNS: Nothing. Manipulates many global variables
// =============================================================================
void RightCamFunc(const sensor_msgs::ImageConstPtr& camera_msg1)
{
	if (PA_state != 0) // as long as perception_array is not on standby
	{
		Mat imgHSV1;																											// Holds the HSV copy of the original image
	
		try 
		{		
			int counter_R = 0;																										// Used for placement in arrays
			buoy_total_R = 0;
			size_mask_t1 = 0;
			// Convert original image to bgr8-type and make a copy in HSV color space
			org_img1 = cv_bridge::toCvShare(camera_msg1, "bgr8") -> image;					// Converts message camera_msg into bgr8 type image
			org_img1.copyTo(background1);																			// Copies the original image to background image
			cv::cvtColor(org_img1, imgHSV1, cv::COLOR_BGR2HSV);								// Holds the final BLOB detection result
			HSVFunc(imgHSV1);																							// Sets new color limits and finds blobs of imgHSV1 (contours)
			
			// Find the red BLOBs
			cv::inRange(imgHSV1, red_low, red_high, red_mask_r); 
			cv::inRange(imgHSV1, red_low1, red_high1, red_mask1_r); 
			red_mask_r = red_mask_r + red_mask1_r;															// Image containing only Red BLOBs
			// Find the white BLOBs 
			cv::inRange(imgHSV1, white_low, white_high, white_mask_r);
			// Find the orange BLOBs
			cv::inRange(imgHSV1, orange_low, orange_high, orange_mask_r);
			// Find the black BLOBs
			cv::inRange(imgHSV1, black_low, black_high, black_mask_r);
			// Find the green BLOBs
			cv::inRange(imgHSV1, green_low, green_high, green_mask_r);
			
			// Apply median filter with a kernal size of 3 to the new color images to get rid of small noise
			cv::medianBlur(black_mask_r, black_mask_r, 3);
			cv::medianBlur(white_mask_r, white_mask_r, 3);
			cv::medianBlur(orange_mask_r, orange_mask_r, 3);
			cv::medianBlur(red_mask_r, red_mask_r, 3);
			cv::medianBlur(green_mask_r, green_mask_r, 3);
			
			cv::findContours(red_mask_r , contours1, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);			// Finds the contours of the "red" image
			counter_R = ClassLocFunc(counter_R, 1, 0);																											// Classifies the buoy and marks its centroid
			cv::findContours(green_mask_r , contours1, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);		// Finds the contours of the "green" image
			counter_R = ClassLocFunc(counter_R, 2, 0);																											// Classifies the buoy and marks its centroid
			cv::findContours(white_mask_r , contours1, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);		// Finds the contours of the "white" image
			counter_R = ClassLocFunc(counter_R, 3, 0);																											// Classifies the buoy and marks its centroid
			cv::findContours(orange_mask_r , contours1, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);		// Finds the contours of the "orange" image
			counter_R = ClassLocFunc(counter_R, 4, 0);																											// Classifies the buoy and marks its centroid
			cv::findContours(black_mask_r , contours1, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);		// Finds the contours of the "black" image
			counter_R = ClassLocFunc(counter_R, 5, 0);																											// Classifies the buoy and marks its centroid
			
			buoy_total_R = counter_R;
			//size_mask_t1 = size_red + size_green + size_white + size_orange + size_black;
			
			if (buoy_total_R == 0) // Nothing was found
			{
				//ROS_INFO("No Object Detected by Right Camera. {perception_array}");
			}
			else 
			{
				for (int i = 0; i < buoy_total_R; i++)
				{
					MC1[i].x = u_x1[i];
					MC1[i].y = v_y1[i];
				}
			}
			cv::waitKey(30);
		}
		catch (cv_bridge::Exception& e) //looks for errors 
		{
			ROS_ERROR("Right Camera could not convert from '%s' to 'bgr8'. {perception_array}", camera_msg1 -> encoding.c_str()); //prints out the encoding string
		}
	} // if (PA_state != 0) 
} // end of RightCamFunc()
//............................................................End of funcs............................................................


//...............................................................Main Program..............................................................
int main(int argc, char **argv)
{
	ros::init(argc, argv, "perception_array");
	
	// Initializations
	// NOTE: comment the following line out for the docker image, but uncomment for user display of cameras and detected buoys
	//cv::namedWindow("Left Camera Updated", cv::WINDOW_AUTOSIZE);					
	//cv::namedWindow("Left Camera Features", cv::WINDOW_AUTOSIZE);
	//cv::namedWindow("Right Camera Updated", cv::WINDOW_AUTOSIZE);
	//cv::namedWindow("Right Camera Features", cv::WINDOW_AUTOSIZE);
  
	// Subscriber NodeHandles
	ros::NodeHandle nh1, nh2, nh3, nh4, nh5, nh6, nh7;
	
	// Subscribers
	ros::Subscriber na_state_sub = nh1.subscribe("na_state", 1, na_state_update);
	ros::Subscriber pp_state_sub = nh2.subscribe("pp_state", 1, pp_state_update);
	ros::Subscriber ps_state_sub = nh3.subscribe("ps_state", 1, ps_state_update);
	ros::Subscriber pa_state_sub = nh4.subscribe("pa_state", 1, pa_state_update);
	ros::Subscriber nav_NED_sub = nh5.subscribe("nav_ned", 1, pose_update);														// Obtains the USV pose in global NED from mission_control
	image_transport::ImageTransport it1(nh6);		// transports the images from published node to subscriber
	image_transport::ImageTransport it2(nh7);
	image_transport::Subscriber camera_sub = it1.subscribe("/wamv/sensors/cameras/front_left_camera/image_raw", 1, LeftCamFunc);				// Analyzes left camera image
	image_transport::Subscriber camera_sub1 = it2.subscribe("/wamv/sensors/cameras/front_right_camera/image_raw", 1, RightCamFunc);	// Analyzes right camera image
	
	// Publisher NodeHandles
	ros::NodeHandle nh8, nh9, nh10, nh11, nh12;
	
	// Publishers
	task3_pub = nh8.advertise<geographic_msgs::GeoPoseStamped>("/vrx/perception/landmark", 100);
	ros::Publisher objects_pub = nh9.advertise<std_msgs::Float32>("/objects", 10);					// For publishing object information to mission_control
	ros::Publisher target_pub = nh10.advertise<std_msgs::Float32>("/target", 10);						// For publishing a target to the weapon_system
	pa_initialization_state_pub = nh11.advertise<std_msgs::Bool>("pa_initialization_state", 1);	// state of initialization
	NED_buoys_pub = nh12.advertise<amore::NED_objects>("NED_buoys", 100);					// current buoy IDs with respective locations for planner to use to generate path
	
	// Initialize global variables
	pa_initialization_status.data = false;
	current_time = ros::Time::now();   											// sets current time to the time it is now
	last_time = ros::Time::now();        											// sets last time to the time it is now
  
	// Set the loop sleep rate
	ros::Rate loop_rate(50);																// {Hz} Camera update rate: 30, 16 beam lidar update rate: 10

	ros::spinOnce();
	loop_rate.sleep();
	
	// NOTE: comment the following line out for the docker image, but uncomment for user display of cameras and detected buoys
	//cv::destroyWindow("Left Camera Updated");
	//cv::destroyWindow("Left Camera Features");
	//cv::destroyWindow("Right Camera Updated");
	//cv::destroyWindow("Right Camera Features");
	
	while(ros::ok())
	{
		PERCEPTION_ARRAY_inspector();
		//	0 = On standby
		//	1 = General State
		//	3 = Task 3: Landmark Localization and Characterization
		//	4 = Task 4: Wildlife Encounter and Avoid
		//	5 = Task 5: Channel Navigation, Acoustic Beacon Localization and Obstacle Avoidance
		//	6 = Task 6: Scan and Dock and Deliver
		/* switch(PA_state)
		{
			case 0:						// On standby
			
				break;
			case 1:						// General State
			
				break;
			case 3:						// Task 3: Landmark Localization and Characterization
			
				break;
			case 4:						// Task 4: Wildlife Encounter and Avoid
			
				break;
			case 5:						// Task 5: Channel Navigation, Acoustic Beacon Localization and Obstacle Avoidance
			
				break;
			case 6:						// Task 6: Scan and Dock and Deliver
			
				break;
			default:
				break;
		} */
		ros::spinOnce();										// update subscribers
		loop_rate.sleep();									// sleep for set loop_rate
		last_time = current_time;						// update last_time
		//loop_count += 1;									// increment loop counter
	}
	
	return 0;
} // end of main()
//............................................................End of Main Program...........................................................
