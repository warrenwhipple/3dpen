/*! \file nav_eq.c
	\brief The c-file for the OpenShoe navigation algorithm. 
	
	\details This is the c-file for the OpenShoe navigation algorithm. It includes  
	the signal processing functions needed to implement a zero-velocity aided inertial navigation system using a nine 
	state Kalman filter working in a complimentary feedback configuration. It also includes the functions needed for 
	implementing a accelerometer bias calibration framework.      
	
	\authors John-Olof Nilsson, Isaac Skog
 	\copyright Copyright (c) 2011 OpenShoe, ISC License (open source)
 */

///\addtogroup nav_eq
//@{

#include "nav_eq.h"


/*!
\name Accelerometer calibration parameters.   

  Parameters controlling accelerometer calibration, and vectors and matrices used store the biases.    
  
*/ 
//@{

/// Accelerometer biases (x,y,z-axis) [\f$m/s^2\f$].
vec3 accelerometer_biases;										 		

/// Matrix holding the mean of the accelerometer measurements at each calibration orientation [\f$m/s^2\f$].
static precision acceleration_mean_matrix[3][MAX_ORIENTATIONS];	

///Threshold used to check that accelerometers were stationary during the calibration [\f$(m/s^2)^2\f$].   	
precision acceleration_variance_threshold=0.002;					  

/// Number of samples used at each orientation in the calibration procedure.
uint32_t nr_of_calibration_samples=800;						

/// Number of orientations used in the accelerometer calibration procedure. OBS! Most be at least 3 and less than 13. 		     
uint8_t nr_of_calibration_orientations=6;						

/// Flag that is set to true when the IMU should be place in a new orientation. Should be set to false when the calibration procedure is started, and when the IMU has been placed in a new orientation by the user.
bool new_orientation_flag=false;

/// Flag that is set to true when the calibration is finished. Must be set to false before the calibration is started.								
bool acc_calibration_finished_flag=false;

//@}




/*!
\name General control parameters.   

  Parameters controlling general settings of the navigation algorithm     
  
*/ 
//@{
	
/// Rough latitude of the system [\f$degrees\f$]. (Used to calculate the magnitude of the gravity vector)	
precision latitude=13;

/// Rough altitude of the system [\f$m\f$]. (Used to calculate the magnitude of the gravity vector)								
precision altitude=920;

/// Magnitude of the local gravity acceleration [\f$m/s^2\f$]								
precision g=9.782940329221166;	

/// Sampling period [\f$s\f$]					
precision dt=0.001220703125000;

/// Input variable which is assumed to contain the time difference since last inertial measurement
uint32_t imu_dt;

/// Error signaling vector. If zero no error has occurred.						
//extern
uint8_t error_signal;

//@}


/*!
\name IMU data buffer variables.   

  Vectors and variables related to the IMU data buffer.    
  
*/ 
//@{

/// Buffer for the x-axis accelerometer readings.							
static precision acc_buffer_x_axis[UINT8_MAX];		

/// Buffer for the y-axis accelerometer readings.
static precision acc_buffer_y_axis[UINT8_MAX];		

/// Buffer for the z-axis accelerometer readings.
static precision acc_buffer_z_axis[UINT8_MAX];		

/// Buffer for the x-axis gyroscope readings.
static precision gyro_buffer_x_axis[UINT8_MAX];	

/// Buffer for the y-axis gyroscope readings.	
static precision gyro_buffer_y_axis[UINT8_MAX];	

/// Buffer for the z-axis gyroscope readings.
static precision gyro_buffer_z_axis[UINT8_MAX];

/// Buffer for the dt values of the IMU readings.
static precision dt_buffer[UINT8_MAX];

//TODO: Remove the XXX_in variables and make user (OpenShoe_runtime_framework) use extern instead
/// Accelerations read from the IMU [\f$m/s^2\f$]. These are written into the IMU data buffer.
//extern
vec3 accelerations_in;

/// Angular rates read from the IMU [\f$rad/s\f$]. These are written into the IMU data buffer.
//extern
vec3 angular_rates_in;

/// Accelerations outputted from the IMU data buffer [\f$m/s^2\f$].
vec3 accelerations_out;					

/// Angular rates outputted from the IMU data buffer [\f$rad/s\f$].
vec3 angular_rates_out;			    
//@}



/*!
\name Initialization control parameters   

  Parameters controlling the initialization of the navigation algorithm, 
  i.e., the initial states of the inertial navigation system equations 
  and the initial Kalman filter covariance matrix.    
  
  
*/ 
//@{
/// A flag that should be set to true when initialization is started and that becomes false when the initialization is finished.    
bool initialize_flag=true;
	
/// Number of samples used in the initial alignment.
uint8_t nr_of_inital_alignment_samples=16;//4;  

/// Initial heading [\f$rad\f$]							
precision initial_heading=0;					

/// Initial position (North, East, Down) [\f$m\f$] 
vec3 initial_pos = {0, 0, 0};
	
/// Standard deviations in the initial position uncertainties [\f$m\f$].                   
vec3 sigma_initial_position ={0.00001,0.00001,0.00001};      

/// Standard deviations in the initial velocity uncertainties [\f$m/s\f$].
vec3 sigma_initial_velocity ={0.01,0.01,0.01};		

/// Standard deviations in the initial attitude uncertainties [\f$rad\f$].			
vec3 sigma_initial_attitude ={0.00174,0.00174,0.00174};		
//@}

						  




/*!
\name Kalman filter control parameters   

  Parameters controlling the behavior of the Kalman filter. 
  The parameters can be changed while the filter is running 
  to adapt the filter to the current motion dynamics.
  
  \note The default noise standard deviation figures are not set to 
  reflect the true noise figures of the IMU sensors, but 
  rather to model the sum of all the errors (biases, 
  scale factors, nonlinearities, etc.) in the system and 
  the measurement model.           		

*/ 
//@{
/// Accelerometer process noise standard deviation [\f$m/s^2\f$]
precision sigma_acceleration=0.7;				

/// Gyroscope process noise standard deviation [\f$rad/s\f$]
precision sigma_gyroscope=0.005235987755983;

/// Pseudo zero-velocity measurement noise standard deviations (north, east, down) [\f$m/s\f$]	
vec3 sigma_velocity={0.1,0.1,0.1};					
//@}




/*!
\name Navigation and filter state variables.   

  Vectors that holds the current navigation state estimate and the covariance and gain of the Kalman filter.     
  
*/ 
//@{
///  Position estimate (North,East,Down) [\f$m\f$].
vec3 position;

/// Velocity estimate (North,East,Down) [\f$m/s\f$]						
vec3 velocity;						

/// Attitude (quaternions) estimate
quat_vec quaternions;				

/// Rotation matrix used as an "aiding" variable in the filter algorithm. Holds the same information as the quaternions. 
mat3 Rb2t;							 

/// Vector representation of the Kalman filter covariance matrix.
mat9sym cov_vector;

/// Vector representation of the Kalman filter gain matrix. 				
mat9by3 kalman_gain;			
//@}


/*!
\name Zero-velocity detector control parameters   

  Parameters controlling the behavior of the zero-velocity detector. All the detector 
  control parameters, except the \a detector_Window_size may be changed will the navigation algorithm is running in 
  order to adapt the behavior of the detector to the current motion dynamics.   
  
*/ 
//@{

/// Accelerometer noise standard deviation figure [\f$m/s^2\f$], which is used to control how much the detector should trusts the accelerometer data.  
precision sigma_acc_det=0.035;//0.01;

/// Gyroscope noise standard deviation figure [\f$rad/s\f$], which is used to control how much the detector should trusts the gyroscope data.  				
precision sigma_gyro_det=0.006;//0.001745329251994;			

/// The data window size used in the detector (OBS! Must be an odd number.).
volatile uint8_t detector_Window_size=3;					

/// Threshold used in the detector.
precision detector_threshold=10000;

/// Flag that is set to true if a zero-velocity update should be done.			
bool zupt=false;								

/// Variable holding the test statistics for the generalized likelihood ratio test, i.e., the zero-velocity detector.	
precision Test_statistics=0;					
//@}


/*!
\name Step-wise dead-reckoning parameters and variables

  Parameters and variables controlling the resets of the step-wise dead-reckoning mode.   
  
*/ 
//@{

/// Covariance threshold for filter reset
precision reset_cov_threshold = 0.0001;

/// Minimum step length/spacing between filter resets (time)
uint16_t min_time_between_resets = 600;

/// Flag signaling that the filter has been reset
bool filter_reset_flag = false;

/// Parameter giving maximum stationary periods before a pending filter reset is done 
uint16_t max_time_reset_pending = 200;

/// Displacement and heading change vector
vec4 dx;

/// Displacement and heading change covariance vector
mat4sym dP;

//@}




//******************* INLINE FUNCTIONS ******************************//

/**
	\defgroup aux_func Auxilliary functions
	\brief Auxilliary functions for calculation various matrix and scalar operations.

	@{
*/

/*! \brief Function for calculating the square root. 
	

	\details Function for calculating the square root using the hardware multipliers if the precision used is float, 
	otherwise use the standard square root function.  
 */   
precision sqrt_hf(precision);
inline precision sqrt_hf(precision arg){
	
	/*
	if(sizeof(precision)==4)
	{
	float tmp1, tmp2, tmp3;
	__asm__ __volatile__ ( "frsqrta.s %0, %1" : "=r" (tmp3) : "r" (arg));
	tmp1 = tmp3*tmp3;
	tmp2 = 3.0f - tmp1*arg;
	tmp3 = 0.5f * (tmp2 * tmp3);
	tmp1 = tmp3*tmp3;
	tmp2 = 3.0f - tmp1*arg;
	tmp3 = 0.5f * (tmp2 * tmp3);
	tmp1 = tmp3*tmp3;
	tmp2 = 3.0f - tmp1*arg;
	tmp3 = 0.5f * (tmp2 * tmp3);
	return tmp3*arg;
	}
	else
    */
	{
	return sqrt(arg);			
	}	
	
}


/*! \brief Function that calculates the squared Euclidean norm of a vector.
	
	 @param[out] norm2				The squared Euclidean norm of the input vector. 
	 @param[in] arg_vec				The input vectors 
	 @param[in] len					The length of the input vector.
 */ 
precision vecnorm2(precision *, uint8_t);
inline precision vecnorm2(precision *arg_vec, uint8_t len)
{
precision norm2=0;
uint8_t ctr;

	for (ctr=0; ctr<len; ctr++){
	norm2=norm2+arg_vec[ctr]*arg_vec[ctr];
	}

return norm2;	
} 


/*! \brief Function that converts Euler angles ([roll,pitch,yaw]) into a rotation matrix \f$R_b^t\f$.
	
	 @param[out] rotmat				Vector representation of the rotation matrix. 
	 @param[in] euler				Vector of euler angles.
 */
void euler2rotation(mat3, const vec3);
inline void euler2rotation(mat3 rotmat, const vec3 euler){


// Trigonometric value variables	
precision sin_phi = sin(euler[0]);
precision cos_phi = cos(euler[0]);
precision sin_theta =sin(euler[1]);
precision cos_theta =cos(euler[1]);
precision sin_psi = sin(euler[2]);
precision cos_psi = cos(euler[2]);



rotmat[0]=cos_psi*cos_theta;								//Matrix element [1,1]
rotmat[3]=sin_psi*cos_theta;								//Matrix element [1,2]
rotmat[6]=-sin_theta;										//Matrix element [1,3]
rotmat[1]=(-sin_psi*cos_phi) + cos_psi*(sin_theta*sin_phi);		//Matrix element [2,1]
rotmat[4]=(cos_psi*cos_phi) + sin_psi*(sin_theta*sin_phi);		//Matrix element [2,2]
rotmat[7]=cos_theta*sin_phi;								//Matrix element [2,3] 
rotmat[2]=(sin_psi*sin_phi) + cos_psi*(sin_theta*cos_phi);		//Matrix element [3,1]
rotmat[5]=(-cos_psi*sin_phi) + sin_psi*(sin_theta*cos_phi);		//Matrix element [3,2]		          
rotmat[8]=cos_theta*cos_phi;								//Matrix element [3,3]	
}



/*! \brief Function for converting a rotation matrix \f$R_b^t\f$ to quaternions.
	
	 @param[out] q					Vector of quaternions. 
	 @param[in] rotmat				Vector of representation of the rotation matrix.
 */    
void rotation2quat(quat_vec, const mat3);
inline void rotation2quat(quat_vec q,const mat3 rotmat){
	
	// For checking robustness of DCM, diagonal elements
	precision T = 1 + rotmat[0] + rotmat[4]+rotmat[8];  // 1+diag(R)
	
	// Calculate quaternion based on largest diagonal element
	if(T > (0.00000001)){
		precision S = 0.5 / sqrt_hf(T);
		q[3] = 0.25 / S;
		q[0] =(rotmat[7]-rotmat[5]) * S;	//(R(3,2) - R(2,3))*S
		q[1] =(rotmat[2]-rotmat[6]) * S;  	//( R(1,3) - R(3,1) ) * S;
		q[2] =(rotmat[3]-rotmat[1]) * S;	// ( R(2,1) - R(1,2) ) * S;
		}
	else if( (rotmat[0] > rotmat[4] ) && (rotmat[0] > rotmat[8]) ) //(R(1,1) > R(2,2)) && (R(1,1) > R(3,3)) 
	{
		precision S = sqrt_hf(1 + rotmat[0]-rotmat[4]-rotmat[8]) * 2; //S=sqrt_hf(1 + R(1,1) - R(2,2) - R(3,3)) * 2
		q[3] =(rotmat[6]-rotmat[5])/S;		//(R(3,1) - R(2,3)) / S;
		q[0] = 0.25 * S;
		q[1] = (rotmat[1]+rotmat[3])/S;		//(R(1,2) + R(2,1)) / S;
		q[2] = (rotmat[2]+rotmat[6])/S;		//(R(1,3) + R(3,1)) / S;
		}
	else if (rotmat[4]>rotmat[8])		//(R(2,2) > R(3,3))
	{
		precision S = sqrt_hf(1+rotmat[4]-rotmat[0]-rotmat[8])*2;	//sqrt_hf( 1 + R(2,2) - R(1,1) - R(3,3) ) * 2; 
		q[3] = (rotmat[2]-rotmat[6])/S;						//(R(1,3) - R(3,1)) / S;
		q[0] =	(rotmat[1]+rotmat[3])/S;					//(R(1,2) + R(2,1)) / S;
		q[1] = 0.25 * S;
		q[2] = (rotmat[5]+rotmat[7])/S;						//(R(2,3) + R(3,2)) / S;
		}
	else{
		precision S=sqrt_hf(1+rotmat[8]-rotmat[0]-rotmat[4])*2;		//S = sqrt_hf( 1 + R(3,3) - R(1,1) - R(2,2) ) * 2; 
		q[3] = (rotmat[3]-rotmat[1])/S;						//(R(2,1) - R(1,2)) / S;
		q[0] = (rotmat[2]+rotmat[6])/S;						//(R(1,3) + R(3,1)) / S;
		q[1] = (rotmat[1]+rotmat[3])/S;						//(R(1,2) + R(2,1)) / S;
		q[2] = 0.25 * S;
		}
	
	
	//Normalize the quaternion
	T=sqrt_hf(vecnorm2(q,4));
	
	q[0]=q[0]/T;
	q[1]=q[1]/T;
	q[2]=q[2]/T;
	q[3]=q[3]/T;
}



/*! \brief Function for converting quaternions to a rotation matrix \f$R_b^t\f$.
	
	@param[out] rotmat				Vector of representation of the rotation matrix.	
	 @param[in] q					Vector of quaternions. 
	 
 */   
void quat2rotation(mat3, const quat_vec);
inline void quat2rotation(mat3 rotmat,const quat_vec q){

precision p[6];

p[0]=q[0]*q[0];
p[1]=q[1]*q[1];
p[2]=q[2]*q[2];
p[3]=q[3]*q[3];

p[4]=p[1]+p[2];

if (p[0]+p[3]+p[4] != 0)
{
	p[5]=2/(p[0]+p[3]+p[4]);
}
else
{
	p[5]=0;	
}

rotmat[0]=1-p[5]*p[4];			//R(1,1)=1-p(6)*p(5);
rotmat[4]=1-p[5]*(p[0]+p[2]);	//R(2,2)=1-p(6)*(p(1)+p(3));
rotmat[8]=1-p[5]*(p[0]+p[1]);	//R(3,3)=1-p(6)*(p(1)+p(2));

p[0]=p[5]*q[0];					//p(1)=p(6)*q(1); 
p[1]=p[5]*q[1];					//p(2)=p(6)*q(2);
p[4]=p[5]*q[2]*q[3];			//p(5)=p(6)*q(3)*q(4);
p[5]=p[0]*q[1];					//p(6)=p(1)*q(2);

rotmat[1]=p[5]-p[4];			//R(1,2)=p(6)-p(5);
rotmat[3]=p[5]+p[4];			//R(2,1)=p(6)+p(5);

p[4]=p[1]*q[3];					//p(5)=p(2)*q(4);
p[5]=p[0]*q[2];					//p(6)=p(1)*q(3);

rotmat[2]=p[5]+p[4];			//R(1,3)=p(6)+p(5);
rotmat[6]=p[5]-p[4];			//R(3,1)=p(6)-p(5);

p[4]=p[0]*q[3];					//p(5)=p(1)*q(4);
p[5]=p[1]*q[2];					//p(6)=p(2)*q(3);

rotmat[5]=p[5]-p[4];			//R(2,3)=p(6)-p(5);
rotmat[7]=p[5]+p[4];			//R(3,2)=p(6)+p(5);


}
	 


/*! \brief Function for converting a rotation matrix \f$R_b^t\f$ to Euler angles ([roll,pitch,yaw]).
	
	@param[out] euler				Vector of euler angles.
	@param[in] rotmat				Vector of representation of the rotation matrix.		 
 */	 
void rotation2euler(vec3, const mat3);
inline void rotation2euler(vec3 euler, const mat3 rotmat){
	

	// Compute Euler angles [WARNING! check atan2]
	euler[0] = atan2(rotmat[7],rotmat[8]);	//atan2( R(3,2), R(3,3) );
	euler[1] = asin(-rotmat[6]);			//asin( -R(3,1) );
	euler[2] = -atan2(rotmat[3],rotmat[0]);	//atan2( R(2,1), R(1,1));
}




/*! \brief Function for calculating the Kalman filter innovation covariance.
	
	@param[out] re		Vector representation of the innovation covariance matrix.
	@param[in] pvec		Vector representation of the Kalman filter covariance matrix.
	@param[in] sigma	Vector representation of pseudo zero-velocity measurement noise standard deviations.		 
 */	
void innovation_cov(mat3sym, mat9sym);
inline void innovation_cov(mat3sym re, mat9sym pvec){



re[0]=pvec[24]+sigma_velocity[0]*sigma_velocity[0];
re[1]=pvec[25];
re[2]=pvec[26];
re[3]=pvec[30]+sigma_velocity[1]*sigma_velocity[1];
re[4]=pvec[31];
re[5]=pvec[35]+sigma_velocity[2]*sigma_velocity[2];

}




/*! \brief Function for inverting a 3 by 3 matrix hermitian matrix.
	
	@param[out] ainv		Vector representation of the inverted matrix.
	@param[out] error		Error signaling vector.
	@param[in]	a			Vector representation of the matrix to be inverted.		 
 */	
void invmat3sys(mat3sym, mat3sym);
inline void invmat3sys(mat3sym ainv, mat3sym a){


// Calculate the determinant of matrix  
precision det=-a[2]*(a[2]*a[3]) + 2*a[1]*(a[2]*a[4]) - a[0]*(a[4]*a[4]) - a[1]*(a[1]*a[5]) + a[0]*(a[3]*a[5]);

	// Check the size of the determinant and send a error message if it is to small. 
	if(absf(det)<0.0000001)
	{
		error_signal=MATRIX_INVERSION_ERROR;
	}


ainv[0]=(a[3]*a[5]-a[4]*a[4])/det;
ainv[1]=(a[2]*a[4]-a[1]*a[5])/det;
ainv[2]=(a[1]*a[4]-a[2]*a[3])/det;
ainv[3]=(a[0]*a[5]-a[2]*a[2])/det;
ainv[4]=(a[1]*a[2]-a[0]*a[4])/det;
ainv[5]=(a[0]*a[3]-a[1]*a[1])/det;
}




/*! \brief	Function that calculates the maximum value of a vector and returns 
			the max value and the index of the vector element holding the maximum value.  
	
	@param[out] max_v		Largest value of the input vector.
	@param[out] index		Index of the vector element holding the largest value. 
	@param[in]	arg_vec		The input vector.		 
 */	
void max_value(precision *,uint8_t *, precision *);
inline void max_value(precision *max_v,uint8_t *index, precision *arg_vec){

// Set the initial max value and vector element index
*max_v=arg_vec[0];
*index=0;

//Iterate through the vector
	for(uint8_t ctr=1;ctr<(sizeof(arg_vec)/sizeof(arg_vec[0]));ctr++)
	{
		//If the current element of the vector is larger than the previous element, update the max value and the index.
		if(arg_vec[ctr]>arg_vec[ctr-1])
		{
		
			*max_v=arg_vec[ctr];
			*index=ctr;		
		} 		
	}
}

//@}

/**
	\defgroup miscel Miscellaneous
	\brief Miscellaneous routines.
	
	@{
*/

/*! \brief Function that calculates the magnitude of the local gravity vector based upon the WGS84 gravity model. 
	
	 @param[out] g			The magnitude of the local gravity vector.
	 @param[in] latitude	The latitude of the navigation system. 
	 @param[in] altitude	The altitude of the navigation system. 
	 
 */
void gravity(void);
inline void gravity(void){
								
	precision lambda=M_PI/180.0*latitude;  //latitude [rad]

	g=9.780327*(1+0.0053024*(sin(lambda)*sin(lambda))-0.0000058*(sin(2*lambda)*sin(2*lambda)))-(0.0000030877-0.000000004*(sin(lambda)*sin(lambda)))*altitude+0.000000000000072*(altitude*altitude);
	
} 

//@}
  
/**
	\defgroup main_func ZUPT aided INS functions
	\brief the main functions for running the ZUPT aided INS.

	@{
*/

//********************* ORDINARY FUNCTIONS *********************************//
 
void update_imu_data_buffers(void){

// The index of the IMU data buffers which the data from the IMU should be written to.
static uint8_t in_data_buffer_index;
//TODO: explicit initialization of above

// The index of the IMU data buffers which the out data should be read from.
int16_t out_data_buffer_index;
//TODO: should this variable be static?

//Update the IMU data buffer with the latest read IMU data.
acc_buffer_x_axis[in_data_buffer_index]=accelerations_in[0];
acc_buffer_y_axis[in_data_buffer_index]=accelerations_in[1];
acc_buffer_z_axis[in_data_buffer_index]=accelerations_in[2];		
gyro_buffer_x_axis[in_data_buffer_index]=angular_rates_in[0];
gyro_buffer_y_axis[in_data_buffer_index]=angular_rates_in[1];
gyro_buffer_z_axis[in_data_buffer_index]=angular_rates_in[2];		

//Update the IMU time differential buffer.
dt_buffer[in_data_buffer_index]=((precision)imu_dt) / ((precision)48000000UL);


/* Read the ''middle'' samples from the IMU data buffer. 
This is used to update the navigation equations at this iteration. */

//Calculate the out data index.
out_data_buffer_index=in_data_buffer_index-(detector_Window_size/2);

if(out_data_buffer_index<0){
	out_data_buffer_index=in_data_buffer_index+(detector_Window_size/2)+1;
}

//Update the global variables that are used in the update the navigation equations.
accelerations_out[0]=acc_buffer_x_axis[out_data_buffer_index];
accelerations_out[1]=acc_buffer_y_axis[out_data_buffer_index];
accelerations_out[2]=acc_buffer_z_axis[out_data_buffer_index];	
angular_rates_out[0]=gyro_buffer_x_axis[out_data_buffer_index];
angular_rates_out[1]=gyro_buffer_y_axis[out_data_buffer_index];
angular_rates_out[2]=gyro_buffer_z_axis[out_data_buffer_index];
dt=dt_buffer[out_data_buffer_index];

//Update the in data buffer counter. If the in data buffer counter is equal to length of the buffer, reset the counter. 
if(in_data_buffer_index==(detector_Window_size-1))
{
	in_data_buffer_index=0;		
}
else
{
	in_data_buffer_index=in_data_buffer_index+1;	
}

}


void strapdown_mechanisation_equations(void){	
	/*
		The inputs and outputs are the following:
		
		position - position vector in the n-frame [m]
		velocity - velocity vector in the n-frame [m]
		quaternions - quaternions describing the orientation of the navigation platform
		Rb2t - The rotation matrix Rb2t (Includes the same information as the quaternionsuaternions but we use it other functions, i.e., we saves computations) 
		accelerations - Measured accelerations vector (acceleration) in the b-frame [m/s^2]     
		angular_rates - Measured angular-rate vector in the b-frame [rad/s]
		
		The following global variables most be defined in the main function: 
		
		g - The gravity constant [m/s^2]
		dt - The sampling period [s]	
		
   Note that this is a rudimentary mechanization of the navigation equaternionsuations which only is suitable for
   use with systems that uses low-cost/low-performance inertial sensors and where only short periods of 
   free inertial navigation is expected.   		
*/ 
	

	
	// Working variables
	vec3 an_hat;				
    vec3 angular_rates_dt;
	quat_vec quat_tmp;
	precision cos_v;
	precision sin_v;

	
	
	//********** Integrate angular rates  **********//
	
	// If we are measuring some angular rates, update the quaternion vector. 
    if((angular_rates_out[0]!=0)|(angular_rates_out[1]!=0)|(angular_rates_out[2]!=0))
	{
			
	// Multiply the angular rates with the sampling period dt.
	angular_rates_dt[0]=angular_rates_out[0]*dt;
	angular_rates_dt[1]=angular_rates_out[1]*dt;
	angular_rates_dt[2]=angular_rates_out[2]*dt;
	
	// Calculate the norm of the vector angular_rates_dt
	precision v=( sqrt_hf( vecnorm2(angular_rates_dt, 3) ) );
		
	cos_v=cos(v/2);	
	sin_v=(sin(v/2)/v);
	
	// Time update of the quaternions 	
	quat_tmp[0]=cos_v*quaternions[0]+sin_v*(angular_rates_dt[2]*quaternions[1]-angular_rates_dt[1]*quaternions[2]+angular_rates_dt[0]*quaternions[3]);	// w_tb(2)*quaternions(1)-w_tb(1)*quaternions(2)+w_tb(0)*quaternions(3)		
	quat_tmp[1]=cos_v*quaternions[1]+sin_v*(-angular_rates_dt[2]*quaternions[0]+angular_rates_dt[0]*quaternions[2]+angular_rates_dt[1]*quaternions[3]);  //-w_tb(2)*quaternions(0)+w_tb(0)*quaternions(2)+w_tb(1)*quaternions(3)
	quat_tmp[2]=cos_v*quaternions[2]+sin_v*(angular_rates_dt[1]*quaternions[0]-angular_rates_dt[0]*quaternions[1]+angular_rates_dt[2]*quaternions[3]);   //w_tb(1)*quaternions(0)-w_tb(0)*quaternions(1)+w_tb(2)*quaternions(3)
	quat_tmp[3]=cos_v*quaternions[3]+sin_v*(-angular_rates_dt[0]*quaternions[0]-angular_rates_dt[1]*quaternions[1]-angular_rates_dt[2]*quaternions[2]);  //-w_tb(0)*quaternions(0)-w_tb(1)*quaternions(1)-w_tb(2)*quaternions(2)
		

	
	// Re-normalize the quaternions and update the global variable
	v=sqrt_hf(vecnorm2(quat_tmp, 4));
	quaternions[0]=quat_tmp[0]/v;
	quaternions[1]=quat_tmp[1]/v;
	quaternions[2]=quat_tmp[2]/v;
	quaternions[3]=quat_tmp[3]/v;
	}	
	//*****************************************************//
		
	
		
	//******** Update the position and velocity *******//
	 
	// Convert quaternions to rotation matrix
	quat2rotation(Rb2t,quaternions);  //Rb2t

	// Compute acceleration in navigation coordinate frame and subtract the acceleration due to the earth gravity force. 
	an_hat[0]=Rb2t[0]*accelerations_out[0]+Rb2t[1]*accelerations_out[1]+Rb2t[2]*accelerations_out[2];
	an_hat[1]=Rb2t[3]*accelerations_out[0]+Rb2t[4]*accelerations_out[1]+Rb2t[5]*accelerations_out[2];
	an_hat[2]=Rb2t[6]*accelerations_out[0]+Rb2t[7]*accelerations_out[1]+Rb2t[8]*accelerations_out[2]+g;
	

	// Integrate the acceleration to get the velocity
	velocity[0]=velocity[0]+an_hat[0]*dt;
	velocity[1]=velocity[1]+an_hat[1]*dt;
	velocity[2]=velocity[2]+an_hat[2]*dt;
	
	// Integrate the velocity to get the position
	position[0]=position[0]+velocity[0]*dt;
	position[1]=position[1]+velocity[1]*dt;
	position[2]=position[2]+velocity[2]*dt;	
	//******************************************************//
}
	 

void time_up_data(void){
	
	//Working variables
	uint8_t ctr=0;
	precision dt2_sigma2_acc= (dt*dt)*(sigma_acceleration*sigma_acceleration); 
	precision dt2_sigma2_gyro=(dt*dt)*(sigma_gyroscope*sigma_gyroscope);
	mat9sym ppvec;		//Temporary vector holding the update covariances 
	vec3 s;				//Specific accelerations vector in the n-frame.
	
	
	// Calculate the acceleration (specific-force) vector "s" in the n-frame 
	s[0]=Rb2t[0]*accelerations_out[0]+Rb2t[1]*accelerations_out[1]+Rb2t[2]*accelerations_out[2];
	s[1]=Rb2t[3]*accelerations_out[0]+Rb2t[4]*accelerations_out[1]+Rb2t[5]*accelerations_out[2];
	s[2]=Rb2t[6]*accelerations_out[0]+Rb2t[7]*accelerations_out[1]+Rb2t[8]*accelerations_out[2]; 	
	
	
	
// First row of the covariance matrix
ppvec[0]=cov_vector[0]+dt*(2*cov_vector[3] + dt*cov_vector[24]);

ppvec[1]=cov_vector[1]+dt*(cov_vector[4]+cov_vector[11]+dt*cov_vector[25]);

ppvec[2]=cov_vector[2]+dt*(cov_vector[5]+cov_vector[18]+dt*cov_vector[26]);
      
ppvec[3]=cov_vector[3] + dt*(cov_vector[24]+cov_vector[8]*s[1]+cov_vector[29]*(s[1]*dt) - s[2]*(cov_vector[7]+cov_vector[28]*dt));

ppvec[4]=cov_vector[4]+dt*cov_vector[25]+ (s[2]*dt)*(cov_vector[6] + dt*cov_vector[27]) - (s[0]*dt)*(cov_vector[8] + dt*cov_vector[29]);

ppvec[5]=cov_vector[5]+dt*(cov_vector[26]+cov_vector[7]*s[0]+cov_vector[28]*(s[0]*dt)-s[1]*(cov_vector[6]+cov_vector[27]*dt));

ppvec[6]=cov_vector[6] + cov_vector[27]*dt;

ppvec[7]=cov_vector[7] + cov_vector[28]*dt;

ppvec[8]=cov_vector[8]+cov_vector[29]*dt;

// Second row of the covariance matrix
ppvec[9]=cov_vector[9] + dt*(2*cov_vector[12] + cov_vector[30]*dt);

ppvec[10]=cov_vector[10] + dt*(cov_vector[13] + cov_vector[19] + cov_vector[31]*dt);

ppvec[11]=cov_vector[11] + dt*(cov_vector[25] + cov_vector[16]*s[1] + cov_vector[34]*(s[1]*dt) - s[2]*(cov_vector[15]+cov_vector[33]*dt));

ppvec[12]=cov_vector[12] + cov_vector[30]*dt + s[2]*dt*(cov_vector[14] + cov_vector[32]*dt)-(s[0]*dt)*( cov_vector[16]+cov_vector[34]*dt );

ppvec[13]=cov_vector[13] + dt*(cov_vector[31] + cov_vector[15]*s[0] + cov_vector[33]*(s[0]*dt) -s[1]*(cov_vector[14] + cov_vector[32]*dt) );

ppvec[14]=cov_vector[14] + cov_vector[32]*dt;

ppvec[15]=cov_vector[15] + cov_vector[33]*dt;

ppvec[16]=cov_vector[16] + cov_vector[34]*dt;

// Third row of the covariance matrix
ppvec[17]=cov_vector[17] + dt*(2*cov_vector[20]+cov_vector[35]*dt);

ppvec[18]=cov_vector[18] + dt*(cov_vector[26] + cov_vector[23]*s[1] + cov_vector[38]*s[1]*dt - s[2]*(cov_vector[22] + cov_vector[37]*dt));

ppvec[19]=cov_vector[19] + cov_vector[31]*dt + s[2]*dt*(cov_vector[21] + cov_vector[36]*dt) - (s[0]*dt)*(cov_vector[23] + cov_vector[38]*dt);

ppvec[20]=cov_vector[20] + dt*(cov_vector[35] + cov_vector[22]*s[0] + cov_vector[37]*(s[0]*dt) - s[1]*(cov_vector[21] + cov_vector[36]*dt));


ppvec[21]=cov_vector[21] + cov_vector[36]*dt;

ppvec[22]=cov_vector[22]+cov_vector[37]*dt;

ppvec[23]=cov_vector[23]+cov_vector[38]*dt;

// Forth row of the covariance matrix
ppvec[24]=cov_vector[24] + dt*(2*(cov_vector[29]*s[1])+(cov_vector[44]*s[1])*(s[1]*dt) + s[2]*(-2*cov_vector[28] - (2*cov_vector[43])*(s[1]*dt)+cov_vector[42]*(s[2]*dt)))+dt2_sigma2_acc;

ppvec[25]=cov_vector[25] + dt*(-cov_vector[29]*s[0] + cov_vector[34]*s[1] - (cov_vector[44]*s[0])*(s[1]*dt) + s[2]*(cov_vector[27]-cov_vector[33]+cov_vector[43]*(s[0]*dt) + cov_vector[41]*(s[1]*dt) - cov_vector[40]*(s[2]*dt)));

ppvec[26]=cov_vector[26] + cov_vector[38]*(s[1]*dt) - cov_vector[37]*(s[2]*dt) - (s[1]*dt)*(cov_vector[27] + cov_vector[41]*(s[1]*dt) - cov_vector[40]*(s[2]*dt)) + (s[0]*dt)*(cov_vector[28]+cov_vector[43]*(s[1]*dt) - cov_vector[42]*(s[2]*dt));

ppvec[27]=cov_vector[27]+cov_vector[41]*(s[1]*dt) - cov_vector[40]*(s[2]*dt);

ppvec[28]=cov_vector[28] + cov_vector[43]*(s[1]*dt) - cov_vector[42]*(s[2]*dt);

ppvec[29]=cov_vector[29] + cov_vector[44]*(s[1]*dt) - cov_vector[43]*(s[2]*dt);

// Fifth row of the covariance matrix
ppvec[30]=cov_vector[30]+dt*(-2*(cov_vector[34]*s[0])+(cov_vector[44]*s[0])*(s[0]*dt) + s[2]*(2*cov_vector[32]-(2*cov_vector[41])*(s[0]*dt)+cov_vector[39]*(s[2]*dt)))+dt2_sigma2_acc;

ppvec[31]=cov_vector[31] - cov_vector[38]*(s[0]*dt) + cov_vector[36]*(s[2]*dt) - (s[1]*dt)*(cov_vector[32] - cov_vector[41]*(s[0]*dt) + cov_vector[39]*(s[2]*dt)) + (s[0]*dt)*(cov_vector[33]-cov_vector[43]*s[0]*dt+cov_vector[40]*(s[2]*dt));

ppvec[32]=cov_vector[32] - cov_vector[41]*s[0]*dt + cov_vector[39]*(s[2]*dt);

ppvec[33]=cov_vector[33] - cov_vector[43]*s[0]*dt + cov_vector[40]*(s[2]*dt);

ppvec[34]=cov_vector[34]-cov_vector[44]*(s[0]*dt)+cov_vector[41]*(s[2]*dt);

// Sixth row of the covariance matrix
ppvec[35]=cov_vector[35]+dt*(2*(cov_vector[37]*s[0])+(cov_vector[42]*s[0])*(s[0]*dt)+s[1]*(-2*cov_vector[36]-2*cov_vector[40]*s[0]*dt+cov_vector[39]*s[1]*dt))+dt2_sigma2_acc;

ppvec[36]=cov_vector[36] + cov_vector[40]*s[0]*dt - cov_vector[39]*s[1]*dt;

ppvec[37]=cov_vector[37] + cov_vector[42]*s[0]*dt - cov_vector[40]*s[1]*dt;

ppvec[38]=cov_vector[38] + cov_vector[43]*s[0]*dt - cov_vector[41]*s[1]*dt;

// Seventh row of the covariance matrix
ppvec[39]=cov_vector[39]+dt2_sigma2_gyro;

ppvec[40]=cov_vector[40];

ppvec[41]=cov_vector[41];

// Eight row of the covariance matrix
ppvec[42]=cov_vector[42]+dt2_sigma2_gyro;

ppvec[43]=cov_vector[43];

// Ninth row of the covariance matrix
ppvec[44]=cov_vector[44]+dt2_sigma2_gyro;

	
// Copy the temporary vector to the global covariance vector	
for(ctr=0;ctr<45;ctr++){
cov_vector[ctr]=ppvec[ctr];
}		
} 



void gain_matrix(void){


mat3sym Re;			//Innovation matrix
mat3sym invRe;		//Inverse of the innovation matrix


/************ Calculate the Kalman filter innovation matrix *******************/
innovation_cov(Re,cov_vector);

/************ Calculate the inverse of the innovation matrix *********/
invmat3sys(invRe,Re);

/******************* Calculate the Kalman filter gain **************************/

// First row of the gain matrix
kalman_gain[0]=cov_vector[3]*invRe[0] + cov_vector[4]*invRe[1] + cov_vector[5]*invRe[2];
kalman_gain[1]=cov_vector[3]*invRe[1] + cov_vector[4]*invRe[3] + cov_vector[5]*invRe[4];
kalman_gain[2]=cov_vector[3]*invRe[2] + cov_vector[4]*invRe[4] + cov_vector[5]*invRe[5];


// Second row of the gain matrix
kalman_gain[3]=cov_vector[11]*invRe[0] + cov_vector[12]*invRe[1] + cov_vector[13]*invRe[2];
kalman_gain[4]=cov_vector[11]*invRe[1] + cov_vector[12]*invRe[3] + cov_vector[13]*invRe[4];
kalman_gain[5]=cov_vector[11]*invRe[2] + cov_vector[12]*invRe[4] + cov_vector[13]*invRe[5];

// Third row of the gain matrix
kalman_gain[6]=cov_vector[18]*invRe[0] + cov_vector[19]*invRe[1] + cov_vector[20]*invRe[2];
kalman_gain[7]=cov_vector[18]*invRe[1] + cov_vector[19]*invRe[3] + cov_vector[20]*invRe[4];
kalman_gain[8]=cov_vector[18]*invRe[2] + cov_vector[19]*invRe[4] + cov_vector[20]*invRe[5];


//Forth row of the gain matrix
kalman_gain[9]=cov_vector[24]*invRe[0] + cov_vector[25]*invRe[1] + cov_vector[26]*invRe[2];
kalman_gain[10]=cov_vector[24]*invRe[1] + cov_vector[25]*invRe[3] + cov_vector[26]*invRe[4];
kalman_gain[11]=cov_vector[24]*invRe[2] + cov_vector[25]*invRe[4] + cov_vector[26]*invRe[5];


//Fifth row of the gain matrix
kalman_gain[12]=cov_vector[25]*invRe[0] + cov_vector[30]*invRe[1] + cov_vector[31]*invRe[2];
kalman_gain[13]=cov_vector[25]*invRe[1] + cov_vector[30]*invRe[3] + cov_vector[31]*invRe[4];
kalman_gain[14]=cov_vector[25]*invRe[2] + cov_vector[30]*invRe[4] + cov_vector[31]*invRe[5];


//Sixth row of the gain matrix
kalman_gain[15]=cov_vector[26]*invRe[0] + cov_vector[31]*invRe[1] + cov_vector[35]*invRe[2];
kalman_gain[16]=cov_vector[26]*invRe[1] + cov_vector[31]*invRe[3] + cov_vector[35]*invRe[4];
kalman_gain[17]=cov_vector[26]*invRe[2] + cov_vector[31]*invRe[4] + cov_vector[35]*invRe[5];


//Seventh row of the gain matrix
kalman_gain[18]=cov_vector[27]*invRe[0] + cov_vector[32]*invRe[1] + cov_vector[36]*invRe[2];
kalman_gain[19]=cov_vector[27]*invRe[1] + cov_vector[32]*invRe[3] + cov_vector[36]*invRe[4];
kalman_gain[20]=cov_vector[27]*invRe[2] + cov_vector[32]*invRe[4] + cov_vector[36]*invRe[5];


//Eight row of the gain matrix
kalman_gain[21]=cov_vector[28]*invRe[0] + cov_vector[33]*invRe[1] + cov_vector[37]*invRe[2];
kalman_gain[22]=cov_vector[28]*invRe[1] + cov_vector[33]*invRe[3] + cov_vector[37]*invRe[4];
kalman_gain[23]=cov_vector[28]*invRe[2] + cov_vector[33]*invRe[4] + cov_vector[37]*invRe[5];


//Ninth row of the gain matrix
kalman_gain[24]=cov_vector[29]*invRe[0] + cov_vector[34]*invRe[1] + cov_vector[38]*invRe[2];
kalman_gain[25]=cov_vector[29]*invRe[1] + cov_vector[34]*invRe[3] + cov_vector[38]*invRe[4];
kalman_gain[26]=cov_vector[29]*invRe[2] + cov_vector[34]*invRe[4] + cov_vector[38]*invRe[5];

}  


void measurement_update(void){

uint8_t ctr=0;
mat9sym ppvec;		//Temporary vector holding the update covariances 


// First row
ppvec[0]=cov_vector[0] - kalman_gain[0]*cov_vector[3] - kalman_gain[1]*cov_vector[4] - kalman_gain[2]*cov_vector[5];

ppvec[1]=cov_vector[1] - kalman_gain[0]* cov_vector[11] - kalman_gain[1]* cov_vector[12] - kalman_gain[2]* cov_vector[13];

ppvec[2]=cov_vector[2]-kalman_gain[0]* cov_vector[18] - kalman_gain[1]* cov_vector[19] - kalman_gain[2]* cov_vector[20];

ppvec[3]=cov_vector[3]-kalman_gain[0]* cov_vector[24] - kalman_gain[1]* cov_vector[25] - kalman_gain[2]* cov_vector[26];

ppvec[4]=cov_vector[4]-kalman_gain[0]* cov_vector[25] - kalman_gain[1]* cov_vector[30] - kalman_gain[2]* cov_vector[31];

ppvec[5]=cov_vector[5]-kalman_gain[0]* cov_vector[26] - kalman_gain[1]* cov_vector[31] - kalman_gain[2]* cov_vector[35];

ppvec[6]=cov_vector[6]-kalman_gain[0]* cov_vector[27] - kalman_gain[1]* cov_vector[32] - kalman_gain[2]* cov_vector[36];

ppvec[7]=cov_vector[7]-kalman_gain[0]* cov_vector[28] - kalman_gain[1]* cov_vector[33] - kalman_gain[2]* cov_vector[37];

ppvec[8]=cov_vector[8]-kalman_gain[0]* cov_vector[29] - kalman_gain[1]* cov_vector[34] - kalman_gain[2]* cov_vector[38];

// Second row
ppvec[9]=cov_vector[9]-kalman_gain[3]* cov_vector[11] - kalman_gain[4]* cov_vector[12] - kalman_gain[5]* cov_vector[13];

ppvec[10]=cov_vector[10]-kalman_gain[3]* cov_vector[18] - kalman_gain[4]* cov_vector[19] - kalman_gain[5]* cov_vector[20];

ppvec[11]=cov_vector[11]-kalman_gain[3]* cov_vector[24] - kalman_gain[4]* cov_vector[25] - kalman_gain[5]* cov_vector[26];

ppvec[12]=cov_vector[12]-kalman_gain[3]* cov_vector[25] - kalman_gain[4]* cov_vector[30] - kalman_gain[5]* cov_vector[31];

ppvec[13]=cov_vector[13]-kalman_gain[3]* cov_vector[26] - kalman_gain[4]* cov_vector[31] - kalman_gain[5]* cov_vector[35];

ppvec[14]=cov_vector[14]-kalman_gain[3]* cov_vector[27] - kalman_gain[4]* cov_vector[32] - kalman_gain[5]* cov_vector[36];

ppvec[15]=cov_vector[15]-kalman_gain[3]* cov_vector[28] - kalman_gain[4]* cov_vector[33] - kalman_gain[5]* cov_vector[37];
 
ppvec[16]=cov_vector[16]-kalman_gain[3]* cov_vector[29] - kalman_gain[4]* cov_vector[34] - kalman_gain[5]* cov_vector[38];

// Third row
ppvec[17]=cov_vector[17]-kalman_gain[6]* cov_vector[18] - kalman_gain[7]* cov_vector[19] - kalman_gain[8]* cov_vector[20];

ppvec[18]=cov_vector[18]-kalman_gain[6]* cov_vector[24] - kalman_gain[7]* cov_vector[25] - kalman_gain[8]* cov_vector[26];

ppvec[19]=cov_vector[19]-kalman_gain[6]* cov_vector[25] - kalman_gain[7]* cov_vector[30] - kalman_gain[8]* cov_vector[31];

ppvec[20]=cov_vector[20]-kalman_gain[6]* cov_vector[26] - kalman_gain[7]* cov_vector[31] - kalman_gain[8]* cov_vector[35];


ppvec[21]=cov_vector[21]-kalman_gain[6]* cov_vector[27] - kalman_gain[7]* cov_vector[32] - kalman_gain[8]* cov_vector[36];

ppvec[22]=cov_vector[22]-kalman_gain[6]* cov_vector[28] - kalman_gain[7]* cov_vector[33] - kalman_gain[8]* cov_vector[37];

ppvec[23]=cov_vector[23]-kalman_gain[6]* cov_vector[29] - kalman_gain[7]* cov_vector[34] - kalman_gain[8]* cov_vector[38];

// Forth row
ppvec[24]=cov_vector[24]-kalman_gain[9]* cov_vector[24] - kalman_gain[10]* cov_vector[25] - kalman_gain[11]* cov_vector[26];

ppvec[25]=cov_vector[25]-kalman_gain[9]* cov_vector[25] - kalman_gain[10]* cov_vector[30] - kalman_gain[11]* cov_vector[31];

ppvec[26]=cov_vector[26]-kalman_gain[9]* cov_vector[26] - kalman_gain[10]* cov_vector[31] - kalman_gain[11]* cov_vector[35];

ppvec[27]=cov_vector[27]-kalman_gain[9]* cov_vector[27] - kalman_gain[10]* cov_vector[32] - kalman_gain[11]* cov_vector[36];

ppvec[28]=cov_vector[28]-kalman_gain[9]* cov_vector[28] - kalman_gain[10]* cov_vector[33] - kalman_gain[11]* cov_vector[37];

ppvec[29]=cov_vector[29]-kalman_gain[9]* cov_vector[29] - kalman_gain[10]* cov_vector[34] - kalman_gain[11]* cov_vector[38];

// Fifth row
ppvec[30]=cov_vector[30]-kalman_gain[12]* cov_vector[25] - kalman_gain[13]* cov_vector[30] - kalman_gain[14]* cov_vector[31];

ppvec[31]=cov_vector[31]-kalman_gain[12]* cov_vector[26] - kalman_gain[13]* cov_vector[31] - kalman_gain[14]* cov_vector[35];

ppvec[32]=cov_vector[32]-kalman_gain[12]* cov_vector[27] - kalman_gain[13]* cov_vector[32] - kalman_gain[14]* cov_vector[36];

ppvec[33]=cov_vector[33]-kalman_gain[12]* cov_vector[28] - kalman_gain[13]* cov_vector[33] - kalman_gain[14]* cov_vector[37];

ppvec[34]=cov_vector[34]-kalman_gain[12]* cov_vector[29] - kalman_gain[13]* cov_vector[34] - kalman_gain[14]* cov_vector[38];

// Sixth row
ppvec[35]=cov_vector[35]-kalman_gain[15]* cov_vector[26] - kalman_gain[16]* cov_vector[31] - kalman_gain[17]* cov_vector[35];

ppvec[36]=cov_vector[36]-kalman_gain[15]* cov_vector[27] - kalman_gain[16]* cov_vector[32] - kalman_gain[17]* cov_vector[36];

ppvec[37]=cov_vector[37]-kalman_gain[15]* cov_vector[28] - kalman_gain[16]* cov_vector[33] - kalman_gain[17]* cov_vector[37];

ppvec[38]=cov_vector[38]-kalman_gain[15]* cov_vector[29] - kalman_gain[16]* cov_vector[34] - kalman_gain[17]* cov_vector[38];

// Seventh row
ppvec[39]=cov_vector[39]-kalman_gain[18]* cov_vector[27] - kalman_gain[19]* cov_vector[32] - kalman_gain[20]* cov_vector[36];

ppvec[40]=cov_vector[40]-kalman_gain[18]* cov_vector[28] - kalman_gain[19]* cov_vector[33] - kalman_gain[20]* cov_vector[37];

ppvec[41]=cov_vector[41]-kalman_gain[18]* cov_vector[29] - kalman_gain[19]* cov_vector[34] - kalman_gain[20]* cov_vector[38];

// Eight row
ppvec[42]=cov_vector[42]-kalman_gain[21]* cov_vector[28] - kalman_gain[22]* cov_vector[33] - kalman_gain[23]* cov_vector[37];

ppvec[43]=cov_vector[43]-kalman_gain[21]* cov_vector[29] - kalman_gain[22]* cov_vector[34] - kalman_gain[23]* cov_vector[38];

// Ninth row
ppvec[44]=cov_vector[44]-kalman_gain[24]* cov_vector[29] - kalman_gain[25]* cov_vector[34] - kalman_gain[26]* cov_vector[38];


// Copy the temporary vector to the covariance vector	
for(ctr=0;ctr<45;ctr++){
cov_vector[ctr]=ppvec[ctr];
}	
}



void correct_navigation_states(void){
	
vec3 velocity_tmp; // Temporary vector holding the corrected velocity state. 	

// Correct the position and velocity 
position[0]=position[0]-kalman_gain[0]*velocity[0]-kalman_gain[1]*velocity[1]-kalman_gain[2]*velocity[2];
position[1]=position[1]-kalman_gain[3]*velocity[0]-kalman_gain[4]*velocity[1]-kalman_gain[5]*velocity[2];
position[2]=position[2]-kalman_gain[6]*velocity[0]-kalman_gain[7]*velocity[1]-kalman_gain[8]*velocity[2];
velocity_tmp[0]=velocity[0]-kalman_gain[9]*velocity[0]-kalman_gain[10]*velocity[1]-kalman_gain[11]*velocity[2];
velocity_tmp[1]=velocity[1]-kalman_gain[12]*velocity[0]-kalman_gain[13]*velocity[1]-kalman_gain[14]*velocity[2];
velocity_tmp[2]=velocity[2]-kalman_gain[15]*velocity[0]-kalman_gain[16]*velocity[1]-kalman_gain[17]*velocity[2];

precision delta_roll=kalman_gain[18]*velocity[0]+kalman_gain[19]*velocity[1]+kalman_gain[20]*velocity[2];
precision delta_pitch=kalman_gain[21]*velocity[0]+kalman_gain[22]*velocity[1]+kalman_gain[23]*velocity[2];
precision delta_yaw=kalman_gain[24]*velocity[0]+kalman_gain[25]*velocity[1]+kalman_gain[26]*velocity[2];

//Update the velocity state with the temporary velocity state 
velocity[0]=velocity_tmp[0];
velocity[1]=velocity_tmp[1];
velocity[2]=velocity_tmp[2];


// Correct the rotation matrix
mat3 new_rotmat;

new_rotmat[0]=Rb2t[0]-delta_yaw*Rb2t[3] + delta_pitch*Rb2t[6];
new_rotmat[1]=Rb2t[1]-delta_yaw*Rb2t[4] + delta_pitch*Rb2t[7];
new_rotmat[2]=Rb2t[2]-delta_yaw*Rb2t[5] + delta_pitch*Rb2t[8];
new_rotmat[3]=delta_yaw*Rb2t[0] + Rb2t[3] - delta_roll*Rb2t[6];
new_rotmat[4]=delta_yaw*Rb2t[1] + Rb2t[4] - delta_roll*Rb2t[7];
new_rotmat[5]=delta_yaw*Rb2t[2] + Rb2t[5] - delta_roll*Rb2t[8];
new_rotmat[6]=-delta_pitch*Rb2t[0] + delta_roll*Rb2t[3] + Rb2t[6];
new_rotmat[7]=-delta_pitch*Rb2t[1] + delta_roll*Rb2t[4] + Rb2t[7];
new_rotmat[8]=-delta_pitch*Rb2t[2] + delta_roll*Rb2t[5] + Rb2t[8];


// Calculate the corrected quaternions
rotation2quat(quaternions,new_rotmat);
}


void ZUPT_detector(void){
	
/************ Calculate the mean of the accelerations in the in-data buffer ***********/
uint8_t ctr;
vec3 acceleration_mean={0,0,0};		//Mean acceleration within the data window
	
for(ctr=0; ctr<detector_Window_size; ctr++){	
	acceleration_mean[0]=acceleration_mean[0]+acc_buffer_x_axis[ctr];
	acceleration_mean[1]=acceleration_mean[1]+acc_buffer_y_axis[ctr];
	acceleration_mean[2]=acceleration_mean[2]+acc_buffer_z_axis[ctr];
}

acceleration_mean[0]=acceleration_mean[0]/detector_Window_size;
acceleration_mean[1]=acceleration_mean[1]/detector_Window_size;
acceleration_mean[2]=acceleration_mean[2]/detector_Window_size;
	
	
	
	
/****************** Calculate the likelihood ratio  *******************************/ 

									
precision acceleration_mean_norm = sqrt_hf(vecnorm2(acceleration_mean,3));
precision sigma2_acc_det=sigma_acc_det*sigma_acc_det;
precision sigma2_gyro_det=sigma_gyro_det*sigma_gyro_det;
vec3 acceleration_mean_normalized;
vec3 tmp1;
vec3 tmp2;

acceleration_mean_normalized[0]=(g*acceleration_mean[0])/acceleration_mean_norm;
acceleration_mean_normalized[1]=(g*acceleration_mean[1])/acceleration_mean_norm;
acceleration_mean_normalized[2]=(g*acceleration_mean[2])/acceleration_mean_norm;

Test_statistics=0;
for(ctr=0;ctr<detector_Window_size;ctr++){
	
	tmp1[0]=acc_buffer_x_axis[ctr]-acceleration_mean_normalized[0];
	tmp1[1]=acc_buffer_y_axis[ctr]-acceleration_mean_normalized[1];
	tmp1[2]=acc_buffer_z_axis[ctr]-acceleration_mean_normalized[2];
	
	tmp2[0]=gyro_buffer_x_axis[ctr];
	tmp2[1]=gyro_buffer_y_axis[ctr];
	tmp2[2]=gyro_buffer_z_axis[ctr];
	
	Test_statistics=Test_statistics+vecnorm2(tmp1,3)/sigma2_acc_det+vecnorm2(tmp2,3)/sigma2_gyro_det;	
}
Test_statistics=Test_statistics/detector_Window_size;


	/******************** Check if the test statistics T are below or above the detector threshold ******************/
	if(Test_statistics<detector_threshold){
	zupt=true;
	}  
	else{
	zupt=false;	
	}
	
}

//@}

/**
	\defgroup init Initialization routines
	\brief Routines for initializing the system. Only coarse initial alignment is implemented

	@{
*/

void initialize_navigation_algorithm(void){
	
	
// Counter that counts the number of initialization samples that have been processed.						
static uint8_t initialize_sample_ctr;	


// Mean acceleration vector used in the initial alignment.      
static vec3 acceleration_mean;	

// Reset the mean if we start a new initialization
if (initialize_sample_ctr==0)
{
	acceleration_mean[0]=0;
	acceleration_mean[1]=0;
	acceleration_mean[2]=0;	
}

//Calculate the cumulative sum of the accelerometer readings until we have the number 
//of samples specified in the initial alignment settings
if(initialize_sample_ctr<nr_of_inital_alignment_samples){
	
	//Cumulative sum
	acceleration_mean[0]=acceleration_mean[0]+accelerations_in[0];
	acceleration_mean[1]=acceleration_mean[1]+accelerations_in[1];
	acceleration_mean[2]=acceleration_mean[2]+accelerations_in[2];
	
	//Increase the counter	
	initialize_sample_ctr=initialize_sample_ctr+1;
}
	

// If we are at the last iteration of the initial alignment, do this: 
if(initialize_sample_ctr==nr_of_inital_alignment_samples){
			
vec3 initial_attitude;	
	 /************* Initialize the navigation states *************/
	
	// Calculate the mean acceleration from the cumulative sum. 	
	acceleration_mean[0]=acceleration_mean[0]/nr_of_inital_alignment_samples;
	acceleration_mean[1]=acceleration_mean[1]/nr_of_inital_alignment_samples;
	acceleration_mean[2]=acceleration_mean[2]/nr_of_inital_alignment_samples;


	//Calculate the roll and pitch
	initial_attitude[0]=atan2(-acceleration_mean[1],-acceleration_mean[2]);		//roll
	initial_attitude[1]=atan2(acceleration_mean[0],sqrt_hf((acceleration_mean[1]*acceleration_mean[1])+(acceleration_mean[2]*acceleration_mean[2]))); //pitch
	
	//Set the initial heading
	initial_attitude[2]=initial_heading;
	
	
	// Calculate the initial rotation matrix, used as temporary variable in the calculation of the initial quaternions 
	mat3 initial_rotmat;
	euler2rotation(initial_rotmat,initial_attitude);
	
	
	
	//Set the initial quaternions using the initial rotation matrix
	rotation2quat(quaternions,initial_rotmat);
	
	//Set the initial velocity (must be zero)
	velocity[0]=0;
	velocity[1]=0;
	velocity[2]=0;
	
	//Set the initial position
	position[0]=initial_pos[1];
	position[1]=initial_pos[1];
	position[2]=initial_pos[2];
	
	
	// Set the gravity magnitude based upon the latitude and height of the navigation platform. 
	gravity();
	
	/*************************************************************/
	
	
	
	/************** Initialize the filter covariance *************/ 
	cov_vector[0]=sigma_initial_position[0]*sigma_initial_position[0];
	cov_vector[9]=sigma_initial_position[1]*sigma_initial_position[1];
	cov_vector[17]=sigma_initial_position[2]*sigma_initial_position[2];
	

	cov_vector[24]=sigma_initial_velocity[0]*sigma_initial_velocity[0];
	cov_vector[30]=sigma_initial_velocity[1]*sigma_initial_velocity[1];
	cov_vector[35]=sigma_initial_velocity[2]*sigma_initial_velocity[2];
	
	
	cov_vector[39]=sigma_initial_attitude[0]*sigma_initial_attitude[0];
	cov_vector[42]=sigma_initial_attitude[1]*sigma_initial_attitude[1];
	cov_vector[44]=sigma_initial_attitude[2]*sigma_initial_attitude[2];
	/*************************************************************/
	
	//Reset the initialization ctr
	initialize_sample_ctr=0;

	//Turn of the initialization flag
	initialize_flag=false;
}


}

//@}

/**
	\defgroup calib Calibration routines
	\brief Calibration routines. Only accelerometer calibration is implemented
	since basic gyro calibration (bias) is trivial and is available on the IMU.
	
	@{
*/

void estimate_accelerometer_biases(void){
	
	vec3 tmp1;					//Aiding variable
	vec3 tmp2;					//Aiding variable
	precision tmp3;				//Aiding variable
	uint8_t index;				//Variable indicating the element index of the maximum value of a vector  		
	precision maximum_value;	//The maximum value of a vector			
	uint8_t orientation_check = 0;	//Variable used in the check of the goodness of the user chosen orientations. 	
	
	
	// Set the magnitude of the gravity magnitude based upon the latitude and height of the navigation system.
	gravity();
	
	//*********************** Check if the excitation is sufficient *****************************************//
	
	/*
	
	This is a add-hoc way to check if the excitation to the accelerometer bias estimation algorithm is sufficient.
	 	
	Check that the projections of the gravity vector onto the axes of the accelerometer cluster during the calibration have been such that: 
	 1) The largest elements have been along the x,y, and z axes.
	 2) The magnitude of the largest element at each orientation is close to the gravity magnitude g.
	 	   
	*/	      
	for(uint8_t orientation_ctr=0;orientation_ctr<nr_of_calibration_orientations;orientation_ctr++){   
	
		tmp1[0]=absf(acceleration_mean_matrix[0][orientation_ctr]);
		tmp1[1]=absf(acceleration_mean_matrix[1][orientation_ctr]);
		tmp1[2]=absf(acceleration_mean_matrix[2][orientation_ctr]);
	
		max_value(&maximum_value,&index,tmp1);
		
		//Test if the absolute value of the largest element in the  acceleration mean vector at orientation given by "orientation_ctr" is within cos(pi/18) of g.  
		if((maximum_value-cos(M_PI/18)*g)>0)
		{
			orientation_check|=(1<<index);						
		}   
	}
	
	// If the orientation_check variable is not equal to 7 then we should send an error message to the user 
	// telling him that the estimated accelerometer bias values may be of poor quality   		   
	if(orientation_check!=7)
	{
		error_signal=ACC_CALIBRATION_ILLCONDITIONED;			
	}
	
	
	//***************************** NOW LET US ESTIMATES THE BIASES *************************//
	
	// Reset the accelerometer biases
	accelerometer_biases[0]=0;
	accelerometer_biases[1]=0;
	accelerometer_biases[2]=0;
	
	
	// Outer loop of the estimation algorithm 
	for(uint8_t itr_ctr=0; itr_ctr<30; itr_ctr++){
		
		tmp1[0]=0;
		tmp1[1]=0;
		tmp1[2]=0;
		
		//Inner loop in the estimation algorithm  
		for(uint8_t orientation_ctr=0;orientation_ctr<nr_of_calibration_orientations;orientation_ctr++){
		
			// Update the aiding variable tmp2  
			tmp2[0]=acceleration_mean_matrix[0][orientation_ctr]-accelerometer_biases[0];
			tmp2[1]=acceleration_mean_matrix[1][orientation_ctr]-accelerometer_biases[1];
			tmp2[2]=acceleration_mean_matrix[2][orientation_ctr]-accelerometer_biases[2];
		
			//Update the cumulative sum of the "new" accelerometer bias  
			tmp3=sqrt_hf(vecnorm2(tmp2,3));
			tmp1[0]=tmp1[0]+acceleration_mean_matrix[0][orientation_ctr]-g*(tmp2[0]/tmp3);
			tmp1[1]=tmp1[1]+acceleration_mean_matrix[1][orientation_ctr]-g*(tmp2[1]/tmp3);
			tmp1[2]=tmp1[2]+acceleration_mean_matrix[2][orientation_ctr]-g*(tmp2[2]/tmp3);	
			
		}
		
		// Calculate the "new" bias estimate from the cumulative sum 
		accelerometer_biases[0]=tmp1[0]/nr_of_calibration_orientations;
		accelerometer_biases[1]=tmp1[1]/nr_of_calibration_orientations;
		accelerometer_biases[2]=tmp1[2]/nr_of_calibration_orientations;		
	}
}

void calibrate_accelerometers(void){
	
	static uint32_t sample_ctr=0;
	static uint8_t orientation_ctr=0;
	static vec3 acceleration_second_moment={0,0,0};						//Vector of the second order moments (the power) of the accelerometer measurements during the calibration [(m/s^2)^2]
	vec3 acceleration_variance;											//Vector of the variances of of the accelerometer measurements during the calibration [(m/s^2)^2]  	
	precision tmp;
	
		
	
	// While the number of samples taken at the current orientation are less then the value specified. Do the following 
	if(sample_ctr<nr_of_calibration_samples){
		
		tmp=((precision)(sample_ctr))/(sample_ctr+1);
	
		// Update the recursively calculated mean of the acceleration measurements	
		acceleration_mean_matrix[0][orientation_ctr]=tmp*acceleration_mean_matrix[0][orientation_ctr]+accelerations_in[0]/(sample_ctr+1);	
		acceleration_mean_matrix[1][orientation_ctr]=tmp*acceleration_mean_matrix[1][orientation_ctr]+accelerations_in[1]/(sample_ctr+1);	
		acceleration_mean_matrix[2][orientation_ctr]=tmp*acceleration_mean_matrix[2][orientation_ctr]+accelerations_in[2]/(sample_ctr+1);	
	
		// Update the recursively calculated variance of the acceleration measurements	
		acceleration_second_moment[0]=tmp*acceleration_second_moment[0]+(accelerations_in[0]*accelerations_in[0])/(sample_ctr+1);
		acceleration_second_moment[1]=tmp*acceleration_second_moment[1]+(accelerations_in[1]*accelerations_in[1])/(sample_ctr+1);
		acceleration_second_moment[2]=tmp*acceleration_second_moment[2]+(accelerations_in[2]*accelerations_in[2])/(sample_ctr+1);
	
		//Update the sample counter
		sample_ctr=sample_ctr+1;
	}
	
	
	// If the number of samples taken at the current orientation are equal to the value specified. Do the following 
	if(sample_ctr==nr_of_calibration_samples)
	{
		//Reset the sample counter
		sample_ctr=0;
	
		//Calculate the variance 
		acceleration_variance[0]=acceleration_second_moment[0]-acceleration_mean_matrix[0][orientation_ctr]*acceleration_mean_matrix[0][orientation_ctr];
		acceleration_variance[1]=acceleration_second_moment[1]-acceleration_mean_matrix[1][orientation_ctr]*acceleration_mean_matrix[1][orientation_ctr];
		acceleration_variance[2]=acceleration_second_moment[2]-acceleration_mean_matrix[2][orientation_ctr]*acceleration_mean_matrix[2][orientation_ctr];
	
		//Check that IMU has been stationary during the calibration. If so do the following 
		if((acceleration_variance[0]<acceleration_variance_threshold) & (acceleration_variance[1]<acceleration_variance_threshold) & (acceleration_variance[2]<acceleration_variance_threshold))
		{
		
			
			//Check if this was the last orientation in the calibration procedure, then do the following 
			if(orientation_ctr==(nr_of_calibration_orientations-1))
			{
			
			// Estimate the bias estimation algorithm
			estimate_accelerometer_biases();
			
			//Set the flag that signals that the calibration was successful.
			acc_calibration_finished_flag=true;
			 
			
			//Reset the orientation counter
			orientation_ctr=0;
			}
			else
			{
				
			//Send a message to the user that data was successfully calculated and the IMU should be place in a new orientation   	
			new_orientation_flag=true;
			
			//Update the orientation counter
			orientation_ctr=orientation_ctr+1;	
			}										
		} 
		else
		{ 
			
			//Send a message to the user that the IMU wasn't stationary during the data collection at the current orientation 
			error_signal=ACC_STATIONARITY_ERROR;
			
		}				
		
	}
	
}	

//@}

/**
	\addtogroup miscel
	@{
*/

/// Routine collecting the functions which need to be run to make a ZUPT update.
void zupt_update(void){
	if(zupt)
	{
	
		//Calculate the Kalman filter gain
		gain_matrix();	
	
		//Correct the navigation states
		correct_navigation_states();	
	
		//Update the covariance matrix
		measurement_update();		
	}
}

//@}

/**
	\defgroup stepwise Step-wise dead reckoning routines
	\brief Routines for running step-wise dead reckoning. These routines serve
	to reset the filter at every step and transmit the displacement and heading
	change estimates together with related covariances to the user. Thereby the
	user can run "step-wise" dead reckoning based on this data.

	@{
*/
void extract_step_data(void);
void extract_step_data(void){
	// Get relative position estimates
	dx[0]=position[0];
	dx[1]=position[1];
	dx[2]=position[2];
	// Get relative heading estimate
	dx[3]=atan2(Rb2t[3],Rb2t[0]);
	
	// Get related covariance estimates
	dP[0]=cov_vector[0];
	dP[1]=cov_vector[1];
	dP[2]=cov_vector[2];
	dP[3]=cov_vector[8];
	////////////////////	
	dP[4]=cov_vector[9];
	dP[5]=cov_vector[10];
	dP[6]=cov_vector[16];
	////////////////////
	dP[7]=cov_vector[17];
	dP[8]=cov_vector[23];
	////////////////////
	dP[9]=cov_vector[44];
}

void reset_filter(void);
void reset_filter(void){
	
	// Reset position
	position[0]=0;
	position[1]=0;
	position[2]=0;

	// Reset heading	
	vec3 euler;
	rotation2euler(euler,Rb2t);
	euler[2] = 0;
	euler2rotation(Rb2t,euler);
	rotation2quat(quaternions,Rb2t);
	
	// Reset covariance estimates
	// x-terms
	cov_vector[0] = 0;
	cov_vector[1] = 0;
	cov_vector[2] = 0;
	cov_vector[3] = 0;
	cov_vector[4] = 0;
	cov_vector[5] = 0;
	cov_vector[6] = 0;
	cov_vector[7] = 0;
	cov_vector[8] = 0;
	// y-terms
	cov_vector[9] = 0;
	cov_vector[10] = 0;
	cov_vector[11] = 0;
	cov_vector[12] = 0;
	cov_vector[13] = 0;
	cov_vector[14] = 0;
	cov_vector[15] = 0;
	cov_vector[16] = 0;
	// z-terms
	cov_vector[17] = 0;
	cov_vector[18] = 0;
	cov_vector[19] = 0;
	cov_vector[20] = 0;
	cov_vector[21] = 0;
	cov_vector[22] = 0;
	cov_vector[23] = 0;
	// heading-terms
	cov_vector[29] = 0;
	cov_vector[34] = 0;
	cov_vector[38] = 0;
	cov_vector[41] = 0;
	cov_vector[43] = 0;
	cov_vector[44] = 0;
}

void stepwise_system_reset(void) {
	// Local counters	
	static uint16_t time_since_last_reset = 0;
	static uint16_t time_since_nec_cond = 0;
	
	// No reset per default
	filter_reset_flag=false;
	time_since_last_reset++;

	//TODO: Change to the simple covariance threshold to some better "coupling" measure.
	
	// Check if necessary conditions for reset are met (low coupling AND reset spacing)
	if (cov_vector[24]<reset_cov_threshold && time_since_last_reset>min_time_between_resets){
		time_since_nec_cond++;
		// Check if sufficient conditions for reset are met (end of OR sufficiently long stationary period)
		if ( !zupt || time_since_nec_cond>max_time_reset_pending) {
			// Save displacement, heading change, and covariance estimates before reset.
			extract_step_data();
			// Set reset flag to trigger transmission of above.
			filter_reset_flag = true;
			// Do reset
			reset_filter();
			// Reset local counters
			time_since_last_reset = 0;
			time_since_nec_cond = 0;
		}
	}
	
}

//@}

//@}
