/*! \file
	\brief Header file for the OpenShoe navigation algorithm. 
	
	\details This is the header file for the OpenShoe navigation algorithm. It includes the function declarations for all 
	the signal processing functions needed to implement a zero-velocity aided inertial navigation system using a nine 
	state Kalman filter working in a complimentary feedback configuration. It also includes function declarations for 
	the functions needed to implement a accelerometer bias calibration framework.      
	
	\authors John-Olof Nilsson, Isaac Skog
 	\copyright Copyright (c) 2011 OpenShoe, ISC License (open source)
 */ 

/** \defgroup nav_eq OpenShoe filtering algorithms
	This group contain the filtering algorithms written for OpenShoe. That is, the ZUPT aided INS and calibration algorithms
	\ingroup nav_eq
	@{
*/


#ifndef NAV_EQ_H_
#define NAV_EQ_H_


#include "nav_types.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
//#include "compiler.h"


//************* Definitions *************//


/// Maximum numbers of orientations that can be used in the accelerometer calibration
#define MAX_ORIENTATIONS 12

/// Minimum numbers of orientations that can be used in the accelerometer calibration
#define MIN_ORIENTATIONS 3

/// Value returned in the error message if an error occurred in the matrix inversion. 
#define MATRIX_INVERSION_ERROR 1

/// Value returned in the error message if the IMU was not stationary during the accelerometer calibration.  
#define ACC_STATIONARITY_ERROR 2

/// Value returned in the error message if the orientations used in the accelerometer calibration were poorly chosen. 
#define ACC_CALIBRATION_ILLCONDITIONED 3

/// Value returned in the error message if the number of orientations specified for the accelerometer calibration is to large. It has been changed to 12.
#define NUMBER_OF_ORIENTATIONS_TO_LARGE 4   

/// Value returned in the error message if the number of orientations specified for the accelerometer calibration is to few. It has been changed to 3. 
#define NUMBER_OF_ORIENTATIONS_TO_SMALL 5   

/// Absolute value of a floating point variable.
#define absf(a)(a>0 ? a:-a)


//************* Function declarations  **************//



/*! \brief Function for calibrating the accelerometer biases
	

	\details This function is used to calibrate the biases of the accelerometers in the IMU and requires the users to place 
	the IMU into at least three different orientations. The calibration method is based upon the calibration algorithm described 
	in the paper <A href="https://eeweb01.ee.kth.se/upload/publications/reports/2010/IR-EE-SB_2010_046.pdf">Calibration of the 
	Accelerometer Triad of an Inertial Measurement Unit, Maximum Likelihood Estimation and Cramer-Rao Bound</A>, but does only 
	calibrate the accelerometer bias.       
	
	Before the calibration is started the flags \a new_orientation_flag and \a acc_calibration_successful_flag should be set to false. 
	Then the function should called every time new IMU-data have been read from the IMU and as long as the new flags \a new_orientation_flag
	and \a acc_calibration_successful_flag are false. When the flag \a new_orientation_flag becomes true a message should be sent 
	to the user, which then should place the IMU in a new orientation and reset the flag. When the flag \a acc_calibration_successful_flag
	becomes true the calibration is finished and the accelerometer calibration parameters have been written into the memory of the IMU. 
	
	
	\note	The function can return one error and one warning message that are stored in the variable #error_vec. The error message may 
	be sent if the was no stationary during one of the calibration phases and new accelerometer data most be recorded with the IMU in 
	the same orientation. A warning message may be sent if the orientations the IMU was placed in may have caused a poor estimate of 
	the accelerometer biases.   	
	
	 @param[out]		error_vec						A variable that is set non zero value of an error/warning has occurred during the 
	 @param[in,out]		acc_calibration_successful_flag	A flag that should be false when calibration is started and that becomes 
														true when the calibration is finished.  
	 @param[in,out]		new_orientation_flag			A flag that should be false when calibration is started and that becomes 
														true when IMU should be placed in a new orientation. When the IMU has been placed in 
														a new orientation it should be set to false.    
														execution of the function.
	 @param[in]			accelerations_in				The from the IMU latest read acceleration data.
	 
	 	 
*/ 
void calibrate_accelerometers(void);

/*! \brief	Function that estimates the accelerometer biases given a matrix of 
	the mean of the measured acceleration at different orientations. 
	
	@param[out] max_v		Largest value of the input vector.
	@param[out] index		Index of the vector element holding the largest value. 
	@param[in]	arg_vec		The input vector.		 
 */	
void estimate_accelerometer_biases(void);

/*! \brief Function for correcting the navigation states given a zero-velocity detection.
	

	\details When called this function takes the velocity estimate of the navigation system and treat it as an observation of 
	the current velocity error of the system. The errors in all navigation states (position, velocity, and attitude) are then 
	estimated by multiplying the "velocity error" with the Kalman gain. The estimated errors in the navigation states are 
	then used to correct the navigation states.             
	
	 @param[in,out] position		The position estimate of the navigation system.
	 @param[in,out] velocity		The velocity estimate of the navigation system.
	 @param[out]	quaternions		The orientation estimate of the navigation system.
	 @param[in]		Rb2t			The vector representation of the body to navigation coordinate system rotation matrix estimate.
	 @param[in]		kalman_gain		The vector representation of the Kalman filter gain matrix.	 
*/ 
void correct_navigation_states(void);



/*! \brief Function for calculating the Kalman filter gain matrix.
	

	\details When called the function calculates the Kalman filter gain and store it in the vector \a kalman_gain. 
	To calculate the gain the function uses the Kalman filter covariance and the pseudo velocity measurement noise 
	standard deviation.    
	
	 @param[out] kalman_gain		The vector representation of the Kalman filter gain matrix. 
	 @param[in] cov_vector			The vector representation of the Kalman filter covariance matrix. 
	 @param[in] sigma_velocity		The standard deviation of the pseudo velocity measurement error (Note: This parameters enters the function through a sub-function.).
 */ 
void gain_matrix(void);



/*! \brief Function for initializing the navigation algorithm.
	

	\details This function initializes the navigation algorithm and should during the initialization of the 
	navigation system be called every time new IMU-data have been read from the IMU. Before the initialization is 
	started the flag \a initialize_flag should be set to true, and the counter \a initialize_sample_ctr to zero. 
	The initialization is finished when the flag \a initialize_flag becomes false.    
	
	The initialization function first runs an initial alignment of the navigation system, where the 
	roll and pitch are estimated from the average of the accelerometer readings. Then, the function sets the 
	initial navigation states (position, velocity, and quaternions) and the initial Kalman filter covariance. 
	
	\note	The navigation system most be stationary during the initialization, and the number of samples used in the 
			initial alignment most be larger than the length of the zero-velocity detector window.            
	
	 @param[out]	position						The position estimate of the navigation system.
	 @param[out]	velocity						The velocity estimate of the navigation system.
	 @param[out]	quaternions						The orientation estimate of the navigation system.
	 @param[out]	cov_vector						The vector representation of the Kalman filter covariance matrix.
	 @param[in,out]	initialize_flag					A flag that should be set to true when initialization is started and that becomes false when the initialization is finished.  
	 @param[in]		nr_of_inital_alignment_samples	The number of samples used in the initial alignment. 
	 @param[in]		initial_heading					The initial heading of the navigation system.
	 @param[in]		initial_pos						The initial position of the navigation system.
	 @param[in]		sigma_initial_position			The standard deviations of the uncertainties in the initial position. 
	 @param[in]		sigma_initial_velocity			The standard deviations of the uncertainties in the initial velocity. 
	 @param[in]		sigma_initial_attitude			The standard deviations of the uncertainties in the initial attitude.   
	 	 
*/ 
void initialize_navigation_algorithm(void);




/*! \brief Function for doing a measurement update of the Kalman filter covariance.
	

	\details When called the function does a measurement update of the Kalman filter state covariance matrix stored in the vector \a cov_vector.   
	
	 @param[in,out] cov_vector		The vector representation of the Kalman filter covariance matrix. 
	 @param[in] kalman_gain			The vector representation of the Kalman filter gain matrix.
*/ 
void measurement_update(void);



/*! \brief Function for doing a time update of the mechanized navigation equations.
	

	\details When called the function does a time update of the mechanized inertial navigation system equations. That is, first the quaternions stored in \a quaternions 
	is updated using the angular rate measurements in \a angular_rates_out. Then the position and velocity state vectors \a position and \a velocity are updated 
	using the acceleration measurements in \a accelerations_out. The function also updates the ''aiding'' vector \a Rb2t (body to navigation coordinate system rotation matrix) which is 
	used in the Kalman filter. 
	
	\note   This is a rudimentary mechanization of the inertial navigation equations which only is suitable for
			use with systems that uses low-cost/low-performance inertial sensors and where only short periods of 
			free inertial navigation is expected.   	          
	
	 @param[in,out] position		The position estimate of the navigation system.
	 @param[in,out] velocity		The velocity estimate of the navigation system.
	 @param[in,out] quaternions		The orientation estimate of the navigation system.
	 @param[out] Rb2t				The body to navigation coordinate system rotation matrix estimate.	 
	 @param[in] accelerations_out	The acceleration measurements used in the update of the inertial navigation system equations.
	 @param[in] angular_rates_out	The angular rate measurements used in the update of the inertial navigation system equations.
	 @param[in] dt					The sampling period of the system.
	 @param[in] g					The magnitude of the local gravity vector.
 */ 
void strapdown_mechanisation_equations(void);



/*! \brief Function for doing a time update of the Kalman filter state covariance.
	

	\details When called the function does a time update of the Kalman filter state covariance matrix stored in the vector \a cov_vector.   
	
	 @param[in,out] cov_vector		The vector representation of the Kalman filter covariance matrix. 
	 @param[in] dt					The sampling period of the system.
	 @param[in] sigma_acceleration	The standard deviation of the accelerometer process noise.
	 @param[in] sigma_gyroscope		The standard deviation of the gyroscope process noise.	 
	 @param[in] accelerations_out	The acceleration measurements used in the update of the inertial navigation system equations.
	 @param[in] Rb2t				The vector current body to navigation coordinate system rotation matrix estimate.
 */ 
void time_up_data(void);



/*! \brief Function that updates the IMU data buffers with the latest values read from the IMU, and writes 
	the IMU data to that should be process at the current iteration to the processing variables. 
	

	\details The function updates the IMU data buffers \a acc_buffer_x_axis, \a acc_buffer_y_axis, \a acc_buffer_z_axis, \a gyro_buffer_x_axis, \a gyro_buffer_y_axis, 
	\a gyro_buffer_z_axis with the values stored in the vectors \a accelerations_in and \a angular_rates_in. The function also updates the vectors 
	\a accelerations_out and \a angular_rates_out. The data stored in these vectors is the data processed in the next iteration of the navigation algorithm. 
	
	\note This function should be called after data have been read from the IMU through the SPI interface and before the navigation algorithm is processed.
	
	 @param[out] accelerations_out The acceleration data that is to be used in the next iteration of the navigation algorithm.
	 @param[out] angular_rates_out The angular rate data that is to be used in the next iteration of the navigation algorithm.
	 @param[in] accelerations_in The from the IMU latest read acceleration data.
	 @param[in] angular_rates_in The from the IMU latest read angular rate data. 
 */ 
void update_imu_data_buffers(void);


/*! \brief Function for detecting when the system has zero-velocity.
	

	\details When called this function takes the acceleration and angular rate measurements stored in the IMU data buffers and 
	runs a generalized likelihood ratio test to determine if the navigation system is stationary or moving. More information 
	about the detector and it characteristics can be found in following papers:
	
	\li <A href="https://eeweb01.ee.kth.se/upload/publications/reports/2010/IR-EE-SB_2010_038.pdf">Zero-Velocity Detection -- An Algorithm Evaluation</A>                    
	\li <A href="https://eeweb01.ee.kth.se/upload/publications/reports/2010/IR-EE-SB_2010_043.pdf">Evaluation of Zero-Velocity Detectors for Foot-Mounted Inertial Navigation Systems</A>                     
	
	
	 @param[out]	zupt					The zero-velocity detection flag 
	 @param[in]		detector_Window_size	The window size of the zero-velocity detector.
	 @param[in]		detector_threshold		The threshold used in the detector.
	 @param[in]		sigma_acc_det			The standard deviation figure used to control the importance of the accelerometer measurements in the detection algorithm.    
	 @param[in]		sigma_gyro_det			The standard deviation figure used to control the importance of the gyroscope measurements in the detection algorithm.
	 @param[in]		g						The magnitude of the local gravity vector.	 
*/ 
void ZUPT_detector(void);


/*! \brief	Wrapper function that checks if a zero-velocity update should be done, and then calls all navigation algorithm 
			functions that should be executed during a zero-velocity update.   
	

	\details Wrapper function that checks if a zero-velocity update should be done, and then calls all navigation algorithm 
			functions that should be executed during a zero-velocity update. The function first calls \a ZUPT_detector. If 
			then flag \a zupt is set to true, it also calls the following functions:
			
			\li gain_matrix	
			\li correct_navigation_states	
			\li measurement_update	 
*/ 
void zupt_update(void);


/*! \brief	Checks conditions for and execute system reset for step-wise dead reckoning.   
	
	\details The functions checks for the conditions for filter reset of the ZUPT-aided INS. The filter is reset if: the coupling
	between the velocity states, the pitch, and the roll falls below a threshold and then start to increase again (no zupt) or the
	system is stationary long enough, and there is sufficient spacing from last reset. The reset means that the position and
	heading is set to zero and all cross covariances to other states are set to zero. The reset also triggers the transmission of
	the relative displacement, heading change, and covariance estimates since last reset.
			
	@param[in/out]	cov_vector	Used to check if we should allow a reset and subsequently reset if this is the case
	@param[in/out]	position	Copied to dx and set to zero in case of reset.
	@param[in/out]	Rb2t		Copied (heading) and heading set to zero in case of reset.
	@param[out]		quaternions	Heading set to zero in case of reset.
	@param[out]		dx			Displacement and heading change since last reset.
	@param[out]		dP			Covariance of displacement and heading change since last reset.
*/ 
void stepwise_system_reset(void);


#endif /* NAV_EQ_H_ */

//@}
