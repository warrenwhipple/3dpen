/*
 * nav_types.h
 *
 * Created: 2011-10-13 15:50:57
 *  Author: skog
 */ 


#ifndef NAV_TYPES_H_
#define NAV_TYPES_H_


// Define the "precision" of our processor 
typedef float precision;

// ZUPT-aided INS filter types
typedef precision vec3[3];
typedef precision mat3[9];
typedef precision mat3sym[6];
typedef precision mat9sym[45];
typedef precision quat_vec[4];
typedef precision mat9by3[27];

// Step-wise dead reckoning data types
typedef precision vec4[4];
typedef precision mat4sym[10];

#endif /* NAV_TYPES_H_ */