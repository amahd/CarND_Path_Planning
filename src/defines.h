/*
 * defines.h
 *
 *  Created on: Aug 22, 2017
 *      Author: s
 */

#ifndef DEFINES_H_
#define DEFINES_H_


#define MAX(a,b) ((a) > (b) ? a : b)
#define MIN(a,b) ((a) < (b) ? a : b)


#define LRGENUM   1000000 // very large number

#define LC 2 	//lane centre

#define LW 4   //lane width

#define SIM_TICK 0.02  // 20ms


#define DISTINC 0.5   //distance increment to adjust speed, not used

#define PATH_HORIZON 30.0   // path horizon for spline


#define LANE_HORIZON  30.0  // Check how far ahead inside your lane


#define PATH_POINTS 50    // Total points in the future path


#define REF_VEL 45.0

#define MAXCOST 10000



























#endif /* DEFINES_H_ */
