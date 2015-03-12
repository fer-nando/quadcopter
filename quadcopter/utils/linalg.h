/*
 * linalg.h
 *
 *  Created on: 13/12/2014
 *      Author: Fernando1
 */

#ifndef LINALG_H_
#define LINALG_H_

// definitions
#define rad2deg(x)          (x*57.2957795130823208767)
#define min(a,b)            ((a < b) ? a : b)
#define max(a,b)            ((a > b) ? a : b)
#define dot2(a,b)           (a[0]*b[0]+a[1]*b[1])
#define dot3(a,b)           (a[0]*b[0]+a[1]*b[1]+a[2]*b[2])
#define norm2(a)            (sqrt(dot2(a,a)))
#define norm3(a)            (sqrt(dot3(a,a)))
#define constrain(x,a,b)    (min(max(x,a),b))

// structures
typedef struct vector {
  float x, y, z;
} vector;

// functions
void Inv2(float* a, float* ia);
void Dot2(float* a, float* b, float *c);

#endif /* LINALG_H_ */
