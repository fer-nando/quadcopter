/*
 * rc.h
 *
 *  Created on: 04/02/2015
 *      Author: Fernando
 */

#ifndef RC_H_
#define RC_H_


// functions
void RCInit();
void RCGetValues(int * rc);
void RCLowPassFilter(int num);


#endif /* RC_H_ */
