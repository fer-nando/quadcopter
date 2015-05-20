/*
 * rc.h
 *
 *  Created on: 04/02/2015
 *      Author: Fernando
 */

#ifndef RC_H_
#define RC_H_

#include "inc/hw_types.h"

// functions
void RCInit();
tBoolean RCGetValues(int * rc);
void RCLowPassFilter(int num);


#endif /* RC_H_ */
