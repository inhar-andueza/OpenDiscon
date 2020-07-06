/*
Copyright (C) 2020 IKERLAN

This file is part of OpenDiscon.
 
OpenDiscon is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
 
OpenDiscon is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
 
You should have received a copy of the GNU General Public License
along with OpenDiscon. If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file ikMooringWTPitchOffset.h
 * 
 * @brief Mooringsense wind turbine controller input modification
 */

#ifndef IKMOORINGWTPITCHOFFSET_H
#define IKMOORINGWTPITCHOFFSET_H

#ifdef __cplusplus
extern "C" {
#endif
	
#include "ikMooringWTCon.h"
#include "ikLutbl.h"
#include <stdio.h>

	void ikMooringPitchLutbl_init(ikMooringWTCon *self);
	void ikMooringPitchLutbl_initParams(ikMooringWTCon *self);
	void ikMooringWTPitchOffset_step(ikMooringWTCon *self, double T);
	
#ifdef __cplusplus
}
#endif

#endif /* IKMOORINGWTPITCHOFFSET_H */

