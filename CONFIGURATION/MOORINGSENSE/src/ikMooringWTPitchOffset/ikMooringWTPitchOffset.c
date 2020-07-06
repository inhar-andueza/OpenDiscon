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
 * @file ikMooringWTPitchOffset.c
 *
 * @brief Mooringsense wind turbine controller input modification
 */
#include "ikMooringWTPitchOffset.h"

void ikMooringPitchLutbl_init(ikMooringWTCon *self) {
	int err;
	/* initialise look-up tables */
	ikLutbl_init(&(self->priv.lutblPitchOffset));
	err = ikLutbl_setPoints((&self->priv.lutblPitchOffset), self->priv.pitchOffsetTableN, self->priv.pitchOffsetTableX, self->priv.pitchOffsetTableY);
}

void ikMooringPitchLutbl_initParams(ikMooringWTCon *self) {

	/* make the minimum pitch 0 */
	self->priv.pitchOffsetTableN = 1;
	self->priv.pitchOffsetTableX[0] = 0.0;
	self->priv.pitchOffsetTableY[0] = 1.0;
}

void ikMooringWTPitchOffset_step(ikMooringWTCon *self, double T) {
	static double t;
	double speed;
	double pitch;

	speed = self->in.generatorSpeed / self->in.maximumSpeed;

	pitch = ikLutbl_eval(&(self->priv.lutblPitchOffset), speed);

	if (pitch - self->priv.minimumPitchOffset > 0) {
		self->priv.minimumPitchOffset = 5.0 * T + self->priv.minimumPitchOffset;
		t = 0.0;
	}
	else if (pitch - self->priv.minimumPitchOffset < 0 && t > 2.0) {
		self->priv.minimumPitchOffset = -0.5 * T + self->priv.minimumPitchOffset;
	} 

	t = t + T;
}
