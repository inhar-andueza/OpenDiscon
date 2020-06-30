/*
Copyright (C) 2017 IK4-IKERLAN

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

#define NINT(a) ((a) >= 0.0 ? (int) ((a)+0.5) : ((a)-0.5))

#include "ikMooringWTConfig.h"
#include "OpenDiscon_EXPORT.h"
#include <stdio.h>

void OpenDiscon_EXPORT DISCON(float *DATA, int FLAG, const char *INFILE, const char *OUTNAME, char *MESSAGE) {
	int err;
	static ikMooringWTCon con;
	double output = -12.0;
	static FILE *f = NULL;
	const double deratingRatio = 0.00; /* later to be got via the supercontroller interface */
	static int po = 0;
	static int pof = 0;
	static int tpo = 0;
		
	if (NINT(DATA[0]) == 0) {
		ikMooringWTConParams param;
		ikMooringWTCon_initParams(&param);
		setParams(&param);
		ikMooringWTCon_init(&con, &param);
		f = fopen("log.bin", "wb");
	}
//TODO lower maximum torque according to maximum power with derating (it may be time to bring the power manager back)
	con.in.deratingRatio = deratingRatio;
	con.in.externalMaximumTorque = 212.0; /* kNm */
	con.in.externalMinimumTorque = 0.0; /* kNm */
	con.in.externalMaximumPitch = 90.0; /* deg */
	/*con.in.externalMinimumPitch = 0.0; /* deg */
	con.in.generatorSpeed = (double) DATA[19]; /* rad/s */
	con.in.maximumSpeed = 480.0/30*3.1416; /* rpm to rad/s */

	if ((double)DATA[22] > 0.85*con.in.externalMaximumTorque*1000.0 && po < 1)	po = 1;
	else if ((double)DATA[22] > 0.9*con.in.externalMaximumTorque*1000.0 && po < 2)	po = 2;
	else if ((double)DATA[22] > 0.95*con.in.externalMaximumTorque*1000.0 && po < 3)	po = 3;
	else if ((double)DATA[22] > 1.0*con.in.externalMaximumTorque*1000.0 && po < 4)	po = 4;
	else if ((double)DATA[22] < 0.85*con.in.externalMaximumTorque*1000.0) pof = 1;
	else pof = 0;

	if (pof) tpo++;
	else tpo = 0.0;
	if (tpo > 26.0 / ((double)DATA[2])) { po = 0; tpo = 0.0; pof = 0; }

	switch (po) {
	case 1:
		if (con.in.externalMinimumPitch < 8.0*(1.0 / 4.0)) {
			con.in.externalMinimumPitch = 10.0 * DATA[2] + con.in.externalMinimumPitch;
		}
		else con.in.externalMinimumPitch = 8.0 * (1.0 / 4.0);
		break;
	case 2:
		if (con.in.externalMinimumPitch < 8.0*(1.0 / 2.0)) {
			con.in.externalMinimumPitch = 10.0 * DATA[2] + con.in.externalMinimumPitch;
		}
		else con.in.externalMinimumPitch = 8.0 * (1.0 / 2.0);
		break;
	case 3:
		if (con.in.externalMinimumPitch < 8.0*(3.0 / 4.0)) {
			con.in.externalMinimumPitch = 10.0 * DATA[2] + con.in.externalMinimumPitch;
		}
		else con.in.externalMinimumPitch = 8.0 * (3.0 / 4.0);
		break;
	case 4:
		if (con.in.externalMinimumPitch < 8.0) {
			con.in.externalMinimumPitch = 10.0 * DATA[2] + con.in.externalMinimumPitch;
		}
		else con.in.externalMinimumPitch = 8.0;
		break;
	default:
		con.in.externalMinimumPitch = 0.0;
	}
	
	ikMooringWTCon_step(&con);
	
	DATA[46] = (float) (con.out.torqueDemand*1.0e3); /* kNm to Nm */
	DATA[41] = (float) (con.out.pitchDemandBlade1/180.0*3.1416); /* deg to rad */
	DATA[42] = (float) (con.out.pitchDemandBlade2/180.0*3.1416); /* deg to rad */
	DATA[43] = (float) (con.out.pitchDemandBlade3/180.0*3.1416); /* deg to rad */
	DATA[44] = (float) (con.out.pitchDemandBlade1/180.0*3.1416); /* deg to rad (collective pitch angle) */

	err = ikMooringWTCon_getOutput(&con, &output, "maximum torque");
	fwrite(&(output), 1, sizeof(output), f);
}	
