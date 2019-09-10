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

#include "ikClwindconWTConfig.h"
#include "OpenDiscon_EXPORT.h"
#include <stdio.h>

void OpenDiscon_EXPORT DISCON(float *DATA, int FLAG, const char *INFILE, const char *OUTNAME, char *MESSAGE) {
	int err;
	static ikClwindconWTCon con;
	double output = -12.0;
	static FILE *f = NULL;
	static FILE *dr = NULL;
	static float deratingRatio; /* later to be got via the supercontroller interface */
	/*static int startup = 1;	*/
	/*static int shutdown = 0;*/
	static double minSpeed = 1000.0*3.1416/30.0;
	static int t = 0;

	if (NINT(DATA[0]) == 0) {
		ikClwindconWTConParams param;
		ikClwindconWTCon_initParams(&param);
		setParams(&param);
		ikClwindconWTCon_init(&con, &param);
		dr = fopen("DeratingValue.txt","rt");
		fscanf(dr,"%f",&deratingRatio);
		fclose(dr);
		f = fopen("log.bin", "wb");
	}
//TODO lower maximum torque according to maximum power with derating (it may be time to bring the power manager back)
	con.in.deratingRatio = (double) deratingRatio;
	con.in.externalMaximumTorque = 8376.58e-3; /* kNm */
	con.in.externalMinimumTorque = 0.0;
	con.in.externalMaximumPitch = 90.0; /* deg */
	con.in.externalMinimumPitch = 0.0; /* deg */
	con.in.generatorSpeed = (double) DATA[19]; /* rad/s */
	con.in.maximumSpeed = 1800.0/30.0*3.1416; /* rpm to rad/s */
	
	/*if (shutdown) {
		con.in.externalMaximumTorque = 0.0;
		con.in.externalMinimumPitch = 90.0;
	}*/

	/*if (con.in.generatorSpeed < minSpeed) con.in.externalMaximumTorque = 0.0;

	/*if( con.in.generatorSpeed > 0.95*con.in.maximumSpeed){		
		double pitchOffset;
		pitchOffset = 100.0*(con.in.generatorSpeed/con.in.maximumSpeed) - 95.0;
		con.in.externalMinimumPitch = pitchOffset;
	}*/

	ikClwindconWTCon_step(&con);

	if (con.out.torqueDemand < 0.0) con.out.torqueDemand = 0.0;
	
	DATA[46] = (float) (con.out.torqueDemand*1.0e3); /* kNm to Nm */
	//DATA[41] = (float) (con.out.pitchDemandBlade1/180.0*3.1416); /* deg to rad */
	//DATA[42] = (float) (con.out.pitchDemandBlade2/180.0*3.1416); /* deg to rad */
	//DATA[43] = (float) (con.out.pitchDemandBlade3/180.0*3.1416); /* deg to rad */
	DATA[44] = (float) (con.out.pitchDemandBlade1/180.0*3.1416); /* deg to rad (collective pitch angle) */

	/*if (DATA[1] > 90.0 && !shutdown) {
		if (con.in.generatorSpeed < minSpeed*0.9) con.in.externalMaximumTorque = 0.0;
		if (DATA[3] < 45.0*3.1416/180.0){
			DATA[44] = 5.0*3.1416/180.0 * DATA[2] + DATA[3];
		} else {
			DATA[44] = 10.0*3.1416/180.0 * DATA[2] + DATA[3];
		}
	}

	if (con.in.externalMaximumTorque <= 0.0 && DATA[3] >= 90.0*3.1416/180.0){
		shutdown = 1;
	}

	/*if ((DATA[3] <= 0.0 || DATA[44] >= DATA[3]) && DATA[1] > 30.0) startup = 0;
	//if (con.in.generatorSpeed > con.in.maximumSpeed && con.out.torqueDemand > con.in.externalMaximumTorque || DATA[3] <= 0.0) startup = 0;

	if (startup) {
		if (DATA[1] < 60.0) {
			DATA[46] = 0.0; 
			DATA[44] = 90.0*3.1416/180.0;
		} else if (DATA[3] > 40.0*3.1416/180.0) {
			DATA[46] = 0.0; 
			DATA[44] = -5.0*3.1416/180.0 * DATA[2] + DATA[3];
		} else if (DATA[19] < 300.0/30.0*3.1416) {
			DATA[46] = 0.0; 
			DATA[44] = -1.0*3.1416/180.0 * DATA[2] + DATA[3];
		} else if (DATA[19] < 360.0/30.0*3.1416) {
			DATA[46] = (float) (con.out.torqueDemand*1.0e3); 
			DATA[44] = -1.0*3.1416/180.0 * DATA[2] + DATA[3];
		} else if (DATA[19] > 360.0/30.0*3.1416) {
			DATA[46] = (float) (con.out.torqueDemand*1.0e3); 
			DATA[44] = -2.0*3.1416/180.0 * DATA[2] + DATA[3];
			if (con.in.generatorSpeed > con.in.maximumSpeed || con.out.torqueDemand > con.in.externalMaximumTorque) {
				DATA[44] = DATA[3];
			}
		} 	
	}*/

	err = ikClwindconWTCon_getOutput(&con, &output, "maximum torque");
	fwrite(&(output), 1, sizeof(output), f);
	err = ikClwindconWTCon_getOutput(&con, &output, "below rated torque");
	fwrite(&(output), 1, sizeof(output), f);
	err = ikClwindconWTCon_getOutput(&con, &output, "collective pitch demand");
	fwrite(&(output), 1, sizeof(output), f);
}	
