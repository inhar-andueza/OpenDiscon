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
#include "dlfcn.h"

void OpenDiscon_EXPORT DISCON(float *DATA, int FLAG, const char *INFILE, const char *OUTNAME, char *MESSAGE) {
	int err;
	static ikClwindconWTCon con;
	double output = -12.0;
	static FILE *f = NULL;
	
	float FC_DATA[2]; /* FC_DATA[0] = derating ratio // FC_DATA[1] = Forced yaw angle rad/s */
	static void * fcLibHandle;
	void (*loadFarmData)(float *,int *,float *);
	char *error;
	static int iTurb;
		
	if (NINT(DATA[0]) == 0) {
		ikClwindconWTConParams param;
		ikClwindconWTCon_initParams(&param);
		setParams(&param);
		ikClwindconWTCon_init(&con, &param);
		f = fopen("log.bin", "wb");		
		
		fcLibHandle = dlopen("./libFarmControl.so", RTLD_LAZY);
		if (!fcLibHandle) {
			fprintf(stderr, "%s\n", dlerror());
		}

		dlerror();    /* Clear any existing error */
	}

       /* Writing: cosine = (double (*)(double)) dlsym(handle, "cos"); would seem more natural, but the C99 standard leaves
       casting from "void *" to a function pointer undefined. The assignment used below is the POSIX.1-2003 (Technical
       Corrigendum 1) workaround; see the Rationale for the POSIX specification of dlsym(). */

	*(void **) (&loadFarmData) = dlsym(fcLibHandle, "loadFarmData");
	//loadFarmData = dlsym(fcLibHandle, "loadFarmData");

	if ((error = dlerror()) != NULL)  {
		fprintf(stderr, "%s\n", error);
	}

	(void) loadFarmData(DATA,&iTurb,FC_DATA);

	//dlclose(fcLibHandle);	

//TODO lower maximum torque according to maximum power with derating (it may be time to bring the power manager back)
	con.in.deratingRatio = FC_DATA[0];
	con.in.externalMaximumTorque = 230.0; /* kNm */
	con.in.externalMinimumTorque = 0.0; /* kNm */
	con.in.externalMaximumPitch = 90.0; /* deg */
	con.in.externalMinimumPitch = 0.0; /* deg */
	con.in.generatorSpeed = (double) DATA[19]; /* rad/s */
	con.in.maximumSpeed = 480.0/30*3.1416; /* rpm to rad/s */
	
	ikClwindconWTCon_step(&con);
	
	DATA[46] = (float) (con.out.torqueDemand*1.0e3); /* kNm to Nm */
	DATA[41] = (float) (con.out.pitchDemandBlade1/180.0*3.1416); /* deg to rad */
	DATA[42] = (float) (con.out.pitchDemandBlade2/180.0*3.1416); /* deg to rad */
	DATA[43] = (float) (con.out.pitchDemandBlade3/180.0*3.1416); /* deg to rad */
	DATA[44] = (float) (con.out.pitchDemandBlade1/180.0*3.1416); /* deg to rad (collective pitch angle) */
	
	DATA[47] = FC_DATA[1]; /* Forced yaw angle rad/s */


	err = ikClwindconWTCon_getOutput(&con, &output, "maximum torque");
	fwrite(&(output), 1, sizeof(output), f);
}	
