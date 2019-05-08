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

//#include "FCDiscon.h"
#include <stdio.h>
//#include <math.h>

void loadFarmData(float *DATA,int *iTurb, float *FC_DATA){

	static int n = 0;
	static int n_turb = 0;	
	FILE *fid = NULL;
	static int yaw = 0;

	static double drT1 = 0.9;	
	static double drT2 = 0.9;
	static double drT3 = 0.9;
		
	static double yawT1 = 15.1 * 3.1416 / 180.0; /*10.565*/
	static double yawT2 = 15.1 * 3.1416 / 180.0;
	static double yawT3 = 15.1 * 3.1416 / 180.0;

	/*static double drYawT1 = pow( drT1 , 1/3 );/*15.00 * 3.1416 / 180.0;*/
	/*static double drYawT2 = pow( drT2 , 1/3 );
	static double drYawT3 = pow( drT3 , 1/3 );*/

	/*yawT1 = acos(pow( drT1 , 1/3 ));// - DATA[36];
	yawT2 = acos(pow( drT2 , 1/3 ));// - DATA[36];
	yawT3 = acos(pow( drT3 , 1/3 ));// - DATA[36];*/

	if(n < 10 && DATA[1] == 0.0){
		n_turb++;
		*iTurb = n_turb;
	}

	//FC_DATA[0] = 0.0;

	/*if(DATA[1] < 999.9 || DATA[1] > 999.9){
		if(*iTurb == 1){
			FC_DATA[0] = 1 - drT1;
		}
		if(*iTurb == 2){
			FC_DATA[0] = 1 - drT2;
		}
		if(*iTurb == 3){
			FC_DATA[0] = 1 - drT3;
		}
	}/**/
	if((DATA[1] < 500.0 || DATA[1] > 650.0) && *iTurb == 1){
		if(DATA[1] > 650.0 && DATA[1] < 800.0){
			FC_DATA[0] = (1.0 - drT1)/2;
		} else {/**/
			FC_DATA[0] = (1.0 - drT1);
		}
	} else if((DATA[1] < 500.0 || DATA[1] > 800.0) && *iTurb == 2){
		if(DATA[1] > 800.0 && DATA[1] < 950.0){
			FC_DATA[0] = (1.0 - drT2)/2;
		} else {/**/
			FC_DATA[0] = (1.0 - drT2);
		}
	} else if((DATA[1] < 500.0 || DATA[1] > 950) && *iTurb == 3){
		if(DATA[1] > 950.0 && DATA[1] < 1100.0){
			FC_DATA[0] = (1.0 - drT3)/2;
		} else {/**/
			FC_DATA[0] = (1.0 - drT3);
		}
	} else {
		FC_DATA[0] = 0.0;
	} /**/
	
	/*if((DATA[1] < 999.9 || DATA[1] > 999.9) && *iTurb == 1){
		if(-DATA[23] < (yawT1 - 0.05*3.1416/180.0)){
			yaw = 1;
		} else if(-DATA[23] > (yawT1 + 0.05*3.1416/180.0)){
			yaw = 2;
		} else {
			yaw = 0;
		}
	} else if((DATA[1] < 999.9 || DATA[1] > 999.9) && *iTurb == 2){
		if(-DATA[23] < (yawT2 - 0.05*3.1416/180.0)){
			yaw = 1;
		} else if(-DATA[23] > (yawT2 + 0.05*3.1416/180.0)){
			yaw = 2;
		} else {
			yaw = 0;
		}
	} else if((DATA[1] < 999.9 || DATA[1] > 999.9) && *iTurb == 3){
		if(-DATA[23] < (yawT3 - 0.05*3.1416/180.0)){
			yaw = 1;
		} else if(-DATA[23] > (yawT3 + 0.05*3.1416/180.0)){
			yaw = 2;
		} else {
			yaw = 0;
		}
	} else {
		if(-DATA[23] > 0.0){
			yaw = 2;
		} else {
			yaw = 0;
		}
	}/**/

	switch (yaw){
		case 0:
			FC_DATA[1] = 0.0;
			break;
		case 1:
			FC_DATA[1] = 0.5 * 3.1416 / 180.0;
			break;
		case 2:
			FC_DATA[1] = -0.5 * 3.1416 / 180.0;
			break;
	}/**/

	n++;
	fid = fopen("logfile.txt", "a");
	fprintf(fid,"%d, %f, %d, %f, %f, %f \n",n,DATA[1],*iTurb,DATA[36]*180.0/3.1416,FC_DATA[0],FC_DATA[1]);
	fclose(fid);
	
}
