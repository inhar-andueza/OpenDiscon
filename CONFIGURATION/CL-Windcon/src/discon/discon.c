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

#include "ikClwindconInputMod.h"
#include "ikClwindconWTConfig.h"
#include "OpenDiscon_EXPORT.h"
#include <stdio.h>
#include <stdlib.h>

void OpenDiscon_EXPORT DISCON(float *DATA, int FLAG, const char *INFILE, const char *OUTNAME, char *MESSAGE) {
	/* ----------------------------------- */	
	static float TStart;
	static int c;
	static float param1;
	static float param2;
	static int flag;
	/* ----------------------------------- */
	static int po;
	static int tpo;
	static int pof;
	static float MinP;
	/* ----------------------------------- */
	int err;
	static ikClwindconWTCon con;
	double output = -12.0;
	static FILE *f = NULL;
	const double deratingRatio = 0.0; /* later to be got via the supercontroller interface */

	if (NINT(DATA[0]) == 0) {
		const double T = (double) DATA[2];
		FILE *p;
		char *inputFile = (char *) calloc(256,sizeof(char));
		ikClwindconWTConParams param;
		ikClwindconWTCon_initParams(&param);
		setParams(&param, T);
		ikClwindconWTCon_init(&con, &param);
		f = fopen("log.bin", "wb");
		/* ----------------------------------- */
		p = fopen("Controller.txt","r");
		fscanf(p,"%f",&TStart);
		fscanf(p,"%*[^\n]\n"); 
		fscanf(p,"%i",&flag);
		fscanf(p,"%*[^\n]\n"); 
		fscanf(p,"%s ",inputFile);
		fscanf(p,"%*[^\n]\n");
		fscanf(p,"%f",&MinP);
		fclose(p);
		
		if(flag > 0){
			p = fopen(inputFile,"r");
			if(p != 0){
				fscanf(p,"%i\n%f\n%f\n",&c,&param1,&param2);
				fclose(p);
			} else {
				printf("Aborting. Wrong '.par' file name");
				exit(0);
			}
		} 
		/*p = fopen("output2.txt","w");
		fprintf(p,"%f %i %s %i %f %f",TStart,flag,inputFile,c,param1,param2);
		fclose(p);/**/
		
		free(inputFile);
		/* ----------------------------------- */
	}
	
	con.in.deratingRatio = deratingRatio;
	con.in.externalMaximumTorque = 11.5868; /* kNm */
	con.in.externalMinimumTorque = 0.0; /* kNm */
	con.in.externalMaximumPitch = 90.0; /* deg */
	/*con.in.externalMinimumPitch = 0.0; /* deg */
	con.in.generatorSpeed = (double) DATA[19]; /* rad/s */
	con.in.rotorSpeed = (double) DATA[20]; /* rad/s */
	con.in.maximumSpeed = 17.1*105.4/30.0*3.1416; /* rpm to rad/s */
	con.in.azimuth = 180.0/3.1416 * (double) DATA[59]; /* rad to deg */
	con.in.maximumIndividualPitch = 0.0; /* deg */
	con.in.yawErrorReference = 0.0; /* deg */
	con.in.yawError = 180.0/3.1416 * (double) DATA[23]; /* rad to deg */
	con.in.bladeRootMoments[0].c[0] = 0.0;/*1.0e-3 * (double) DATA[68]; /* Nm to kNm */
	con.in.bladeRootMoments[0].c[1] = 0.0;/*1.0e-3 * (double) DATA[29]; /* Nm to kNm */
	con.in.bladeRootMoments[0].c[2] = 0.0; /* kNm */
	con.in.bladeRootMoments[1].c[0] = 0.0;/*1.0e-3 * (double) DATA[69]; /* Nm to kNm */
	con.in.bladeRootMoments[1].c[1] = 0.0;/*1.0e-3 * (double) DATA[30]; /* Nm to kNm */
	con.in.bladeRootMoments[1].c[2] = 0.0; /* kNm */
	con.in.bladeRootMoments[2].c[0] = 0.0;/*1.0e-3 * (double) DATA[70]; /* Nm to kNm */
	con.in.bladeRootMoments[2].c[1] = 0.0;/*1.0e-3 * (double) DATA[31]; /* Nm to kNm */
	con.in.bladeRootMoments[2].c[2] = 0.0; /* kNm */

	/*if (NINT(DATA[0]) == 0) {
		con.in.externalMinimumPitch = (double)DATA[3]*180.0/3.1416;
		ikClwindconInputMod(&(con.in));
		ikClwindconWTCon_step(&con);
	}*/

	DATA[46] = (float) (con.out.torqueDemand*1.0e3); /* kNm to Nm */
	DATA[41] = (float) (con.out.pitchDemandBlade1/180.0*3.1416); /* deg to rad */
	DATA[42] = (float) (con.out.pitchDemandBlade2/180.0*3.1416); /* deg to rad			+0.3		*/
	DATA[43] = (float) (con.out.pitchDemandBlade3/180.0*3.1416); /* deg to rad			-0.3		*/
	
	if(c == 12 && DATA[1] <= (TStart + 15.0)){
		DATA[41] = (float) (90.0 * 3.1416 / 180.0);
 		DATA[42] = (float) (90.0 * 3.1416 / 180.0);
 		DATA[43] = (float) (90.0 * 3.1416 / 180.0);
		con.in.externalMaximumTorque = 0.0; 
	}	
	
	if(DATA[1] > (TStart + 15.0) ){	/*&& flag > 0*/  
		static double minimumSpeed = 7.5 * 3.1416 / 30.0; /* rpm to rad/s */
		static int t = 0;
		static int ShutDown = 0;
		static int oSa = 0;
			
		double overSpeedN4;
		double overSpeedNA;
		double pitchRate;
		double finalPitchAngle = 90.0 * 3.1416 / 180;
		double finePitchAngle = 0.0;
		double yawRate;
		double FinalYaw;
		
		double overSpeedN;		
		double refTime = TStart + 15.0;

		double timeStep = (double) DATA[2];
		double pitchBlade1 = (double) DATA[3];
		double pitchBlade2 = (double) DATA[32];	
		double pitchBlade3 = (double) DATA[33];	

		static int trq = 0;
			
		double delay = DATA[1] - refTime;
		
		FILE *out;

		if (flag > 0){	
			static double friedPitch;
			static double pB1, pB2, pB3;
			static int p = 0;
			if (p < 1){
				p++;
				pB1 = pitchBlade1;
				pB2 = pitchBlade2;
				pB3 = pitchBlade3;
				friedPitch = pB1;
			}

			switch (c) {	
				case 0 :		/* OverSpeed_n4*/		
						overSpeedN4 = param1 * 3.1416 / 30.0;
						pitchRate = param2 * 3.1416 / 180.0; 
						
						con.in.generatorSpeed = (double) (DATA[19] * 0.9);
						
						overSpeedN = overSpeedN4;
						if(DATA[20] > (float) overSpeedN && t < 1){
							t++;
							//FILE *out;
										
							out = fopen("Alarms.txt","wt");
							fprintf(out,"%s %f %s\n","T = ",DATA[1], "sec");
							fprintf(out,"%s %f\n %s","Warning: Overspeed limit reached: (rpm)",(overSpeedN * 30.0 / 3.1416),"\nShutting down wind turbine.\n");
							fclose(out);
						}		
						break;
						
				case 1 :		/* OverSpeed_nA*/		
						overSpeedNA = param1 * 3.1416 / 30.0;
						pitchRate = param2 * 3.1416 / 180.0; 
				
						con.in.generatorSpeed = (double) (DATA[19] * 0.8);
						
						overSpeedN = overSpeedNA;
						if(DATA[20] > (float) overSpeedN && t < 1){
							t++;
							//FILE *out;
										
							out = fopen("Alarms.txt","wt");
							fprintf(out,"%s %f %s\n","T = ",DATA[1], "sec");
							fprintf(out,"%s %f\n %s","Warning: Overspeed limit reached: (rpm)",(overSpeedN * 30.0 / 3.1416),"\nShutting down wind turbine.\n");
							fclose(out);
						}
						break;
				
				case 2 :		/* Blade1ToFeather */			
						pitchRate = param1 * 3.1416 / 180;
						if(t < 1){
							if( pitchBlade1 < (pitchBlade2 + 5*3.1416/180.0) ){
								DATA[41] = (float) (pitchRate*timeStep + pB1);
								pB1 = (double) DATA[41];
								/*fwrite(&pitchRate, 1, sizeof(pitchRate), f);
								fwrite(&timeStep, 1, sizeof(timeStep), f);
								fwrite(&pitchBlade1, 1, sizeof(pitchBlade1), f);
								fwrite(&pB1, 1, sizeof(pB1), f);
								output = (double) DATA[41];
								fwrite(&(output), 1, sizeof(output), f);
								output = (double) DATA[1];
								fwrite(&(output), 1, sizeof(output), f);*/
							} else {
									t++;
									//FILE *out;
									
									out = fopen("Alarms.txt","wt");
									fprintf(out,"%s %f %s\n","T = ",DATA[1], "sec");
									fprintf(out,"%s\n","Warning: Blade 1 to feather.\n");
									fclose(out);
							}
						}			
						break;
				
				case 3 :		/* Blade1ToFine */	
						pitchRate = param1 * 3.1416 / 180;
						if(t < 1){
							if( pitchBlade1 > pitchBlade2 - 5*3.1416/180.0 ){
								DATA[41] = (float) ((-pitchRate)*timeStep + pB1);
								pB1 = (double) DATA[41];
							} else {
									t++;
									//FILE *out;
									
									out = fopen("Alarms.txt","wt");
									fprintf(out,"%s %f %s\n","T = ",DATA[1], "sec");
									fprintf(out,"%s\n","Warning: Blade 1 to fine.\n");
									fclose(out);
							}
						} 
						break;
				
				case 4 :		/* AllBladesToFine */	
						pitchRate = param1 * 3.1416 / 180.0;	
						if(t < 1){
							DATA[41] = (float) ((-pitchRate)*timeStep + pB1);
							DATA[42] = (float) ((-pitchRate)*timeStep + pB2);
							DATA[43] = (float) ((-pitchRate)*timeStep + pB3);
							pB1 = (double) DATA[41];
							pB2 = (double) DATA[42];
							pB3 = (double) DATA[43];
						}
						if(pitchBlade1 <= 0 && pitchBlade2 <= 0 && pitchBlade3 <= 0){
								t = 2;
								
								//FILE *out;
								
								out = fopen("Alarms.txt","wt");
								fprintf(out,"%s %f %s\n","T = ",DATA[1], "sec");
								fprintf(out,"%s\n","Warning: All blades to fine.\n ");
								fclose(out);
						}
						if(t > 1){
								DATA[41] = 0.0;
								DATA[42] = 0.0;
								DATA[43] = 0.0;
						}
						
						break;
				
				case 5 :		/* YawRunAwayPlus */
						yawRate = param1 * 3.1416 / 180.0;
						FinalYaw = param2 * 3.1416 / 180.0;		
						
						if(DATA[36] < (float) FinalYaw){
							DATA[47] = (float) yawRate;
						} else {
							DATA[47] = 0.0;
						}
						if(DATA[23] > 0.5*3.1416/180.0 && t < 1){
							t++;
							//FILE *out;
							
							out = fopen("Alarms.txt","wt");
							fprintf(out,"%s %f %s\n","T = ",DATA[1], "sec");
							fprintf(out,"%s\n","Warning: Yaw clockwise run away.\n");
							fclose(out);
						}
						break;
						
				case 6 :		/* YawRunAwayMinus */
						yawRate = param1 * 3.1416 / 180.0;
						FinalYaw = param2 * 3.1416 / 180.0;	
						
						if(DATA[36] > (float) FinalYaw){
							DATA[47] = (float) (-yawRate);
						} else {
							DATA[47] = 0.0;
						}
						if(DATA[23] < -0.5*3.1416/180.0 && t < 1){
							t++;
							//FILE *out;
							
							out = fopen("Alarms.txt","wt");
							fprintf(out,"%s %f %s\n","T = ",DATA[1], "sec");
							fprintf(out,"%s\n","Warning: Yaw counter-clockwise run away.\n");
							fclose(out);
						}
						break;
				
				case 7 :		/* BladeFried */
						DATA[41] = (float) friedPitch;

						if (t < 1){
								t++;

								//FILE *out;
							
								out = fopen("Alarms.txt","wt");
								fprintf(out,"%s %f %s\n","T = ",DATA[1], "sec");
								fprintf(out,"%s\n","Warning: Unable to change blade 1 pitch angle.\n");
								fclose(out);
						} 
												
						break;
				
				case 8 :		/* GridLoss_0 */		
						pitchRate = param1 * 3.1416 / 180;	
				
						con.in.externalMaximumTorque = 0.0;

						if(t < 1){
							t++;
							//FILE *out;
							
							out = fopen("Alarms.txt","wt");
							fprintf(out,"%s %f %s\n","T = ",DATA[1], "sec");
							fprintf(out,"%s\n","Warning: Grid connection lost.\n Shutting down wind turbine.\n");
							fclose(out);
						} else {
								ShutDown = 1;
						}
						break;
				
				case 9 :		/* GridLoss_2p25 */			
						pitchRate = param1 * 3.1416 / 180;
						
						if( delay > 2.25 ){
							con.in.externalMaximumTorque = 0.0; 
							if(t < 1){
								t++;
								//FILE *out;
								
								out = fopen("Alarms.txt","wt");
								fprintf(out,"%s %f %s\n","T = ",DATA[1], "sec");
								fprintf(out,"%s\n","Warning: Grid connection lost.\n Shutting down wind turbine.\n");
								fclose(out);
							} else {
								ShutDown = 1;
							}
						}
						break;
				
				case 10 :		/* GridLoss_4 */		
						pitchRate = param1 * 3.1416 / 180;					
						if( delay > 4.0 ){
							con.in.externalMaximumTorque = 0.0;
							if(t < 1){
								t++;
								//FILE *out;
								
								out = fopen("Alarms.txt","wt");
								fprintf(out,"%s %f %s\n","T = ",DATA[1], "sec");
								fprintf(out,"%s\n","Warning: Grid connection lost.\n Shutting down wind turbine.\n");
								fclose(out);
							} else {
								ShutDown = 1;
							}
						}
						break;
				
				case 11 :		/* GridLoss_5p25 */		
						pitchRate = param1 * 3.1416 / 180;																	
						if( delay > 5.25 ){
							con.in.externalMaximumTorque = 0.0; 
							if(t < 1){
								t++;
								//FILE *out;
								
								out = fopen("Alarms.txt","wt");
								fprintf(out,"%s %f %s\n","T = ",DATA[1], "sec");
								fprintf(out,"%s\n","Warning: Grid connection lost.\n Shutting down wind turbine.\n");
								fclose(out);
							} else {
								ShutDown = 1;
							}
						}
						break;
				
				case 12 :		/* Startup */	
						pitchRate = param1 * 3.1416 / 180;
						
						if(con.in.generatorSpeed < con.in.maximumSpeed * 0.9){
							/*if( pitchBlade1 > finePitchAngle ){
								DATA[41] = (float) ((-pitchRate)*timeStep + pB1);
								pB1 = (double) DATA[41];
							} else {
								DATA[41] = (float) finePitchAngle;
							}		
							if( pitchBlade2 > finePitchAngle ){
								DATA[42] = (float) ((-pitchRate)*timeStep + pB2);
								pB2 = (double) DATA[42];
							} else {
								DATA[42] = (float) finePitchAngle;
							}		
							if( pitchBlade3 > finePitchAngle ){
								DATA[43] = (float) ((-pitchRate)*timeStep + pB3);
								pB3 = (double) DATA[43];
							} else {
								DATA[43] = (float) finePitchAngle;
							}		
						}*/
							if(t < 1){
								DATA[41] = (float) ((-pitchRate)*timeStep + pB1);
								DATA[42] = (float) ((-pitchRate)*timeStep + pB2);
								DATA[43] = (float) ((-pitchRate)*timeStep + pB3);
								pB1 = (double) DATA[41];
								pB2 = (double) DATA[42];
								pB3 = (double) DATA[43];
							}
							if(pitchBlade1 <= 0 && pitchBlade2 <= 0 && pitchBlade3 <= 0){
									t++;
								
									//FILE *out;
								
									out = fopen("Alarms.txt","wt");
									fprintf(out,"%s %f %s\n","T = ",DATA[1], "sec");
									fprintf(out,"%s\n","Warning: Minimum speed reached.\n Connecting turbine to grid.\n");
									fclose(out);
							}
							if(t > 0){
									DATA[41] = 0.0;
									DATA[42] = 0.0;
									DATA[43] = 0.0;
							}
						}

						if(con.in.generatorSpeed < minimumSpeed * 1.05 && trq < 1){
							con.in.externalMaximumTorque = 0.0;
						} else {	
							trq++;
						}

						break;
				
				case 13 :		/* Stop_Soft_0 */	
						pitchRate = param1 * 3.1416 / 180;
				
						if(t < 1){
							t++;
							//FILE *out;
								
							out = fopen("Alarms.txt","wt");
							fprintf(out,"%s %f %s\n","T = ",DATA[1], "sec");
							fprintf(out,"%s\n","Warning: Shutting down wind turbine.\n");
							fclose(out);
						} else {
								ShutDown = 1;
						}
						break;
				
				case 14 :		/* Stop_Soft_2p25 */	
						pitchRate = param1 * 3.1416 / 180;
						
						if( delay > 2.25 ){
							if(t < 1){
								t++;
								//FILE *out;
									
								out = fopen("Alarms.txt","wt");
								fprintf(out,"%s %f %s\n","T = ",DATA[1], "sec");
								fprintf(out,"%s\n","Warning: Shutting down wind turbine.\n");
								fclose(out);
							} else {
								ShutDown = 1;
							}
						}
						break;
				
				case 15 :		/* Stop_Soft_4 */
						pitchRate = param1 * 3.1416 / 180;
						
						if( delay > 4.0 ){
							if(t < 1){
								t++;
								//FILE *out;
									
								out = fopen("Alarms.txt","wt");
								fprintf(out,"%s %f %s\n","T = ",DATA[1], "sec");
								fprintf(out,"%s\n","Warning: Shutting down wind turbine.\n");
								fclose(out);
							} else {
								ShutDown = 1;
							}
						}
						
						break;
				
				case 16 :		/* Stop_Soft_5p25 */
						pitchRate = param1 * 3.1416 / 180;
						
						if( delay > 5.25 ){
							if(t < 1){
								t++;
								//FILE *out;
									
								out = fopen("Alarms.txt","wt");
								fprintf(out,"%s %f %s\n","T = ",DATA[1], "sec");
								fprintf(out,"%s\n","Warning: Shutting down wind turbine.\n");
								fclose(out);
							} else {
								ShutDown = 1;
							}
						} 
						break;
				
				case 17 :		/* EmergencyStop */		
						pitchRate = param1 * 3.1416 / 180;
						
						if(t < 1){
							t++;
							//FILE *out;
								
							out = fopen("Alarms.txt","wt");
							fprintf(out,"%s %f %s\n","T = ",DATA[1], "sec");
							fprintf(out,"%s\n","Warning: Emergency brake deployed.\nShutting down wind turbine.\n");
							fclose(out);
						}
				
						DATA[35] = 1;
												
						ShutDown = 1;
						
						break;
						
				case 18 :		/* No Events */
						pitchRate = param1 * 3.1416 / 180;
						break;
			}
			
			if(c != 0 && c != 1){
				overSpeedN = 20.5 * 3.1416 / 30.0;
			}
			if(c == 5 || c == 6 || c == 7){
				pitchRate = 5.0 * 3.1416 / 180.0;
			}
			
			if(DATA[20] > (float) overSpeedN ){
				ShutDown = 1;
				if(oSa < 1 && c != 0 && c != 1){
					oSa++;
					//FILE *out;
										
					out = fopen("Alarms.txt","at");
					fprintf(out,"%s %f %s\n","T = ",DATA[1], "sec");
					fprintf(out,"%s %f\n %s","Warning: Overspeed limit reached: (rpm)",(overSpeedN * 30.0 / 3.1416),"\nShutting down wind turbine.\n");
					fclose(out);
				}
			}
			if(ShutDown){
				static int s = 0;
				static int p1 = 0;
				static int p2 = 0;
				static int p3 = 0;
				if( pitchBlade1 < finalPitchAngle ){
					if(!p1){
						pB1 = pitchBlade1;
						//pB2 = pitchBlade2;
						//pB3 = pitchBlade3;
						p1++;
					}
					DATA[41] = (float) (pitchRate*timeStep + pB1);
					/*DATA[42] = (float) (pitchRate*timeStep + pB2);
					DATA[43] = (float) (pitchRate*timeStep + pB3);*/
					pB1 = DATA[41];
					/*pB2 = DATA[42];
					pB3 = DATA[43];*/
				} else {
					s++;
				}
				if( pitchBlade2 < finalPitchAngle ){
					if(!p2){
						pB2 = pitchBlade2;
						p2++;
					}
					DATA[42] = (float) (pitchRate*timeStep + pB2);
					pB2 = DATA[42];
				} else {
					s++;
				}
				if( pitchBlade3 < finalPitchAngle ){
					if(!p3){
						pB3 = pitchBlade3;
						p3++;
					}
					DATA[43] = (float) (pitchRate*timeStep + pB3);
					pB3 = DATA[43];
				} else {
					s++;
								
					//FILE *out;
								
					out = fopen("Alarms.txt","at");
					fprintf(out,"%s %f %s\n","T = ",DATA[1], "sec");
					fprintf(out,"%s\n","Warning: Shutting down turbine.\n ");
					fclose(out);
				}
				if(s > 2){
					DATA[41] = (float) finalPitchAngle;
					DATA[42] = (float) finalPitchAngle;
					DATA[43] = (float) finalPitchAngle;
				}
				
				if(DATA[20] < (float) minimumSpeed){
					con.in.externalMaximumTorque = 0.0;
				} 
			} 
		
			/* High wind deviation */ 
		
			if(DATA[23] > 30.0*3.1416/180.0 || DATA[23] < -30.0*3.1416/180.0){
				if(t < 1){
					t++;
					//FILE *out;
								
					out = fopen("Alarms.txt","at");
					fprintf(out,"%s %f %s\n","T = ",DATA[1], "sec");
					fprintf(out,"%s\n","Warning: High wind direction deviation.\nShutting down wind turbine.\n");
					fclose(out);
				}
				ShutDown = 1;
			}
		}
	}/**/ 
	
	if( (double) DATA[22] > 0.85*11.5868*1000.0 && po < 1)	po = 1;
	else if( (double) DATA[22] > 0.9*11.5868*1000.0 && po < 2)	po = 2;
	else if( (double) DATA[22] > 0.95*11.5868*1000.0 && po < 3)	po = 3;
	else if( (double) DATA[22] > 1.0*11.5868*1000.0 && po < 4)	po = 4;
	else if((double) DATA[22] < 0.85*11.5868*1000.0) pof = 1;
	else pof = 0;
		
	if (pof) tpo++;
	else tpo = 0.0;
	if (tpo > 25.0/((double)DATA[2])) {po = 0; tpo = 0.0; pof = 0;}

	switch (po){
		case 1:
			if (con.in.externalMinimumPitch < MinP*(1.0/4.0)){
				con.in.externalMinimumPitch = 10.0 * DATA[2] + con.in.externalMinimumPitch;
			} else con.in.externalMinimumPitch = MinP*(1.0/4.0);
			break;
		case 2:
			if (con.in.externalMinimumPitch < MinP*(1.0/2.0)){
				con.in.externalMinimumPitch = 10.0 * DATA[2] + con.in.externalMinimumPitch;
			} else con.in.externalMinimumPitch = MinP*(1.0/2.0);
			break;
		case 3:
			if (con.in.externalMinimumPitch < MinP*(3.0/4.0)){
				con.in.externalMinimumPitch = 10.0 * DATA[2] + con.in.externalMinimumPitch;
			} else con.in.externalMinimumPitch = MinP*(3.0/4.0);
			break;
		case 4:
			if (con.in.externalMinimumPitch < MinP){
				con.in.externalMinimumPitch = 10.0 * DATA[2] + con.in.externalMinimumPitch;
			} else con.in.externalMinimumPitch = MinP;
			break;
		default:
			con.in.externalMinimumPitch = 0.0;
	}

	ikClwindconInputMod(&(con.in));
	ikClwindconWTCon_step(&con);
	
	/*err = ikClwindconWTCon_getOutput(&con, &output, "collective pitch demand");
	/*DATA[44] = (float) (output/180.0*3.1416); /* deg to rad (collective pitch angle) */

	/*
	err = ikClwindconWTCon_getOutput(&con, &output, "maximum pitch");
	fwrite(&(output), 1, sizeof(output), f);
	err = ikClwindconWTCon_getOutput(&con, &output, "torque control>setpoint");
	fwrite(&(output), 1, sizeof(output), f);
	err = ikClwindconWTCon_getOutput(&con, &output, "individual pitch control>My");
	fwrite(&(output), 1, sizeof(output), f);
	err = ikClwindconWTCon_getOutput(&con, &output, "individual pitch control>Mz");
	fwrite(&(output), 1, sizeof(output), f);
	err = ikClwindconWTCon_getOutput(&con, &output, "individual pitch control>pitch increment 1");
	fwrite(&(output), 1, sizeof(output), f);
	err = ikClwindconWTCon_getOutput(&con, &output, "individual pitch control>pitch increment 2");
	fwrite(&(output), 1, sizeof(output), f);
	err = ikClwindconWTCon_getOutput(&con, &output, "individual pitch control>pitch increment 3");
	fwrite(&(output), 1, sizeof(output), f);
	err = ikClwindconWTCon_getOutput(&con, &output, "generator speed equivalent");
	fwrite(&(output), 1, sizeof(output), f);
	err = ikClwindconWTCon_getOutput(&con, &output, "speed sensor manager>signal 1");
	fwrite(&(output), 1, sizeof(output), f);
	err = ikClwindconWTCon_getOutput(&con, &output, "speed sensor manager>signal 2");
	fwrite(&(output), 1, sizeof(output), f);
	err = ikClwindconWTCon_getOutput(&con, &output, "speed sensor manager>signal 3");
	fwrite(&(output), 1, sizeof(output), f);
	*/
	
}	
