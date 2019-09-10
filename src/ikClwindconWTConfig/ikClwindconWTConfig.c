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

/**
 * @file ikClwindconWTConfig.c
 *
 * @brief CL-Windcon wind turbine controller configuration implementation
 */

#include "ikClwindconWTConfig.h"

void setParams(ikClwindconWTConParams *param) {
	double T = 0.005;

	ikLimitPitchRate(&(param->collectivePitchControl), T);
	ikTuneDrivetrainDamper(&(param->drivetrainDamper), T);
	ikTuneSpeedRange(&(param->torqueControl));
	ikTunePowerSettings(&(param->powerManager));
	ikTuneDeratingTorqueStrategy(&(param->powerManager));
	ikTuneDeratingPitchStrategy(&(param->powerManager));
	ikTunePitchPIGainSchedule(&(param->collectivePitchControl));
	ikTunePitchLowpassFilter(&(param->collectivePitchControl), T);
	ikTunePitchNotches(&(param->collectivePitchControl), T);
	ikTunePitchPI(&(param->collectivePitchControl), T);
	ikTuneTorqueLowpassFilter(&(param->torqueControl), T);
	ikTuneTorqueNotches(&(param->torqueControl), T);
	ikTuneTorquePI(&(param->torqueControl), T);

}

void ikLimitPitchRate(ikConLoopParams *params, double T) {
	
	static double maxPitchRateT;
	static double minPitchRateT;
	/*
	####################################################################
	*/
	const double maxPitchRate = 10.0;
	const double minPitchRate = -10.0;
	/*
	####################################################################
	*/
	
	maxPitchRateT = maxPitchRate * T;
	minPitchRateT = minPitchRate * T;
	
	params->linearController.maxPostGainValue = &maxPitchRateT;
	params->linearController.minPostGainValue = &minPitchRateT;
	
}

void ikTuneDrivetrainDamper(ikConLoopParams *params, double T) {

	/*! [CL-Windcon drivetrain damper] */
    /*
	####################################################################
                     Drivetrain damper

    Transfer function:

    D(s) = G*s*w^2/(s^2 + 2*d*w*s + w^2)

    The sampling time is given by function parameter T.

    Set parameters here:
	*/
    double G = 0.0955; /* [kNm s^2/rad] 10 Nm s/rpm */
    double d = 0.5; /* [-] */
    double w = 23.0; /* [rad/s] */
    /*
    ####################################################################
	*/
	/*! [CL-Windcon drivetrain damper] */


    /*
	tune the drivetrain damper to this tf:
                       z^2 - 1
    D(z) = G*T/2*w^2 -------------------------------------------------------------------------------
                     (1 + T*d*w + T^2*w^2/4)*z^2 -2*(1 - T^2*w^2/4)*z + (1 - T*d*w + T^2*w^2/4)
    rad/s --> kNm
	*/
    params->linearController.errorTfs.tfParams[0].enable = 1;
    params->linearController.errorTfs.tfParams[0].b[0] = 1.0;
    params->linearController.errorTfs.tfParams[0].b[1] = 0.0;
    params->linearController.errorTfs.tfParams[0].b[2] = -1.0;
    params->linearController.errorTfs.tfParams[0].a[0] = 1.0 + T*d*w + T*T*w*w/4.0;
    params->linearController.errorTfs.tfParams[0].a[1] = -2.0*(1.0 - T*T*w*w/4.0);
    params->linearController.errorTfs.tfParams[0].a[2] = (1.0 - T*d*w + T*T*w*w/4.0);
    params->linearController.errorTfs.tfParams[1].enable = 1;
    params->linearController.errorTfs.tfParams[1].b[0] = -G*T/2.0*w*w;

}

void ikTuneSpeedRange(ikConLoopParams *params) {

	/*
	####################################################################
					 Variable generator speed range
					 
	Set parameters here:
	*/
	const double Wmin = 104.7198; /* [rad/s] */
	const double Wmax = 188.4956; /* [rad/s] */
	/*
	####################################################################
	*/

    params->setpointGenerator.nzones = 1;
    params->setpointGenerator.setpoints[0][0] = Wmin;
    params->setpointGenerator.setpoints[1][0] = Wmax;

}
	
void ikTunePowerSettings(ikPowmanParams *params) {
	
	/*
	####################################################################
					 Power settings
					 
	Set parameters here:
	*/
	const double Pn = 1.5e3; /* kW */
	const double eff = 0.95; /* - */
	/*
	####################################################################
	*/

	params->ratedPower = Pn;
	params->efficiency = eff;
}

void ikTuneDeratingTorqueStrategy(ikPowmanParams *params) {
/*
This is an original implementation of derating strategy 3a as described by ECN in deliverable D2.1 of H2020 project CL-Windcon.
*/

	int i;
	
	/*! [Optimum torque] */
    /*
	####################################################################
					 Below rated speed-torque curve

	Curve:

	Q = Kopt(dr) * w^2

	The default values for dr and Kopt have been kindly provided by ECN, who have calculated them to suit the DTU 10MW reference wind turbine from FP7 project INNWIND.
	Set parameters here:
	*/
	const int n = 11; /* number of points in the lookup table */
	const double dr[] = {0.00, 0.025, 0.05, 0.075, 0.10, 0.125, 0.15, 0.175, 0.20, 0.225, 0.25}; /* - */
	const double Kopt[] = {0.2522, 0.245895, 0.23959, 0.233285, 0.22698, 0.220675, 0.21437, 0.208065, 0.20176, 0.195455, 0.18915}; /* 0.2357, 0.2298, 0.2239, 0.2180, 0.2121, 0.2062, 0.2003, 0.1945, 0.1886, 0.1827, 0.1768 Nm*s^2/rad^2 */
	/*
	####################################################################
	*/
	/*! [Optimum torque] */

	params->belowRatedTorqueGainTableN = n;
	for (i = 0; i < n; i++) {
		params->belowRatedTorqueGainTableX[i] = dr[i];
		params->belowRatedTorqueGainTableY[i] = Kopt[i]/1.0e3;
	}		
}

void ikTuneDeratingPitchStrategy(ikPowmanParams *params) {
/*
This is an original implementation of derating strategy 3a as described by ECN in deliverable D2.1 of H2020 project CL-Windcon.
*/

	int i;
	
	/*! [Minimum pitch] */
    /*
	####################################################################
					 Minimum pitch

	The default values have been kindly provided by ECN, who have calculated them to suit the DTU 10MW reference wind turbine from FP7 project INNWIND.
	Set parameters here:
	*/
	const int n = 11; /* number of points in the lookup table */
	const double dr[] = {0.00, 0.025, 0.05, 0.075, 0.10, 0.125, 0.15, 0.175, 0.20, 0.225, 0.25}; /* - */
	const double pitch[] = {0.00, 0.014166, 0.025607, 0.036172, 0.045483, 0.053842, 0.061789, 0.069504, 0.076228, 0.083127, 0.08969};/*{0.00, 0.0806427, 0.0920838, 0.102649,  0.111960, 0.120319, 0.128266, 0.135981, 0.142705, 0.149604, 0.156167}; /* rad */
	/*
	####################################################################
	*/
	/*! [Minimum pitch] */

	params->minimumPitchTableN = n;
	for (i = 0; i < n; i++) {
		params->minimumPitchTableX[i] = dr[i];
		params->minimumPitchTableY[i] = pitch[i]/3.1416*180.0;
	}		
}

void ikTunePitchPIGainSchedule(ikConLoopParams *params) {
	int i;
	
	/*! [Gain schedule] */
    /*
	####################################################################
                     Pitch Gain Schedule

	Set parameters here:
	*/
	const int n = 8; /* number of points in the lookup table */
	const double pitch[] = {3.8089,       7.3146,       12.496,       16.181,       19.288,       21.959,       24.317,       26.486}; /* degrees */
	const double gain[] = {1.0,    0.6449,    0.4560,    0.3869,    0.3472,    0.2504,    0.2104,    0.2041}; /* - */
	/*
    ####################################################################
	*/
	/*! [Gain schedule] */

	params->linearController.gainSchedN = n;

	for (i = 0; i < n; i++) {
		params->linearController.gainSchedX[i] = pitch[i];
		params->linearController.gainSchedY[i] = gain[i];
	}	
}

void ikTunePitchLowpassFilter(ikConLoopParams *params, double T) {

	/*! [Pitch lowpass filter] */
    /*
	####################################################################
                     Speed feedback low pass filter

    Transfer function (to be done twice - we want a 4th order filter):
    H(s) = w^2 / (s^2 + 2*d*w*s + w^2)

    The sampling time is given by function parameter T.

    Set parameters here:
	*/
    double w = 10.856; /* [rad/s] */
    double d = 0.8; /* [-] */
    /*
    ####################################################################
	*/
	/*! [Pitch lowpass filter] */

    /*
	tune the pitch control feedback filter to this tf (twice, mind you):
                   (0.5*T*w)^2                                                                     z^2 + 2z + 1
    H(z) =  -----------------------------   ------------------------------------------------------------------------------------------------------------------------
            1 + T*d*w +  (0.5*T*w)^2    z^2 - 2*(1 - (0.5*T*w)^2) / (1 + T*d*w +  (0.5*T*w)^2)z +  (1 - T*d*w +  (0.5*T*w)^2) / (1 + T*d*w +  (0.5*T*w)^2)
    */
	params->linearController.measurementTfs.tfParams[1].enable = 1;
    params->linearController.measurementTfs.tfParams[1].b[0] = 1.0;
    params->linearController.measurementTfs.tfParams[1].b[1] = 2.0;
    params->linearController.measurementTfs.tfParams[1].b[2] = 1.0;
    params->linearController.measurementTfs.tfParams[1].a[0] = 1.0;
    params->linearController.measurementTfs.tfParams[1].a[1] = -2 * (1 - (0.5*T*w)*(0.5*T*w)) / (1 + T*d*w + (0.5*T*w)*(0.5*T*w));
    params->linearController.measurementTfs.tfParams[1].a[2] = (1 - T*d*w + (0.5*T*w)*(0.5*T*w)) / (1 + T*d*w + (0.5*T*w)*(0.5*T*w));

    /*params->linearController.measurementTfs.tfParams[2].enable = 1;
    params->linearController.measurementTfs.tfParams[2].b[0] = 1.0;
    params->linearController.measurementTfs.tfParams[2].b[1] = 2.0;
    params->linearController.measurementTfs.tfParams[2].b[2] = 1.0;
    params->linearController.measurementTfs.tfParams[2].a[0] = 1.0;
    params->linearController.measurementTfs.tfParams[2].a[1] = -2 * (1 - (0.5*T*w)*(0.5*T*w)) / (1 + T*d*w + (0.5*T*w)*(0.5*T*w));
    params->linearController.measurementTfs.tfParams[2].a[2] = (1 - T*d*w + (0.5*T*w)*(0.5*T*w)) / (1 + T*d*w + (0.5*T*w)*(0.5*T*w));*/

    params->linearController.measurementTfs.tfParams[2].enable = 1;
    params->linearController.measurementTfs.tfParams[2].b[0] = ((0.5*T*w)*(0.5*T*w) / (1 + T*d*w + (0.5*T*w)*(0.5*T*w))); /** ((0.5*T*w)*(0.5*T*w) / (1 + T*d*w + (0.5*T*w)*(0.5*T*w)));*/

}

void ikTunePitchNotches(ikConLoopParams *params, double T) {

	/*! [1st fore-aft tower mode filter] */
    /*
	####################################################################
                     1st fore-aft tower mode filter

    Transfer function:
    H(s) = (s^2 + 2*dnum*w*s + w^2) / (s^2 + 2*dden*w*s + w^2)

    The sampling time is given by function parameter T.

    Set parameters here:
	*/
    double w = 2.5235; /* [rad/s] */
    double dnum = 0.05; /* [-] */
    double dden = 0.2; /* [-] */
    /*
    ####################################################################
	*/
	/*! [1st fore-aft tower mode filter] */

    params->linearController.measurementNotches.dT = T;
    params->linearController.measurementNotches.notchParams[0].enable = 1;
    params->linearController.measurementNotches.notchParams[0].freq = w;
    params->linearController.measurementNotches.notchParams[0].dampNum = dnum;
    params->linearController.measurementNotches.notchParams[0].dampDen = dden;

}

void ikTunePitchPI(ikConLoopParams *params, double T) {

	/*! [Pitch PI] */
    /*
	####################################################################
                     Pitch PI

    Transfer function:

    C(s) = (Kp*s + Ki)/s

    The sampling time is given by function parameter T.

    Set parameters here:
	*/
    double Kp = -1.0972; /* [degs/rad] rad/rpm */
    double Ki = -1.0972*0.64; /* [deg/rad]  rad/rpms */
    /*
    ####################################################################
	*/
	/*! [Pitch PI] */


	/*
	tune the speed control to this tf:
           (Kp + Ki*T/2)z - (Kp - Ki*T/2)
    C(z) = ------------------------------
                      z - 1
	rad/s --> deg
	*/
	params->linearController.errorTfs.tfParams[0].enable = 1;
    params->linearController.errorTfs.tfParams[0].b[0] = (Kp + Ki*T/2);
    params->linearController.errorTfs.tfParams[0].b[1] = -(Kp - Ki*T/2);
    params->linearController.errorTfs.tfParams[0].b[2] = 0.0;
    params->linearController.errorTfs.tfParams[0].a[0] = 1.0;
    params->linearController.errorTfs.tfParams[0].a[1] = 0.0;
    params->linearController.errorTfs.tfParams[0].a[2] = 0.0;

	params->linearController.postGainTfs.tfParams[0].enable = 1;
    params->linearController.postGainTfs.tfParams[0].b[0] = 1.0;
    params->linearController.postGainTfs.tfParams[0].b[1] = 0.0;
    params->linearController.postGainTfs.tfParams[0].b[2] = 0.0;
    params->linearController.postGainTfs.tfParams[0].a[0] = 1.0;
    params->linearController.postGainTfs.tfParams[0].a[1] = -1.0;
    params->linearController.postGainTfs.tfParams[0].a[2] = 0.0;

}

void ikTuneTorqueLowpassFilter(ikConLoopParams *params, double T) {

	/*! [Torque lowpass filter] */
    /*
	####################################################################
                    Speed feedback low pass filter

    Transfer function (to be done twice - we want a 4th order filter):
    H(s) = w^2 / (s^2 + 2*d*w*s + w^2)

    The sampling time is given by function parameter T.

    Set parameters here:
	*/
    double w = 2.5032; /* [rad/s] */
    double d = 0.1; /* [-] */
    /*
    ####################################################################
	*/
	/*! [Torque lowpass filter] */

    /*
	tune the torque control feedback filter to this tf (twice, mind you):
                   (0.5*T*w)^2                                                                     z^2 + 2z + 1
    H(z) =  -----------------------------   ------------------------------------------------------------------------------------------------------------------------
            1 + T*d*w +  (0.5*T*w)^2    z^2 - 2*(1 - (0.5*T*w)^2) / (1 + T*d*w +  (0.5*T*w)^2)z +  (1 - T*d*w +  (0.5*T*w)^2) / (1 + T*d*w +  (0.5*T*w)^2)
	*/
    params->linearController.measurementTfs.tfParams[0].enable = 1;
    params->linearController.measurementTfs.tfParams[0].b[0] = 1.0;
    params->linearController.measurementTfs.tfParams[0].b[1] = 2.0;
    params->linearController.measurementTfs.tfParams[0].b[2] = 1.0;
    params->linearController.measurementTfs.tfParams[0].a[0] = 1.0;
    params->linearController.measurementTfs.tfParams[0].a[1] = -2 * (1 - (0.5*T*w)*(0.5*T*w)) / (1 + T*d*w + (0.5*T*w)*(0.5*T*w));
    params->linearController.measurementTfs.tfParams[0].a[2] = (1 - T*d*w + (0.5*T*w)*(0.5*T*w)) / (1 + T*d*w + (0.5*T*w)*(0.5*T*w));

	params->linearController.measurementTfs.tfParams[1].enable = 1;
    params->linearController.measurementTfs.tfParams[1].b[0] = ((0.5*T*w)*(0.5*T*w) / (1 + T*d*w + (0.5*T*w)*(0.5*T*w)));

    /*params->linearController.measurementTfs.tfParams[1].enable = 1;
    params->linearController.measurementTfs.tfParams[1].b[0] = 1.0;
    params->linearController.measurementTfs.tfParams[1].b[1] = 2.0;
    params->linearController.measurementTfs.tfParams[1].b[2] = 1.0;
    params->linearController.measurementTfs.tfParams[1].a[0] = 1.0;
    params->linearController.measurementTfs.tfParams[1].a[1] = -2 * (1 - (0.5*T*w)*(0.5*T*w)) / (1 + T*d*w + (0.5*T*w)*(0.5*T*w));
    params->linearController.measurementTfs.tfParams[1].a[2] = (1 - T*d*w + (0.5*T*w)*(0.5*T*w)) / (1 + T*d*w + (0.5*T*w)*(0.5*T*w));

    params->linearController.measurementTfs.tfParams[2].enable = 1;
    params->linearController.measurementTfs.tfParams[2].b[0] = ((0.5*T*w)*(0.5*T*w) / (1 + T*d*w + (0.5*T*w)*(0.5*T*w))) * ((0.5*T*w)*(0.5*T*w) / (1 + T*d*w + (0.5*T*w)*(0.5*T*w)));*/

}

void ikTuneTorqueNotches(ikConLoopParams *params, double T) {

	/*! [1st side-side tower mode filter] */
    /*
	####################################################################
                    1st side-side tower mode filter

    Transfer function:
    H(s) = (s^2 + 2*dnum*w*s + w^2) / (s^2 + 2*dden*w*s + w^2)

    The sampling time is given by function parameter T.

    Set parameters here:
	*/
    double w = 2.5781; /* [rad/s] */
    double dnum = 0.05; /* [-] */
    double dden = 0.5; /* [-] */
    /*
    ####################################################################
	*/
	/*! [1st side-side tower mode filter] */

    params->linearController.measurementNotches.dT = T;
    params->linearController.measurementNotches.notchParams[0].enable = 1;
    params->linearController.measurementNotches.notchParams[0].freq = w;
    params->linearController.measurementNotches.notchParams[0].dampNum = dnum;
    params->linearController.measurementNotches.notchParams[0].dampDen = dden;

}

void ikTuneTorquePI(ikConLoopParams *params, double T) {

	/*! [Torque PI] */
    /*
	####################################################################
                    Torque PI

    Transfer function:

    C(s) = (Kp*s + Ki)/s

    The sampling time is given by function parameter T.

    Set parameters here:
	*/
    double Kp = -0.3252; /* [kNms/rad] Nm/rpm */
    double Ki = -0.3252*0.75; /* [kNm/rad] Nm/rpms */
    /*
    ####################################################################
	*/
	/*! [Torque PI] */


	/*
	tune the torque control to this tf:
           (Kp + Ki*T/2)z - (Kp - Ki*T/2)
    C(z) = ------------------------------
                      z - 1
	rad/s --> kNm
	*/
	params->linearController.errorTfs.tfParams[0].enable = 1;
    params->linearController.errorTfs.tfParams[0].b[0] = (Kp + Ki*T/2);
    params->linearController.errorTfs.tfParams[0].b[1] = -(Kp - Ki*T/2);
    params->linearController.errorTfs.tfParams[0].b[2] = 0.0;
    params->linearController.errorTfs.tfParams[0].a[0] = 1.0;
    params->linearController.errorTfs.tfParams[0].a[1] = 0.0;
    params->linearController.errorTfs.tfParams[0].a[2] = 0.0;

	params->linearController.postGainTfs.tfParams[0].enable = 1;
    params->linearController.postGainTfs.tfParams[0].b[0] = 1.0;
    params->linearController.postGainTfs.tfParams[0].b[1] = 0.0;
    params->linearController.postGainTfs.tfParams[0].b[2] = 0.0;
    params->linearController.postGainTfs.tfParams[0].a[0] = 1.0;
    params->linearController.postGainTfs.tfParams[0].a[1] = -1.0;
    params->linearController.postGainTfs.tfParams[0].a[2] = 0.0;

}
