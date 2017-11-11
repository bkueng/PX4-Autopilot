/****************************************************************************
* Copyright (c) 2014, Paul Riseborough All rights reserved.
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are met:
* 
* Redistributions of source code must retain the above copyright notice, this 
* list of conditions and the following disclaimer.
* 
* Redistributions in binary form must reproduce the above copyright notice, 
* this list of conditions and the following disclaimer in the documentation 
* and/or other materials provided with the distribution.
* 
* Neither the name of the {organization} nor the names of its contributors 
* may be used to endorse or promote products derived from this software without 
* specific prior written permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE.
****************************************************************************/

/**
 * @file estimator_22states.cpp
 *
 * Implementation of the attitude and position estimator.
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include "estimator_22states.h"

#include <px4_defines.h>
#include <mathlib/mathlib.h>
#include <cmath>

using std::cos;
using std::sin;

#define MIN_AIRSPEED_MEAS 5.0f

constexpr float ekf_covariance_diverged = 1.0e8f;

AttPosEKF::AttPosEKF() :
    covTimeStepMax(0.0f),
    covDelAngMax(0.0f),
    rngFinderPitch(0.0f),
    yawVarScale(0.0f),
    windVelSigma(0.0f),
    dAngBiasSigma(0.0f),
    dVelBiasSigma(0.0f),
    magEarthSigma(0.0f),
    magBodySigma(0.0f),
    gndHgtSigma(0.0f),
    vneSigma(0.0f),
    vdSigma(0.0f),
    posNeSigma(0.0f),
    posDSigma(0.0f),
    magMeasurementSigma(0.0f),
    airspeedMeasurementSigma(0.0f),
    gyroProcessNoise(0.0f),
    accelProcessNoise(0.0f),
    EAS2TAS(1.0f),
    magstate{},
    resetMagState{},
    KH{},
    KHP{},
    P{},
    Kfusion{},
    states{},
    resetStates{},
    storedStates{},
    statetimeStamp{},
    lastVelPosFusion(millis()),
    statesAtVelTime{},
    statesAtPosTime{},
    statesAtHgtTime{},
    statesAtMagMeasTime{},
    statesAtVtasMeasTime{},
    statesAtRngTime{},
    statesAtFlowTime{},
    accNavMag(),
    dtIMU(0.005f),
    dtIMUfilt(0.005f),
    dtVelPos(0.01f),
    dtVelPosFilt(0.01f),
    dtHgtFilt(0.01f),
    dtGpsFilt(0.1f),
    fusionModeGPS(0),
    innovVelPos{},
    varInnovVelPos{},
    velNED{},
    posNE{},
    hgtMea(0.0f),
    baroHgtOffset(0.0f),
    rngMea(0.0f),
    innovMag{},
    varInnovMag{},
    innovVtas(0.0f),
    innovRng(0.0f),
    innovOptFlow{},
    varInnovOptFlow{},
    varInnovVtas(0.0f),
    varInnovRng(0.0f),
    VtasMeas(0.0f),
    magDeclination(0.0f),
    latRef(0.0),
    lonRef(-M_PI),
    hgtRef(0.0f),
    refSet(false),
    covSkipCount(0),
    lastFixTime_ms(0),
    globalTimeStamp_ms(0),
    gpsLat(0.0),
    gpsLon(-M_PI),
    gpsHgt(0.0f),
    GPSstatus(0),
    baroHgt(0.0f),
    statesInitialised(false),
    fuseVelData(false),
    fusePosData(false),
    fuseHgtData(false),
    fuseMagData(false),
    fuseVtasData(false),
    fuseRngData(false),
    fuseOptFlowData(false),

    inhibitWindStates(true),
    inhibitMagStates(true),
    inhibitGndState(true),
    inhibitScaleState(true),

    staticMode(true),
    useGPS(false),
    useAirspeed(true),
    useCompass(true),
    useRangeFinder(true),
    useOpticalFlow(false),

    ekfDiverged(false),
    lastReset(0),
    current_ekf_state{},
    last_ekf_error{},
    numericalProtection(true),
    storeIndex(0),  
    storedOmega{},
    Popt{},
    flowStates{},
    prevPosN(0.0f),
    prevPosE(0.0f),
    auxFlowObsInnov{},
    auxFlowObsInnovVar{},
    fScaleFactorVar(0.0f),
    R_LOS(0.0f),
    auxFlowTestRatio{},
    auxRngTestRatio(0.0f),
    flowInnovGate(0.0f),
    auxFlowInnovGate(0.0f),
    rngInnovGate(0.0f),
    minFlowRng(0.0f),
    moCompR_LOS(0.0f),

    _isFixedWing(false),
    _onGround(true),
    _accNavMagHorizontal(0.0f)
{

    memset(&last_ekf_error, 0, sizeof(last_ekf_error));
    memset(&current_ekf_state, 0, sizeof(current_ekf_state));
    ZeroVariables();
    InitialiseParameters();
}

AttPosEKF::~AttPosEKF()
{
    //dtor
}

void AttPosEKF::initialiseParameters()
{
    covTimeStepMax = 0.07f; // maximum time allowed between covariance predictions
    covDelAngMax = 0.02f; // maximum delta angle between covariance predictions
    rngFinderPitch = 0.0f; // pitch angle of laser range finder in radians. Zero is aligned with the Z body axis. Positive is RH rotation about Y body axis.
    EAS2TAS = 1.0f;

    yawVarScale = 1.0f;
    windVelSigma = 0.1f;
    dAngBiasSigma = 1.0e-6;
    dVelBiasSigma = 0.0002f;
    magEarthSigma = 0.0003f;
    magBodySigma  = 0.0003f;

    vneSigma = 0.2f;
    vdSigma = 0.3f;
    posNeSigma = 2.0f;
    posDSigma = 2.0f;

    magMeasurementSigma = 0.05;
    airspeedMeasurementSigma = 1.4f;
    gyroProcessNoise = 1.4544411e-2f;
    accelProcessNoise = 0.5f;

    gndHgtSigma  = 0.1f; // terrain gradient 1-sigma
    R_LOS = 0.3f; // optical flow measurement noise variance (rad/sec)^2
    flowInnovGate = 3.0f; // number of standard deviations applied to the optical flow innovation consistency check
    auxFlowInnovGate = 10.0f; // number of standard deviations applied to the optical flow innovation consistency check used by the auxiliary filter
    rngInnovGate = 5.0f; // number of standard deviations applied to the range finder innovation consistency check
    minFlowRng = 0.3f; //minimum range between ground and flow sensor
    moCompR_LOS = 0.0; // scaler from sensor gyro rate to uncertainty in LOS rate
}


void AttPosEKF::updateStrapdownEquationsNed()
{
    Vector3f del_vel_nav;
    float q00;
    float q11;
    float q22;
    float q33;
    float q01;
    float q02;
    float q03;
    float q12;
    float q13;
    float q23;
    float rotation_mag;
    float q_updated[4];
    float quat_mag;
    float delta_quat[4];
    const Vector3f gravity_ned(0.0f, 0.0f, GRAVITY_MSS);

    // Remove sensor bias errors
    correctedDelAng.x = dAngIMU.x - states[10];
    correctedDelAng.y = dAngIMU.y - states[11];
    correctedDelAng.z = dAngIMU.z - states[12];

    Vector3f d_vel_imu_rel;

    d_vel_imu_rel.x = dVelIMU.x;
    d_vel_imu_rel.y = dVelIMU.y;
    d_vel_imu_rel.z = dVelIMU.z - states[13];

    delAngTotal.x += correctedDelAng.x;
    delAngTotal.y += correctedDelAng.y;
    delAngTotal.z += correctedDelAng.z;

    // Apply corrections for earths rotation rate and coning errors
    // * and + operators have been overloaded
    correctedDelAng = correctedDelAng - Tnb*earthRateNED*dtIMU + 8.333333333333333e-2f*(prevDelAng % correctedDelAng);
    prevDelAng = correctedDelAng;

    // Convert the rotation vector to its equivalent quaternion
    rotation_mag = correctedDelAng.length();
    if (rotation_mag < 1e-12f)
    {
        delta_quat[0] = 1.0;
        delta_quat[1] = 0.0;
        delta_quat[2] = 0.0;
        delta_quat[3] = 0.0;
    }
    else
    {
        // We are using double here as we are unsure how small
        // the angle differences are and if we get into numeric
        // issues with float. The runtime impact is not measurable
        // for these quantities.
        delta_quat[0] = cos(0.5*(double)rotation_mag);
        float rot_scaler = (sin(0.5*(double)rotation_mag))/(double)rotation_mag;
        delta_quat[1] = correctedDelAng.x*rot_scaler;
        delta_quat[2] = correctedDelAng.y*rot_scaler;
        delta_quat[3] = correctedDelAng.z*rot_scaler;
    }

    // Update the quaternions by rotating from the previous attitude through
    // the delta angle rotation quaternion
    q_updated[0] = states[0]*delta_quat[0] - states[1]*delta_quat[1] - states[2]*delta_quat[2] - states[3]*delta_quat[3];
    q_updated[1] = states[0]*delta_quat[1] + states[1]*delta_quat[0] + states[2]*delta_quat[3] - states[3]*delta_quat[2];
    q_updated[2] = states[0]*delta_quat[2] + states[2]*delta_quat[0] + states[3]*delta_quat[1] - states[1]*delta_quat[3];
    q_updated[3] = states[0]*delta_quat[3] + states[3]*delta_quat[0] + states[1]*delta_quat[2] - states[2]*delta_quat[1];

    // Normalise the quaternions and update the quaternion states
    quat_mag = sqrtf(sq(q_updated[0]) + sq(q_updated[1]) + sq(q_updated[2]) + sq(q_updated[3]));
    if (quat_mag > 1e-16f)
    {
        float quat_mag_inv = 1.0f/quat_mag;
        states[0] = quat_mag_inv*q_updated[0];
        states[1] = quat_mag_inv*q_updated[1];
        states[2] = quat_mag_inv*q_updated[2];
        states[3] = quat_mag_inv*q_updated[3];
    }

    // Calculate the body to nav cosine matrix
    q00 = sq(states[0]);
    q11 = sq(states[1]);
    q22 = sq(states[2]);
    q33 = sq(states[3]);
    q01 =  states[0]*states[1];
    q02 =  states[0]*states[2];
    q03 =  states[0]*states[3];
    q12 =  states[1]*states[2];
    q13 =  states[1]*states[3];
    q23 =  states[2]*states[3];

    Tbn.x.x = q00 + q11 - q22 - q33;
    Tbn.y.y = q00 - q11 + q22 - q33;
    Tbn.z.z = q00 - q11 - q22 + q33;
    Tbn.x.y = 2*(q12 - q03);
    Tbn.x.z = 2*(q13 + q02);
    Tbn.y.x = 2*(q12 + q03);
    Tbn.y.z = 2*(q23 - q01);
    Tbn.z.x = 2*(q13 - q02);
    Tbn.z.y = 2*(q23 + q01);

    Tnb = Tbn.transpose();

    // transform body delta velocities to delta velocities in the nav frame
    // * and + operators have been overloaded
    //delVelNav = Tbn*dVelIMU + gravityNED*dtIMU;
    del_vel_nav.x = Tbn.x.x*d_vel_imu_rel.x + Tbn.x.y*d_vel_imu_rel.y + Tbn.x.z*d_vel_imu_rel.z + gravity_ned.x*dtIMU;
    del_vel_nav.y = Tbn.y.x*d_vel_imu_rel.x + Tbn.y.y*d_vel_imu_rel.y + Tbn.y.z*d_vel_imu_rel.z + gravity_ned.y*dtIMU;
    del_vel_nav.z = Tbn.z.x*d_vel_imu_rel.x + Tbn.z.y*d_vel_imu_rel.y + Tbn.z.z*d_vel_imu_rel.z + gravity_ned.z*dtIMU;

    // calculate the magnitude of the nav acceleration (required for GPS
    // variance estimation)
    accNavMag = del_vel_nav.length()/dtIMU;

    //First order low-pass filtered magnitude of horizontal nav acceleration
    Vector3f derivative_nav = (del_vel_nav / dtIMU);
    float derivative_vel_nav_magnitude = sqrtf(sq(derivative_nav.x) + sq(derivative_nav.y));
    _accNavMagHorizontal = _accNavMagHorizontal * 0.95f + derivative_vel_nav_magnitude * 0.05f;

    // If calculating position save previous velocity
    float last_velocity[3];
    last_velocity[0] = states[4];
    last_velocity[1] = states[5];
    last_velocity[2] = states[6];

    // Sum delta velocities to get velocity
    states[4] = states[4] + del_vel_nav.x;
    states[5] = states[5] + del_vel_nav.y;
    states[6] = states[6] + del_vel_nav.z;

    // If calculating postions, do a trapezoidal integration for position
    states[7] = states[7] + 0.5f*(states[4] + last_velocity[0])*dtIMU;
    states[8] = states[8] + 0.5f*(states[5] + last_velocity[1])*dtIMU;
    states[9] = states[9] + 0.5f*(states[6] + last_velocity[2])*dtIMU;

    // Constrain states (to protect against filter divergence)
    ConstrainStates();

    // update filtered IMU time step length
    dtIMUfilt = 0.99f * dtIMUfilt + 0.01f * dtIMU;
}

void AttPosEKF::covariancePrediction(float dt)
{
    // scalars
    float dax_cov;
    float day_cov;
    float daz_cov;
    float dvx_cov;
    float dvy_cov;
    float dvz_cov;
    float dvx;
    float dvy;
    float dvz;
    float dax;
    float day;
    float daz;
    float q0;
    float q1;
    float q2;
    float q3;
    float dax_b;
    float day_b;
    float daz_b;
    float dvz_b;

    // arrays
    float process_noise[ekf_state_estimates];
    float sf[15];
    float sg[8];
    float sq[11];
    float spp[8] = {0};
    float next_p[ekf_state_estimates][ekf_state_estimates];

    // calculate covariance prediction process noise
    for (uint8_t i= 0; i<4;  i++) process_noise[i] = 1.0e-9f;
    for (uint8_t i= 4; i<10;  i++) process_noise[i] = 1.0e-9f;
    // scale gyro bias noise when on ground to allow for faster bias estimation
    float gyro_bias_scale = (_onGround) ? 2.0f : 1.0f;

    for (uint8_t i=10; i<=12; i++) process_noise[i] = dt * dAngBiasSigma * gyro_bias_scale;
    process_noise[13] = dVelBiasSigma;
    if (!inhibitWindStates) {
        for (uint8_t i=14; i<=15; i++) process_noise[i] = dt * windVelSigma;
    } else {
        for (uint8_t i=14; i<=15; i++) process_noise[i] = 0;
    }
    if (!inhibitMagStates) {
        for (uint8_t i=16; i<=18; i++) process_noise[i] = dt * magEarthSigma;
        for (uint8_t i=19; i < ekf_state_estimates; i++) process_noise[i] = dt * magBodySigma;
    } else {
        for (uint8_t i=16; i < ekf_state_estimates; i++) process_noise[i] = 0;
    }

    // square all sigmas
    for (size_t i = 0; i < ekf_state_estimates; i++) process_noise[i] = sq(process_noise[i]);

    // set variables used to calculate covariance growth
    dvx = summedDelVel.x;
    dvy = summedDelVel.y;
    dvz = summedDelVel.z;
    dax = summedDelAng.x;
    day = summedDelAng.y;
    daz = summedDelAng.z;
    q0 = states[0];
    q1 = states[1];
    q2 = states[2];
    q3 = states[3];
    dax_b = states[10];
    day_b = states[11];
    daz_b = states[12];
    dvz_b =  states[13];
    gyroProcessNoise = ConstrainFloat(gyroProcessNoise, 1e-3f, 5e-2f);
    dax_cov = sq(dt*gyroProcessNoise);
    day_cov = sq(dt*gyroProcessNoise);
    daz_cov = sq(dt*gyroProcessNoise);
    if (_onGround) daz_cov = daz_cov * sq(yawVarScale);
    accelProcessNoise = ConstrainFloat(accelProcessNoise, 5e-2, 1.0f);
    dvx_cov = sq(dt*accelProcessNoise);
    dvy_cov = sq(dt*accelProcessNoise);
    dvz_cov = sq(dt*accelProcessNoise);

    // Predicted covariance calculation
    sf[0] = dvz - dvz_b;
    sf[1] = 2*q3*sf[0] + 2*dvx*q1 + 2*dvy*q2;
    sf[2] = 2*dvx*q3 - 2*q1*sf[0] + 2*dvy*q0;
    sf[3] = 2*q2*sf[0] + 2*dvx*q0 - 2*dvy*q3;
    sf[4] = day/2 - day_b/2;
    sf[5] = daz/2 - daz_b/2;
    sf[6] = dax/2 - dax_b/2;
    sf[7] = dax_b/2 - dax/2;
    sf[8] = daz_b/2 - daz/2;
    sf[9] = day_b/2 - day/2;
    sf[10] = 2*q0*sf[0];
    sf[11] = q1/2;
    sf[12] = q2/2;
    sf[13] = q3/2;
    sf[14] = 2*dvy*q1;

    sg[0] = q0/2;
    sg[1] = sq(q3);
    sg[2] = sq(q2);
    sg[3] = sq(q1);
    sg[4] = sq(q0);
    sg[5] = 2*q2*q3;
    sg[6] = 2*q1*q3;
    sg[7] = 2*q1*q2;

    sq[0] = dvz_cov*(sg[5] - 2*q0*q1)*(sg[1] - sg[2] - sg[3] + sg[4]) - dvy_cov*(sg[5] + 2*q0*q1)*(sg[1] - sg[2] + sg[3] - sg[4]) + dvx_cov*(sg[6] - 2*q0*q2)*(sg[7] + 2*q0*q3);
    sq[1] = dvz_cov*(sg[6] + 2*q0*q2)*(sg[1] - sg[2] - sg[3] + sg[4]) - dvx_cov*(sg[6] - 2*q0*q2)*(sg[1] + sg[2] - sg[3] - sg[4]) + dvy_cov*(sg[5] + 2*q0*q1)*(sg[7] - 2*q0*q3);
    sq[2] = dvz_cov*(sg[5] - 2*q0*q1)*(sg[6] + 2*q0*q2) - dvy_cov*(sg[7] - 2*q0*q3)*(sg[1] - sg[2] + sg[3] - sg[4]) - dvx_cov*(sg[7] + 2*q0*q3)*(sg[1] + sg[2] - sg[3] - sg[4]);
    sq[3] = (day_cov*q1*sg[0])/2 - (daz_cov*q1*sg[0])/2 - (dax_cov*q2*q3)/4;
    sq[4] = (daz_cov*q2*sg[0])/2 - (dax_cov*q2*sg[0])/2 - (day_cov*q1*q3)/4;
    sq[5] = (dax_cov*q3*sg[0])/2 - (day_cov*q3*sg[0])/2 - (daz_cov*q1*q2)/4;
    sq[6] = (dax_cov*q1*q2)/4 - (daz_cov*q3*sg[0])/2 - (day_cov*q1*q2)/4;
    sq[7] = (daz_cov*q1*q3)/4 - (dax_cov*q1*q3)/4 - (day_cov*q2*sg[0])/2;
    sq[8] = (day_cov*q2*q3)/4 - (dax_cov*q1*sg[0])/2 - (daz_cov*q2*q3)/4;
    sq[9] = sq(sg[0]);
    sq[10] = sq(q1);

    spp[0] = sf[10] + sf[14] - 2*dvx*q2;
    spp[1] = 2*q2*sf[0] + 2*dvx*q0 - 2*dvy*q3;
    spp[2] = 2*dvx*q3 - 2*q1*sf[0] + 2*dvy*q0;
    spp[3] = 2*q0*q1 - 2*q2*q3;
    spp[4] = 2*q0*q2 + 2*q1*q3;
    spp[5] = sq(q0) - sq(q1) - sq(q2) + sq(q3);
    spp[6] = sf[13];
    spp[7] = sf[12];

    next_p[0][0] = P[0][0] + P[1][0]*sf[7] + P[2][0]*sf[9] + P[3][0]*sf[8] + P[10][0]*sf[11] + P[11][0]*spp[7] + P[12][0]*spp[6] + (dax_cov*sq[10])/4 + sf[7]*(P[0][1] + P[1][1]*sf[7] + P[2][1]*sf[9] + P[3][1]*sf[8] + P[10][1]*sf[11] + P[11][1]*spp[7] + P[12][1]*spp[6]) + sf[9]*(P[0][2] + P[1][2]*sf[7] + P[2][2]*sf[9] + P[3][2]*sf[8] + P[10][2]*sf[11] + P[11][2]*spp[7] + P[12][2]*spp[6]) + sf[8]*(P[0][3] + P[1][3]*sf[7] + P[2][3]*sf[9] + P[3][3]*sf[8] + P[10][3]*sf[11] + P[11][3]*spp[7] + P[12][3]*spp[6]) + sf[11]*(P[0][10] + P[1][10]*sf[7] + P[2][10]*sf[9] + P[3][10]*sf[8] + P[10][10]*sf[11] + P[11][10]*spp[7] + P[12][10]*spp[6]) + spp[7]*(P[0][11] + P[1][11]*sf[7] + P[2][11]*sf[9] + P[3][11]*sf[8] + P[10][11]*sf[11] + P[11][11]*spp[7] + P[12][11]*spp[6]) + spp[6]*(P[0][12] + P[1][12]*sf[7] + P[2][12]*sf[9] + P[3][12]*sf[8] + P[10][12]*sf[11] + P[11][12]*spp[7] + P[12][12]*spp[6]) + (day_cov*sq(q2))/4 + (daz_cov*sq(q3))/4;
    next_p[0][1] = P[0][1] + sq[8] + P[1][1]*sf[7] + P[2][1]*sf[9] + P[3][1]*sf[8] + P[10][1]*sf[11] + P[11][1]*spp[7] + P[12][1]*spp[6] + sf[6]*(P[0][0] + P[1][0]*sf[7] + P[2][0]*sf[9] + P[3][0]*sf[8] + P[10][0]*sf[11] + P[11][0]*spp[7] + P[12][0]*spp[6]) + sf[5]*(P[0][2] + P[1][2]*sf[7] + P[2][2]*sf[9] + P[3][2]*sf[8] + P[10][2]*sf[11] + P[11][2]*spp[7] + P[12][2]*spp[6]) + sf[9]*(P[0][3] + P[1][3]*sf[7] + P[2][3]*sf[9] + P[3][3]*sf[8] + P[10][3]*sf[11] + P[11][3]*spp[7] + P[12][3]*spp[6]) + spp[6]*(P[0][11] + P[1][11]*sf[7] + P[2][11]*sf[9] + P[3][11]*sf[8] + P[10][11]*sf[11] + P[11][11]*spp[7] + P[12][11]*spp[6]) - spp[7]*(P[0][12] + P[1][12]*sf[7] + P[2][12]*sf[9] + P[3][12]*sf[8] + P[10][12]*sf[11] + P[11][12]*spp[7] + P[12][12]*spp[6]) - (q0*(P[0][10] + P[1][10]*sf[7] + P[2][10]*sf[9] + P[3][10]*sf[8] + P[10][10]*sf[11] + P[11][10]*spp[7] + P[12][10]*spp[6]))/2;
    next_p[0][2] = P[0][2] + sq[7] + P[1][2]*sf[7] + P[2][2]*sf[9] + P[3][2]*sf[8] + P[10][2]*sf[11] + P[11][2]*spp[7] + P[12][2]*spp[6] + sf[4]*(P[0][0] + P[1][0]*sf[7] + P[2][0]*sf[9] + P[3][0]*sf[8] + P[10][0]*sf[11] + P[11][0]*spp[7] + P[12][0]*spp[6]) + sf[8]*(P[0][1] + P[1][1]*sf[7] + P[2][1]*sf[9] + P[3][1]*sf[8] + P[10][1]*sf[11] + P[11][1]*spp[7] + P[12][1]*spp[6]) + sf[6]*(P[0][3] + P[1][3]*sf[7] + P[2][3]*sf[9] + P[3][3]*sf[8] + P[10][3]*sf[11] + P[11][3]*spp[7] + P[12][3]*spp[6]) + sf[11]*(P[0][12] + P[1][12]*sf[7] + P[2][12]*sf[9] + P[3][12]*sf[8] + P[10][12]*sf[11] + P[11][12]*spp[7] + P[12][12]*spp[6]) - spp[6]*(P[0][10] + P[1][10]*sf[7] + P[2][10]*sf[9] + P[3][10]*sf[8] + P[10][10]*sf[11] + P[11][10]*spp[7] + P[12][10]*spp[6]) - (q0*(P[0][11] + P[1][11]*sf[7] + P[2][11]*sf[9] + P[3][11]*sf[8] + P[10][11]*sf[11] + P[11][11]*spp[7] + P[12][11]*spp[6]))/2;
    next_p[0][3] = P[0][3] + sq[6] + P[1][3]*sf[7] + P[2][3]*sf[9] + P[3][3]*sf[8] + P[10][3]*sf[11] + P[11][3]*spp[7] + P[12][3]*spp[6] + sf[5]*(P[0][0] + P[1][0]*sf[7] + P[2][0]*sf[9] + P[3][0]*sf[8] + P[10][0]*sf[11] + P[11][0]*spp[7] + P[12][0]*spp[6]) + sf[4]*(P[0][1] + P[1][1]*sf[7] + P[2][1]*sf[9] + P[3][1]*sf[8] + P[10][1]*sf[11] + P[11][1]*spp[7] + P[12][1]*spp[6]) + sf[7]*(P[0][2] + P[1][2]*sf[7] + P[2][2]*sf[9] + P[3][2]*sf[8] + P[10][2]*sf[11] + P[11][2]*spp[7] + P[12][2]*spp[6]) - sf[11]*(P[0][11] + P[1][11]*sf[7] + P[2][11]*sf[9] + P[3][11]*sf[8] + P[10][11]*sf[11] + P[11][11]*spp[7] + P[12][11]*spp[6]) + spp[7]*(P[0][10] + P[1][10]*sf[7] + P[2][10]*sf[9] + P[3][10]*sf[8] + P[10][10]*sf[11] + P[11][10]*spp[7] + P[12][10]*spp[6]) - (q0*(P[0][12] + P[1][12]*sf[7] + P[2][12]*sf[9] + P[3][12]*sf[8] + P[10][12]*sf[11] + P[11][12]*spp[7] + P[12][12]*spp[6]))/2;
    next_p[0][4] = P[0][4] + P[1][4]*sf[7] + P[2][4]*sf[9] + P[3][4]*sf[8] + P[10][4]*sf[11] + P[11][4]*spp[7] + P[12][4]*spp[6] + sf[3]*(P[0][0] + P[1][0]*sf[7] + P[2][0]*sf[9] + P[3][0]*sf[8] + P[10][0]*sf[11] + P[11][0]*spp[7] + P[12][0]*spp[6]) + sf[1]*(P[0][1] + P[1][1]*sf[7] + P[2][1]*sf[9] + P[3][1]*sf[8] + P[10][1]*sf[11] + P[11][1]*spp[7] + P[12][1]*spp[6]) + spp[0]*(P[0][2] + P[1][2]*sf[7] + P[2][2]*sf[9] + P[3][2]*sf[8] + P[10][2]*sf[11] + P[11][2]*spp[7] + P[12][2]*spp[6]) - spp[2]*(P[0][3] + P[1][3]*sf[7] + P[2][3]*sf[9] + P[3][3]*sf[8] + P[10][3]*sf[11] + P[11][3]*spp[7] + P[12][3]*spp[6]) - spp[4]*(P[0][13] + P[1][13]*sf[7] + P[2][13]*sf[9] + P[3][13]*sf[8] + P[10][13]*sf[11] + P[11][13]*spp[7] + P[12][13]*spp[6]);
    next_p[0][5] = P[0][5] + P[1][5]*sf[7] + P[2][5]*sf[9] + P[3][5]*sf[8] + P[10][5]*sf[11] + P[11][5]*spp[7] + P[12][5]*spp[6] + sf[2]*(P[0][0] + P[1][0]*sf[7] + P[2][0]*sf[9] + P[3][0]*sf[8] + P[10][0]*sf[11] + P[11][0]*spp[7] + P[12][0]*spp[6]) + sf[1]*(P[0][2] + P[1][2]*sf[7] + P[2][2]*sf[9] + P[3][2]*sf[8] + P[10][2]*sf[11] + P[11][2]*spp[7] + P[12][2]*spp[6]) + sf[3]*(P[0][3] + P[1][3]*sf[7] + P[2][3]*sf[9] + P[3][3]*sf[8] + P[10][3]*sf[11] + P[11][3]*spp[7] + P[12][3]*spp[6]) - spp[0]*(P[0][1] + P[1][1]*sf[7] + P[2][1]*sf[9] + P[3][1]*sf[8] + P[10][1]*sf[11] + P[11][1]*spp[7] + P[12][1]*spp[6]) + spp[3]*(P[0][13] + P[1][13]*sf[7] + P[2][13]*sf[9] + P[3][13]*sf[8] + P[10][13]*sf[11] + P[11][13]*spp[7] + P[12][13]*spp[6]);
    next_p[0][6] = P[0][6] + P[1][6]*sf[7] + P[2][6]*sf[9] + P[3][6]*sf[8] + P[10][6]*sf[11] + P[11][6]*spp[7] + P[12][6]*spp[6] + sf[2]*(P[0][1] + P[1][1]*sf[7] + P[2][1]*sf[9] + P[3][1]*sf[8] + P[10][1]*sf[11] + P[11][1]*spp[7] + P[12][1]*spp[6]) + sf[1]*(P[0][3] + P[1][3]*sf[7] + P[2][3]*sf[9] + P[3][3]*sf[8] + P[10][3]*sf[11] + P[11][3]*spp[7] + P[12][3]*spp[6]) + spp[0]*(P[0][0] + P[1][0]*sf[7] + P[2][0]*sf[9] + P[3][0]*sf[8] + P[10][0]*sf[11] + P[11][0]*spp[7] + P[12][0]*spp[6]) - spp[1]*(P[0][2] + P[1][2]*sf[7] + P[2][2]*sf[9] + P[3][2]*sf[8] + P[10][2]*sf[11] + P[11][2]*spp[7] + P[12][2]*spp[6]) - (sq(q0) - sq(q1) - sq(q2) + sq(q3))*(P[0][13] + P[1][13]*sf[7] + P[2][13]*sf[9] + P[3][13]*sf[8] + P[10][13]*sf[11] + P[11][13]*spp[7] + P[12][13]*spp[6]);
    next_p[0][7] = P[0][7] + P[1][7]*sf[7] + P[2][7]*sf[9] + P[3][7]*sf[8] + P[10][7]*sf[11] + P[11][7]*spp[7] + P[12][7]*spp[6] + dt*(P[0][4] + P[1][4]*sf[7] + P[2][4]*sf[9] + P[3][4]*sf[8] + P[10][4]*sf[11] + P[11][4]*spp[7] + P[12][4]*spp[6]);
    next_p[0][8] = P[0][8] + P[1][8]*sf[7] + P[2][8]*sf[9] + P[3][8]*sf[8] + P[10][8]*sf[11] + P[11][8]*spp[7] + P[12][8]*spp[6] + dt*(P[0][5] + P[1][5]*sf[7] + P[2][5]*sf[9] + P[3][5]*sf[8] + P[10][5]*sf[11] + P[11][5]*spp[7] + P[12][5]*spp[6]);
    next_p[0][9] = P[0][9] + P[1][9]*sf[7] + P[2][9]*sf[9] + P[3][9]*sf[8] + P[10][9]*sf[11] + P[11][9]*spp[7] + P[12][9]*spp[6] + dt*(P[0][6] + P[1][6]*sf[7] + P[2][6]*sf[9] + P[3][6]*sf[8] + P[10][6]*sf[11] + P[11][6]*spp[7] + P[12][6]*spp[6]);
    next_p[0][10] = P[0][10] + P[1][10]*sf[7] + P[2][10]*sf[9] + P[3][10]*sf[8] + P[10][10]*sf[11] + P[11][10]*spp[7] + P[12][10]*spp[6];
    next_p[0][11] = P[0][11] + P[1][11]*sf[7] + P[2][11]*sf[9] + P[3][11]*sf[8] + P[10][11]*sf[11] + P[11][11]*spp[7] + P[12][11]*spp[6];
    next_p[0][12] = P[0][12] + P[1][12]*sf[7] + P[2][12]*sf[9] + P[3][12]*sf[8] + P[10][12]*sf[11] + P[11][12]*spp[7] + P[12][12]*spp[6];
    next_p[0][13] = P[0][13] + P[1][13]*sf[7] + P[2][13]*sf[9] + P[3][13]*sf[8] + P[10][13]*sf[11] + P[11][13]*spp[7] + P[12][13]*spp[6];
    next_p[0][14] = P[0][14] + P[1][14]*sf[7] + P[2][14]*sf[9] + P[3][14]*sf[8] + P[10][14]*sf[11] + P[11][14]*spp[7] + P[12][14]*spp[6];
    next_p[0][15] = P[0][15] + P[1][15]*sf[7] + P[2][15]*sf[9] + P[3][15]*sf[8] + P[10][15]*sf[11] + P[11][15]*spp[7] + P[12][15]*spp[6];
    next_p[0][16] = P[0][16] + P[1][16]*sf[7] + P[2][16]*sf[9] + P[3][16]*sf[8] + P[10][16]*sf[11] + P[11][16]*spp[7] + P[12][16]*spp[6];
    next_p[0][17] = P[0][17] + P[1][17]*sf[7] + P[2][17]*sf[9] + P[3][17]*sf[8] + P[10][17]*sf[11] + P[11][17]*spp[7] + P[12][17]*spp[6];
    next_p[0][18] = P[0][18] + P[1][18]*sf[7] + P[2][18]*sf[9] + P[3][18]*sf[8] + P[10][18]*sf[11] + P[11][18]*spp[7] + P[12][18]*spp[6];
    next_p[0][19] = P[0][19] + P[1][19]*sf[7] + P[2][19]*sf[9] + P[3][19]*sf[8] + P[10][19]*sf[11] + P[11][19]*spp[7] + P[12][19]*spp[6];
    next_p[0][20] = P[0][20] + P[1][20]*sf[7] + P[2][20]*sf[9] + P[3][20]*sf[8] + P[10][20]*sf[11] + P[11][20]*spp[7] + P[12][20]*spp[6];
    next_p[0][21] = P[0][21] + P[1][21]*sf[7] + P[2][21]*sf[9] + P[3][21]*sf[8] + P[10][21]*sf[11] + P[11][21]*spp[7] + P[12][21]*spp[6];
    next_p[1][0] = P[1][0] + sq[8] + P[0][0]*sf[6] + P[2][0]*sf[5] + P[3][0]*sf[9] + P[11][0]*spp[6] - P[12][0]*spp[7] - (P[10][0]*q0)/2 + sf[7]*(P[1][1] + P[0][1]*sf[6] + P[2][1]*sf[5] + P[3][1]*sf[9] + P[11][1]*spp[6] - P[12][1]*spp[7] - (P[10][1]*q0)/2) + sf[9]*(P[1][2] + P[0][2]*sf[6] + P[2][2]*sf[5] + P[3][2]*sf[9] + P[11][2]*spp[6] - P[12][2]*spp[7] - (P[10][2]*q0)/2) + sf[8]*(P[1][3] + P[0][3]*sf[6] + P[2][3]*sf[5] + P[3][3]*sf[9] + P[11][3]*spp[6] - P[12][3]*spp[7] - (P[10][3]*q0)/2) + sf[11]*(P[1][10] + P[0][10]*sf[6] + P[2][10]*sf[5] + P[3][10]*sf[9] + P[11][10]*spp[6] - P[12][10]*spp[7] - (P[10][10]*q0)/2) + spp[7]*(P[1][11] + P[0][11]*sf[6] + P[2][11]*sf[5] + P[3][11]*sf[9] + P[11][11]*spp[6] - P[12][11]*spp[7] - (P[10][11]*q0)/2) + spp[6]*(P[1][12] + P[0][12]*sf[6] + P[2][12]*sf[5] + P[3][12]*sf[9] + P[11][12]*spp[6] - P[12][12]*spp[7] - (P[10][12]*q0)/2);
    next_p[1][1] = P[1][1] + P[0][1]*sf[6] + P[2][1]*sf[5] + P[3][1]*sf[9] + P[11][1]*spp[6] - P[12][1]*spp[7] + dax_cov*sq[9] - (P[10][1]*q0)/2 + sf[6]*(P[1][0] + P[0][0]*sf[6] + P[2][0]*sf[5] + P[3][0]*sf[9] + P[11][0]*spp[6] - P[12][0]*spp[7] - (P[10][0]*q0)/2) + sf[5]*(P[1][2] + P[0][2]*sf[6] + P[2][2]*sf[5] + P[3][2]*sf[9] + P[11][2]*spp[6] - P[12][2]*spp[7] - (P[10][2]*q0)/2) + sf[9]*(P[1][3] + P[0][3]*sf[6] + P[2][3]*sf[5] + P[3][3]*sf[9] + P[11][3]*spp[6] - P[12][3]*spp[7] - (P[10][3]*q0)/2) + spp[6]*(P[1][11] + P[0][11]*sf[6] + P[2][11]*sf[5] + P[3][11]*sf[9] + P[11][11]*spp[6] - P[12][11]*spp[7] - (P[10][11]*q0)/2) - spp[7]*(P[1][12] + P[0][12]*sf[6] + P[2][12]*sf[5] + P[3][12]*sf[9] + P[11][12]*spp[6] - P[12][12]*spp[7] - (P[10][12]*q0)/2) + (day_cov*sq(q3))/4 + (daz_cov*sq(q2))/4 - (q0*(P[1][10] + P[0][10]*sf[6] + P[2][10]*sf[5] + P[3][10]*sf[9] + P[11][10]*spp[6] - P[12][10]*spp[7] - (P[10][10]*q0)/2))/2;
    next_p[1][2] = P[1][2] + sq[5] + P[0][2]*sf[6] + P[2][2]*sf[5] + P[3][2]*sf[9] + P[11][2]*spp[6] - P[12][2]*spp[7] - (P[10][2]*q0)/2 + sf[4]*(P[1][0] + P[0][0]*sf[6] + P[2][0]*sf[5] + P[3][0]*sf[9] + P[11][0]*spp[6] - P[12][0]*spp[7] - (P[10][0]*q0)/2) + sf[8]*(P[1][1] + P[0][1]*sf[6] + P[2][1]*sf[5] + P[3][1]*sf[9] + P[11][1]*spp[6] - P[12][1]*spp[7] - (P[10][1]*q0)/2) + sf[6]*(P[1][3] + P[0][3]*sf[6] + P[2][3]*sf[5] + P[3][3]*sf[9] + P[11][3]*spp[6] - P[12][3]*spp[7] - (P[10][3]*q0)/2) + sf[11]*(P[1][12] + P[0][12]*sf[6] + P[2][12]*sf[5] + P[3][12]*sf[9] + P[11][12]*spp[6] - P[12][12]*spp[7] - (P[10][12]*q0)/2) - spp[6]*(P[1][10] + P[0][10]*sf[6] + P[2][10]*sf[5] + P[3][10]*sf[9] + P[11][10]*spp[6] - P[12][10]*spp[7] - (P[10][10]*q0)/2) - (q0*(P[1][11] + P[0][11]*sf[6] + P[2][11]*sf[5] + P[3][11]*sf[9] + P[11][11]*spp[6] - P[12][11]*spp[7] - (P[10][11]*q0)/2))/2;
    next_p[1][3] = P[1][3] + sq[4] + P[0][3]*sf[6] + P[2][3]*sf[5] + P[3][3]*sf[9] + P[11][3]*spp[6] - P[12][3]*spp[7] - (P[10][3]*q0)/2 + sf[5]*(P[1][0] + P[0][0]*sf[6] + P[2][0]*sf[5] + P[3][0]*sf[9] + P[11][0]*spp[6] - P[12][0]*spp[7] - (P[10][0]*q0)/2) + sf[4]*(P[1][1] + P[0][1]*sf[6] + P[2][1]*sf[5] + P[3][1]*sf[9] + P[11][1]*spp[6] - P[12][1]*spp[7] - (P[10][1]*q0)/2) + sf[7]*(P[1][2] + P[0][2]*sf[6] + P[2][2]*sf[5] + P[3][2]*sf[9] + P[11][2]*spp[6] - P[12][2]*spp[7] - (P[10][2]*q0)/2) - sf[11]*(P[1][11] + P[0][11]*sf[6] + P[2][11]*sf[5] + P[3][11]*sf[9] + P[11][11]*spp[6] - P[12][11]*spp[7] - (P[10][11]*q0)/2) + spp[7]*(P[1][10] + P[0][10]*sf[6] + P[2][10]*sf[5] + P[3][10]*sf[9] + P[11][10]*spp[6] - P[12][10]*spp[7] - (P[10][10]*q0)/2) - (q0*(P[1][12] + P[0][12]*sf[6] + P[2][12]*sf[5] + P[3][12]*sf[9] + P[11][12]*spp[6] - P[12][12]*spp[7] - (P[10][12]*q0)/2))/2;
    next_p[1][4] = P[1][4] + P[0][4]*sf[6] + P[2][4]*sf[5] + P[3][4]*sf[9] + P[11][4]*spp[6] - P[12][4]*spp[7] - (P[10][4]*q0)/2 + sf[3]*(P[1][0] + P[0][0]*sf[6] + P[2][0]*sf[5] + P[3][0]*sf[9] + P[11][0]*spp[6] - P[12][0]*spp[7] - (P[10][0]*q0)/2) + sf[1]*(P[1][1] + P[0][1]*sf[6] + P[2][1]*sf[5] + P[3][1]*sf[9] + P[11][1]*spp[6] - P[12][1]*spp[7] - (P[10][1]*q0)/2) + spp[0]*(P[1][2] + P[0][2]*sf[6] + P[2][2]*sf[5] + P[3][2]*sf[9] + P[11][2]*spp[6] - P[12][2]*spp[7] - (P[10][2]*q0)/2) - spp[2]*(P[1][3] + P[0][3]*sf[6] + P[2][3]*sf[5] + P[3][3]*sf[9] + P[11][3]*spp[6] - P[12][3]*spp[7] - (P[10][3]*q0)/2) - spp[4]*(P[1][13] + P[0][13]*sf[6] + P[2][13]*sf[5] + P[3][13]*sf[9] + P[11][13]*spp[6] - P[12][13]*spp[7] - (P[10][13]*q0)/2);
    next_p[1][5] = P[1][5] + P[0][5]*sf[6] + P[2][5]*sf[5] + P[3][5]*sf[9] + P[11][5]*spp[6] - P[12][5]*spp[7] - (P[10][5]*q0)/2 + sf[2]*(P[1][0] + P[0][0]*sf[6] + P[2][0]*sf[5] + P[3][0]*sf[9] + P[11][0]*spp[6] - P[12][0]*spp[7] - (P[10][0]*q0)/2) + sf[1]*(P[1][2] + P[0][2]*sf[6] + P[2][2]*sf[5] + P[3][2]*sf[9] + P[11][2]*spp[6] - P[12][2]*spp[7] - (P[10][2]*q0)/2) + sf[3]*(P[1][3] + P[0][3]*sf[6] + P[2][3]*sf[5] + P[3][3]*sf[9] + P[11][3]*spp[6] - P[12][3]*spp[7] - (P[10][3]*q0)/2) - spp[0]*(P[1][1] + P[0][1]*sf[6] + P[2][1]*sf[5] + P[3][1]*sf[9] + P[11][1]*spp[6] - P[12][1]*spp[7] - (P[10][1]*q0)/2) + spp[3]*(P[1][13] + P[0][13]*sf[6] + P[2][13]*sf[5] + P[3][13]*sf[9] + P[11][13]*spp[6] - P[12][13]*spp[7] - (P[10][13]*q0)/2);
    next_p[1][6] = P[1][6] + P[0][6]*sf[6] + P[2][6]*sf[5] + P[3][6]*sf[9] + P[11][6]*spp[6] - P[12][6]*spp[7] - (P[10][6]*q0)/2 + sf[2]*(P[1][1] + P[0][1]*sf[6] + P[2][1]*sf[5] + P[3][1]*sf[9] + P[11][1]*spp[6] - P[12][1]*spp[7] - (P[10][1]*q0)/2) + sf[1]*(P[1][3] + P[0][3]*sf[6] + P[2][3]*sf[5] + P[3][3]*sf[9] + P[11][3]*spp[6] - P[12][3]*spp[7] - (P[10][3]*q0)/2) + spp[0]*(P[1][0] + P[0][0]*sf[6] + P[2][0]*sf[5] + P[3][0]*sf[9] + P[11][0]*spp[6] - P[12][0]*spp[7] - (P[10][0]*q0)/2) - spp[1]*(P[1][2] + P[0][2]*sf[6] + P[2][2]*sf[5] + P[3][2]*sf[9] + P[11][2]*spp[6] - P[12][2]*spp[7] - (P[10][2]*q0)/2) - (sq(q0) - sq(q1) - sq(q2) + sq(q3))*(P[1][13] + P[0][13]*sf[6] + P[2][13]*sf[5] + P[3][13]*sf[9] + P[11][13]*spp[6] - P[12][13]*spp[7] - (P[10][13]*q0)/2);
    next_p[1][7] = P[1][7] + P[0][7]*sf[6] + P[2][7]*sf[5] + P[3][7]*sf[9] + P[11][7]*spp[6] - P[12][7]*spp[7] - (P[10][7]*q0)/2 + dt*(P[1][4] + P[0][4]*sf[6] + P[2][4]*sf[5] + P[3][4]*sf[9] + P[11][4]*spp[6] - P[12][4]*spp[7] - (P[10][4]*q0)/2);
    next_p[1][8] = P[1][8] + P[0][8]*sf[6] + P[2][8]*sf[5] + P[3][8]*sf[9] + P[11][8]*spp[6] - P[12][8]*spp[7] - (P[10][8]*q0)/2 + dt*(P[1][5] + P[0][5]*sf[6] + P[2][5]*sf[5] + P[3][5]*sf[9] + P[11][5]*spp[6] - P[12][5]*spp[7] - (P[10][5]*q0)/2);
    next_p[1][9] = P[1][9] + P[0][9]*sf[6] + P[2][9]*sf[5] + P[3][9]*sf[9] + P[11][9]*spp[6] - P[12][9]*spp[7] - (P[10][9]*q0)/2 + dt*(P[1][6] + P[0][6]*sf[6] + P[2][6]*sf[5] + P[3][6]*sf[9] + P[11][6]*spp[6] - P[12][6]*spp[7] - (P[10][6]*q0)/2);
    next_p[1][10] = P[1][10] + P[0][10]*sf[6] + P[2][10]*sf[5] + P[3][10]*sf[9] + P[11][10]*spp[6] - P[12][10]*spp[7] - (P[10][10]*q0)/2;
    next_p[1][11] = P[1][11] + P[0][11]*sf[6] + P[2][11]*sf[5] + P[3][11]*sf[9] + P[11][11]*spp[6] - P[12][11]*spp[7] - (P[10][11]*q0)/2;
    next_p[1][12] = P[1][12] + P[0][12]*sf[6] + P[2][12]*sf[5] + P[3][12]*sf[9] + P[11][12]*spp[6] - P[12][12]*spp[7] - (P[10][12]*q0)/2;
    next_p[1][13] = P[1][13] + P[0][13]*sf[6] + P[2][13]*sf[5] + P[3][13]*sf[9] + P[11][13]*spp[6] - P[12][13]*spp[7] - (P[10][13]*q0)/2;
    next_p[1][14] = P[1][14] + P[0][14]*sf[6] + P[2][14]*sf[5] + P[3][14]*sf[9] + P[11][14]*spp[6] - P[12][14]*spp[7] - (P[10][14]*q0)/2;
    next_p[1][15] = P[1][15] + P[0][15]*sf[6] + P[2][15]*sf[5] + P[3][15]*sf[9] + P[11][15]*spp[6] - P[12][15]*spp[7] - (P[10][15]*q0)/2;
    next_p[1][16] = P[1][16] + P[0][16]*sf[6] + P[2][16]*sf[5] + P[3][16]*sf[9] + P[11][16]*spp[6] - P[12][16]*spp[7] - (P[10][16]*q0)/2;
    next_p[1][17] = P[1][17] + P[0][17]*sf[6] + P[2][17]*sf[5] + P[3][17]*sf[9] + P[11][17]*spp[6] - P[12][17]*spp[7] - (P[10][17]*q0)/2;
    next_p[1][18] = P[1][18] + P[0][18]*sf[6] + P[2][18]*sf[5] + P[3][18]*sf[9] + P[11][18]*spp[6] - P[12][18]*spp[7] - (P[10][18]*q0)/2;
    next_p[1][19] = P[1][19] + P[0][19]*sf[6] + P[2][19]*sf[5] + P[3][19]*sf[9] + P[11][19]*spp[6] - P[12][19]*spp[7] - (P[10][19]*q0)/2;
    next_p[1][20] = P[1][20] + P[0][20]*sf[6] + P[2][20]*sf[5] + P[3][20]*sf[9] + P[11][20]*spp[6] - P[12][20]*spp[7] - (P[10][20]*q0)/2;
    next_p[1][21] = P[1][21] + P[0][21]*sf[6] + P[2][21]*sf[5] + P[3][21]*sf[9] + P[11][21]*spp[6] - P[12][21]*spp[7] - (P[10][21]*q0)/2;
    next_p[2][0] = P[2][0] + sq[7] + P[0][0]*sf[4] + P[1][0]*sf[8] + P[3][0]*sf[6] + P[12][0]*sf[11] - P[10][0]*spp[6] - (P[11][0]*q0)/2 + sf[7]*(P[2][1] + P[0][1]*sf[4] + P[1][1]*sf[8] + P[3][1]*sf[6] + P[12][1]*sf[11] - P[10][1]*spp[6] - (P[11][1]*q0)/2) + sf[9]*(P[2][2] + P[0][2]*sf[4] + P[1][2]*sf[8] + P[3][2]*sf[6] + P[12][2]*sf[11] - P[10][2]*spp[6] - (P[11][2]*q0)/2) + sf[8]*(P[2][3] + P[0][3]*sf[4] + P[1][3]*sf[8] + P[3][3]*sf[6] + P[12][3]*sf[11] - P[10][3]*spp[6] - (P[11][3]*q0)/2) + sf[11]*(P[2][10] + P[0][10]*sf[4] + P[1][10]*sf[8] + P[3][10]*sf[6] + P[12][10]*sf[11] - P[10][10]*spp[6] - (P[11][10]*q0)/2) + spp[7]*(P[2][11] + P[0][11]*sf[4] + P[1][11]*sf[8] + P[3][11]*sf[6] + P[12][11]*sf[11] - P[10][11]*spp[6] - (P[11][11]*q0)/2) + spp[6]*(P[2][12] + P[0][12]*sf[4] + P[1][12]*sf[8] + P[3][12]*sf[6] + P[12][12]*sf[11] - P[10][12]*spp[6] - (P[11][12]*q0)/2);
    next_p[2][1] = P[2][1] + sq[5] + P[0][1]*sf[4] + P[1][1]*sf[8] + P[3][1]*sf[6] + P[12][1]*sf[11] - P[10][1]*spp[6] - (P[11][1]*q0)/2 + sf[6]*(P[2][0] + P[0][0]*sf[4] + P[1][0]*sf[8] + P[3][0]*sf[6] + P[12][0]*sf[11] - P[10][0]*spp[6] - (P[11][0]*q0)/2) + sf[5]*(P[2][2] + P[0][2]*sf[4] + P[1][2]*sf[8] + P[3][2]*sf[6] + P[12][2]*sf[11] - P[10][2]*spp[6] - (P[11][2]*q0)/2) + sf[9]*(P[2][3] + P[0][3]*sf[4] + P[1][3]*sf[8] + P[3][3]*sf[6] + P[12][3]*sf[11] - P[10][3]*spp[6] - (P[11][3]*q0)/2) + spp[6]*(P[2][11] + P[0][11]*sf[4] + P[1][11]*sf[8] + P[3][11]*sf[6] + P[12][11]*sf[11] - P[10][11]*spp[6] - (P[11][11]*q0)/2) - spp[7]*(P[2][12] + P[0][12]*sf[4] + P[1][12]*sf[8] + P[3][12]*sf[6] + P[12][12]*sf[11] - P[10][12]*spp[6] - (P[11][12]*q0)/2) - (q0*(P[2][10] + P[0][10]*sf[4] + P[1][10]*sf[8] + P[3][10]*sf[6] + P[12][10]*sf[11] - P[10][10]*spp[6] - (P[11][10]*q0)/2))/2;
    next_p[2][2] = P[2][2] + P[0][2]*sf[4] + P[1][2]*sf[8] + P[3][2]*sf[6] + P[12][2]*sf[11] - P[10][2]*spp[6] + day_cov*sq[9] + (daz_cov*sq[10])/4 - (P[11][2]*q0)/2 + sf[4]*(P[2][0] + P[0][0]*sf[4] + P[1][0]*sf[8] + P[3][0]*sf[6] + P[12][0]*sf[11] - P[10][0]*spp[6] - (P[11][0]*q0)/2) + sf[8]*(P[2][1] + P[0][1]*sf[4] + P[1][1]*sf[8] + P[3][1]*sf[6] + P[12][1]*sf[11] - P[10][1]*spp[6] - (P[11][1]*q0)/2) + sf[6]*(P[2][3] + P[0][3]*sf[4] + P[1][3]*sf[8] + P[3][3]*sf[6] + P[12][3]*sf[11] - P[10][3]*spp[6] - (P[11][3]*q0)/2) + sf[11]*(P[2][12] + P[0][12]*sf[4] + P[1][12]*sf[8] + P[3][12]*sf[6] + P[12][12]*sf[11] - P[10][12]*spp[6] - (P[11][12]*q0)/2) - spp[6]*(P[2][10] + P[0][10]*sf[4] + P[1][10]*sf[8] + P[3][10]*sf[6] + P[12][10]*sf[11] - P[10][10]*spp[6] - (P[11][10]*q0)/2) + (dax_cov*sq(q3))/4 - (q0*(P[2][11] + P[0][11]*sf[4] + P[1][11]*sf[8] + P[3][11]*sf[6] + P[12][11]*sf[11] - P[10][11]*spp[6] - (P[11][11]*q0)/2))/2;
    next_p[2][3] = P[2][3] + sq[3] + P[0][3]*sf[4] + P[1][3]*sf[8] + P[3][3]*sf[6] + P[12][3]*sf[11] - P[10][3]*spp[6] - (P[11][3]*q0)/2 + sf[5]*(P[2][0] + P[0][0]*sf[4] + P[1][0]*sf[8] + P[3][0]*sf[6] + P[12][0]*sf[11] - P[10][0]*spp[6] - (P[11][0]*q0)/2) + sf[4]*(P[2][1] + P[0][1]*sf[4] + P[1][1]*sf[8] + P[3][1]*sf[6] + P[12][1]*sf[11] - P[10][1]*spp[6] - (P[11][1]*q0)/2) + sf[7]*(P[2][2] + P[0][2]*sf[4] + P[1][2]*sf[8] + P[3][2]*sf[6] + P[12][2]*sf[11] - P[10][2]*spp[6] - (P[11][2]*q0)/2) - sf[11]*(P[2][11] + P[0][11]*sf[4] + P[1][11]*sf[8] + P[3][11]*sf[6] + P[12][11]*sf[11] - P[10][11]*spp[6] - (P[11][11]*q0)/2) + spp[7]*(P[2][10] + P[0][10]*sf[4] + P[1][10]*sf[8] + P[3][10]*sf[6] + P[12][10]*sf[11] - P[10][10]*spp[6] - (P[11][10]*q0)/2) - (q0*(P[2][12] + P[0][12]*sf[4] + P[1][12]*sf[8] + P[3][12]*sf[6] + P[12][12]*sf[11] - P[10][12]*spp[6] - (P[11][12]*q0)/2))/2;
    next_p[2][4] = P[2][4] + P[0][4]*sf[4] + P[1][4]*sf[8] + P[3][4]*sf[6] + P[12][4]*sf[11] - P[10][4]*spp[6] - (P[11][4]*q0)/2 + sf[3]*(P[2][0] + P[0][0]*sf[4] + P[1][0]*sf[8] + P[3][0]*sf[6] + P[12][0]*sf[11] - P[10][0]*spp[6] - (P[11][0]*q0)/2) + sf[1]*(P[2][1] + P[0][1]*sf[4] + P[1][1]*sf[8] + P[3][1]*sf[6] + P[12][1]*sf[11] - P[10][1]*spp[6] - (P[11][1]*q0)/2) + spp[0]*(P[2][2] + P[0][2]*sf[4] + P[1][2]*sf[8] + P[3][2]*sf[6] + P[12][2]*sf[11] - P[10][2]*spp[6] - (P[11][2]*q0)/2) - spp[2]*(P[2][3] + P[0][3]*sf[4] + P[1][3]*sf[8] + P[3][3]*sf[6] + P[12][3]*sf[11] - P[10][3]*spp[6] - (P[11][3]*q0)/2) - spp[4]*(P[2][13] + P[0][13]*sf[4] + P[1][13]*sf[8] + P[3][13]*sf[6] + P[12][13]*sf[11] - P[10][13]*spp[6] - (P[11][13]*q0)/2);
    next_p[2][5] = P[2][5] + P[0][5]*sf[4] + P[1][5]*sf[8] + P[3][5]*sf[6] + P[12][5]*sf[11] - P[10][5]*spp[6] - (P[11][5]*q0)/2 + sf[2]*(P[2][0] + P[0][0]*sf[4] + P[1][0]*sf[8] + P[3][0]*sf[6] + P[12][0]*sf[11] - P[10][0]*spp[6] - (P[11][0]*q0)/2) + sf[1]*(P[2][2] + P[0][2]*sf[4] + P[1][2]*sf[8] + P[3][2]*sf[6] + P[12][2]*sf[11] - P[10][2]*spp[6] - (P[11][2]*q0)/2) + sf[3]*(P[2][3] + P[0][3]*sf[4] + P[1][3]*sf[8] + P[3][3]*sf[6] + P[12][3]*sf[11] - P[10][3]*spp[6] - (P[11][3]*q0)/2) - spp[0]*(P[2][1] + P[0][1]*sf[4] + P[1][1]*sf[8] + P[3][1]*sf[6] + P[12][1]*sf[11] - P[10][1]*spp[6] - (P[11][1]*q0)/2) + spp[3]*(P[2][13] + P[0][13]*sf[4] + P[1][13]*sf[8] + P[3][13]*sf[6] + P[12][13]*sf[11] - P[10][13]*spp[6] - (P[11][13]*q0)/2);
    next_p[2][6] = P[2][6] + P[0][6]*sf[4] + P[1][6]*sf[8] + P[3][6]*sf[6] + P[12][6]*sf[11] - P[10][6]*spp[6] - (P[11][6]*q0)/2 + sf[2]*(P[2][1] + P[0][1]*sf[4] + P[1][1]*sf[8] + P[3][1]*sf[6] + P[12][1]*sf[11] - P[10][1]*spp[6] - (P[11][1]*q0)/2) + sf[1]*(P[2][3] + P[0][3]*sf[4] + P[1][3]*sf[8] + P[3][3]*sf[6] + P[12][3]*sf[11] - P[10][3]*spp[6] - (P[11][3]*q0)/2) + spp[0]*(P[2][0] + P[0][0]*sf[4] + P[1][0]*sf[8] + P[3][0]*sf[6] + P[12][0]*sf[11] - P[10][0]*spp[6] - (P[11][0]*q0)/2) - spp[1]*(P[2][2] + P[0][2]*sf[4] + P[1][2]*sf[8] + P[3][2]*sf[6] + P[12][2]*sf[11] - P[10][2]*spp[6] - (P[11][2]*q0)/2) - (sq(q0) - sq(q1) - sq(q2) + sq(q3))*(P[2][13] + P[0][13]*sf[4] + P[1][13]*sf[8] + P[3][13]*sf[6] + P[12][13]*sf[11] - P[10][13]*spp[6] - (P[11][13]*q0)/2);
    next_p[2][7] = P[2][7] + P[0][7]*sf[4] + P[1][7]*sf[8] + P[3][7]*sf[6] + P[12][7]*sf[11] - P[10][7]*spp[6] - (P[11][7]*q0)/2 + dt*(P[2][4] + P[0][4]*sf[4] + P[1][4]*sf[8] + P[3][4]*sf[6] + P[12][4]*sf[11] - P[10][4]*spp[6] - (P[11][4]*q0)/2);
    next_p[2][8] = P[2][8] + P[0][8]*sf[4] + P[1][8]*sf[8] + P[3][8]*sf[6] + P[12][8]*sf[11] - P[10][8]*spp[6] - (P[11][8]*q0)/2 + dt*(P[2][5] + P[0][5]*sf[4] + P[1][5]*sf[8] + P[3][5]*sf[6] + P[12][5]*sf[11] - P[10][5]*spp[6] - (P[11][5]*q0)/2);
    next_p[2][9] = P[2][9] + P[0][9]*sf[4] + P[1][9]*sf[8] + P[3][9]*sf[6] + P[12][9]*sf[11] - P[10][9]*spp[6] - (P[11][9]*q0)/2 + dt*(P[2][6] + P[0][6]*sf[4] + P[1][6]*sf[8] + P[3][6]*sf[6] + P[12][6]*sf[11] - P[10][6]*spp[6] - (P[11][6]*q0)/2);
    next_p[2][10] = P[2][10] + P[0][10]*sf[4] + P[1][10]*sf[8] + P[3][10]*sf[6] + P[12][10]*sf[11] - P[10][10]*spp[6] - (P[11][10]*q0)/2;
    next_p[2][11] = P[2][11] + P[0][11]*sf[4] + P[1][11]*sf[8] + P[3][11]*sf[6] + P[12][11]*sf[11] - P[10][11]*spp[6] - (P[11][11]*q0)/2;
    next_p[2][12] = P[2][12] + P[0][12]*sf[4] + P[1][12]*sf[8] + P[3][12]*sf[6] + P[12][12]*sf[11] - P[10][12]*spp[6] - (P[11][12]*q0)/2;
    next_p[2][13] = P[2][13] + P[0][13]*sf[4] + P[1][13]*sf[8] + P[3][13]*sf[6] + P[12][13]*sf[11] - P[10][13]*spp[6] - (P[11][13]*q0)/2;
    next_p[2][14] = P[2][14] + P[0][14]*sf[4] + P[1][14]*sf[8] + P[3][14]*sf[6] + P[12][14]*sf[11] - P[10][14]*spp[6] - (P[11][14]*q0)/2;
    next_p[2][15] = P[2][15] + P[0][15]*sf[4] + P[1][15]*sf[8] + P[3][15]*sf[6] + P[12][15]*sf[11] - P[10][15]*spp[6] - (P[11][15]*q0)/2;
    next_p[2][16] = P[2][16] + P[0][16]*sf[4] + P[1][16]*sf[8] + P[3][16]*sf[6] + P[12][16]*sf[11] - P[10][16]*spp[6] - (P[11][16]*q0)/2;
    next_p[2][17] = P[2][17] + P[0][17]*sf[4] + P[1][17]*sf[8] + P[3][17]*sf[6] + P[12][17]*sf[11] - P[10][17]*spp[6] - (P[11][17]*q0)/2;
    next_p[2][18] = P[2][18] + P[0][18]*sf[4] + P[1][18]*sf[8] + P[3][18]*sf[6] + P[12][18]*sf[11] - P[10][18]*spp[6] - (P[11][18]*q0)/2;
    next_p[2][19] = P[2][19] + P[0][19]*sf[4] + P[1][19]*sf[8] + P[3][19]*sf[6] + P[12][19]*sf[11] - P[10][19]*spp[6] - (P[11][19]*q0)/2;
    next_p[2][20] = P[2][20] + P[0][20]*sf[4] + P[1][20]*sf[8] + P[3][20]*sf[6] + P[12][20]*sf[11] - P[10][20]*spp[6] - (P[11][20]*q0)/2;
    next_p[2][21] = P[2][21] + P[0][21]*sf[4] + P[1][21]*sf[8] + P[3][21]*sf[6] + P[12][21]*sf[11] - P[10][21]*spp[6] - (P[11][21]*q0)/2;
    next_p[3][0] = P[3][0] + sq[6] + P[0][0]*sf[5] + P[1][0]*sf[4] + P[2][0]*sf[7] - P[11][0]*sf[11] + P[10][0]*spp[7] - (P[12][0]*q0)/2 + sf[7]*(P[3][1] + P[0][1]*sf[5] + P[1][1]*sf[4] + P[2][1]*sf[7] - P[11][1]*sf[11] + P[10][1]*spp[7] - (P[12][1]*q0)/2) + sf[9]*(P[3][2] + P[0][2]*sf[5] + P[1][2]*sf[4] + P[2][2]*sf[7] - P[11][2]*sf[11] + P[10][2]*spp[7] - (P[12][2]*q0)/2) + sf[8]*(P[3][3] + P[0][3]*sf[5] + P[1][3]*sf[4] + P[2][3]*sf[7] - P[11][3]*sf[11] + P[10][3]*spp[7] - (P[12][3]*q0)/2) + sf[11]*(P[3][10] + P[0][10]*sf[5] + P[1][10]*sf[4] + P[2][10]*sf[7] - P[11][10]*sf[11] + P[10][10]*spp[7] - (P[12][10]*q0)/2) + spp[7]*(P[3][11] + P[0][11]*sf[5] + P[1][11]*sf[4] + P[2][11]*sf[7] - P[11][11]*sf[11] + P[10][11]*spp[7] - (P[12][11]*q0)/2) + spp[6]*(P[3][12] + P[0][12]*sf[5] + P[1][12]*sf[4] + P[2][12]*sf[7] - P[11][12]*sf[11] + P[10][12]*spp[7] - (P[12][12]*q0)/2);
    next_p[3][1] = P[3][1] + sq[4] + P[0][1]*sf[5] + P[1][1]*sf[4] + P[2][1]*sf[7] - P[11][1]*sf[11] + P[10][1]*spp[7] - (P[12][1]*q0)/2 + sf[6]*(P[3][0] + P[0][0]*sf[5] + P[1][0]*sf[4] + P[2][0]*sf[7] - P[11][0]*sf[11] + P[10][0]*spp[7] - (P[12][0]*q0)/2) + sf[5]*(P[3][2] + P[0][2]*sf[5] + P[1][2]*sf[4] + P[2][2]*sf[7] - P[11][2]*sf[11] + P[10][2]*spp[7] - (P[12][2]*q0)/2) + sf[9]*(P[3][3] + P[0][3]*sf[5] + P[1][3]*sf[4] + P[2][3]*sf[7] - P[11][3]*sf[11] + P[10][3]*spp[7] - (P[12][3]*q0)/2) + spp[6]*(P[3][11] + P[0][11]*sf[5] + P[1][11]*sf[4] + P[2][11]*sf[7] - P[11][11]*sf[11] + P[10][11]*spp[7] - (P[12][11]*q0)/2) - spp[7]*(P[3][12] + P[0][12]*sf[5] + P[1][12]*sf[4] + P[2][12]*sf[7] - P[11][12]*sf[11] + P[10][12]*spp[7] - (P[12][12]*q0)/2) - (q0*(P[3][10] + P[0][10]*sf[5] + P[1][10]*sf[4] + P[2][10]*sf[7] - P[11][10]*sf[11] + P[10][10]*spp[7] - (P[12][10]*q0)/2))/2;
    next_p[3][2] = P[3][2] + sq[3] + P[0][2]*sf[5] + P[1][2]*sf[4] + P[2][2]*sf[7] - P[11][2]*sf[11] + P[10][2]*spp[7] - (P[12][2]*q0)/2 + sf[4]*(P[3][0] + P[0][0]*sf[5] + P[1][0]*sf[4] + P[2][0]*sf[7] - P[11][0]*sf[11] + P[10][0]*spp[7] - (P[12][0]*q0)/2) + sf[8]*(P[3][1] + P[0][1]*sf[5] + P[1][1]*sf[4] + P[2][1]*sf[7] - P[11][1]*sf[11] + P[10][1]*spp[7] - (P[12][1]*q0)/2) + sf[6]*(P[3][3] + P[0][3]*sf[5] + P[1][3]*sf[4] + P[2][3]*sf[7] - P[11][3]*sf[11] + P[10][3]*spp[7] - (P[12][3]*q0)/2) + sf[11]*(P[3][12] + P[0][12]*sf[5] + P[1][12]*sf[4] + P[2][12]*sf[7] - P[11][12]*sf[11] + P[10][12]*spp[7] - (P[12][12]*q0)/2) - spp[6]*(P[3][10] + P[0][10]*sf[5] + P[1][10]*sf[4] + P[2][10]*sf[7] - P[11][10]*sf[11] + P[10][10]*spp[7] - (P[12][10]*q0)/2) - (q0*(P[3][11] + P[0][11]*sf[5] + P[1][11]*sf[4] + P[2][11]*sf[7] - P[11][11]*sf[11] + P[10][11]*spp[7] - (P[12][11]*q0)/2))/2;
    next_p[3][3] = P[3][3] + P[0][3]*sf[5] + P[1][3]*sf[4] + P[2][3]*sf[7] - P[11][3]*sf[11] + P[10][3]*spp[7] + (day_cov*sq[10])/4 + daz_cov*sq[9] - (P[12][3]*q0)/2 + sf[5]*(P[3][0] + P[0][0]*sf[5] + P[1][0]*sf[4] + P[2][0]*sf[7] - P[11][0]*sf[11] + P[10][0]*spp[7] - (P[12][0]*q0)/2) + sf[4]*(P[3][1] + P[0][1]*sf[5] + P[1][1]*sf[4] + P[2][1]*sf[7] - P[11][1]*sf[11] + P[10][1]*spp[7] - (P[12][1]*q0)/2) + sf[7]*(P[3][2] + P[0][2]*sf[5] + P[1][2]*sf[4] + P[2][2]*sf[7] - P[11][2]*sf[11] + P[10][2]*spp[7] - (P[12][2]*q0)/2) - sf[11]*(P[3][11] + P[0][11]*sf[5] + P[1][11]*sf[4] + P[2][11]*sf[7] - P[11][11]*sf[11] + P[10][11]*spp[7] - (P[12][11]*q0)/2) + spp[7]*(P[3][10] + P[0][10]*sf[5] + P[1][10]*sf[4] + P[2][10]*sf[7] - P[11][10]*sf[11] + P[10][10]*spp[7] - (P[12][10]*q0)/2) + (dax_cov*sq(q2))/4 - (q0*(P[3][12] + P[0][12]*sf[5] + P[1][12]*sf[4] + P[2][12]*sf[7] - P[11][12]*sf[11] + P[10][12]*spp[7] - (P[12][12]*q0)/2))/2;
    next_p[3][4] = P[3][4] + P[0][4]*sf[5] + P[1][4]*sf[4] + P[2][4]*sf[7] - P[11][4]*sf[11] + P[10][4]*spp[7] - (P[12][4]*q0)/2 + sf[3]*(P[3][0] + P[0][0]*sf[5] + P[1][0]*sf[4] + P[2][0]*sf[7] - P[11][0]*sf[11] + P[10][0]*spp[7] - (P[12][0]*q0)/2) + sf[1]*(P[3][1] + P[0][1]*sf[5] + P[1][1]*sf[4] + P[2][1]*sf[7] - P[11][1]*sf[11] + P[10][1]*spp[7] - (P[12][1]*q0)/2) + spp[0]*(P[3][2] + P[0][2]*sf[5] + P[1][2]*sf[4] + P[2][2]*sf[7] - P[11][2]*sf[11] + P[10][2]*spp[7] - (P[12][2]*q0)/2) - spp[2]*(P[3][3] + P[0][3]*sf[5] + P[1][3]*sf[4] + P[2][3]*sf[7] - P[11][3]*sf[11] + P[10][3]*spp[7] - (P[12][3]*q0)/2) - spp[4]*(P[3][13] + P[0][13]*sf[5] + P[1][13]*sf[4] + P[2][13]*sf[7] - P[11][13]*sf[11] + P[10][13]*spp[7] - (P[12][13]*q0)/2);
    next_p[3][5] = P[3][5] + P[0][5]*sf[5] + P[1][5]*sf[4] + P[2][5]*sf[7] - P[11][5]*sf[11] + P[10][5]*spp[7] - (P[12][5]*q0)/2 + sf[2]*(P[3][0] + P[0][0]*sf[5] + P[1][0]*sf[4] + P[2][0]*sf[7] - P[11][0]*sf[11] + P[10][0]*spp[7] - (P[12][0]*q0)/2) + sf[1]*(P[3][2] + P[0][2]*sf[5] + P[1][2]*sf[4] + P[2][2]*sf[7] - P[11][2]*sf[11] + P[10][2]*spp[7] - (P[12][2]*q0)/2) + sf[3]*(P[3][3] + P[0][3]*sf[5] + P[1][3]*sf[4] + P[2][3]*sf[7] - P[11][3]*sf[11] + P[10][3]*spp[7] - (P[12][3]*q0)/2) - spp[0]*(P[3][1] + P[0][1]*sf[5] + P[1][1]*sf[4] + P[2][1]*sf[7] - P[11][1]*sf[11] + P[10][1]*spp[7] - (P[12][1]*q0)/2) + spp[3]*(P[3][13] + P[0][13]*sf[5] + P[1][13]*sf[4] + P[2][13]*sf[7] - P[11][13]*sf[11] + P[10][13]*spp[7] - (P[12][13]*q0)/2);
    next_p[3][6] = P[3][6] + P[0][6]*sf[5] + P[1][6]*sf[4] + P[2][6]*sf[7] - P[11][6]*sf[11] + P[10][6]*spp[7] - (P[12][6]*q0)/2 + sf[2]*(P[3][1] + P[0][1]*sf[5] + P[1][1]*sf[4] + P[2][1]*sf[7] - P[11][1]*sf[11] + P[10][1]*spp[7] - (P[12][1]*q0)/2) + sf[1]*(P[3][3] + P[0][3]*sf[5] + P[1][3]*sf[4] + P[2][3]*sf[7] - P[11][3]*sf[11] + P[10][3]*spp[7] - (P[12][3]*q0)/2) + spp[0]*(P[3][0] + P[0][0]*sf[5] + P[1][0]*sf[4] + P[2][0]*sf[7] - P[11][0]*sf[11] + P[10][0]*spp[7] - (P[12][0]*q0)/2) - spp[1]*(P[3][2] + P[0][2]*sf[5] + P[1][2]*sf[4] + P[2][2]*sf[7] - P[11][2]*sf[11] + P[10][2]*spp[7] - (P[12][2]*q0)/2) - (sq(q0) - sq(q1) - sq(q2) + sq(q3))*(P[3][13] + P[0][13]*sf[5] + P[1][13]*sf[4] + P[2][13]*sf[7] - P[11][13]*sf[11] + P[10][13]*spp[7] - (P[12][13]*q0)/2);
    next_p[3][7] = P[3][7] + P[0][7]*sf[5] + P[1][7]*sf[4] + P[2][7]*sf[7] - P[11][7]*sf[11] + P[10][7]*spp[7] - (P[12][7]*q0)/2 + dt*(P[3][4] + P[0][4]*sf[5] + P[1][4]*sf[4] + P[2][4]*sf[7] - P[11][4]*sf[11] + P[10][4]*spp[7] - (P[12][4]*q0)/2);
    next_p[3][8] = P[3][8] + P[0][8]*sf[5] + P[1][8]*sf[4] + P[2][8]*sf[7] - P[11][8]*sf[11] + P[10][8]*spp[7] - (P[12][8]*q0)/2 + dt*(P[3][5] + P[0][5]*sf[5] + P[1][5]*sf[4] + P[2][5]*sf[7] - P[11][5]*sf[11] + P[10][5]*spp[7] - (P[12][5]*q0)/2);
    next_p[3][9] = P[3][9] + P[0][9]*sf[5] + P[1][9]*sf[4] + P[2][9]*sf[7] - P[11][9]*sf[11] + P[10][9]*spp[7] - (P[12][9]*q0)/2 + dt*(P[3][6] + P[0][6]*sf[5] + P[1][6]*sf[4] + P[2][6]*sf[7] - P[11][6]*sf[11] + P[10][6]*spp[7] - (P[12][6]*q0)/2);
    next_p[3][10] = P[3][10] + P[0][10]*sf[5] + P[1][10]*sf[4] + P[2][10]*sf[7] - P[11][10]*sf[11] + P[10][10]*spp[7] - (P[12][10]*q0)/2;
    next_p[3][11] = P[3][11] + P[0][11]*sf[5] + P[1][11]*sf[4] + P[2][11]*sf[7] - P[11][11]*sf[11] + P[10][11]*spp[7] - (P[12][11]*q0)/2;
    next_p[3][12] = P[3][12] + P[0][12]*sf[5] + P[1][12]*sf[4] + P[2][12]*sf[7] - P[11][12]*sf[11] + P[10][12]*spp[7] - (P[12][12]*q0)/2;
    next_p[3][13] = P[3][13] + P[0][13]*sf[5] + P[1][13]*sf[4] + P[2][13]*sf[7] - P[11][13]*sf[11] + P[10][13]*spp[7] - (P[12][13]*q0)/2;
    next_p[3][14] = P[3][14] + P[0][14]*sf[5] + P[1][14]*sf[4] + P[2][14]*sf[7] - P[11][14]*sf[11] + P[10][14]*spp[7] - (P[12][14]*q0)/2;
    next_p[3][15] = P[3][15] + P[0][15]*sf[5] + P[1][15]*sf[4] + P[2][15]*sf[7] - P[11][15]*sf[11] + P[10][15]*spp[7] - (P[12][15]*q0)/2;
    next_p[3][16] = P[3][16] + P[0][16]*sf[5] + P[1][16]*sf[4] + P[2][16]*sf[7] - P[11][16]*sf[11] + P[10][16]*spp[7] - (P[12][16]*q0)/2;
    next_p[3][17] = P[3][17] + P[0][17]*sf[5] + P[1][17]*sf[4] + P[2][17]*sf[7] - P[11][17]*sf[11] + P[10][17]*spp[7] - (P[12][17]*q0)/2;
    next_p[3][18] = P[3][18] + P[0][18]*sf[5] + P[1][18]*sf[4] + P[2][18]*sf[7] - P[11][18]*sf[11] + P[10][18]*spp[7] - (P[12][18]*q0)/2;
    next_p[3][19] = P[3][19] + P[0][19]*sf[5] + P[1][19]*sf[4] + P[2][19]*sf[7] - P[11][19]*sf[11] + P[10][19]*spp[7] - (P[12][19]*q0)/2;
    next_p[3][20] = P[3][20] + P[0][20]*sf[5] + P[1][20]*sf[4] + P[2][20]*sf[7] - P[11][20]*sf[11] + P[10][20]*spp[7] - (P[12][20]*q0)/2;
    next_p[3][21] = P[3][21] + P[0][21]*sf[5] + P[1][21]*sf[4] + P[2][21]*sf[7] - P[11][21]*sf[11] + P[10][21]*spp[7] - (P[12][21]*q0)/2;
    next_p[4][0] = P[4][0] + P[0][0]*sf[3] + P[1][0]*sf[1] + P[2][0]*spp[0] - P[3][0]*spp[2] - P[13][0]*spp[4] + sf[7]*(P[4][1] + P[0][1]*sf[3] + P[1][1]*sf[1] + P[2][1]*spp[0] - P[3][1]*spp[2] - P[13][1]*spp[4]) + sf[9]*(P[4][2] + P[0][2]*sf[3] + P[1][2]*sf[1] + P[2][2]*spp[0] - P[3][2]*spp[2] - P[13][2]*spp[4]) + sf[8]*(P[4][3] + P[0][3]*sf[3] + P[1][3]*sf[1] + P[2][3]*spp[0] - P[3][3]*spp[2] - P[13][3]*spp[4]) + sf[11]*(P[4][10] + P[0][10]*sf[3] + P[1][10]*sf[1] + P[2][10]*spp[0] - P[3][10]*spp[2] - P[13][10]*spp[4]) + spp[7]*(P[4][11] + P[0][11]*sf[3] + P[1][11]*sf[1] + P[2][11]*spp[0] - P[3][11]*spp[2] - P[13][11]*spp[4]) + spp[6]*(P[4][12] + P[0][12]*sf[3] + P[1][12]*sf[1] + P[2][12]*spp[0] - P[3][12]*spp[2] - P[13][12]*spp[4]);
    next_p[4][1] = P[4][1] + P[0][1]*sf[3] + P[1][1]*sf[1] + P[2][1]*spp[0] - P[3][1]*spp[2] - P[13][1]*spp[4] + sf[6]*(P[4][0] + P[0][0]*sf[3] + P[1][0]*sf[1] + P[2][0]*spp[0] - P[3][0]*spp[2] - P[13][0]*spp[4]) + sf[5]*(P[4][2] + P[0][2]*sf[3] + P[1][2]*sf[1] + P[2][2]*spp[0] - P[3][2]*spp[2] - P[13][2]*spp[4]) + sf[9]*(P[4][3] + P[0][3]*sf[3] + P[1][3]*sf[1] + P[2][3]*spp[0] - P[3][3]*spp[2] - P[13][3]*spp[4]) + spp[6]*(P[4][11] + P[0][11]*sf[3] + P[1][11]*sf[1] + P[2][11]*spp[0] - P[3][11]*spp[2] - P[13][11]*spp[4]) - spp[7]*(P[4][12] + P[0][12]*sf[3] + P[1][12]*sf[1] + P[2][12]*spp[0] - P[3][12]*spp[2] - P[13][12]*spp[4]) - (q0*(P[4][10] + P[0][10]*sf[3] + P[1][10]*sf[1] + P[2][10]*spp[0] - P[3][10]*spp[2] - P[13][10]*spp[4]))/2;
    next_p[4][2] = P[4][2] + P[0][2]*sf[3] + P[1][2]*sf[1] + P[2][2]*spp[0] - P[3][2]*spp[2] - P[13][2]*spp[4] + sf[4]*(P[4][0] + P[0][0]*sf[3] + P[1][0]*sf[1] + P[2][0]*spp[0] - P[3][0]*spp[2] - P[13][0]*spp[4]) + sf[8]*(P[4][1] + P[0][1]*sf[3] + P[1][1]*sf[1] + P[2][1]*spp[0] - P[3][1]*spp[2] - P[13][1]*spp[4]) + sf[6]*(P[4][3] + P[0][3]*sf[3] + P[1][3]*sf[1] + P[2][3]*spp[0] - P[3][3]*spp[2] - P[13][3]*spp[4]) + sf[11]*(P[4][12] + P[0][12]*sf[3] + P[1][12]*sf[1] + P[2][12]*spp[0] - P[3][12]*spp[2] - P[13][12]*spp[4]) - spp[6]*(P[4][10] + P[0][10]*sf[3] + P[1][10]*sf[1] + P[2][10]*spp[0] - P[3][10]*spp[2] - P[13][10]*spp[4]) - (q0*(P[4][11] + P[0][11]*sf[3] + P[1][11]*sf[1] + P[2][11]*spp[0] - P[3][11]*spp[2] - P[13][11]*spp[4]))/2;
    next_p[4][3] = P[4][3] + P[0][3]*sf[3] + P[1][3]*sf[1] + P[2][3]*spp[0] - P[3][3]*spp[2] - P[13][3]*spp[4] + sf[5]*(P[4][0] + P[0][0]*sf[3] + P[1][0]*sf[1] + P[2][0]*spp[0] - P[3][0]*spp[2] - P[13][0]*spp[4]) + sf[4]*(P[4][1] + P[0][1]*sf[3] + P[1][1]*sf[1] + P[2][1]*spp[0] - P[3][1]*spp[2] - P[13][1]*spp[4]) + sf[7]*(P[4][2] + P[0][2]*sf[3] + P[1][2]*sf[1] + P[2][2]*spp[0] - P[3][2]*spp[2] - P[13][2]*spp[4]) - sf[11]*(P[4][11] + P[0][11]*sf[3] + P[1][11]*sf[1] + P[2][11]*spp[0] - P[3][11]*spp[2] - P[13][11]*spp[4]) + spp[7]*(P[4][10] + P[0][10]*sf[3] + P[1][10]*sf[1] + P[2][10]*spp[0] - P[3][10]*spp[2] - P[13][10]*spp[4]) - (q0*(P[4][12] + P[0][12]*sf[3] + P[1][12]*sf[1] + P[2][12]*spp[0] - P[3][12]*spp[2] - P[13][12]*spp[4]))/2;
    next_p[4][4] = P[4][4] + P[0][4]*sf[3] + P[1][4]*sf[1] + P[2][4]*spp[0] - P[3][4]*spp[2] - P[13][4]*spp[4] + dvy_cov*sq(sg[7] - 2*q0*q3) + dvz_cov*sq(sg[6] + 2*q0*q2) + sf[3]*(P[4][0] + P[0][0]*sf[3] + P[1][0]*sf[1] + P[2][0]*spp[0] - P[3][0]*spp[2] - P[13][0]*spp[4]) + sf[1]*(P[4][1] + P[0][1]*sf[3] + P[1][1]*sf[1] + P[2][1]*spp[0] - P[3][1]*spp[2] - P[13][1]*spp[4]) + spp[0]*(P[4][2] + P[0][2]*sf[3] + P[1][2]*sf[1] + P[2][2]*spp[0] - P[3][2]*spp[2] - P[13][2]*spp[4]) - spp[2]*(P[4][3] + P[0][3]*sf[3] + P[1][3]*sf[1] + P[2][3]*spp[0] - P[3][3]*spp[2] - P[13][3]*spp[4]) - spp[4]*(P[4][13] + P[0][13]*sf[3] + P[1][13]*sf[1] + P[2][13]*spp[0] - P[3][13]*spp[2] - P[13][13]*spp[4]) + dvx_cov*sq(sg[1] + sg[2] - sg[3] - sg[4]);
    next_p[4][5] = P[4][5] + sq[2] + P[0][5]*sf[3] + P[1][5]*sf[1] + P[2][5]*spp[0] - P[3][5]*spp[2] - P[13][5]*spp[4] + sf[2]*(P[4][0] + P[0][0]*sf[3] + P[1][0]*sf[1] + P[2][0]*spp[0] - P[3][0]*spp[2] - P[13][0]*spp[4]) + sf[1]*(P[4][2] + P[0][2]*sf[3] + P[1][2]*sf[1] + P[2][2]*spp[0] - P[3][2]*spp[2] - P[13][2]*spp[4]) + sf[3]*(P[4][3] + P[0][3]*sf[3] + P[1][3]*sf[1] + P[2][3]*spp[0] - P[3][3]*spp[2] - P[13][3]*spp[4]) - spp[0]*(P[4][1] + P[0][1]*sf[3] + P[1][1]*sf[1] + P[2][1]*spp[0] - P[3][1]*spp[2] - P[13][1]*spp[4]) + spp[3]*(P[4][13] + P[0][13]*sf[3] + P[1][13]*sf[1] + P[2][13]*spp[0] - P[3][13]*spp[2] - P[13][13]*spp[4]);
    next_p[4][6] = P[4][6] + sq[1] + P[0][6]*sf[3] + P[1][6]*sf[1] + P[2][6]*spp[0] - P[3][6]*spp[2] - P[13][6]*spp[4] + sf[2]*(P[4][1] + P[0][1]*sf[3] + P[1][1]*sf[1] + P[2][1]*spp[0] - P[3][1]*spp[2] - P[13][1]*spp[4]) + sf[1]*(P[4][3] + P[0][3]*sf[3] + P[1][3]*sf[1] + P[2][3]*spp[0] - P[3][3]*spp[2] - P[13][3]*spp[4]) + spp[0]*(P[4][0] + P[0][0]*sf[3] + P[1][0]*sf[1] + P[2][0]*spp[0] - P[3][0]*spp[2] - P[13][0]*spp[4]) - spp[1]*(P[4][2] + P[0][2]*sf[3] + P[1][2]*sf[1] + P[2][2]*spp[0] - P[3][2]*spp[2] - P[13][2]*spp[4]) - (sq(q0) - sq(q1) - sq(q2) + sq(q3))*(P[4][13] + P[0][13]*sf[3] + P[1][13]*sf[1] + P[2][13]*spp[0] - P[3][13]*spp[2] - P[13][13]*spp[4]);
    next_p[4][7] = P[4][7] + P[0][7]*sf[3] + P[1][7]*sf[1] + P[2][7]*spp[0] - P[3][7]*spp[2] - P[13][7]*spp[4] + dt*(P[4][4] + P[0][4]*sf[3] + P[1][4]*sf[1] + P[2][4]*spp[0] - P[3][4]*spp[2] - P[13][4]*spp[4]);
    next_p[4][8] = P[4][8] + P[0][8]*sf[3] + P[1][8]*sf[1] + P[2][8]*spp[0] - P[3][8]*spp[2] - P[13][8]*spp[4] + dt*(P[4][5] + P[0][5]*sf[3] + P[1][5]*sf[1] + P[2][5]*spp[0] - P[3][5]*spp[2] - P[13][5]*spp[4]);
    next_p[4][9] = P[4][9] + P[0][9]*sf[3] + P[1][9]*sf[1] + P[2][9]*spp[0] - P[3][9]*spp[2] - P[13][9]*spp[4] + dt*(P[4][6] + P[0][6]*sf[3] + P[1][6]*sf[1] + P[2][6]*spp[0] - P[3][6]*spp[2] - P[13][6]*spp[4]);
    next_p[4][10] = P[4][10] + P[0][10]*sf[3] + P[1][10]*sf[1] + P[2][10]*spp[0] - P[3][10]*spp[2] - P[13][10]*spp[4];
    next_p[4][11] = P[4][11] + P[0][11]*sf[3] + P[1][11]*sf[1] + P[2][11]*spp[0] - P[3][11]*spp[2] - P[13][11]*spp[4];
    next_p[4][12] = P[4][12] + P[0][12]*sf[3] + P[1][12]*sf[1] + P[2][12]*spp[0] - P[3][12]*spp[2] - P[13][12]*spp[4];
    next_p[4][13] = P[4][13] + P[0][13]*sf[3] + P[1][13]*sf[1] + P[2][13]*spp[0] - P[3][13]*spp[2] - P[13][13]*spp[4];
    next_p[4][14] = P[4][14] + P[0][14]*sf[3] + P[1][14]*sf[1] + P[2][14]*spp[0] - P[3][14]*spp[2] - P[13][14]*spp[4];
    next_p[4][15] = P[4][15] + P[0][15]*sf[3] + P[1][15]*sf[1] + P[2][15]*spp[0] - P[3][15]*spp[2] - P[13][15]*spp[4];
    next_p[4][16] = P[4][16] + P[0][16]*sf[3] + P[1][16]*sf[1] + P[2][16]*spp[0] - P[3][16]*spp[2] - P[13][16]*spp[4];
    next_p[4][17] = P[4][17] + P[0][17]*sf[3] + P[1][17]*sf[1] + P[2][17]*spp[0] - P[3][17]*spp[2] - P[13][17]*spp[4];
    next_p[4][18] = P[4][18] + P[0][18]*sf[3] + P[1][18]*sf[1] + P[2][18]*spp[0] - P[3][18]*spp[2] - P[13][18]*spp[4];
    next_p[4][19] = P[4][19] + P[0][19]*sf[3] + P[1][19]*sf[1] + P[2][19]*spp[0] - P[3][19]*spp[2] - P[13][19]*spp[4];
    next_p[4][20] = P[4][20] + P[0][20]*sf[3] + P[1][20]*sf[1] + P[2][20]*spp[0] - P[3][20]*spp[2] - P[13][20]*spp[4];
    next_p[4][21] = P[4][21] + P[0][21]*sf[3] + P[1][21]*sf[1] + P[2][21]*spp[0] - P[3][21]*spp[2] - P[13][21]*spp[4];
    next_p[5][0] = P[5][0] + P[0][0]*sf[2] + P[2][0]*sf[1] + P[3][0]*sf[3] - P[1][0]*spp[0] + P[13][0]*spp[3] + sf[7]*(P[5][1] + P[0][1]*sf[2] + P[2][1]*sf[1] + P[3][1]*sf[3] - P[1][1]*spp[0] + P[13][1]*spp[3]) + sf[9]*(P[5][2] + P[0][2]*sf[2] + P[2][2]*sf[1] + P[3][2]*sf[3] - P[1][2]*spp[0] + P[13][2]*spp[3]) + sf[8]*(P[5][3] + P[0][3]*sf[2] + P[2][3]*sf[1] + P[3][3]*sf[3] - P[1][3]*spp[0] + P[13][3]*spp[3]) + sf[11]*(P[5][10] + P[0][10]*sf[2] + P[2][10]*sf[1] + P[3][10]*sf[3] - P[1][10]*spp[0] + P[13][10]*spp[3]) + spp[7]*(P[5][11] + P[0][11]*sf[2] + P[2][11]*sf[1] + P[3][11]*sf[3] - P[1][11]*spp[0] + P[13][11]*spp[3]) + spp[6]*(P[5][12] + P[0][12]*sf[2] + P[2][12]*sf[1] + P[3][12]*sf[3] - P[1][12]*spp[0] + P[13][12]*spp[3]);
    next_p[5][1] = P[5][1] + P[0][1]*sf[2] + P[2][1]*sf[1] + P[3][1]*sf[3] - P[1][1]*spp[0] + P[13][1]*spp[3] + sf[6]*(P[5][0] + P[0][0]*sf[2] + P[2][0]*sf[1] + P[3][0]*sf[3] - P[1][0]*spp[0] + P[13][0]*spp[3]) + sf[5]*(P[5][2] + P[0][2]*sf[2] + P[2][2]*sf[1] + P[3][2]*sf[3] - P[1][2]*spp[0] + P[13][2]*spp[3]) + sf[9]*(P[5][3] + P[0][3]*sf[2] + P[2][3]*sf[1] + P[3][3]*sf[3] - P[1][3]*spp[0] + P[13][3]*spp[3]) + spp[6]*(P[5][11] + P[0][11]*sf[2] + P[2][11]*sf[1] + P[3][11]*sf[3] - P[1][11]*spp[0] + P[13][11]*spp[3]) - spp[7]*(P[5][12] + P[0][12]*sf[2] + P[2][12]*sf[1] + P[3][12]*sf[3] - P[1][12]*spp[0] + P[13][12]*spp[3]) - (q0*(P[5][10] + P[0][10]*sf[2] + P[2][10]*sf[1] + P[3][10]*sf[3] - P[1][10]*spp[0] + P[13][10]*spp[3]))/2;
    next_p[5][2] = P[5][2] + P[0][2]*sf[2] + P[2][2]*sf[1] + P[3][2]*sf[3] - P[1][2]*spp[0] + P[13][2]*spp[3] + sf[4]*(P[5][0] + P[0][0]*sf[2] + P[2][0]*sf[1] + P[3][0]*sf[3] - P[1][0]*spp[0] + P[13][0]*spp[3]) + sf[8]*(P[5][1] + P[0][1]*sf[2] + P[2][1]*sf[1] + P[3][1]*sf[3] - P[1][1]*spp[0] + P[13][1]*spp[3]) + sf[6]*(P[5][3] + P[0][3]*sf[2] + P[2][3]*sf[1] + P[3][3]*sf[3] - P[1][3]*spp[0] + P[13][3]*spp[3]) + sf[11]*(P[5][12] + P[0][12]*sf[2] + P[2][12]*sf[1] + P[3][12]*sf[3] - P[1][12]*spp[0] + P[13][12]*spp[3]) - spp[6]*(P[5][10] + P[0][10]*sf[2] + P[2][10]*sf[1] + P[3][10]*sf[3] - P[1][10]*spp[0] + P[13][10]*spp[3]) - (q0*(P[5][11] + P[0][11]*sf[2] + P[2][11]*sf[1] + P[3][11]*sf[3] - P[1][11]*spp[0] + P[13][11]*spp[3]))/2;
    next_p[5][3] = P[5][3] + P[0][3]*sf[2] + P[2][3]*sf[1] + P[3][3]*sf[3] - P[1][3]*spp[0] + P[13][3]*spp[3] + sf[5]*(P[5][0] + P[0][0]*sf[2] + P[2][0]*sf[1] + P[3][0]*sf[3] - P[1][0]*spp[0] + P[13][0]*spp[3]) + sf[4]*(P[5][1] + P[0][1]*sf[2] + P[2][1]*sf[1] + P[3][1]*sf[3] - P[1][1]*spp[0] + P[13][1]*spp[3]) + sf[7]*(P[5][2] + P[0][2]*sf[2] + P[2][2]*sf[1] + P[3][2]*sf[3] - P[1][2]*spp[0] + P[13][2]*spp[3]) - sf[11]*(P[5][11] + P[0][11]*sf[2] + P[2][11]*sf[1] + P[3][11]*sf[3] - P[1][11]*spp[0] + P[13][11]*spp[3]) + spp[7]*(P[5][10] + P[0][10]*sf[2] + P[2][10]*sf[1] + P[3][10]*sf[3] - P[1][10]*spp[0] + P[13][10]*spp[3]) - (q0*(P[5][12] + P[0][12]*sf[2] + P[2][12]*sf[1] + P[3][12]*sf[3] - P[1][12]*spp[0] + P[13][12]*spp[3]))/2;
    next_p[5][4] = P[5][4] + sq[2] + P[0][4]*sf[2] + P[2][4]*sf[1] + P[3][4]*sf[3] - P[1][4]*spp[0] + P[13][4]*spp[3] + sf[3]*(P[5][0] + P[0][0]*sf[2] + P[2][0]*sf[1] + P[3][0]*sf[3] - P[1][0]*spp[0] + P[13][0]*spp[3]) + sf[1]*(P[5][1] + P[0][1]*sf[2] + P[2][1]*sf[1] + P[3][1]*sf[3] - P[1][1]*spp[0] + P[13][1]*spp[3]) + spp[0]*(P[5][2] + P[0][2]*sf[2] + P[2][2]*sf[1] + P[3][2]*sf[3] - P[1][2]*spp[0] + P[13][2]*spp[3]) - spp[2]*(P[5][3] + P[0][3]*sf[2] + P[2][3]*sf[1] + P[3][3]*sf[3] - P[1][3]*spp[0] + P[13][3]*spp[3]) - spp[4]*(P[5][13] + P[0][13]*sf[2] + P[2][13]*sf[1] + P[3][13]*sf[3] - P[1][13]*spp[0] + P[13][13]*spp[3]);
    next_p[5][5] = P[5][5] + P[0][5]*sf[2] + P[2][5]*sf[1] + P[3][5]*sf[3] - P[1][5]*spp[0] + P[13][5]*spp[3] + dvx_cov*sq(sg[7] + 2*q0*q3) + dvz_cov*sq(sg[5] - 2*q0*q1) + sf[2]*(P[5][0] + P[0][0]*sf[2] + P[2][0]*sf[1] + P[3][0]*sf[3] - P[1][0]*spp[0] + P[13][0]*spp[3]) + sf[1]*(P[5][2] + P[0][2]*sf[2] + P[2][2]*sf[1] + P[3][2]*sf[3] - P[1][2]*spp[0] + P[13][2]*spp[3]) + sf[3]*(P[5][3] + P[0][3]*sf[2] + P[2][3]*sf[1] + P[3][3]*sf[3] - P[1][3]*spp[0] + P[13][3]*spp[3]) - spp[0]*(P[5][1] + P[0][1]*sf[2] + P[2][1]*sf[1] + P[3][1]*sf[3] - P[1][1]*spp[0] + P[13][1]*spp[3]) + spp[3]*(P[5][13] + P[0][13]*sf[2] + P[2][13]*sf[1] + P[3][13]*sf[3] - P[1][13]*spp[0] + P[13][13]*spp[3]) + dvy_cov*sq(sg[1] - sg[2] + sg[3] - sg[4]);
    next_p[5][6] = P[5][6] + sq[0] + P[0][6]*sf[2] + P[2][6]*sf[1] + P[3][6]*sf[3] - P[1][6]*spp[0] + P[13][6]*spp[3] + sf[2]*(P[5][1] + P[0][1]*sf[2] + P[2][1]*sf[1] + P[3][1]*sf[3] - P[1][1]*spp[0] + P[13][1]*spp[3]) + sf[1]*(P[5][3] + P[0][3]*sf[2] + P[2][3]*sf[1] + P[3][3]*sf[3] - P[1][3]*spp[0] + P[13][3]*spp[3]) + spp[0]*(P[5][0] + P[0][0]*sf[2] + P[2][0]*sf[1] + P[3][0]*sf[3] - P[1][0]*spp[0] + P[13][0]*spp[3]) - spp[1]*(P[5][2] + P[0][2]*sf[2] + P[2][2]*sf[1] + P[3][2]*sf[3] - P[1][2]*spp[0] + P[13][2]*spp[3]) - (sq(q0) - sq(q1) - sq(q2) + sq(q3))*(P[5][13] + P[0][13]*sf[2] + P[2][13]*sf[1] + P[3][13]*sf[3] - P[1][13]*spp[0] + P[13][13]*spp[3]);
    next_p[5][7] = P[5][7] + P[0][7]*sf[2] + P[2][7]*sf[1] + P[3][7]*sf[3] - P[1][7]*spp[0] + P[13][7]*spp[3] + dt*(P[5][4] + P[0][4]*sf[2] + P[2][4]*sf[1] + P[3][4]*sf[3] - P[1][4]*spp[0] + P[13][4]*spp[3]);
    next_p[5][8] = P[5][8] + P[0][8]*sf[2] + P[2][8]*sf[1] + P[3][8]*sf[3] - P[1][8]*spp[0] + P[13][8]*spp[3] + dt*(P[5][5] + P[0][5]*sf[2] + P[2][5]*sf[1] + P[3][5]*sf[3] - P[1][5]*spp[0] + P[13][5]*spp[3]);
    next_p[5][9] = P[5][9] + P[0][9]*sf[2] + P[2][9]*sf[1] + P[3][9]*sf[3] - P[1][9]*spp[0] + P[13][9]*spp[3] + dt*(P[5][6] + P[0][6]*sf[2] + P[2][6]*sf[1] + P[3][6]*sf[3] - P[1][6]*spp[0] + P[13][6]*spp[3]);
    next_p[5][10] = P[5][10] + P[0][10]*sf[2] + P[2][10]*sf[1] + P[3][10]*sf[3] - P[1][10]*spp[0] + P[13][10]*spp[3];
    next_p[5][11] = P[5][11] + P[0][11]*sf[2] + P[2][11]*sf[1] + P[3][11]*sf[3] - P[1][11]*spp[0] + P[13][11]*spp[3];
    next_p[5][12] = P[5][12] + P[0][12]*sf[2] + P[2][12]*sf[1] + P[3][12]*sf[3] - P[1][12]*spp[0] + P[13][12]*spp[3];
    next_p[5][13] = P[5][13] + P[0][13]*sf[2] + P[2][13]*sf[1] + P[3][13]*sf[3] - P[1][13]*spp[0] + P[13][13]*spp[3];
    next_p[5][14] = P[5][14] + P[0][14]*sf[2] + P[2][14]*sf[1] + P[3][14]*sf[3] - P[1][14]*spp[0] + P[13][14]*spp[3];
    next_p[5][15] = P[5][15] + P[0][15]*sf[2] + P[2][15]*sf[1] + P[3][15]*sf[3] - P[1][15]*spp[0] + P[13][15]*spp[3];
    next_p[5][16] = P[5][16] + P[0][16]*sf[2] + P[2][16]*sf[1] + P[3][16]*sf[3] - P[1][16]*spp[0] + P[13][16]*spp[3];
    next_p[5][17] = P[5][17] + P[0][17]*sf[2] + P[2][17]*sf[1] + P[3][17]*sf[3] - P[1][17]*spp[0] + P[13][17]*spp[3];
    next_p[5][18] = P[5][18] + P[0][18]*sf[2] + P[2][18]*sf[1] + P[3][18]*sf[3] - P[1][18]*spp[0] + P[13][18]*spp[3];
    next_p[5][19] = P[5][19] + P[0][19]*sf[2] + P[2][19]*sf[1] + P[3][19]*sf[3] - P[1][19]*spp[0] + P[13][19]*spp[3];
    next_p[5][20] = P[5][20] + P[0][20]*sf[2] + P[2][20]*sf[1] + P[3][20]*sf[3] - P[1][20]*spp[0] + P[13][20]*spp[3];
    next_p[5][21] = P[5][21] + P[0][21]*sf[2] + P[2][21]*sf[1] + P[3][21]*sf[3] - P[1][21]*spp[0] + P[13][21]*spp[3];
    next_p[6][0] = P[6][0] + P[1][0]*sf[2] + P[3][0]*sf[1] + P[0][0]*spp[0] - P[2][0]*spp[1] - P[13][0]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + sf[7]*(P[6][1] + P[1][1]*sf[2] + P[3][1]*sf[1] + P[0][1]*spp[0] - P[2][1]*spp[1] - P[13][1]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + sf[9]*(P[6][2] + P[1][2]*sf[2] + P[3][2]*sf[1] + P[0][2]*spp[0] - P[2][2]*spp[1] - P[13][2]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + sf[8]*(P[6][3] + P[1][3]*sf[2] + P[3][3]*sf[1] + P[0][3]*spp[0] - P[2][3]*spp[1] - P[13][3]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + sf[11]*(P[6][10] + P[1][10]*sf[2] + P[3][10]*sf[1] + P[0][10]*spp[0] - P[2][10]*spp[1] - P[13][10]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + spp[7]*(P[6][11] + P[1][11]*sf[2] + P[3][11]*sf[1] + P[0][11]*spp[0] - P[2][11]*spp[1] - P[13][11]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + spp[6]*(P[6][12] + P[1][12]*sf[2] + P[3][12]*sf[1] + P[0][12]*spp[0] - P[2][12]*spp[1] - P[13][12]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)));
    next_p[6][1] = P[6][1] + P[1][1]*sf[2] + P[3][1]*sf[1] + P[0][1]*spp[0] - P[2][1]*spp[1] - P[13][1]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + sf[6]*(P[6][0] + P[1][0]*sf[2] + P[3][0]*sf[1] + P[0][0]*spp[0] - P[2][0]*spp[1] - P[13][0]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + sf[5]*(P[6][2] + P[1][2]*sf[2] + P[3][2]*sf[1] + P[0][2]*spp[0] - P[2][2]*spp[1] - P[13][2]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + sf[9]*(P[6][3] + P[1][3]*sf[2] + P[3][3]*sf[1] + P[0][3]*spp[0] - P[2][3]*spp[1] - P[13][3]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + spp[6]*(P[6][11] + P[1][11]*sf[2] + P[3][11]*sf[1] + P[0][11]*spp[0] - P[2][11]*spp[1] - P[13][11]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - spp[7]*(P[6][12] + P[1][12]*sf[2] + P[3][12]*sf[1] + P[0][12]*spp[0] - P[2][12]*spp[1] - P[13][12]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - (q0*(P[6][10] + P[1][10]*sf[2] + P[3][10]*sf[1] + P[0][10]*spp[0] - P[2][10]*spp[1] - P[13][10]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))))/2;
    next_p[6][2] = P[6][2] + P[1][2]*sf[2] + P[3][2]*sf[1] + P[0][2]*spp[0] - P[2][2]*spp[1] - P[13][2]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + sf[4]*(P[6][0] + P[1][0]*sf[2] + P[3][0]*sf[1] + P[0][0]*spp[0] - P[2][0]*spp[1] - P[13][0]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + sf[8]*(P[6][1] + P[1][1]*sf[2] + P[3][1]*sf[1] + P[0][1]*spp[0] - P[2][1]*spp[1] - P[13][1]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + sf[6]*(P[6][3] + P[1][3]*sf[2] + P[3][3]*sf[1] + P[0][3]*spp[0] - P[2][3]*spp[1] - P[13][3]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + sf[11]*(P[6][12] + P[1][12]*sf[2] + P[3][12]*sf[1] + P[0][12]*spp[0] - P[2][12]*spp[1] - P[13][12]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - spp[6]*(P[6][10] + P[1][10]*sf[2] + P[3][10]*sf[1] + P[0][10]*spp[0] - P[2][10]*spp[1] - P[13][10]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - (q0*(P[6][11] + P[1][11]*sf[2] + P[3][11]*sf[1] + P[0][11]*spp[0] - P[2][11]*spp[1] - P[13][11]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))))/2;
    next_p[6][3] = P[6][3] + P[1][3]*sf[2] + P[3][3]*sf[1] + P[0][3]*spp[0] - P[2][3]*spp[1] - P[13][3]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + sf[5]*(P[6][0] + P[1][0]*sf[2] + P[3][0]*sf[1] + P[0][0]*spp[0] - P[2][0]*spp[1] - P[13][0]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + sf[4]*(P[6][1] + P[1][1]*sf[2] + P[3][1]*sf[1] + P[0][1]*spp[0] - P[2][1]*spp[1] - P[13][1]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + sf[7]*(P[6][2] + P[1][2]*sf[2] + P[3][2]*sf[1] + P[0][2]*spp[0] - P[2][2]*spp[1] - P[13][2]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - sf[11]*(P[6][11] + P[1][11]*sf[2] + P[3][11]*sf[1] + P[0][11]*spp[0] - P[2][11]*spp[1] - P[13][11]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + spp[7]*(P[6][10] + P[1][10]*sf[2] + P[3][10]*sf[1] + P[0][10]*spp[0] - P[2][10]*spp[1] - P[13][10]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - (q0*(P[6][12] + P[1][12]*sf[2] + P[3][12]*sf[1] + P[0][12]*spp[0] - P[2][12]*spp[1] - P[13][12]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))))/2;
    next_p[6][4] = P[6][4] + sq[1] + P[1][4]*sf[2] + P[3][4]*sf[1] + P[0][4]*spp[0] - P[2][4]*spp[1] - P[13][4]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + sf[3]*(P[6][0] + P[1][0]*sf[2] + P[3][0]*sf[1] + P[0][0]*spp[0] - P[2][0]*spp[1] - P[13][0]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + sf[1]*(P[6][1] + P[1][1]*sf[2] + P[3][1]*sf[1] + P[0][1]*spp[0] - P[2][1]*spp[1] - P[13][1]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + spp[0]*(P[6][2] + P[1][2]*sf[2] + P[3][2]*sf[1] + P[0][2]*spp[0] - P[2][2]*spp[1] - P[13][2]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - spp[2]*(P[6][3] + P[1][3]*sf[2] + P[3][3]*sf[1] + P[0][3]*spp[0] - P[2][3]*spp[1] - P[13][3]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - spp[4]*(P[6][13] + P[1][13]*sf[2] + P[3][13]*sf[1] + P[0][13]*spp[0] - P[2][13]*spp[1] - P[13][13]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)));
    next_p[6][5] = P[6][5] + sq[0] + P[1][5]*sf[2] + P[3][5]*sf[1] + P[0][5]*spp[0] - P[2][5]*spp[1] - P[13][5]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + sf[2]*(P[6][0] + P[1][0]*sf[2] + P[3][0]*sf[1] + P[0][0]*spp[0] - P[2][0]*spp[1] - P[13][0]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + sf[1]*(P[6][2] + P[1][2]*sf[2] + P[3][2]*sf[1] + P[0][2]*spp[0] - P[2][2]*spp[1] - P[13][2]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + sf[3]*(P[6][3] + P[1][3]*sf[2] + P[3][3]*sf[1] + P[0][3]*spp[0] - P[2][3]*spp[1] - P[13][3]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - spp[0]*(P[6][1] + P[1][1]*sf[2] + P[3][1]*sf[1] + P[0][1]*spp[0] - P[2][1]*spp[1] - P[13][1]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + spp[3]*(P[6][13] + P[1][13]*sf[2] + P[3][13]*sf[1] + P[0][13]*spp[0] - P[2][13]*spp[1] - P[13][13]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)));
    next_p[6][6] = P[6][6] + P[1][6]*sf[2] + P[3][6]*sf[1] + P[0][6]*spp[0] - P[2][6]*spp[1] - P[13][6]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + dvx_cov*sq(sg[6] - 2*q0*q2) + dvy_cov*sq(sg[5] + 2*q0*q1) - spp[5]*(P[6][13] + P[1][13]*sf[2] + P[3][13]*sf[1] + P[0][13]*spp[0] - P[2][13]*spp[1] - P[13][13]*spp[5]) + sf[2]*(P[6][1] + P[1][1]*sf[2] + P[3][1]*sf[1] + P[0][1]*spp[0] - P[2][1]*spp[1] - P[13][1]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + sf[1]*(P[6][3] + P[1][3]*sf[2] + P[3][3]*sf[1] + P[0][3]*spp[0] - P[2][3]*spp[1] - P[13][3]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + spp[0]*(P[6][0] + P[1][0]*sf[2] + P[3][0]*sf[1] + P[0][0]*spp[0] - P[2][0]*spp[1] - P[13][0]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - spp[1]*(P[6][2] + P[1][2]*sf[2] + P[3][2]*sf[1] + P[0][2]*spp[0] - P[2][2]*spp[1] - P[13][2]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + dvz_cov*sq(sg[1] - sg[2] - sg[3] + sg[4]);
    next_p[6][7] = P[6][7] + P[1][7]*sf[2] + P[3][7]*sf[1] + P[0][7]*spp[0] - P[2][7]*spp[1] - P[13][7]*spp[5] + dt*(P[6][4] + P[1][4]*sf[2] + P[3][4]*sf[1] + P[0][4]*spp[0] - P[2][4]*spp[1] - P[13][4]*spp[5]);
    next_p[6][8] = P[6][8] + P[1][8]*sf[2] + P[3][8]*sf[1] + P[0][8]*spp[0] - P[2][8]*spp[1] - P[13][8]*spp[5] + dt*(P[6][5] + P[1][5]*sf[2] + P[3][5]*sf[1] + P[0][5]*spp[0] - P[2][5]*spp[1] - P[13][5]*spp[5]);
    next_p[6][9] = P[6][9] + P[1][9]*sf[2] + P[3][9]*sf[1] + P[0][9]*spp[0] - P[2][9]*spp[1] - P[13][9]*spp[5] + dt*(P[6][6] + P[1][6]*sf[2] + P[3][6]*sf[1] + P[0][6]*spp[0] - P[2][6]*spp[1] - P[13][6]*spp[5]);
    next_p[6][10] = P[6][10] + P[1][10]*sf[2] + P[3][10]*sf[1] + P[0][10]*spp[0] - P[2][10]*spp[1] - P[13][10]*spp[5];
    next_p[6][11] = P[6][11] + P[1][11]*sf[2] + P[3][11]*sf[1] + P[0][11]*spp[0] - P[2][11]*spp[1] - P[13][11]*spp[5];
    next_p[6][12] = P[6][12] + P[1][12]*sf[2] + P[3][12]*sf[1] + P[0][12]*spp[0] - P[2][12]*spp[1] - P[13][12]*spp[5];
    next_p[6][13] = P[6][13] + P[1][13]*sf[2] + P[3][13]*sf[1] + P[0][13]*spp[0] - P[2][13]*spp[1] - P[13][13]*spp[5];
    next_p[6][14] = P[6][14] + P[1][14]*sf[2] + P[3][14]*sf[1] + P[0][14]*spp[0] - P[2][14]*spp[1] - P[13][14]*spp[5];
    next_p[6][15] = P[6][15] + P[1][15]*sf[2] + P[3][15]*sf[1] + P[0][15]*spp[0] - P[2][15]*spp[1] - P[13][15]*spp[5];
    next_p[6][16] = P[6][16] + P[1][16]*sf[2] + P[3][16]*sf[1] + P[0][16]*spp[0] - P[2][16]*spp[1] - P[13][16]*spp[5];
    next_p[6][17] = P[6][17] + P[1][17]*sf[2] + P[3][17]*sf[1] + P[0][17]*spp[0] - P[2][17]*spp[1] - P[13][17]*spp[5];
    next_p[6][18] = P[6][18] + P[1][18]*sf[2] + P[3][18]*sf[1] + P[0][18]*spp[0] - P[2][18]*spp[1] - P[13][18]*spp[5];
    next_p[6][19] = P[6][19] + P[1][19]*sf[2] + P[3][19]*sf[1] + P[0][19]*spp[0] - P[2][19]*spp[1] - P[13][19]*spp[5];
    next_p[6][20] = P[6][20] + P[1][20]*sf[2] + P[3][20]*sf[1] + P[0][20]*spp[0] - P[2][20]*spp[1] - P[13][20]*spp[5];
    next_p[6][21] = P[6][21] + P[1][21]*sf[2] + P[3][21]*sf[1] + P[0][21]*spp[0] - P[2][21]*spp[1] - P[13][21]*spp[5];
    next_p[7][0] = P[7][0] + P[4][0]*dt + sf[7]*(P[7][1] + P[4][1]*dt) + sf[9]*(P[7][2] + P[4][2]*dt) + sf[8]*(P[7][3] + P[4][3]*dt) + sf[11]*(P[7][10] + P[4][10]*dt) + spp[7]*(P[7][11] + P[4][11]*dt) + spp[6]*(P[7][12] + P[4][12]*dt);
    next_p[7][1] = P[7][1] + P[4][1]*dt + sf[6]*(P[7][0] + P[4][0]*dt) + sf[5]*(P[7][2] + P[4][2]*dt) + sf[9]*(P[7][3] + P[4][3]*dt) + spp[6]*(P[7][11] + P[4][11]*dt) - spp[7]*(P[7][12] + P[4][12]*dt) - (q0*(P[7][10] + P[4][10]*dt))/2;
    next_p[7][2] = P[7][2] + P[4][2]*dt + sf[4]*(P[7][0] + P[4][0]*dt) + sf[8]*(P[7][1] + P[4][1]*dt) + sf[6]*(P[7][3] + P[4][3]*dt) + sf[11]*(P[7][12] + P[4][12]*dt) - spp[6]*(P[7][10] + P[4][10]*dt) - (q0*(P[7][11] + P[4][11]*dt))/2;
    next_p[7][3] = P[7][3] + P[4][3]*dt + sf[5]*(P[7][0] + P[4][0]*dt) + sf[4]*(P[7][1] + P[4][1]*dt) + sf[7]*(P[7][2] + P[4][2]*dt) - sf[11]*(P[7][11] + P[4][11]*dt) + spp[7]*(P[7][10] + P[4][10]*dt) - (q0*(P[7][12] + P[4][12]*dt))/2;
    next_p[7][4] = P[7][4] + P[4][4]*dt + sf[1]*(P[7][1] + P[4][1]*dt) + sf[3]*(P[7][0] + P[4][0]*dt) + spp[0]*(P[7][2] + P[4][2]*dt) - spp[2]*(P[7][3] + P[4][3]*dt) - spp[4]*(P[7][13] + P[4][13]*dt);
    next_p[7][5] = P[7][5] + P[4][5]*dt + sf[2]*(P[7][0] + P[4][0]*dt) + sf[1]*(P[7][2] + P[4][2]*dt) + sf[3]*(P[7][3] + P[4][3]*dt) - spp[0]*(P[7][1] + P[4][1]*dt) + spp[3]*(P[7][13] + P[4][13]*dt);
    next_p[7][6] = P[7][6] + P[4][6]*dt + sf[2]*(P[7][1] + P[4][1]*dt) + sf[1]*(P[7][3] + P[4][3]*dt) + spp[0]*(P[7][0] + P[4][0]*dt) - spp[1]*(P[7][2] + P[4][2]*dt) - spp[5]*(P[7][13] + P[4][13]*dt);
    next_p[7][7] = P[7][7] + P[4][7]*dt + dt*(P[7][4] + P[4][4]*dt);
    next_p[7][8] = P[7][8] + P[4][8]*dt + dt*(P[7][5] + P[4][5]*dt);
    next_p[7][9] = P[7][9] + P[4][9]*dt + dt*(P[7][6] + P[4][6]*dt);
    next_p[7][10] = P[7][10] + P[4][10]*dt;
    next_p[7][11] = P[7][11] + P[4][11]*dt;
    next_p[7][12] = P[7][12] + P[4][12]*dt;
    next_p[7][13] = P[7][13] + P[4][13]*dt;
    next_p[7][14] = P[7][14] + P[4][14]*dt;
    next_p[7][15] = P[7][15] + P[4][15]*dt;
    next_p[7][16] = P[7][16] + P[4][16]*dt;
    next_p[7][17] = P[7][17] + P[4][17]*dt;
    next_p[7][18] = P[7][18] + P[4][18]*dt;
    next_p[7][19] = P[7][19] + P[4][19]*dt;
    next_p[7][20] = P[7][20] + P[4][20]*dt;
    next_p[7][21] = P[7][21] + P[4][21]*dt;
    next_p[8][0] = P[8][0] + P[5][0]*dt + sf[7]*(P[8][1] + P[5][1]*dt) + sf[9]*(P[8][2] + P[5][2]*dt) + sf[8]*(P[8][3] + P[5][3]*dt) + sf[11]*(P[8][10] + P[5][10]*dt) + spp[7]*(P[8][11] + P[5][11]*dt) + spp[6]*(P[8][12] + P[5][12]*dt);
    next_p[8][1] = P[8][1] + P[5][1]*dt + sf[6]*(P[8][0] + P[5][0]*dt) + sf[5]*(P[8][2] + P[5][2]*dt) + sf[9]*(P[8][3] + P[5][3]*dt) + spp[6]*(P[8][11] + P[5][11]*dt) - spp[7]*(P[8][12] + P[5][12]*dt) - (q0*(P[8][10] + P[5][10]*dt))/2;
    next_p[8][2] = P[8][2] + P[5][2]*dt + sf[4]*(P[8][0] + P[5][0]*dt) + sf[8]*(P[8][1] + P[5][1]*dt) + sf[6]*(P[8][3] + P[5][3]*dt) + sf[11]*(P[8][12] + P[5][12]*dt) - spp[6]*(P[8][10] + P[5][10]*dt) - (q0*(P[8][11] + P[5][11]*dt))/2;
    next_p[8][3] = P[8][3] + P[5][3]*dt + sf[5]*(P[8][0] + P[5][0]*dt) + sf[4]*(P[8][1] + P[5][1]*dt) + sf[7]*(P[8][2] + P[5][2]*dt) - sf[11]*(P[8][11] + P[5][11]*dt) + spp[7]*(P[8][10] + P[5][10]*dt) - (q0*(P[8][12] + P[5][12]*dt))/2;
    next_p[8][4] = P[8][4] + P[5][4]*dt + sf[1]*(P[8][1] + P[5][1]*dt) + sf[3]*(P[8][0] + P[5][0]*dt) + spp[0]*(P[8][2] + P[5][2]*dt) - spp[2]*(P[8][3] + P[5][3]*dt) - spp[4]*(P[8][13] + P[5][13]*dt);
    next_p[8][5] = P[8][5] + P[5][5]*dt + sf[2]*(P[8][0] + P[5][0]*dt) + sf[1]*(P[8][2] + P[5][2]*dt) + sf[3]*(P[8][3] + P[5][3]*dt) - spp[0]*(P[8][1] + P[5][1]*dt) + spp[3]*(P[8][13] + P[5][13]*dt);
    next_p[8][6] = P[8][6] + P[5][6]*dt + sf[2]*(P[8][1] + P[5][1]*dt) + sf[1]*(P[8][3] + P[5][3]*dt) + spp[0]*(P[8][0] + P[5][0]*dt) - spp[1]*(P[8][2] + P[5][2]*dt) - spp[5]*(P[8][13] + P[5][13]*dt);
    next_p[8][7] = P[8][7] + P[5][7]*dt + dt*(P[8][4] + P[5][4]*dt);
    next_p[8][8] = P[8][8] + P[5][8]*dt + dt*(P[8][5] + P[5][5]*dt);
    next_p[8][9] = P[8][9] + P[5][9]*dt + dt*(P[8][6] + P[5][6]*dt);
    next_p[8][10] = P[8][10] + P[5][10]*dt;
    next_p[8][11] = P[8][11] + P[5][11]*dt;
    next_p[8][12] = P[8][12] + P[5][12]*dt;
    next_p[8][13] = P[8][13] + P[5][13]*dt;
    next_p[8][14] = P[8][14] + P[5][14]*dt;
    next_p[8][15] = P[8][15] + P[5][15]*dt;
    next_p[8][16] = P[8][16] + P[5][16]*dt;
    next_p[8][17] = P[8][17] + P[5][17]*dt;
    next_p[8][18] = P[8][18] + P[5][18]*dt;
    next_p[8][19] = P[8][19] + P[5][19]*dt;
    next_p[8][20] = P[8][20] + P[5][20]*dt;
    next_p[8][21] = P[8][21] + P[5][21]*dt;
    next_p[9][0] = P[9][0] + P[6][0]*dt + sf[7]*(P[9][1] + P[6][1]*dt) + sf[9]*(P[9][2] + P[6][2]*dt) + sf[8]*(P[9][3] + P[6][3]*dt) + sf[11]*(P[9][10] + P[6][10]*dt) + spp[7]*(P[9][11] + P[6][11]*dt) + spp[6]*(P[9][12] + P[6][12]*dt);
    next_p[9][1] = P[9][1] + P[6][1]*dt + sf[6]*(P[9][0] + P[6][0]*dt) + sf[5]*(P[9][2] + P[6][2]*dt) + sf[9]*(P[9][3] + P[6][3]*dt) + spp[6]*(P[9][11] + P[6][11]*dt) - spp[7]*(P[9][12] + P[6][12]*dt) - (q0*(P[9][10] + P[6][10]*dt))/2;
    next_p[9][2] = P[9][2] + P[6][2]*dt + sf[4]*(P[9][0] + P[6][0]*dt) + sf[8]*(P[9][1] + P[6][1]*dt) + sf[6]*(P[9][3] + P[6][3]*dt) + sf[11]*(P[9][12] + P[6][12]*dt) - spp[6]*(P[9][10] + P[6][10]*dt) - (q0*(P[9][11] + P[6][11]*dt))/2;
    next_p[9][3] = P[9][3] + P[6][3]*dt + sf[5]*(P[9][0] + P[6][0]*dt) + sf[4]*(P[9][1] + P[6][1]*dt) + sf[7]*(P[9][2] + P[6][2]*dt) - sf[11]*(P[9][11] + P[6][11]*dt) + spp[7]*(P[9][10] + P[6][10]*dt) - (q0*(P[9][12] + P[6][12]*dt))/2;
    next_p[9][4] = P[9][4] + P[6][4]*dt + sf[1]*(P[9][1] + P[6][1]*dt) + sf[3]*(P[9][0] + P[6][0]*dt) + spp[0]*(P[9][2] + P[6][2]*dt) - spp[2]*(P[9][3] + P[6][3]*dt) - spp[4]*(P[9][13] + P[6][13]*dt);
    next_p[9][5] = P[9][5] + P[6][5]*dt + sf[2]*(P[9][0] + P[6][0]*dt) + sf[1]*(P[9][2] + P[6][2]*dt) + sf[3]*(P[9][3] + P[6][3]*dt) - spp[0]*(P[9][1] + P[6][1]*dt) + spp[3]*(P[9][13] + P[6][13]*dt);
    next_p[9][6] = P[9][6] + P[6][6]*dt + sf[2]*(P[9][1] + P[6][1]*dt) + sf[1]*(P[9][3] + P[6][3]*dt) + spp[0]*(P[9][0] + P[6][0]*dt) - spp[1]*(P[9][2] + P[6][2]*dt) - spp[5]*(P[9][13] + P[6][13]*dt);
    next_p[9][7] = P[9][7] + P[6][7]*dt + dt*(P[9][4] + P[6][4]*dt);
    next_p[9][8] = P[9][8] + P[6][8]*dt + dt*(P[9][5] + P[6][5]*dt);
    next_p[9][9] = P[9][9] + P[6][9]*dt + dt*(P[9][6] + P[6][6]*dt);
    next_p[9][10] = P[9][10] + P[6][10]*dt;
    next_p[9][11] = P[9][11] + P[6][11]*dt;
    next_p[9][12] = P[9][12] + P[6][12]*dt;
    next_p[9][13] = P[9][13] + P[6][13]*dt;
    next_p[9][14] = P[9][14] + P[6][14]*dt;
    next_p[9][15] = P[9][15] + P[6][15]*dt;
    next_p[9][16] = P[9][16] + P[6][16]*dt;
    next_p[9][17] = P[9][17] + P[6][17]*dt;
    next_p[9][18] = P[9][18] + P[6][18]*dt;
    next_p[9][19] = P[9][19] + P[6][19]*dt;
    next_p[9][20] = P[9][20] + P[6][20]*dt;
    next_p[9][21] = P[9][21] + P[6][21]*dt;
    next_p[10][0] = P[10][0] + P[10][1]*sf[7] + P[10][2]*sf[9] + P[10][3]*sf[8] + P[10][10]*sf[11] + P[10][11]*spp[7] + P[10][12]*spp[6];
    next_p[10][1] = P[10][1] + P[10][0]*sf[6] + P[10][2]*sf[5] + P[10][3]*sf[9] + P[10][11]*spp[6] - P[10][12]*spp[7] - (P[10][10]*q0)/2;
    next_p[10][2] = P[10][2] + P[10][0]*sf[4] + P[10][1]*sf[8] + P[10][3]*sf[6] + P[10][12]*sf[11] - P[10][10]*spp[6] - (P[10][11]*q0)/2;
    next_p[10][3] = P[10][3] + P[10][0]*sf[5] + P[10][1]*sf[4] + P[10][2]*sf[7] - P[10][11]*sf[11] + P[10][10]*spp[7] - (P[10][12]*q0)/2;
    next_p[10][4] = P[10][4] + P[10][1]*sf[1] + P[10][0]*sf[3] + P[10][2]*spp[0] - P[10][3]*spp[2] - P[10][13]*spp[4];
    next_p[10][5] = P[10][5] + P[10][0]*sf[2] + P[10][2]*sf[1] + P[10][3]*sf[3] - P[10][1]*spp[0] + P[10][13]*spp[3];
    next_p[10][6] = P[10][6] + P[10][1]*sf[2] + P[10][3]*sf[1] + P[10][0]*spp[0] - P[10][2]*spp[1] - P[10][13]*spp[5];
    next_p[10][7] = P[10][7] + P[10][4]*dt;
    next_p[10][8] = P[10][8] + P[10][5]*dt;
    next_p[10][9] = P[10][9] + P[10][6]*dt;
    next_p[10][10] = P[10][10];
    next_p[10][11] = P[10][11];
    next_p[10][12] = P[10][12];
    next_p[10][13] = P[10][13];
    next_p[10][14] = P[10][14];
    next_p[10][15] = P[10][15];
    next_p[10][16] = P[10][16];
    next_p[10][17] = P[10][17];
    next_p[10][18] = P[10][18];
    next_p[10][19] = P[10][19];
    next_p[10][20] = P[10][20];
    next_p[10][21] = P[10][21];
    next_p[11][0] = P[11][0] + P[11][1]*sf[7] + P[11][2]*sf[9] + P[11][3]*sf[8] + P[11][10]*sf[11] + P[11][11]*spp[7] + P[11][12]*spp[6];
    next_p[11][1] = P[11][1] + P[11][0]*sf[6] + P[11][2]*sf[5] + P[11][3]*sf[9] + P[11][11]*spp[6] - P[11][12]*spp[7] - (P[11][10]*q0)/2;
    next_p[11][2] = P[11][2] + P[11][0]*sf[4] + P[11][1]*sf[8] + P[11][3]*sf[6] + P[11][12]*sf[11] - P[11][10]*spp[6] - (P[11][11]*q0)/2;
    next_p[11][3] = P[11][3] + P[11][0]*sf[5] + P[11][1]*sf[4] + P[11][2]*sf[7] - P[11][11]*sf[11] + P[11][10]*spp[7] - (P[11][12]*q0)/2;
    next_p[11][4] = P[11][4] + P[11][1]*sf[1] + P[11][0]*sf[3] + P[11][2]*spp[0] - P[11][3]*spp[2] - P[11][13]*spp[4];
    next_p[11][5] = P[11][5] + P[11][0]*sf[2] + P[11][2]*sf[1] + P[11][3]*sf[3] - P[11][1]*spp[0] + P[11][13]*spp[3];
    next_p[11][6] = P[11][6] + P[11][1]*sf[2] + P[11][3]*sf[1] + P[11][0]*spp[0] - P[11][2]*spp[1] - P[11][13]*spp[5];
    next_p[11][7] = P[11][7] + P[11][4]*dt;
    next_p[11][8] = P[11][8] + P[11][5]*dt;
    next_p[11][9] = P[11][9] + P[11][6]*dt;
    next_p[11][10] = P[11][10];
    next_p[11][11] = P[11][11];
    next_p[11][12] = P[11][12];
    next_p[11][13] = P[11][13];
    next_p[11][14] = P[11][14];
    next_p[11][15] = P[11][15];
    next_p[11][16] = P[11][16];
    next_p[11][17] = P[11][17];
    next_p[11][18] = P[11][18];
    next_p[11][19] = P[11][19];
    next_p[11][20] = P[11][20];
    next_p[11][21] = P[11][21];
    next_p[12][0] = P[12][0] + P[12][1]*sf[7] + P[12][2]*sf[9] + P[12][3]*sf[8] + P[12][10]*sf[11] + P[12][11]*spp[7] + P[12][12]*spp[6];
    next_p[12][1] = P[12][1] + P[12][0]*sf[6] + P[12][2]*sf[5] + P[12][3]*sf[9] + P[12][11]*spp[6] - P[12][12]*spp[7] - (P[12][10]*q0)/2;
    next_p[12][2] = P[12][2] + P[12][0]*sf[4] + P[12][1]*sf[8] + P[12][3]*sf[6] + P[12][12]*sf[11] - P[12][10]*spp[6] - (P[12][11]*q0)/2;
    next_p[12][3] = P[12][3] + P[12][0]*sf[5] + P[12][1]*sf[4] + P[12][2]*sf[7] - P[12][11]*sf[11] + P[12][10]*spp[7] - (P[12][12]*q0)/2;
    next_p[12][4] = P[12][4] + P[12][1]*sf[1] + P[12][0]*sf[3] + P[12][2]*spp[0] - P[12][3]*spp[2] - P[12][13]*spp[4];
    next_p[12][5] = P[12][5] + P[12][0]*sf[2] + P[12][2]*sf[1] + P[12][3]*sf[3] - P[12][1]*spp[0] + P[12][13]*spp[3];
    next_p[12][6] = P[12][6] + P[12][1]*sf[2] + P[12][3]*sf[1] + P[12][0]*spp[0] - P[12][2]*spp[1] - P[12][13]*spp[5];
    next_p[12][7] = P[12][7] + P[12][4]*dt;
    next_p[12][8] = P[12][8] + P[12][5]*dt;
    next_p[12][9] = P[12][9] + P[12][6]*dt;
    next_p[12][10] = P[12][10];
    next_p[12][11] = P[12][11];
    next_p[12][12] = P[12][12];
    next_p[12][13] = P[12][13];
    next_p[12][14] = P[12][14];
    next_p[12][15] = P[12][15];
    next_p[12][16] = P[12][16];
    next_p[12][17] = P[12][17];
    next_p[12][18] = P[12][18];
    next_p[12][19] = P[12][19];
    next_p[12][20] = P[12][20];
    next_p[12][21] = P[12][21];
    next_p[13][0] = P[13][0] + P[13][1]*sf[7] + P[13][2]*sf[9] + P[13][3]*sf[8] + P[13][10]*sf[11] + P[13][11]*spp[7] + P[13][12]*spp[6];
    next_p[13][1] = P[13][1] + P[13][0]*sf[6] + P[13][2]*sf[5] + P[13][3]*sf[9] + P[13][11]*spp[6] - P[13][12]*spp[7] - (P[13][10]*q0)/2;
    next_p[13][2] = P[13][2] + P[13][0]*sf[4] + P[13][1]*sf[8] + P[13][3]*sf[6] + P[13][12]*sf[11] - P[13][10]*spp[6] - (P[13][11]*q0)/2;
    next_p[13][3] = P[13][3] + P[13][0]*sf[5] + P[13][1]*sf[4] + P[13][2]*sf[7] - P[13][11]*sf[11] + P[13][10]*spp[7] - (P[13][12]*q0)/2;
    next_p[13][4] = P[13][4] + P[13][1]*sf[1] + P[13][0]*sf[3] + P[13][2]*spp[0] - P[13][3]*spp[2] - P[13][13]*spp[4];
    next_p[13][5] = P[13][5] + P[13][0]*sf[2] + P[13][2]*sf[1] + P[13][3]*sf[3] - P[13][1]*spp[0] + P[13][13]*spp[3];
    next_p[13][6] = P[13][6] + P[13][1]*sf[2] + P[13][3]*sf[1] + P[13][0]*spp[0] - P[13][2]*spp[1] - P[13][13]*spp[5];
    next_p[13][7] = P[13][7] + P[13][4]*dt;
    next_p[13][8] = P[13][8] + P[13][5]*dt;
    next_p[13][9] = P[13][9] + P[13][6]*dt;
    next_p[13][10] = P[13][10];
    next_p[13][11] = P[13][11];
    next_p[13][12] = P[13][12];
    next_p[13][13] = P[13][13];
    next_p[13][14] = P[13][14];
    next_p[13][15] = P[13][15];
    next_p[13][16] = P[13][16];
    next_p[13][17] = P[13][17];
    next_p[13][18] = P[13][18];
    next_p[13][19] = P[13][19];
    next_p[13][20] = P[13][20];
    next_p[13][21] = P[13][21];
    next_p[14][0] = P[14][0] + P[14][1]*sf[7] + P[14][2]*sf[9] + P[14][3]*sf[8] + P[14][10]*sf[11] + P[14][11]*spp[7] + P[14][12]*spp[6];
    next_p[14][1] = P[14][1] + P[14][0]*sf[6] + P[14][2]*sf[5] + P[14][3]*sf[9] + P[14][11]*spp[6] - P[14][12]*spp[7] - (P[14][10]*q0)/2;
    next_p[14][2] = P[14][2] + P[14][0]*sf[4] + P[14][1]*sf[8] + P[14][3]*sf[6] + P[14][12]*sf[11] - P[14][10]*spp[6] - (P[14][11]*q0)/2;
    next_p[14][3] = P[14][3] + P[14][0]*sf[5] + P[14][1]*sf[4] + P[14][2]*sf[7] - P[14][11]*sf[11] + P[14][10]*spp[7] - (P[14][12]*q0)/2;
    next_p[14][4] = P[14][4] + P[14][1]*sf[1] + P[14][0]*sf[3] + P[14][2]*spp[0] - P[14][3]*spp[2] - P[14][13]*spp[4];
    next_p[14][5] = P[14][5] + P[14][0]*sf[2] + P[14][2]*sf[1] + P[14][3]*sf[3] - P[14][1]*spp[0] + P[14][13]*spp[3];
    next_p[14][6] = P[14][6] + P[14][1]*sf[2] + P[14][3]*sf[1] + P[14][0]*spp[0] - P[14][2]*spp[1] - P[14][13]*spp[5];
    next_p[14][7] = P[14][7] + P[14][4]*dt;
    next_p[14][8] = P[14][8] + P[14][5]*dt;
    next_p[14][9] = P[14][9] + P[14][6]*dt;
    next_p[14][10] = P[14][10];
    next_p[14][11] = P[14][11];
    next_p[14][12] = P[14][12];
    next_p[14][13] = P[14][13];
    next_p[14][14] = P[14][14];
    next_p[14][15] = P[14][15];
    next_p[14][16] = P[14][16];
    next_p[14][17] = P[14][17];
    next_p[14][18] = P[14][18];
    next_p[14][19] = P[14][19];
    next_p[14][20] = P[14][20];
    next_p[14][21] = P[14][21];
    next_p[15][0] = P[15][0] + P[15][1]*sf[7] + P[15][2]*sf[9] + P[15][3]*sf[8] + P[15][10]*sf[11] + P[15][11]*spp[7] + P[15][12]*spp[6];
    next_p[15][1] = P[15][1] + P[15][0]*sf[6] + P[15][2]*sf[5] + P[15][3]*sf[9] + P[15][11]*spp[6] - P[15][12]*spp[7] - (P[15][10]*q0)/2;
    next_p[15][2] = P[15][2] + P[15][0]*sf[4] + P[15][1]*sf[8] + P[15][3]*sf[6] + P[15][12]*sf[11] - P[15][10]*spp[6] - (P[15][11]*q0)/2;
    next_p[15][3] = P[15][3] + P[15][0]*sf[5] + P[15][1]*sf[4] + P[15][2]*sf[7] - P[15][11]*sf[11] + P[15][10]*spp[7] - (P[15][12]*q0)/2;
    next_p[15][4] = P[15][4] + P[15][1]*sf[1] + P[15][0]*sf[3] + P[15][2]*spp[0] - P[15][3]*spp[2] - P[15][13]*spp[4];
    next_p[15][5] = P[15][5] + P[15][0]*sf[2] + P[15][2]*sf[1] + P[15][3]*sf[3] - P[15][1]*spp[0] + P[15][13]*spp[3];
    next_p[15][6] = P[15][6] + P[15][1]*sf[2] + P[15][3]*sf[1] + P[15][0]*spp[0] - P[15][2]*spp[1] - P[15][13]*spp[5];
    next_p[15][7] = P[15][7] + P[15][4]*dt;
    next_p[15][8] = P[15][8] + P[15][5]*dt;
    next_p[15][9] = P[15][9] + P[15][6]*dt;
    next_p[15][10] = P[15][10];
    next_p[15][11] = P[15][11];
    next_p[15][12] = P[15][12];
    next_p[15][13] = P[15][13];
    next_p[15][14] = P[15][14];
    next_p[15][15] = P[15][15];
    next_p[15][16] = P[15][16];
    next_p[15][17] = P[15][17];
    next_p[15][18] = P[15][18];
    next_p[15][19] = P[15][19];
    next_p[15][20] = P[15][20];
    next_p[15][21] = P[15][21];
    next_p[16][0] = P[16][0] + P[16][1]*sf[7] + P[16][2]*sf[9] + P[16][3]*sf[8] + P[16][10]*sf[11] + P[16][11]*spp[7] + P[16][12]*spp[6];
    next_p[16][1] = P[16][1] + P[16][0]*sf[6] + P[16][2]*sf[5] + P[16][3]*sf[9] + P[16][11]*spp[6] - P[16][12]*spp[7] - (P[16][10]*q0)/2;
    next_p[16][2] = P[16][2] + P[16][0]*sf[4] + P[16][1]*sf[8] + P[16][3]*sf[6] + P[16][12]*sf[11] - P[16][10]*spp[6] - (P[16][11]*q0)/2;
    next_p[16][3] = P[16][3] + P[16][0]*sf[5] + P[16][1]*sf[4] + P[16][2]*sf[7] - P[16][11]*sf[11] + P[16][10]*spp[7] - (P[16][12]*q0)/2;
    next_p[16][4] = P[16][4] + P[16][1]*sf[1] + P[16][0]*sf[3] + P[16][2]*spp[0] - P[16][3]*spp[2] - P[16][13]*spp[4];
    next_p[16][5] = P[16][5] + P[16][0]*sf[2] + P[16][2]*sf[1] + P[16][3]*sf[3] - P[16][1]*spp[0] + P[16][13]*spp[3];
    next_p[16][6] = P[16][6] + P[16][1]*sf[2] + P[16][3]*sf[1] + P[16][0]*spp[0] - P[16][2]*spp[1] - P[16][13]*spp[5];
    next_p[16][7] = P[16][7] + P[16][4]*dt;
    next_p[16][8] = P[16][8] + P[16][5]*dt;
    next_p[16][9] = P[16][9] + P[16][6]*dt;
    next_p[16][10] = P[16][10];
    next_p[16][11] = P[16][11];
    next_p[16][12] = P[16][12];
    next_p[16][13] = P[16][13];
    next_p[16][14] = P[16][14];
    next_p[16][15] = P[16][15];
    next_p[16][16] = P[16][16];
    next_p[16][17] = P[16][17];
    next_p[16][18] = P[16][18];
    next_p[16][19] = P[16][19];
    next_p[16][20] = P[16][20];
    next_p[16][21] = P[16][21];
    next_p[17][0] = P[17][0] + P[17][1]*sf[7] + P[17][2]*sf[9] + P[17][3]*sf[8] + P[17][10]*sf[11] + P[17][11]*spp[7] + P[17][12]*spp[6];
    next_p[17][1] = P[17][1] + P[17][0]*sf[6] + P[17][2]*sf[5] + P[17][3]*sf[9] + P[17][11]*spp[6] - P[17][12]*spp[7] - (P[17][10]*q0)/2;
    next_p[17][2] = P[17][2] + P[17][0]*sf[4] + P[17][1]*sf[8] + P[17][3]*sf[6] + P[17][12]*sf[11] - P[17][10]*spp[6] - (P[17][11]*q0)/2;
    next_p[17][3] = P[17][3] + P[17][0]*sf[5] + P[17][1]*sf[4] + P[17][2]*sf[7] - P[17][11]*sf[11] + P[17][10]*spp[7] - (P[17][12]*q0)/2;
    next_p[17][4] = P[17][4] + P[17][1]*sf[1] + P[17][0]*sf[3] + P[17][2]*spp[0] - P[17][3]*spp[2] - P[17][13]*spp[4];
    next_p[17][5] = P[17][5] + P[17][0]*sf[2] + P[17][2]*sf[1] + P[17][3]*sf[3] - P[17][1]*spp[0] + P[17][13]*spp[3];
    next_p[17][6] = P[17][6] + P[17][1]*sf[2] + P[17][3]*sf[1] + P[17][0]*spp[0] - P[17][2]*spp[1] - P[17][13]*spp[5];
    next_p[17][7] = P[17][7] + P[17][4]*dt;
    next_p[17][8] = P[17][8] + P[17][5]*dt;
    next_p[17][9] = P[17][9] + P[17][6]*dt;
    next_p[17][10] = P[17][10];
    next_p[17][11] = P[17][11];
    next_p[17][12] = P[17][12];
    next_p[17][13] = P[17][13];
    next_p[17][14] = P[17][14];
    next_p[17][15] = P[17][15];
    next_p[17][16] = P[17][16];
    next_p[17][17] = P[17][17];
    next_p[17][18] = P[17][18];
    next_p[17][19] = P[17][19];
    next_p[17][20] = P[17][20];
    next_p[17][21] = P[17][21];
    next_p[18][0] = P[18][0] + P[18][1]*sf[7] + P[18][2]*sf[9] + P[18][3]*sf[8] + P[18][10]*sf[11] + P[18][11]*spp[7] + P[18][12]*spp[6];
    next_p[18][1] = P[18][1] + P[18][0]*sf[6] + P[18][2]*sf[5] + P[18][3]*sf[9] + P[18][11]*spp[6] - P[18][12]*spp[7] - (P[18][10]*q0)/2;
    next_p[18][2] = P[18][2] + P[18][0]*sf[4] + P[18][1]*sf[8] + P[18][3]*sf[6] + P[18][12]*sf[11] - P[18][10]*spp[6] - (P[18][11]*q0)/2;
    next_p[18][3] = P[18][3] + P[18][0]*sf[5] + P[18][1]*sf[4] + P[18][2]*sf[7] - P[18][11]*sf[11] + P[18][10]*spp[7] - (P[18][12]*q0)/2;
    next_p[18][4] = P[18][4] + P[18][1]*sf[1] + P[18][0]*sf[3] + P[18][2]*spp[0] - P[18][3]*spp[2] - P[18][13]*spp[4];
    next_p[18][5] = P[18][5] + P[18][0]*sf[2] + P[18][2]*sf[1] + P[18][3]*sf[3] - P[18][1]*spp[0] + P[18][13]*spp[3];
    next_p[18][6] = P[18][6] + P[18][1]*sf[2] + P[18][3]*sf[1] + P[18][0]*spp[0] - P[18][2]*spp[1] - P[18][13]*spp[5];
    next_p[18][7] = P[18][7] + P[18][4]*dt;
    next_p[18][8] = P[18][8] + P[18][5]*dt;
    next_p[18][9] = P[18][9] + P[18][6]*dt;
    next_p[18][10] = P[18][10];
    next_p[18][11] = P[18][11];
    next_p[18][12] = P[18][12];
    next_p[18][13] = P[18][13];
    next_p[18][14] = P[18][14];
    next_p[18][15] = P[18][15];
    next_p[18][16] = P[18][16];
    next_p[18][17] = P[18][17];
    next_p[18][18] = P[18][18];
    next_p[18][19] = P[18][19];
    next_p[18][20] = P[18][20];
    next_p[18][21] = P[18][21];
    next_p[19][0] = P[19][0] + P[19][1]*sf[7] + P[19][2]*sf[9] + P[19][3]*sf[8] + P[19][10]*sf[11] + P[19][11]*spp[7] + P[19][12]*spp[6];
    next_p[19][1] = P[19][1] + P[19][0]*sf[6] + P[19][2]*sf[5] + P[19][3]*sf[9] + P[19][11]*spp[6] - P[19][12]*spp[7] - (P[19][10]*q0)/2;
    next_p[19][2] = P[19][2] + P[19][0]*sf[4] + P[19][1]*sf[8] + P[19][3]*sf[6] + P[19][12]*sf[11] - P[19][10]*spp[6] - (P[19][11]*q0)/2;
    next_p[19][3] = P[19][3] + P[19][0]*sf[5] + P[19][1]*sf[4] + P[19][2]*sf[7] - P[19][11]*sf[11] + P[19][10]*spp[7] - (P[19][12]*q0)/2;
    next_p[19][4] = P[19][4] + P[19][1]*sf[1] + P[19][0]*sf[3] + P[19][2]*spp[0] - P[19][3]*spp[2] - P[19][13]*spp[4];
    next_p[19][5] = P[19][5] + P[19][0]*sf[2] + P[19][2]*sf[1] + P[19][3]*sf[3] - P[19][1]*spp[0] + P[19][13]*spp[3];
    next_p[19][6] = P[19][6] + P[19][1]*sf[2] + P[19][3]*sf[1] + P[19][0]*spp[0] - P[19][2]*spp[1] - P[19][13]*spp[5];
    next_p[19][7] = P[19][7] + P[19][4]*dt;
    next_p[19][8] = P[19][8] + P[19][5]*dt;
    next_p[19][9] = P[19][9] + P[19][6]*dt;
    next_p[19][10] = P[19][10];
    next_p[19][11] = P[19][11];
    next_p[19][12] = P[19][12];
    next_p[19][13] = P[19][13];
    next_p[19][14] = P[19][14];
    next_p[19][15] = P[19][15];
    next_p[19][16] = P[19][16];
    next_p[19][17] = P[19][17];
    next_p[19][18] = P[19][18];
    next_p[19][19] = P[19][19];
    next_p[19][20] = P[19][20];
    next_p[19][21] = P[19][21];
    next_p[20][0] = P[20][0] + P[20][1]*sf[7] + P[20][2]*sf[9] + P[20][3]*sf[8] + P[20][10]*sf[11] + P[20][11]*spp[7] + P[20][12]*spp[6];
    next_p[20][1] = P[20][1] + P[20][0]*sf[6] + P[20][2]*sf[5] + P[20][3]*sf[9] + P[20][11]*spp[6] - P[20][12]*spp[7] - (P[20][10]*q0)/2;
    next_p[20][2] = P[20][2] + P[20][0]*sf[4] + P[20][1]*sf[8] + P[20][3]*sf[6] + P[20][12]*sf[11] - P[20][10]*spp[6] - (P[20][11]*q0)/2;
    next_p[20][3] = P[20][3] + P[20][0]*sf[5] + P[20][1]*sf[4] + P[20][2]*sf[7] - P[20][11]*sf[11] + P[20][10]*spp[7] - (P[20][12]*q0)/2;
    next_p[20][4] = P[20][4] + P[20][1]*sf[1] + P[20][0]*sf[3] + P[20][2]*spp[0] - P[20][3]*spp[2] - P[20][13]*spp[4];
    next_p[20][5] = P[20][5] + P[20][0]*sf[2] + P[20][2]*sf[1] + P[20][3]*sf[3] - P[20][1]*spp[0] + P[20][13]*spp[3];
    next_p[20][6] = P[20][6] + P[20][1]*sf[2] + P[20][3]*sf[1] + P[20][0]*spp[0] - P[20][2]*spp[1] - P[20][13]*spp[5];
    next_p[20][7] = P[20][7] + P[20][4]*dt;
    next_p[20][8] = P[20][8] + P[20][5]*dt;
    next_p[20][9] = P[20][9] + P[20][6]*dt;
    next_p[20][10] = P[20][10];
    next_p[20][11] = P[20][11];
    next_p[20][12] = P[20][12];
    next_p[20][13] = P[20][13];
    next_p[20][14] = P[20][14];
    next_p[20][15] = P[20][15];
    next_p[20][16] = P[20][16];
    next_p[20][17] = P[20][17];
    next_p[20][18] = P[20][18];
    next_p[20][19] = P[20][19];
    next_p[20][20] = P[20][20];
    next_p[20][21] = P[20][21];
    next_p[21][0] = P[21][0] + P[21][1]*sf[7] + P[21][2]*sf[9] + P[21][3]*sf[8] + P[21][10]*sf[11] + P[21][11]*spp[7] + P[21][12]*spp[6];
    next_p[21][1] = P[21][1] + P[21][0]*sf[6] + P[21][2]*sf[5] + P[21][3]*sf[9] + P[21][11]*spp[6] - P[21][12]*spp[7] - (P[21][10]*q0)/2;
    next_p[21][2] = P[21][2] + P[21][0]*sf[4] + P[21][1]*sf[8] + P[21][3]*sf[6] + P[21][12]*sf[11] - P[21][10]*spp[6] - (P[21][11]*q0)/2;
    next_p[21][3] = P[21][3] + P[21][0]*sf[5] + P[21][1]*sf[4] + P[21][2]*sf[7] - P[21][11]*sf[11] + P[21][10]*spp[7] - (P[21][12]*q0)/2;
    next_p[21][4] = P[21][4] + P[21][1]*sf[1] + P[21][0]*sf[3] + P[21][2]*spp[0] - P[21][3]*spp[2] - P[21][13]*spp[4];
    next_p[21][5] = P[21][5] + P[21][0]*sf[2] + P[21][2]*sf[1] + P[21][3]*sf[3] - P[21][1]*spp[0] + P[21][13]*spp[3];
    next_p[21][6] = P[21][6] + P[21][1]*sf[2] + P[21][3]*sf[1] + P[21][0]*spp[0] - P[21][2]*spp[1] - P[21][13]*spp[5];
    next_p[21][7] = P[21][7] + P[21][4]*dt;
    next_p[21][8] = P[21][8] + P[21][5]*dt;
    next_p[21][9] = P[21][9] + P[21][6]*dt;
    next_p[21][10] = P[21][10];
    next_p[21][11] = P[21][11];
    next_p[21][12] = P[21][12];
    next_p[21][13] = P[21][13];
    next_p[21][14] = P[21][14];
    next_p[21][15] = P[21][15];
    next_p[21][16] = P[21][16];
    next_p[21][17] = P[21][17];
    next_p[21][18] = P[21][18];
    next_p[21][19] = P[21][19];
    next_p[21][20] = P[21][20];
    next_p[21][21] = P[21][21];

    for (size_t i = 0; i < ekf_state_estimates; i++)
    {
        next_p[i][i] = next_p[i][i] + process_noise[i];
    }

    // If the total position variance exceds 1E6 (1000m), then stop covariance
    // growth by setting the predicted to the previous values
    // This prevent an ill conditioned matrix from occurring for long periods
    // without GPS
    if ((P[7][7] + P[8][8]) > 1E6f)
    {
        for (uint8_t i=7; i<=8; i++)
        {
            for (size_t j = 0; j < ekf_state_estimates; j++)
            {
                next_p[i][j] = P[i][j];
                next_p[j][i] = P[j][i];
            }
        }
    }

    // Copy covariance
    for (size_t i = 0; i < ekf_state_estimates; i++) {
        P[i][i] = next_p[i][i];
    }

    // force symmetry for observable states
    for (size_t i = 1; i < ekf_state_estimates; i++)
    {
        for (uint8_t j = 0; j < i; j++)
        {
            P[i][j] = 0.5f * (next_p[i][j] + next_p[j][i]);
            P[j][i] = P[i][j];
        }
    }

        ConstrainVariances();
}

void AttPosEKF::updateDtGpsFilt(float dt)
{
    dtGpsFilt = ConstrainFloat(dt, 0.001f, 2.0f) * 0.05f + dtGpsFilt * 0.95f;
}

void AttPosEKF::updateDtHgtFilt(float dt)
{
    dtHgtFilt = ConstrainFloat(dt, 0.001f, 2.0f) * 0.05f + dtHgtFilt * 0.95f;
}

void AttPosEKF::updateDtVelPosFilt(float dt)
{
    dtVelPosFilt = ConstrainFloat(dt, 0.0005f, 2.0f) * 0.05f + dtVelPosFilt * 0.95f;
}

void AttPosEKF::fuseVelposNed()
{

    // declare variables used by fault isolation logic
    uint32_t gps_retry_time = 3000; // time in msec before GPS fusion will be retried following innovation consistency failure
    uint32_t gps_retry_time_no_tas = 500; // retry time if no TAS measurement available
    uint32_t hgt_retry_time = 500; // height measurement retry time
    uint32_t horiz_retry_time;

    // declare variables used to check measurement errors
    float vel_innov[3] = {0.0f,0.0f,0.0f};
    float pos_innov[2] = {0.0f,0.0f};
    float hgt_innov = 0.0f;

    // declare variables used to control access to arrays
    bool fuse_data[6] = {false,false,false,false,false,false};
    uint8_t state_index;
    uint8_t obs_index;
    uint8_t index_limit = 21;

    // declare variables used by state and covariance update calculations
    float vel_err;
    float pos_err;
    float r_obs[6];
    float observation[6];
    float sk;
    float quat_mag;

    // Perform sequential fusion of GPS measurements. This assumes that the
    // errors in the different velocity and position components are
    // uncorrelated which is not true, however in the absence of covariance
    // data from the GPS receiver it is the only assumption we can make
    // so we might as well take advantage of the computational efficiencies
    // associated with sequential fusion
    if (fuseVelData || fusePosData || fuseHgtData)
    {
        uint64_t t_now = get_micros();
        updateDtVelPosFilt((t_now - lastVelPosFusion) / 1e6f);
        lastVelPosFusion = t_now;

        // scaler according to the number of repetitions of the
        // same measurement in one fusion step
        float gps_variance_scaler = dtGpsFilt / dtVelPosFilt;

        // scaler according to the number of repetitions of the
        // same measurement in one fusion step
        float hgt_variance_scaler = dtHgtFilt / dtVelPosFilt;

        // set the GPS data timeout depending on whether airspeed data is present
        if (useAirspeed) {
            horiz_retry_time = gps_retry_time;
        } else {
            horiz_retry_time = gps_retry_time_no_tas;
        }

        // Form the observation vector
        for (uint8_t i=0; i <=2; i++) observation[i] = velNED[i];
        for (uint8_t i=3; i <=4; i++) observation[i] = posNE[i-3];
        observation[5] = -(hgtMea);

        // Estimate the GPS Velocity, GPS horiz position and height measurement variances.
        vel_err = 0.2f*accNavMag; // additional error in GPS velocities caused by manoeuvring
        pos_err = 0.2f*accNavMag; // additional error in GPS position caused by manoeuvring
        r_obs[0] = gps_variance_scaler * sq(vneSigma) + sq(vel_err);
        r_obs[1] = r_obs[0];
        r_obs[2] = gps_variance_scaler * sq(vdSigma) + sq(vel_err);
        r_obs[3] = gps_variance_scaler * sq(posNeSigma) + sq(pos_err);
        r_obs[4] = r_obs[3];
        r_obs[5] = hgt_variance_scaler * sq(posDSigma) + sq(pos_err);

        // calculate innovations and check GPS data validity using an innovation consistency check
        if (fuseVelData)
        {
            // test velocity measurements
            uint8_t imax = 2;
            if (fusionModeGPS == 1) imax = 1;
            for (uint8_t i = 0; i<=imax; i++)
            {
                vel_innov[i] = statesAtVelTime[i+4] - velNED[i];
                state_index = 4 + i;
                varInnovVelPos[i] = P[state_index][state_index] + r_obs[i];
            }
            // apply a 5-sigma threshold
            current_ekf_state.velHealth = (sq(vel_innov[0]) + sq(vel_innov[1]) + sq(vel_innov[2])) < 25.0f * (varInnovVelPos[0] + varInnovVelPos[1] + varInnovVelPos[2]);
            current_ekf_state.velTimeout = (millis() - current_ekf_state.velFailTime) > horiz_retry_time;
            if (current_ekf_state.velHealth || staticMode) {
                current_ekf_state.velHealth = true;
                current_ekf_state.velFailTime = millis();
            } else if (current_ekf_state.velTimeout || !current_ekf_state.posHealth) {
                current_ekf_state.velHealth = true;
                ResetVelocity();

                // do not fuse bad data
                fuseVelData = false;
            }
            else
            {
                current_ekf_state.velHealth = false;
            }
        }

        if (fusePosData)
        {
            // test horizontal position measurements
            pos_innov[0] = statesAtPosTime[7] - posNE[0];
            pos_innov[1] = statesAtPosTime[8] - posNE[1];
            varInnovVelPos[3] = P[7][7] + r_obs[3];
            varInnovVelPos[4] = P[8][8] + r_obs[4];
            // apply a 10-sigma threshold
            current_ekf_state.posHealth = (sq(pos_innov[0]) + sq(pos_innov[1])) < 100.0f*(varInnovVelPos[3] + varInnovVelPos[4]);
            current_ekf_state.posTimeout = (millis() - current_ekf_state.posFailTime) > horiz_retry_time;
            if (current_ekf_state.posHealth || current_ekf_state.posTimeout)
            {
                current_ekf_state.posHealth = true;
                current_ekf_state.posFailTime = millis();

                if (current_ekf_state.posTimeout) {
                    ResetPosition();
                    
                    // do not fuse position data on this time
                    // step
                    fusePosData = false;
                }
            }
            else
            {
                current_ekf_state.posHealth = false;
            }
        }

        // test height measurements
        if (fuseHgtData)
        {
            hgt_innov = statesAtHgtTime[9] + hgtMea;
            varInnovVelPos[5] = P[9][9] + r_obs[5];
            // apply a 10-sigma threshold
            current_ekf_state.hgtHealth = sq(hgt_innov) < 100.0f*varInnovVelPos[5];
            current_ekf_state.hgtTimeout = (millis() - current_ekf_state.hgtFailTime) > hgt_retry_time;
            if (current_ekf_state.hgtHealth || current_ekf_state.hgtTimeout || staticMode)
            {
                current_ekf_state.hgtHealth = true;
                current_ekf_state.hgtFailTime = millis();

                // if we just reset from a timeout, do not fuse
                // the height data, but reset height and stored states
                if (current_ekf_state.hgtTimeout) {
                    ResetHeight();
                    fuseHgtData = false;
                }
            }
            else
            {
                current_ekf_state.hgtHealth = false;
            }
        }
        // Set range for sequential fusion of velocity and position measurements depending
        // on which data is available and its health
        if (fuseVelData && fusionModeGPS == 0 && current_ekf_state.velHealth)
        {
            fuse_data[0] = true;
            fuse_data[1] = true;
            fuse_data[2] = true;
        }
        if (fuseVelData && fusionModeGPS == 1 && current_ekf_state.velHealth)
        {
            fuse_data[0] = true;
            fuse_data[1] = true;
        }
        if (fusePosData && fusionModeGPS <= 2 && current_ekf_state.posHealth)
        {
            fuse_data[3] = true;
            fuse_data[4] = true;
        }
        if (fuseHgtData && current_ekf_state.hgtHealth)
        {
            fuse_data[5] = true;
        }
        // Fuse measurements sequentially
        for (obs_index=0; obs_index<=5; obs_index++)
        {
            if (fuse_data[obs_index])
            {
                state_index = 4 + obs_index;
                // Calculate the measurement innovation, using states from a
                // different time coordinate if fusing height data
                if (obs_index <= 2)
                {
                    innovVelPos[obs_index] = statesAtVelTime[state_index] - observation[obs_index];
                }
                else if (obs_index == 3 || obs_index == 4)
                {
                    innovVelPos[obs_index] = statesAtPosTime[state_index] - observation[obs_index];
                }
                else if (obs_index == 5)
                {
                    innovVelPos[obs_index] = statesAtHgtTime[state_index] - observation[obs_index];
                }
                // Calculate the Kalman Gain
                // Calculate innovation variances - also used for data logging
                varInnovVelPos[obs_index] = P[state_index][state_index] + r_obs[obs_index];
                sk = 1.0/(double)varInnovVelPos[obs_index];
                for (uint8_t i= 0; i<=index_limit; i++)
                {
                    Kfusion[i] = P[i][state_index]*sk;
                }

                // Don't update Z accel bias state unless using a height observation (GPS velocities can be biased)
                if (obs_index != 5) {
                    Kfusion[13] = 0;
                }
                // Don't update wind states if inhibited
                if (inhibitWindStates) {
                    Kfusion[14] = 0;
                    Kfusion[15] = 0;
                }
                // Don't update magnetic field states if inhibited
                if (inhibitMagStates) {
                    for (uint8_t i = 16; i < ekf_state_estimates; i++)
                    {
                        Kfusion[i] = 0;
                    }
                }

                // Calculate state corrections and re-normalise the quaternions
                for (uint8_t i = 0; i<=index_limit; i++)
                {
                    states[i] = states[i] - Kfusion[i] * innovVelPos[obs_index];
                }
                quat_mag = sqrtf(states[0]*states[0] + states[1]*states[1] + states[2]*states[2] + states[3]*states[3]);
                if (quat_mag > 1e-12f) // divide by  0 protection
                {
                    for (uint8_t i = 0; i<=3; i++)
                    {
                        states[i] = states[i] / quat_mag;
                    }
                }
                // Update the covariance - take advantage of direct observation of a
                // single state at index = stateIndex to reduce computations
                // Optimised implementation of standard equation P = (I - K*H)*P;
                for (uint8_t i= 0; i<=index_limit; i++)
                {
                    for (uint8_t j= 0; j<=index_limit; j++)
                    {
                        KHP[i][j] = Kfusion[i] * P[state_index][j];
                    }
                }
                for (uint8_t i= 0; i<=index_limit; i++)
                {
                    for (uint8_t j= 0; j<=index_limit; j++)
                    {
                        P[i][j] = P[i][j] - KHP[i][j];
                    }
                }
            }
        }
    }

    ForceSymmetry();
    ConstrainVariances();

}

void AttPosEKF::fuseMagnetometer()
{

    float &q0 = magstate.q0;
    float &q1 = magstate.q1;
    float &q2 = magstate.q2;
    float &q3 = magstate.q3;
    float &mag_n = magstate.magN;
    float &mag_e = magstate.magE;
    float &mag_d = magstate.magD;
    float &mag_xbias = magstate.magXbias;
    float &mag_ybias = magstate.magYbias;
    float &mag_zbias = magstate.magZbias;
    unsigned &obs_index = magstate.obsIndex;
    Mat3f &dcm = magstate.DCM;
    float *mag_pred = &magstate.MagPred[0];
    float &r_mag = magstate.R_MAG;
    float *sh_mag = &magstate.SH_MAG[0];

    float sk_mx[6];
    float sk_my[5];
    float sk_mz[6];
    float h_mag[ekf_state_estimates];
    for (uint8_t i = 0; i < ekf_state_estimates; i++) {
        h_mag[i] = 0.0f;
    }

    // Perform sequential fusion of Magnetometer measurements.
    // This assumes that the errors in the different components are
    // uncorrelated which is not true, however in the absence of covariance
    // data fit is the only assumption we can make
    // so we might as well take advantage of the computational efficiencies
    // associated with sequential fusion
    if (useCompass && fuseMagData && (obs_index < 3))
    {
        // Calculate observation jacobians and Kalman gains
        if (obs_index == 0)
        {
            // Copy required states to local variable names
            q0       = statesAtMagMeasTime[0];
            q1       = statesAtMagMeasTime[1];
            q2       = statesAtMagMeasTime[2];
            q3       = statesAtMagMeasTime[3];
            mag_n     = statesAtMagMeasTime[16];
            mag_e     = statesAtMagMeasTime[17];
            mag_d     = statesAtMagMeasTime[18];
            mag_xbias = statesAtMagMeasTime[19];
            mag_ybias = statesAtMagMeasTime[20];
            mag_zbias = statesAtMagMeasTime[21];

            // rotate predicted earth components into body axes and calculate
            // predicted measurments
            dcm.x.x = q0*q0 + q1*q1 - q2*q2 - q3*q3;
            dcm.x.y = 2*(q1*q2 + q0*q3);
            dcm.x.z = 2*(q1*q3-q0*q2);
            dcm.y.x = 2*(q1*q2 - q0*q3);
            dcm.y.y = q0*q0 - q1*q1 + q2*q2 - q3*q3;
            dcm.y.z = 2*(q2*q3 + q0*q1);
            dcm.z.x = 2*(q1*q3 + q0*q2);
            dcm.z.y = 2*(q2*q3 - q0*q1);
            dcm.z.z = q0*q0 - q1*q1 - q2*q2 + q3*q3;
            mag_pred[0] = dcm.x.x*mag_n + dcm.x.y*mag_e  + dcm.x.z*mag_d + mag_xbias;
            mag_pred[1] = dcm.y.x*mag_n + dcm.y.y*mag_e  + dcm.y.z*mag_d + mag_ybias;
            mag_pred[2] = dcm.z.x*mag_n + dcm.z.y*mag_e  + dcm.z.z*mag_d + mag_zbias;

            // scale magnetometer observation error with total angular rate
            r_mag = sq(magMeasurementSigma) + sq(0.05f*dAngIMU.length()/dtIMU);

            // Calculate observation jacobians
            sh_mag[0] = 2*mag_d*q3 + 2*mag_e*q2 + 2*mag_n*q1;
            sh_mag[1] = 2*mag_d*q0 - 2*mag_e*q1 + 2*mag_n*q2;
            sh_mag[2] = 2*mag_d*q1 + 2*mag_e*q0 - 2*mag_n*q3;
            sh_mag[3] = sq(q3);
            sh_mag[4] = sq(q2);
            sh_mag[5] = sq(q1);
            sh_mag[6] = sq(q0);
            sh_mag[7] = 2*mag_n*q0;
            sh_mag[8] = 2*mag_e*q3;

            for (uint8_t i = 0; i < ekf_state_estimates; i++) h_mag[i] = 0;
            h_mag[0] = sh_mag[7] + sh_mag[8] - 2*mag_d*q2;
            h_mag[1] = sh_mag[0];
            h_mag[2] = 2*mag_e*q1 - 2*mag_d*q0 - 2*mag_n*q2;
            h_mag[3] = sh_mag[2];
            h_mag[16] = sh_mag[5] - sh_mag[4] - sh_mag[3] + sh_mag[6];
            h_mag[17] = 2*q0*q3 + 2*q1*q2;
            h_mag[18] = 2*q1*q3 - 2*q0*q2;
            h_mag[19] = 1.0f;

            // Calculate Kalman gain
            float temp = (P[19][19] + r_mag + P[1][19]*sh_mag[0] + P[3][19]*sh_mag[2] - P[16][19]*(sh_mag[3] + sh_mag[4] - sh_mag[5] - sh_mag[6]) - (2*mag_d*q0 - 2*mag_e*q1 + 2*mag_n*q2)*(P[19][2] + P[1][2]*sh_mag[0] + P[3][2]*sh_mag[2] - P[16][2]*(sh_mag[3] + sh_mag[4] - sh_mag[5] - sh_mag[6]) + P[17][2]*(2*q0*q3 + 2*q1*q2) - P[18][2]*(2*q0*q2 - 2*q1*q3) - P[2][2]*(2*mag_d*q0 - 2*mag_e*q1 + 2*mag_n*q2) + P[0][2]*(sh_mag[7] + sh_mag[8] - 2*mag_d*q2)) + (sh_mag[7] + sh_mag[8] - 2*mag_d*q2)*(P[19][0] + P[1][0]*sh_mag[0] + P[3][0]*sh_mag[2] - P[16][0]*(sh_mag[3] + sh_mag[4] - sh_mag[5] - sh_mag[6]) + P[17][0]*(2*q0*q3 + 2*q1*q2) - P[18][0]*(2*q0*q2 - 2*q1*q3) - P[2][0]*(2*mag_d*q0 - 2*mag_e*q1 + 2*mag_n*q2) + P[0][0]*(sh_mag[7] + sh_mag[8] - 2*mag_d*q2)) + sh_mag[0]*(P[19][1] + P[1][1]*sh_mag[0] + P[3][1]*sh_mag[2] - P[16][1]*(sh_mag[3] + sh_mag[4] - sh_mag[5] - sh_mag[6]) + P[17][1]*(2*q0*q3 + 2*q1*q2) - P[18][1]*(2*q0*q2 - 2*q1*q3) - P[2][1]*(2*mag_d*q0 - 2*mag_e*q1 + 2*mag_n*q2) + P[0][1]*(sh_mag[7] + sh_mag[8] - 2*mag_d*q2)) + sh_mag[2]*(P[19][3] + P[1][3]*sh_mag[0] + P[3][3]*sh_mag[2] - P[16][3]*(sh_mag[3] + sh_mag[4] - sh_mag[5] - sh_mag[6]) + P[17][3]*(2*q0*q3 + 2*q1*q2) - P[18][3]*(2*q0*q2 - 2*q1*q3) - P[2][3]*(2*mag_d*q0 - 2*mag_e*q1 + 2*mag_n*q2) + P[0][3]*(sh_mag[7] + sh_mag[8] - 2*mag_d*q2)) - (sh_mag[3] + sh_mag[4] - sh_mag[5] - sh_mag[6])*(P[19][16] + P[1][16]*sh_mag[0] + P[3][16]*sh_mag[2] - P[16][16]*(sh_mag[3] + sh_mag[4] - sh_mag[5] - sh_mag[6]) + P[17][16]*(2*q0*q3 + 2*q1*q2) - P[18][16]*(2*q0*q2 - 2*q1*q3) - P[2][16]*(2*mag_d*q0 - 2*mag_e*q1 + 2*mag_n*q2) + P[0][16]*(sh_mag[7] + sh_mag[8] - 2*mag_d*q2)) + P[17][19]*(2*q0*q3 + 2*q1*q2) - P[18][19]*(2*q0*q2 - 2*q1*q3) - P[2][19]*(2*mag_d*q0 - 2*mag_e*q1 + 2*mag_n*q2) + (2*q0*q3 + 2*q1*q2)*(P[19][17] + P[1][17]*sh_mag[0] + P[3][17]*sh_mag[2] - P[16][17]*(sh_mag[3] + sh_mag[4] - sh_mag[5] - sh_mag[6]) + P[17][17]*(2*q0*q3 + 2*q1*q2) - P[18][17]*(2*q0*q2 - 2*q1*q3) - P[2][17]*(2*mag_d*q0 - 2*mag_e*q1 + 2*mag_n*q2) + P[0][17]*(sh_mag[7] + sh_mag[8] - 2*mag_d*q2)) - (2*q0*q2 - 2*q1*q3)*(P[19][18] + P[1][18]*sh_mag[0] + P[3][18]*sh_mag[2] - P[16][18]*(sh_mag[3] + sh_mag[4] - sh_mag[5] - sh_mag[6]) + P[17][18]*(2*q0*q3 + 2*q1*q2) - P[18][18]*(2*q0*q2 - 2*q1*q3) - P[2][18]*(2*mag_d*q0 - 2*mag_e*q1 + 2*mag_n*q2) + P[0][18]*(sh_mag[7] + sh_mag[8] - 2*mag_d*q2)) + P[0][19]*(sh_mag[7] + sh_mag[8] - 2*mag_d*q2));
            if (temp >= r_mag) {
                sk_mx[0] = 1.0f / temp;
            } else {
                // the calculation is badly conditioned, so we cannot perform fusion on this step
                // we increase the state variances and try again next time
                P[19][19] += 0.1f*r_mag;
                obs_index = 1;
                return;
            }
            sk_mx[1] = sh_mag[3] + sh_mag[4] - sh_mag[5] - sh_mag[6];
            sk_mx[2] = 2*mag_d*q0 - 2*mag_e*q1 + 2*mag_n*q2;
            sk_mx[3] = sh_mag[7] + sh_mag[8] - 2*mag_d*q2;
            sk_mx[4] = 2*q0*q2 - 2*q1*q3;
            sk_mx[5] = 2*q0*q3 + 2*q1*q2;
            Kfusion[0] = sk_mx[0]*(P[0][19] + P[0][1]*sh_mag[0] + P[0][3]*sh_mag[2] + P[0][0]*sk_mx[3] - P[0][2]*sk_mx[2] - P[0][16]*sk_mx[1] + P[0][17]*sk_mx[5] - P[0][18]*sk_mx[4]);
            Kfusion[1] = sk_mx[0]*(P[1][19] + P[1][1]*sh_mag[0] + P[1][3]*sh_mag[2] + P[1][0]*sk_mx[3] - P[1][2]*sk_mx[2] - P[1][16]*sk_mx[1] + P[1][17]*sk_mx[5] - P[1][18]*sk_mx[4]);
            Kfusion[2] = sk_mx[0]*(P[2][19] + P[2][1]*sh_mag[0] + P[2][3]*sh_mag[2] + P[2][0]*sk_mx[3] - P[2][2]*sk_mx[2] - P[2][16]*sk_mx[1] + P[2][17]*sk_mx[5] - P[2][18]*sk_mx[4]);
            Kfusion[3] = sk_mx[0]*(P[3][19] + P[3][1]*sh_mag[0] + P[3][3]*sh_mag[2] + P[3][0]*sk_mx[3] - P[3][2]*sk_mx[2] - P[3][16]*sk_mx[1] + P[3][17]*sk_mx[5] - P[3][18]*sk_mx[4]);
            Kfusion[4] = sk_mx[0]*(P[4][19] + P[4][1]*sh_mag[0] + P[4][3]*sh_mag[2] + P[4][0]*sk_mx[3] - P[4][2]*sk_mx[2] - P[4][16]*sk_mx[1] + P[4][17]*sk_mx[5] - P[4][18]*sk_mx[4]);
            Kfusion[5] = sk_mx[0]*(P[5][19] + P[5][1]*sh_mag[0] + P[5][3]*sh_mag[2] + P[5][0]*sk_mx[3] - P[5][2]*sk_mx[2] - P[5][16]*sk_mx[1] + P[5][17]*sk_mx[5] - P[5][18]*sk_mx[4]);
            Kfusion[6] = sk_mx[0]*(P[6][19] + P[6][1]*sh_mag[0] + P[6][3]*sh_mag[2] + P[6][0]*sk_mx[3] - P[6][2]*sk_mx[2] - P[6][16]*sk_mx[1] + P[6][17]*sk_mx[5] - P[6][18]*sk_mx[4]);
            Kfusion[7] = sk_mx[0]*(P[7][19] + P[7][1]*sh_mag[0] + P[7][3]*sh_mag[2] + P[7][0]*sk_mx[3] - P[7][2]*sk_mx[2] - P[7][16]*sk_mx[1] + P[7][17]*sk_mx[5] - P[7][18]*sk_mx[4]);
            Kfusion[8] = sk_mx[0]*(P[8][19] + P[8][1]*sh_mag[0] + P[8][3]*sh_mag[2] + P[8][0]*sk_mx[3] - P[8][2]*sk_mx[2] - P[8][16]*sk_mx[1] + P[8][17]*sk_mx[5] - P[8][18]*sk_mx[4]);
            Kfusion[9] = sk_mx[0]*(P[9][19] + P[9][1]*sh_mag[0] + P[9][3]*sh_mag[2] + P[9][0]*sk_mx[3] - P[9][2]*sk_mx[2] - P[9][16]*sk_mx[1] + P[9][17]*sk_mx[5] - P[9][18]*sk_mx[4]);
            Kfusion[10] = sk_mx[0]*(P[10][19] + P[10][1]*sh_mag[0] + P[10][3]*sh_mag[2] + P[10][0]*sk_mx[3] - P[10][2]*sk_mx[2] - P[10][16]*sk_mx[1] + P[10][17]*sk_mx[5] - P[10][18]*sk_mx[4]);
            Kfusion[11] = sk_mx[0]*(P[11][19] + P[11][1]*sh_mag[0] + P[11][3]*sh_mag[2] + P[11][0]*sk_mx[3] - P[11][2]*sk_mx[2] - P[11][16]*sk_mx[1] + P[11][17]*sk_mx[5] - P[11][18]*sk_mx[4]);
            Kfusion[12] = sk_mx[0]*(P[12][19] + P[12][1]*sh_mag[0] + P[12][3]*sh_mag[2] + P[12][0]*sk_mx[3] - P[12][2]*sk_mx[2] - P[12][16]*sk_mx[1] + P[12][17]*sk_mx[5] - P[12][18]*sk_mx[4]);
            // Only height measurements are allowed to modify the Z delta velocity bias state. This improves the stability of the estimate
            Kfusion[13] = 0.0f;//SK_MX[0]*(P[13][19] + P[13][1]*SH_MAG[0] + P[13][3]*SH_MAG[2] + P[13][0]*SK_MX[3] - P[13][2]*SK_MX[2] - P[13][16]*SK_MX[1] + P[13][17]*SK_MX[5] - P[13][18]*SK_MX[4]);
            // Estimation of selected states is inhibited by setting their Kalman gains to zero
            if (!inhibitWindStates) {
                Kfusion[14] = sk_mx[0]*(P[14][19] + P[14][1]*sh_mag[0] + P[14][3]*sh_mag[2] + P[14][0]*sk_mx[3] - P[14][2]*sk_mx[2] - P[14][16]*sk_mx[1] + P[14][17]*sk_mx[5] - P[14][18]*sk_mx[4]);
                Kfusion[15] = sk_mx[0]*(P[15][19] + P[15][1]*sh_mag[0] + P[15][3]*sh_mag[2] + P[15][0]*sk_mx[3] - P[15][2]*sk_mx[2] - P[15][16]*sk_mx[1] + P[15][17]*sk_mx[5] - P[15][18]*sk_mx[4]);
            } else {
                Kfusion[14] = 0;
                Kfusion[15] = 0;
            }
            if (!inhibitMagStates) {
                Kfusion[16] = sk_mx[0]*(P[16][19] + P[16][1]*sh_mag[0] + P[16][3]*sh_mag[2] + P[16][0]*sk_mx[3] - P[16][2]*sk_mx[2] - P[16][16]*sk_mx[1] + P[16][17]*sk_mx[5] - P[16][18]*sk_mx[4]);
                Kfusion[17] = sk_mx[0]*(P[17][19] + P[17][1]*sh_mag[0] + P[17][3]*sh_mag[2] + P[17][0]*sk_mx[3] - P[17][2]*sk_mx[2] - P[17][16]*sk_mx[1] + P[17][17]*sk_mx[5] - P[17][18]*sk_mx[4]);
                Kfusion[18] = sk_mx[0]*(P[18][19] + P[18][1]*sh_mag[0] + P[18][3]*sh_mag[2] + P[18][0]*sk_mx[3] - P[18][2]*sk_mx[2] - P[18][16]*sk_mx[1] + P[18][17]*sk_mx[5] - P[18][18]*sk_mx[4]);
                Kfusion[19] = sk_mx[0]*(P[19][19] + P[19][1]*sh_mag[0] + P[19][3]*sh_mag[2] + P[19][0]*sk_mx[3] - P[19][2]*sk_mx[2] - P[19][16]*sk_mx[1] + P[19][17]*sk_mx[5] - P[19][18]*sk_mx[4]);
                Kfusion[20] = sk_mx[0]*(P[20][19] + P[20][1]*sh_mag[0] + P[20][3]*sh_mag[2] + P[20][0]*sk_mx[3] - P[20][2]*sk_mx[2] - P[20][16]*sk_mx[1] + P[20][17]*sk_mx[5] - P[20][18]*sk_mx[4]);
                Kfusion[21] = sk_mx[0]*(P[21][19] + P[21][1]*sh_mag[0] + P[21][3]*sh_mag[2] + P[21][0]*sk_mx[3] - P[21][2]*sk_mx[2] - P[21][16]*sk_mx[1] + P[21][17]*sk_mx[5] - P[21][18]*sk_mx[4]);
            } else {
                for (uint8_t i=16; i < ekf_state_estimates; i++) {
                    Kfusion[i] = 0;
                }
            }
            varInnovMag[0] = 1.0f/sk_mx[0];
            innovMag[0] = mag_pred[0] - magData.x;
        }
        else if (obs_index == 1) // we are now fusing the Y measurement
        {
            // Calculate observation jacobians
            for (size_t i = 0; i < ekf_state_estimates; i++) h_mag[i] = 0;
            h_mag[0] = sh_mag[2];
            h_mag[1] = sh_mag[1];
            h_mag[2] = sh_mag[0];
            h_mag[3] = 2*mag_d*q2 - sh_mag[8] - sh_mag[7];
            h_mag[16] = 2*q1*q2 - 2*q0*q3;
            h_mag[17] = sh_mag[4] - sh_mag[3] - sh_mag[5] + sh_mag[6];
            h_mag[18] = 2*q0*q1 + 2*q2*q3;
            h_mag[20] = 1;

            // Calculate Kalman gain
            float temp = (P[20][20] + r_mag + P[0][20]*sh_mag[2] + P[1][20]*sh_mag[1] + P[2][20]*sh_mag[0] - P[17][20]*(sh_mag[3] - sh_mag[4] + sh_mag[5] - sh_mag[6]) - (2*q0*q3 - 2*q1*q2)*(P[20][16] + P[0][16]*sh_mag[2] + P[1][16]*sh_mag[1] + P[2][16]*sh_mag[0] - P[17][16]*(sh_mag[3] - sh_mag[4] + sh_mag[5] - sh_mag[6]) - P[16][16]*(2*q0*q3 - 2*q1*q2) + P[18][16]*(2*q0*q1 + 2*q2*q3) - P[3][16]*(sh_mag[7] + sh_mag[8] - 2*mag_d*q2)) + (2*q0*q1 + 2*q2*q3)*(P[20][18] + P[0][18]*sh_mag[2] + P[1][18]*sh_mag[1] + P[2][18]*sh_mag[0] - P[17][18]*(sh_mag[3] - sh_mag[4] + sh_mag[5] - sh_mag[6]) - P[16][18]*(2*q0*q3 - 2*q1*q2) + P[18][18]*(2*q0*q1 + 2*q2*q3) - P[3][18]*(sh_mag[7] + sh_mag[8] - 2*mag_d*q2)) - (sh_mag[7] + sh_mag[8] - 2*mag_d*q2)*(P[20][3] + P[0][3]*sh_mag[2] + P[1][3]*sh_mag[1] + P[2][3]*sh_mag[0] - P[17][3]*(sh_mag[3] - sh_mag[4] + sh_mag[5] - sh_mag[6]) - P[16][3]*(2*q0*q3 - 2*q1*q2) + P[18][3]*(2*q0*q1 + 2*q2*q3) - P[3][3]*(sh_mag[7] + sh_mag[8] - 2*mag_d*q2)) - P[16][20]*(2*q0*q3 - 2*q1*q2) + P[18][20]*(2*q0*q1 + 2*q2*q3) + sh_mag[2]*(P[20][0] + P[0][0]*sh_mag[2] + P[1][0]*sh_mag[1] + P[2][0]*sh_mag[0] - P[17][0]*(sh_mag[3] - sh_mag[4] + sh_mag[5] - sh_mag[6]) - P[16][0]*(2*q0*q3 - 2*q1*q2) + P[18][0]*(2*q0*q1 + 2*q2*q3) - P[3][0]*(sh_mag[7] + sh_mag[8] - 2*mag_d*q2)) + sh_mag[1]*(P[20][1] + P[0][1]*sh_mag[2] + P[1][1]*sh_mag[1] + P[2][1]*sh_mag[0] - P[17][1]*(sh_mag[3] - sh_mag[4] + sh_mag[5] - sh_mag[6]) - P[16][1]*(2*q0*q3 - 2*q1*q2) + P[18][1]*(2*q0*q1 + 2*q2*q3) - P[3][1]*(sh_mag[7] + sh_mag[8] - 2*mag_d*q2)) + sh_mag[0]*(P[20][2] + P[0][2]*sh_mag[2] + P[1][2]*sh_mag[1] + P[2][2]*sh_mag[0] - P[17][2]*(sh_mag[3] - sh_mag[4] + sh_mag[5] - sh_mag[6]) - P[16][2]*(2*q0*q3 - 2*q1*q2) + P[18][2]*(2*q0*q1 + 2*q2*q3) - P[3][2]*(sh_mag[7] + sh_mag[8] - 2*mag_d*q2)) - (sh_mag[3] - sh_mag[4] + sh_mag[5] - sh_mag[6])*(P[20][17] + P[0][17]*sh_mag[2] + P[1][17]*sh_mag[1] + P[2][17]*sh_mag[0] - P[17][17]*(sh_mag[3] - sh_mag[4] + sh_mag[5] - sh_mag[6]) - P[16][17]*(2*q0*q3 - 2*q1*q2) + P[18][17]*(2*q0*q1 + 2*q2*q3) - P[3][17]*(sh_mag[7] + sh_mag[8] - 2*mag_d*q2)) - P[3][20]*(sh_mag[7] + sh_mag[8] - 2*mag_d*q2));
            if (temp >= r_mag) {
                sk_my[0] = 1.0f / temp;
            } else {
                // the calculation is badly conditioned, so we cannot perform fusion on this step
                // we increase the state variances and try again next time
                P[20][20] += 0.1f*r_mag;
                obs_index = 2;
                return;
            }
            sk_my[1] = sh_mag[3] - sh_mag[4] + sh_mag[5] - sh_mag[6];
            sk_my[2] = sh_mag[7] + sh_mag[8] - 2*mag_d*q2;
            sk_my[3] = 2*q0*q3 - 2*q1*q2;
            sk_my[4] = 2*q0*q1 + 2*q2*q3;
            Kfusion[0] = sk_my[0]*(P[0][20] + P[0][0]*sh_mag[2] + P[0][1]*sh_mag[1] + P[0][2]*sh_mag[0] - P[0][3]*sk_my[2] - P[0][17]*sk_my[1] - P[0][16]*sk_my[3] + P[0][18]*sk_my[4]);
            Kfusion[1] = sk_my[0]*(P[1][20] + P[1][0]*sh_mag[2] + P[1][1]*sh_mag[1] + P[1][2]*sh_mag[0] - P[1][3]*sk_my[2] - P[1][17]*sk_my[1] - P[1][16]*sk_my[3] + P[1][18]*sk_my[4]);
            Kfusion[2] = sk_my[0]*(P[2][20] + P[2][0]*sh_mag[2] + P[2][1]*sh_mag[1] + P[2][2]*sh_mag[0] - P[2][3]*sk_my[2] - P[2][17]*sk_my[1] - P[2][16]*sk_my[3] + P[2][18]*sk_my[4]);
            Kfusion[3] = sk_my[0]*(P[3][20] + P[3][0]*sh_mag[2] + P[3][1]*sh_mag[1] + P[3][2]*sh_mag[0] - P[3][3]*sk_my[2] - P[3][17]*sk_my[1] - P[3][16]*sk_my[3] + P[3][18]*sk_my[4]);
            Kfusion[4] = sk_my[0]*(P[4][20] + P[4][0]*sh_mag[2] + P[4][1]*sh_mag[1] + P[4][2]*sh_mag[0] - P[4][3]*sk_my[2] - P[4][17]*sk_my[1] - P[4][16]*sk_my[3] + P[4][18]*sk_my[4]);
            Kfusion[5] = sk_my[0]*(P[5][20] + P[5][0]*sh_mag[2] + P[5][1]*sh_mag[1] + P[5][2]*sh_mag[0] - P[5][3]*sk_my[2] - P[5][17]*sk_my[1] - P[5][16]*sk_my[3] + P[5][18]*sk_my[4]);
            Kfusion[6] = sk_my[0]*(P[6][20] + P[6][0]*sh_mag[2] + P[6][1]*sh_mag[1] + P[6][2]*sh_mag[0] - P[6][3]*sk_my[2] - P[6][17]*sk_my[1] - P[6][16]*sk_my[3] + P[6][18]*sk_my[4]);
            Kfusion[7] = sk_my[0]*(P[7][20] + P[7][0]*sh_mag[2] + P[7][1]*sh_mag[1] + P[7][2]*sh_mag[0] - P[7][3]*sk_my[2] - P[7][17]*sk_my[1] - P[7][16]*sk_my[3] + P[7][18]*sk_my[4]);
            Kfusion[8] = sk_my[0]*(P[8][20] + P[8][0]*sh_mag[2] + P[8][1]*sh_mag[1] + P[8][2]*sh_mag[0] - P[8][3]*sk_my[2] - P[8][17]*sk_my[1] - P[8][16]*sk_my[3] + P[8][18]*sk_my[4]);
            Kfusion[9] = sk_my[0]*(P[9][20] + P[9][0]*sh_mag[2] + P[9][1]*sh_mag[1] + P[9][2]*sh_mag[0] - P[9][3]*sk_my[2] - P[9][17]*sk_my[1] - P[9][16]*sk_my[3] + P[9][18]*sk_my[4]);
            Kfusion[10] = sk_my[0]*(P[10][20] + P[10][0]*sh_mag[2] + P[10][1]*sh_mag[1] + P[10][2]*sh_mag[0] - P[10][3]*sk_my[2] - P[10][17]*sk_my[1] - P[10][16]*sk_my[3] + P[10][18]*sk_my[4]);
            Kfusion[11] = sk_my[0]*(P[11][20] + P[11][0]*sh_mag[2] + P[11][1]*sh_mag[1] + P[11][2]*sh_mag[0] - P[11][3]*sk_my[2] - P[11][17]*sk_my[1] - P[11][16]*sk_my[3] + P[11][18]*sk_my[4]);
            Kfusion[12] = sk_my[0]*(P[12][20] + P[12][0]*sh_mag[2] + P[12][1]*sh_mag[1] + P[12][2]*sh_mag[0] - P[12][3]*sk_my[2] - P[12][17]*sk_my[1] - P[12][16]*sk_my[3] + P[12][18]*sk_my[4]);
            // Only height measurements are allowed to modify the Z delta velocity bias state. This improves the stability of the estimate
            Kfusion[13] = 0.0f;//SK_MY[0]*(P[13][20] + P[13][0]*SH_MAG[2] + P[13][1]*SH_MAG[1] + P[13][2]*SH_MAG[0] - P[13][3]*SK_MY[2] - P[13][17]*SK_MY[1] - P[13][16]*SK_MY[3] + P[13][18]*SK_MY[4]);
            // Estimation of selected states is inhibited by setting their Kalman gains to zero
            if (!inhibitWindStates) {
                Kfusion[14] = sk_my[0]*(P[14][20] + P[14][0]*sh_mag[2] + P[14][1]*sh_mag[1] + P[14][2]*sh_mag[0] - P[14][3]*sk_my[2] - P[14][17]*sk_my[1] - P[14][16]*sk_my[3] + P[14][18]*sk_my[4]);
                Kfusion[15] = sk_my[0]*(P[15][20] + P[15][0]*sh_mag[2] + P[15][1]*sh_mag[1] + P[15][2]*sh_mag[0] - P[15][3]*sk_my[2] - P[15][17]*sk_my[1] - P[15][16]*sk_my[3] + P[15][18]*sk_my[4]);
            } else {
                Kfusion[14] = 0;
                Kfusion[15] = 0;
            }
            if (!inhibitMagStates) {
                Kfusion[16] = sk_my[0]*(P[16][20] + P[16][0]*sh_mag[2] + P[16][1]*sh_mag[1] + P[16][2]*sh_mag[0] - P[16][3]*sk_my[2] - P[16][17]*sk_my[1] - P[16][16]*sk_my[3] + P[16][18]*sk_my[4]);
                Kfusion[17] = sk_my[0]*(P[17][20] + P[17][0]*sh_mag[2] + P[17][1]*sh_mag[1] + P[17][2]*sh_mag[0] - P[17][3]*sk_my[2] - P[17][17]*sk_my[1] - P[17][16]*sk_my[3] + P[17][18]*sk_my[4]);
                Kfusion[18] = sk_my[0]*(P[18][20] + P[18][0]*sh_mag[2] + P[18][1]*sh_mag[1] + P[18][2]*sh_mag[0] - P[18][3]*sk_my[2] - P[18][17]*sk_my[1] - P[18][16]*sk_my[3] + P[18][18]*sk_my[4]);
                Kfusion[19] = sk_my[0]*(P[19][20] + P[19][0]*sh_mag[2] + P[19][1]*sh_mag[1] + P[19][2]*sh_mag[0] - P[19][3]*sk_my[2] - P[19][17]*sk_my[1] - P[19][16]*sk_my[3] + P[19][18]*sk_my[4]);
                Kfusion[20] = sk_my[0]*(P[20][20] + P[20][0]*sh_mag[2] + P[20][1]*sh_mag[1] + P[20][2]*sh_mag[0] - P[20][3]*sk_my[2] - P[20][17]*sk_my[1] - P[20][16]*sk_my[3] + P[20][18]*sk_my[4]);
                Kfusion[21] = sk_my[0]*(P[21][20] + P[21][0]*sh_mag[2] + P[21][1]*sh_mag[1] + P[21][2]*sh_mag[0] - P[21][3]*sk_my[2] - P[21][17]*sk_my[1] - P[21][16]*sk_my[3] + P[21][18]*sk_my[4]);
            } else {
                Kfusion[16] = 0;
                Kfusion[17] = 0;
                Kfusion[18] = 0;
                Kfusion[19] = 0;
                Kfusion[20] = 0;
                Kfusion[21] = 0;
            }
            varInnovMag[1] = 1.0f/sk_my[0];
            innovMag[1] = mag_pred[1] - magData.y;
        }
        else if (obs_index == 2) // we are now fusing the Z measurement
        {
            // Calculate observation jacobians
            for (uint8_t i = 0; i < ekf_state_estimates; i++) h_mag[i] = 0;
            h_mag[0] = sh_mag[1];
            h_mag[1] = 2*mag_n*q3 - 2*mag_e*q0 - 2*mag_d*q1;
            h_mag[2] = sh_mag[7] + sh_mag[8] - 2*mag_d*q2;
            h_mag[3] = sh_mag[0];
            h_mag[16] = 2*q0*q2 + 2*q1*q3;
            h_mag[17] = 2*q2*q3 - 2*q0*q1;
            h_mag[18] = sh_mag[3] - sh_mag[4] - sh_mag[5] + sh_mag[6];
            h_mag[21] = 1;

            // Calculate Kalman gain
            float temp = (P[21][21] + r_mag + P[0][21]*sh_mag[1] + P[3][21]*sh_mag[0] + P[18][21]*(sh_mag[3] - sh_mag[4] - sh_mag[5] + sh_mag[6]) - (2*mag_d*q1 + 2*mag_e*q0 - 2*mag_n*q3)*(P[21][1] + P[0][1]*sh_mag[1] + P[3][1]*sh_mag[0] + P[18][1]*(sh_mag[3] - sh_mag[4] - sh_mag[5] + sh_mag[6]) + P[16][1]*(2*q0*q2 + 2*q1*q3) - P[17][1]*(2*q0*q1 - 2*q2*q3) - P[1][1]*(2*mag_d*q1 + 2*mag_e*q0 - 2*mag_n*q3) + P[2][1]*(sh_mag[7] + sh_mag[8] - 2*mag_d*q2)) + (sh_mag[7] + sh_mag[8] - 2*mag_d*q2)*(P[21][2] + P[0][2]*sh_mag[1] + P[3][2]*sh_mag[0] + P[18][2]*(sh_mag[3] - sh_mag[4] - sh_mag[5] + sh_mag[6]) + P[16][2]*(2*q0*q2 + 2*q1*q3) - P[17][2]*(2*q0*q1 - 2*q2*q3) - P[1][2]*(2*mag_d*q1 + 2*mag_e*q0 - 2*mag_n*q3) + P[2][2]*(sh_mag[7] + sh_mag[8] - 2*mag_d*q2)) + sh_mag[1]*(P[21][0] + P[0][0]*sh_mag[1] + P[3][0]*sh_mag[0] + P[18][0]*(sh_mag[3] - sh_mag[4] - sh_mag[5] + sh_mag[6]) + P[16][0]*(2*q0*q2 + 2*q1*q3) - P[17][0]*(2*q0*q1 - 2*q2*q3) - P[1][0]*(2*mag_d*q1 + 2*mag_e*q0 - 2*mag_n*q3) + P[2][0]*(sh_mag[7] + sh_mag[8] - 2*mag_d*q2)) + sh_mag[0]*(P[21][3] + P[0][3]*sh_mag[1] + P[3][3]*sh_mag[0] + P[18][3]*(sh_mag[3] - sh_mag[4] - sh_mag[5] + sh_mag[6]) + P[16][3]*(2*q0*q2 + 2*q1*q3) - P[17][3]*(2*q0*q1 - 2*q2*q3) - P[1][3]*(2*mag_d*q1 + 2*mag_e*q0 - 2*mag_n*q3) + P[2][3]*(sh_mag[7] + sh_mag[8] - 2*mag_d*q2)) + (sh_mag[3] - sh_mag[4] - sh_mag[5] + sh_mag[6])*(P[21][18] + P[0][18]*sh_mag[1] + P[3][18]*sh_mag[0] + P[18][18]*(sh_mag[3] - sh_mag[4] - sh_mag[5] + sh_mag[6]) + P[16][18]*(2*q0*q2 + 2*q1*q3) - P[17][18]*(2*q0*q1 - 2*q2*q3) - P[1][18]*(2*mag_d*q1 + 2*mag_e*q0 - 2*mag_n*q3) + P[2][18]*(sh_mag[7] + sh_mag[8] - 2*mag_d*q2)) + P[16][21]*(2*q0*q2 + 2*q1*q3) - P[17][21]*(2*q0*q1 - 2*q2*q3) - P[1][21]*(2*mag_d*q1 + 2*mag_e*q0 - 2*mag_n*q3) + (2*q0*q2 + 2*q1*q3)*(P[21][16] + P[0][16]*sh_mag[1] + P[3][16]*sh_mag[0] + P[18][16]*(sh_mag[3] - sh_mag[4] - sh_mag[5] + sh_mag[6]) + P[16][16]*(2*q0*q2 + 2*q1*q3) - P[17][16]*(2*q0*q1 - 2*q2*q3) - P[1][16]*(2*mag_d*q1 + 2*mag_e*q0 - 2*mag_n*q3) + P[2][16]*(sh_mag[7] + sh_mag[8] - 2*mag_d*q2)) - (2*q0*q1 - 2*q2*q3)*(P[21][17] + P[0][17]*sh_mag[1] + P[3][17]*sh_mag[0] + P[18][17]*(sh_mag[3] - sh_mag[4] - sh_mag[5] + sh_mag[6]) + P[16][17]*(2*q0*q2 + 2*q1*q3) - P[17][17]*(2*q0*q1 - 2*q2*q3) - P[1][17]*(2*mag_d*q1 + 2*mag_e*q0 - 2*mag_n*q3) + P[2][17]*(sh_mag[7] + sh_mag[8] - 2*mag_d*q2)) + P[2][21]*(sh_mag[7] + sh_mag[8] - 2*mag_d*q2));
            if (temp >= r_mag) {
                sk_mz[0] = 1.0f / temp;
            } else {
                // the calculation is badly conditioned, so we cannot perform fusion on this step
                // we increase the state variances and try again next time
                P[21][21] += 0.1f*r_mag;
                obs_index = 3;
                return;
            }
            sk_mz[1] = sh_mag[3] - sh_mag[4] - sh_mag[5] + sh_mag[6];
            sk_mz[2] = 2*mag_d*q1 + 2*mag_e*q0 - 2*mag_n*q3;
            sk_mz[3] = sh_mag[7] + sh_mag[8] - 2*mag_d*q2;
            sk_mz[4] = 2*q0*q1 - 2*q2*q3;
            sk_mz[5] = 2*q0*q2 + 2*q1*q3;
            Kfusion[0] = sk_mz[0]*(P[0][21] + P[0][0]*sh_mag[1] + P[0][3]*sh_mag[0] - P[0][1]*sk_mz[2] + P[0][2]*sk_mz[3] + P[0][18]*sk_mz[1] + P[0][16]*sk_mz[5] - P[0][17]*sk_mz[4]);
            Kfusion[1] = sk_mz[0]*(P[1][21] + P[1][0]*sh_mag[1] + P[1][3]*sh_mag[0] - P[1][1]*sk_mz[2] + P[1][2]*sk_mz[3] + P[1][18]*sk_mz[1] + P[1][16]*sk_mz[5] - P[1][17]*sk_mz[4]);
            Kfusion[2] = sk_mz[0]*(P[2][21] + P[2][0]*sh_mag[1] + P[2][3]*sh_mag[0] - P[2][1]*sk_mz[2] + P[2][2]*sk_mz[3] + P[2][18]*sk_mz[1] + P[2][16]*sk_mz[5] - P[2][17]*sk_mz[4]);
            Kfusion[3] = sk_mz[0]*(P[3][21] + P[3][0]*sh_mag[1] + P[3][3]*sh_mag[0] - P[3][1]*sk_mz[2] + P[3][2]*sk_mz[3] + P[3][18]*sk_mz[1] + P[3][16]*sk_mz[5] - P[3][17]*sk_mz[4]);
            Kfusion[4] = sk_mz[0]*(P[4][21] + P[4][0]*sh_mag[1] + P[4][3]*sh_mag[0] - P[4][1]*sk_mz[2] + P[4][2]*sk_mz[3] + P[4][18]*sk_mz[1] + P[4][16]*sk_mz[5] - P[4][17]*sk_mz[4]);
            Kfusion[5] = sk_mz[0]*(P[5][21] + P[5][0]*sh_mag[1] + P[5][3]*sh_mag[0] - P[5][1]*sk_mz[2] + P[5][2]*sk_mz[3] + P[5][18]*sk_mz[1] + P[5][16]*sk_mz[5] - P[5][17]*sk_mz[4]);
            Kfusion[6] = sk_mz[0]*(P[6][21] + P[6][0]*sh_mag[1] + P[6][3]*sh_mag[0] - P[6][1]*sk_mz[2] + P[6][2]*sk_mz[3] + P[6][18]*sk_mz[1] + P[6][16]*sk_mz[5] - P[6][17]*sk_mz[4]);
            Kfusion[7] = sk_mz[0]*(P[7][21] + P[7][0]*sh_mag[1] + P[7][3]*sh_mag[0] - P[7][1]*sk_mz[2] + P[7][2]*sk_mz[3] + P[7][18]*sk_mz[1] + P[7][16]*sk_mz[5] - P[7][17]*sk_mz[4]);
            Kfusion[8] = sk_mz[0]*(P[8][21] + P[8][0]*sh_mag[1] + P[8][3]*sh_mag[0] - P[8][1]*sk_mz[2] + P[8][2]*sk_mz[3] + P[8][18]*sk_mz[1] + P[8][16]*sk_mz[5] - P[8][17]*sk_mz[4]);
            Kfusion[9] = sk_mz[0]*(P[9][21] + P[9][0]*sh_mag[1] + P[9][3]*sh_mag[0] - P[9][1]*sk_mz[2] + P[9][2]*sk_mz[3] + P[9][18]*sk_mz[1] + P[9][16]*sk_mz[5] - P[9][17]*sk_mz[4]);
            Kfusion[10] = sk_mz[0]*(P[10][21] + P[10][0]*sh_mag[1] + P[10][3]*sh_mag[0] - P[10][1]*sk_mz[2] + P[10][2]*sk_mz[3] + P[10][18]*sk_mz[1] + P[10][16]*sk_mz[5] - P[10][17]*sk_mz[4]);
            Kfusion[11] = sk_mz[0]*(P[11][21] + P[11][0]*sh_mag[1] + P[11][3]*sh_mag[0] - P[11][1]*sk_mz[2] + P[11][2]*sk_mz[3] + P[11][18]*sk_mz[1] + P[11][16]*sk_mz[5] - P[11][17]*sk_mz[4]);
            Kfusion[12] = sk_mz[0]*(P[12][21] + P[12][0]*sh_mag[1] + P[12][3]*sh_mag[0] - P[12][1]*sk_mz[2] + P[12][2]*sk_mz[3] + P[12][18]*sk_mz[1] + P[12][16]*sk_mz[5] - P[12][17]*sk_mz[4]);
            // Only height measurements are allowed to modify the Z delta velocity bias state. This improves the stability of the estimate
            Kfusion[13] = 0.0f;//SK_MZ[0]*(P[13][21] + P[13][0]*SH_MAG[1] + P[13][3]*SH_MAG[0] - P[13][1]*SK_MZ[2] + P[13][2]*SK_MZ[3] + P[13][18]*SK_MZ[1] + P[13][16]*SK_MZ[5] - P[13][17]*SK_MZ[4]);
            // Estimation of selected states is inhibited by setting their Kalman gains to zero
            if (!inhibitWindStates) {
                Kfusion[14] = sk_mz[0]*(P[14][21] + P[14][0]*sh_mag[1] + P[14][3]*sh_mag[0] - P[14][1]*sk_mz[2] + P[14][2]*sk_mz[3] + P[14][18]*sk_mz[1] + P[14][16]*sk_mz[5] - P[14][17]*sk_mz[4]);
                Kfusion[15] = sk_mz[0]*(P[15][21] + P[15][0]*sh_mag[1] + P[15][3]*sh_mag[0] - P[15][1]*sk_mz[2] + P[15][2]*sk_mz[3] + P[15][18]*sk_mz[1] + P[15][16]*sk_mz[5] - P[15][17]*sk_mz[4]);
            } else {
                Kfusion[14] = 0;
                Kfusion[15] = 0;
            }
            if (!inhibitMagStates) {
                Kfusion[16] = sk_mz[0]*(P[16][21] + P[16][0]*sh_mag[1] + P[16][3]*sh_mag[0] - P[16][1]*sk_mz[2] + P[16][2]*sk_mz[3] + P[16][18]*sk_mz[1] + P[16][16]*sk_mz[5] - P[16][17]*sk_mz[4]);
                Kfusion[17] = sk_mz[0]*(P[17][21] + P[17][0]*sh_mag[1] + P[17][3]*sh_mag[0] - P[17][1]*sk_mz[2] + P[17][2]*sk_mz[3] + P[17][18]*sk_mz[1] + P[17][16]*sk_mz[5] - P[17][17]*sk_mz[4]);
                Kfusion[18] = sk_mz[0]*(P[18][21] + P[18][0]*sh_mag[1] + P[18][3]*sh_mag[0] - P[18][1]*sk_mz[2] + P[18][2]*sk_mz[3] + P[18][18]*sk_mz[1] + P[18][16]*sk_mz[5] - P[18][17]*sk_mz[4]);
                Kfusion[19] = sk_mz[0]*(P[19][21] + P[19][0]*sh_mag[1] + P[19][3]*sh_mag[0] - P[19][1]*sk_mz[2] + P[19][2]*sk_mz[3] + P[19][18]*sk_mz[1] + P[19][16]*sk_mz[5] - P[19][17]*sk_mz[4]);
                Kfusion[20] = sk_mz[0]*(P[20][21] + P[20][0]*sh_mag[1] + P[20][3]*sh_mag[0] - P[20][1]*sk_mz[2] + P[20][2]*sk_mz[3] + P[20][18]*sk_mz[1] + P[20][16]*sk_mz[5] - P[20][17]*sk_mz[4]);
                Kfusion[21] = sk_mz[0]*(P[21][21] + P[21][0]*sh_mag[1] + P[21][3]*sh_mag[0] - P[21][1]*sk_mz[2] + P[21][2]*sk_mz[3] + P[21][18]*sk_mz[1] + P[21][16]*sk_mz[5] - P[21][17]*sk_mz[4]);
            } else {
                Kfusion[16] = 0;
                Kfusion[17] = 0;
                Kfusion[18] = 0;
                Kfusion[19] = 0;
                Kfusion[20] = 0;
                Kfusion[21] = 0;
            }
            varInnovMag[2] = 1.0f/sk_mz[0];
            innovMag[2] = mag_pred[2] - magData.z;

        }

        // Check the innovation for consistency and don't fuse if > 5Sigma
        if ((innovMag[obs_index]*innovMag[obs_index]/varInnovMag[obs_index]) < 25.0f)
        {
            // correct the state vector
            for (uint8_t j= 0; j < ekf_state_estimates; j++)
            {
                states[j] = states[j] - Kfusion[j] * innovMag[obs_index];
            }
            // normalise the quaternion states
            float quat_mag = sqrtf(states[0]*states[0] + states[1]*states[1] + states[2]*states[2] + states[3]*states[3]);
            if (quat_mag > 1e-12f)
            {
                for (uint8_t j= 0; j<=3; j++)
                {
                    float quat_mag_inv = 1.0f/quat_mag;
                    states[j] = states[j] * quat_mag_inv;
                }
            }
            // correct the covariance P = (I - K*H)*P
            // take advantage of the empty columns in KH to reduce the
            // number of operations
            for (uint8_t i = 0; i < ekf_state_estimates; i++)
            {
                for (uint8_t j = 0; j <= 3; j++)
                {
                    KH[i][j] = Kfusion[i] * h_mag[j];
                }
                for (uint8_t j = 4; j <= 15; j++) KH[i][j] = 0.0f;
                if (!_onGround)
                {
                    for (uint8_t j = 16; j < ekf_state_estimates; j++)
                    {
                        KH[i][j] = Kfusion[i] * h_mag[j];
                    }
                }
                else
                {
                    for (uint8_t j = 16; j < ekf_state_estimates; j++)
                    {
                        KH[i][j] = 0.0f;
                    }
                }
            }
            for (uint8_t i = 0; i < ekf_state_estimates; i++)
            {
                for (uint8_t j = 0; j < ekf_state_estimates; j++)
                {
                    KHP[i][j] = 0.0f;
                    for (uint8_t k = 0; k <= 3; k++)
                    {
                        KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                    }
                    if (!_onGround)
                    {
                        for (uint8_t k = 16; k < ekf_state_estimates; k++)
                        {
                            KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                        }
                    }
                }
            }
        }
        for (uint8_t i = 0; i < ekf_state_estimates; i++)
        {
            for (uint8_t j = 0; j < ekf_state_estimates; j++)
            {
                P[i][j] = P[i][j] - KHP[i][j];
            }
        }
    }
    obs_index = obs_index + 1;

    ForceSymmetry();
    ConstrainVariances();
}

void AttPosEKF::fuseAirspeed()
{
    float vn;
    float ve;
    float vd;
    float vwn;
    float vwe;
    float r_tas = sq(airspeedMeasurementSigma);
    float sh_tas[3];
    float sk_tas;
    float vtas_pred;

    // Copy required states to local variable names
    vn = statesAtVtasMeasTime[4];
    ve = statesAtVtasMeasTime[5];
    vd = statesAtVtasMeasTime[6];
    vwn = statesAtVtasMeasTime[14];
    vwe = statesAtVtasMeasTime[15];

    // Need to check that it is flying before fusing airspeed data
    // Calculate the predicted airspeed
    vtas_pred = sqrtf((ve - vwe)*(ve - vwe) + (vn - vwn)*(vn - vwn) + vd*vd);
    // Perform fusion of True Airspeed measurement
    if (useAirspeed && fuseVtasData && (vtas_pred > 1.0f) && (VtasMeas > MIN_AIRSPEED_MEAS))
    {
        // Calculate observation jacobians
        sh_tas[0] = 1/(sqrtf(sq(ve - vwe) + sq(vn - vwn) + sq(vd)));
        sh_tas[1] = (sh_tas[0]*(2.0f*ve - 2*vwe))/2.0f;
        sh_tas[2] = (sh_tas[0]*(2.0f*vn - 2*vwn))/2.0f;

        float h_tas[ekf_state_estimates];
        for (uint8_t i = 0; i < ekf_state_estimates; i++) h_tas[i] = 0.0f;
        h_tas[4] = sh_tas[2];
        h_tas[5] = sh_tas[1];
        h_tas[6] = vd*sh_tas[0];
        h_tas[14] = -sh_tas[2];
        h_tas[15] = -sh_tas[1];

        // Calculate Kalman gains
        float temp = (r_tas + sh_tas[2]*(P[4][4]*sh_tas[2] + P[5][4]*sh_tas[1] - P[14][4]*sh_tas[2] - P[15][4]*sh_tas[1] + P[6][4]*vd*sh_tas[0]) + sh_tas[1]*(P[4][5]*sh_tas[2] + P[5][5]*sh_tas[1] - P[14][5]*sh_tas[2] - P[15][5]*sh_tas[1] + P[6][5]*vd*sh_tas[0]) - sh_tas[2]*(P[4][14]*sh_tas[2] + P[5][14]*sh_tas[1] - P[14][14]*sh_tas[2] - P[15][14]*sh_tas[1] + P[6][14]*vd*sh_tas[0]) - sh_tas[1]*(P[4][15]*sh_tas[2] + P[5][15]*sh_tas[1] - P[14][15]*sh_tas[2] - P[15][15]*sh_tas[1] + P[6][15]*vd*sh_tas[0]) + vd*sh_tas[0]*(P[4][6]*sh_tas[2] + P[5][6]*sh_tas[1] - P[14][6]*sh_tas[2] - P[15][6]*sh_tas[1] + P[6][6]*vd*sh_tas[0]));
        if (temp >= r_tas) {
            sk_tas = 1.0f / temp;
        } else {
            // the calculation is badly conditioned, so we cannot perform fusion on this step
            // we increase the wind state variances and try again next time
            P[14][14] += 0.05f*r_tas;
            P[15][15] += 0.05f*r_tas;
            return;
        }
        Kfusion[0] = sk_tas*(P[0][4]*sh_tas[2] - P[0][14]*sh_tas[2] + P[0][5]*sh_tas[1] - P[0][15]*sh_tas[1] + P[0][6]*vd*sh_tas[0]);
        Kfusion[1] = sk_tas*(P[1][4]*sh_tas[2] - P[1][14]*sh_tas[2] + P[1][5]*sh_tas[1] - P[1][15]*sh_tas[1] + P[1][6]*vd*sh_tas[0]);
        Kfusion[2] = sk_tas*(P[2][4]*sh_tas[2] - P[2][14]*sh_tas[2] + P[2][5]*sh_tas[1] - P[2][15]*sh_tas[1] + P[2][6]*vd*sh_tas[0]);
        Kfusion[3] = sk_tas*(P[3][4]*sh_tas[2] - P[3][14]*sh_tas[2] + P[3][5]*sh_tas[1] - P[3][15]*sh_tas[1] + P[3][6]*vd*sh_tas[0]);
        Kfusion[4] = sk_tas*(P[4][4]*sh_tas[2] - P[4][14]*sh_tas[2] + P[4][5]*sh_tas[1] - P[4][15]*sh_tas[1] + P[4][6]*vd*sh_tas[0]);
        Kfusion[5] = sk_tas*(P[5][4]*sh_tas[2] - P[5][14]*sh_tas[2] + P[5][5]*sh_tas[1] - P[5][15]*sh_tas[1] + P[5][6]*vd*sh_tas[0]);
        Kfusion[6] = sk_tas*(P[6][4]*sh_tas[2] - P[6][14]*sh_tas[2] + P[6][5]*sh_tas[1] - P[6][15]*sh_tas[1] + P[6][6]*vd*sh_tas[0]);
        Kfusion[7] = sk_tas*(P[7][4]*sh_tas[2] - P[7][14]*sh_tas[2] + P[7][5]*sh_tas[1] - P[7][15]*sh_tas[1] + P[7][6]*vd*sh_tas[0]);
        Kfusion[8] = sk_tas*(P[8][4]*sh_tas[2] - P[8][14]*sh_tas[2] + P[8][5]*sh_tas[1] - P[8][15]*sh_tas[1] + P[8][6]*vd*sh_tas[0]);
        Kfusion[9] = sk_tas*(P[9][4]*sh_tas[2] - P[9][14]*sh_tas[2] + P[9][5]*sh_tas[1] - P[9][15]*sh_tas[1] + P[9][6]*vd*sh_tas[0]);
        Kfusion[10] = sk_tas*(P[10][4]*sh_tas[2] - P[10][14]*sh_tas[2] + P[10][5]*sh_tas[1] - P[10][15]*sh_tas[1] + P[10][6]*vd*sh_tas[0]);
        Kfusion[11] = sk_tas*(P[11][4]*sh_tas[2] - P[11][14]*sh_tas[2] + P[11][5]*sh_tas[1] - P[11][15]*sh_tas[1] + P[11][6]*vd*sh_tas[0]);
        Kfusion[12] = sk_tas*(P[12][4]*sh_tas[2] - P[12][14]*sh_tas[2] + P[12][5]*sh_tas[1] - P[12][15]*sh_tas[1] + P[12][6]*vd*sh_tas[0]);
        // Only height measurements are allowed to modify the Z delta velocity bias state. This improves the stability of the estimate
        Kfusion[13] = 0.0f;//SK_TAS*(P[13][4]*SH_TAS[2] - P[13][14]*SH_TAS[2] + P[13][5]*SH_TAS[1] - P[13][15]*SH_TAS[1] + P[13][6]*vd*SH_TAS[0]);
        // Estimation of selected states is inhibited by setting their Kalman gains to zero
        if (!inhibitWindStates) {
            Kfusion[14] = sk_tas*(P[14][4]*sh_tas[2] - P[14][14]*sh_tas[2] + P[14][5]*sh_tas[1] - P[14][15]*sh_tas[1] + P[14][6]*vd*sh_tas[0]);
            Kfusion[15] = sk_tas*(P[15][4]*sh_tas[2] - P[15][14]*sh_tas[2] + P[15][5]*sh_tas[1] - P[15][15]*sh_tas[1] + P[15][6]*vd*sh_tas[0]);
        } else {
            Kfusion[14] = 0;
            Kfusion[15] = 0;
        }
        if (!inhibitMagStates) {
            Kfusion[16] = sk_tas*(P[16][4]*sh_tas[2] - P[16][14]*sh_tas[2] + P[16][5]*sh_tas[1] - P[16][15]*sh_tas[1] + P[16][6]*vd*sh_tas[0]);
            Kfusion[17] = sk_tas*(P[17][4]*sh_tas[2] - P[17][14]*sh_tas[2] + P[17][5]*sh_tas[1] - P[17][15]*sh_tas[1] + P[17][6]*vd*sh_tas[0]);
            Kfusion[18] = sk_tas*(P[18][4]*sh_tas[2] - P[18][14]*sh_tas[2] + P[18][5]*sh_tas[1] - P[18][15]*sh_tas[1] + P[18][6]*vd*sh_tas[0]);
            Kfusion[19] = sk_tas*(P[19][4]*sh_tas[2] - P[19][14]*sh_tas[2] + P[19][5]*sh_tas[1] - P[19][15]*sh_tas[1] + P[19][6]*vd*sh_tas[0]);
            Kfusion[20] = sk_tas*(P[20][4]*sh_tas[2] - P[20][14]*sh_tas[2] + P[20][5]*sh_tas[1] - P[20][15]*sh_tas[1] + P[20][6]*vd*sh_tas[0]);
            Kfusion[21] = sk_tas*(P[21][4]*sh_tas[2] - P[21][14]*sh_tas[2] + P[21][5]*sh_tas[1] - P[21][15]*sh_tas[1] + P[21][6]*vd*sh_tas[0]);
        } else {
            for (uint8_t i=16; i < ekf_state_estimates; i++) {
                Kfusion[i] = 0;
            }
        }
        varInnovVtas = 1.0f/sk_tas;

        // Calculate the measurement innovation
        innovVtas = vtas_pred - VtasMeas;
        // Check the innovation for consistency and don't fuse if > 5Sigma
        if ((innovVtas*innovVtas*sk_tas) < 25.0f)
        {
            // correct the state vector
            for (uint8_t j=0; j < ekf_state_estimates; j++)
            {
                states[j] = states[j] - Kfusion[j] * innovVtas;
            }
            // normalise the quaternion states
            float quat_mag = sqrtf(states[0]*states[0] + states[1]*states[1] + states[2]*states[2] + states[3]*states[3]);
            if (quat_mag > 1e-12f)
            {
                for (uint8_t j= 0; j <= 3; j++)
                {
                    float quat_mag_inv = 1.0f/quat_mag;
                    states[j] = states[j] * quat_mag_inv;
                }
            }
            // correct the covariance P = (I - K*H)*P
            // take advantage of the empty columns in H to reduce the
            // number of operations
            for (uint8_t i = 0; i < ekf_state_estimates; i++)
            {
                for (uint8_t j = 0; j <= 3; j++) KH[i][j] = 0.0;
                for (uint8_t j = 4; j <= 6; j++)
                {
                    KH[i][j] = Kfusion[i] * h_tas[j];
                }
                for (uint8_t j = 7; j <= 13; j++) KH[i][j] = 0.0;
                for (uint8_t j = 14; j <= 15; j++)
                {
                    KH[i][j] = Kfusion[i] * h_tas[j];
                }
                for (uint8_t j = 16; j < ekf_state_estimates; j++) KH[i][j] = 0.0;
            }
            for (uint8_t i = 0; i < ekf_state_estimates; i++)
            {
                for (uint8_t j = 0; j < ekf_state_estimates; j++)
                {
                    KHP[i][j] = 0.0;
                    for (uint8_t k = 4; k <= 6; k++)
                    {
                        KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                    }
                    for (uint8_t k = 14; k <= 15; k++)
                    {
                        KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                    }
                }
            }
            for (uint8_t i = 0; i < ekf_state_estimates; i++)
            {
                for (uint8_t j = 0; j < ekf_state_estimates; j++)
                {
                    P[i][j] = P[i][j] - KHP[i][j];
                }
            }
        }
    }

    ForceSymmetry();
    ConstrainVariances();
}

void AttPosEKF::zeroRows(float (&cov_mat)[ekf_state_estimates][ekf_state_estimates], uint8_t first, uint8_t last)
{
    uint8_t row;
    uint8_t col;
    for (row=first; row<=last; row++)
    {
        for (col=0; col<ekf_state_estimates; col++)
        {
            cov_mat[row][col] = 0.0;
        }
    }
}

void AttPosEKF::fuseOptFlow()
{
    static float sh_los[13];
    static float sk_los[9];
    static float q0 = 0.0f;
    static float q1 = 0.0f;
    static float q2 = 0.0f;
    static float q3 = 1.0f;
    static float vn = 0.0f;
    static float ve = 0.0f;
    static float vd = 0.0f;
    static float pd = 0.0f;
    static float ptd = 0.0f;
    static float los_pred[2];

    // Transformation matrix from nav to body axes
    float h_los[2][ekf_state_estimates];
    float k_los[2][ekf_state_estimates];
    Vector3f vel_ned_local;
    Vector3f rel_vel_sensor;

    // Perform sequential fusion of optical flow measurements only with valid tilt and height
    flowStates[1] = math::max(flowStates[1], statesAtFlowTime[9] + minFlowRng);
    float height_above_gnd_est = flowStates[1] - statesAtFlowTime[9];
    bool valid_tilt = Tnb.z.z > 0.71f;
    if (valid_tilt)
    {
        // Sequential fusion of XY components.

        // Calculate observation jacobians and Kalman gains
        if (fuseOptFlowData)
        {
            // Copy required states to local variable names
            q0       = statesAtFlowTime[0];
            q1       = statesAtFlowTime[1];
            q2       = statesAtFlowTime[2];
            q3       = statesAtFlowTime[3];
            vn       = statesAtFlowTime[4];
            ve       = statesAtFlowTime[5];
            vd       = statesAtFlowTime[6];
            pd       = statesAtFlowTime[9];
            ptd      = flowStates[1];
            vel_ned_local.x = vn;
            vel_ned_local.y = ve;
            vel_ned_local.z = vd;

            // calculate range from ground plain to centre of sensor fov assuming flat earth
            float range = height_above_gnd_est/Tnb_flow.z.z;

            // calculate relative velocity in sensor frame
            rel_vel_sensor = Tnb_flow*vel_ned_local;

            // divide velocity by range  and include angular rate effects to get predicted angular LOS rates relative to X and Y axes
            los_pred[0] =  rel_vel_sensor.y/range;
            los_pred[1] = -rel_vel_sensor.x/range;

            // Calculate common expressions for observation jacobians
            sh_los[0] = sq(q0) - sq(q1) - sq(q2) + sq(q3);
            sh_los[1] = vn*(sq(q0) + sq(q1) - sq(q2) - sq(q3)) - vd*(2*q0*q2 - 2*q1*q3) + ve*(2*q0*q3 + 2*q1*q2);
            sh_los[2] = ve*(sq(q0) - sq(q1) + sq(q2) - sq(q3)) + vd*(2*q0*q1 + 2*q2*q3) - vn*(2*q0*q3 - 2*q1*q2);
            sh_los[3] = 1/(pd - ptd);
            sh_los[4] = 1/sq(pd - ptd);

            // Calculate common expressions for Kalman gains
            sk_los[0] = 1.0f/((R_LOS + sq(omegaAcrossFlowTime[0] * moCompR_LOS)) + (sh_los[0]*sh_los[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) + 2*q0*sh_los[1]*sh_los[3])*(P[0][0]*(sh_los[0]*sh_los[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) + 2*q0*sh_los[1]*sh_los[3]) + P[1][0]*(sh_los[0]*sh_los[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q1*sh_los[1]*sh_los[3]) - P[2][0]*(sh_los[0]*sh_los[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) + 2*q2*sh_los[1]*sh_los[3]) + P[3][0]*(sh_los[0]*sh_los[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q3*sh_los[1]*sh_los[3]) + P[5][0]*sh_los[0]*sh_los[3]*(2*q0*q3 + 2*q1*q2) - P[6][0]*sh_los[0]*sh_los[3]*(2*q0*q2 - 2*q1*q3) - P[9][0]*sh_los[0]*sh_los[1]*sh_los[4] + P[4][0]*sh_los[0]*sh_los[3]*(sq(q0) + sq(q1) - sq(q2) - sq(q3))) + (sh_los[0]*sh_los[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q1*sh_los[1]*sh_los[3])*(P[0][1]*(sh_los[0]*sh_los[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) + 2*q0*sh_los[1]*sh_los[3]) + P[1][1]*(sh_los[0]*sh_los[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q1*sh_los[1]*sh_los[3]) - P[2][1]*(sh_los[0]*sh_los[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) + 2*q2*sh_los[1]*sh_los[3]) + P[3][1]*(sh_los[0]*sh_los[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q3*sh_los[1]*sh_los[3]) + P[5][1]*sh_los[0]*sh_los[3]*(2*q0*q3 + 2*q1*q2) - P[6][1]*sh_los[0]*sh_los[3]*(2*q0*q2 - 2*q1*q3) - P[9][1]*sh_los[0]*sh_los[1]*sh_los[4] + P[4][1]*sh_los[0]*sh_los[3]*(sq(q0) + sq(q1) - sq(q2) - sq(q3))) - (sh_los[0]*sh_los[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) + 2*q2*sh_los[1]*sh_los[3])*(P[0][2]*(sh_los[0]*sh_los[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) + 2*q0*sh_los[1]*sh_los[3]) + P[1][2]*(sh_los[0]*sh_los[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q1*sh_los[1]*sh_los[3]) - P[2][2]*(sh_los[0]*sh_los[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) + 2*q2*sh_los[1]*sh_los[3]) + P[3][2]*(sh_los[0]*sh_los[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q3*sh_los[1]*sh_los[3]) + P[5][2]*sh_los[0]*sh_los[3]*(2*q0*q3 + 2*q1*q2) - P[6][2]*sh_los[0]*sh_los[3]*(2*q0*q2 - 2*q1*q3) - P[9][2]*sh_los[0]*sh_los[1]*sh_los[4] + P[4][2]*sh_los[0]*sh_los[3]*(sq(q0) + sq(q1) - sq(q2) - sq(q3))) + (sh_los[0]*sh_los[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q3*sh_los[1]*sh_los[3])*(P[0][3]*(sh_los[0]*sh_los[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) + 2*q0*sh_los[1]*sh_los[3]) + P[1][3]*(sh_los[0]*sh_los[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q1*sh_los[1]*sh_los[3]) - P[2][3]*(sh_los[0]*sh_los[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) + 2*q2*sh_los[1]*sh_los[3]) + P[3][3]*(sh_los[0]*sh_los[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q3*sh_los[1]*sh_los[3]) + P[5][3]*sh_los[0]*sh_los[3]*(2*q0*q3 + 2*q1*q2) - P[6][3]*sh_los[0]*sh_los[3]*(2*q0*q2 - 2*q1*q3) - P[9][3]*sh_los[0]*sh_los[1]*sh_los[4] + P[4][3]*sh_los[0]*sh_los[3]*(sq(q0) + sq(q1) - sq(q2) - sq(q3))) - sh_los[0]*sh_los[1]*sh_los[4]*(P[0][9]*(sh_los[0]*sh_los[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) + 2*q0*sh_los[1]*sh_los[3]) + P[1][9]*(sh_los[0]*sh_los[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q1*sh_los[1]*sh_los[3]) - P[2][9]*(sh_los[0]*sh_los[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) + 2*q2*sh_los[1]*sh_los[3]) + P[3][9]*(sh_los[0]*sh_los[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q3*sh_los[1]*sh_los[3]) + P[5][9]*sh_los[0]*sh_los[3]*(2*q0*q3 + 2*q1*q2) - P[6][9]*sh_los[0]*sh_los[3]*(2*q0*q2 - 2*q1*q3) - P[9][9]*sh_los[0]*sh_los[1]*sh_los[4] + P[4][9]*sh_los[0]*sh_los[3]*(sq(q0) + sq(q1) - sq(q2) - sq(q3))) + sh_los[0]*sh_los[3]*(sq(q0) + sq(q1) - sq(q2) - sq(q3))*(P[0][4]*(sh_los[0]*sh_los[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) + 2*q0*sh_los[1]*sh_los[3]) + P[1][4]*(sh_los[0]*sh_los[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q1*sh_los[1]*sh_los[3]) - P[2][4]*(sh_los[0]*sh_los[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) + 2*q2*sh_los[1]*sh_los[3]) + P[3][4]*(sh_los[0]*sh_los[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q3*sh_los[1]*sh_los[3]) + P[5][4]*sh_los[0]*sh_los[3]*(2*q0*q3 + 2*q1*q2) - P[6][4]*sh_los[0]*sh_los[3]*(2*q0*q2 - 2*q1*q3) - P[9][4]*sh_los[0]*sh_los[1]*sh_los[4] + P[4][4]*sh_los[0]*sh_los[3]*(sq(q0) + sq(q1) - sq(q2) - sq(q3))) + sh_los[0]*sh_los[3]*(2*q0*q3 + 2*q1*q2)*(P[0][5]*(sh_los[0]*sh_los[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) + 2*q0*sh_los[1]*sh_los[3]) + P[1][5]*(sh_los[0]*sh_los[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q1*sh_los[1]*sh_los[3]) - P[2][5]*(sh_los[0]*sh_los[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) + 2*q2*sh_los[1]*sh_los[3]) + P[3][5]*(sh_los[0]*sh_los[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q3*sh_los[1]*sh_los[3]) + P[5][5]*sh_los[0]*sh_los[3]*(2*q0*q3 + 2*q1*q2) - P[6][5]*sh_los[0]*sh_los[3]*(2*q0*q2 - 2*q1*q3) - P[9][5]*sh_los[0]*sh_los[1]*sh_los[4] + P[4][5]*sh_los[0]*sh_los[3]*(sq(q0) + sq(q1) - sq(q2) - sq(q3))) - sh_los[0]*sh_los[3]*(2*q0*q2 - 2*q1*q3)*(P[0][6]*(sh_los[0]*sh_los[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) + 2*q0*sh_los[1]*sh_los[3]) + P[1][6]*(sh_los[0]*sh_los[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q1*sh_los[1]*sh_los[3]) - P[2][6]*(sh_los[0]*sh_los[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) + 2*q2*sh_los[1]*sh_los[3]) + P[3][6]*(sh_los[0]*sh_los[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q3*sh_los[1]*sh_los[3]) + P[5][6]*sh_los[0]*sh_los[3]*(2*q0*q3 + 2*q1*q2) - P[6][6]*sh_los[0]*sh_los[3]*(2*q0*q2 - 2*q1*q3) - P[9][6]*sh_los[0]*sh_los[1]*sh_los[4] + P[4][6]*sh_los[0]*sh_los[3]*(sq(q0) + sq(q1) - sq(q2) - sq(q3))));
            sk_los[1] = 1.0f/((R_LOS + sq(omegaAcrossFlowTime[1] * moCompR_LOS))+ (sh_los[0]*sh_los[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q0*sh_los[2]*sh_los[3])*(P[0][0]*(sh_los[0]*sh_los[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q0*sh_los[2]*sh_los[3]) + P[1][0]*(sh_los[0]*sh_los[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) - 2*q1*sh_los[2]*sh_los[3]) + P[2][0]*(sh_los[0]*sh_los[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q2*sh_los[2]*sh_los[3]) - P[3][0]*(sh_los[0]*sh_los[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) - 2*q3*sh_los[2]*sh_los[3]) - P[4][0]*sh_los[0]*sh_los[3]*(2*q0*q3 - 2*q1*q2) + P[6][0]*sh_los[0]*sh_los[3]*(2*q0*q1 + 2*q2*q3) - P[9][0]*sh_los[0]*sh_los[2]*sh_los[4] + P[5][0]*sh_los[0]*sh_los[3]*(sq(q0) - sq(q1) + sq(q2) - sq(q3))) + (sh_los[0]*sh_los[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) - 2*q1*sh_los[2]*sh_los[3])*(P[0][1]*(sh_los[0]*sh_los[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q0*sh_los[2]*sh_los[3]) + P[1][1]*(sh_los[0]*sh_los[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) - 2*q1*sh_los[2]*sh_los[3]) + P[2][1]*(sh_los[0]*sh_los[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q2*sh_los[2]*sh_los[3]) - P[3][1]*(sh_los[0]*sh_los[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) - 2*q3*sh_los[2]*sh_los[3]) - P[4][1]*sh_los[0]*sh_los[3]*(2*q0*q3 - 2*q1*q2) + P[6][1]*sh_los[0]*sh_los[3]*(2*q0*q1 + 2*q2*q3) - P[9][1]*sh_los[0]*sh_los[2]*sh_los[4] + P[5][1]*sh_los[0]*sh_los[3]*(sq(q0) - sq(q1) + sq(q2) - sq(q3))) + (sh_los[0]*sh_los[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q2*sh_los[2]*sh_los[3])*(P[0][2]*(sh_los[0]*sh_los[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q0*sh_los[2]*sh_los[3]) + P[1][2]*(sh_los[0]*sh_los[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) - 2*q1*sh_los[2]*sh_los[3]) + P[2][2]*(sh_los[0]*sh_los[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q2*sh_los[2]*sh_los[3]) - P[3][2]*(sh_los[0]*sh_los[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) - 2*q3*sh_los[2]*sh_los[3]) - P[4][2]*sh_los[0]*sh_los[3]*(2*q0*q3 - 2*q1*q2) + P[6][2]*sh_los[0]*sh_los[3]*(2*q0*q1 + 2*q2*q3) - P[9][2]*sh_los[0]*sh_los[2]*sh_los[4] + P[5][2]*sh_los[0]*sh_los[3]*(sq(q0) - sq(q1) + sq(q2) - sq(q3))) - (sh_los[0]*sh_los[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) - 2*q3*sh_los[2]*sh_los[3])*(P[0][3]*(sh_los[0]*sh_los[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q0*sh_los[2]*sh_los[3]) + P[1][3]*(sh_los[0]*sh_los[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) - 2*q1*sh_los[2]*sh_los[3]) + P[2][3]*(sh_los[0]*sh_los[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q2*sh_los[2]*sh_los[3]) - P[3][3]*(sh_los[0]*sh_los[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) - 2*q3*sh_los[2]*sh_los[3]) - P[4][3]*sh_los[0]*sh_los[3]*(2*q0*q3 - 2*q1*q2) + P[6][3]*sh_los[0]*sh_los[3]*(2*q0*q1 + 2*q2*q3) - P[9][3]*sh_los[0]*sh_los[2]*sh_los[4] + P[5][3]*sh_los[0]*sh_los[3]*(sq(q0) - sq(q1) + sq(q2) - sq(q3))) - sh_los[0]*sh_los[2]*sh_los[4]*(P[0][9]*(sh_los[0]*sh_los[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q0*sh_los[2]*sh_los[3]) + P[1][9]*(sh_los[0]*sh_los[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) - 2*q1*sh_los[2]*sh_los[3]) + P[2][9]*(sh_los[0]*sh_los[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q2*sh_los[2]*sh_los[3]) - P[3][9]*(sh_los[0]*sh_los[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) - 2*q3*sh_los[2]*sh_los[3]) - P[4][9]*sh_los[0]*sh_los[3]*(2*q0*q3 - 2*q1*q2) + P[6][9]*sh_los[0]*sh_los[3]*(2*q0*q1 + 2*q2*q3) - P[9][9]*sh_los[0]*sh_los[2]*sh_los[4] + P[5][9]*sh_los[0]*sh_los[3]*(sq(q0) - sq(q1) + sq(q2) - sq(q3))) + sh_los[0]*sh_los[3]*(sq(q0) - sq(q1) + sq(q2) - sq(q3))*(P[0][5]*(sh_los[0]*sh_los[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q0*sh_los[2]*sh_los[3]) + P[1][5]*(sh_los[0]*sh_los[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) - 2*q1*sh_los[2]*sh_los[3]) + P[2][5]*(sh_los[0]*sh_los[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q2*sh_los[2]*sh_los[3]) - P[3][5]*(sh_los[0]*sh_los[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) - 2*q3*sh_los[2]*sh_los[3]) - P[4][5]*sh_los[0]*sh_los[3]*(2*q0*q3 - 2*q1*q2) + P[6][5]*sh_los[0]*sh_los[3]*(2*q0*q1 + 2*q2*q3) - P[9][5]*sh_los[0]*sh_los[2]*sh_los[4] + P[5][5]*sh_los[0]*sh_los[3]*(sq(q0) - sq(q1) + sq(q2) - sq(q3))) - sh_los[0]*sh_los[3]*(2*q0*q3 - 2*q1*q2)*(P[0][4]*(sh_los[0]*sh_los[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q0*sh_los[2]*sh_los[3]) + P[1][4]*(sh_los[0]*sh_los[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) - 2*q1*sh_los[2]*sh_los[3]) + P[2][4]*(sh_los[0]*sh_los[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q2*sh_los[2]*sh_los[3]) - P[3][4]*(sh_los[0]*sh_los[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) - 2*q3*sh_los[2]*sh_los[3]) - P[4][4]*sh_los[0]*sh_los[3]*(2*q0*q3 - 2*q1*q2) + P[6][4]*sh_los[0]*sh_los[3]*(2*q0*q1 + 2*q2*q3) - P[9][4]*sh_los[0]*sh_los[2]*sh_los[4] + P[5][4]*sh_los[0]*sh_los[3]*(sq(q0) - sq(q1) + sq(q2) - sq(q3))) + sh_los[0]*sh_los[3]*(2*q0*q1 + 2*q2*q3)*(P[0][6]*(sh_los[0]*sh_los[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q0*sh_los[2]*sh_los[3]) + P[1][6]*(sh_los[0]*sh_los[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) - 2*q1*sh_los[2]*sh_los[3]) + P[2][6]*(sh_los[0]*sh_los[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q2*sh_los[2]*sh_los[3]) - P[3][6]*(sh_los[0]*sh_los[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) - 2*q3*sh_los[2]*sh_los[3]) - P[4][6]*sh_los[0]*sh_los[3]*(2*q0*q3 - 2*q1*q2) + P[6][6]*sh_los[0]*sh_los[3]*(2*q0*q1 + 2*q2*q3) - P[9][6]*sh_los[0]*sh_los[2]*sh_los[4] + P[5][6]*sh_los[0]*sh_los[3]*(sq(q0) - sq(q1) + sq(q2) - sq(q3))));
            sk_los[2] = sh_los[0]*sh_los[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn);
            sk_los[3] = sh_los[0]*sh_los[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn);
            sk_los[4] = sh_los[0]*sh_los[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn);
            sk_los[5] = sh_los[0]*sh_los[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn);
            sk_los[6] = sq(q0) - sq(q1) + sq(q2) - sq(q3);
            sk_los[7] = sq(q0) + sq(q1) - sq(q2) - sq(q3);
            sk_los[8] = sh_los[3];

            // Calculate common intermediate terms
            float temp_var[9];
            temp_var[0] = sh_los[0]*sk_los[6]*sk_los[8];
            temp_var[1] = sh_los[0]*sh_los[2]*sh_los[4];
            temp_var[2] = 2.0f*sh_los[2]*sk_los[8];
            temp_var[3] = sh_los[0]*sk_los[8]*(2.0f*q0*q1 + 2.0f*q2*q3);
            temp_var[4] = sh_los[0]*sk_los[8]*(2.0f*q0*q3 - 2.0f*q1*q2);
            temp_var[5] = (sk_los[5] - q2*temp_var[2]);
            temp_var[6] = (sk_los[2] - q3*temp_var[2]);
            temp_var[7] = (sk_los[3] - q1*temp_var[2]);
            temp_var[8] = (sk_los[4] + q0*temp_var[2]);

            // calculate observation jacobians for X LOS rate
            for (uint8_t i = 0; i < ekf_state_estimates; i++) h_los[0][i] = 0;
            h_los[0][0] = - sh_los[0]*sh_los[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) - 2*q0*sh_los[2]*sh_los[3];
            h_los[0][1] = 2*q1*sh_los[2]*sh_los[3] - sh_los[0]*sh_los[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn);
            h_los[0][2] = 2*q2*sh_los[2]*sh_los[3] - sh_los[0]*sh_los[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn);
            h_los[0][3] = sh_los[0]*sh_los[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) - 2*q3*sh_los[2]*sh_los[3];
            h_los[0][4] = sh_los[0]*sh_los[3]*(2*q0*q3 - 2*q1*q2);
            h_los[0][5] = -sh_los[0]*sh_los[3]*(sq(q0) - sq(q1) + sq(q2) - sq(q3));
            h_los[0][6] = -sh_los[0]*sh_los[3]*(2*q0*q1 + 2*q2*q3);
            h_los[0][9] = temp_var[1];

            // calculate Kalman gains for X LOS rate
            k_los[0][0] = -(P[0][0]*temp_var[8] + P[0][1]*temp_var[7] - P[0][3]*temp_var[6] + P[0][2]*temp_var[5] - P[0][4]*temp_var[4] + P[0][6]*temp_var[3] - P[0][9]*temp_var[1] + P[0][5]*temp_var[0])/(R_LOS + temp_var[8]*(P[0][0]*temp_var[8] + P[1][0]*temp_var[7] + P[2][0]*temp_var[5] - P[3][0]*temp_var[6] - P[4][0]*temp_var[4] + P[6][0]*temp_var[3] - P[9][0]*temp_var[1] + P[5][0]*temp_var[0]) + temp_var[7]*(P[0][1]*temp_var[8] + P[1][1]*temp_var[7] + P[2][1]*temp_var[5] - P[3][1]*temp_var[6] - P[4][1]*temp_var[4] + P[6][1]*temp_var[3] - P[9][1]*temp_var[1] + P[5][1]*temp_var[0]) + temp_var[5]*(P[0][2]*temp_var[8] + P[1][2]*temp_var[7] + P[2][2]*temp_var[5] - P[3][2]*temp_var[6] - P[4][2]*temp_var[4] + P[6][2]*temp_var[3] - P[9][2]*temp_var[1] + P[5][2]*temp_var[0]) - temp_var[6]*(P[0][3]*temp_var[8] + P[1][3]*temp_var[7] + P[2][3]*temp_var[5] - P[3][3]*temp_var[6] - P[4][3]*temp_var[4] + P[6][3]*temp_var[3] - P[9][3]*temp_var[1] + P[5][3]*temp_var[0]) - temp_var[4]*(P[0][4]*temp_var[8] + P[1][4]*temp_var[7] + P[2][4]*temp_var[5] - P[3][4]*temp_var[6] - P[4][4]*temp_var[4] + P[6][4]*temp_var[3] - P[9][4]*temp_var[1] + P[5][4]*temp_var[0]) + temp_var[3]*(P[0][6]*temp_var[8] + P[1][6]*temp_var[7] + P[2][6]*temp_var[5] - P[3][6]*temp_var[6] - P[4][6]*temp_var[4] + P[6][6]*temp_var[3] - P[9][6]*temp_var[1] + P[5][6]*temp_var[0]) - temp_var[1]*(P[0][9]*temp_var[8] + P[1][9]*temp_var[7] + P[2][9]*temp_var[5] - P[3][9]*temp_var[6] - P[4][9]*temp_var[4] + P[6][9]*temp_var[3] - P[9][9]*temp_var[1] + P[5][9]*temp_var[0]) + temp_var[0]*(P[0][5]*temp_var[8] + P[1][5]*temp_var[7] + P[2][5]*temp_var[5] - P[3][5]*temp_var[6] - P[4][5]*temp_var[4] + P[6][5]*temp_var[3] - P[9][5]*temp_var[1] + P[5][5]*temp_var[0]));
            k_los[0][1] = -sk_los[1]*(P[1][0]*temp_var[8] + P[1][1]*temp_var[7] - P[1][3]*temp_var[6] + P[1][2]*temp_var[5] - P[1][4]*temp_var[4] + P[1][6]*temp_var[3] - P[1][9]*temp_var[1] + P[1][5]*temp_var[0]);
            k_los[0][2] = -sk_los[1]*(P[2][0]*temp_var[8] + P[2][1]*temp_var[7] - P[2][3]*temp_var[6] + P[2][2]*temp_var[5] - P[2][4]*temp_var[4] + P[2][6]*temp_var[3] - P[2][9]*temp_var[1] + P[2][5]*temp_var[0]);
            k_los[0][3] = -sk_los[1]*(P[3][0]*temp_var[8] + P[3][1]*temp_var[7] - P[3][3]*temp_var[6] + P[3][2]*temp_var[5] - P[3][4]*temp_var[4] + P[3][6]*temp_var[3] - P[3][9]*temp_var[1] + P[3][5]*temp_var[0]);
            k_los[0][4] = -sk_los[1]*(P[4][0]*temp_var[8] + P[4][1]*temp_var[7] - P[4][3]*temp_var[6] + P[4][2]*temp_var[5] - P[4][4]*temp_var[4] + P[4][6]*temp_var[3] - P[4][9]*temp_var[1] + P[4][5]*temp_var[0]);
            k_los[0][5] = -sk_los[1]*(P[5][0]*temp_var[8] + P[5][1]*temp_var[7] - P[5][3]*temp_var[6] + P[5][2]*temp_var[5] - P[5][4]*temp_var[4] + P[5][6]*temp_var[3] - P[5][9]*temp_var[1] + P[5][5]*temp_var[0]);
            k_los[0][6] = -sk_los[1]*(P[6][0]*temp_var[8] + P[6][1]*temp_var[7] - P[6][3]*temp_var[6] + P[6][2]*temp_var[5] - P[6][4]*temp_var[4] + P[6][6]*temp_var[3] - P[6][9]*temp_var[1] + P[6][5]*temp_var[0]);
            k_los[0][7] = -sk_los[1]*(P[7][0]*temp_var[8] + P[7][1]*temp_var[7] - P[7][3]*temp_var[6] + P[7][2]*temp_var[5] - P[7][4]*temp_var[4] + P[7][6]*temp_var[3] - P[7][9]*temp_var[1] + P[7][5]*temp_var[0]);
            k_los[0][8] = -sk_los[1]*(P[8][0]*temp_var[8] + P[8][1]*temp_var[7] - P[8][3]*temp_var[6] + P[8][2]*temp_var[5] - P[8][4]*temp_var[4] + P[8][6]*temp_var[3] - P[8][9]*temp_var[1] + P[8][5]*temp_var[0]);
            k_los[0][9] = -sk_los[1]*(P[9][0]*temp_var[8] + P[9][1]*temp_var[7] - P[9][3]*temp_var[6] + P[9][2]*temp_var[5] - P[9][4]*temp_var[4] + P[9][6]*temp_var[3] - P[9][9]*temp_var[1] + P[9][5]*temp_var[0]);
            k_los[0][10] = -sk_los[1]*(P[10][0]*temp_var[8] + P[10][1]*temp_var[7] - P[10][3]*temp_var[6] + P[10][2]*temp_var[5] - P[10][4]*temp_var[4] + P[10][6]*temp_var[3] - P[10][9]*temp_var[1] + P[10][5]*temp_var[0]);
            k_los[0][11] = -sk_los[1]*(P[11][0]*temp_var[8] + P[11][1]*temp_var[7] - P[11][3]*temp_var[6] + P[11][2]*temp_var[5] - P[11][4]*temp_var[4] + P[11][6]*temp_var[3] - P[11][9]*temp_var[1] + P[11][5]*temp_var[0]);
            k_los[0][12] = -sk_los[1]*(P[12][0]*temp_var[8] + P[12][1]*temp_var[7] - P[12][3]*temp_var[6] + P[12][2]*temp_var[5] - P[12][4]*temp_var[4] + P[12][6]*temp_var[3] - P[12][9]*temp_var[1] + P[12][5]*temp_var[0]);
            // only height measurements are allowed to modify the Z bias state to improve the stability of the estimate
            k_los[0][13] = 0.0f;//-SK_LOS[1]*(P[13][0]*tempVar[8] + P[13][1]*tempVar[7] - P[13][3]*tempVar[6] + P[13][2]*tempVar[5] - P[13][4]*tempVar[4] + P[13][6]*tempVar[3] - P[13][9]*tempVar[1] + P[13][5]*tempVar[0]);
            if (inhibitWindStates) {
                k_los[0][14] = -sk_los[1]*(P[14][0]*temp_var[8] + P[14][1]*temp_var[7] - P[14][3]*temp_var[6] + P[14][2]*temp_var[5] - P[14][4]*temp_var[4] + P[14][6]*temp_var[3] - P[14][9]*temp_var[1] + P[14][5]*temp_var[0]);
                k_los[0][15] = -sk_los[1]*(P[15][0]*temp_var[8] + P[15][1]*temp_var[7] - P[15][3]*temp_var[6] + P[15][2]*temp_var[5] - P[15][4]*temp_var[4] + P[15][6]*temp_var[3] - P[15][9]*temp_var[1] + P[15][5]*temp_var[0]);
            } else {
                k_los[0][14] = 0.0f;
                k_los[0][15] = 0.0f;
            }
            if (inhibitMagStates) {
                k_los[0][16] = -sk_los[1]*(P[16][0]*temp_var[8] + P[16][1]*temp_var[7] - P[16][3]*temp_var[6] + P[16][2]*temp_var[5] - P[16][4]*temp_var[4] + P[16][6]*temp_var[3] - P[16][9]*temp_var[1] + P[16][5]*temp_var[0]);
                k_los[0][17] = -sk_los[1]*(P[17][0]*temp_var[8] + P[17][1]*temp_var[7] - P[17][3]*temp_var[6] + P[17][2]*temp_var[5] - P[17][4]*temp_var[4] + P[17][6]*temp_var[3] - P[17][9]*temp_var[1] + P[17][5]*temp_var[0]);
                k_los[0][18] = -sk_los[1]*(P[18][0]*temp_var[8] + P[18][1]*temp_var[7] - P[18][3]*temp_var[6] + P[18][2]*temp_var[5] - P[18][4]*temp_var[4] + P[18][6]*temp_var[3] - P[18][9]*temp_var[1] + P[18][5]*temp_var[0]);
                k_los[0][19] = -sk_los[1]*(P[19][0]*temp_var[8] + P[19][1]*temp_var[7] - P[19][3]*temp_var[6] + P[19][2]*temp_var[5] - P[19][4]*temp_var[4] + P[19][6]*temp_var[3] - P[19][9]*temp_var[1] + P[19][5]*temp_var[0]);
                k_los[0][20] = -sk_los[1]*(P[20][0]*temp_var[8] + P[20][1]*temp_var[7] - P[20][3]*temp_var[6] + P[20][2]*temp_var[5] - P[20][4]*temp_var[4] + P[20][6]*temp_var[3] - P[20][9]*temp_var[1] + P[20][5]*temp_var[0]);
                k_los[0][21] = -sk_los[1]*(P[21][0]*temp_var[8] + P[21][1]*temp_var[7] - P[21][3]*temp_var[6] + P[21][2]*temp_var[5] - P[21][4]*temp_var[4] + P[21][6]*temp_var[3] - P[21][9]*temp_var[1] + P[21][5]*temp_var[0]);
            } else {
                for (uint8_t i = 16; i < ekf_state_estimates; i++) {
                    k_los[0][i] = 0.0f;
                }
            }

            // calculate innovation variance and innovation for X axis observation
            varInnovOptFlow[0] = 1.0f/sk_los[0];
            innovOptFlow[0] = los_pred[0] - flowRadXYcomp[0];

            // calculate intermediate common variables
            temp_var[0] = 2.0f*sh_los[1]*sk_los[8];
            temp_var[1] = (sk_los[2] + q0*temp_var[0]);
            temp_var[2] = (sk_los[5] - q1*temp_var[0]);
            temp_var[3] = (sk_los[3] + q2*temp_var[0]);
            temp_var[4] = (sk_los[4] + q3*temp_var[0]);
            temp_var[5] = sh_los[0]*sk_los[8]*(2*q0*q3 + 2*q1*q2);
            temp_var[6] = sh_los[0]*sk_los[8]*(2*q0*q2 - 2*q1*q3);
            temp_var[7] = sh_los[0]*sh_los[1]*sh_los[4];
            temp_var[8] = sh_los[0]*sk_los[7]*sk_los[8];

            // Calculate observation jacobians for Y LOS rate
            for (uint8_t i = 0; i < ekf_state_estimates; i++) h_los[1][i] = 0;
            h_los[1][0] = sh_los[0]*sh_los[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) + 2*q0*sh_los[1]*sh_los[3];
            h_los[1][1] = sh_los[0]*sh_los[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q1*sh_los[1]*sh_los[3];
            h_los[1][2] = - sh_los[0]*sh_los[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) - 2*q2*sh_los[1]*sh_los[3];
            h_los[1][3] = sh_los[0]*sh_los[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q3*sh_los[1]*sh_los[3];
            h_los[1][4] = sh_los[0]*sh_los[3]*(sq(q0) + sq(q1) - sq(q2) - sq(q3));
            h_los[1][5] = sh_los[0]*sh_los[3]*(2*q0*q3 + 2*q1*q2);
            h_los[1][6] = -sh_los[0]*sh_los[3]*(2*q0*q2 - 2*q1*q3);
            h_los[1][9] = -temp_var[7];

            // Calculate Kalman gains for Y LOS rate
            k_los[1][0] = sk_los[0]*(P[0][0]*temp_var[1] + P[0][1]*temp_var[2] - P[0][2]*temp_var[3] + P[0][3]*temp_var[4] + P[0][5]*temp_var[5] - P[0][6]*temp_var[6] - P[0][9]*temp_var[7] + P[0][4]*temp_var[8]);
            k_los[1][1] = sk_los[0]*(P[1][0]*temp_var[1] + P[1][1]*temp_var[2] - P[1][2]*temp_var[3] + P[1][3]*temp_var[4] + P[1][5]*temp_var[5] - P[1][6]*temp_var[6] - P[1][9]*temp_var[7] + P[1][4]*temp_var[8]);
            k_los[1][2] = sk_los[0]*(P[2][0]*temp_var[1] + P[2][1]*temp_var[2] - P[2][2]*temp_var[3] + P[2][3]*temp_var[4] + P[2][5]*temp_var[5] - P[2][6]*temp_var[6] - P[2][9]*temp_var[7] + P[2][4]*temp_var[8]);
            k_los[1][3] = sk_los[0]*(P[3][0]*temp_var[1] + P[3][1]*temp_var[2] - P[3][2]*temp_var[3] + P[3][3]*temp_var[4] + P[3][5]*temp_var[5] - P[3][6]*temp_var[6] - P[3][9]*temp_var[7] + P[3][4]*temp_var[8]);
            k_los[1][4] = sk_los[0]*(P[4][0]*temp_var[1] + P[4][1]*temp_var[2] - P[4][2]*temp_var[3] + P[4][3]*temp_var[4] + P[4][5]*temp_var[5] - P[4][6]*temp_var[6] - P[4][9]*temp_var[7] + P[4][4]*temp_var[8]);
            k_los[1][5] = sk_los[0]*(P[5][0]*temp_var[1] + P[5][1]*temp_var[2] - P[5][2]*temp_var[3] + P[5][3]*temp_var[4] + P[5][5]*temp_var[5] - P[5][6]*temp_var[6] - P[5][9]*temp_var[7] + P[5][4]*temp_var[8]);
            k_los[1][6] = sk_los[0]*(P[6][0]*temp_var[1] + P[6][1]*temp_var[2] - P[6][2]*temp_var[3] + P[6][3]*temp_var[4] + P[6][5]*temp_var[5] - P[6][6]*temp_var[6] - P[6][9]*temp_var[7] + P[6][4]*temp_var[8]);
            k_los[1][7] = sk_los[0]*(P[7][0]*temp_var[1] + P[7][1]*temp_var[2] - P[7][2]*temp_var[3] + P[7][3]*temp_var[4] + P[7][5]*temp_var[5] - P[7][6]*temp_var[6] - P[7][9]*temp_var[7] + P[7][4]*temp_var[8]);
            k_los[1][8] = sk_los[0]*(P[8][0]*temp_var[1] + P[8][1]*temp_var[2] - P[8][2]*temp_var[3] + P[8][3]*temp_var[4] + P[8][5]*temp_var[5] - P[8][6]*temp_var[6] - P[8][9]*temp_var[7] + P[8][4]*temp_var[8]);
            k_los[1][9] = sk_los[0]*(P[9][0]*temp_var[1] + P[9][1]*temp_var[2] - P[9][2]*temp_var[3] + P[9][3]*temp_var[4] + P[9][5]*temp_var[5] - P[9][6]*temp_var[6] - P[9][9]*temp_var[7] + P[9][4]*temp_var[8]);
            k_los[1][10] = sk_los[0]*(P[10][0]*temp_var[1] + P[10][1]*temp_var[2] - P[10][2]*temp_var[3] + P[10][3]*temp_var[4] + P[10][5]*temp_var[5] - P[10][6]*temp_var[6] - P[10][9]*temp_var[7] + P[10][4]*temp_var[8]);
            k_los[1][11] = sk_los[0]*(P[11][0]*temp_var[1] + P[11][1]*temp_var[2] - P[11][2]*temp_var[3] + P[11][3]*temp_var[4] + P[11][5]*temp_var[5] - P[11][6]*temp_var[6] - P[11][9]*temp_var[7] + P[11][4]*temp_var[8]);
            k_los[1][12] = sk_los[0]*(P[12][0]*temp_var[1] + P[12][1]*temp_var[2] - P[12][2]*temp_var[3] + P[12][3]*temp_var[4] + P[12][5]*temp_var[5] - P[12][6]*temp_var[6] - P[12][9]*temp_var[7] + P[12][4]*temp_var[8]);
            // only height measurements are allowed to modify the Z bias state to improve the stability of the estimate
            k_los[1][13] = 0.0f;//SK_LOS[0]*(P[13][0]*tempVar[1] + P[13][1]*tempVar[2] - P[13][2]*tempVar[3] + P[13][3]*tempVar[4] + P[13][5]*tempVar[5] - P[13][6]*tempVar[6] - P[13][9]*tempVar[7] + P[13][4]*tempVar[8]);
            if (inhibitWindStates) {
                k_los[1][14] = sk_los[0]*(P[14][0]*temp_var[1] + P[14][1]*temp_var[2] - P[14][2]*temp_var[3] + P[14][3]*temp_var[4] + P[14][5]*temp_var[5] - P[14][6]*temp_var[6] - P[14][9]*temp_var[7] + P[14][4]*temp_var[8]);
                k_los[1][15] = sk_los[0]*(P[15][0]*temp_var[1] + P[15][1]*temp_var[2] - P[15][2]*temp_var[3] + P[15][3]*temp_var[4] + P[15][5]*temp_var[5] - P[15][6]*temp_var[6] - P[15][9]*temp_var[7] + P[15][4]*temp_var[8]);
            } else {
                k_los[1][14] = 0.0f;
                k_los[1][15] = 0.0f;
            }
            if (inhibitMagStates) {
                k_los[1][16] = sk_los[0]*(P[16][0]*temp_var[1] + P[16][1]*temp_var[2] - P[16][2]*temp_var[3] + P[16][3]*temp_var[4] + P[16][5]*temp_var[5] - P[16][6]*temp_var[6] - P[16][9]*temp_var[7] + P[16][4]*temp_var[8]);
                k_los[1][17] = sk_los[0]*(P[17][0]*temp_var[1] + P[17][1]*temp_var[2] - P[17][2]*temp_var[3] + P[17][3]*temp_var[4] + P[17][5]*temp_var[5] - P[17][6]*temp_var[6] - P[17][9]*temp_var[7] + P[17][4]*temp_var[8]);
                k_los[1][18] = sk_los[0]*(P[18][0]*temp_var[1] + P[18][1]*temp_var[2] - P[18][2]*temp_var[3] + P[18][3]*temp_var[4] + P[18][5]*temp_var[5] - P[18][6]*temp_var[6] - P[18][9]*temp_var[7] + P[18][4]*temp_var[8]);
                k_los[1][19] = sk_los[0]*(P[19][0]*temp_var[1] + P[19][1]*temp_var[2] - P[19][2]*temp_var[3] + P[19][3]*temp_var[4] + P[19][5]*temp_var[5] - P[19][6]*temp_var[6] - P[19][9]*temp_var[7] + P[19][4]*temp_var[8]);
                k_los[1][20] = sk_los[0]*(P[20][0]*temp_var[1] + P[20][1]*temp_var[2] - P[20][2]*temp_var[3] + P[20][3]*temp_var[4] + P[20][5]*temp_var[5] - P[20][6]*temp_var[6] - P[20][9]*temp_var[7] + P[20][4]*temp_var[8]);
                k_los[1][21] = sk_los[0]*(P[21][0]*temp_var[1] + P[21][1]*temp_var[2] - P[21][2]*temp_var[3] + P[21][3]*temp_var[4] + P[21][5]*temp_var[5] - P[21][6]*temp_var[6] - P[21][9]*temp_var[7] + P[21][4]*temp_var[8]);
            } else {
                for (uint8_t i = 16; i < ekf_state_estimates; i++) {
                    k_los[1][i] = 0.0f;
                }
            }

            // calculate variance and innovation for Y observation
            varInnovOptFlow[1] = 1.0f/sk_los[1];
            innovOptFlow[1] = los_pred[1] - flowRadXYcomp[1];

            // loop through the X and Y observations and fuse them sequentially
            for (uint8_t obs_index = 0; obs_index < 2; obs_index++) {
                // Check the innovation for consistency and don't fuse if > 5Sigma
                if ((innovOptFlow[obs_index]*innovOptFlow[obs_index]/varInnovOptFlow[obs_index]) < 25.0f) {
                    // correct the state vector
                    for (uint8_t j = 0; j < ekf_state_estimates; j++)
                    {
                        states[j] = states[j] - k_los[obs_index][j] * innovOptFlow[obs_index];
                    }
                    // normalise the quaternion states
                    float quat_mag = sqrtf(states[0]*states[0] + states[1]*states[1] + states[2]*states[2] + states[3]*states[3]);
                    if (quat_mag > 1e-12f)
                    {
                        for (uint8_t j= 0; j<=3; j++)
                        {
                            float quat_mag_inv = 1.0f/quat_mag;
                            states[j] = states[j] * quat_mag_inv;
                        }
                    }
                    // correct the covariance P = (I - K*H)*P
                    // take advantage of the empty columns in KH to reduce the
                    // number of operations
                    for (uint8_t i = 0; i < ekf_state_estimates; i++)
                    {
                        for (uint8_t j = 0; j <= 6; j++)
                        {
                            KH[i][j] = k_los[obs_index][i] * h_los[obs_index][j];
                        }
                        for (uint8_t j = 7; j <= 8; j++)
                        {
                            KH[i][j] = 0.0f;
                        }
                        KH[i][9] = k_los[obs_index][i] * h_los[obs_index][9];
                        for (uint8_t j = 10; j < ekf_state_estimates; j++)
                        {
                            KH[i][j] = 0.0f;
                        }
                    }
                    for (uint8_t i = 0; i < ekf_state_estimates; i++)
                    {
                        for (uint8_t j = 0; j < ekf_state_estimates; j++)
                        {
                            KHP[i][j] = 0.0f;
                            for (uint8_t k = 0; k <= 6; k++)
                            {
                                KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                            }
                            KHP[i][j] = KHP[i][j] + KH[i][9] * P[9][j];
                        }
                    }
                    for (uint8_t i = 0; i <  ekf_state_estimates; i++)
                    {
                        for (uint8_t j = 0; j <  ekf_state_estimates; j++)
                        {
                            P[i][j] = P[i][j] - KHP[i][j];
                        }
                    }
                }
            }
        }
        ForceSymmetry();
        ConstrainVariances();
    }
}

void AttPosEKF::opticalFlowEkf()
{
    // propagate ground position state noise each time this is called using the difference in position since the last observations and an RMS gradient assumption
    // limit distance to prevent intialisation afer bad gps causing bad numerical conditioning
    if (!inhibitGndState) {
        float distance_travelled_sq;
        if (fuseRngData) {
            distance_travelled_sq = sq(statesAtRngTime[7] - prevPosN) + sq(statesAtRngTime[8] - prevPosE);
            prevPosN = statesAtRngTime[7];
            prevPosE = statesAtRngTime[8];
        } else if (fuseOptFlowData) {
            distance_travelled_sq = sq(statesAtFlowTime[7] - prevPosN) + sq(statesAtFlowTime[8] - prevPosE);
            prevPosN = statesAtFlowTime[7];
            prevPosE = statesAtFlowTime[8];
        } else {
            return;
        }
        distance_travelled_sq = math::min(distance_travelled_sq, 100.0f);
        Popt[1][1] += (distance_travelled_sq * sq(gndHgtSigma));
    }

    // fuse range finder data
    if (fuseRngData) {
        float range; // range from camera to centre of image
        float q0; // quaternion at optical flow measurement time
        float q1; // quaternion at optical flow measurement time
        float q2; // quaternion at optical flow measurement time
        float q3; // quaternion at optical flow measurement time
        float r_rng = 0.5; // range measurement variance (m^2) TODO make this a function of range and tilt to allow for sensor, alignment and AHRS errors

        // Copy required states to local variable names
        q0             = statesAtRngTime[0];
        q1             = statesAtRngTime[1];
        q2             = statesAtRngTime[2];
        q3             = statesAtRngTime[3];

        // calculate Kalman gains
        float sk_rng[3];
        sk_rng[0] = sq(q0) - sq(q1) - sq(q2) + sq(q3);
        sk_rng[1] = 1/(r_rng + Popt[1][1]/sq(sk_rng[0]));
        sk_rng[2] = 1/sk_rng[0];
        float k_rng[2];
        if (!inhibitScaleState) {
            k_rng[0] = Popt[0][1]*sk_rng[1]*sk_rng[2];
        } else {
            k_rng[0] = 0.0f;
        }
        if (!inhibitGndState) {
            k_rng[1] = Popt[1][1]*sk_rng[1]*sk_rng[2];
        } else {
            k_rng[1] = 0.0f;
        }

        // Calculate the innovation variance for data logging
        varInnovRng = 1.0f/sk_rng[1];

        // constrain terrain height to be below the vehicle
        flowStates[1] = math::max(flowStates[1], statesAtRngTime[9] + minFlowRng);

        // estimate range to centre of image
        range = (flowStates[1] - statesAtRngTime[9]) * sk_rng[2];

        // Calculate the measurement innovation
        innovRng = range - rngMea;

        // calculate the innovation consistency test ratio
        auxRngTestRatio = sq(innovRng) / (sq(rngInnovGate) * varInnovRng);

        // Check the innovation for consistency and don't fuse if out of bounds
        if (auxRngTestRatio < 1.0f)
        {
            // correct the state
            for (uint8_t i = 0; i < 2 ; i++) {
                flowStates[i] -= k_rng[i] * innovRng;
            }
            // constrain the states
            flowStates[0] = ConstrainFloat(flowStates[0], 0.1f, 10.0f);
            flowStates[1] = math::max(flowStates[1], statesAtRngTime[9] + minFlowRng);

            // correct the covariance matrix
            float next_popt[2][2];
            next_popt[0][0] = Popt[0][0] - (Popt[0][1]*Popt[1][0]*sk_rng[1]*sk_rng[2]) * sk_rng[2];
            next_popt[0][1] = Popt[0][1] - (Popt[0][1]*Popt[1][1]*sk_rng[1]*sk_rng[2]) * sk_rng[2];
            next_popt[1][0] = -Popt[1][0]*((Popt[1][1]*sk_rng[1]*sk_rng[2]) * sk_rng[2] - 1.0f);
            next_popt[1][1] = -Popt[1][1]*((Popt[1][1]*sk_rng[1]*sk_rng[2]) * sk_rng[2] - 1.0f);
            // prevent the state variances from becoming negative and maintain symmetry
            Popt[0][0] = math::max(next_popt[0][0],0.0f);
            Popt[1][1] = math::max(next_popt[1][1],0.0f);
            Popt[0][1] = 0.5f * (next_popt[0][1] + next_popt[1][0]);
            Popt[1][0] = Popt[0][1];
        }
    }

    if (fuseOptFlowData) {
        Vector3f vel; // velocity of sensor relative to ground in NED axes
        Vector3f rel_vel_sensor; // velocity of sensor relative to ground in sensor axes
        float los_pred[2]; // predicted optical flow angular rate measurements
        float range; // range from camera to centre of image
        float q0; // quaternion at optical flow measurement time
        float q1; // quaternion at optical flow measurement time
        float q2; // quaternion at optical flow measurement time
        float q3; // quaternion at optical flow measurement time
        float hp[2];
        float sh_opt[6];
        float sk_opt[3];
        float k_opt[2][2];
        float h_opt[2][2];
        float next_popt[2][2];

        // propagate scale factor state noise
        if (!inhibitScaleState) {
            Popt[0][0] += 1e-8f;
        } else {
            Popt[0][0] = 0.0f;
        }

        // Copy required states to local variable names
        q0             = statesAtFlowTime[0];
        q1             = statesAtFlowTime[1];
        q2             = statesAtFlowTime[2];
        q3             = statesAtFlowTime[3];
        vel.x          = statesAtFlowTime[4];
        vel.y          = statesAtFlowTime[5];
        vel.z          = statesAtFlowTime[6];

        // constrain terrain height to be below the vehicle
        flowStates[1] = math::max(flowStates[1], statesAtFlowTime[9] + minFlowRng);

        // estimate range to centre of image
        range = (flowStates[1] - statesAtFlowTime[9]) / Tnb_flow.z.z;

        // calculate relative velocity in sensor frame
        rel_vel_sensor = Tnb_flow * vel;

        // divide velocity by range, subtract body rates and apply scale factor to
        // get predicted sensed angular optical rates relative to X and Y sensor axes
        los_pred[0] =  flowStates[0]*( rel_vel_sensor.y / range) - omegaAcrossFlowTime[0];
        los_pred[1] =  flowStates[0]*(-rel_vel_sensor.x / range) - omegaAcrossFlowTime[1];

        // calculate innovations
        auxFlowObsInnov[0] = los_pred[0] - flowRadXY[0];
        auxFlowObsInnov[1] = los_pred[1] - flowRadXY[1];

        // calculate Kalman gains
        sh_opt[0] = sq(q0) - sq(q1) - sq(q2) + sq(q3);
        sh_opt[1] = vel.x*(sq(q0) + sq(q1) - sq(q2) - sq(q3)) + vel.y*(2*q0*q3 + 2*q1*q2) - vel.z*(2*q0*q2 - 2*q1*q3);
        sh_opt[2] = vel.y*(sq(q0) - sq(q1) + sq(q2) - sq(q3)) - vel.x*(2*q0*q3 - 2*q1*q2) + vel.z*(2*q0*q1 + 2*q2*q3);
        sh_opt[3] = statesAtFlowTime[9] - flowStates[1];
        sh_opt[4] = 1.0f/sq(sh_opt[3]);
        sh_opt[5] = 1.0f/sh_opt[3];
        float s_h015 = sh_opt[0]*sh_opt[1]*sh_opt[5];
        float s_h025 = sh_opt[0]*sh_opt[2]*sh_opt[5];
        float s_h014 = sh_opt[0]*sh_opt[1]*sh_opt[4];
        float s_h024 = sh_opt[0]*sh_opt[2]*sh_opt[4];
        sk_opt[0] = 1.0f/(R_LOS + s_h015*(Popt[0][0]*s_h015 + Popt[1][0]*flowStates[0]*s_h014) + flowStates[0]*s_h014*(Popt[0][1]*s_h015 + Popt[1][1]*flowStates[0]*s_h014));
        sk_opt[1] = 1.0f/(R_LOS + s_h025*(Popt[0][0]*s_h025 + Popt[1][0]*flowStates[0]*s_h024) + flowStates[0]*s_h024*(Popt[0][1]*s_h025 + Popt[1][1]*flowStates[0]*s_h024));
        sk_opt[2] = sh_opt[0];
        if (!inhibitScaleState) {
            k_opt[0][0] = -sk_opt[1]*(Popt[0][0]*sh_opt[2]*sh_opt[5]*sk_opt[2] + Popt[0][1]*flowStates[0]*sh_opt[2]*sh_opt[4]*sk_opt[2]);
            k_opt[0][1] =  sk_opt[0]*(Popt[0][0]*sh_opt[1]*sh_opt[5]*sk_opt[2] + Popt[0][1]*flowStates[0]*sh_opt[1]*sh_opt[4]*sk_opt[2]);
        } else {
            k_opt[0][0] = 0.0f;
            k_opt[0][1] = 0.0f;
        }
        if (!inhibitGndState) {
            k_opt[1][0] = -sk_opt[1]*(Popt[1][0]*sh_opt[2]*sh_opt[5]*sk_opt[2] + Popt[1][1]*flowStates[0]*sh_opt[2]*sh_opt[4]*sk_opt[2]);
            k_opt[1][1] =  sk_opt[0]*(Popt[1][0]*sh_opt[1]*sh_opt[5]*sk_opt[2] + Popt[1][1]*flowStates[0]*sh_opt[1]*sh_opt[4]*sk_opt[2]);
        } else {
            k_opt[1][0] = 0.0f;
            k_opt[1][1] = 0.0f;
        }

        // calculate innovation variances
        auxFlowObsInnovVar[0] = 1.0f/sk_opt[1];
        auxFlowObsInnovVar[1] = 1.0f/sk_opt[0];

        // calculate observations jacobians
        h_opt[0][0] = -s_h025;
        h_opt[0][1] = -flowStates[0]*s_h024;
        h_opt[1][0] = s_h015;
        h_opt[1][1] = flowStates[0]*s_h014;

        // Check the innovation for consistency and don't fuse if > threshold
        for (uint8_t obs_index = 0; obs_index < 2; obs_index++) {

            // calculate the innovation consistency test ratio
            auxFlowTestRatio[obs_index] = sq(auxFlowObsInnov[obs_index]) / (sq(auxFlowInnovGate) * auxFlowObsInnovVar[obs_index]);
            if (auxFlowTestRatio[obs_index] < 1.0f) {
                // correct the state
                for (uint8_t i = 0; i < 2 ; i++) {
                    flowStates[i] -= k_opt[i][obs_index] * auxFlowObsInnov[obs_index];
                }
                // constrain the states
                flowStates[0] = ConstrainFloat(flowStates[0], 0.1f, 10.0f);
                flowStates[1] = math::max(flowStates[1], statesAtFlowTime[9] + minFlowRng);

                // correct the covariance matrix
                for (uint8_t i = 0; i < 2 ; i++) {
                    hp[i] = 0.0f;
                    for (uint8_t j = 0; j < 2 ; j++) {
                        hp[i] += h_opt[obs_index][j] * P[j][i];
                    }
                }
                for (uint8_t i = 0; i < 2 ; i++) {
                    for (uint8_t j = 0; j < 2 ; j++) {
                        next_popt[i][j] = P[i][j] - k_opt[i][obs_index] * hp[j];
                    }
                }

                // prevent the state variances from becoming negative and maintain symmetry
                Popt[0][0] = math::max(next_popt[0][0],0.0f);
                Popt[1][1] = math::max(next_popt[1][1],0.0f);
                Popt[0][1] = 0.5f * (next_popt[0][1] + next_popt[1][0]);
                Popt[1][0] = Popt[0][1];
            }
        }
    }

}

void AttPosEKF::zeroCols(float (&cov_mat)[ekf_state_estimates][ekf_state_estimates], uint8_t first, uint8_t last)
{
    uint8_t row;
    uint8_t col;
    for (col=first; col<=last; col++)
    {
        for (row=0; row < ekf_state_estimates; row++)
        {
            cov_mat[row][col] = 0.0;
        }
    }
}

// Store states in a history array along with time stamp
void AttPosEKF::storeStates(uint64_t timestamp_ms)
{
    for (size_t i = 0; i < ekf_state_estimates; i++) {
        storedStates[i][storeIndex] = states[i];
    }

    storedOmega[0][storeIndex] = angRate.x;
    storedOmega[1][storeIndex] = angRate.y;
    storedOmega[2][storeIndex] = angRate.z;
    statetimeStamp[storeIndex] = timestamp_ms;

    // increment to next storage index
    storeIndex++;
    if (storeIndex >= ekf_data_buffer_size) {
        storeIndex = 0;
    }
}

void AttPosEKF::resetStoredStates()
{
    // reset all stored states
    memset(&storedStates[0][0], 0, sizeof(storedStates));
    memset(&storedOmega[0][0], 0, sizeof(storedOmega));
    memset(&statetimeStamp[0], 0, sizeof(statetimeStamp));

    // reset store index to first
    storeIndex = 0;

    //Reset stored state to current state
    StoreStates(millis());
}

// Output the state vector stored at the time that best matches that specified by msec
int AttPosEKF::recallStates(float* states_for_fusion, uint64_t msec)
{
    int ret = 0;

    int64_t best_time_delta = 200;
    size_t best_store_index = 0;
    for (size_t store_index_local = 0; store_index_local < ekf_data_buffer_size; store_index_local++)
    {
        // Work around a GCC compiler bug - we know 64bit support on ARM is
        // sketchy in GCC.
        uint64_t time_delta;

        if (msec > statetimeStamp[store_index_local]) {
            time_delta = msec - statetimeStamp[store_index_local];
        } else {
            time_delta = statetimeStamp[store_index_local] - msec;
        }

        if (time_delta < (uint64_t)best_time_delta)
        {
            best_store_index = store_index_local;
            best_time_delta = time_delta;
        }
    }
    if (best_time_delta < 200) // only output stored state if < 200 msec retrieval error
    {
        for (size_t i=0; i < ekf_state_estimates; i++) {
            if (PX4_ISFINITE(storedStates[i][best_store_index])) {
                states_for_fusion[i] = storedStates[i][best_store_index];
            } else if (PX4_ISFINITE(states[i])) {
                states_for_fusion[i] = states[i];
            } else {
                // There is not much we can do here, except reporting the error we just
                // found.
                ret++;
            }
        }
    }
    else // otherwise output current state
    {
        for (size_t i = 0; i < ekf_state_estimates; i++) {
            if (PX4_ISFINITE(states[i])) {
                states_for_fusion[i] = states[i];
            } else {
                ret++;
            }
        }
    }

    return ret;
}

void AttPosEKF::recallOmega(float* omega_for_fusion, uint64_t msec)
{
    // work back in time and calculate average angular rate over the time interval
    for (size_t i=0; i < 3; i++) {
        omega_for_fusion[i] = 0.0f;
    }
    uint8_t sum_index = 0;
    int64_t time_delta;
    for (size_t store_index_local = 0; store_index_local < ekf_data_buffer_size; store_index_local++)
    {
        // calculate the average of all samples younger than msec
        time_delta = statetimeStamp[store_index_local] - msec;
        if (time_delta > 0)
        {
            for (size_t i=0; i < 3; i++) {
                omega_for_fusion[i] += storedOmega[i][store_index_local];
            }
            sum_index += 1;
        }
    }
    if (sum_index >= 1) {
        for (size_t i=0; i < 3; i++) {
            omega_for_fusion[i] = omega_for_fusion[i] / float(sum_index);
        }
    } else {
        omega_for_fusion[0] = angRate.x;
        omega_for_fusion[1] = angRate.y;
        omega_for_fusion[2] = angRate.z;
    }
}

#if 0
void AttPosEKF::quat2Tnb(Mat3f &Tnb, const float (&quat)[4])
{
    // Calculate the nav to body cosine matrix
    float q00 = sq(quat[0]);
    float q11 = sq(quat[1]);
    float q22 = sq(quat[2]);
    float q33 = sq(quat[3]);
    float q01 =  quat[0]*quat[1];
    float q02 =  quat[0]*quat[2];
    float q03 =  quat[0]*quat[3];
    float q12 =  quat[1]*quat[2];
    float q13 =  quat[1]*quat[3];
    float q23 =  quat[2]*quat[3];

    Tnb.x.x = q00 + q11 - q22 - q33;
    Tnb.y.y = q00 - q11 + q22 - q33;
    Tnb.z.z = q00 - q11 - q22 + q33;
    Tnb.y.x = 2*(q12 - q03);
    Tnb.z.x = 2*(q13 + q02);
    Tnb.x.y = 2*(q12 + q03);
    Tnb.z.y = 2*(q23 - q01);
    Tnb.x.z = 2*(q13 - q02);
    Tnb.y.z = 2*(q23 + q01);
}
#endif

void AttPosEKF::quat2Tbn(Mat3f &tbn_ret, const float (&quat)[4])
{
    // Calculate the body to nav cosine matrix
    float q00 = sq(quat[0]);
    float q11 = sq(quat[1]);
    float q22 = sq(quat[2]);
    float q33 = sq(quat[3]);
    float q01 =  quat[0]*quat[1];
    float q02 =  quat[0]*quat[2];
    float q03 =  quat[0]*quat[3];
    float q12 =  quat[1]*quat[2];
    float q13 =  quat[1]*quat[3];
    float q23 =  quat[2]*quat[3];

    tbn_ret.x.x = q00 + q11 - q22 - q33;
    tbn_ret.y.y = q00 - q11 + q22 - q33;
    tbn_ret.z.z = q00 - q11 - q22 + q33;
    tbn_ret.x.y = 2*(q12 - q03);
    tbn_ret.x.z = 2*(q13 + q02);
    tbn_ret.y.x = 2*(q12 + q03);
    tbn_ret.y.z = 2*(q23 - q01);
    tbn_ret.z.x = 2*(q13 - q02);
    tbn_ret.z.y = 2*(q23 + q01);
}

void AttPosEKF::eul2quat(float (&quat)[4], const float (&eul)[3])
{
    float u1 = cos(0.5f*eul[0]);
    float u2 = cos(0.5f*eul[1]);
    float u3 = cos(0.5f*eul[2]);
    float u4 = sin(0.5f*eul[0]);
    float u5 = sin(0.5f*eul[1]);
    float u6 = sin(0.5f*eul[2]);
    quat[0] = u1*u2*u3+u4*u5*u6;
    quat[1] = u4*u2*u3-u1*u5*u6;
    quat[2] = u1*u5*u3+u4*u2*u6;
    quat[3] = u1*u2*u6-u4*u5*u3;
}

void AttPosEKF::quat2eul(float (&y)[3], const float (&u)[4])
{
    y[0] = atan2f((2.0f*(u[2]*u[3]+u[0]*u[1])) , (u[0]*u[0]-u[1]*u[1]-u[2]*u[2]+u[3]*u[3]));
    y[1] = -asinf(2.0f*(u[1]*u[3]-u[0]*u[2]));
    y[2] = atan2f((2.0f*(u[1]*u[2]+u[0]*u[3])) , (u[0]*u[0]+u[1]*u[1]-u[2]*u[2]-u[3]*u[3]));
}

void AttPosEKF::setOnGround(const bool is_landed)
{
    _onGround = is_landed;

    if (staticMode) {
        staticMode = (!refSet || (GPSstatus < GPS_FIX_3D));
    }
    // don't update wind states if there is no airspeed measurement
    if (_onGround || !useAirspeed) {
        inhibitWindStates = true;
    } else {
        inhibitWindStates = false;
    }

    //Check if we are accelerating forward, only then is the mag offset is observable
    bool is_moving_forward = _accNavMagHorizontal > 0.5f;

    // don't update magnetic field states if on ground or not using compass
    inhibitMagStates = (!useCompass || _onGround) || (!_isFixedWing && !is_moving_forward);

    // don't update terrain offset state if there is no range finder and flying at low velocity or without GPS
    if ((_onGround || !useGPS) && !useRangeFinder) {
        inhibitGndState = true;
    } else {
        inhibitGndState = false;
    }

    // don't update terrain offset state if there is no range finder and flying at low velocity, or without GPS, as it is poorly observable
    if ((_onGround || (globalTimeStamp_ms - lastFixTime_ms) > 1000) && !useRangeFinder) {
        inhibitGndState = true;
    } else {
        inhibitGndState = false;
    }

    // Don't update focal length offset state if there is no range finder or optical flow sensor
    // we need both sensors to do this estimation
    if (!useRangeFinder || !useOpticalFlow) {
        inhibitScaleState = true;
    } else {
        inhibitScaleState = false;
    }
}

void AttPosEKF::calcEarthRateNED(Vector3f &omega, float latitude)
{
    //Define Earth rotation vector in the NED navigation frame
    omega.x  = earthRate*cosf(latitude);
    omega.y  = 0.0f;
    omega.z  = -earthRate*sinf(latitude);
}

void AttPosEKF::covarianceInit()
{
    // Calculate the initial covariance matrix P
    P[0][0]   = 0.25f * sq(1.0f*deg2rad);
    P[1][1]   = 0.25f * sq(1.0f*deg2rad);
    P[2][2]   = 0.25f * sq(1.0f*deg2rad);
    P[3][3]   = 0.25f * sq(10.0f*deg2rad);

    //velocities
    P[4][4]   = sq(0.7f);
    P[5][5]   = P[4][4];
    P[6][6]   = sq(0.7f);

    //positions
    P[7][7]   = sq(15.0f);
    P[8][8]   = P[7][7];
    P[9][9]   = sq(5.0f);

    //delta angle biases
    P[10][10] = sq(0.1f*deg2rad*dtIMU);
    P[11][11] = P[10][10];
    P[12][12] = P[10][10];

    //Z delta velocity bias
    P[13][13] = sq(0.2f*dtIMU);

    //Wind velocities
    P[14][14] = 0.01f;
    P[15][15]  = P[14][14];

    //Earth magnetic field
    P[16][16] = sq(0.02f);
    P[17][17] = P[16][16];
    P[18][18] = P[16][16];

    //Body magnetic field
    P[19][19] = sq(0.02f);
    P[20][20] = P[19][19];
    P[21][21] = P[19][19];

    //Optical flow
    fScaleFactorVar = 0.001f; // focal length scale factor variance
    Popt[0][0] = 0.001f;
}

float AttPosEKF::constrainFloat(float val, float min_val, float max_val)
{
    float ret;
    if (val > max_val) {
        ret = max_val;
        ekf_debug("> max: %8.4f, val: %8.4f", (double)max_val, (double)val);
    } else if (val < min_val) {
        ret = min_val;
        ekf_debug("< min: %8.4f, val: %8.4f", (double)min_val, (double)val);
    } else {
        ret = val;
    }

    if (!PX4_ISFINITE(val)) {
        ekf_debug("constrain: non-finite!");
    }

    return ret;
}

void AttPosEKF::constrainVariances()
{
    if (!numericalProtection) {
        return;
    }

    // State vector:
    // 0-3: quaternions (q0, q1, q2, q3)
    // 4-6: Velocity - m/sec (North, East, Down)
    // 7-9: Position - m (North, East, Down)
    // 10-12: Delta Angle bias - rad (X,Y,Z)
    // 13: Delta Velocity bias - m/s (Z)
    // 14-15: Wind Vector  - m/sec (North,East)
    // 16-18: Earth Magnetic Field Vector - gauss (North, East, Down)
    // 19-21: Body Magnetic Field Vector - gauss (X,Y,Z)

    // Constrain quaternion variances
    for (size_t i = 0; i <= 3; i++) {
        P[i][i] = ConstrainFloat(P[i][i], 0.0f, 1.0f);
    }

    // Constrain velocity variances
    for (size_t i = 4; i <= 6; i++) {
        P[i][i] = ConstrainFloat(P[i][i], 0.0f, 1.0e3f);
    }

    // Constrain position variances
    for (size_t i = 7; i <= 9; i++) {
        P[i][i] = ConstrainFloat(P[i][i], 0.0f, 1.0e6f);
    }

    // Constrain delta angle bias variances
    for (size_t i = 10; i <= 12; i++) {
        P[i][i] = ConstrainFloat(P[i][i], 0.0f, sq(0.12f * dtIMU));
    }

    // Constrain delta velocity bias variance
    P[13][13] = ConstrainFloat(P[13][13], 0.0f, sq(1.0f * dtIMU));

    // Wind velocity variances
    for (size_t i = 14; i <= 15; i++) {
        P[i][i] = ConstrainFloat(P[i][i], 0.0f, 1.0e3f);
    }

    // Earth magnetic field variances
    for (size_t i = 16; i <= 18; i++) {
        P[i][i] = ConstrainFloat(P[i][i], 0.0f, 1.0f);
    }

    // Body magnetic field variances
    for (size_t i = 19; i <= 21; i++) {
        P[i][i] = ConstrainFloat(P[i][i], 0.0f, 1.0f);
    }

}

void AttPosEKF::constrainStates()
{
    if (!numericalProtection) {
        return;
    }

    // State vector:
    // 0-3: quaternions (q0, q1, q2, q3)
    // 4-6: Velocity - m/sec (North, East, Down)
    // 7-9: Position - m (North, East, Down)
    // 10-12: Delta Angle bias - rad (X,Y,Z)
    // 13: Delta Velocity bias - m/s (Z)
    // 14-15: Wind Vector  - m/sec (North,East)
    // 16-18: Earth Magnetic Field Vector - gauss (North, East, Down)
    // 19-21: Body Magnetic Field Vector - gauss (X,Y,Z)

    // Constrain dtIMUfilt
    if (!PX4_ISFINITE(dtIMUfilt) || (fabsf(dtIMU - dtIMUfilt) > 0.01f)) {
        dtIMUfilt = dtIMU;
    }

    // Constrain quaternion
    for (size_t i = 0; i <= 3; i++) {
        states[i] = ConstrainFloat(states[i], -1.0f, 1.0f);
    }

    // Constrain velocities to what GPS can do for us
    for (size_t i = 4; i <= 6; i++) {
        states[i] = ConstrainFloat(states[i], -5.0e2f, 5.0e2f);
    }

    // Constrain position to a reasonable vehicle range (in meters)
    for (size_t i = 7; i <= 8; i++) {
        states[i] = ConstrainFloat(states[i], -1.0e6f, 1.0e6f);
    }

    // Constrain altitude
    // NOT FOR FLIGHT : Upper value of 0.0 is a temporary fix to get around lack of range finder data during development testing
    states[9] = ConstrainFloat(states[9], -4.0e4f, 4.0e4f);

    // Angle bias limit - set to 8 degrees / sec
    for (size_t i = 10; i <= 12; i++) {
        states[i] = ConstrainFloat(states[i], -0.16f * dtIMUfilt, 0.16f * dtIMUfilt);
    }

    // Constrain delta velocity bias
    states[13] = ConstrainFloat(states[13], -1.0f * dtIMUfilt, 1.0f * dtIMUfilt);

    // Wind velocity limits - assume 120 m/s max velocity
    for (size_t i = 14; i <= 15; i++) {
        states[i] = ConstrainFloat(states[i], -120.0f, 120.0f);
    }

    // Earth magnetic field limits (in Gauss)
    for (size_t i = 16; i <= 18; i++) {
        states[i] = ConstrainFloat(states[i], -1.0f, 1.0f);
    }

    // Body magnetic field variances (in Gauss).
    // the max offset should be in this range.
    for (size_t i = 19; i <= 21; i++) {
        states[i] = ConstrainFloat(states[i], -0.5f, 0.5f);
    }

}

void AttPosEKF::forceSymmetry()
{
    if (!numericalProtection) {
        return;
    }

    // Force symmetry on the covariance matrix to prevent ill-conditioning
    // of the matrix which would cause the filter to blow-up
    for (size_t i = 1; i < ekf_state_estimates; i++)
    {
        for (uint8_t j = 0; j < i; j++)
        {
            P[i][j] = 0.5f * (P[i][j] + P[j][i]);
            P[j][i] = P[i][j];

            if ((fabsf(P[i][j]) > ekf_covariance_diverged) ||
                (fabsf(P[j][i]) > ekf_covariance_diverged)) {
                current_ekf_state.covariancesExcessive = true;
                current_ekf_state.error |= true;
                InitializeDynamic(velNED, magDeclination);
                return;
            }

            float symmetric = 0.5f * (P[i][j] + P[j][i]);
            P[i][j] = symmetric;
            P[j][i] = symmetric;
        }
    }
}

bool AttPosEKF::gyroOffsetsDiverged()
{
    // Detect divergence by looking for rapid changes of the gyro offset
    Vector3f current_bias;
    current_bias.x = states[10];
    current_bias.y = states[11];
    current_bias.z = states[12];

    Vector3f delta = current_bias - lastGyroOffset;
    float delta_len = delta.length();
    float delta_len_scaled = 0.0f;

    // Protect against division by zero
    if (delta_len > 0.0f) {
        float cov_mag = ConstrainFloat((P[10][10] + P[11][11] + P[12][12]), 1e-12f, 1e-2f);
        delta_len_scaled = (5e-7 / (double)cov_mag) * (double)delta_len / (double)dtIMUfilt;
    }

    bool diverged = (delta_len_scaled > 1.0f);
    lastGyroOffset = current_bias;
    current_ekf_state.error |= diverged;
    current_ekf_state.gyroOffsetsExcessive = diverged;

    return diverged;
}

bool AttPosEKF::velNedDiverged()
{
    Vector3f current_vel;
    current_vel.x = states[4];
    current_vel.y = states[5];
    current_vel.z = states[6];

    Vector3f gps_vel;
    gps_vel.x = velNED[0];
    gps_vel.y = velNED[1];
    gps_vel.z = velNED[2];

    Vector3f delta = current_vel - gps_vel;
    float delta_len = delta.length();

    bool excessive = (delta_len > 30.0f);

    current_ekf_state.error |= excessive;
    current_ekf_state.velOffsetExcessive = excessive;

    return excessive;
}

bool AttPosEKF::filterHealthy()
{
    if (!statesInitialised) {
        return false;
    }

    // XXX Check state vector for NaNs and ill-conditioning

    // Check if any of the major inputs timed out
    if (current_ekf_state.posTimeout || current_ekf_state.velTimeout || current_ekf_state.hgtTimeout) {
        return false;
    }

    // Nothing fired, return ok.
    return true;
}

void AttPosEKF::resetPosition()
{
    if (staticMode) {
        states[7] = 0;
        states[8] = 0;
    } else if (GPSstatus >= GPS_FIX_3D) {

        // reset the states from the GPS measurements
        states[7] = posNE[0];
        states[8] = posNE[1];

        // stored horizontal position states to prevent subsequent GPS measurements from being rejected
        for (size_t i = 0; i < ekf_data_buffer_size; ++i){
            storedStates[7][i] = states[7];
            storedStates[8][i] = states[8];
        }
    }

    //reset position covariance
    P[7][7]   = sq(15.0f);
    P[8][8]   = P[7][7];    
}

void AttPosEKF::resetHeight()
{
    // write to the state vector
    states[9]   = -hgtMea;

    // stored horizontal position states to prevent subsequent Barometer measurements from being rejected
    for (size_t i = 0; i < ekf_data_buffer_size; ++i){
        storedStates[9][i] = states[9];
    }    

    //reset altitude covariance
    P[9][9] = sq(5.0f);
    P[6][6] = sq(0.7f);
}

void AttPosEKF::resetVelocity()
{
    if (staticMode) {
        states[4] = 0.0f;
        states[5] = 0.0f;
        states[6] = 0.0f;
    } 
    else if (GPSstatus >= GPS_FIX_3D) {
        //Do not use Z velocity, we trust the Barometer history more

        states[4]  = velNED[0]; // north velocity from last reading
        states[5]  = velNED[1]; // east velocity from last reading

        // stored horizontal position states to prevent subsequent GPS measurements from being rejected
        for (size_t i = 0; i < ekf_data_buffer_size; ++i){
            storedStates[4][i] = states[4];
            storedStates[5][i] = states[5];
        }          
    }

    //reset velocities covariance
    P[4][4]   = sq(0.7f);
    P[5][5]   = P[4][4]; 
}

bool AttPosEKF::statesNaN() {
    bool err = false;

    // check all integrators
    if (!PX4_ISFINITE(summedDelAng.x) || !PX4_ISFINITE(summedDelAng.y) || !PX4_ISFINITE(summedDelAng.z)) {
        current_ekf_state.angNaN = true;
        ekf_debug("summedDelAng NaN: x: %f y: %f z: %f", (double)summedDelAng.x, (double)summedDelAng.y, (double)summedDelAng.z);
        err = true;
        goto out;
    } // delta angles

    if (!PX4_ISFINITE(correctedDelAng.x) || !PX4_ISFINITE(correctedDelAng.y) || !PX4_ISFINITE(correctedDelAng.z)) {
        current_ekf_state.angNaN = true;
        ekf_debug("correctedDelAng NaN: x: %f y: %f z: %f", (double)correctedDelAng.x, (double)correctedDelAng.y, (double)correctedDelAng.z);
        err = true;
        goto out;
    } // delta angles

    if (!PX4_ISFINITE(summedDelVel.x) || !PX4_ISFINITE(summedDelVel.y) || !PX4_ISFINITE(summedDelVel.z)) {
        current_ekf_state.summedDelVelNaN = true;
        ekf_debug("summedDelVel NaN: x: %f y: %f z: %f", (double)summedDelVel.x, (double)summedDelVel.y, (double)summedDelVel.z);
        err = true;
        goto out;
    } // delta velocities

    // check all states and covariance matrices
    for (size_t i = 0; i < ekf_state_estimates; i++) {
        for (size_t j = 0; j < ekf_state_estimates; j++) {
            if (!PX4_ISFINITE(KH[i][j])) {

                current_ekf_state.KHNaN = true;
                err = true;
                ekf_debug("KH NaN");
                goto out;
            } //  intermediate result used for covariance updates

            if (!PX4_ISFINITE(KHP[i][j])) {

                current_ekf_state.KHPNaN = true;
                err = true;
                ekf_debug("KHP NaN");
                goto out;
            } // intermediate result used for covariance updates

            if (!PX4_ISFINITE(P[i][j])) {

                current_ekf_state.covarianceNaN = true;
                err = true;
                ekf_debug("P NaN");
            } // covariance matrix
        }

        if (!PX4_ISFINITE(Kfusion[i])) {

            current_ekf_state.kalmanGainsNaN = true;
            ekf_debug("Kfusion NaN");
            err = true;
            goto out;
        } // Kalman gains

        if (!PX4_ISFINITE(states[i])) {

            current_ekf_state.statesNaN = true;
            ekf_debug("states NaN: i: %u val: %f", i, (double)states[i]);
            err = true;
            goto out;
        } // state matrix
    }

out:
    if (err) {
        current_ekf_state.error |= true;
    }

    return err;

}

int AttPosEKF::checkAndBound(struct ekf_status_report *last_error)
{

    // Store the old filter state
    bool curr_static_mode = staticMode;

    // Limit reset rate to 5 Hz to allow the filter
    // to settle
    if (millis() - lastReset < 200) {
        return 0;
    }

    if (ekfDiverged) {
        ekfDiverged = false;
    }

    int ret = 0;

    // Reset the filter if the states went NaN
    if (StatesNaN()) {
        ekf_debug("re-initializing dynamic");

        // Reset and fill error report
	    InitializeDynamic(velNED, magDeclination);

        ret = 1;
    }

    // Reset the filter if the IMU data is too old
    if (dtIMU > 0.3f) {

        current_ekf_state.imuTimeout = true;

        // Fill error report
        GetFilterState(&last_ekf_error);

        ResetVelocity();
        ResetPosition();
        ResetHeight();
        ResetStoredStates();

        // Timeout cleared with this reset
        current_ekf_state.imuTimeout = false;

        // that's all we can do here, return
        ret = 2;
    }

    // Check if we switched between states
    if (curr_static_mode != staticMode) {
        // Fill error report, but not setting error flag
        GetFilterState(&last_ekf_error);

        ResetVelocity();
        ResetPosition();
        ResetHeight();
        ResetStoredStates();

        ret = 0;
    }

    // Reset the filter if gyro offsets are excessive
    if (GyroOffsetsDiverged()) {

        // Reset and fill error report
        InitializeDynamic(velNED, magDeclination);

        // that's all we can do here, return
        ret = 4;
    }

    // Reset the filter if it diverges too far from GPS
    if (VelNEDDiverged()) {

        // Reset and fill error report
        InitializeDynamic(velNED, magDeclination);

        // that's all we can do here, return
        ret = 5;
    }

    // The excessive covariance detection already
    // reset the filter. Just need to report here.
    if (last_ekf_error.covariancesExcessive) {
        ret = 6;
    }

    if (ret) {
        ekfDiverged = true;
        lastReset = millis();

        // This reads the last error and clears it
        GetLastErrorState(last_error);
    }

    return ret;
}

void AttPosEKF::attitudeInit(float ax, float ay, float az, float mx, float my, float mz, float declination, float *init_quat)
{
    float initial_roll, initial_pitch;
    float cos_roll, sin_roll, cos_pitch, sin_pitch;
    float mag_x, mag_y;
    float initial_hdg, cos_heading, sin_heading;

    initial_roll = atan2f(-ay, -az);
    initial_pitch = atan2f(ax, -az);

    cos_roll = cosf(initial_roll);
    sin_roll = sinf(initial_roll);
    cos_pitch = cosf(initial_pitch);
    sin_pitch = sinf(initial_pitch);

    mag_x = mx * cos_pitch + my * sin_roll * sin_pitch + mz * cos_roll * sin_pitch;

    mag_y = my * cos_roll - mz * sin_roll;

    initial_hdg = atan2f(-mag_y, mag_x);
    /* true heading is the mag heading minus declination */
    initial_hdg += declination;

    cos_roll = cosf(initial_roll * 0.5f);
    sin_roll = sinf(initial_roll * 0.5f);

    cos_pitch = cosf(initial_pitch * 0.5f);
    sin_pitch = sinf(initial_pitch * 0.5f);

    cos_heading = cosf(initial_hdg * 0.5f);
    sin_heading = sinf(initial_hdg * 0.5f);

    init_quat[0] = cos_roll * cos_pitch * cos_heading + sin_roll * sin_pitch * sin_heading;
    init_quat[1] = sin_roll * cos_pitch * cos_heading - cos_roll * sin_pitch * sin_heading;
    init_quat[2] = cos_roll * sin_pitch * cos_heading + sin_roll * cos_pitch * sin_heading;
    init_quat[3] = cos_roll * cos_pitch * sin_heading - sin_roll * sin_pitch * cos_heading;

    /* normalize */
    float norm = sqrtf(init_quat[0]*init_quat[0] + init_quat[1]*init_quat[1] + init_quat[2]*init_quat[2] + init_quat[3]*init_quat[3]);

    init_quat[0] /= norm;
    init_quat[1] /= norm;
    init_quat[2] /= norm;
    init_quat[3] /= norm;
}

void AttPosEKF::initializeDynamic(float (&initvel_ned)[3], float declination)
{
    if (current_ekf_state.error) {
        GetFilterState(&last_ekf_error);
    }

    ZeroVariables();

    // Reset error states
    current_ekf_state.error = false;
    current_ekf_state.angNaN = false;
    current_ekf_state.summedDelVelNaN = false;
    current_ekf_state.KHNaN = false;
    current_ekf_state.KHPNaN = false;
    current_ekf_state.PNaN = false;
    current_ekf_state.covarianceNaN = false;
    current_ekf_state.kalmanGainsNaN = false;
    current_ekf_state.statesNaN = false;

    current_ekf_state.velHealth = true;
    current_ekf_state.posHealth = true;
    current_ekf_state.hgtHealth = true;
    
    current_ekf_state.velTimeout = false;
    current_ekf_state.posTimeout = false;
    current_ekf_state.hgtTimeout = false;

    fuseVelData = false;
    fusePosData = false;
    fuseHgtData = false;
    fuseMagData = false;
    fuseVtasData = false;

    // Fill variables with valid data
    velNED[0] = initvel_ned[0];
    velNED[1] = initvel_ned[1];
    velNED[2] = initvel_ned[2];
    magDeclination = declination;

    // Calculate initial filter quaternion states from raw measurements
    float init_quat[4];
    Vector3f init_mag_xyz;
    init_mag_xyz   = magData - magBias;
    AttitudeInit(accel.x, accel.y, accel.z, init_mag_xyz.x, init_mag_xyz.y, init_mag_xyz.z, declination, init_quat);

    // Calculate initial Tbn matrix and rotate Mag measurements into NED
    // to set initial NED magnetic field states
    quat2Tbn(Tbn, init_quat);
    Tnb = Tbn.transpose();
    Vector3f init_mag_ned;
    init_mag_ned.x = Tbn.x.x*init_mag_xyz.x + Tbn.x.y*init_mag_xyz.y + Tbn.x.z*init_mag_xyz.z;
    init_mag_ned.y = Tbn.y.x*init_mag_xyz.x + Tbn.y.y*init_mag_xyz.y + Tbn.y.z*init_mag_xyz.z;
    init_mag_ned.z = Tbn.z.x*init_mag_xyz.x + Tbn.z.y*init_mag_xyz.y + Tbn.z.z*init_mag_xyz.z;

    magstate.q0 = init_quat[0];
    magstate.q1 = init_quat[1];
    magstate.q2 = init_quat[2];
    magstate.q3 = init_quat[3];
    magstate.magN = init_mag_ned.x;
    magstate.magE = init_mag_ned.y;
    magstate.magD = init_mag_ned.z;
    magstate.magXbias = magBias.x;
    magstate.magYbias = magBias.y;
    magstate.magZbias = magBias.z;
    magstate.R_MAG = sq(magMeasurementSigma);
    magstate.DCM = Tbn;

    // write to state vector
    for (uint8_t j=0; j<=3; j++) states[j] = init_quat[j]; // quaternions
    for (uint8_t j=4; j<=6; j++) states[j] = initvel_ned[j-4]; // velocities
    // positions:
    states[7] = posNE[0];
    states[8] = posNE[1];
    states[9] = -hgtMea;
    for (uint8_t j=10; j<=15; j++) states[j] = 0.0f; // dAngBias, dVelBias, windVel
    states[16] = init_mag_ned.x; // Magnetic Field North
    states[17] = init_mag_ned.y; // Magnetic Field East
    states[18] = init_mag_ned.z; // Magnetic Field Down
    states[19] = magBias.x; // Magnetic Field Bias X
    states[20] = magBias.y; // Magnetic Field Bias Y
    states[21] = magBias.z; // Magnetic Field Bias Z

    ResetVelocity();
    ResetPosition();
    ResetHeight();
    ResetStoredStates();

    // initialise focal length scale factor estimator states
    flowStates[0] = 1.0f;

    statesInitialised = true;

    // initialise the covariance matrix
    CovarianceInit();

    //Define Earth rotation vector in the NED navigation frame
    calcEarthRateNED(earthRateNED, latRef);
}

void AttPosEKF::initialiseFilter(float (&initvel_ned)[3], double reference_lat, double reference_lon, float reference_hgt, float declination)
{
    // store initial lat,long and height
    latRef = reference_lat;
    lonRef = reference_lon;
    hgtRef = reference_hgt;
    refSet = true;

    // we are at reference position, so measurement must be zero
    posNE[0] = 0.0f;
    posNE[1] = 0.0f;

    // we are at an unknown, possibly non-zero altitude - so altitude
    // is not reset (hgtMea)

    // the baro offset must be this difference now
    baroHgtOffset = baroHgt - reference_hgt;

    InitializeDynamic(initvel_ned, declination);
}

void AttPosEKF::zeroVariables()
{

    // Initialize on-init initialized variables
    dtIMUfilt = ConstrainFloat(dtIMU, 0.001f, 0.02f);
    dtVelPosFilt = ConstrainFloat(dtVelPos, 0.04f, 0.5f);
    dtGpsFilt = 1.0f / 5.0f;
    dtHgtFilt = 1.0f / 100.0f;
    storeIndex = 0;

    lastVelPosFusion = millis();

    // Do the data structure init
    for (size_t i = 0; i < ekf_state_estimates; i++) {
        for (size_t j = 0; j < ekf_state_estimates; j++) {
            KH[i][j] = 0.0f; //  intermediate result used for covariance updates
            KHP[i][j] = 0.0f; // intermediate result used for covariance updates
            P[i][j] = 0.0f; // covariance matrix
        }

        Kfusion[i] = 0.0f; // Kalman gains
        states[i] = 0.0f; // state matrix
    }

    // initialise the variables for the focal length scale factor to unity
    flowStates[0] = 1.0f;

    correctedDelAng.zero();
    summedDelAng.zero();
    summedDelVel.zero();
    prevDelAng.zero();
    dAngIMU.zero();
    dVelIMU.zero();
    lastGyroOffset.zero();

    // initialise states used by OpticalFlowEKF
    flowStates[0] = 1.0f;
    flowStates[1] = 0.0f;

    for (size_t i = 0; i < ekf_data_buffer_size; i++) {

        for (size_t j = 0; j < ekf_state_estimates; j++) {
            storedStates[j][i] = 0.0f;
        }

        statetimeStamp[i] = 0;
    }

    memset(&magstate, 0, sizeof(magstate));
    magstate.q0 = 1.0f;
    magstate.DCM.identity();

    memset(&current_ekf_state, 0, sizeof(current_ekf_state));
}

void AttPosEKF::getFilterState(struct ekf_status_report *err)
{

    // Copy states
    for (size_t i = 0; i < ekf_state_estimates; i++) {
        current_ekf_state.states[i] = states[i];
    }
    current_ekf_state.n_states = ekf_state_estimates;
    current_ekf_state.onGround = _onGround;
    current_ekf_state.staticMode = staticMode;
    current_ekf_state.useCompass = useCompass;
    current_ekf_state.useAirspeed = useAirspeed;

    memcpy(err, &current_ekf_state, sizeof(*err));
}

void AttPosEKF::getLastErrorState(struct ekf_status_report *last_error)
{
    memcpy(last_error, &last_ekf_error, sizeof(*last_error));
    memset(&last_ekf_error, 0, sizeof(last_ekf_error));
}

void AttPosEKF::setIsFixedWing(const bool fixed_wing)
{
    _isFixedWing = fixed_wing;
}

void AttPosEKF::getCovariance(float c[ekf_state_estimates])
{
    for (unsigned int i = 0; i < ekf_state_estimates; i++) {
        c[i] = P[i][i];
    }
}
