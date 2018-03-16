/*
 * Header file for the Kalman filter class.
 *
 * Copyright (c) 2017 Alessandro Castiglioni, Davide Molinelli.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _Kalman_h_
#define _Kalman_h_

class Kalman {

public:
    // Default constructor
    Kalman();

    /**
     * Implements the kalman filter in order to filter the angle of the robot
     * with respect to the measurement from the MPU6050
     *
     * - Parameters:
     *      - accAngle  : the accelerometer measurement (represents the output of the system) [in degrees]
     *      - gyroRate  : the gyroscope measurement (represents the deterministic input of the system) [in degrees/sec]
     *      - deltaTime : the time interval since the last update of the system [in sec]
     * - Returns: the estimate measurement of the angle of the robot with respect to the vertical position.
     */
    float getAngle(float accAngle, float gyroRate, float deltaTime);

    /**
     * Set the starting angle of the robot with respect to the vertical position
     *
     * - Parameters:
     *      - angle : the value of the starting angle to be set [in degrees]
     */
    void setAngle(float angle);

    float getBias();
    float getInnovation();

private:
    /** Kalman filter variables
     * V1_angle      : the process noise variance specific to the accelerometer
     * V1_bias       : the process noise variance specific to the gyroscope
     * V2_measure    : the measurement noise variance of the angle computed from the accelerometer
     * angle         : the updated angle computed through the Kalman filter
     * bias          : the updated gyroscope bias computed through the Kalman filter
     * rate          : the unbiased rate
     * prevDeltaTime : the time interval from the previous iteration of the Kalman filter (time t-1)
     * P             : the error covariance matrix [2X2]
     * e             : kalman innovation term computed in the previous run of getAngle function
     */
    float V1_angle;
    float V1_bias;
    float V2_measure;
    float angle;
    float bias;
    float rate;
    float prevDeltaTime;
    float P[2][2];

    float e;

};

#endif
