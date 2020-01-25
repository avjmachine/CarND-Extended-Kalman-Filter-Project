# Extended Kalman Filter Project - Project Writeup

In this project, I have utilized a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.The project requires obtaining RMSE values that are lower than the tolerance outlined in the [project rubric.](https://review.udacity.com/#!/rubrics/748/view) 

[//]: # (Image References)

[image1]: ./writeup_images/kalman_cartoon_explanation.png "Kalman Filter Explanation Diagram"
[image2]: ./writeup_images/kalman_equations.png "Kalman Filter Equations"

## To run this code

Installations required:
* uWebSocketIO
* Udacity Term-2 Simulator
* cmake >= 3.5
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
* gcc/g++ >= 5.4
For more information on installation instructions and details refer the [original link](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project/blob/master/README.md) from Udacity.

The project can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

## Introduction
The project objective is to track the x,y position and velocities of a bicycle as observed from a stationary vehicle using Lidar and Radar sensors. The sensor data is fed by a simulator to our main project code through a uWebSocketIO connection.

This data is initially stored in the form of a [text file] (data/obj_pose-laser-radar-synthetic-input.txt) with each row representing a measurement from one of these sensors at a different time. The columns contain information such as the sensor type (laser or radar), sensor measurements such as positions and velocities and the corresponding ground truth values. Our main code receives this information from the simulator through the uWebSocketIO connection and processes it using an Extended Kalman Filter to give us a final filtered output to track the bicycle's position and velocity.

## Description of working of the Extended Kalman Filter

A Kalman filter is basically an algorithm to estimate the state of a system such as position, orientation, velocity, pressure, temperature, voltage, etc. using a combination of physical laws and measurements. We need the filter since only measurements from sensors are usually noisy and only predictions based on physical laws could be erroneous due to uncertainities. The Kalman filter combines inputs from both these sources using laws of probabilities, based on the uncertainities. 

It is basically a two step process - a predict step followed by an update(or measurement) step, which is repeated cyclically. In the first predict step, the new state(for example, position) is predicted using a known physical law (based on certain assumptions such as constant velocity). The uncertainity in this state is noted down. As a second step, this state is updated based on the actual measurement (say, coming in from a position sensor). During this update step, the position is updated based on the ratio of the uncertainities in both the steps. It is similar to taking a weighted average of two quantities, with the higher weight being assigned to the value with lower uncertainity and a lower weight being assigned to the value with higher uncertainity.  

This ratio which is used for the weighing is called the Kalman gain.

The Kalman filter can be understood with the help of the following diagram.
![alt text][image1]

These two steps can be implemented using the following two sets of equations:
![alt text][image2]

where x is the state vector describing the quantity being tracked such as position, velocity, pressure, temperature, voltage, etc.;
P is the state covariance matrix, indicating the uncertainity in the quantity;
F is the state transformation matrix, updating the predictions from time to time based on the physical laws;
Q is the process covariance matrix, consisting of the noise in the process;
z is the sensor measurement, H is the measurement matrix transforming the state vector to the space of the measurement values;
R is the uncertainity in the sensor measurement;
S is the sum of the uncertainities - state covariance and the sensor noise;
K is the ratio of the uncertainity in predict step(state covariance) with respect to the sum of the uncertainities(prediction + measurement);
The prime symbol (') is used with x and P to indicate that it is updated after the prediction step.


In case the prediction and/or measurement steps consist of nonlinearities in place of the linear operations performed by the F and/or H matrix on the state vector x, we need to use an Extended Kalman filter. This is because non-linear transformations on the state vector do not keep the Gaussian/normal assumption valid. The Gaussian assumption is core to the Kalman filter algorithm. In case of such non-linearities, the transformations have to be linearized in the neighborhood of the predicted/measured values. Thus, an Extended Kalman Filter differs from a normal Kalman Filter in the following manner:

a.The H matrix is replaced by its Jacobian matrix Hj when calculating  S, K and P.
b.The F matrix is replaced by its Jacobian Fj matrix when calculating P'.
c.x' is calculated using prediction update function, f instead of matrix F.
d.y is calculated using the function h, instead of matrix H.

In this particular project, the predict step is not affected and will continue to be the same, the measure step for the lidar will also continue to be the same, but the measure step for the radar will use the extended Kalman Filter equations due to non-linear relationship between the measurement z and the state vector x.

## Brief explanation of key blocks in the code

The code consists of the following files:

1. `main.cpp` - This file contains the code to interface with the simulator - to receive the measurement data and ground truths , to supply the algorithm with the measurement values one by one line by line(to the `FusionEKF.cpp`), to call the Kalman filter predict and update steps(in `kalman_filter.cpp`) and receive the Kalman estimated values, to call the Jacobian and performance calculation (`CalculateJacobian`` and `CalculateRMSE` in `tools.cpp`) function and send the estimates and RMSE back to the simulator using the uWebSocketIO connection.
2. `FusionEKF.h` and `FusionEKF.cpp` - These files contain the header with the class and function definitions and the code for processing the received measurements supplied by the`main.cpp`. This segment of the code checks the type of sensor, initializes the state vector and the other matrices used in the calculation, stores and calculates the time difference between subsequent measurements for use in the prediction step, calls the predict and either the update or updateEKF functions based on the type of sensor (laser/radar) in `kalman_filter.cpp`. It also calls the CalculateJacobian function in `tools.cpp` to assign the Jacobian matrix Hj as H, before the update step, in case the sensor is radar.
3. `tools.h` and `tools.cpp` - These files contain the class headers and definition for the calculation of the Jacobian of the measurement matrix, Hj and a measure of the error between the Kalman estimate and the ground truth.
4. `kalman_filter.h` and `kalman_filter.cpp` - These files have the headers and definition for the prediction and update steps. The Kalman filter equations can be found in this segment of the code. There is a common predict step for both the laser and the radar sensors, but a separate measurement update step for each of the laser and the radar sensors. The Update function for the laser sensor uses the normal Kalman update equations, whereas the UpdateEKF function for the radar uses the extended Kalman filter equations, where the H matrix is replaced by the Jacobian Hj matrix for all equations, and the y calculation uses a non-linear h function transforming the cartesian state vector variables to the polar coordinate variables instead of the simple linear transformation using H matrix.
5. `measurement_package.h` - header file with class definition to store sensor type, timestamp and the actual sensor data.
6. `json.hpp` - for communication with the Udacity Unity simulator.
7. `Eigen` library - for vector and matrix operations.

The items 1, 5, 6 and 7 are not modified and are used directly, while the code in 2,3 and 4 has been significantly modified and filled in to program the Kalman filter and measure the performance. 


---

## Project Requirements in the Project Rubric
Here, in this section I mention the project requirements given in the project rubric, all of which have been met.

### Compiling
The code compiles using cmake and make without errors.

### Accuracy
px, py, vx, vy output coordinates have an RMSE <= [.11, .11, 0.52, 0.52] when using the file: "obj_pose-laser-radar-synthetic-input.txt", which is the same as the data file the simulator uses for Dataset 1.

### Correct Algorithm

#### Flow 
The Sensor Fusion algorithm used here follows the general processing flow of first initializing the state vectors and matrices, then predicting the state based on the time that has passed by till the next incoming measurement, and finally updating the state using measurements. This goes on in a cyclic manner of prediction and updation.
#### First measurements
The code uses the first measurements to initialize the state vectors and covariance matrices.
#### Sequence - Predict, then Update
After the first measurement to initialize the vectors and matrices, for every subsequent measurement, the algorithm used in the code predicts the object position to the current timestep and then updates the prediction using the new measurement.
#### Handle both radar and lidar
The algorithm used here sets up the appropriate matrices (such as H and Hj) based on the incoming type of measurement (such as laser and radar) and calls the correct measurement function (such as Update and UpdateEKF) for the given sensor type.

### Code Efficiency
The code is written keeping in mind efficiency as a key criteria without necessarily sacrificing comprehension, stability, robustness or security for speed. For example, I have tried to avoid running the exact same calculation repeatedly, wherever I could run it once, store and reuse. I would appreciate your feedback on improving the code efficiency and style, since I am a beginner and wish to upgrade my skills.
