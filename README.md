    
[image1]: ./NIS_Radar.png
[image2]: ./NIS_Lidar.png
[image3]: ./Tracking.png

# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric.

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Note that the programs that need to be written to accomplish the project are src/ukf.cpp, src/ukf.h, tools.cpp, and tools.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurment that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF` Previous versions use i/o from text files.  The current state uses i/o
from the simulator.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) as much as possible.

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Instructions and Rubric

This information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/c3eb3583-17b2-4d83-abf7-d852ae1b9fff/concepts/f437b8b0-f2d8-43b0-9662-72ac4e4029c1)
for instructions and the project rubric.

# Follows the Correct Algorithm
### Processing flow
#### General processing flow is shown below:

 Prediction step. Method void UKF::Prediction(double).

1. Generate sigma points. Method MatrixXd UKF::Augmented_Sigma_Points().
2. Predict sigma points. Method void UKF::Sigma_Point_Prediction(MatrixXd, double).
3. Predict mean and covariance. Method void UKF::Predict_Mean_Covariance().
4. Update step for Lidar.
5. Update step for Radar sensor
6. Predict measurement. Method void UKF::Predict_Radar_Measurement(VectorXd*, MatrixXd*).
7. Update state:Method void Update_Radar_State(VectorXd&, VectorXd&, MatrixXd&).
8. Update step for LIDAR
9. Predict measurement:Method void UKF::Predict_Lidar_Measurement(VectorXd*, MatrixXd*).
10. Update state: Method void Update_Lidar_State(VectorXd&, VectorXd&, MatrixXd&).
11. First measurement

### After getting the first measurement UKF does the following:

* Initialisation of the state x_ *(depends on sensor type)* and state covariance matrix P_
* Update the time variable time_us and initialisation flag is_initialised

#### Return to the main() function



*First predict then update*

## Radar and/or lidar measurements handling
This is done using flags radar_flag_ and laser_flag in the following lines from void UKF::ProcessMeasurement(MeasurementPackage meas_package):

  Prediction(dt);
  if (meas_package.sensor_type_ == meas_package.RADAR && radar_flag_)
    UpdateRadar(meas_package.raw_measurements_);
  else if (meas_package.sensor_type_ == meas_package.LASER && laser_flag)
    UpdateLidar(meas_package.raw_measurements_);
  else
    std::cout << "The  Measurement has been skipped." << endl;
    
    



# Results

## Visualisation

The following graph compares real and estimated values for car coordinates using data from Dataset 1

![alt_text][image3]

### Noise parameters
Part of the project was to adjust standard deviations `sta_a_` and `std_yawdd` so that RMSE lies below required thresholds. I have conducted experiments for different values of both parameters. The results are presented in the table below (to be short I used the following format: RMSE(`std_a_`, `std_yawdd_`)):

| Parameter | RMSE (3, 2) | RMSE (2, 2) | RMSE (2, 1) | RMSE (1, 1) | RMSE (0.5, 0.5) | RMSE (0.1, 0.1) |
|:---------:|:-----------:|:-----------:|:-----------:|:-----------:|:---------------:|:---------------:|
|x          |0.0736       | 0.0702      | 0.0701      | 0.0647      | **0.0612**          | 0.1250          |
|y          |0.0873       | 0.0858      | 0.0839      | 0.0837      | **0.0860**          | 0.1351          |
|Vx         |0.3681       | 0.3561      | 0.3446      | 0.3353      | **0.3304**          | 0.4175          |
|Vy         |0.2688       | 0.2541      | 0.2293      | 0.2195      | **0.2135**          | 0.3156          |

We can see how reducing the process noise parameters up to (0.1, 0.1) leads to worse RMSE. I've also plotted NIS (Normalised Innovation Squared) to perform consistency check for both radar and lidar sensors:

![alt_text][image1]

![alt_text][image2]

## RMSE

The accuracy requirement is that the algortihm should perform with RMSE error lower than some threshold values. This shown in tables below for both datasets:

Dataset 1:

| Parameter | RMSE (0.5, 0.5) | RMSE threshold |
|:---------:|:----:|:--------------:|
|x          |0.0612| 0.10           |
|y          |0.0860| 0.10           |
|Vx         |0.3304| 0.40           |
|Vy         |0.2135| 0.30           |

Dataset 2:

| Parameter | RMSE (0.5, 0.5) | RMSE threshold |
|:---------:|:----:|:--------------:|
|x          |0.0887| -              |
|y          |0.0611| -              |
|Vx         |0.6589| -              |
|Vy         |0.2822| -              |


### EKF and UKF comparison

The following table compares RMSE values for [EKF](https://github.com/SIakovlev/CarND_Term2_P1) and UKF filters (`std_a_, std_yawdd = (3, 2)`) using Dataset 1:

| Parameter | EKF-RMSE (3, 2) | UKF-RMSE (3, 2) |
|:---------:|:----:|:--------------:|
|x          |0.0974  | **0.0612**         |
|y          |0.0855  | **0.0860**         |
|Vx         |0.4517  | **0.3304**         |
|Vy         |0.4404  | **0.2135**         |

### UKF: radar and/or lidar measurements

The results are presented for `std_a_, std_yawdd = (3, 2)`:

| Parameter | UKF (L+R) | UKF (L) | UKF (R) |
|:---------:|:---------:|:-------:|:-------:|
|x          |**0.0736**   | 0.1148  | 0.1851  |
|y          |**0.0873**   | 0.1053  | 0.2774  |
|Vx         |**0.3681**   | 0.6386  | 0.3474  |
|Vy         |**0.2688**   | 0.3245  | 0.4307  |


