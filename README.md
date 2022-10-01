# Implementation Notes for Reviewer

I would like to document some of my understanding and observations as part of working on this assignment of unscented Kalman filter. I have started to work on this assignment in my local machine and initially struggled with building and executing the project due to multiple memory allocation related errors. I suspect that the root of some of these errors are hidden in the intended PCL version and the Eigen version it depends on for this project. Regardless, I have implemented the Kalman filter in the UKF class. Additionally, I have been able to integrate a functional  plotting mechanism in the Udacity workspace in order to evaluate the implementation with the help NIS (Normalized Innovation Squared) metric.

## My understanding and some implementation details

My final implementation of the UKF class exposes `ProcessMeasurement(MeasurementPackage const &meas_package)` and `Prediction(double delta_t)` as the public API layer functionality of the UKF along with some required attributes to access the NIS score from `Highway` class, after executing the predict and update steps for each car. When `tools.lidarSense` and `tools.radarSense` methods are invoked for each car, from `Highway::stepHighway` method for Lidar and Radar sensors respectively, in each case we execute one cycle of `Predict -> Update` phase of UKF. 

In the `Predict` phase we compute the apriori state vector mean and covariance matrix. To do that we first **augment the state** (incorporate longitudinal and yaw acceleration noise components into state vector and state covariance matrix) and compute augmented sigma points. Then we execute process model to compute a priori state vector and covariance matrix.

In the `Update(Lidar|Radar)` we receive a measurement for time step k+1. We first translate the apriori sigma points, state vector mean, state covariance matrix into measurement space by executing measurement model. For Radar measurement model is nonlinear function. For Lidar it is straightforward. We then incorporate the incoming measurement. Finally, we compute Kalman gain and transform the apriori state vector and covariance matrix to posterior belief about the state.

- For the Lidar I have included an experimental update method `UKF::updateLidarExperimental(MeasurementPackage const &meas_package)` to see the effect on the estimate if we rely on linear transformation only while updating the state. The outcome was not so great state estimate and it was not within the desired RMSE threshold. My interpretation of the outcome is that this way of updating the state is suited for a purely linear motion model. But when we are considering variables such as yaw angle and yaw rate for CTRV model this way of updating the state may not result in accurate estimates. In the final version I have disabled this experimental method but kept for your review.

**I would like to have your feedback on my understanding of the unscented Kalman filter implementation and my interpretation on the behavior of `UKF::updateLidarExperimental(MeasurementPackage const &meas_package)` method.**

## Choice of Process Noise components and NIS Evaluation

I started with both longitudinal and yaw acceleration noise components value as 1 and 1. However after some iterative executions and observations in the NIS plot I came into the conclusion that the estimations tend to be better when longitudinal acceleration noise `std_a_` is slightly higher than yaw acceleration noise `std_yawdd_`. This conclusion of mine is purely based on the observations in the NIS plot. I have made some experiments with different set of values and some of my observations in terms of NIS are as follows. For the final submission I have kept the values as `3.` and `1.` for the aforementioned noise components.

- Longitudinal Acceleration Noise: `1.` Yaw Acceleration Noise: `2.`

<img src="./NIS_UKF_1_2.png" alt="Longitudinal Acceleration Noise: `1.` Yaw Acceleration Noise: `2.`">

- Longitudinal Acceleration Noise: `1.5` Yaw Acceleration Noise: `1.`

<img src="./NIS_UKF_1.5_1.png" alt="Longitudinal Acceleration Noise: `1.5` Yaw Acceleration Noise: `1.`">

- Longitudinal Acceleration Noise: `2.` Yaw Acceleration Noise: `1.`

<img src="./NIS_UKF_2_1.png" alt="Longitudinal Acceleration Noise: `2.` Yaw Acceleration Noise: `1.`">

- Longitudinal Acceleration Noise: `3.` Yaw Acceleration Noise: `1.`

<img src="./NIS_UKF_3_1.png" alt="Longitudinal Acceleration Noise: `3.` Yaw Acceleration Noise: `1.`">

In order to include this metric I have made minor adaptions into the `CMakeLists.txt`, `highway.h`, `main.cpp` and included a file in the workspace `matplotlib.h` from [C++ Matplotlib Wrapper](https://github.com/lava/matplotlib-cpp). I have experimented and verified that included plotting functionality works in the Udacity workspace. Hence, I have kept the plotting mechanism enabled for final submission.

**I would like to have your feedback on, how far my observations are reasonable.**

Thanks a lot for your time to review my project and your feedbacks on any improvement ideas.



