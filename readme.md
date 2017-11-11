
## Program structure

- obj_pose-laser-radar-synthetic-input.txt  
A data file download from course web site, put it in the same folder with excutable file.
- Eigen folder
An C++ library for operate matix and vector and so on. Put this folder in `src` folder.
- main.cpp
The main.cpp readin the data file, extract data to a "Measurement package". Creat a tracking instance to analyze the data.
- kalman_filter.h
Declere `KalmanFilter` class
- kalman_filter.cpp
Implement `KalmanFilter` functions, `Predict()` and `Update()`
- tracking.h
Declare an `tracking` class, it include an `KalmanFilter` instance.
- tracking.cpp
The constructor function `Tracking()` declare the size and initial value of `Kalman filter` matixes.
The `ProcessMeasurement` function process a single measurement, it compute the time elapsed between 

## makefile structure

