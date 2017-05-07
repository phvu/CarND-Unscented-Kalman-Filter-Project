# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

---

## Project cubic

### Compiling

Project can be compiled with:

    $ mkdir -p build && cd build && cmake .. && make

### Accuracy

MSE on the provided dataset `obj_pose-laser-radar-synthetic-input.txt`:

    $ ./UnscentedKF ../data/obj_pose-laser-radar-synthetic-input.txt ../data/obj_pose-laser-radar-synthetic-output.txt
    Radar enabled, Lidar enabled
    RMSE
    0.0854383
     0.098572
      0.19357
     0.221618
    
This satisfies the requirement RMSE <= [.09, .10, .40, .30] in the project rubic.

### Follows the Correct Algorithm

The algorithm is implemented in `ukf.cpp`:

- The initialization is done between lines 297-318 of `ukf.cpp`

    Initialization of the `p_x` and `p_y` component is easy. I initialized the velocity to be `5 m/s^2` because it
    sounds reasonable.
    
    The covariance matrix `P` is initialized with small value everywhere, to allow some variations in the states in
    the first few measurements. The diagonal is increased a bit, depending on the type of the sensor. Laser tends
    to be more accurate than radar in term of position, so we can use a smaller value to initialize if the first
    measurement is from the lidar.

- The predict step is done between lines 106-192 of `ukf.cpp`

    I first generate the augmented sigma points, then predict the sigma points, and update the state mean and covariance.
    
- The update step is done between lines 198-295 of `ukf.cpp`

### Different kinds of sensors

The program can be instructed to use only data from radar, lidar or both, using the third argument:

    $ ./UnscentedKF
    Usage instructions: ./UnscentedKF path/to/input.txt output.txt [r|l|rl]
     The 3rd parameter specifies the type of sensor to use: r(adar) or l(idar) or rl (both) Default value is rl

| Sensors       | RMSE                     |
|---------------|--------------------------|
| Radar         | `[0.21 0.24 0.22 0.25]`  |
| Lidar         | `[0.20 0.15 0.29 0.26]`  |
| Radar + Lidar | `[0.08 0.098 0.19 0.22]` |

### Visualization

[nis]: imgs/NIS.png "NIS"
[loc]: imgs/location.png "Location"
[velo]: imgs/velocity.png "Velocity"

The _normalized innovation squared_ (NIS) score is below the 95% threshold most of the time:
![alt text][nis]

Here are the comparision of the location and velocity between the predictions and the groundtruth:
![alt text][loc]
![alt text][velo]

---

## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./UnscentedKF ../data/obj_pose-laser-radar-synthetic-input.txt`

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
