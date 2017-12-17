# CarND PID Controller

![alt text](results/CarND-PiD.mp4.gif "Result")

## The Project

In this project we'll revisit the lake race track from the [Behavioral Cloning Project](https://github.com/ricardosllm/CarND-Behavioral-Cloning). 
This time, however, we'll implement a PID controller in C++ to maneuver the vehicle around the track.

The simulator will provide you the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle.

The project is considered successfully completed if the following is met:

`No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle).`

## Tuning hyperparameters

I took an iterative approch to tunning the hyperparameters. Started with a set of candidate values based on the lectures and literature found online and proceeded to iteratively change one param and observe the results.
To help my decision I've calculated the absolute average CTE at the 1 lap mark. 
```c++
avg_cte = (runs + abs(cte))*avg_cte / (runs + 1);
runs = runs + 1;

std::cout << "Runs: " << runs << std::endl;
std::cout << "Avg CTE: " << avg_cte << std::endl;
```
The following tables show the results and chosen values.

Formula with starting values
```c++
  double Kp = 2.5 * ku;
  double Ki = ku / (100.0);
  double Kd = ku * (200.0 / 8.0);
```
> Bold indicates param change being evaluated

| Kp | Ki | Kd | Avg CTE | Observation | 
|----|----|----|---------|-------------|
| 2.5| 100| 200 / 8 | 0.0316567 | |
| **2.9**| 100| 200 / 8 | 0.0288258 | |
| **1.9**| 100| 200 / 8 | 0.0416524 | |
| 2.9| **200**| 200 / 8 |  | of track immediately|
| 2.9| **50**| 200 / 8 | 0.0347172 | |
| 2.9| 100| **100** / 8 | | huge oscillation |
| 2.9| 100| **400** / 8 | 0.0247625 | |
| 2.9| 100| **800** / 8 | 0.0209387 | bettter avg cte but too rough turns |
| 2.9| 100| **600** / 8 | 0.0248945 | still big oscillation |
| 2.9| 100| **300** / 8 | 0.0245044 | much better |
| 2.9| 100| 300 / **4** | 0.0237836 | |
| 2.9| 100| 300 / **3** | 0.0224195 | |
| 2.9| 100| 300 / **2** | 0.0251381 | |
| 2.9| 100| 300 / 3 | 0.0224195 | **Chosen values** |

> Now tunning ku

| ku | Avg CTE | Observations |
|----|---------|--------------|
|0.06|0.0224195| Starting value|
|0.03|0.0395097| avg CTE too high |
|0.09|0.0204704|perhaps too oscillating|
|0.12|0.0185899|too much oscillation|
|0.07|0.0189136| **Chosen value**|

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

