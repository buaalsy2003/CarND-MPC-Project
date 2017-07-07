# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
# Model
This project is to build a Motion Predictive Controller(MPC). I used the kinematic model without internal forces:
* x1 = x0 + v0 * cos(psi0) * dt
* y1 = y0 + v0 * sin(psi0) * dt
* psi1 = psi0 + (v0 / Lf) * delta0 * dt
* v1 = v0 + a0 * dt
* cte1 = f(x0) - y0 + v0 * sin(psi0) * dt
* epsi1 = psi0 - arctan(f'(x0)) + (v / Lf) * delta0 * dt

The simulator states:
* x: position x
* y: position y
* psi: current vehicle speed
* v: current vehicle speed

The Actuators:
* cte:  cross track error
* epsi: psi error

# Parameters (N and dt)
We need to fine tune the two parameters in this model.
N is number of timesteps and dt is the timeperiod delta. The MPC model works by creating a reference trajectory for T seconds ahead on the horizon of its current position (T = N * dt). Larger T means longer horizon and smoother changes over time, but also larger deviations in tracking as horizons can change dramatically; smaller T means shorter horizons, more responsive and accurate, but more discrete changes. 
I started with small values of N = 15 and dt = 0.02. Then I gradually increase the N and dt, I settled down to N = 20 and dt = 0.05, with velocity slowly increasing from ~30 mph to 75 mph.

# Polynomial Fitting and MPC Preprocessing
All the points was first converted to and then calculated under the vehicle coordinate system using function (transformGlobalToLocal).
Then using the fitted poly and the vehicle state, I called the solver to minimize the cost function over time.
The cost function consists of amplitude of cte, velocity difference, steering angle, throttle value and the derivatives of steering angle and throttle value.
The weights of each component in cost function is important to the final optimal soluation. To make the vehicle driving smoother, I assigned a large weight to the derivative of steering angle.
Eventually the weights of all components in the cost function were set in MPC.h as follows:
* const int kd   = 1;
* const int ka   = 10;  
* const int kds  = 800;
* const int kas  = 10;

To make the driving easier, the desired driving speed was set to 75. This can be changed to a higher value but the model will become more sensitive to weights. 
* const double ref_v = 75;;

# Latency
In this simulator, the latency is 100 ms, which is 2 dt timesteps. We need consider the latency delays in order to keep the simulated vehicle in a predictable state. To do so, I added the constraints for the duration of latency. The actuations values of previous iteration were stored and applied for the duration of latency; this ensures that actuations are smooth, and optimal trajectory is calculated starting from time after the latency.

----------------------------------------------------------------------------------------
## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
