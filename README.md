# SmartCar
A smart autonomous vehicle project designed for the 2022 National Undergraduate Intelligent Car Competition in China. This project features integrated hardware and software systems to achieve autonomous navigation, path planning, and real-time control.
<p align="center">
  <img src="https://github.com/Vincentive1232/SmartCar/blob/Software/Pics/Pic3.jpg" width="300" alt="Cover">
</p>


## Task Description
utilize a two-wheeled balancing robot equipped with a camera to locate targets emitting infrared signals within a designated area. The targets will appear randomly during the competition, and the goal is to locate and destroy them as quickly as possible.

## Project Structure
- `Software`: 2D and 3D Dynamics Simulation for Crazyflie ("X" configuration).
  - `Balance——Code_Final`: Simulation code for a 2D Multirotor model.
  - `Computer_Monitor_and_Controller`: Simulation code for a 3D Multirotor model with Euler Integration only.
  - `Image_Processing_Simulation`: Simulation code for a 3D Multirotor model with Euler Integration and RK4 Integration.

- `Hardware`: Implementation of lee-controller in Rust, based on the Dynamics code.
  - `MM32F3277_Balance`: Extension board for balance robot using MM32F3277 as core processor.
  - `DC_MotorDriver_2Channel`: 2-Channel Driver Board using DRV8701 for DC motors
 
## Notes
- The source code was created by using IAR, so please first install [IAR](https://www.iar.com/embedded-development-tools/free-trials?hsCtaAttrib=209032649949).
