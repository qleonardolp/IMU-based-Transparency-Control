# Acceleration-based Transparency Control application on an Exoskeleton Knee Joint

This project consist of the development of an application on MS Visual Studio (C/C++) that binds togheter the control of a maxon motor, with EPOS, using IMUs data, in this case Xsens MTw sensors.
With this hardware, using the EPOS command libraries and Xsens SDK herein is applied the acceleration-based transparency control [[1]](https://ieeexplore.ieee.org/abstract/document/7759836) on a 
wearable robot: the lower limbs exoskeleton designed by the Robotic Rehabilitation Laboratory (São Carlos School of Engineering).

## Setting up the Visual Studio solution

This solution has been developed under VS Express 2012, but feel free to try under another version. Follow these steps to run the project after clone this repository:

- Add the 'Win32/include' directory path of this repository in your computer to the include directories ...
- Add the 'Win32/lib' to ...
- Copy the '.lib' files on 'Win32/lib' to the Debug folder, or wherever is the executable, after build the solution in VS. It is necessary to run the program.

## How to use the application

...
