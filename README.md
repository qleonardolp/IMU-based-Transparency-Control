# Acceleration-based Transparency Control application on an Exoskeleton Knee SEA-Joint

This project consist of an application on MS Visual Studio (C/C++) combining the maxon motor control through EPOS and motion sensing data from Xsens MTw Awinda Kit to develop and apply the acceleration-based transparency control [[1]](https://ieeexplore.ieee.org/abstract/document/7759836) on the 
wearable robot ExoTau, designed by the Robotic Rehabilitation Laboratory (São Carlos School of Engineering).

## Visual Studio solution setup

This solution has been developed on VS Express 2012. Follow the steps below to configure the VS solution in order to run the program:

- Add the 'Win32/include' folder from Xsens SDK installation to "Additional Include Directories" in the solution (Configuration Properties > C/C++ > General);
- Repeat the previous step for the path where [Eigen 3.3.7](http://eigen.tuxfamily.org/dox/GettingStarted.html) was extracted;
- Add the 'Win32/lib' folder from Xsens SDK to "Additional Library Directories" (Configuration Properties > Linker > General);
- Repeat the previous step adding '$(NIEXTCCOMPILERSUPP)\lib32\msvc' to "Additional Library Directories";
- Copy the '.lib' files on Xsens SDK 'Win32/lib' to the **Debug** folder, or wherever the executable is after build the solution in VS. It is required to run the program.

## Using the program

## License

## Acknowledgement

* Guilherme Fernandes
* Wilian M. dos Santos
* Juan Carlos Perez Ibarra
* Félix Maurício Escalante
* Jose Yecid
* Ícaro Ostan
* Jonathan C. Jaimes
* Adriano A. G. Siqueira
* Thiago Boaventura

```
//////////////////////////////////////////\/////////\//
// Leonardo Felipe Lima Santos dos Santos /\     ////\/
// leonardo.felipe.santos@usp.br	_____ ___  ___  //|
// github/bitbucket qleonardolp /	| |  | . \/   \  /|
// *Copyright 2021-2026* \//// //  	| |   \ \   |_|  /|
//\///////////////////////\// ////	\_'_/\_`_/__|   ///
///\///////////////////////\ //////////////////\/////\/
```
