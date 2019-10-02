OpenSim-based framework to solve trajectory optimization problems using direct collocation and algorithmic differentiation 
==========================================================================================================================
 
**NOTE: This repository has been forked from [Opensim's master source code](https://github.com/opensim-org/opensim-core). Changes have been
made to enable the use of algorithmic differentiation. Consequently, some features of OpenSim have been disabled. Please rely on the original
OpenSim's source code if you do not intend to exploit algorithmic differentiation when solving trajectory optimization problems with our framework.
In addition, please make sure you verify your results. We cannot guarantee that our changes did not affect the original code.**

OpenSim is a software that lets users develop models of musculoskeletal structures and create dynamic simulations of movement. In this work,
we have expanded OpenSim and created a framework for solving trajectory optimization problems using direct collocation methods.
This framework relies on OpenSim for the musculoskeletal structures and multibody dynamics models and on [CasADi](https://web.casadi.org/) for the
nonlinear optimization and algorithmic differentiation. To enable the use of algorithmic differentiation in OpenSim, we have developed a tool named
Recorder that we integrated as part of a modified version of Simbody. More information about this framework and Recorder can be found in this publication.

Solving trajectory optimization problems with our framework allows generating computationally efficient predictive simulations of movement.
For example, you can produce the following predictive simulation of walking with a complex musculoskeletal models (29 degrees of freedom, 92 muscles,
and 6 contact spheres per foot) in only about 20 minutes of CPU time on a single core of a standard laptop computer:

![Predictive simulation of human walking by Antoine Falisse (doi:10.1098/rsif.2019.0402)](doc/images/opensim_predwalking.gif)

More information about how to generate such predictive simulations can be found in [this publication](https://royalsocietypublishing.org/doi/10.1098/rsif.2019.0402).

Brief overview of the framework
-------------------------------

Solving trajectory optimization problems with our framework consists of different steps:

* Build the source code of the modified versions of OpenSim and Simbody that enable the use of algorithmic differentiation ([On Windows using Microsoft Visual Studio](#on-windows-using-visual-studio)).

* Build the OpenSim code intended to be used when formulating the trajectory optimization problem ([Build external functions](#build-external-functions)). For instance, this code may perform inverse dynamics with joint states and controls as input and joint torques as output. We provide a series of examples of how this code may look like in the folder External_Functions. Among them is the code used for generating the predictive simulation in the animation above. We will refer to such code as an external function. You should build this code as an executable.

* Run the executable ([Run executable](#run-executable)). This will generate a MATLAB file, named by default 'foo.m'. This file contains the expression graph of the external function in a format that CasADi can interpret. Expression graphs are at the core of algorithmic differentiation.

* Generate C-code with CasADi ([Generate C-code](#generate-c-code)). From the expression graph generated in the previous step, CasADi can generate C-code allowing to evaluate the (external) function and its derivatives. To generate the C-code, we rely on the code generation feature of CasADi through a few MATLAB commands. We provide a series of examples of how this should be done in the folder cgeneration (details below).

* Compile the generated c-code as a Dynamic Link Library (dll) ([Compile C-code](#compile-c-code)). This dll can then be imported within the CasADi environment when formulating the trajectory optimization problems. 

* Formulate and solve trajectory optimization problem ([Formulate and solve trajectory optimization problem](#formulate-and-solve-trajectory-optimization-problem)). [In this repository](https://github.com/antoinefalisse/3dpredictsim), you can find the code used to generate the predictive simulation in the animation above. [At this line](https://github.com/antoinefalisse/3dpredictsim/blob/master/OCP/PredSim_all.m#L435), we import the dll (compiled in the previous step) as an external function in our environment. We then [evaluate this function](https://github.com/antoinefalisse/3dpredictsim/blob/master/OCP/PredSim_all.m#L1161) when formulating our nonlinear programming problem (NLP). When solving the problem, CasADi provides the NLP solver (e.g., IPOPT) with evaluations of the NLP objective function, constraints, objective function gradient, constraint Jacobian, and Hessian of the Lagrangian. CasADi efficiently queries evaluation of the external function and its derivatives to construct the full derivative matrices.

Building from the source code
-----------------------------

**NOTE -- In all platforms (Windows, OSX, Linux), it is advised to build all OpenSim Dependencies (Simbody, BTK etc) with same *CMAKE_BUILD_TYPE* (Linux) / *CONFIGURATION* (MSVC/Xcode) as OpenSim. For example, if OpenSim is to be built with *CMAKE_BUILD_TYPE/CONFIGURATION* as *Debug*, Simbody, BTK and all other OpenSim dependencies also should be built with *CMAKE_BUILD_TYPE/CONFIGURATION* as *Debug*. Failing to do so *may* result in mysterious runtime errors like 'segfault' in standard c++ library implementation.**

We have developed this project on Windows. We cannot guarantee that this works fine on other platforms although it should. For Max OSX and Ubuntu, get inspired from the [Windows instructions](#on-windows-using-visual-studio) for the modified version of OpenSim while relying on the original instructions for Max OSX and Ubuntu from the [OpenSim-Core git repository](https://github.com/opensim-org/opensim-core).


On Windows using Visual Studio
------------------------------

#### Get the dependencies

* **operating system**: Windows 10.
* **cross-platform build system**:
  [CMake](http://www.cmake.org/cmake/resources/software.html) >= 3.2
* **compiler / IDE**: [Visual Studio 2015](https://www.visualstudio.com/). We started this project before the release of Visual Studio 2017 and 2019, you might experience bugs with these later versions so please stick to Visual Studio 2015 (or contribute to the code to make it work with the newer versions :)). You should be able to find Visual Studio Community 2015 after a little bit of googling.
    * *Visual Studio Community 2015* is sufficient and is free for everyone.
    * Visual Studio 2015 does not install C++
      support by default. During the installation you must select
      *Custom*, and check *Programming Languages > Visual C++ > Common Tools
      for Visual C++ 2015*.
      You can uncheck all other boxes. If Visual Studio is installed without C++
      support, CMake will report the following errors:
      
      ```
      The C compiler identification is unknown
      The CXX compiler identification is unknown
      ```
      
      If you have already installed Visual Studio without C++ support, simply
      re-run the installer and select *Modify*. Alternatively, go to
      *File > New > Project...* in Visual Studio, select *Visual C++*, and click
      *Install Visual C++ 2015 Tools for Windows Desktop*.
* **physics engine**: Simbody >= 3.6. Two options:
    * Let OpenSim get this for you using superbuild (see below); much easier!
    * [Build on your own: be careful you need to build the modified version that enables the use of AD](
      https://github.com/antoinefalisse/simbody/tree/AD-recorder#windows-using-visual-studio).
* **C3D file support**: Biomechanical-ToolKit Core. Two options:
    * Let OpenSim get this for you using superbuild (see below); much easier!
    * [Build on your own](https://github.com/klshrinidhi/BTKCore).
* **command-line argument parsing**: docopt.cpp. Two options:
    * Let OpenSim get this for you using superbuild (see below); much easier!
    * [Build on your own](https://github.com/docopt/docopt.cpp) (no instructions).

#### Download the OpenSim-Core source code modified to enable algorithmic differentiation (OpenSim-AD-Core)

* Clone the opensim-ad-core git repository. We'll assume you clone it into `C:/opensim-ad-core-source`.
  **Be careful that the repository is not on the `master` branch but on the `AD-recorder` branch.** 

  If using a Git Bash or Git Shell, run the following:
  
        $ git clone -b AD-recorder https://github.com/antoinefalisse/opensim-core.git C:/opensim-ad-core-source  
  
  If using TortoiseGit, open Windows Explorer,
  right-click in the window, select **Git Clone...**, and provide the
  following:
    * **URL**: `https://github.com/antoinefalisse/opensim-core.git`.

    * **Directory**: `C:/opensim-ad-core-source`.
    
    * **Checkout the `AD-recorder` branch.**

#### [Optional] Superbuild: download and build OpenSim dependencies
1. Open the CMake GUI.
2. In the field **Where is the source code**, specify
   `C:/opensim-ad-core-source/dependencies`.
3. In the field **Where to build the binaries**, specify a directory under
   which to build dependencies. Let's say this is
   `C:/opensim-ad-core-dependencies-build`.
4. Click the **Configure** button.
    1. Choose the *Visual Studio 14 2015* generator. Make sure
       your build as 64-bit (x64; it's 32-bit by default in later CMake version).
    2. Click **Finish**.
5. Where do you want to install OpenSim dependencies on your computer? Set this
   by changing the `CMAKE_INSTALL_PREFIX` variable. Let's say this is
   `C:/opensim-ad-core-dependencies-install`.
6. Variables named `SUPERBUILD_<dependency-name>` allow you to selectively
   download dependencies. By default, all dependencies are downloaded,
   configured and built.
7. Click the **Configure** button again. Then, click **Generate** to make
   Visual Studio project files in the build directory.
8. Go to the build directory you specified in step 3 using the command:

        cd C:/opensim-ad-core-dependencies-build

9. Use CMake to download, compile and install the dependencies:

        cmake --build . --config RelWithDebInfo

   Alternative values for `--config` in this command are:
   
   * **Debug**: debugger symbols; no optimizations (more than 10x slower).
     Library names end with `_d`.
   * **Release**: no debugger symbols; optimized.
   * **RelWithDebInfo**: debugger symbols; optimized. Bigger but not slower
     than Release; choose this if unsure.
   * **MinSizeRel**: minimum size; optimized.

   You must run this command for each of the configurations you plan to use
   with OpenSim (see below). You should run this command for the release
   configuration *last* to ensure that you use the release version of the
   command-line applications instead of the slow debug versions.
10. If you like, you can now remove the directory used for building
    dependencies (`c:/opensim-ad-core-dependencies-build`).

#### Configure and generate project files

1. Open the CMake GUI.
2. In the field **Where is the source code**, specify `C:/opensim-ad-core-source`.
3. In the field **Where to build the binaries**, specify something like
   `C:/opensim-ad-core-build`, or some other path that is not inside your source
   directory. This is *not* where we are installing OpenSim-Core; see below.
4. Click the **Configure** button.
    1. Choose the *Visual Studio 14 2015* generator. Make sure
       your build as 64-bit (x64; it's 32-bit by default in later CMake version).
    2. Click **Finish**.
5. Where do you want to install OpenSim-AD-Core on your computer? Set this by
   changing the `CMAKE_INSTALL_PREFIX` variable. We'll assume you set it to
   `C:/opensim-ad-core-install`. If you choose a different installation location, make
   sure to use *yours* where we use `C:/opensim-ad-core-install` below.
6. Tell CMake where to find dependencies. This depends on how you got them.
    * Superbuild: Set the variable `OPENSIM_DEPENDENCIES_DIR` to the root
      directory you specified with superbuild for installation of dependencies.
      In our example, it would be `c:/opensim-ad-core-dependencies-install`.
    * Obtained on your own:
        1. Simbody: Set the `SIMBODY_HOME` variable to where you installed
           Simbody (e.g., `C:/Simbody-ad-install`).
        2. BTK: Set the variable `BTK_DIR` to the directory containing
           `BTKConfig.cmake`. If the root directory of your BTK installation is
           `C:/BTKCore-install`, then set this variable to
           `C:/BTKCore-install/share/btk-0.4dev`.
        3. docopt.cpp. Set the variable `docopt_DIR` to the directory
           containing `docopt-config.cmake`. If the root directory of your 
           docopt.cpp installation is `C:/docopt.cpp-install`, then set this 
           variable to `C:/docopt.cpp-install/lib/cmake`.
7. Set the remaining configuration options.
    * `WITH_RECORDER` to compile OpenSim modified to enable the use of algorithmic differentiation.
    * `BUILD_API_EXAMPLES` to compile C++ API examples. Note that most examples will not work with this new version of OpenSim. You could turn this off.
    * `BUILD_TESTING` to ensure that OpenSim works correctly. Note that most tests will fail with this new version of OpenSim. **Nevertheless, you
    should turn this on to build the external functions.**
    * `BUILD_JAVA_WRAPPING` if you want to access OpenSim through MATLAB or
      Java; see dependencies above. Please turn this off (not relevant for our applications).
    * `BUILD_PYTHON_WRAPPING` if you want to access OpenSim through Python; see
      dependencies above. CMake sets `PYTHON_*` variables to tell you the
      Python version used when building the wrappers. Please turn this off (not relevant for our applications).
    * `OPENSIM_PYTHON_VERSION` to choose if the Python wrapping is built for
      Python 2 or Python 3. Please turn this off (not relevant for our applications).
    * `BUILD_API_ONLY` if you don't want to build the command-line applications.
8. Click the **Configure** button again. Then, click **Generate** to make
   Visual Studio project files in the build directory.

#### Build

1. Open `C:/opensim-ad-core-build/OpenSim.sln` in Visual Studio.
2. Select your desired *Solution configuration* from the drop-down at the top.
    * **Debug**: debugger symbols; no optimizations (more than 10x slower).
      Library names end with `_d`.
    * **Release**: no debugger symbols; optimized.
    * **RelWithDebInfo**: debugger symbols; optimized. Bigger but not slower
      than Release; choose this if unsure.
    * **MinSizeRel**: minimum size; optimized.

    You at least want release libraries (the last 3 count as release), but you
    can have debug libraries coexist with them. To do this, go through the
    installation process twice, once for each of the two configurations. You
    should install the release configuration *last* to ensure that you use the
    release version of the command-line applications instead of the slow debug
    versions.
4. Build the libraries. **For our applications, we only need to build osimCommon and osimSimulation, building all libraries will fail.** 
   Right-click on osimCommon in the folder Libraries and select **Build**. Process in the same way for osimSimulation.
5. Copy Simbody DLLs. Right-click on Copy Simbody DLLs and select **Build**.
   
Build external functions
------------------------

In the folder **External_Functions**, you can find a series of example external functions we used for different applications. To add your own external
function, take a look at an example in `C:/opensim-ad-core/OpenSim/External_Functions`. Don't forget to edit the CMakeLists. Your new external function
will appear in Visual Studio after re-configuring through CMake. For the rest of the instructions, we will use the example **PredSim**.

1. Build the external function. Right-click on PredSim and select **Build**. To skip the next step (Run executable), you can also right-click on PredSim, select
**Set as StartUp Project**, click on Debug (toolbar) and click on **Start Without Debugging**. If you followed the second approach, you should find
a MATLAB file `foo.m` in the folder `C:/opensim-ad-core-build/OpenSim/External_Functions/PredSim`.


Run executable
--------------

If you haven't run the executable yet (e.g., through **Start Without Debugging**):
1. Open `C:/opensim-ad-core-build/RelWithDebInfo` in a terminal window (assuming you are in RelWithDebInfo mode):

        cd C:/opensim-ad-core-build/RelWithDebInfo
    
2. Run the executable:

        PredSim.exe
    
You should find a MATLAB file `foo.m` in the folder `C:/opensim-ad-core-build/RelWithDebInfo`.

Generate C-code
---------------

1. Copy `foo.m` into `C:/opensim-ad-core/cgeneration/PredSim`.
2. [Install CasADi](https://web.casadi.org/).
2. Run `C:/opensim-ad-core/cgeneration/PredSim/generateMain.m` in MATLAB. This might take a while to open the folder if `foo.m` is large (you might consider running the file through the command line).

**If you work with a new external function, you might need to adjust the dimension of `arg` as commented out in the MATLAB script**.

After running the MATLAB script, you should find a C file `foo_jac.c` in `C:/opensim-ad-core/cgeneration/PredSim`.

Compile C-code
--------------

1. Open the CMake GUI.
2. In the field **Where is the source code**, specify `C:/opensim-ad-core/cgeneration/PredSim`.
3. In the field **Where to build the binaries**, specify something like
   `C:/opensim-ad-external-function/PredSim/PredSim-build`.
4. Click the **Configure** button.
    1. Choose the *Visual Studio 14 2015* generator. Make sure
       your build as 64-bit (x64; it's 32-bit by default in later CMake version).
    2. Click **Finish**.
5. Where do you want to install the PredSim dll on your computer? Set this by
   changing the `CMAKE_INSTALL_PREFIX` variable. We'll assume you set it to
   `C:/opensim-ad-external-function/PredSim/PredSim-install`.
6. Click the **Configure** button again. Then, click **Generate** to make
   Visual Studio project files in the build directory.
7. Open a command window and go to the build directory you specified in step 3 using the command:

        cd C:/opensim-ad-external-function/PredSim/PredSim-build
8. Use CMake to build the dll:

        cmake --build . --config RelWithDebInfo
9. Use CMake to install the dll:

        cmake --install . --config RelWithDebInfo
        
You should find a file `PredSim.dll` in `C:/opensim-ad-external-function/PredSim/PredSim-install/bin`.
You can now perform the same steps for PredSim_pp instead of PredSim (this is a different external function).

Formulate and solve trajectory optimization problem
---------------------------------------------------

With the libraries `PredSim.dll` and `PredSim_pp.dll`, you have all your need to formulate and solve your trajectory optimization problem
and generate a predictive simulation of walking such as in the animation above.

Clone the 3dpredictsim git repository. We'll assume you clone it into `C:/3dpredictsim`. 

  If using a Git Bash or Git Shell, run the following:
  
        $ git clone https://github.com/antoinefalisse/3dpredictsim.git C:/3dpredictsim 
        
In `C:/3dpredictsim/ExternalFunctions`, you can see that you already have the libraries `PredSim.dll` and `PredSim_pp.dll`. If you want to make
sure that you performed all the steps correctly, delete those libraries and copy the ones you created before.

Run the script `C:/3dpredictsim/OCP/PredSim_all.m`. It should converge in 1045 iterations (Windows 10, MATLAB2017b) and take about 30 minutes (it depends on your laptop).
Open the OpenSim GUI, select the model `C:/3dpredictsim/OpenSimModel/subject1/subject1.osim` and load the motion file `C:/3dpredictsim/Results/PredSim_all/D_c1_v133_T4_N50_E500_Ak50000_AE1000000_P1000_A2000_eE2_G1_M1_Gm0_W0_vM0_pW0_mE0_cc0.mot` (this file has been generated after solving the optimization problem and processing the results). The animation should look like the one above.

Congrats, you generated a three-dimensional muscle-driven predictive simulation of walking!
