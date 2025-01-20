# OCP/MPC Workshop 2024

## Introduction

This repository contains source of training material for the `OCP/MPC Workshop 2024`. The topics covered are:
- Nonlinear programming 
  - Solving nonlinear programs with CasADi
- Optimal control problems
  - Using Rockit to swiftly specify and solve optimal control problems
- Model predictive control
  - Using Impact to rapidly specify, prototype and deploy model predictive controllers.

## Development environment

### Install Conda

If you have not installed Conda, you should first install Miniconda. Miniconda is a minimal installer for Conda. You can install Miniconda by following these steps:

#### For Linux

1. Download the Miniconda installer for Linux from the [Miniconda website](https://docs.conda.io/en/latest/miniconda.html).
2. Open a terminal.
3. Navigate to the directory where you downloaded the installer.
4. Run the installer with the following command, replacing `Miniconda3-latest-Linux-x86_64.sh` with the name of the file you downloaded:
   ```sh
   bash Miniconda3-latest-Linux-x86_64.sh
   ```
5. Follow the on-screen instructions to complete the installation.
6. Close and reopen your terminal to apply the changes.

#### For Windows

For Windows users, follow these steps to install Miniconda:

1. Download the Miniconda installer for Windows from the [Miniconda website](https://docs.conda.io/en/latest/miniconda.html).
2. Double-click the downloaded file to start the installation.
3. Follow the on-screen instructions, making sure to check the option to "Add Miniconda to my PATH environment variable" for an easier use of Conda from the command prompt.
4. After installation, open the Command Prompt or Anaconda Prompt from the Start menu to start using Conda.

### Creating Conda environment

Create Conda environment with *CasADi 3.6.5* + *Rockit* + *Impact* using the following command:

```[sh]
conda env create -f mecoverse_environment.yml
```

This will create a conda environment called `mecoverse`.

### Start using the created Conda environment

Activate the conda environment by running in a terminal
```sh
conda activate mecoverse
```

## Development environment using robot simulator

For the second part of the Impact tutorial you will need to install additional libraries in your environment. Since some of these libraries downgrade the version of CasADi, we recommend you to create a separate Conda environment for this.  To create a Conda environment that includes Pinocchio, the Robotics Toolbos for Python and the RobotsMECO library (to generate symbolic robot models compatible with CasADi using Pinocchio), just execute the following command:

```sh
conda env create -f mecoverse_robotics_environment.yml
```


This will create a conda environment called `mecoverse-robotics`.

### Start using the created Conda environment

Activate the conda environment by running in a terminal
```sh
conda activate mecoverse-robotics
```

You can follow the instructions provided in the [instructions](./Tutorials/3_impact/part-2/instructions.pdf) file for the Impact tutorial - Part 2.

***

This workshop has received funding from the *MIT-Belgium - KU Leuven Seed Fund* within the framework of the *MIT International Science and Technology Initiatives (MISTI)* grant programme.

***

<a rel="license" href="http://creativecommons.org/licenses/by/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by/4.0/80x15.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by/4.0/">Creative Commons Attribution 4.0 International License</a>.

