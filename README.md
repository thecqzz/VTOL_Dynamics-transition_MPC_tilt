# VTOL_Dynamics

### Description
This project aims to provide a new MPC control strategy for an electric tilt-rotor vertical take-off and landing aircraft (eVTOL). 
<img width="650" alt="1" src="https://github.com/thecqzz/VTOL_Dynamics-transition_MPC_tilt/assets/114604817/0f71122d-3d9b-4348-93c9-6710e64dd9e8">

VTOL is a combination of multicopper and fixed-wing capable of taking off vertically while being energy efficient in long range.
<img width="650" alt="3" src="https://github.com/thecqzz/VTOL_Dynamics-transition_MPC_tilt/assets/114604817/710757f3-f5bc-4d96-afe6-4f90af38fdb0">

There are three different major types of VTOL in the market. Among them, the titrator VTOL has the highest controllability but is mechanically complex. 

Traditional fused PID control method for tilt-rotor VTOL requires controller blending, requiring different controllers at different flight modes. This makes the transition unsmooth and tracking unstable. Thus, developing a new controller strategy for such aircraft is necessary.

### Installation

1. In a command prompt, create a project workspace (optional):
   
    ```bash
    mkdir project_workspace
    ```

2. Clone the project repository:
   
    ```bash
    git clone https://github.com/thecqzz/VTOL_Dynamics-transition_MPC_tilt.git
    cd VTOL_Dynamics-transition_MPC_tilt
    ```

3. Check available branches using:
   
    ```bash
    git branch -r
    ```

4. Switch to the branch with the most features ("failure" branch):
   
    ```bash
    git checkout failure
    ```

5. Pull the latest changes (optional):
   
    ```bash
    git pull
    ```

6. Double-check the current branch (optional):
   
    ```bash
    git branch
    ```
### Usage
When first running the code, there may be some MATLAB toolboxes you need to install. Run the code several times and install the required toolboxes suggested by MATLAB.

1.
