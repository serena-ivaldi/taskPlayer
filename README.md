# taskPlayer

This is an import from the taskPlayer module I developed in the context of the MACSI project. 
It can be used with the iCub robot (reality and simulation).

It loads one/multiple trajectory files, saved previously by the module taskRecorder.
Each file contains an arm trajectory recorded by demonstration.
TaskPlayer simply reads the trajectory and executes it when asked.

It works with:
- WoZ
- TaskRecorder
- TaskRecorderGUI

You can send commands to its main rpc port:
- quit => for terminating the module
- STOP => to stop moving
- Load filename => to load the trajectory in filename
- Clear filename => to delete the trajectory of filename
- Display filename => to display the loaded trajectory on the iCubGui
- Play filename initMode controlMode => to play the trajectory of filename: initMode specifies if playing starts from the same initial position of the recorded trajectory, while controlMode defines if the trajectory is executed by velocity, velocity impedance etc (warning: some modes are not fully tested)

