# walknet-curvewalking
Ongoing reimplementation of the Walknet decentralized architecture for six-legged robots with the goal of adjusting it for curve-walking.

## Excecuting movements
In order to execute a single swing, a single stance movement or a walking behavior the appropriate function must be selected in the controller/singl_leg_controller.py in the main function (lines 154 to 157):

`legController.manage_walk()`

`# legController.bezier_swing()`

`# legController.manage_swing()`

`# legController.manage_stance()`

Currently the manage_walk() function is used. Running this code will execute alternating swing and stance movements of the left middle leg.
In order to execute a single swing movment the manage_swing() function must be used and in order to execute a single stance movment the manage_stance() function must be used. The bezier_swing() function is not executable as of yet.
To change the leg which should execute the movement the name of the leg with which the SingleLegController-Constructor is called needs to be changed. This can be done in the controller/singl_leg_controller.py in the main function (line 151):

`legController = SingleLegController('lm', nh, True)`

Here lm stands for left middle leg, lf would stand for left front leg, lr for left rear leg and rm, rf and rr for the corresponding right legs (middle, front, rear).
