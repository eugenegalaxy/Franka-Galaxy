##What Franka needs before moving to SBS implementation:

~~1. Move HOME position (default speed, but can be provided)~~
~~2. Move TRANSPORTABLE position (default speed, but can be provided)~~
3. Grasp gripper (default epsilons, but can be provided), width, speed.
3.1 Optional: stop/move gripper
4. Move Cartesian x,y,z ABSOLUTE
4.1 Optional: rx, ry, rz
5. Move Cartesian x,y,z RELATIVE dislocation
5.1 Optional rx, ry, rz
~~6. Move Joint Angles~~ 
7. Lock/Unlock brakes NOT in webinterface:

*'libfranka' contributor's comment on this*:
> Unfortunately, we have no plans to support opening and closing brakes from libfranka. This has to be done through the web interface.

*HOWEVER! It seems like you can do this by setting HTTP connection and sending a request to the webinterface.
There is a python code for it, need to implement C++ way of doing it.*

1.  Switch auto/manual mode outside of browser app


##Currently implemented:
setDefaultBehavior
moveJointPos
moveHome
moveTransportable
readRobotState
readEEpose
