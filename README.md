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
7. Lock/Unlock outside of browser app
8. Switch auto/manual mode outside of browser app


##Currently implemented:
setDefaultBehavior
moveJointPos
moveHome
moveTransportable
readRobotState
readEEpose
