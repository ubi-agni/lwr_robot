## Payload data

For better control, **payload data** should be **given** to the controller (cannot retrieve from itself yet). This is similar to the real kuka controller, in which tools inertia parameters must be provided

Currently, only mass and center of mass are handled.

Because the urdf model of the kuka puts the mass of the flange at the surface of the flange, the user must provide the distance between the kuka flange surface and the center of mass of the tool, as well as the mass of the tool. 
This depends on the mounting offset in the urdf.

The mass of the tool should also be given

Both data are passed via the urdf of the robot arm with these two variables :

 * **payloadMass**
 * **payloadCOG**

