Mobility Module
===============

Overview
--------

The mobility module defines how LeoSim nodes move through the simulated environment. It supports moving satellites, fixed gateways, and stationary or mobile user equipment.

Key responsibilities
-------------------

- represent node positions and velocities,
- support waypoint-based mobility updates,
- manage satellites moving along orbital trajectories,
- provide a common mobility interface to the channel, beam, and routing layers.

Primary implementation
----------------------

- ``model/leosim-mobility-model.h``
- ``model/leosim-mobility-model.cc``

Design notes
------------

The mobility model is one of the most important building blocks for a LEO simulation because all downstream behavior depends on node positions changing over time.
