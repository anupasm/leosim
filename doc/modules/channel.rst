Channel Module
==============

Overview
--------

The channel module evaluates the quality of links between LeoSim nodes. It manages both satellite-to-ground links and inter-satellite links and computes whether those links are available, degraded, or down.

Key responsibilities
-------------------

- maintain the link database for active node pairs,
- compute distance-based link state,
- evaluate path loss and signal strength,
- support dynamic updates as nodes move,
- expose channel quality metrics to beam management and routing.

Primary implementation
----------------------

- ``model/leosim-channel-model.h``
- ``model/leosim-channel-model.cc``
- ``model/leosim-channel.h``
- ``model/leosim-channel.cc``

Relevant concepts
-----------------

The channel module is the first place where radio propagation and link viability are assessed. Its output is consumed by the beam and routing layers to make access and forwarding decisions.
