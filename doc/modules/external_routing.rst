External Routing Module
========================

Overview
--------

The external routing module captures interactions between LeoSim and outside routing logic. It is useful when the simulator needs to interface with a broader routing framework or an external control plane.

Key responsibilities
-------------------

- expose routing decisions to external controllers,
- support integration with external path selection logic,
- bridge LeoSim state with external routing components.

Primary implementation
----------------------

- ``model/leosim-external-routing.h``
- ``model/leosim-external-routing.cc``

Functional definition
---------------------

LeoSimExternalRoutingHelper exposes LeoSim routing decisions to an external control or routing layer. It allows the simulator to interoperate with an external logic that wants to inspect or override path selection.

Typical usage
-------------

This module is used when the simulation topology needs to be coordinated with an outside routing policy or test harness. It is especially relevant for research scenarios that compare native LeoSim routing to external algorithms.
