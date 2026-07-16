Routing Module
==============

Overview
--------

The routing module models how traffic flows through the LeoSim network. It evaluates the path between a source and destination and considers satellite motion, beam associations, and link quality.

Key responsibilities
-------------------

- compute candidate routes,
- select paths for data flows,
- integrate with mobility and beam state,
- support dynamic rerouting when topology changes.

Primary implementation
----------------------

- ``model/leosim-routing.h``
- ``model/leosim-routing.cc``

Functional definition
---------------------

LeoSimRoutingCalculator and the related routing model compute how a flow should traverse the satellite network. They turn the current topology, beam associations, and link states into route decisions for packet forwarding.

Typical usage
-------------

The routing layer is used when simulations need to evaluate end-to-end paths under mobility and handover. It is typically installed through the routing helper and consulted during traffic generation and packet delivery.
