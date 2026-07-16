Traffic Module
==============

Overview
--------

The traffic module produces application-level workload for the simulator. It can generate flows, packet patterns, and service demand that exercise the routing and beam management logic.

Key responsibilities
-------------------

- define traffic sources,
- generate application data patterns,
- drive the network with realistic or configurable demand.

Primary implementation
----------------------

- ``model/leosim-traffic.h``
- ``model/leosim-traffic.cc``

Functional definition
---------------------

LeoSimTrafficGeneratorHelper creates and configures traffic sources that exercise the network stack. It provides the workload that the routing, beam manager, and statistics modules then observe.

Typical usage
-------------

Traffic generation is used in scenarios that need to evaluate throughput, delay, or congestion under a known workload. It is typically installed alongside mobility and routing helpers so the generated traffic follows the dynamic topology.
