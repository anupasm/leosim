Beam Load Balancer Module
==========================

Overview
--------

The beam load balancer distributes users or traffic across beams in order to reduce overload and improve service quality.

Key responsibilities
-------------------

- monitor beam utilization,
- rebalance traffic when certain beams become overloaded,
- support fairness and congestion management strategies.

Primary implementation
----------------------

- ``model/leosim-beam-load-balancer.h``
- ``model/leosim-beam-load-balancer.cc``

Functional definition
---------------------

LeoSimBeamLoadBalancer monitors load across the available beams and applies balancing logic when one region or beam becomes overloaded. Its purpose is to improve fairness and avoid excessive congestion.

Typical usage
-------------

The balancer is typically engaged when many UEs concentrate on a small set of beams. It is used together with the beam manager and scheduling logic to redistribute demand or adjust eligibility for service.
