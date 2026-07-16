Operator Module
===============

Overview
--------

The operator module represents the administrative or policy layer of LeoSim. It can be used to model network ownership, service constraints, fairness policies, and carrier-specific behaviors.

Key responsibilities
-------------------

- represent operator policies,
- apply service and access constraints,
- coordinate operator-level decisions with beam and routing management.

Primary implementation
----------------------

- ``model/leosim-operator.h``
- ``model/leosim-operator.cc``

Functional definition
---------------------

LeoSimOperatorModel captures network-policy decisions such as service constraints, resource ownership, or carrier-specific access rules. It acts as a policy layer that can influence beam and routing behavior.

Typical usage
-------------

The operator model is used in simulations that need differentiated service behavior between operators or policy domains. It is commonly installed through LeoSimOperatorHelper and consulted by higher-level control logic.
