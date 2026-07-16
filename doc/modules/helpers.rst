Helpers Module
==============

Overview
--------

The helpers module collects convenience utilities and setup classes that make LeoSim easier to instantiate in simulation scripts. These helpers are often used from example scenarios and test programs.

Key responsibilities
-------------------

- provide reusable construction helpers,
- simplify node and device installation,
- encapsulate common configuration steps.

Primary implementation
----------------------

- ``helper/`` directory

Functional definition
---------------------

The LeoSim helper classes provide the public API for constructing and configuring the core modules. They wrap the lower-level model classes and make scenario setup more concise and reusable.

Typical usage
-------------

Helpers are used from example scripts and test programs to install loaders, mobility models, weather effects, routing calculators, beam managers, traffic sources, and statistics collection in a single setup flow.
