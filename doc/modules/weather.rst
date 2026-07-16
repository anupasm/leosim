Weather Module
==============

Overview
--------

The weather module adds environmental effects to LeoSim link evaluation. It models attenuation caused by precipitation, clouds, gases, and scintillation so that the simulator can represent realistic ground-to-satellite link degradation.

Key responsibilities
-------------------

- evolve weather states over time,
- compute rain, cloud, gaseous, and scintillation attenuation,
- provide weather-dependent attenuation values to the channel and beam modules,
- support both deterministic and trace-driven weather inputs.

Primary implementation
----------------------

- ``model/leosim-weather-model.h``
- ``model/leosim-weather-model.cc``

Design notes
------------

The weather module is used when the simulator must capture the effect of atmospheric impairments on link availability and beam quality.

Functional definition
---------------------

LeoSimWeatherModel is the atmospheric state object that turns weather observations into link attenuation values. It combines precipitation, cloud, gaseous, and scintillation effects into the loss terms consumed by the channel and beam evaluation logic.

Typical usage
-------------

Weather state is installed through LeoSimWeatherHelper and then referenced by the channel model during link-quality estimation. It is especially useful in scenarios where rain or cloud fading changes the effective capacity of a satellite link over time.
