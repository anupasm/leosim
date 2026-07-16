Visualization Module
====================

Overview
--------

The visualization module supports trace export and presentation of results. It helps users inspect network state, beam assignments, mobility paths, and traffic behavior after a simulation run.

Key responsibilities
-------------------

- export traces for visualization tools,
- annotate simulation events,
- support animator or external analysis workflows.

Primary implementation
----------------------

- ``model/leosim-visualization.h``
- ``model/leosim-visualization.cc``

Functional definition
---------------------

LeoSimVisualizationHelper exports simulation state into a form that can be inspected visually or consumed by external analysis tools. Its role is to annotate and organize traces for later interpretation.

Typical usage
-------------

The visualization helper is used in scenarios where users want to inspect beam assignments, route changes, or mobility traces after execution. It complements the statistics and traffic modules rather than replacing them.
