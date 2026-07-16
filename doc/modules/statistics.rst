Statistics Module
=================

Overview
--------

The statistics module collects measurements from simulations and prepares them for analysis. It records link quality, handover events, throughput, delay, and other performance indicators.

Key responsibilities
-------------------

- collect simulation metrics,
- export per-event or aggregate statistics,
- support analysis for routing, beam selection, and handover behavior.

Primary implementation
----------------------

- ``model/leosim-stats.h``
- ``model/leosim-stats.cc``

Functional definition
---------------------

LeoSimStatisticsHelper collects and organizes simulation measurements such as packet statistics, beam events, and handover traces. It is the main interface for turning raw event data into reports and plots.

Typical usage
-------------

The statistics helper is used after or during a run to inspect throughput, latency, or beam-change behavior. It is commonly attached to the simulation setup alongside the channel and beam manager helpers.
