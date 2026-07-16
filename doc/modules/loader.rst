Loader Module
=============

Overview
--------

The loader module ingests scenario data and converts it into simulation-ready objects for LeoSim. It is responsible for loading satellite traces, ground device locations, operator metadata, and related configuration data used by the mobility and beam layers.

Key responsibilities
-------------------

- load satellite positions from CSV or trace-based formats,
- load ground stations and user equipment from text or CSV files,
- assign operator identities to satellites and ground nodes,
- expose lookup helpers for device metadata and operator-aware routing,
- provide a common bridge between external scenario files and the ns-3 runtime.

Primary implementation
----------------------

- ``model/leosim-loader.h``
- ``model/leosim-loader.cc``

Typical usage
-------------

The loader is normally used during scenario initialization to populate the node database before mobility, channel, and beam models are configured.

Notes
-----

The loader is a foundational component. Most other modules assume that the required node and operator metadata already exists before they are used.
