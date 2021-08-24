# RMF Tools

The RMF Tools package serves as a utility package for common helper tools used in the RMF interfaces. In this repo, these tools are shared between the [dashboard](https://github.com/open-rmf/rmf-web/tree/main/packages/dashboard) app package and the [minimal](https://github.com/open-rmf/rmf-web/tree/main/packages/minimal) app package.

Scripts:
Setup - retrieves and configures the required assets and resources for the app

Utility Functions:
Calculation - conversion functions for plotting of trajectories
CSS - correctly binds viewbox for use of leaflet

RMF Launcher:
Helper function that launches and kills all required RMF processes. This assumes required rmf components are installed and launches them locally.
