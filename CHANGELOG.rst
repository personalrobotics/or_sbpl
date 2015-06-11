^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package or_sbpl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2015-06-10)
------------------
* Merge pull request `#3 <https://github.com/personalrobotics/or_sbpl/issues/3>`_ from personalrobotics/feature/yaml0.5
  Added support for yaml-cpp 0.5 (for Ubuntu 14.04)
* Added support for yaml-cpp 0.5.
* Contributors: Michael Koval

1.0.0 (2015-02-17)
------------------
* linked or_sbpl library to openrave plugin
* Fixing bug in or_sbpl angle grid cell assignment
* Moving primitive generatino scripts to timpy and herbpy
* Fixing orientation cost compuation
* Cleaning up documentation. Adding doxygen config file. Modify action definition to contain weight to apply to action cost. Add ability to specify relative weights of linear and orietation change during cost computation.
* Updating cached aciton to only return success if full action can be executed.  Add initial script for generating primitives for tim.
* Adding herb primitives
* Adding script for generating primitives
* Adding ability to set velocities in trajectory
* Package for sbpl related plugins and planners
* Contributors: Aaron Walsman, Evan Shapiro, Jennifer King, Michael Koval, Prasanna Velagapudi
