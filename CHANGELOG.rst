^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package or_sbpl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Adding unit tests
* Adding logic to put the current pose of the robot as the first point in the returned trajectory
* Added a missing dependency to package.xml
* Linking against yaml in plugin
* Updating yaml test so that it builds. Adding yaml test to catkin build
* updating yaml test to be 14.04 compliant
* Contributors: Jennifer King, Michael Koval

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
