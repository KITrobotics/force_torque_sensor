^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package force_torque_sensor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2020-02-25)
------------------
* Update README
* Added static_application parameter to example configs.
* Added functionality for static applications, i.e., the node looks only at the beginning for the transformation and u$
* Reduced INFO output (changed to DEBUG)
* Resorting of definitions in .h file
* Removing used and unnecessary variables
* Using private namespace for filters to be unified with iirob_filters implementation
* Contributors: Denis Stogl

0.9.3 (2020-02-22)
------------------
* Changelog
* Contributors: Denis Štogl

0.9.2 (2020-02-22)
------------------
* Corrected changelogs and release data

0.9.0 (2020-02-22)
------------------
* All dependencies are released
* Changes:

  * UR calibration
  * Use consistanant naming: calibration to calacualte\_offsets
  * Rename 'Recalibrate' service to 'CalculateOffsetsWithoutGravity': it calculates offsets but removing gravity. This is usefull for manipulators.
  * Refractoring and using global variables toward real-time performance.
  
* Code refraction

  * Added new output with data after all filters
  * Filters are now not activated if they are not defined
  * Added mutex between two threads of reading data from the sensor and providing them for other components.
  * Additinal debugging output when regarding setup of the SensorHandle
  
* Add changes from melodic in to kinetic (`#22 <https://github.com/KITrobotics/force_torque_sensor/issues/22>`_)

  * Added travis config for melodic
  * Added Melodic in overview
  * Scenario update melodic (`#7 <https://github.com/KITrobotics/force_torque_sensor/issues/7>`_)
  * added scenario parameter
  * fixed wrong variable names
  * Added service for setting offets from outside
  * Moved to Eigen3 from Eigen
  * Using WrenchTranform in tf2 instead of manual transform.
  * Corrected error with doTranform for wrenches and corrected package.xml with package meta data.
  * Update .travis.yml
  * Update .travis.rosinstall
  * Added joystick and keyboard (`#8 <https://github.com/KITrobotics/force_torque_sensor/issues/8>`_)
  * generated changelog 
* Contributors: Daniel Azanov, Denis Štogl, Florian Aumann, Gilbert Groten (GDwag)
    

0.8.1 (2018-12-11)
------------------
* Added joystick and keyboard (#8)
* Scenario update melodic (#7)

  * added scenario parameter
  * fixed wrong variable names
  * Added service for setting offets from outside
  * Moved to Eigen3 from Eigen
  * Update calibrate_tool.py
  * Using WrenchTranform in tf2 instead of manual transform.
  * Corrected error with doTranform for wrenches and corrected package.xml with package meta data.
  * Update .travis.yml
  * Update .travis.rosinstall

* Added Melodic in overview
* Added travis config for melodic


0.0.1 (2018-12-12)
------------------
* Update CMakeLists.txt
* Updated INSTALL paths
* Update CMakeLists.txt
* Update CMakeLists.txt
* Update .travis.rosinstall (#3)
  * Update .travis.rosinstall for compiling
* Update CMakeLists.txt
* Update package.xml
* Update .travis.rosinstall
* Merge pull request #2 from KITrobotics/master
  Update README.md
* Update README.md
* Merge pull request #1 from KITrobotics/bugs_clean
  Removed bug setting false static offsets paramters; Commenting out an…
* Create .travis.rosinstall
* Create .travis.yml
* Create README.md
* Removed bug setting false static offsets paramters; Commenting out and deleting some unused code.
* Corrected param names for CoG
* Added corrections to work with schunk_ftc
* Moved class loader to handle
* Added namespaces
* First working version
* Contributors: Denis Štogl, Timo Leitritz
