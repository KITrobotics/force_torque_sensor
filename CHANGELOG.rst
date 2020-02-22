^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package force_torque_sensor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.0 (2020-02-22)
------------------
* All dependencies are released
* Chagnes:
  - Use consistanant naming: calibration -> calacualte_offsets
  - Rename 'Recalibrate' service to 'CalculateOffsetsWithoutGravity' -> it calculates offsets but removing gravity. This is usefull for manipulators.
  - Refractoring and using global variables toward real-time performance.
* - Code refraction
  - Added new output with data after all filters
  - Filters are now not activated if they are not defined
  - Added mutex between two threads of reading data from the sensor and providing them for other components.
  - Additinal debugging output when regarding setup of the SensorHandle
* Add changes from melodic in to kinetic (`#22 <https://github.com/KITrobotics/force_torque_sensor/issues/22>`_)
  * Added travis config for melodic
  * Added Melodic in overview
  * Scenario update melodic (`#7 <https://github.com/KITrobotics/force_torque_sensor/issues/7>`_)
  * added scenario parameter
  * fixed wrong variable names
  * Added service for setting offets from outside
  * Moved to Eigen3 from Eigen
  * Update calibrate_tool.py
  * Using WrenchTranform in tf2 instead of manual transform.
  * Corrected error with doTranform for wrenches and corrected package.xml with package meta data.
  * Update .travis.yml
  * Update .travis.rosinstall
  * Added joystick and keyboard (`#8 <https://github.com/KITrobotics/force_torque_sensor/issues/8>`_)
  * generated changelog
  * 0.8.1
  * Update .travis.rosinstall (`#9 <https://github.com/KITrobotics/force_torque_sensor/issues/9>`_)
  * Added support for RS-485 communication (`#11 <https://github.com/KITrobotics/force_torque_sensor/issues/11>`_)
  * Added support for RS-485 communication
  Adjusted the sensor handle to dynamically use CAN or RS-485 communication, depending on the plugin loaded.
  Also added a Configuration file for the RS485 interface.
  * Updated handles when testing RS485 communication
  * generated changelog
  * 0.0.1
  * Publishing before merge
  * Cleaned some lines (`#21 <https://github.com/KITrobotics/force_torque_sensor/issues/21>`_)
  * Cleaned lines (`#23 <https://github.com/KITrobotics/force_torque_sensor/issues/23>`_)
  * Cleaned some lines
  * Add correction of prototype
* Merge branch 'kinetic-devel' into melodic
* Cleaned lines (`#23 <https://github.com/KITrobotics/force_torque_sensor/issues/23>`_)
  * Cleaned some lines
  * Add correction of prototype
* Cleaned some lines (`#21 <https://github.com/KITrobotics/force_torque_sensor/issues/21>`_)
* Melodic to kinetic (`#18 <https://github.com/KITrobotics/force_torque_sensor/issues/18>`_)
  * Added travis config for melodic
  * Added Melodic in overview
  * Scenario update melodic (`#7 <https://github.com/KITrobotics/force_torque_sensor/issues/7>`_)
  * added scenario parameter
  * fixed wrong variable names
  * Added service for setting offets from outside
  * Moved to Eigen3 from Eigen
  * Update calibrate_tool.py
  * Using WrenchTranform in tf2 instead of manual transform.
  * Corrected error with doTranform for wrenches and corrected package.xml with package meta data.
  * Update .travis.yml
  * Update .travis.rosinstall
  * Added joystick and keyboard (`#8 <https://github.com/KITrobotics/force_torque_sensor/issues/8>`_)
  * generated changelog
  * 0.8.1
  * Update .travis.rosinstall (`#9 <https://github.com/KITrobotics/force_torque_sensor/issues/9>`_)
  * Added support for RS-485 communication (`#11 <https://github.com/KITrobotics/force_torque_sensor/issues/11>`_)
  * Added support for RS-485 communication
  Adjusted the sensor handle to dynamically use CAN or RS-485 communication, depending on the plugin loaded.
  Also added a Configuration file for the RS485 interface.
  * Updated handles when testing RS485 communication
  * generated changelog
  * 0.0.1
  * Publishing before merge
  * Update .travis.rosinstall
  * Update .travis.yml
* Added support for RS-485 communication (`#11 <https://github.com/KITrobotics/force_torque_sensor/issues/11>`_)
  * Added support for RS-485 communication
  Adjusted the sensor handle to dynamically use CAN or RS-485 communication, depending on the plugin loaded.
  Also added a Configuration file for the RS485 interface.
  * Updated handles when testing RS485 communication
  * generated changelog
  * 0.0.1
  * Publishing before merge
* Ati rs485 interface kinetic (`#16 <https://github.com/KITrobotics/force_torque_sensor/issues/16>`_)
  * Added travis config for melodic
  * Added Melodic in overview
  * Added support for RS-485 communication
  Adjusted the sensor handle to dynamically use CAN or RS-485 communication, depending on the plugin loaded.
  Also added a Configuration file for the RS485 interface.
  * Updated handles when testing RS485 communication
  * Publishing before merge
  * Kinetic ati-rs485
* Update .travis.rosinstall (`#9 <https://github.com/KITrobotics/force_torque_sensor/issues/9>`_)
* Merge pull request `#12 <https://github.com/KITrobotics/force_torque_sensor/issues/12>`_ from KITrobotics/kinetic-devel-release
  Kinetic devel release
* 0.0.1
* generated changelog
* Contributors: Daniel Azanov, Denis Stogl, Denis Štogl, FlorianAumann, GDwag, Gilbert Groten, SR3 RoboTrainer

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
* Contributors: Denis Štogl, IIROB Praktikum 3, Timo Leitritz
