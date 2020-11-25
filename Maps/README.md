# Maps

This directory contains maps for the Delivery robot. You can add to these specifying. Each sub-directory contains the ff. files

* `[map_name].[png|pgm]` : an image of the map

* `[map_name].world`: a world file containing key information about the robot and environment. Includes a `.inc` file for sensor and robot configurations

* `[map_name].yaml`: a yaml file containing the map image, resolution and origin as well as thresholds for occupied and free regions