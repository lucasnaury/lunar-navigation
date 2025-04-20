# Global Path planning

This README explains how to execute a modified version of the A* global path planning algorithm,
on lunar surface data from NASA.

It is done into multiple steps:
1. [Downloading moon data](#downloading-moon-data)
2. [Processing moon data](#processing-moon-data)
3. [Cropping the selected region](#cropping-the-selected-region)
4. [Drawing the units](#drawing-the-units)
5. [Running path planning](#running-path-planning)
6. [Final output: merged paths](#final-output-merged-paths)

> Note: The global path planning algorithm doesn't need ROS2 nor Gazebo to be run, just Python

## 1. Downloading moon data

This script uses moon data from the [MoonTrek API](https://trek.nasa.gov/tiles/apidoc/trekAPI.html?body=moon).
Specifically, we are using the following data:
- LRO LOLA DEM, S Pole, 87.5 deg, Grayscale ([Preview](https://trek.nasa.gov/tiles/Moon/SP/LRO_LOLA_Gray_SPole875_5mp_v04.html), [Metadata](https://trek.nasa.gov/moon/TrekWS/rest/cat/metadata/fgdc/html?label=LRO_LOLA_Gray_SPole875_5mp_v04))
- LRO LOLA DEM, S Pole, 87.5 deg, Hillshade ([Preview](https://trek.nasa.gov/tiles/Moon/SP/LRO_LOLA_Shade_SPole875_5mp_v04.html), [Metadata](https://trek.nasa.gov/moon/TrekWS/rest/cat/metadata/fgdc/html?label=LRO_LOLA_Shade_SPole875_5mp_v04))
- LRO LOLA Color Slope, S Pole, 87.5 deg ([Preview](https://trek.nasa.gov/tiles/Moon/SP/ColorSlope_LRO_LOLA_DEM_SPole875_5mp_v04.html), [Metadata](https://trek.nasa.gov/moon/TrekWS/rest/cat/metadata/fgdc/html?label=ColorSlope_LRO_LOLA_DEM_SPole875_5mp_v04))


To download the different maps, run the following command:
```bash
python download_maps.py
```

## 2. Processing moon data

To process the downloaded maps, run the following command:
```bash
python process_maps.py
```

> This will extract the grayscale costmap of the coloured slope map.

## 3. Cropping the selected region

The region that we selected for path planning is a 598x598px that's located at the (4730,3050) coordinates. The following script will generate the cropped maps:

```bash
python crop_maps.py 4730 3050 598
```

### Optional: Reduce cropped resolution

If the algorithm takes too much time to run, the image resolution can be reduced using the following script:
```bash
python resize_maps.py "cropped" "cropped_resized" 256
```

## 4. Drawing the units

Unit positions are defined by JSON files in the `units` folder. To draw the units on the map, run the following scripts:

```bash
python draw_units.py config1.json maps/cropped
python draw_units.py config2.json maps/cropped
python draw_units.py config3.json maps/cropped
```

## 5. Running path planning

To run the path planning algorithm, please run the following:
```bash
python path_planning.py "maps/cropped" "all_resized_conf1" "config1.json"
python path_planning.py "maps/cropped" "all_resized_conf2" "config2.json"
python path_planning.py "maps/cropped" "all_resized_conf3" "config3.json"
```
> Note:
> - The first argument corresponds to the map folder path
> - The second argument corresponds to the output folder path (relative to the `output` folder)
> - The last argument corresponds to the unit configuration filename in the `units` folder

## 6. Final output: merged paths

To get the merged paths from the previous results, run the following command:
```bash
python merge_paths.py config1.json cropped all_resized_conf1
python merge_paths.py config2.json cropped all_resized_conf2
python merge_paths.py config3.json cropped all_resized_conf3
```

> Note:
> - The first argument corresponds to the unit configuration filename in the `units` folder
> - The second argument corresponds to the map folder path (relative to the `maps` folder), on which to display the path
> - The last argument corresponds to the output folder path from the previous command
