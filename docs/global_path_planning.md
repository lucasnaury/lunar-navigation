# Global Path planning

This README explains how to execute the path planning algorithm.

## Downloading moon data

This script uses moon data from the [MoonTrek API](https://trek.nasa.gov/tiles/apidoc/trekAPI.html?body=moon).
Specifically, we are using the following data:
- LRO LOLA DEM, S Pole, 87.5 deg, Grayscale ([Preview](https://trek.nasa.gov/tiles/Moon/SP/LRO_LOLA_Gray_SPole875_5mp_v04.html), [Metadata](https://trek.nasa.gov/moon/TrekWS/rest/cat/metadata/fgdc/html?label=LRO_LOLA_Gray_SPole875_5mp_v04))
- LRO LOLA DEM, S Pole, 87.5 deg, Hillshade ([Preview](https://trek.nasa.gov/tiles/Moon/SP/LRO_LOLA_Shade_SPole875_5mp_v04.html), [Metadata](https://trek.nasa.gov/moon/TrekWS/rest/cat/metadata/fgdc/html?label=LRO_LOLA_Shade_SPole875_5mp_v04))
- LRO LOLA Color Slope, S Pole, 87.5 deg ([Preview](https://trek.nasa.gov/tiles/Moon/SP/ColorSlope_LRO_LOLA_DEM_SPole875_5mp_v04.html), [Metadata](https://trek.nasa.gov/moon/TrekWS/rest/cat/metadata/fgdc/html?label=ColorSlope_LRO_LOLA_DEM_SPole875_5mp_v04))


To download the different maps, run the following command:
```bash
python scripts/download_maps.py
```

## Processing moon data

To process the downloaded maps, run the following command:
```bash
python scripts/process_maps.py
```

> This will extract the grayscale costmap of the coloured slope map.