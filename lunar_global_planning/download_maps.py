from include.wmts import main as getMap
from pathlib import Path

def main():

    # Get folder path
    outputFolder = Path(__file__).parent.absolute() / "maps" / "downloaded"
    outputFolder.mkdir(parents=True, exist_ok=True)

    maps = {
        # Heightmap
        "LRO_LOLA_Gray_SPole875_5mp_v04":{
            "zoom": 9,
            "start": (238,238),
            "end": (273,273),
            "outputPath": str(outputFolder / "heightmap.png")
        },

        # Slope map
        "ColorSlope_LRO_LOLA_DEM_SPole875_5mp_v04":{
            "zoom": 9,
            "start": (238,238),
            "end": (273,273),
            "outputPath": str(outputFolder / "slopemap_color.png")
        },

        # Hillshade map
        "LRO_LOLA_Shade_SPole875_5mp_v04":{
            "zoom": 9,
            "start": (238,238),
            "end": (273,273),
            "outputPath": str(outputFolder / "illumination.png")
        },

    }

    for key, data in maps.items():
        print("-------------------------------------------------------------------------")
        print(f"Downloading {key}...")
        print("-------------------------------------------------------------------------")

        # Create MoonTrek WMTS URL
        url = f"https://trek.nasa.gov/tiles/Moon/SP/{key}/1.0.0//default/default028mm/"

        # Get image
        image = getMap(url, data["zoom"], data["start"], data["end"])

        # Crop image
        w, h = image.size
        margin = 74
        image = image.crop((margin,margin, w-margin, h-margin))

        # Save image
        image.save(data["outputPath"])


if __name__ == "__main__":
    main()