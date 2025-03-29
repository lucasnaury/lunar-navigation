import os
import requests
from PIL import Image
from io import BytesIO
import math


def main(wmts_url, zoom_level, start_tile, end_tile):

    X_START, Y_START = start_tile
    X_END, Y_END     = end_tile

    nbTiles = (X_END - X_START + 1) * (Y_END - Y_START + 1)

    # Fetch tiles
    tile_size = 256  # Each tile is 256x256 pixels
    grid_width = (X_END - X_START + 1) * tile_size
    grid_height = (Y_END - Y_START + 1) * tile_size
    full_image = Image.new("RGB", (grid_width, grid_height))

    for x in range(X_START, X_END + 1):
        for y in range(Y_START, Y_END + 1):
            url = f"{wmts_url}/{zoom_level}/{x}/{y}.png"

            progress = (x - X_START) * (Y_END - Y_START + 1) + (y - Y_START + 1)

            print(f"{progress}/{nbTiles} | Fetching tile {x}, {y}")
            
            try:
                response = requests.get(url, timeout=10)
                response.raise_for_status()
                tile_image = Image.open(BytesIO(response.content))

                # Calculate position
                x_offset = (x - X_START) * tile_size
                y_offset = (y - Y_START) * tile_size

                # Paste tile into the full image
                full_image.paste(tile_image, (y_offset,x_offset))

            except requests.exceptions.RequestException as e:
                print(f"Failed to download {url}: {e}")

    return full_image



if __name__ == "__main__":
    
    WMTS_URL = "https://trek.nasa.gov/tiles/Moon/SP/LRO_LOLA_Gray_SPole875_5mp_v04/1.0.0//default/default028mm/"
    
    full_image = main(WMTS_URL, 9, (238,238), (273, 273))

    # Save final image
    path = "moon_high_res.png"
    full_image.save(path)
    print(f"Saved high-resolution image as {path}")