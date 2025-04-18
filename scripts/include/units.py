import json

class Unit:
    def __init__(self, pos, name, id, isFixed=False):
        self.x = pos[0]
        self.y = pos[1]
        self.name = name
        self.id = id
        self.isFixed = isFixed

    def pos(self):
        return (self.x, self.y)
    
    def YXpos(self):
        return (self.y, self.x)

    def loadUnits(path, targetSize=None) -> tuple[list["Unit"], float, float]:
        with open(path) as f:
            # Read JSON as dict
            data = json.load(f)

            # Separate data
            metadata = data["metadata"]
            units_data = data["units"]

            # Calculate scaling factor
            if targetSize == None:
                scale = 1
            else:
                scale = targetSize / int(metadata["mapSize"])

            px2m = float(metadata["px2m"]) / scale

            # Clean data
            units = []
            for i,val in enumerate(units_data):
                # Extract position for each unit
                name = val['name']
                x = int(val['x'])
                y = int(val['y'])
                isFixed = (str.lower(val["fixed"]) == "true") if ("fixed" in val) else False
                
                # Store it in dict
                pos = (int(scale*x), int(scale*y))
                units.append(Unit(pos, name, i, isFixed))

            return units, px2m, scale
    
        print("[ERROR] Couldn't open JSON file")
        return [], 0, 0