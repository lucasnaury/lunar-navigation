import json

class Unit:
    def __init__(self, pos, name, code, id, isFixed=False):
        # Unit variables
        self.x = pos[0]
        self.y = pos[1]
        self.name = name
        self.code = code
        self.id = id
        self.isFixed = isFixed

        # Init with no routes
        self.routes:list[Unit] = []

    def addRoute(self, route):
        """Add route to another unit"""
        self.routes.append(route)


    def pos(self):
        """Return (x,y) pos"""
        return (self.x, self.y)
    
    def YXpos(self):
        """Return (y,x) pos"""
        return (self.y, self.x)

    def loadUnits(path, targetSize=None) -> tuple[list["Unit"], float, float]:
        with open(path) as f:
            # Read JSON as dict
            data = json.load(f)

            # Separate data
            metadata = data["metadata"]
            units_data = data["units"]
            routes_data = data["routes"]

            # Calculate scaling factor
            if targetSize == None:
                scale = 1
            else:
                scale = targetSize / int(metadata["mapSize"])

            px2m = float(metadata["px2m"]) / scale

            # Create units
            units:dict[str, Unit] = {}
            for i,val in enumerate(units_data):
                # Extract position for each unit
                name = val['name']
                x = int(val['x'])
                y = int(val['y'])
                code = val['code']
                isFixed = val["isFixed"] if ("isFixed" in val) else False
                
                # Store it in dict
                pos = (int(scale*x), int(scale*y))
                units[code] = Unit(pos, name, code, i, isFixed)

            # Define routes
            for r in routes_data:
                # Extract unit codes
                unit_from = units[r['from']]
                unit_to   = units[r['to']]

                # Add route
                unit_from.addRoute(unit_to)


            # Convert units to list
            units = list(units.values())


            return units, px2m, scale
    
        print("[ERROR] Couldn't open JSON file")
        return [], 0, 0