"""Check what Network.fromFile produces for LagunaSeca.xodr."""
from pathlib import Path
from scenic.domains.driving.roads import Network

p = Path("assets/maps/dSPACE/LagunaSeca.xodr")
n = Network.fromFile(str(p))
roads = n.roads
conn = getattr(n, "connectingRoads", [])
intersections = getattr(n, "intersections", [])
print("roads count:", len(roads))
print("connectingRoads count:", len(conn))
print("intersections count:", len(intersections))
for i, r in enumerate(roads[:5]):
    print("  road", i, "id=", getattr(r, "id", None), "name=", (getattr(r, "name", "") or "")[:50])
for i, r in enumerate(conn[:5]):
    print("  conn", i, "id=", getattr(r, "id", None), "name=", (getattr(r, "name", "") or "")[:50])
if len(conn) > 5:
    print("  ... and", len(conn) - 5, "more connecting roads")
