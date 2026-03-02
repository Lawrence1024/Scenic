"""Debug why junction links are not added."""
from pathlib import Path
from scenic.domains.driving.roads import Network
from scenic.domains.racing.tracks import RacingTrack, _road_endpoint_and_heading

p = Path("assets/maps/dSPACE/LagunaSeca.xodr")
n = Network.fromFile(str(p))
track = RacingTrack(
    n, direction="counterclockwise", pitLaneRoadName="pit",
    main_loop_connecting_road_ids=None, pit_connecting_road_ids=None,
)
# After _identifyRoadSegments ran in __init__
main = track._mainRacingRoads
pit_roads = track._pitRoads
conn = list(getattr(n, "connectingRoads", ()))
intersections = list(getattr(n, "intersections", ()))
pit = track.pitLaneRoad
main_set = set(main)

print("Intersections:", len(intersections))
print("Connecting roads:", len(conn))
print("Main roads:", [getattr(r, "id", None) for r in main])
print("Pit road id:", getattr(pit, "id", None) if pit else None)

for ji, junc in enumerate(intersections):
    jroads = getattr(junc, "roads", ())
    print(f"\nJunction {ji}: {len(jroads)} roads, ids={[getattr(r, 'id', None) for r in jroads]}")
    main_here = [r for r in jroads if r in main_set]
    print(f"  main_roads_here ({len(main_here)}):", [getattr(r, "id", None) for r in main_here])
    conn_here = [c for c in conn if getattr(c, "_predecessor", None) == junc or getattr(c, "_successor", None) == junc]
    print(f"  conn_roads_here ({len(conn_here)}):", [getattr(c, "id", None) for c in conn_here[:5]], "..." if len(conn_here) > 5 else "")
# What are conn predecessor/successor?
if conn:
    c0 = conn[0]
    pred = getattr(c0, "_predecessor", None)
    succ = getattr(c0, "_successor", None)
    print("\nFirst conn _predecessor:", type(pred), id(pred) if pred else None, getattr(pred, "uid", None) if pred else None)
    print("First conn _successor:", type(succ), id(succ) if succ else None, getattr(succ, "uid", None) if succ else None)
    for ji, junc in enumerate(intersections):
        print(f"  Junction {ji} id:", id(junc), getattr(junc, "uid", None))

# Check one conn's geometry
if conn:
    c = conn[0]
    cl = getattr(c.lanes[0], "centerline", None) if c.lanes else None
    print(f"\nFirst conn id={getattr(c, 'id', None)}, lanes={len(c.lanes)}, centerline len={len(cl) if cl else 0}")
    if cl and len(cl) >= 2:
        print("  start:", cl[0].x, cl[0].y, " end:", cl[-1].x, cl[-1].y)
    pit_start = _road_endpoint_and_heading(pit, at_start=True) if pit else None
    pit_end = _road_endpoint_and_heading(pit, at_start=False) if pit else None
    print("  Pit start:", pit_start, " end:", pit_end)
