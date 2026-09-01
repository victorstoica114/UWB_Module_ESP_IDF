"""List the largest solids in the supplied GPS antenna STEP."""

from pathlib import Path

import cadquery as cq


PROJECT_DIR = Path(__file__).resolve().parent
shape = cq.importers.importStep(
    str(
        PROJECT_DIR
        / "reference_models"
        / "antena_gps"
        / "YN-91A_refined_v04_assembly.step"
    )
).val()

rows = []
for index, solid in enumerate(shape.Solids(), start=1):
    bb = solid.BoundingBox()
    rows.append(
        (
            solid.Volume(),
            index,
            bb.xmin,
            bb.xmax,
            bb.ymin,
            bb.ymax,
            bb.zmin,
            bb.zmax,
        )
    )

for row in sorted(rows, reverse=True)[:30]:
    print(
        f"{row[1]:2d}: volume={row[0]:10.3f} "
        f"x={row[2]:7.3f}..{row[3]:7.3f} "
        f"y={row[4]:7.3f}..{row[5]:7.3f} "
        f"z={row[6]:7.3f}..{row[7]:7.3f}"
    )


# Four mounting holes in the flower plate. The STEP contains circular edge
# duplicates on several faces, so centres are rounded and de-duplicated.
mount_centres = set()
for edge in shape.Edges():
    if edge.geomType() != "CIRCLE":
        continue
    centre = edge.Center()
    try:
        radius = edge.radius()
    except Exception:
        continue
    if abs(radius - 1.3) > 1e-4 or abs(centre.z - 10.5) > 1e-4:
        continue
    mount_centres.add((round(centre.x, 4), round(centre.y, 4)))

print("\nGPS flower mounting holes measured from STEP:")
for x, y in sorted(mount_centres):
    print(f"  centre=({x:8.4f}, {y:8.4f})  diameter=2.6000 mm")
