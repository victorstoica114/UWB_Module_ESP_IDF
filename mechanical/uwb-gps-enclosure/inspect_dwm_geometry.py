"""List horizontal faces of the DWM module from the supplied board STEP."""

from pathlib import Path

import cadquery as cq
import generate_enclosure as gen


PROJECT_DIR = Path(__file__).resolve().parent
shape = cq.importers.importStep(
    str(PROJECT_DIR / "reference_models" / "modul_radio" / "UWB.step")
).val()

DWM_BOX = (70.299, 93.299, -69.438, -56.237, 1.595, 5.595)

rows = []
for face in shape.Faces():
    bb = face.BoundingBox()
    if not (
        bb.xmin >= DWM_BOX[0] - 0.01
        and bb.xmax <= DWM_BOX[1] + 0.01
        and bb.ymin >= DWM_BOX[2] - 0.01
        and bb.ymax <= DWM_BOX[3] + 0.01
        and bb.zmin >= DWM_BOX[4] - 0.01
        and bb.zmax <= DWM_BOX[5] + 0.01
    ):
        continue
    if face.geomType() != "PLANE" or bb.zlen > 0.001:
        continue
    center = face.Center()
    rows.append(
        (
            round(center.z, 4),
            round(face.Area(), 4),
            round(bb.xmin, 4),
            round(bb.xmax, 4),
            round(bb.ymin, 4),
            round(bb.ymax, 4),
        )
    )

for row in sorted(rows, reverse=True):
    print(
        f"z={row[0]:7.4f} area={row[1]:9.4f} "
        f"x={row[2]:8.4f}..{row[3]:8.4f} "
        f"y={row[4]:9.4f}..{row[5]:9.4f}"
    )


def print_intersection(name: str, first: cq.Shape, second: cq.Shape) -> None:
    overlap = first.intersect(second)
    print(f"\n{name}: volume={overlap.Volume():.5f}")
    for index, solid in enumerate(overlap.Solids(), start=1):
        bb = solid.BoundingBox()
        print(
            f"  {index}: volume={solid.Volume():.5f} "
            f"x={bb.xmin:.3f}..{bb.xmax:.3f} "
            f"y={bb.ymin:.3f}..{bb.ymax:.3f} "
            f"z={bb.zmin:.3f}..{bb.zmax:.3f}"
        )


refs = gen.build_reference_shapes()
chassis = gen.build_chassis().val()
print_intersection("PCB / chassis", refs["pcb"], chassis)

retainer = gen.build_battery_retainer().val().moved(
    cq.Location(
        (
            0.0,
            gen.CFG.battery_tray_center_y,
            gen.CFG.battery_retainer_z,
        )
    )
)
print_intersection("Chassis / battery retainer", chassis, retainer)
print_intersection(
    "Chassis / SMA mating guard",
    chassis,
    gen.build_sma_mating_placeholder(),
)
