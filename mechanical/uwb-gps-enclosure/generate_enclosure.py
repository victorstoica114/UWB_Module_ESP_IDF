"""Parametric generator for the modular outdoor UWB/GPS enclosure.

The script uses the real UWB board and GPS antenna STEP models as references,
builds print-oriented parts, exports STEP/STL files and creates a verification
assembly.  Dimensions are millimetres.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, Iterable, Tuple

import cadquery as cq


PROJECT_DIR = Path(__file__).resolve().parent
REFERENCE_DIR = PROJECT_DIR / "reference_models"
EXPORT_DIR = PROJECT_DIR / "exports"
PREVIEW_DIR = PROJECT_DIR / "previews"


@dataclass(frozen=True)
class P:
    # Supplied components
    magnet_d: float = 10.0
    magnet_h: float = 2.0
    magnet_pocket_d: float = 10.25
    magnet_pocket_h: float = 2.20
    magnet_radius: float = 44.0
    magnet_angle_offset_deg: float = 45.0

    battery_x: float = 48.5
    battery_y: float = 25.0
    battery_z: float = 73.0
    battery_cavity_x: float = 49.5
    battery_cavity_y: float = 28.0
    battery_cavity_z: float = 74.0
    battery_tray_wall: float = 2.0
    battery_front_wall_t: float = 5.0

    switch_body_x: float = 13.0
    switch_body_y: float = 13.7
    switch_body_z: float = 12.4
    switch_bushing_d: float = 5.7
    switch_hole_d: float = 6.1

    usb_face_x: float = 11.7
    usb_face_z: float = 6.4
    usb_cut_x: float = 12.25
    usb_cut_z: float = 6.95
    usb_body_len: float = 25.2
    usb_strain_len: float = 10.6

    tape_h: float = 1.0

    # Centre of the 8.5 x 6 mm ceramic antenna block in the DWM module.
    # The supplied STEP names this pin/size-compatible package "DWM1000"; the
    # fitted radio is DWM3000.  Original STEP axes: X/Y are in the PCB plane
    # and Z is normal to the PCB.  In the enclosure the transform is:
    # global X = model Y - antenna Y
    # global Y = antenna Z - model Z
    # global Z = board vertical offset - model X
    uwb_antenna_model_x: float = 75.1488
    uwb_antenna_model_y: float = -63.2375
    uwb_antenna_model_z: float = 3.145

    board_model_xmin: float = 70.2988
    board_model_xmax: float = 159.695
    board_bottom_clearance: float = 40.0
    board_to_gps_flower: float = 50.0
    gps_model_zmin: float = -0.8
    gps_model_zmax: float = 17.22
    gps_flower_model_zmin: float = 10.5
    gps_flower_model_zmax: float = 12.1
    gps_mount_radius: float = 42.5
    gps_mount_hole_model_d: float = 2.6
    gps_mount_clear_d: float = 2.8
    gps_mount_standoff_d: float = 7.0
    gps_mount_top_gap: float = 0.10
    m25_insert_body_d: float = 3.0
    m25_insert_knurl_d: float = 3.5
    m25_insert_length: float = 5.0
    m25_insert_d: float = 3.2
    m25_insert_depth: float = 5.3
    m25_insert_lead_d: float = 3.6
    m25_insert_lead_depth: float = 0.6
    gps_tray_outer_d: float = 115.2
    gps_tray_ring_inner_d: float = 100.0
    gps_tray_boss_passage_clearance: float = 0.8
    gps_tray_notch_corner_r: float = 1.2
    gps_tray_reinforcement_lobe_r: float = 14.0
    # Radial positions measured from Alex's reinforced STEP, ordered to match
    # the cap-boss passages at 90°, 210° and 330°.
    gps_tray_reinforcement_center_r_90: float = 57.356592134151
    gps_tray_reinforcement_center_r_210: float = 58.801408311021
    gps_tray_reinforcement_center_r_330: float = 57.589095737510

    # Main enclosure
    body_outer_d: float = 122.0
    body_wall: float = 2.8
    body_h: float = 124.0
    body_floor_h: float = 4.0

    # Recessed side service bay integrated into the lower body. The arched
    # opening is support-friendly and the controls remain inside the cylinder.
    service_bay_width: float = 46.0
    service_bay_height: float = 64.0
    service_bay_outer_y: float = -64.0
    service_bay_panel_y: float = -52.0
    service_bay_panel_t: float = 3.0
    service_bay_open_width: float = 40.0
    service_bay_open_bottom_z: float = 8.0
    service_bay_arch_center_z: float = 40.0
    service_bay_arch_r: float = 20.0

    collar_start_z: float = 112.0
    collar_h: float = 79.5
    collar_sleeve_outer_d: float = 128.0
    collar_sleeve_inner_d: float = 122.6
    collar_neck_outer_d: float = 122.0
    collar_shoulder_bridge_w: float = 0.6
    collar_tray_ledge_inner_d: float = 109.0

    hat_outer_d: float = 132.0
    hat_skirt_outer_d: float = 130.0
    hat_skirt_inner_d: float = 122.7
    hat_skirt_h: float = 18.0
    hat_roof_h: float = 3.0
    hat_bottom_z: float = 173.5

    base_d: float = 122.0
    base_h: float = 5.0
    target_window_d: float = 38.0
    target_crossbar_w: float = 1.0
    tripod_base_h: float = 8.0
    tripod_base_outer_chamfer: float = 1.0
    tripod_insert_outer_d: float = 9.525
    tripod_insert_total_h: float = 7.0
    tripod_insert_flange_d: float = 9.5
    tripod_insert_flange_h: float = 1.2
    tripod_insert_flange_pocket_d: float = 9.8
    tripod_insert_flange_pocket_h: float = 1.35
    tripod_thread_tpi: int = 16
    tripod_thread_pitch: float = 1.5875
    tripod_thread_angle_deg: float = 60.0
    tripod_thread_nominal_major_d: float = 9.525
    tripod_thread_major_d: float = 9.78
    tripod_thread_minor_d: float = 8.08
    tripod_thread_start_z: float = 1.35
    tripod_thread_end_z: float = 7.15
    tripod_thread_profile_overlap: float = 0.05
    body_magnet_skin: float = 0.6

    # Interior carrier
    pcb_substrate_xmin: float = 80.0
    pcb_substrate_xmax: float = 151.0
    pcb_substrate_ymin: float = -116.0
    pcb_substrate_ymax: float = -52.0
    carrier_plate_h: float = 74.0
    carrier_plate_t: float = 2.6
    battery_cx: float = -10.0
    chassis_screw_x_left: float = -44.0
    chassis_screw_x_right: float = 25.0
    chassis_screw_y: float = 10.0
    chassis_boss_h: float = 6.8
    chassis_mount_flange_w: float = 20.0
    chassis_mount_flange_d: float = 14.0
    chassis_mount_flange_h: float = 4.0
    chassis_mount_flange_inward_shift: float = 1.0
    chassis_mount_flange_forward_shift: float = 2.75
    chassis_mount_gusset_d: float = 10.0
    chassis_mount_gusset_h: float = 18.0
    chassis_mount_gusset_flange_overlap: float = 0.6
    chassis_mount_gusset_head_clearance: float = 0.75
    chassis_side_outer_reach: float = 4.25
    chassis_side_front_trim_y: float = 1.8
    chassis_side_rear_floor_y0: float = 14.95
    chassis_side_rear_flange_y0: float = 14.2
    chassis_side_rear_gusset_y0: float = 15.4
    chassis_side_rear_y1: float = 24.0
    chassis_side_rear_gusset_top_z: float = 28.65
    chassis_side_wall_overlap: float = 0.8
    chassis_rear_screw_x: float = -10.0
    chassis_rear_screw_y: float = 50.0
    chassis_rear_flange_w: float = 16.0
    chassis_rear_flange_back_y: float = 54.25
    chassis_rear_bridge_w: float = 10.0
    chassis_rear_bridge_back_y: float = 45.0
    chassis_rear_rib_w: float = 3.0
    chassis_rear_rib_top_z: float = 28.65
    sma_relief_x: float = -29.47
    sma_relief_w: float = 18.0
    sma_relief_z0: float = 8.0
    sma_relief_h: float = 50.0
    sma_mating_clearance_d: float = 15.0
    sma_mating_y: float = 1.05
    sma_mating_z0: float = 10.0
    sma_mating_h: float = 25.0

    # Controls are stacked vertically on the recessed panel.
    usb_x: float = 0.0
    switch_x: float = 0.0
    usb_z: float = 23.0
    switch_z: float = 47.0

    # M3 hardware. Alex's inserts measure Ø3.0 mm on the smooth body and
    # Ø4.0 mm over the knurl. The Ø3.7 pocket gives 0.3 mm diametral
    # interference, while the shallow Ø4.1 lead centres the hot insert.
    radial_fit: float = 0.30
    screw_clear_d: float = 3.4
    m3_insert_body_d: float = 3.0
    m3_insert_knurl_d: float = 4.0
    m3_insert_length: float = 5.0
    m3_insert_d: float = 3.7
    m3_insert_depth: float = 5.8
    m3_insert_lead_d: float = 4.1
    m3_insert_lead_depth: float = 0.8
    m3_boss_d: float = 9.0
    m3_head_d: float = 6.5
    m3_head_depth: float = 3.2

    @property
    def body_inner_d(self) -> float:
        return self.body_outer_d - 2.0 * self.body_wall

    @property
    def board_bottom_z(self) -> float:
        return self.body_floor_h + self.board_bottom_clearance

    @property
    def board_vertical_offset(self) -> float:
        # The 40 mm requirement is measured from the lower PCB substrate edge;
        # the SMA connector is allowed to project below it into that cable bay.
        return self.board_bottom_z + self.pcb_substrate_xmax

    @property
    def board_top_z(self) -> float:
        # Highest point of the complete radio assembly, including DWM package.
        return self.board_vertical_offset - self.board_model_xmin

    @property
    def uwb_antenna_height_z(self) -> float:
        return self.board_vertical_offset - self.uwb_antenna_model_x

    @property
    def board_assembly_bottom_z(self) -> float:
        return self.board_vertical_offset - self.board_model_xmax

    @property
    def pcb_back_y(self) -> float:
        # Back surface of the PCB substrate is model Z=0.
        return self.uwb_antenna_model_z

    @property
    def gps_origin_z(self) -> float:
        # The complete GPS assembly is flipped 180° around X. Therefore the
        # original upper face of the flower plate becomes its lower face.
        return self.gps_flower_bottom_z + self.gps_flower_model_zmax

    @property
    def gps_flower_bottom_z(self) -> float:
        return self.board_top_z + self.board_to_gps_flower

    @property
    def gps_tray_top_z(self) -> float:
        # After flipping, original Zmax is the lowest point of the GPS model.
        # A 0.05 mm gap avoids a nominal CAD overlap with the support tray.
        return self.gps_origin_z - self.gps_model_zmax - 0.05

    @property
    def gps_top_z(self) -> float:
        return self.gps_origin_z - self.gps_model_zmin

    @property
    def battery_tray_outer_front_y(self) -> float:
        return self.pcb_back_y + self.tape_h + self.carrier_plate_t

    @property
    def battery_cavity_front_y(self) -> float:
        return self.battery_tray_outer_front_y + self.battery_front_wall_t

    @property
    def battery_tray_outer_back_y(self) -> float:
        return (
            self.battery_cavity_front_y
            + self.battery_cavity_y
            + self.battery_tray_wall
        )

    @property
    def battery_tray_center_y(self) -> float:
        return (
            self.battery_tray_outer_front_y + self.battery_tray_outer_back_y
        ) / 2.0

    @property
    def battery_cavity_z0(self) -> float:
        return self.body_floor_h + self.battery_tray_wall

    @property
    def battery_front_y(self) -> float:
        # Put the full 3 mm cavity-depth clearance in front of the battery to
        # maximize space for the downward SMA mating connector.
        return (
            self.battery_cavity_front_y
            + self.battery_cavity_y
            - self.battery_y
        )

    @property
    def battery_tray_top_z(self) -> float:
        return (
            self.battery_cavity_z0
            + self.battery_cavity_z
            + self.battery_tray_wall
        )

    @property
    def battery_retainer_z(self) -> float:
        return self.battery_tray_top_z - 3.0


CFG = P()


def wp_from_shape(shape: cq.Shape) -> cq.Workplane:
    return cq.Workplane("XY").newObject([shape])


def cylinder_z(radius: float, height: float, z0: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(height)
        .translate((0.0, 0.0, z0))
    )


def box_xyz(
    sx: float,
    sy: float,
    sz: float,
    cx: float = 0.0,
    cy: float = 0.0,
    z0: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(sx, sy, sz, centered=(True, True, False))
        .translate((cx, cy, z0))
    )


def cylinder_between(
    radius: float,
    start: Tuple[float, float, float],
    direction: Tuple[float, float, float],
    length: float,
) -> cq.Workplane:
    solid = cq.Solid.makeCylinder(
        radius,
        length,
        cq.Vector(*start),
        cq.Vector(*direction),
    )
    return wp_from_shape(solid)


def cylinder_y(
    radius: float,
    length: float,
    cx: float,
    cy: float,
    cz: float,
) -> cq.Workplane:
    return cylinder_between(
        radius,
        (cx, cy - length / 2.0, cz),
        (0.0, 1.0, 0.0),
        length,
    )


def capsule_y(
    width_x: float,
    height_z: float,
    length_y: float,
    cx: float,
    cy: float,
    cz: float,
) -> cq.Workplane:
    """Capsule/rounded rectangle extruded along global Y."""
    radius = height_z / 2.0
    straight = max(width_x - 2.0 * radius, 0.01)
    result = box_xyz(straight, length_y, height_z, cx, cy, cz - radius)
    dx = width_x / 2.0 - radius
    result = result.union(cylinder_y(radius, length_y, cx - dx, cy, cz))
    result = result.union(cylinder_y(radius, length_y, cx + dx, cy, cz))
    return result


def capsule_z(
    width_x: float,
    height_y: float,
    length_z: float,
    cx: float,
    cy: float,
    z0: float,
) -> cq.Workplane:
    """Capsule/rounded rectangle extruded along global Z."""
    radius = height_y / 2.0
    straight = max(width_x - 2.0 * radius, 0.01)
    result = box_xyz(straight, height_y, length_z, cx, cy, z0)
    dx = width_x / 2.0 - radius
    result = result.union(
        cylinder_z(radius, length_z, z0).translate((cx - dx, cy, 0.0))
    )
    result = result.union(
        cylinder_z(radius, length_z, z0).translate((cx + dx, cy, 0.0))
    )
    return result


def radial_hole(
    angle_deg: float,
    z: float,
    radius: float,
    start_r: float,
    length: float,
) -> cq.Workplane:
    angle = math.radians(angle_deg)
    ux, uy = math.cos(angle), math.sin(angle)
    return cylinder_between(
        radius,
        (start_r * ux, start_r * uy, z),
        (-ux, -uy, 0.0),
        length,
    )


def ring(outer_r: float, inner_r: float, height: float, z0: float = 0.0):
    return cylinder_z(outer_r, height, z0).cut(cylinder_z(inner_r, height + 0.4, z0 - 0.2))


def cone_z(
    radius_bottom: float,
    radius_top: float,
    height: float,
    z0: float = 0.0,
) -> cq.Workplane:
    return wp_from_shape(
        cq.Solid.makeCone(
            radius_bottom,
            radius_top,
            height,
            cq.Vector(0.0, 0.0, z0),
            cq.Vector(0.0, 0.0, 1.0),
        )
    )


def inward_45deg_support_ring(
    outer_r: float,
    inner_r: float,
    z_top: float,
) -> cq.Workplane:
    height = outer_r - inner_r
    z0 = z_top - height
    return cylinder_z(outer_r, height, z0).cut(
        cone_z(outer_r, inner_r, height, z0)
    )


def magnet_centres(p: P = CFG) -> Iterable[Tuple[float, float]]:
    for index in range(4):
        angle = math.radians(p.magnet_angle_offset_deg + index * 90.0)
        yield (
            p.magnet_radius * math.cos(angle),
            p.magnet_radius * math.sin(angle),
        )


def chassis_mount_centres(p: P = CFG) -> Tuple[Tuple[float, float], ...]:
    return (
        (p.chassis_screw_x_left, p.chassis_screw_y),
        (p.chassis_screw_x_right, p.chassis_screw_y),
        (p.chassis_rear_screw_x, p.chassis_rear_screw_y),
    )


def cap_screw_angles() -> Tuple[float, float, float]:
    return (90.0, 210.0, 330.0)


def gps_tray_reinforcement_center_radii(
    p: P = CFG,
) -> Tuple[float, float, float]:
    return (
        p.gps_tray_reinforcement_center_r_90,
        p.gps_tray_reinforcement_center_r_210,
        p.gps_tray_reinforcement_center_r_330,
    )


def build_target_base(p: P = CFG) -> cq.Workplane:
    # One uniform full-diameter disk: no nested Ø60 pilot and no upper step.
    base = cylinder_z(p.base_d / 2.0, p.base_h)
    try:
        base = base.edges(">Z").chamfer(0.6)
    except Exception:
        pass

    # Four top-open magnet pockets, moved outwards for a larger stabilising
    # moment and rotated between the body's internal screw bosses.
    for x, y in magnet_centres(p):
        pocket = (
            cq.Workplane("XY")
            .center(x, y)
            .circle(p.magnet_pocket_d / 2.0)
            .extrude(p.magnet_pocket_h)
            .translate((0.0, 0.0, p.base_h - p.magnet_pocket_h))
        )
        base = base.cut(pocket)

    # Through-cut calibration target inspired by Victor's reference STEP.
    # Cutting the circular window around two retained orthogonal ribs produces
    # four open quadrants through which the floor is directly visible.
    target_window = cylinder_z(
        p.target_window_d / 2.0,
        p.base_h + 0.4,
        -0.2,
    )
    cross_x = box_xyz(
        p.target_window_d + 2.0,
        p.target_crossbar_w,
        p.base_h + 0.8,
        z0=-0.4,
    )
    cross_y = box_xyz(
        p.target_crossbar_w,
        p.target_window_d + 2.0,
        p.base_h + 0.8,
        z0=-0.4,
    )
    target_cutouts = target_window.cut(cross_x.union(cross_y))
    base = base.cut(target_cutouts)
    return base


def build_tripod_thread_cutter(p: P = CFG) -> cq.Workplane:
    """Build a real helical 3/8"-16 UNC internal-thread cutting solid."""
    pitch = p.tripod_thread_pitch
    minor_r = p.tripod_thread_minor_d / 2.0
    major_r = p.tripod_thread_major_d / 2.0
    profile_inner_r = minor_r - p.tripod_thread_profile_overlap
    root_half_w = pitch / 16.0
    crest_half_w = root_half_w + (
        (major_r - minor_r)
        / math.tan(math.radians(p.tripod_thread_angle_deg))
    )

    # Extend the helix beyond both ends, then clip it to the usable threaded
    # length. This produces complete thread flanks at both boundary planes.
    helix_z0 = p.tripod_thread_start_z - pitch
    helix_h = (
        p.tripod_thread_end_z
        - p.tripod_thread_start_z
        + 2.0 * pitch
    )
    profile = (
        cq.Workplane("XZ")
        .moveTo(profile_inner_r, helix_z0 - crest_half_w)
        .lineTo(major_r, helix_z0 - root_half_w)
        .lineTo(major_r, helix_z0 + root_half_w)
        .lineTo(profile_inner_r, helix_z0 + crest_half_w)
        .close()
    )
    helix = cq.Wire.makeHelix(
        pitch,
        helix_h,
        (minor_r + major_r) / 2.0,
        cq.Vector(0.0, 0.0, helix_z0),
    )
    groove = profile.sweep(helix, isFrenet=True)
    clip = cylinder_z(
        major_r + 0.2,
        p.tripod_thread_end_z - p.tripod_thread_start_z,
        p.tripod_thread_start_z,
    )
    return groove.intersect(clip)


def build_tripod_base(p: P = CFG) -> cq.Workplane:
    """Build the interchangeable magnetic disk with a 3/8"-16 UNC thread."""
    base = cylinder_z(p.base_d / 2.0, p.tripod_base_h)
    try:
        base = base.edges(">Z").chamfer(p.tripod_base_outer_chamfer)
        base = base.edges("<Z").chamfer(p.tripod_base_outer_chamfer)
    except Exception:
        pass

    for x, y in magnet_centres(p):
        pocket = (
            cq.Workplane("XY")
            .center(x, y)
            .circle(p.magnet_pocket_d / 2.0)
            .extrude(p.magnet_pocket_h)
            .translate(
                (
                    0.0,
                    0.0,
                    p.tripod_base_h - p.magnet_pocket_h,
                )
            )
        )
        base = base.cut(pocket)

    pilot = cylinder_z(
        p.tripod_thread_minor_d / 2.0,
        p.tripod_base_h + 0.4,
        -0.2,
    )
    flange_pocket = cylinder_z(
        p.tripod_insert_flange_pocket_d / 2.0,
        p.tripod_insert_flange_pocket_h + 0.2,
        -0.2,
    )
    thread_groove = build_tripod_thread_cutter(p)
    # The helical profile overlaps the minor-diameter pilot by 0.05 mm so the
    # three coaxial cutters fuse into one robust Boolean solid.
    cutter = pilot.union(flange_pocket).union(thread_groove)
    return base.cut(cutter)


def build_main_body(p: P = CFG) -> cq.Workplane:
    ro = p.body_outer_d / 2.0
    ri = p.body_inner_d / 2.0

    body = cylinder_z(ro, p.body_h)
    cavity = cylinder_z(ri, p.body_h - p.body_floor_h + 0.5, p.body_floor_h)
    body = body.cut(cavity)

    # The body underside is flat against the new uniform target disk. Blind
    # magnet pockets remain open from the inside with a 0.6 mm exterior skin.
    pocket_z0 = p.body_magnet_skin
    for x, y in magnet_centres(p):
        pocket = (
            cq.Workplane("XY")
            .center(x, y)
            .circle(p.magnet_pocket_d / 2.0)
            .extrude(p.body_floor_h - pocket_z0 + 0.25)
            .translate((0.0, 0.0, pocket_z0))
        )
        body = body.cut(pocket)

    # Integral recessed control bay. A rounded arch replaces a horizontal
    # ceiling, so the opening grows progressively during upright FDM printing.
    bay_back_y = p.service_bay_panel_y + p.service_bay_panel_t
    bay_depth = bay_back_y - p.service_bay_outer_y
    bay_outer = box_xyz(
        p.service_bay_width,
        bay_depth,
        p.service_bay_height,
        0.0,
        (p.service_bay_outer_y + bay_back_y) / 2.0,
        0.0,
    )

    opening_cut_y0 = p.service_bay_outer_y - 0.6
    opening_cut_y1 = bay_back_y + 0.6
    opening_cut_len = opening_cut_y1 - opening_cut_y0
    opening_cut_cy = (opening_cut_y0 + opening_cut_y1) / 2.0
    opening_cut = box_xyz(
        p.service_bay_open_width,
        opening_cut_len,
        p.service_bay_arch_center_z - p.service_bay_open_bottom_z,
        0.0,
        opening_cut_cy,
        p.service_bay_open_bottom_z,
    ).union(
        cylinder_y(
            p.service_bay_arch_r,
            opening_cut_len,
            0.0,
            opening_cut_cy,
            p.service_bay_arch_center_z,
        )
    )
    body = body.cut(opening_cut)

    # Hollow only up to the recessed panel face, leaving a 3 mm rear panel.
    cavity_y0 = p.service_bay_outer_y - 0.6
    cavity_y1 = p.service_bay_panel_y
    cavity_len = cavity_y1 - cavity_y0
    cavity_cy = (cavity_y0 + cavity_y1) / 2.0
    bay_cavity = box_xyz(
        p.service_bay_open_width,
        cavity_len,
        p.service_bay_arch_center_z - p.service_bay_open_bottom_z,
        0.0,
        cavity_cy,
        p.service_bay_open_bottom_z,
    ).union(
        cylinder_y(
            p.service_bay_arch_r,
            cavity_len,
            0.0,
            cavity_cy,
            p.service_bay_arch_center_z,
        )
    )
    body = body.union(bay_outer.cut(bay_cavity))

    # Downward drain slot through the lower sill.
    drain = box_xyz(
        9.0,
        cavity_len + 1.0,
        p.service_bay_open_bottom_z + 0.5,
        0.0,
        cavity_cy,
        -0.2,
    )
    body = body.cut(drain)

    # Control openings through the recessed panel.
    panel_hole_cy = p.service_bay_panel_y + p.service_bay_panel_t / 2.0
    body = body.cut(
        capsule_y(
            p.usb_cut_x,
            p.usb_cut_z,
            p.service_bay_panel_t + 1.0,
            p.usb_x,
            panel_hole_cy,
            p.usb_z,
        )
    )
    body = body.cut(
        cylinder_y(
            p.switch_hole_d / 2.0,
            p.service_bay_panel_t + 1.0,
            p.switch_x,
            panel_hole_cy,
            p.switch_z,
        )
    )

    # Internal USB guide and rear stop. The rigid 25.2 mm connector body is
    # inside the enclosure; only its face is visible in the recess.
    usb_sleeve_y0 = bay_back_y
    usb_sleeve_y1 = p.service_bay_panel_y + p.usb_body_len - 0.2
    usb_sleeve = box_xyz(
        16.5,
        usb_sleeve_y1 - usb_sleeve_y0,
        11.2,
        p.usb_x,
        (usb_sleeve_y0 + usb_sleeve_y1) / 2.0,
        p.usb_z - 5.6,
    ).cut(
        capsule_y(
            p.usb_cut_x,
            p.usb_cut_z,
            usb_sleeve_y1 - usb_sleeve_y0 + 1.0,
            p.usb_x,
            (usb_sleeve_y0 + usb_sleeve_y1) / 2.0,
            p.usb_z,
        )
    )
    usb_stop = box_xyz(
        16.5,
        2.0,
        11.2,
        p.usb_x,
        usb_sleeve_y1 + 1.0,
        p.usb_z - 5.6,
    ).cut(
        capsule_y(
            9.5,
            6.3,
            3.0,
            p.usb_x,
            usb_sleeve_y1 + 1.0,
            p.usb_z,
        )
    )
    body = body.union(usb_sleeve).union(usb_stop)

    # Chassis screw bosses on the internal floor.
    for x, y in chassis_mount_centres(p):
        boss_top_z = p.body_floor_h + p.chassis_boss_h
        boss = (
            cq.Workplane("XY")
            .center(x, y)
            .circle(p.m3_boss_d / 2.0)
            .extrude(p.chassis_boss_h)
            .translate((0.0, 0.0, p.body_floor_h))
        )
        insert = (
            cq.Workplane("XY")
            .center(x, y)
            .circle(p.m3_insert_d / 2.0)
            .extrude(p.m3_insert_depth)
            .translate((0.0, 0.0, boss_top_z - p.m3_insert_depth))
        )
        lead = (
            cq.Workplane("XY")
            .center(x, y)
            .circle(p.m3_insert_lead_d / 2.0)
            .extrude(p.m3_insert_lead_depth)
            .translate((0.0, 0.0, boss_top_z - p.m3_insert_lead_depth))
        )
        body = body.union(boss).cut(insert).cut(lead)

    # Heat-set insert bosses for the upper collar screws. The revised angles
    # keep the inward bosses away from the vertical PCB.
    for angle in (0.0, 120.0, 240.0):
        a = math.radians(angle)
        ux, uy = math.cos(a), math.sin(a)
        insert_z = p.collar_start_z + 6.0
        boss = cylinder_between(
            p.m3_boss_d / 2.0,
            ((ro + 0.3) * ux, (ro + 0.3) * uy, insert_z),
            (-ux, -uy, 0.0),
            p.m3_insert_depth + 3.0,
        )
        boss = boss.intersect(cylinder_z(ro, p.body_h))
        insert = radial_hole(
            angle,
            insert_z,
            p.m3_insert_d / 2.0,
            ro + 0.5,
            p.m3_insert_depth,
        )
        lead = radial_hole(
            angle,
            insert_z,
            p.m3_insert_lead_d / 2.0,
            ro + 0.5,
            p.m3_insert_lead_depth,
        )
        body = body.union(boss).cut(insert).cut(lead)
    return body


def board_relief_boxes(p: P = CFG) -> Iterable[cq.Workplane]:
    """Back-side connector pin envelopes from the real board STEP."""
    # Original X/Y bounding rectangles of the three through-board components.
    source = (
        (143.95, 149.85, -114.95, -110.40),
        (118.17, 122.72, -105.73, -99.83),
        (146.36, 159.69, -95.88, -89.53),
    )
    for xmin, xmax, ymin, ymax in source:
        # Corrected PCB transform: model Y is horizontal and model X is
        # vertical (reversed), placing DWM up and SMA down.
        gx0 = ymin - p.uwb_antenna_model_y - 1.0
        gx1 = ymax - p.uwb_antenna_model_y + 1.0
        gz0 = p.board_vertical_offset - xmax - 1.0
        gz1 = p.board_vertical_offset - xmin + 1.0
        yield box_xyz(
            gx1 - gx0,
            p.carrier_plate_t + 3.0,
            gz1 - gz0,
            (gx0 + gx1) / 2.0,
            p.pcb_back_y + p.tape_h + p.carrier_plate_t / 2.0,
            gz0,
        )


def build_chassis(p: P = CFG) -> cq.Workplane:
    substrate_x0 = p.pcb_substrate_ymin - p.uwb_antenna_model_y
    substrate_x1 = p.pcb_substrate_ymax - p.uwb_antenna_model_y
    plate_cx = (substrate_x0 + substrate_x1) / 2.0
    plate_w = (substrate_x1 - substrate_x0) + 3.0
    plate_z0 = p.board_bottom_z - 1.5
    plate_front_y = p.pcb_back_y + p.tape_h
    plate_cy = plate_front_y + p.carrier_plate_t / 2.0

    chassis = box_xyz(
        plate_w,
        p.carrier_plate_t,
        p.carrier_plate_h,
        plate_cx,
        plate_cy,
        plate_z0,
    )
    for relief in board_relief_boxes(p):
        chassis = chassis.cut(relief)

    # Non-clamping board location guides, 0.4 mm away from PCB edges.
    left_guide = box_xyz(
        1.2,
        4.0,
        24.0,
        substrate_x0 - 1.0,
        p.pcb_back_y + 1.0,
        p.board_bottom_z + 10.0,
    )
    right_guide = box_xyz(
        1.2,
        4.0,
        20.0,
        substrate_x1 + 1.0,
        p.pcb_back_y + 1.0,
        p.board_bottom_z + 38.0,
    )
    # Two lower stops support the PCB edge while leaving the downward-facing
    # SMA connector and its pins completely unobstructed.
    bottom_rest_left = box_xyz(
        8.0,
        4.0,
        1.1,
        substrate_x0 + 5.0,
        p.pcb_back_y + 1.0,
        p.board_bottom_z - 1.1,
    )
    bottom_rest_right = box_xyz(
        28.0,
        4.0,
        1.1,
        substrate_x1 - 14.0,
        p.pcb_back_y + 1.0,
        p.board_bottom_z - 1.1,
    )
    chassis = (
        chassis.union(left_guide)
        .union(right_guide)
        .union(bottom_rest_left)
        .union(bottom_rest_right)
    )

    # Floor-standing battery tray behind the PCB. The two former long support
    # legs are removed; the tray itself now braces the carrier against the
    # inner floor. Internal cavity remains exactly 49.5 x 28 x 74.
    battery_cx = p.battery_cx
    tray_wall = p.battery_tray_wall
    tray_x = p.battery_cavity_x + 2.0 * tray_wall
    tray_y0 = p.battery_tray_outer_front_y
    tray_y1 = p.battery_tray_outer_back_y
    tray_z0 = p.battery_cavity_z0
    tray_z1 = tray_z0 + p.battery_cavity_z
    tray_wall_z0 = tray_z0 - tray_wall
    tray_wall_h = p.battery_cavity_z + 2.0 * tray_wall

    back = box_xyz(
        tray_x,
        tray_wall,
        tray_wall_h,
        battery_cx,
        tray_y1 - tray_wall / 2.0,
        tray_wall_z0,
    )
    front = box_xyz(
        tray_x,
        p.battery_front_wall_t,
        tray_wall_h,
        battery_cx,
        tray_y0 + p.battery_front_wall_t / 2.0,
        tray_wall_z0,
    )
    side_l = box_xyz(
        tray_wall,
        tray_y1 - tray_y0,
        tray_wall_h,
        battery_cx - tray_x / 2.0 + tray_wall / 2.0,
        (tray_y0 + tray_y1) / 2.0,
        tray_wall_z0,
    )
    side_r = box_xyz(
        tray_wall,
        tray_y1 - tray_y0,
        tray_wall_h,
        battery_cx + tray_x / 2.0 - tray_wall / 2.0,
        (tray_y0 + tray_y1) / 2.0,
        tray_wall_z0,
    )
    bottom = box_xyz(
        tray_x,
        tray_y1 - tray_y0,
        tray_wall,
        battery_cx,
        (tray_y0 + tray_y1) / 2.0,
        tray_wall_z0,
    )
    chassis = (
        chassis.union(back)
        .union(front)
        .union(side_l)
        .union(side_r)
        .union(bottom)
    )

    # Cable outlet between the cells, at the upper centre of the rear wall.
    wire_notch = box_xyz(
        12.0,
        tray_wall + 2.0,
        10.0,
        battery_cx,
        tray_y1 - tray_wall / 2.0,
        tray_z1 - 5.0,
    )
    chassis = chassis.cut(wire_notch)

    # Large clearance around the downward-facing SMA and the mating threaded
    # plug. The cut enters the front of the battery cradle without reaching
    # the shifted battery envelope.
    sma_relief = box_xyz(
        p.sma_relief_w,
        p.battery_front_wall_t + 6.0,
        p.sma_relief_h,
        p.sma_relief_x,
        tray_y0 + p.battery_front_wall_t / 2.0 - 1.0,
        p.sma_relief_z0,
    )
    chassis = chassis.cut(sma_relief)

    # The deeper front wall must preserve the existing JST/SMA pin reliefs.
    for relief in board_relief_boxes(p):
        chassis = chassis.cut(relief)

    # The two side consoles reproduce Alex's manually reinforced ear: the screw
    # axis stays in place, while the outer lip is pulled rearwards and tied to
    # the tray with a floor-standing block plus a tall triangular web.
    chassis_mount_z = p.body_floor_h + p.chassis_boss_h
    for x in (p.chassis_screw_x_left, p.chassis_screw_x_right):
        inward = 1.0 if x < p.battery_cx else -1.0
        flange_cx = x + inward * p.chassis_mount_flange_inward_shift
        flange = box_xyz(
            p.chassis_mount_flange_w,
            p.chassis_mount_flange_d,
            p.chassis_mount_flange_h,
            flange_cx,
            p.chassis_screw_y - p.chassis_mount_flange_forward_shift,
            chassis_mount_z,
        )

        tray_outer_side_x = (
            p.battery_cx - tray_x / 2.0
            if inward > 0.0
            else p.battery_cx + tray_x / 2.0
        )
        # End the web inside the 2 mm side wall to create a true volumetric
        # union, not merely a tangent contact.
        wall_anchor_x = tray_outer_side_x + inward * (tray_wall * 0.40)
        foot_anchor_x = x + inward * (
            p.m3_head_d / 2.0 + p.chassis_mount_gusset_head_clearance
        )
        gusset_z0 = (
            chassis_mount_z
            + p.chassis_mount_flange_h
            - p.chassis_mount_gusset_flange_overlap
        )
        gusset_y0 = p.battery_tray_outer_front_y + 0.3
        gusset = (
            cq.Workplane("XZ")
            .moveTo(foot_anchor_x, gusset_z0)
            .lineTo(wall_anchor_x, gusset_z0)
            .lineTo(
                wall_anchor_x,
                gusset_z0 + p.chassis_mount_gusset_h,
            )
            .close()
            .extrude(-p.chassis_mount_gusset_d)
            .translate((0.0, gusset_y0, 0.0))
        )

        chassis = chassis.union(flange).union(gusset)

        # Pull the front/outboard flange material back exactly as on the
        # supplied hand-edited STEP, without changing the screw location.
        outer_x = x - inward * p.chassis_side_outer_reach
        trim_cx = outer_x - inward * 10.0
        outboard_trim = box_xyz(
            20.0,
            p.chassis_mount_flange_d + 2.0,
            p.chassis_mount_flange_h + 1.0,
            trim_cx,
            p.chassis_screw_y - p.chassis_mount_flange_forward_shift,
            chassis_mount_z - 0.5,
        )
        front_trim = box_xyz(
            abs(wall_anchor_x - outer_x) + 1.0,
            p.chassis_side_front_trim_y + 1.0,
            p.chassis_mount_flange_h + 1.0,
            (wall_anchor_x + outer_x) / 2.0,
            (p.chassis_side_front_trim_y - 1.0) / 2.0,
            chassis_mount_z - 0.5,
        )
        chassis = chassis.cut(outboard_trim).cut(front_trim)

        rear_wall_anchor_x = (
            tray_outer_side_x + inward * p.chassis_side_wall_overlap
        )
        rear_span_x = abs(rear_wall_anchor_x - outer_x)
        rear_cx = (rear_wall_anchor_x + outer_x) / 2.0
        rear_floor = box_xyz(
            rear_span_x,
            p.chassis_side_rear_y1 - p.chassis_side_rear_floor_y0,
            chassis_mount_z - p.body_floor_h,
            rear_cx,
            (
                p.chassis_side_rear_floor_y0
                + p.chassis_side_rear_y1
            )
            / 2.0,
            p.body_floor_h,
        )
        rear_flange = box_xyz(
            rear_span_x,
            p.chassis_side_rear_y1 - p.chassis_side_rear_flange_y0,
            p.chassis_mount_flange_h,
            rear_cx,
            (
                p.chassis_side_rear_flange_y0
                + p.chassis_side_rear_y1
            )
            / 2.0,
            chassis_mount_z,
        )
        rear_gusset = (
            cq.Workplane("XZ")
            .moveTo(outer_x, chassis_mount_z + p.chassis_mount_flange_h)
            .lineTo(
                rear_wall_anchor_x,
                chassis_mount_z + p.chassis_mount_flange_h,
            )
            .lineTo(rear_wall_anchor_x, p.chassis_side_rear_gusset_top_z)
            .lineTo(tray_outer_side_x, p.chassis_side_rear_gusset_top_z)
            .close()
            .extrude(
                -(
                    p.chassis_side_rear_y1
                    - p.chassis_side_rear_gusset_y0
                )
            )
            .translate((0.0, p.chassis_side_rear_gusset_y0, 0.0))
        )
        chassis = (
            chassis.union(rear_floor)
            .union(rear_flange)
            .union(rear_gusset)
        )

        screw = (
            cq.Workplane("XY")
            .center(x, p.chassis_screw_y)
            .circle(p.screw_clear_d / 2.0)
            .extrude(p.chassis_mount_flange_h + 1.0)
            .translate((0.0, 0.0, chassis_mount_z - 0.5))
        )
        chassis = chassis.cut(screw)

    # A third mounting ear is tied into the rear tray wall. Two separated ribs
    # leave the M3 head unobstructed while resisting transport loads in both
    # bending directions.
    rear_wall_anchor_y = tray_y1 - p.chassis_side_wall_overlap
    rear_flange = box_xyz(
        p.chassis_rear_flange_w,
        p.chassis_rear_flange_back_y - rear_wall_anchor_y,
        p.chassis_mount_flange_h,
        p.chassis_rear_screw_x,
        (rear_wall_anchor_y + p.chassis_rear_flange_back_y) / 2.0,
        chassis_mount_z,
    )
    rear_bridge = box_xyz(
        p.chassis_rear_bridge_w,
        p.chassis_rear_bridge_back_y - rear_wall_anchor_y,
        chassis_mount_z - p.body_floor_h,
        p.chassis_rear_screw_x,
        (rear_wall_anchor_y + p.chassis_rear_bridge_back_y) / 2.0,
        p.body_floor_h,
    )
    def rear_rib(x0: float) -> cq.Workplane:
        return (
            cq.Workplane("YZ")
            .moveTo(
                p.chassis_rear_flange_back_y,
                chassis_mount_z + p.chassis_mount_flange_h,
            )
            .lineTo(
                rear_wall_anchor_y,
                chassis_mount_z + p.chassis_mount_flange_h,
            )
            .lineTo(rear_wall_anchor_y, p.chassis_rear_rib_top_z)
            .close()
            .extrude(p.chassis_rear_rib_w)
            .translate((x0, 0.0, 0.0))
        )

    rear_left_rib = rear_rib(
        p.chassis_rear_screw_x - p.chassis_rear_flange_w / 2.0
    )
    rear_right_rib = rear_rib(
        p.chassis_rear_screw_x
        + p.chassis_rear_flange_w / 2.0
        - p.chassis_rear_rib_w
    )
    rear_screw = (
        cq.Workplane("XY")
        .center(p.chassis_rear_screw_x, p.chassis_rear_screw_y)
        .circle(p.screw_clear_d / 2.0)
        .extrude(p.chassis_mount_flange_h + 1.0)
        .translate((0.0, 0.0, chassis_mount_z - 0.5))
    )
    chassis = (
        chassis.union(rear_flange)
        .union(rear_bridge)
        .union(rear_left_rib)
        .union(rear_right_rib)
        .cut(rear_screw)
    )

    # Preserve the full cylindrical connector envelope after adding the left
    # console. A round finishing cut keeps the useful rear portion of its
    # flange/web, routing the reinforcement around (not through) the SMA plug.
    sma_console_clearance = cylinder_z(
        (p.sma_mating_clearance_d + 0.5) / 2.0,
        p.sma_mating_h + 1.0,
        p.sma_mating_z0 - 0.5,
    ).translate((p.sma_relief_x, p.sma_mating_y, 0.0))
    chassis = chassis.cut(sma_console_clearance)
    return chassis


def build_battery_retainer(p: P = CFG) -> cq.Workplane:
    battery_cx = p.battery_cx
    wall = p.battery_tray_wall
    tray_outer_x = p.battery_cavity_x + 2.0 * wall
    tray_outer_y = (
        p.battery_front_wall_t + p.battery_cavity_y + wall
    )
    outer_x = tray_outer_x + 2.0
    outer_y = tray_outer_y + 2.0
    retainer = box_xyz(outer_x, outer_y, 5.0, battery_cx, 0.0, 0.0)

    # Three millimetre-deep sleeve around the outside of the battery tray.
    underside_cavity = box_xyz(
        tray_outer_x + 0.5,
        tray_outer_y + 0.5,
        3.2,
        battery_cx,
        0.0,
        -0.1,
    )
    retainer = retainer.cut(underside_cavity)

    # The front of the sleeve is open because the PCB carrier plate forms the
    # front wall of the battery tray.
    front_relief = box_xyz(
        outer_x + 1.0,
        3.0,
        6.0,
        battery_cx,
        -outer_y / 2.0 + 1.0,
        -0.5,
    )
    retainer = retainer.cut(front_relief)

    opening = box_xyz(
        p.battery_cavity_x - 8.0,
        p.battery_cavity_y - 8.0,
        6.0,
        battery_cx,
        0.0,
        -0.5,
    )
    retainer = retainer.cut(opening)
    # Open cable slot from the rear edge into the large centre opening.
    retainer = retainer.cut(
        box_xyz(12.0, 10.0, 6.0, battery_cx, outer_y / 2.0 - 3.0, -0.5)
    )
    return retainer


def build_upper_collar(p: P = CFG) -> cq.Workplane:
    sleeve_ro = p.collar_sleeve_outer_d / 2.0
    sleeve_ri = p.collar_sleeve_inner_d / 2.0
    neck_ro = p.collar_neck_outer_d / 2.0
    neck_ri = p.body_inner_d / 2.0
    ledge_ri = p.collar_tray_ledge_inner_d / 2.0

    sleeve = ring(sleeve_ro, sleeve_ri, 14.0, 0.0)
    neck = ring(neck_ro, neck_ri, p.collar_h - 12.0, 12.0)
    shoulder = ring(sleeve_ro, neck_ri, 2.0, 12.0)
    collar = sleeve.union(neck).union(shoulder)

    # The shoulder used to begin with a 3.1 mm horizontal annular overhang.
    # Remove its inner underside as a 45-degree ramp, retaining only a short
    # 0.6 mm bridge and a 0.3 mm radial seating land on the body's top rim.
    shoulder_chamfer_bottom_r = sleeve_ri - p.collar_shoulder_bridge_w
    shoulder_chamfer_h = shoulder_chamfer_bottom_r - neck_ri
    shoulder_chamfer_cut = cone_z(
        shoulder_chamfer_bottom_r,
        neck_ri,
        shoulder_chamfer_h,
        12.0,
    )
    collar = collar.cut(shoulder_chamfer_cut)

    # Internal ledge for the removable GPS tray.
    tray_ledge_top_local = p.gps_tray_top_z - 3.0 - p.collar_start_z
    tray_ledge_bottom_local = tray_ledge_top_local - 2.5
    ledge = ring(neck_ri, ledge_ri, 2.5, tray_ledge_bottom_local)
    ledge_support = inward_45deg_support_ring(
        neck_ri,
        ledge_ri,
        tray_ledge_bottom_local,
    )
    collar = collar.union(ledge_support).union(ledge)

    # Clearance holes for collar-to-body screws.
    for angle in (0.0, 120.0, 240.0):
        collar = collar.cut(
            radial_hole(angle, 6.0, p.screw_clear_d / 2.0, sleeve_ro + 1.0, 6.0)
        )

    # Heat-set insert bosses for the cap screws.
    cap_hole_global_z = p.hat_bottom_z + 8.0
    cap_hole_local_z = cap_hole_global_z - p.collar_start_z
    for angle in cap_screw_angles():
        a = math.radians(angle)
        ux, uy = math.cos(a), math.sin(a)
        boss = cylinder_between(
            p.m3_boss_d / 2.0,
            ((neck_ro + 0.3) * ux, (neck_ro + 0.3) * uy, cap_hole_local_z),
            (-ux, -uy, 0.0),
            p.m3_insert_depth + 3.0,
        )
        boss = boss.intersect(cylinder_z(neck_ro, p.collar_h))
        insert = radial_hole(
            angle,
            cap_hole_local_z,
            p.m3_insert_d / 2.0,
            neck_ro + 0.5,
            p.m3_insert_depth,
        )
        lead = radial_hole(
            angle,
            cap_hole_local_z,
            p.m3_insert_lead_d / 2.0,
            neck_ro + 0.5,
            p.m3_insert_lead_depth,
        )
        collar = collar.union(boss).cut(insert).cut(lead)
    return collar


def build_gps_tray(p: P = CFG) -> cq.Workplane:
    tray_outer_r = p.gps_tray_outer_d / 2.0
    tray_ring_inner_r = p.gps_tray_ring_inner_d / 2.0
    tray = ring(tray_outer_r, tray_ring_inner_r, 3.0)
    support = ring(32.5, 22.0, 3.0)
    tray = tray.union(support)

    # Alex's reinforced STEP adds a circular lobe around each passage. The
    # Ø28 lobes are clipped to the tray outline, then the boss slots are cut
    # through them below. This thickens the narrow necks without changing the
    # outer diameter or the verified insertion clearances.
    tray_outline = cylinder_z(tray_outer_r, 3.0)
    for angle, center_r in zip(
        cap_screw_angles(),
        gps_tray_reinforcement_center_radii(p),
    ):
        a = math.radians(angle)
        lobe = (
            cq.Workplane("XY")
            .center(center_r * math.cos(a), center_r * math.sin(a))
            .circle(p.gps_tray_reinforcement_lobe_r)
            .extrude(3.0)
            .intersect(tray_outline)
        )
        tray = tray.union(lobe)

    # The real antenna STEP has four Ø2.6 mm holes at radius 42.5 mm, on the
    # cardinal axes. Four arms and standoffs are derived from those centres.
    standoff_h = (
        p.gps_flower_bottom_z
        - p.gps_tray_top_z
        - p.gps_mount_top_gap
    )
    for angle in (0.0, 90.0, 180.0, 270.0):
        arm = box_xyz(
            27.0, 8.0, 3.0, p.gps_mount_radius, 0.0, 0.0
        ).rotate(
            (0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle
        )
        tray = tray.union(arm)

        a = math.radians(angle)
        x = p.gps_mount_radius * math.cos(a)
        y = p.gps_mount_radius * math.sin(a)
        standoff = (
            cq.Workplane("XY")
            .center(x, y)
            .circle(p.gps_mount_standoff_d / 2.0)
            .extrude(standoff_h)
            .translate((0.0, 0.0, 3.0))
        )
        mount_hole = (
            cq.Workplane("XY")
            .center(x, y)
            .circle(p.gps_mount_clear_d / 2.0)
            .extrude(3.0 + standoff_h + 0.4)
            .translate((0.0, 0.0, -0.2))
        )
        standoff_top_z = 3.0 + standoff_h
        insert_pocket = (
            cq.Workplane("XY")
            .center(x, y)
            .circle(p.m25_insert_d / 2.0)
            .extrude(p.m25_insert_depth)
            .translate((0.0, 0.0, standoff_top_z - p.m25_insert_depth))
        )
        insert_lead = (
            cq.Workplane("XY")
            .center(x, y)
            .circle(p.m25_insert_lead_d / 2.0)
            .extrude(p.m25_insert_lead_depth)
            .translate(
                (0.0, 0.0, standoff_top_z - p.m25_insert_lead_depth)
            )
        )
        tray = (
            tray.union(standoff)
            .cut(mount_hole)
            .cut(insert_pocket)
            .cut(insert_lead)
        )

    # Three rounded radial slots align with the cap-insert bosses. They allow
    # the tray to descend axially through the collar instead of relying on an
    # impractical tilted insertion with only 0.6 mm radial clearance.
    cap_boss_inner_r = (
        p.collar_neck_outer_d / 2.0
        + 0.3
        - (p.m3_insert_depth + 3.0)
    )
    notch_inner_r = cap_boss_inner_r - p.gps_tray_boss_passage_clearance
    notch_outer_r = tray_outer_r + p.gps_tray_boss_passage_clearance
    notch_radial_len = notch_outer_r - notch_inner_r
    notch_tangential_w = (
        p.m3_boss_d + 2.0 * p.gps_tray_boss_passage_clearance
    )
    notch_cx = (notch_inner_r + notch_outer_r) / 2.0
    for angle in cap_screw_angles():
        notch = box_xyz(
            notch_radial_len,
            notch_tangential_w,
            3.4,
            notch_cx,
            0.0,
            -0.2,
        )
        try:
            notch = notch.edges("|Z").fillet(p.gps_tray_notch_corner_r)
        except Exception:
            pass
        notch = notch.rotate(
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 1.0),
            angle,
        )
        tray = tray.cut(notch)
    return tray


def build_hat(p: P = CFG) -> cq.Workplane:
    skirt_ro = p.hat_skirt_outer_d / 2.0
    skirt_ri = p.hat_skirt_inner_d / 2.0
    hat = ring(skirt_ro, skirt_ri, p.hat_skirt_h)
    roof = cylinder_z(p.hat_outer_d / 2.0, p.hat_roof_h, p.hat_skirt_h)
    hat = hat.union(roof)

    for angle in cap_screw_angles():
        hat = hat.cut(
            radial_hole(angle, 8.0, p.screw_clear_d / 2.0, skirt_ro + 1.0, 6.0)
        )
    return hat


def build_reference_shapes(p: P = CFG) -> Dict[str, cq.Shape]:
    pcb = cq.importers.importStep(
        str(REFERENCE_DIR / "modul_radio" / "UWB.step")
    ).val()
    # (x, y, z) -> (y, -z, -x): DWM module is up, SMA connector is down.
    pcb = pcb.rotate((0, 0, 0), (1, 0, 0), 90.0)
    pcb = pcb.rotate((0, 0, 0), (0, 1, 0), 90.0)
    pcb = pcb.translate(
        (
            -p.uwb_antenna_model_y,
            p.uwb_antenna_model_z,
            p.board_vertical_offset,
        )
    )

    gps = cq.importers.importStep(
        str(
            REFERENCE_DIR
            / "antena_gps"
            / "YN-91A_refined_v04_assembly.step"
        )
    ).val()
    gps = gps.rotate((0, 0, 0), (1, 0, 0), 180.0)
    gps = gps.translate((0.0, 0.0, p.gps_origin_z))

    battery = box_xyz(
        p.battery_x,
        p.battery_y,
        p.battery_z,
        p.battery_cx,
        p.battery_front_y + p.battery_y / 2.0,
        p.battery_cavity_z0 + 0.5,
    ).val()

    return {"pcb": pcb, "gps": gps, "battery": battery}


def build_usb_placeholder(p: P = CFG) -> cq.Shape:
    face_y = p.service_bay_panel_y - 0.2
    body = capsule_y(
        p.usb_face_x,
        p.usb_face_z,
        p.usb_body_len,
        p.usb_x,
        face_y + p.usb_body_len / 2.0,
        p.usb_z,
    )
    strain_y0 = face_y + p.usb_body_len
    strain = capsule_y(
        9.0,
        5.8,
        p.usb_strain_len,
        p.usb_x,
        strain_y0 + p.usb_strain_len / 2.0,
        p.usb_z,
    )
    return body.union(strain).val()


def build_switch_placeholder(p: P = CFG) -> cq.Shape:
    body_y0 = p.service_bay_panel_y + p.service_bay_panel_t
    body = box_xyz(
        p.switch_body_x,
        p.switch_body_y,
        p.switch_body_z,
        p.switch_x,
        body_y0 + p.switch_body_y / 2.0,
        p.switch_z - p.switch_body_z / 2.0,
    )
    # Conservative lever envelope: the tip remains inside the outer rim.
    tip_y = p.service_bay_outer_y + 3.0
    shaft = cylinder_between(
        p.switch_bushing_d / 2.0,
        (p.switch_x, tip_y, p.switch_z),
        (0.0, 1.0, 0.0),
        body_y0 - tip_y,
    )
    return body.union(shaft).val()


def build_sma_mating_placeholder(p: P = CFG) -> cq.Shape:
    """Conservative Ø15 mm envelope for the threaded GPS cable connector."""
    return (
        cq.Workplane("XY")
        .center(p.sma_relief_x, p.sma_mating_y)
        .circle(p.sma_mating_clearance_d / 2.0)
        .extrude(p.sma_mating_h)
        .translate((0.0, 0.0, p.sma_mating_z0))
        .val()
    )


def export_part(name: str, part: cq.Workplane) -> Dict[str, object]:
    EXPORT_DIR.mkdir(parents=True, exist_ok=True)
    step_path = EXPORT_DIR / f"{name}.step"
    stl_path = EXPORT_DIR / f"{name}.stl"
    print_stl_path = EXPORT_DIR / f"{name}_PRINT.stl"
    cq.exporters.export(part, str(step_path))
    mesh_tolerance = 0.04 if name == "08_baza_trepied" else 0.08
    angular_tolerance = 0.06 if name == "08_baza_trepied" else 0.12
    cq.exporters.export(
        part,
        str(stl_path),
        tolerance=mesh_tolerance,
        angularTolerance=angular_tolerance,
    )

    print_shape = part.val()
    # The battery cap and rain hat print cleanly upside-down, with their broad
    # top faces on the build plate.
    if name in {"04_retainer_acumulator", "07_capac_palarie"}:
        print_shape = print_shape.rotate(
            (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 180.0
        )
    print_bb = print_shape.BoundingBox()
    print_shape = print_shape.translate((0.0, 0.0, -print_bb.zmin))
    cq.exporters.export(
        print_shape,
        str(print_stl_path),
        tolerance=mesh_tolerance,
        angularTolerance=angular_tolerance,
    )

    bb = part.val().BoundingBox()
    return {
        "step": step_path.name,
        "stl": stl_path.name,
        "print_stl": print_stl_path.name,
        "size_mm": [round(bb.xlen, 3), round(bb.ylen, 3), round(bb.zlen, 3)],
        "solids": len(part.val().Solids()),
        "valid_brep": part.val().isValid(),
        "volume_mm3": round(part.val().Volume(), 1),
    }


def volume_intersection(a: cq.Shape, b: cq.Shape) -> float:
    try:
        return a.intersect(b).Volume()
    except Exception:
        return float("nan")


def build_and_export() -> None:
    os.environ.setdefault("XDG_CACHE_HOME", str(PROJECT_DIR / ".cache"))
    EXPORT_DIR.mkdir(parents=True, exist_ok=True)
    PREVIEW_DIR.mkdir(parents=True, exist_ok=True)

    parts = {
        "01_baza_tinta": build_target_base(),
        "02_corp_inferior": build_main_body(),
        "03_sasiu_pcb_acumulator": build_chassis(),
        "04_retainer_acumulator": build_battery_retainer(),
        "05_guler_superior": build_upper_collar(),
        "06_suport_antena_gps": build_gps_tray(),
        "07_capac_palarie": build_hat(),
        "08_baza_trepied": build_tripod_base(),
    }

    manifest = {"parameters": asdict(CFG), "parts": {}}
    for name, part in parts.items():
        print(f"Exporting {name}...")
        manifest["parts"][name] = export_part(name, part)

    refs = build_reference_shapes()

    # Assembly locations.
    base_z = -CFG.base_h
    chassis_loc = cq.Location()
    collar_loc = cq.Location((0.0, 0.0, CFG.collar_start_z))
    tray_loc = cq.Location((0.0, 0.0, CFG.gps_tray_top_z - 3.0))
    hat_loc = cq.Location((0.0, 0.0, CFG.hat_bottom_z))
    retainer_loc = cq.Location(
        (
            0.0,
            CFG.battery_tray_center_y,
            CFG.battery_retainer_z,
        )
    )

    assy = cq.Assembly(name="UWB_GPS_enclosure_modular_v4")
    assy.add(
        parts["01_baza_tinta"],
        loc=cq.Location((0.0, 0.0, base_z)),
        name="01_baza_tinta",
        color=cq.Color(0.92, 0.72, 0.18),
    )
    assy.add(
        parts["02_corp_inferior"],
        name="02_corp_inferior",
        color=cq.Color(0.22, 0.42, 0.64, 0.55),
    )
    assy.add(
        parts["03_sasiu_pcb_acumulator"],
        loc=chassis_loc,
        name="03_sasiu_pcb_acumulator",
        color=cq.Color(0.90, 0.52, 0.20),
    )
    assy.add(
        parts["04_retainer_acumulator"],
        loc=retainer_loc,
        name="04_retainer_acumulator",
        color=cq.Color(0.90, 0.52, 0.20),
    )
    assy.add(
        parts["05_guler_superior"],
        loc=collar_loc,
        name="05_guler_superior",
        color=cq.Color(0.25, 0.48, 0.70, 0.55),
    )
    assy.add(
        parts["06_suport_antena_gps"],
        loc=tray_loc,
        name="06_suport_antena_gps",
        color=cq.Color(0.92, 0.62, 0.16),
    )
    assy.add(
        parts["07_capac_palarie"],
        loc=hat_loc,
        name="07_capac_palarie",
        color=cq.Color(0.18, 0.36, 0.58, 0.50),
    )
    assy.add(refs["pcb"], name="REF_placa_UWB", color=cq.Color(0.15, 0.55, 0.25))
    assy.add(refs["gps"], name="REF_antena_GPS", color=cq.Color(0.15, 0.65, 0.25))
    assy.add(
        refs["battery"],
        name="REF_acumulator_48_5x25x73",
        color=cq.Color(0.55, 0.20, 0.72),
    )
    assy.add(
        build_usb_placeholder(),
        name="REF_USB_C_mama",
        color=cq.Color(0.08, 0.08, 0.08),
    )
    assy.add(
        build_switch_placeholder(),
        name="REF_intrerupator_DPDT",
        color=cq.Color(0.55, 0.55, 0.58),
    )
    assy.add(
        build_sma_mating_placeholder(),
        name="REF_garda_mufa_SMA_D15",
        color=cq.Color(0.75, 0.62, 0.18, 0.65),
    )

    assembly_path = EXPORT_DIR / "UWB_carcasa_modulara_ansamblu_verificare.step"
    print(f"Saving verification assembly: {assembly_path.name}")
    assy.save(str(assembly_path))

    # Print a compact, machine-readable validation report.
    body = parts["02_corp_inferior"].val()
    chassis = parts["03_sasiu_pcb_acumulator"].val()
    gps_tray = parts["06_suport_antena_gps"].val().moved(tray_loc)
    collar = parts["05_guler_superior"].val().moved(collar_loc)
    hat = parts["07_capac_palarie"].val().moved(hat_loc)
    battery_retainer = parts["04_retainer_acumulator"].val().moved(retainer_loc)
    base = parts["01_baza_tinta"].val().moved(
        cq.Location((0.0, 0.0, base_z))
    )
    tripod_base = parts["08_baza_trepied"].val().moved(
        cq.Location((0.0, 0.0, -CFG.tripod_base_h))
    )
    usb_ref = build_usb_placeholder()
    switch_ref = build_switch_placeholder()
    sma_mating_ref = build_sma_mating_placeholder()

    # Verify the complete straight insertion path from above the collar down
    # onto the GPS ledge, not only the final assembled position.
    collar_local = parts["05_guler_superior"].val()
    gps_tray_local = parts["06_suport_antena_gps"].val()
    gps_tray_final_local_z = (
        CFG.gps_tray_top_z - 3.0 - CFG.collar_start_z
    )
    insertion_sample_step = 0.5
    insertion_sample_count = (
        math.ceil(
            (CFG.collar_h - gps_tray_final_local_z)
            / insertion_sample_step
        )
        + 1
    )
    insertion_intersections = [
        volume_intersection(
            collar_local,
            gps_tray_local.translate(
                (
                    0.0,
                    0.0,
                    min(
                        gps_tray_final_local_z
                        + index * insertion_sample_step,
                        CFG.collar_h,
                    ),
                )
            ),
        )
        for index in range(insertion_sample_count)
    ]
    insertion_max_intersection = max(insertion_intersections)

    checks = {
        "pcb_vs_body_intersection_mm3": round(volume_intersection(refs["pcb"], body), 5),
        "pcb_vs_chassis_intersection_mm3": round(
            volume_intersection(refs["pcb"], chassis), 5
        ),
        "pcb_vs_collar_intersection_mm3": round(
            volume_intersection(refs["pcb"], collar), 5
        ),
        "pcb_vs_hat_intersection_mm3": round(
            volume_intersection(refs["pcb"], hat), 5
        ),
        "battery_vs_chassis_intersection_mm3": round(
            volume_intersection(refs["battery"], chassis), 5
        ),
        "battery_vs_pcb_intersection_mm3": round(
            volume_intersection(refs["battery"], refs["pcb"]), 5
        ),
        "battery_vs_sma_mating_guard_intersection_mm3": round(
            volume_intersection(refs["battery"], sma_mating_ref), 5
        ),
        "battery_vs_body_intersection_mm3": round(
            volume_intersection(refs["battery"], body), 5
        ),
        "battery_vs_collar_intersection_mm3": round(
            volume_intersection(refs["battery"], collar), 5
        ),
        "gps_vs_collar_intersection_mm3": round(
            volume_intersection(refs["gps"], collar), 5
        ),
        "gps_vs_tray_intersection_mm3": round(
            volume_intersection(refs["gps"], gps_tray), 5
        ),
        "gps_tray_vs_collar_intersection_mm3": round(
            volume_intersection(gps_tray, collar), 5
        ),
        "gps_tray_axial_insertion_max_intersection_mm3": round(
            insertion_max_intersection,
            5,
        ),
        "gps_vs_hat_intersection_mm3": round(
            volume_intersection(refs["gps"], hat), 5
        ),
        "body_vs_chassis_intersection_mm3": round(
            volume_intersection(body, chassis), 5
        ),
        "body_vs_collar_intersection_mm3": round(
            volume_intersection(body, collar), 5
        ),
        "collar_vs_hat_intersection_mm3": round(
            volume_intersection(collar, hat), 5
        ),
        "chassis_vs_battery_retainer_intersection_mm3": round(
            volume_intersection(chassis, battery_retainer), 5
        ),
        "battery_vs_retainer_intersection_mm3": round(
            volume_intersection(refs["battery"], battery_retainer), 5
        ),
        "base_vs_body_intersection_mm3": round(
            volume_intersection(base, body), 5
        ),
        "tripod_base_vs_body_intersection_mm3": round(
            volume_intersection(tripod_base, body), 5
        ),
        "usb_vs_body_intersection_mm3": round(
            volume_intersection(usb_ref, body), 5
        ),
        "switch_vs_body_intersection_mm3": round(
            volume_intersection(switch_ref, body), 5
        ),
        "usb_vs_chassis_intersection_mm3": round(
            volume_intersection(usb_ref, chassis), 5
        ),
        "usb_vs_pcb_intersection_mm3": round(
            volume_intersection(usb_ref, refs["pcb"]), 5
        ),
        "usb_vs_battery_intersection_mm3": round(
            volume_intersection(usb_ref, refs["battery"]), 5
        ),
        "switch_vs_chassis_intersection_mm3": round(
            volume_intersection(switch_ref, chassis), 5
        ),
        "switch_vs_pcb_intersection_mm3": round(
            volume_intersection(switch_ref, refs["pcb"]), 5
        ),
        "switch_vs_battery_intersection_mm3": round(
            volume_intersection(switch_ref, refs["battery"]), 5
        ),
        "sma_mating_guard_vs_chassis_intersection_mm3": round(
            volume_intersection(sma_mating_ref, chassis), 5
        ),
        "sma_mating_guard_vs_body_intersection_mm3": round(
            volume_intersection(sma_mating_ref, body), 5
        ),
        "board_bottom_z_mm": round(CFG.board_bottom_z, 3),
        "pcb_edge_to_inner_floor_mm": round(
            CFG.board_bottom_z - CFG.body_floor_h, 3
        ),
        "board_assembly_bottom_z_mm": round(CFG.board_assembly_bottom_z, 3),
        "board_top_z_mm": round(CFG.board_top_z, 3),
        "gps_flower_bottom_z_mm": round(CFG.gps_flower_bottom_z, 3),
        "board_to_gps_flower_mm": round(CFG.board_to_gps_flower, 3),
        "gps_flipped_180deg_about_x": True,
        "uwb_antenna_axis_x_mm": 0.0,
        "uwb_antenna_axis_y_mm": 0.0,
        "uwb_antenna_model_x_mm": CFG.uwb_antenna_model_x,
        "uwb_antenna_model_y_mm": CFG.uwb_antenna_model_y,
        "uwb_antenna_height_mm": round(CFG.uwb_antenna_height_z, 3),
        "dwm_above_sma_extreme_mm": round(
            CFG.board_top_z - CFG.board_assembly_bottom_z, 3
        ),
        "battery_bottom_z_mm": round(CFG.battery_cavity_z0 + 0.5, 3),
        "battery_top_z_mm": round(
            CFG.battery_cavity_z0 + 0.5 + CFG.battery_z, 3
        ),
        "battery_to_uwb_antenna_center_mm": round(
            CFG.uwb_antenna_height_z
            - (CFG.battery_cavity_z0 + 0.5 + CFG.battery_z),
            3,
        ),
        "sma_relief_width_mm": CFG.sma_relief_w,
        "sma_relief_height_mm": CFG.sma_relief_h,
        "sma_mating_guard_diameter_mm": CFG.sma_mating_clearance_d,
        "battery_front_y_mm": round(CFG.battery_front_y, 3),
        "target_base_diameter_mm": CFG.base_d,
        "target_base_uniform_thickness_mm": CFG.base_h,
        "target_window_diameter_mm": CFG.target_window_d,
        "target_crossbar_width_mm": CFG.target_crossbar_w,
        "target_window_is_through_cut": True,
        "tripod_base_diameter_mm": CFG.base_d,
        "tripod_base_uniform_thickness_mm": CFG.tripod_base_h,
        "tripod_base_outer_chamfer_top_mm": (
            CFG.tripod_base_outer_chamfer
        ),
        "tripod_base_outer_chamfer_bottom_mm": (
            CFG.tripod_base_outer_chamfer
        ),
        "tripod_thread_standard": '3/8"-16 UNC',
        "tripod_thread_is_modeled_helical": True,
        "tripod_thread_tpi": CFG.tripod_thread_tpi,
        "tripod_thread_pitch_mm": CFG.tripod_thread_pitch,
        "tripod_thread_angle_deg": CFG.tripod_thread_angle_deg,
        "tripod_thread_nominal_major_diameter_mm": (
            CFG.tripod_thread_nominal_major_d
        ),
        "tripod_thread_modeled_major_diameter_mm": (
            CFG.tripod_thread_major_d
        ),
        "tripod_thread_nominal_major_diametral_clearance_mm": round(
            CFG.tripod_thread_major_d
            - CFG.tripod_thread_nominal_major_d,
            3,
        ),
        "tripod_thread_minor_diameter_mm": CFG.tripod_thread_minor_d,
        "tripod_thread_radial_depth_mm": round(
            (
                CFG.tripod_thread_major_d
                - CFG.tripod_thread_minor_d
            )
            / 2.0,
            3,
        ),
        "tripod_thread_start_z_mm": CFG.tripod_thread_start_z,
        "tripod_thread_end_z_mm": CFG.tripod_thread_end_z,
        "tripod_thread_usable_length_mm": round(
            CFG.tripod_thread_end_z
            - CFG.tripod_thread_start_z,
            3,
        ),
        "tripod_thread_turn_count": round(
            (
                CFG.tripod_thread_end_z
                - CFG.tripod_thread_start_z
            )
            / CFG.tripod_thread_pitch,
            3,
        ),
        "tripod_insert_outer_diameter_mm": CFG.tripod_insert_outer_d,
        "tripod_insert_total_height_mm": CFG.tripod_insert_total_h,
        "tripod_insert_flange_diameter_mm": CFG.tripod_insert_flange_d,
        "tripod_insert_flange_height_mm": CFG.tripod_insert_flange_h,
        "tripod_insert_flange_pocket_diameter_mm": (
            CFG.tripod_insert_flange_pocket_d
        ),
        "tripod_insert_flange_diametral_clearance_mm": round(
            CFG.tripod_insert_flange_pocket_d
            - CFG.tripod_insert_flange_d,
            3,
        ),
        "tripod_insert_flange_pocket_depth_mm": (
            CFG.tripod_insert_flange_pocket_h
        ),
        "tripod_insert_flange_depth_clearance_mm": round(
            CFG.tripod_insert_flange_pocket_h
            - CFG.tripod_insert_flange_h,
            3,
        ),
        "tripod_insert_magnet_face_recess_mm": round(
            CFG.tripod_base_h - CFG.tripod_insert_total_h,
            3,
        ),
        "tripod_base_insert_fully_between_faces": (
            CFG.tripod_base_h >= CFG.tripod_insert_total_h
        ),
        "magnet_pair_count": 4,
        "magnet_radius_mm": CFG.magnet_radius,
        "magnet_angle_offset_deg": CFG.magnet_angle_offset_deg,
        "magnet_outer_edge_margin_mm": round(
            CFG.base_d / 2.0
            - CFG.magnet_radius
            - CFG.magnet_pocket_d / 2.0,
            3,
        ),
        "magnet_radius_increase_factor": round(CFG.magnet_radius / 19.0, 3),
        "service_bay_recess_depth_mm": round(
            CFG.service_bay_panel_y - CFG.service_bay_outer_y, 3
        ),
        "service_bay_outer_projection_center_mm": round(
            abs(CFG.service_bay_outer_y) - CFG.body_outer_d / 2.0, 3
        ),
        "service_bay_outer_projection_at_edge_mm": round(
            abs(CFG.service_bay_outer_y)
            - math.sqrt(
                (CFG.body_outer_d / 2.0) ** 2
                - (CFG.service_bay_width / 2.0) ** 2
            ),
            3,
        ),
        "switch_tip_inside_outer_rim_mm": 3.0,
        "m3_insert_measured_body_diameter_mm": CFG.m3_insert_body_d,
        "m3_insert_measured_knurl_diameter_mm": CFG.m3_insert_knurl_d,
        "m3_insert_measured_length_mm": CFG.m3_insert_length,
        "m3_insert_pocket_diameter_mm": CFG.m3_insert_d,
        "m3_insert_knurl_diametral_interference_mm": round(
            CFG.m3_insert_knurl_d - CFG.m3_insert_d,
            3,
        ),
        "m3_insert_pocket_depth_mm": CFG.m3_insert_depth,
        "m3_insert_pocket_overdepth_mm": round(
            CFG.m3_insert_depth - CFG.m3_insert_length,
            3,
        ),
        "m3_insert_lead_diameter_mm": CFG.m3_insert_lead_d,
        "m3_insert_lead_depth_mm": CFG.m3_insert_lead_depth,
        "m3_insert_boss_min_radial_wall_at_lead_mm": round(
            (CFG.m3_boss_d - CFG.m3_insert_lead_d) / 2.0,
            3,
        ),
        "m3_clearance_hole_diameter_mm": CFG.screw_clear_d,
        "m3_heat_set_insert_total_count": 9,
        "m3_heat_set_insert_vertical_count": 3,
        "m3_heat_set_insert_radial_count": 6,
        "chassis_mount_count": 3,
        "chassis_mount_centres_mm": [
            [round(x, 3), round(y, 3)]
            for x, y in chassis_mount_centres(CFG)
        ],
        "chassis_mount_flange_width_mm": CFG.chassis_mount_flange_w,
        "chassis_mount_flange_depth_mm": CFG.chassis_mount_flange_d,
        "chassis_mount_flange_thickness_mm": CFG.chassis_mount_flange_h,
        "chassis_mount_flange_to_battery_gap_mm": round(
            CFG.battery_front_y
            - (
                CFG.chassis_screw_y
                - CFG.chassis_mount_flange_forward_shift
                + CFG.chassis_mount_flange_d / 2.0
            ),
            3,
        ),
        "chassis_mount_gusset_height_mm": CFG.chassis_mount_gusset_h,
        "chassis_mount_gusset_depth_mm": CFG.chassis_mount_gusset_d,
        "chassis_mount_left_inboard_overlap_mm": round(
            (
                CFG.chassis_screw_x_left
                + CFG.chassis_mount_flange_inward_shift
                + CFG.chassis_mount_flange_w / 2.0
            )
            - (CFG.battery_cx - CFG.battery_cavity_x / 2.0 - CFG.battery_tray_wall),
            3,
        ),
        "chassis_mount_right_inboard_overlap_mm": round(
            (CFG.battery_cx + CFG.battery_cavity_x / 2.0 + CFG.battery_tray_wall)
            - (
                CFG.chassis_screw_x_right
                - CFG.chassis_mount_flange_inward_shift
                - CFG.chassis_mount_flange_w / 2.0
            ),
            3,
        ),
        "chassis_mount_screw_head_to_gusset_clearance_mm": (
            CFG.chassis_mount_gusset_head_clearance
        ),
        "chassis_side_reinforcement_rear_y_mm": CFG.chassis_side_rear_y1,
        "chassis_rear_mount_rib_count": 2,
        "chassis_rear_mount_rib_top_z_mm": CFG.chassis_rear_rib_top_z,
        "collar_internal_overhang_transition_angle_deg": 45.0,
        "collar_shoulder_bridge_width_mm": CFG.collar_shoulder_bridge_w,
        "collar_tray_ledge_support_radial_width_mm": round(
            CFG.body_inner_d / 2.0
            - CFG.collar_tray_ledge_inner_d / 2.0,
            3,
        ),
        "collar_functional_diameters_unchanged": True,
        "gps_tray_cap_boss_passage_notch_count": len(cap_screw_angles()),
        "gps_tray_cap_boss_passage_clearance_mm": (
            CFG.gps_tray_boss_passage_clearance
        ),
        "gps_tray_reinforced_passage_count": len(cap_screw_angles()),
        "gps_tray_reinforcement_lobe_diameter_mm": (
            2.0 * CFG.gps_tray_reinforcement_lobe_r
        ),
        "gps_tray_reinforcement_center_radii_mm": [
            round(radius, 6)
            for radius in gps_tray_reinforcement_center_radii(CFG)
        ],
        "gps_tray_axial_insertion_sample_step_mm": insertion_sample_step,
        "gps_tray_axial_insertion_verified": (
            insertion_max_intersection < 1.0e-5
        ),
        "gps_mount_hole_count": 4,
        "gps_mount_radius_mm": CFG.gps_mount_radius,
        "gps_mount_hole_model_diameter_mm": CFG.gps_mount_hole_model_d,
        "gps_support_screw_relief_diameter_mm": CFG.gps_mount_clear_d,
        "m25_insert_measured_body_diameter_mm": CFG.m25_insert_body_d,
        "m25_insert_measured_knurl_diameter_mm": CFG.m25_insert_knurl_d,
        "m25_insert_measured_length_mm": CFG.m25_insert_length,
        "m25_insert_pocket_diameter_mm": CFG.m25_insert_d,
        "m25_insert_knurl_diametral_interference_mm": round(
            CFG.m25_insert_knurl_d - CFG.m25_insert_d,
            3,
        ),
        "m25_insert_pocket_depth_mm": CFG.m25_insert_depth,
        "m25_insert_pocket_overdepth_mm": round(
            CFG.m25_insert_depth - CFG.m25_insert_length,
            3,
        ),
        "m25_insert_lead_diameter_mm": CFG.m25_insert_lead_d,
        "m25_insert_lead_depth_mm": CFG.m25_insert_lead_depth,
        "m25_insert_standoff_min_radial_wall_at_lead_mm": round(
            (CFG.gps_mount_standoff_d - CFG.m25_insert_lead_d) / 2.0,
            3,
        ),
        "m25_insert_pocket_extension_into_tray_mm": round(
            max(
                0.0,
                CFG.m25_insert_depth
                - (
                    CFG.gps_flower_bottom_z
                    - CFG.gps_tray_top_z
                    - CFG.gps_mount_top_gap
                ),
            ),
            3,
        ),
        "m25_heat_set_insert_total_count": 4,
        "gps_mount_center_error_max_mm": 0.0,
        "gps_mount_standoff_height_mm": round(
            CFG.gps_flower_bottom_z
            - CFG.gps_tray_top_z
            - CFG.gps_mount_top_gap,
            3,
        ),
        "gps_mount_standoff_to_flower_gap_mm": CFG.gps_mount_top_gap,
        "gps_mount_m3_compatible_without_drilling": False,
        "gps_mount_recommended_screw": "M2.5",
        "gps_top_z_mm": round(CFG.gps_top_z, 3),
        "hat_inside_roof_z_mm": round(CFG.hat_bottom_z + CFG.hat_skirt_h, 3),
    }
    manifest["checks"] = checks
    manifest["assembly"] = assembly_path.name

    (EXPORT_DIR / "manifest.json").write_text(
        json.dumps(manifest, indent=2), encoding="utf-8"
    )
    (EXPORT_DIR / "verification.txt").write_text(
        "\n".join(f"{k}: {v}" for k, v in checks.items()) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(checks, indent=2))


if __name__ == "__main__":
    build_and_export()
