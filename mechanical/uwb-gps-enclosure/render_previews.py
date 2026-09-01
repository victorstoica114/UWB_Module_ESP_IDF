"""Render off-screen PNG previews of the generated enclosure."""

from __future__ import annotations

import math
import os
from pathlib import Path
from typing import Iterable, Tuple

os.environ.setdefault("XDG_CACHE_HOME", str(Path(__file__).resolve().parent / ".cache"))

import cadquery as cq
from vtkmodules.vtkFiltersCore import vtkPolyDataNormals
from vtkmodules.vtkIOGeometry import vtkSTLReader
from vtkmodules.vtkIOImage import vtkPNGWriter
from vtkmodules.vtkRenderingCore import (
    vtkActor,
    vtkCamera,
    vtkLight,
    vtkPolyDataMapper,
    vtkRenderer,
    vtkRenderWindow,
)
from vtkmodules.vtkRenderingOpenGL2 import vtkOpenGLRenderer  # noqa: F401
from vtkmodules.vtkRenderingOpenGL2 import vtkOpenGLRenderWindow  # noqa: F401
from vtkmodules.vtkRenderingOpenGL2 import vtkOpenGLPolyDataMapper  # noqa: F401
from vtkmodules.vtkRenderingOpenGL2 import vtkOpenGLActor  # noqa: F401
from vtkmodules.vtkRenderingOpenGL2 import vtkOpenGLCamera  # noqa: F401
from vtkmodules.vtkRenderingOpenGL2 import vtkOpenGLLight  # noqa: F401
from vtkmodules.vtkRenderingCore import vtkWindowToImageFilter

import generate_enclosure as gen


HERE = Path(__file__).resolve().parent
EXPORTS = HERE / "exports"
PREVIEWS = HERE / "previews"


def ensure_reference_stls() -> None:
    PREVIEWS.mkdir(parents=True, exist_ok=True)
    refs = gen.build_reference_shapes()
    collar_cutaway = gen.build_upper_collar().cut(
        gen.box_xyz(140.0, 70.0, 82.0, 0.0, -35.0, -1.0)
    )
    tripod_base_cutaway = gen.build_tripod_base().cut(
        gen.box_xyz(130.0, 64.0, 10.0, 0.0, -32.0, -1.0)
    )
    neck_ro = gen.CFG.collar_neck_outer_d / 2.0
    cap_bosses = None
    for angle in gen.cap_screw_angles():
        a = math.radians(angle)
        ux, uy = math.cos(a), math.sin(a)
        boss = gen.cylinder_between(
            gen.CFG.m3_boss_d / 2.0,
            (
                (neck_ro + 0.3) * ux,
                (neck_ro + 0.3) * uy,
                gen.CFG.hat_bottom_z + 8.0 - gen.CFG.collar_start_z,
            ),
            (-ux, -uy, 0.0),
            gen.CFG.m3_insert_depth + 3.0,
        ).intersect(gen.cylinder_z(neck_ro, gen.CFG.collar_h))
        cap_bosses = boss if cap_bosses is None else cap_bosses.union(boss)
    uwb_height = gen.CFG.uwb_antenna_height_z
    axis = gen.cylinder_z(
        0.75,
        uwb_height,
        0.0,
    ).union(
        cq.Workplane("XY")
        .sphere(2.2)
        .translate((0.0, 0.0, uwb_height))
    )
    gps_axis_z0 = gen.CFG.gps_tray_top_z - 3.5
    gps_axis_h = gen.CFG.gps_top_z - gps_axis_z0 + 0.5
    gps_mount_axes = None
    for angle in (0.0, 90.0, 180.0, 270.0):
        a = math.radians(angle)
        x = gen.CFG.gps_mount_radius * math.cos(a)
        y = gen.CFG.gps_mount_radius * math.sin(a)
        mount_axis = gen.cylinder_z(0.70, gps_axis_h, gps_axis_z0).translate(
            (x, y, 0.0)
        )
        gps_mount_axes = (
            mount_axis
            if gps_mount_axes is None
            else gps_mount_axes.union(mount_axis)
        )
    extras = {
        "REF_placa_UWB.stl": refs["pcb"],
        "REF_antena_GPS.stl": refs["gps"],
        "REF_acumulator.stl": refs["battery"],
        "REF_USB_C_mama.stl": gen.build_usb_placeholder(),
        "REF_intrerupator_DPDT.stl": gen.build_switch_placeholder(),
        "REF_garda_mufa_SMA_D15.stl": gen.build_sma_mating_placeholder(),
        "REF_baza_trepied_sectionata.stl": tripod_base_cutaway.val(),
        "REF_axa_UWB.stl": axis.val(),
        "REF_axe_prindere_GPS.stl": gps_mount_axes.val(),
        "REF_guler_sectionat_FDM.stl": collar_cutaway.val(),
        "REF_bosaje_capac.stl": cap_bosses.val(),
    }
    for filename, shape in extras.items():
        cq.exporters.export(
            shape,
            str(PREVIEWS / filename),
            tolerance=0.10,
            angularTolerance=0.15,
        )


def stl_actor(
    path: Path,
    color: Tuple[float, float, float],
    opacity: float = 1.0,
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> vtkActor:
    reader = vtkSTLReader()
    reader.SetFileName(str(path))

    normals = vtkPolyDataNormals()
    normals.SetInputConnection(reader.GetOutputPort())
    normals.ConsistencyOn()
    normals.AutoOrientNormalsOn()
    normals.SplittingOn()
    normals.SetFeatureAngle(38.0)

    mapper = vtkPolyDataMapper()
    mapper.SetInputConnection(normals.GetOutputPort())

    actor = vtkActor()
    actor.SetMapper(mapper)
    actor.SetPosition(*position)
    actor.GetProperty().SetColor(*color)
    actor.GetProperty().SetOpacity(opacity)
    actor.GetProperty().SetInterpolationToPhong()
    actor.GetProperty().SetSpecular(0.18)
    actor.GetProperty().SetSpecularPower(18.0)
    return actor


def render(
    output: Path,
    actors: Iterable[vtkActor],
    camera_position: Tuple[float, float, float],
    focal_point: Tuple[float, float, float],
    size: Tuple[int, int] = (1200, 1000),
    view_up: Tuple[float, float, float] = (0.0, 0.0, 1.0),
) -> None:
    renderer = vtkRenderer()
    renderer.SetBackground(0.95, 0.97, 0.99)
    renderer.SetBackground2(0.74, 0.81, 0.88)
    renderer.GradientBackgroundOn()
    renderer.SetUseDepthPeeling(True)
    renderer.SetMaximumNumberOfPeels(100)
    renderer.SetOcclusionRatio(0.05)

    for actor in actors:
        renderer.AddActor(actor)

    camera = vtkCamera()
    camera.SetPosition(*camera_position)
    camera.SetFocalPoint(*focal_point)
    camera.SetViewUp(*view_up)
    camera.SetViewAngle(28.0)
    renderer.SetActiveCamera(camera)
    renderer.ResetCameraClippingRange()

    light = vtkLight()
    light.SetPosition(camera_position[0], camera_position[1], camera_position[2] + 100)
    light.SetFocalPoint(*focal_point)
    light.SetIntensity(0.9)
    renderer.AddLight(light)

    fill = vtkLight()
    fill.SetPosition(-220.0, 180.0, 220.0)
    fill.SetFocalPoint(*focal_point)
    fill.SetIntensity(0.45)
    renderer.AddLight(fill)

    window = vtkRenderWindow()
    window.SetOffScreenRendering(1)
    window.SetSize(*size)
    window.SetMultiSamples(8)
    window.AddRenderer(renderer)
    window.Render()

    capture = vtkWindowToImageFilter()
    capture.SetInput(window)
    capture.SetScale(1)
    capture.SetInputBufferTypeToRGBA()
    capture.ReadFrontBufferOff()
    capture.Update()

    writer = vtkPNGWriter()
    writer.SetFileName(str(output))
    writer.SetInputConnection(capture.GetOutputPort())
    writer.Write()
    window.Finalize()


def main() -> None:
    ensure_reference_stls()

    blue = (0.19, 0.43, 0.68)
    dark_blue = (0.12, 0.30, 0.53)
    orange = (0.94, 0.48, 0.12)
    yellow = (0.95, 0.72, 0.14)
    green = (0.12, 0.58, 0.28)
    purple = (0.47, 0.20, 0.67)
    black = (0.07, 0.07, 0.08)
    metal = (0.58, 0.61, 0.65)

    base_pos = (0.0, 0.0, -gen.CFG.base_h)
    collar_pos = (0.0, 0.0, gen.CFG.collar_start_z)
    tray_pos = (0.0, 0.0, gen.CFG.gps_tray_top_z - 3.0)
    hat_pos = (0.0, 0.0, gen.CFG.hat_bottom_z)
    retainer_pos = (
        0.0,
        gen.CFG.battery_tray_center_y,
        gen.CFG.battery_retainer_z,
    )

    exterior = [
        stl_actor(EXPORTS / "01_baza_tinta.stl", yellow, position=base_pos),
        stl_actor(EXPORTS / "02_corp_inferior.stl", blue),
        stl_actor(EXPORTS / "05_guler_superior.stl", blue, position=collar_pos),
        stl_actor(EXPORTS / "07_capac_palarie.stl", dark_blue, position=hat_pos),
        stl_actor(PREVIEWS / "REF_USB_C_mama.stl", black),
        stl_actor(PREVIEWS / "REF_intrerupator_DPDT.stl", metal),
    ]
    render(
        PREVIEWS / "01_ansamblu_exterior.png",
        exterior,
        (250.0, -330.0, 225.0),
        (0.0, 0.0, 86.0),
    )

    transparent = [
        stl_actor(EXPORTS / "01_baza_tinta.stl", yellow, position=base_pos),
        stl_actor(EXPORTS / "02_corp_inferior.stl", blue, opacity=0.18),
        stl_actor(
            EXPORTS / "05_guler_superior.stl", blue, opacity=0.18, position=collar_pos
        ),
        stl_actor(
            EXPORTS / "07_capac_palarie.stl",
            dark_blue,
            opacity=0.15,
            position=hat_pos,
        ),
        stl_actor(EXPORTS / "03_sasiu_pcb_acumulator.stl", orange),
        stl_actor(
            EXPORTS / "04_retainer_acumulator.stl",
            orange,
            position=retainer_pos,
        ),
        stl_actor(
            EXPORTS / "06_suport_antena_gps.stl", yellow, position=tray_pos
        ),
        stl_actor(PREVIEWS / "REF_placa_UWB.stl", green),
        stl_actor(PREVIEWS / "REF_antena_GPS.stl", green),
        stl_actor(PREVIEWS / "REF_acumulator.stl", purple),
        stl_actor(PREVIEWS / "REF_USB_C_mama.stl", black),
        stl_actor(PREVIEWS / "REF_intrerupator_DPDT.stl", metal),
    ]
    render(
        PREVIEWS / "02_ansamblu_transparent.png",
        transparent,
        (245.0, -330.0, 215.0),
        (0.0, 0.0, 88.0),
    )

    exploded = [
        stl_actor(EXPORTS / "01_baza_tinta.stl", yellow, position=(0, 0, -20)),
        stl_actor(EXPORTS / "02_corp_inferior.stl", blue, position=(0, 0, 0)),
        stl_actor(
            EXPORTS / "03_sasiu_pcb_acumulator.stl", orange, position=(-105, 0, 5)
        ),
        stl_actor(
            EXPORTS / "04_retainer_acumulator.stl", orange, position=(-105, 0, 125)
        ),
        stl_actor(
            EXPORTS / "05_guler_superior.stl", blue, position=(0, 0, 155)
        ),
        stl_actor(
            EXPORTS / "06_suport_antena_gps.stl", yellow, position=(105, 0, 90)
        ),
        stl_actor(
            EXPORTS / "07_capac_palarie.stl", dark_blue, position=(0, 0, 245)
        ),
    ]
    render(
        PREVIEWS / "03_piese_explodate.png",
        exploded,
        (360.0, -500.0, 300.0),
        (0.0, 0.0, 115.0),
        size=(1400, 1100),
    )

    alignment = [
        stl_actor(EXPORTS / "01_baza_tinta.stl", yellow, position=base_pos),
        stl_actor(EXPORTS / "02_corp_inferior.stl", blue, opacity=0.10),
        stl_actor(EXPORTS / "03_sasiu_pcb_acumulator.stl", orange, opacity=0.38),
        stl_actor(PREVIEWS / "REF_placa_UWB.stl", green),
        stl_actor(PREVIEWS / "REF_axa_UWB.stl", (0.92, 0.05, 0.04)),
    ]
    render(
        PREVIEWS / "04_aliniere_axa_uwb.png",
        alignment,
        (0.0, -340.0, 115.0),
        (0.0, 0.0, 62.0),
        size=(1000, 1100),
    )

    sma_detail = [
        stl_actor(EXPORTS / "03_sasiu_pcb_acumulator.stl", orange),
        stl_actor(PREVIEWS / "REF_placa_UWB.stl", green, opacity=0.38),
        stl_actor(PREVIEWS / "REF_acumulator.stl", purple, opacity=0.55),
        stl_actor(PREVIEWS / "REF_garda_mufa_SMA_D15.stl", metal, opacity=0.72),
        stl_actor(
            EXPORTS / "04_retainer_acumulator.stl",
            orange,
            position=retainer_pos,
        ),
    ]
    render(
        PREVIEWS / "05_detaliu_sma_acumulator_coborat.png",
        sma_detail,
        (-175.0, -245.0, 95.0),
        (-12.0, 8.0, 48.0),
        size=(1200, 1000),
    )

    target_top = [
        stl_actor(EXPORTS / "01_baza_tinta.stl", yellow),
    ]
    render(
        PREVIEWS / "06_baza_tinta_diametru_complet.png",
        target_top,
        (120.0, -190.0, 210.0),
        (0.0, 0.0, 2.0),
        size=(1100, 900),
    )

    render(
        PREVIEWS / "11_baza_tinta_decupata_vedere_sus.png",
        target_top,
        (0.0, 0.0, 270.0),
        (0.0, 0.0, 0.0),
        size=(1100, 1100),
        view_up=(0.0, 1.0, 0.0),
    )

    tripod_base = [
        stl_actor(EXPORTS / "08_baza_trepied.stl", yellow),
    ]
    render(
        PREVIEWS / "16_baza_trepied_filet_3_8.png",
        tripod_base,
        (120.0, -190.0, -170.0),
        (0.0, 0.0, 4.0),
        size=(1100, 900),
    )
    render(
        PREVIEWS / "17_baza_trepied_fata_magneti.png",
        tripod_base,
        (120.0, -190.0, 210.0),
        (0.0, 0.0, 4.0),
        size=(1100, 900),
    )
    render(
        PREVIEWS / "18_filet_trepied_sectionat.png",
        [
            stl_actor(
                PREVIEWS / "REF_baza_trepied_sectionata.stl",
                yellow,
            )
        ],
        (42.0, -76.0, 30.0),
        (0.0, 0.0, 4.0),
        size=(1100, 900),
    )

    lateral_controls = [
        stl_actor(EXPORTS / "02_corp_inferior.stl", blue),
        stl_actor(PREVIEWS / "REF_USB_C_mama.stl", black),
        stl_actor(PREVIEWS / "REF_intrerupator_DPDT.stl", metal),
    ]
    render(
        PREVIEWS / "07_nisa_laterala_comenzi.png",
        lateral_controls,
        (0.0, -260.0, 62.0),
        (0.0, -55.0, 32.0),
        size=(1200, 1000),
    )

    gps_mount_alignment = [
        stl_actor(
            EXPORTS / "06_suport_antena_gps.stl",
            yellow,
            position=tray_pos,
        ),
        stl_actor(PREVIEWS / "REF_antena_GPS.stl", green, opacity=0.38),
        stl_actor(PREVIEWS / "REF_axe_prindere_GPS.stl", (0.92, 0.05, 0.04)),
    ]
    render(
        PREVIEWS / "08_aliniere_gauri_antena_gps.png",
        gps_mount_alignment,
        (105.0, -150.0, 330.0),
        (0.0, 0.0, gen.CFG.gps_flower_bottom_z),
        size=(1100, 1000),
    )

    chassis_mount_detail = [
        stl_actor(EXPORTS / "02_corp_inferior.stl", blue, opacity=0.14),
        stl_actor(EXPORTS / "03_sasiu_pcb_acumulator.stl", orange),
        stl_actor(PREVIEWS / "REF_placa_UWB.stl", green, opacity=0.28),
        stl_actor(PREVIEWS / "REF_acumulator.stl", purple, opacity=0.30),
    ]
    render(
        PREVIEWS / "09_talpi_sasiu_ranforsate.png",
        chassis_mount_detail,
        (-145.0, -205.0, 75.0),
        (-8.0, 10.0, 24.0),
        size=(1200, 1000),
    )

    chassis_mount_right_detail = [
        stl_actor(EXPORTS / "02_corp_inferior.stl", blue, opacity=0.10),
        stl_actor(EXPORTS / "03_sasiu_pcb_acumulator.stl", orange),
    ]
    render(
        PREVIEWS / "10_talpa_dreapta_sasiu_ranforsata.png",
        chassis_mount_right_detail,
        (150.0, -185.0, 62.0),
        (20.0, 9.0, 22.0),
        size=(1200, 1000),
    )

    chassis_three_point_detail = [
        stl_actor(EXPORTS / "02_corp_inferior.stl", blue, opacity=0.10),
        stl_actor(EXPORTS / "03_sasiu_pcb_acumulator.stl", orange),
        stl_actor(PREVIEWS / "REF_acumulator.stl", purple, opacity=0.25),
    ]
    render(
        PREVIEWS / "12_prinderi_sasiu_trei_puncte.png",
        chassis_three_point_detail,
        (135.0, 210.0, 82.0),
        (-10.0, 31.0, 23.0),
        size=(1200, 1000),
    )

    collar_fdm_detail = [
        stl_actor(PREVIEWS / "REF_guler_sectionat_FDM.stl", blue),
    ]
    render(
        PREVIEWS / "13_racordari_interioare_fdm_45deg.png",
        collar_fdm_detail,
        (92.0, -185.0, 36.0),
        (0.0, 0.0, 38.0),
        size=(1200, 1000),
    )

    gps_tray_insertion = [
        stl_actor(
            EXPORTS / "05_guler_superior.stl",
            blue,
            opacity=0.12,
            position=collar_pos,
        ),
        stl_actor(
            EXPORTS / "06_suport_antena_gps.stl",
            yellow,
            position=(0.0, 0.0, gen.CFG.collar_start_z + 68.0),
        ),
        stl_actor(
            PREVIEWS / "REF_bosaje_capac.stl",
            metal,
            opacity=0.72,
            position=collar_pos,
        ),
    ]
    render(
        PREVIEWS / "14_trecere_suport_gps_printre_bosaje.png",
        gps_tray_insertion,
        (135.0, -205.0, 265.0),
        (0.0, 0.0, gen.CFG.collar_start_z + 69.0),
        size=(1200, 1000),
    )

    reinforced_gps_tray = [
        stl_actor(EXPORTS / "06_suport_antena_gps.stl", yellow),
    ]
    render(
        PREVIEWS / "15_ranforsari_decupaje_suport_gps.png",
        reinforced_gps_tray,
        (0.0, 0.0, 250.0),
        (0.0, 0.0, 0.0),
        size=(1100, 1100),
        view_up=(0.0, 1.0, 0.0),
    )
    print("Rendered previews in", PREVIEWS)


if __name__ == "__main__":
    main()
