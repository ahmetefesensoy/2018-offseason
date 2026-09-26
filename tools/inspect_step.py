"""Print non-mutating STEP assembly bounds using FreeCADCmd.

Usage: FreeCADCmd tools/inspect_step.py "Assembly 1.step"
"""

import sys

import FreeCAD as App
import Import


def volume_of_bounds(obj):
    bounds = obj.Shape.BoundBox
    return bounds.XLength * bounds.YLength * bounds.ZLength


def has_valid_bounds(obj):
    bounds = obj.Shape.BoundBox
    extrema = (
        bounds.XMin,
        bounds.XMax,
        bounds.YMin,
        bounds.YMax,
        bounds.ZMin,
        bounds.ZMax,
    )
    return all(-1e90 < value < 1e90 for value in extrema)


if len(sys.argv) < 2:
    raise SystemExit("usage: FreeCADCmd inspect_step.py <assembly.step>")

Import.open(sys.argv[-1])
document = App.ActiveDocument
shapes = [
    obj
    for obj in document.Objects
    if hasattr(obj, "Shape")
    and not obj.Shape.isNull()
    and has_valid_bounds(obj)
]

print(f"objects={len(document.Objects)}")
print(f"shape_objects={len(shapes)}")

if shapes:
    x_values = [obj.Shape.BoundBox.XMin for obj in shapes] + [
        obj.Shape.BoundBox.XMax for obj in shapes
    ]
    y_values = [obj.Shape.BoundBox.YMin for obj in shapes] + [
        obj.Shape.BoundBox.YMax for obj in shapes
    ]
    z_values = [obj.Shape.BoundBox.ZMin for obj in shapes] + [
        obj.Shape.BoundBox.ZMax for obj in shapes
    ]
    print(
        "assembly_bounds_mm="
        f"x[{min(x_values):.2f},{max(x_values):.2f}] "
        f"y[{min(y_values):.2f},{max(y_values):.2f}] "
        f"z[{min(z_values):.2f},{max(z_values):.2f}]"
    )

for obj in sorted(shapes, key=volume_of_bounds, reverse=True)[:30]:
    bounds = obj.Shape.BoundBox
    print(
        f"{obj.Label}: "
        f"size=({bounds.XLength:.2f},{bounds.YLength:.2f},{bounds.ZLength:.2f})mm "
        f"center=({bounds.Center.x:.2f},{bounds.Center.y:.2f},{bounds.Center.z:.2f})mm "
        f"placement=({obj.Placement.Base.x:.2f},{obj.Placement.Base.y:.2f},"
        f"{obj.Placement.Base.z:.2f})mm"
    )
