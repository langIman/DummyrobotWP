"""Extract the first named component from a STEP assembly."""

import argparse

import cadquery as cq
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFDoc import XCAFDoc_DocumentTool


def name_of(label: TDF_Label) -> str:
    attribute = TDataStd_Name()
    return attribute.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), attribute) else ""


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("step_file")
    parser.add_argument("component_name")
    parser.add_argument("output_file")
    args = parser.parse_args()

    document = TDocStd_Document(TCollection_ExtendedString("step"))
    reader = STEPCAFControl_Reader()
    reader.ReadFile(args.step_file)
    if not reader.Transfer(document):
        raise RuntimeError("STEP transfer failed")
    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(document.Main())

    roots = TDF_LabelSequence()
    shape_tool.GetFreeShapes(roots)

    def find(label: TDF_Label):
        if name_of(label).split(":", 1)[0].casefold() == args.component_name.casefold():
            return label
        referred = TDF_Label()
        target = label
        if shape_tool.IsComponent_s(label) and shape_tool.GetReferredShape_s(label, referred):
            target = referred
        components = TDF_LabelSequence()
        if shape_tool.IsAssembly_s(target):
            shape_tool.GetComponents_s(target, components, False)
            for index in range(1, components.Length() + 1):
                result = find(components.Value(index))
                if result is not None:
                    return result
        return None

    found = None
    for index in range(1, roots.Length() + 1):
        found = find(roots.Value(index))
        if found is not None:
            break
    if found is None:
        raise ValueError(f"Component not found: {args.component_name}")

    shape = cq.Shape.cast(shape_tool.GetShape_s(found))
    box = shape.BoundingBox()
    center = cq.Vector(
        -(box.xmin + box.xmax) / 2,
        -(box.ymin + box.ymax) / 2,
        -(box.zmin + box.zmax) / 2,
    )
    centered = shape.translate(center)
    cq.exporters.export(centered, args.output_file, tolerance=0.02, angularTolerance=0.1)
    print(
        f"{name_of(found)} size=({box.xlen:.3f}, {box.ylen:.3f}, {box.zlen:.3f}) "
        f"center=({-center.x:.3f}, {-center.y:.3f}, {-center.z:.3f})"
    )


if __name__ == "__main__":
    main()
