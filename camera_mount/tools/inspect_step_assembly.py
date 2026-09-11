"""Print an XCAF STEP assembly tree with instance bounding boxes."""

import argparse

import cadquery as cq
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFDoc import XCAFDoc_DocumentTool


def label_name(label: TDF_Label) -> str:
    attribute = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), attribute):
        return attribute.Get().ToExtString()
    return "<unnamed>"


def bounding_box(shape) -> str:
    if shape.IsNull():
        return "null"
    box = cq.Shape.cast(shape).BoundingBox()
    return (
        f"size=({box.xlen:.2f}, {box.ylen:.2f}, {box.zlen:.2f}) "
        f"center=({(box.xmin + box.xmax) / 2:.2f}, "
        f"{(box.ymin + box.ymax) / 2:.2f}, {(box.zmin + box.zmax) / 2:.2f})"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("step_file")
    parser.add_argument("--match", default="")
    parser.add_argument("--depth", type=int, default=4)
    args = parser.parse_args()

    document = TDocStd_Document(TCollection_ExtendedString("step"))
    reader = STEPCAFControl_Reader()
    status = reader.ReadFile(args.step_file)
    if "RetDone" not in str(status) or not reader.Transfer(document):
        raise RuntimeError(f"Could not import STEP file: {status}")

    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(document.Main())
    roots = TDF_LabelSequence()
    shape_tool.GetFreeShapes(roots)

    def walk(label: TDF_Label, depth: int, force: bool = False) -> bool:
        name = label_name(label)
        referred = TDF_Label()
        target = label
        if shape_tool.IsComponent_s(label) and shape_tool.GetReferredShape_s(label, referred):
            target = referred
        children = TDF_LabelSequence()
        if shape_tool.IsAssembly_s(target):
            shape_tool.GetComponents_s(target, children, False)

        child_matches = []
        if depth < args.depth:
            for index in range(1, children.Length() + 1):
                child_matches.append(walk(children.Value(index), depth + 1, force or args.match.lower() in name.lower()))
        matched = force or not args.match or args.match.lower() in name.lower() or any(child_matches)
        if matched:
            shape = shape_tool.GetShape_s(label)
            print(f"{'  ' * depth}{name}: {bounding_box(shape)}")
        return matched

    for index in range(1, roots.Length() + 1):
        walk(roots.Value(index), 0)


if __name__ == "__main__":
    main()
