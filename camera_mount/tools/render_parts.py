"""Render multiple STL parts with distinct colors."""

import argparse

import vtk


PALETTE = (
    (0.25, 0.28, 0.33),
    (0.95, 0.38, 0.12),
    (0.95, 0.38, 0.12),
    (0.10, 0.55, 0.85),
    (0.10, 0.63, 0.34),
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("png_file")
    parser.add_argument("stl_files", nargs="+")
    parser.add_argument("--view", choices=("iso", "front", "side"), default="iso")
    args = parser.parse_args()

    renderer = vtk.vtkRenderer()
    renderer.SetBackground(0.055, 0.075, 0.105)
    for index, filename in enumerate(args.stl_files):
        reader = vtk.vtkSTLReader()
        reader.SetFileName(filename)
        normals = vtk.vtkPolyDataNormals()
        normals.SetInputConnection(reader.GetOutputPort())
        normals.SetFeatureAngle(40)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(normals.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(*PALETTE[index % len(PALETTE)])
        actor.GetProperty().SetSpecular(0.2)
        actor.GetProperty().SetSpecularPower(20)
        renderer.AddActor(actor)

    window = vtk.vtkRenderWindow()
    window.SetOffScreenRendering(True)
    window.SetSize(1400, 1050)
    window.AddRenderer(renderer)
    bounds = [0.0] * 6
    renderer.ComputeVisiblePropBounds(bounds)
    center = tuple((bounds[i * 2] + bounds[i * 2 + 1]) / 2 for i in range(3))
    size = tuple(bounds[i * 2 + 1] - bounds[i * 2] for i in range(3))
    span = max(size)
    directions = {
        "iso": (1.45, -1.55, 1.15),
        "front": (2.5, 0, 0),
        "side": (0, -2.5, 0),
    }
    direction = directions[args.view]
    camera = renderer.GetActiveCamera()
    camera.SetFocalPoint(*center)
    camera.SetPosition(*(center[i] + direction[i] * span for i in range(3)))
    camera.SetViewUp(0, 0, 1)
    camera.ParallelProjectionOn()
    camera.SetParallelScale(span * 0.58)

    window.Render()
    image_filter = vtk.vtkWindowToImageFilter()
    image_filter.SetInput(window)
    image_filter.SetInputBufferTypeToRGBA()
    image_filter.ReadFrontBufferOff()
    image_filter.Update()
    writer = vtk.vtkPNGWriter()
    writer.SetFileName(args.png_file)
    writer.SetInputConnection(image_filter.GetOutputPort())
    writer.Write()


if __name__ == "__main__":
    main()
