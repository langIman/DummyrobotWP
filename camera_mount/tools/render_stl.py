"""Render an STL to a transparent PNG using VTK offscreen rendering."""

import argparse

import vtk


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("stl_file")
    parser.add_argument("png_file")
    parser.add_argument("--view", choices=("iso", "front", "side", "top"), default="iso")
    parser.add_argument("--width", type=int, default=1200)
    parser.add_argument("--height", type=int, default=900)
    args = parser.parse_args()

    reader = vtk.vtkSTLReader()
    reader.SetFileName(args.stl_file)
    normals = vtk.vtkPolyDataNormals()
    normals.SetInputConnection(reader.GetOutputPort())
    normals.SetFeatureAngle(40)

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(normals.GetOutputPort())
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(0.78, 0.82, 0.88)
    actor.GetProperty().SetSpecular(0.25)
    actor.GetProperty().SetSpecularPower(25)

    renderer = vtk.vtkRenderer()
    renderer.SetBackground(0.055, 0.075, 0.105)
    renderer.AddActor(actor)

    window = vtk.vtkRenderWindow()
    window.SetOffScreenRendering(True)
    window.SetSize(args.width, args.height)
    window.AddRenderer(renderer)

    reader.Update()
    bounds = reader.GetOutput().GetBounds()
    center = tuple((bounds[i * 2] + bounds[i * 2 + 1]) / 2 for i in range(3))
    size = tuple(bounds[i * 2 + 1] - bounds[i * 2] for i in range(3))
    span = max(size)
    directions = {
        "iso": (1.45, -1.55, 1.2),
        "front": (0, -2.5, 0),
        "side": (2.5, 0, 0),
        "top": (0, 0, 2.5),
    }
    direction = directions[args.view]
    camera = renderer.GetActiveCamera()
    camera.SetFocalPoint(*center)
    camera.SetPosition(*(center[i] + direction[i] * span for i in range(3)))
    camera.SetViewUp(0, 0, 1 if args.view != "top" else 0)
    if args.view == "top":
        camera.SetViewUp(0, 1, 0)
    camera.ParallelProjectionOn()
    camera.SetParallelScale(span * 0.62)

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
