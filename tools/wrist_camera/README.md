# Wrist camera preview

Local preview and UVC mode checker for the DECXIN wrist camera. It defaults to
`http://127.0.0.1:8766/` so it can run alongside the Dummy Windows bridge on
port 8765.

```powershell
python -m pip install -r requirements.txt
python camera_stream.py
```

Optional environment variables:

- `WRIST_CAMERA_HOST`: listen address, default `127.0.0.1`
- `WRIST_CAMERA_PORT`: HTTP port, default `8766`

The page can change the requested resolution and frame rate, show measured FPS,
and release/reopen the camera device when preview is stopped or resumed.
