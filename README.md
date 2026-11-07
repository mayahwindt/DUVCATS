# DUVCATS Control GUI

Control and measurement software for the DUVCATS Deep-UV autocollimator
(University of Arizona senior design, sponsored by Xcimer Energy).

Reads a Basler ace2 a2A4504-18umPRO camera, finds the return spot, and
reports mirror tilt (AZ/EL) in arc-seconds or micro-radians.

## Modes
- **Alignment:** Source/Return positions on a graph of the whole sensor
- **Metrology:** live camera image (ROI) with overlays and statistics

## Setup
1. Install the **Basler pylon Software Suite** from basler.com
2. In a terminal, in this folder:
   ```
   python3 -m venv .venv
   source .venv/bin/activate        # Windows: .venv\Scripts\activate
   python -m pip install -r requirements.txt
   ```

## Run
```
python duv_autocollimator_gui.py                          # GUI
python duv_autocollimator_gui.py --check-camera           # camera test
python duv_autocollimator_gui.py --check-camera --emulator  # test without hardware
```
Close pylon Viewer before running (only one program can use the camera).
If several cameras are connected, choose one by serial number:
```
DUV_CAMERA_SERIAL=40123456 python duv_autocollimator_gui.py
```

## Camera sources
- **Camera:** real Basler over USB 3
- **Emulator:** pylon's built-in emulator, with a simulated return spot
- **Simulator:** synthetic spot, no pylon needed; enables Test Point mode

 
