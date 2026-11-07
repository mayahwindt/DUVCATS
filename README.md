# DUVCATS Control GUI

Control and measurement software for the DUVCATS Deep-UV autocollimator
(University of Arizona senior design, sponsored by Xcimer Energy).

Reads a Basler ace2 a2A4504-18umPRO camera, finds the return spot, and
reports mirror tilt (AZ/EL) in arc-seconds or micro-radians.

## Modes
- **Alignment:** Source/Return positions on a graph of the whole sensor
- **Metrology:** live camera image (ROI) with overlays and statistics

## Setup (macOS)
1. Install **Python 3.12** from python.org (Downloads → macOS installer).
   Do NOT use the Mac's built-in Python 3.9 — its old Tk draws the GUI
   incorrectly. If Anaconda is installed, run `conda deactivate` first.
2. Install the **Basler pylon Software Suite** from basler.com
3. In a terminal, in this folder:
   ```
   /Library/Frameworks/Python.framework/Versions/3.12/bin/python3.12 -m venv .venv
   source .venv/bin/activate
   python -m pip install -r requirements.txt
   ```
4. Check: `python -c "import tkinter; print(tkinter.TkVersion)"` should print 8.6 or higher.
5. In VS Code: Cmd+Shift+P → Python: Select Interpreter → choose `.venv`

Each time you open a new terminal: `source .venv/bin/activate`

## Run
```
python maincode.py                                # GUI
python maincode.py --check-camera                 # camera test
python maincode.py --check-camera --emulator      # test without hardware
```
Close pylon Viewer before running (only one program can use the camera).

If several cameras are connected, choose one by serial number:
```
DUV_CAMERA_SERIAL=40123456 python maincode.py
```

## Camera sources
- **Camera:** real Basler over USB 3
- **Emulator:** pylon's built-in emulator, with a simulated return spot
- **Simulator:** synthetic spot, no pylon needed; enables Test Point mode

## macOS permissions
The Screenshot and Record Video buttons need screen-recording permission:
System Settings → Privacy & Security → Screen Recording → turn on
Visual Studio Code (or Terminal), then restart it.

## Team
DUVCATS 
Software Team: Mayah Windt and Jonathan Galhandro
