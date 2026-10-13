PANTHERA TWO-ARM RECORDED PLAYBACK

1. Extract this ZIP. Keep playback.py, scene.xml, meshes/, and joint_data.csv together.
2. In Ubuntu, activate your existing environment:
   source ~/panthera-env/bin/activate
3. Change into this extracted panthera_playback folder.
4. Install the viewing dependencies:
   python -m pip install -r requirements.txt
5. Launch:
   python playback.py

SPACE pauses/resumes. R restarts. Close the window to exit.
Playback stops at the last frame by default. For continuous looping:
   python playback.py --loop
For half-speed playback:
   python playback.py --speed 0.5
For another CSV produced by record_two_arms.py:
   python playback.py --csv /path/to/joint_data.csv
To validate without opening a graphical window:
   python playback.py --check
Windows Python also works using the same pip/playback commands in this folder.
On macOS use mjpython playback.py for the graphical viewer.

WHAT YOU SEE
Blue pedestal = Leader; orange pedestal = Follower. Those are the recorder's
names only: both recorded arms were moved independently. Their bases are placed
0.65 m apart, base-center to base-center, estimated visually from the supplied
photos (rough plausible range 0.50-0.80 m, not a calibrated measurement).
Bases remain parallel and level; their exact relative orientation is unmeasured.
The 29.99-second recording contains 3,000 samples per arm (6,000 CSV rows).
Each arm's six joint-angle columns map by name to its six MuJoCo hinge joints.
The arms share the CSV elapsed-time axis, preserving their host-read offset.
Linear interpolation fills between recorded samples. No angle offsets or sign
changes have been added, and out-of-range samples are not silently clamped.
The CSV is included unchanged.

This is kinematic visual playback: positions are assigned and mj_forward computes
geometry. It does not integrate dynamics, evaluate gravity compensation or contact
forces, or reproduce measured torque. Collisions are disabled for the scene.
There is no robot SDK import and no physical robot communication.
You do not need the arms connected or powered for this viewer.

GRIPPER ANIMATION
The original link6 mesh is split into a fixed housing and two moving finger
assemblies. Finger shapes are preserved. Opposing slide joints approximate the
roughly parallel closing visible in the supplied video; the actual linkage is
not reconstructed. Each finger travels up to 40.5 mm inward from the mesh pose.
Each arm uses its own recorded gripper_q_rad, interpolated on its own timestamps.
The provisional mapping assumes 0 rad closed and 2 rad fully open, based on SDK
limits, NOT measured endpoint calibration. Adjust GRIPPER_CLOSED_RAD,
GRIPPER_OPEN_RAD and FINGER_TRAVEL_M near the top of playback.py if measured.
Angles outside that interval are clipped for finger visualization only; the CSV
is unchanged. This is visual animation, not a grasp/contact physics model.
Replace playback.py, scene.xml AND meshes/ together when upgrading.

DATA LIMITATIONS
Timestamps come from host reads of cached SDK state, not hardware sampling times.
Both tracks are aligned on those timestamps, not hardware synchronized.
A completed file with changing angles supports visual replay, but does not prove
the accuracy of robot calibration or the gravity model. No metadata.json was supplied.

MODEL CONVERSION
Meshes were converted from binary STL to OBJ without decimation. One source STL
has 321,432 triangles, exceeding MuJoCo's STL decoder face limit in the tested
version. URDFs were imported using MuJoCo and combined with prefixed joint names.
Duplicate collision geometry was removed; visual meshes and joint transforms
were retained. The original URDF files are included in source_urdf/ for reference;
they still use the original package mesh paths and are not the entry point.

VALIDATION
Tested with MuJoCo 3.15.0: scene compilation, all 6,000 sample timestamps through
forward kinematics, and an offscreen rendered frame. GUI operation on your computer
has not been tested. If the window cannot open, paste the terminal error.

MuJoCo viewer/API documentation:
https://mujoco.readthedocs.io/en/stable/python.html
