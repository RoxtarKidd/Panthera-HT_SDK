## Quick start: simulation and recordings

### Watch recorded motion in MuJoCo

No physical robots are needed. Run these commands in a terminal with your Python environment activated:

```bash
git clone https://github.com/RoxtarKidd/Panthera-HT_SDK.git
cd Panthera-HT_SDK/panthera_python/scripts/panthera_playback
python -m pip install -r requirements.txt
python playback.py
```

Press **Space** to pause/resume and **R** to restart.

To play another recording, replace `SESSION_FOLDER` with its folder name:

```bash
python playback.py --csv ../recordings/SESSION_FOLDER/joint_data.csv
```

Playback visualizes recorded motion. Gripper motion and base placement are approximate; this is not a grasping physics simulation.

### Find the right script

Paths below are relative to `panthera_python/scripts/`.

| Task | Script or folder |
|---|---|
| MuJoCo playback | [panthera_playback/playback.py](panthera_python/scripts/panthera_playback/playback.py) |
| Recorded data | [recordings/](panthera_python/scripts/recordings/) | (Some recordings aren't as good as others)
| Record both arms independently with gravity compensation | [record_two_arms.py](panthera_python/scripts/record_two_arms.py) |
| Use one physical arm to control the other | [5_teleop_control.py](panthera_python/scripts/5_teleop_control.py) |
| Quest control of one arm | [7_vr_cartesian_control.py](panthera_python/scripts/7_vr_cartesian_control.py) |
| Quest control of both arms | [7_vr_dual_cartesian_control.py](panthera_python/scripts/7_vr_dual_cartesian_control.py) |

Each recording session contains `joint_data.csv` and `metadata.json`.
The current two-arm recorder does not record camera video.

### Physical robot setup

Before running hardware scripts, follow the [Python SDK setup instructions](panthera_python/README.md) and verify your Leader/Follower serial-port configuration. Port mappings can change after reconnecting USB devices.

Run only one robot control program at a time.

Quest control additionally requires [Quest_controller_stream](https://github.com/HighTorque-Robotics/Quest_controller_stream), forwarding controller data to UDP port `5005`.
