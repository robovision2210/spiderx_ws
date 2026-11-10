# SpiderX Engineering Console

An interactive, single file explanation of this repository: the robot, the control path, the
ROS 2 graph, the kinematics, the evidence and the roadmap. It is a snapshot of one commit and is
not connected to any robot or simulator.

Open `dist/spiderx_console.html` in a browser. Nothing needs to be installed.

## Rebuild

```bash
python3 tools/engineering_console/build.py
```

Python 3 standard library only. The build reads the URDF, the STL meshes, eight screenshots and
one committed evidence CSV. It imports nothing from the robot packages, runs none of the
repository scripts and changes nothing outside this folder. It stops with an error if a file path
cited by the console does not exist at the current commit.

```bash
node tools/engineering_console/selftest.js
```

Optional. Checks the browser math against numbers documented in `docs/` (joint axis points, the
derived foot tip, the 0.1223 rad crouch figure and both negative inverse kinematics targets).

## Layout

| File | Role |
|---|---|
| `extract.py` | Read only extraction: links, joints, quantised meshes, derived foot tips, recorded playback, images |
| `build.py` | Assembles the page and checks every cited path |
| `src/05_content.js` | All text and evidence labels shown by the console |
| `src/10_robot.js` | URDF forward kinematics and a port of the analytic inverse kinematics and gait phase rules |
| `src/20_gl.js` | Small WebGL viewer for the real meshes, no external library |
| `src/30_ui.js` | The nine stations, the evidence drawer and the image viewer |
| `src/styles.css`, `src/body.html` | Presentation |

## Evidence labels

| Label | Meaning |
|---|---|
| VERIFIED IN CODE | Read in the source at the snapshot commit. Code existing does not prove it runs |
| VERIFIED BY TEST EVIDENCE | Raw evidence files are committed and were inspected |
| REPORTED / NOT RECHECKED | Stated in a project document. Raw logs are not in the repository |
| PLANNED | Roadmap item, not implemented |

The browser math is a re-implementation for explanation. It is not the project's Python code and
does not validate it.
