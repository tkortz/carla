Project Information
===============

This repository was used for the paper ["The Price of Schedulability in Multi-ObjectTracking: The History-vs.-Accuracy Trade-Off"](https://www.cs.unc.edu/~tamert/papers/isorc20.pdf), presented at ISORC 2020.

Other relevant repositories:

* [Tracking-by-Detection](TODO)

Some relevant dependencies (see [CARLA build instructions][buildlinuxlink] for more):

* Ubuntu 16.04
* Unreal Engine 4.22
* Python 3.5
* A semi-powerful NVIDIA GPU
* A lot of disk space (30-50GB recommended)

In order to reproduce the results in our paper, the following steps are necessary:
1. Record the scenarios, or use our `.rec` files.
2. Replay each scenario in the CARLA client to generate the ground-truth vehicle/pedestrian detections and RGB, depth, and semantic-segmentation images.  Note that this step takes a long time.
3. Post-process the images and detections from CARLA.
4. (Optional) Process images to detect vehicles/pedestrians.
5. Use our Tracking-by-Detection to track vehicles/pedestrians in each scenario for each history probability-mass function.
6. Process the results to compute tracking metrics.

We now walk through in each step in detail.  These instructions assume the repository root directory is `$CARLA_DIR`.

## 1. Record scenarios

If you want to use our recorded scenario files, skip to [Step (g) Locating the recordings](#g\)-Locating-the-recordings).  If, however, you plan to record the scenarios yourself, read on!

### a) Setup

First, follow the steps to [build CARLA for Linux][buildlinuxlink].

Then, clone our fork of the [CARLA scenario runner repository](https://github.com/Yougmark/scenario_runner/tree/isorc20) and check out the `isorc20` branch.

Finally, you'll need to do some setup:

* [Installing prerequisites](https://github.com/Yougmark/scenario_runner/blob/isorc20/Docs/getting_started.md#installing-prerequisites)
* [Pointing to your CARLA install and running an example scenario](https://github.com/Yougmark/scenario_runner/blob/isorc20/Docs/getting_started.md#running-the-follow-vehicle-example)

### b) Run the server

Now, you're ready to run and record the scenarios for yourself.  First, launch the CARLA server.  With our setup, we use the following commands:

```
cd $CARLA_DIR
./Dist/CARLA_Shipping_0.9.6-23-g89e329b/LinuxNoEditor/CarlaUE4.sh
```

### c) Run the scenario

The following table lists the scenarios we evaluated in our paper, as well as the towns they appear in and the name of the scenario in the `carla-scenario-runner` repository (`$SCENARIO_NAME` below):

| Our Name   	| Town   	| Name in carla-scenario-runner     	|
|------------	|--------	|-----------------------------------	|
| Scenario 1 	| Town 1 	| VehicleTurningRight_1             	|
| Scenario 2 	| Town 3 	| SignalizedJunctionRightTurn_1     	|
| Scenario 3 	| Town 3 	| OppositeVehicleRunningRedLight032 	|
| Scenario 4 	| Town 4 	| SignalizedJunctionLeftTurn_3      	|

To run a scenario, open a second terminal window and navigate to the root directory of this repo and run `scenario_runner.py`:

```
cd $CARLA_SCENARIO_RUNNER_DIR
python3 scenario_runner.py --scenario $SCENARIO_NAME
```

### d) Add pedestrians

From a third terminal window, navigate to the CARLA Python API directory and use `spawn_npc.py` to add pedestrians (you don't need to add vehicles, as they are part of the scenario configuration).

```
cd $CARLA_DIR/PythonAPI/examples
python3 spawn_npc.py -n 0 -w 400
```

Note that there will be many collisions in positions for pedestrians, but trying to spawn 400 should successfully spawn at least 250, some of which should appear in the scenario.

### e) Control the ego vehicle

From a fourth terminal window, navigate to the scenario-runner directory again and run `manual_control.py` to control the "ego vehicle":

```
cd $CARLA_SCENARIO_RUNNER_DIR
python3 manual_control.py
```

Before moving on to recording the scenario, play around with it a few times.  You can cancel the simulation at any time by hitting `escape` in the client window.  (You might need to kill the scenario_runner process.)  Good luck!

### f) Record the scenario

While controlling the ego vehicle, press `ctrl+r` in the client window to begin recording.  When you are finished, either press `escape` to close the client, or press `ctrl+r` again to end the recording.

The ID of the ego vehicle will be printed to the terminal in which you ran `manual_control.py`.  Make note of this ID, as you'll need it to replay the recorded scenario in the next step.  In the example below, the ID is 2762.

```
Hero ID: 2762
```

### g) Locating the recordings

The recordings should be saved in `~/.config/Epic/CarlaUE4/Saved/`, with the hero agent ID as the filename.  For example, using our pre-recorded scenario files you should have:
```
~/.config/Epic/CarlaUE4/Saved/2762.rec
~/.config/Epic/CarlaUE4/Saved/4853.rec
~/.config/Epic/CarlaUE4/Saved/6591.rec
~/.config/Epic/CarlaUE4/Saved/7856.rec
```

Note the mapping:

* Scenario 1: 6591.rec
* Scenario 2: 4853.rec
* Scenario 3: 2762.rec
* Scenario 4: 7856.rec

If you did not record your own scenarios, make sure to copy our recordings to this directory.  They can be found in `$CARLA_DIR/PythonAPI/examples/recorded_scenarios/`.

If instead you did record your own scenarios, be sure to update lines 134-141 of `$CARLA_DIR/PythonAPI/examples/manual_control_synchronous.py` accordingy.

## 2. Replay each scenario to generate ground-truth data and images

Again, first launch the CARLA server (you can leave it running if you just recorded the scenario(s)):

```
cd $CARLA_DIR
./Dist/CARLA_Shipping_0.9.6-23-g89e329b/LinuxNoEditor/CarlaUE4.sh
```

Next, set up the scenario you want to replay using `config.py`, where `$TOWN` comes from the table in [Step 1 (c) above](#c\)-Run-the-scenario).

```
cd $CARLA_DIR/PythonAPI/util
python3 config.py -m $TOWN --w ClearNoon
```

Then, create the folder `$CARLA_DIR/PythonAPI/examples/isorc20/{SCENARIO_NAME}/` for each scenario you plan to run (e.g., `scenario_1`).  Once the CARLA client has started, navigate to the PythonAPI-examples directory and run the following Python script to enable the synchronous replay:

```
cd $CARLA_DIR/PythonAPI/examples
python3 manual_control_synchronous.py
```

This script will spawn a vehicle (which we'll ignore), and is ready by default to replay scenario 1.  To change scenarios, change line 132 of `manual_control_synchronous.py` to set `SCENARIO_NAME` accordingly.

Once the vehicle spawns, press `ctrl+p` once to replay the scenario.  This causes the following to occur:

* Enables the display of ground-truth bounding boxes for both vehicles and pedestrians.
* Writes the corresponding ground-truth positions of vehicles and pedestrians to files in the directory `$CARLA_DIR/PythonAPI/examples/isorc20/{SCENARIO_NAME}/`.
* Outputs one RGB, one depth, and one semantic segmentation image per frame from the camera on the front of the ego vehicle to per-image-type directories in `$CARLA_DIR/PythonAPI/examples/isorc20/{SCENARIO_NAME}/`.

Note that this step is very slow, as it requires running the server synchronously and outputting a lot of data per frame.  If you're just testing the workflow, you can hit `escape` after a couple of frames to close the client.

```
TODO: mention what to do when it's done
```

## 3. Post-process CARLA output

The images and ground-truth detections outputted by CARLA need to be post-processed.

### a) Remove images from before the replay started

The sensors (e.g., RGB camera) can begin saving images before the replay is entirely set up.  Fortunately, the ground-truth detection files include in the filename the starting frame number, e.g., `vehicle_bboxes_96243.txt`.  Verify that the two detection files have the same starting frame, and then delete any images with a lower frame number from the following three directories:

* `$CARLA_DIR/PythonAPI/examples/isorc20/{SCENARIO_NAME}/rgb`
* `$CARLA_DIR/PythonAPI/examples/isorc20/{SCENARIO_NAME}/depth`
* `$CARLA_DIR/PythonAPI/examples/isorc20/{SCENARIO_NAME}/semseg`

### b) Filter out fully occluded pedestrians and vehicles

For the ground-truth detections, this means filtering out any that aren't visible to the camera on the ego vehicle.  This is done using the semantic segmentation information (it isn't perfect, but it's a close proxy).  Given a rectangle in 2D image space (the ground-truth detection result), the semantic label of each pixel in the rectangle is checked; if none matches the target type (pedestrian or vehicle), the detection is filtered out.

For each scenario and each target type, update lines 7, 8, and 12 of `remove_invisible_targets.py` and run it.  You'll need the starting frame number from the previous step.

```
cd $CARLA_DIR/PythonAPI/examples
python3 remove_invisible_targets.py
```

This step will result in one new ground-truth detection file per target type, with a filename like `vehicle_bboxes_96243_vis.txt`.

## 4. (Optional) Detect vehicles/pedestrians in images

```
TODO
```

The RGB images in `isorc/rgb/` can be used to perform vehicle or pedestrian detection.

## 5. Use TBD to track vehicles/pedestrians

```
TODO
```

## 6. Compute tracking metrics

```
TODO
```

## Original CARLA README follows:

CARLA Simulator
===============

[![Build Status](https://travis-ci.org/carla-simulator/carla.svg?branch=master)](https://travis-ci.org/carla-simulator/carla)
[![Documentation](https://readthedocs.org/projects/carla/badge/?version=latest)](http://carla.readthedocs.io)

[![carla.org](Docs/img/btn/web.png)](http://carla.org)
[![download](Docs/img/btn/download.png)](https://github.com/carla-simulator/carla/blob/master/Docs/download.md)
[![documentation](Docs/img/btn/docs.png)](http://carla.readthedocs.io)
[![discord](Docs/img/btn/chat.png)](https://discord.gg/8kqACuC)
<!-- [![forum](Docs/img/btn/forum.png)](link here) -->

CARLA is an open-source simulator for autonomous driving research. CARLA has
been developed from the ground up to support development, training, and
validation of autonomous driving systems. In addition to open-source code
and protocols, CARLA provides open digital assets (urban layouts, buildings,
vehicles) that were created for this purpose and can be used freely. The
simulation platform supports flexible specification of sensor suites and
environmental conditions.

[![CARLA Video](Docs/img/video_thumbnail.png)](https://www.youtube.com/watch?v=TOojcifcRBA)

If you want to benchmark your model in the same conditions as in our CoRL’17
paper, check out
[Benchmarking](https://github.com/carla-simulator/driving-benchmarks).

[**Get CARLA overnight build**](http://carla-assets-internal.s3.amazonaws.com/Releases/Linux/Dev/CARLA_Latest.tar.gz)

## CARLA Ecosystem
Repositories associated to the CARLA simulation platform:

* [**Scenario_Runner**](https://github.com/carla-simulator/scenario_runner): Engine to execute traffic scenarios in CARLA 0.9.X
* [**ROS-bridge**](https://github.com/carla-simulator/ros-bridge): Interface to connect CARLA 0.9.X to ROS
* [**Driving-benchmarks**](https://github.com/carla-simulator/driving-benchmarks): Benchmark tools for Autonomous Driving tasks
* [**Conditional Imitation-Learning**](https://github.com/felipecode/coiltraine): Training and testing Conditional Imitation Learning models in CARLA
* [**AutoWare AV stack**](https://github.com/carla-simulator/carla-autoware): Bridge to connect AutoWare AV stack to CARLA
* [**Reinforcement-Learning**](https://github.com/carla-simulator/reinforcement-learning): Code for running Conditional Reinforcement Learning models in CARLA
* [**Map Editor**](https://github.com/carla-simulator/carla-map-editor): Standalone GUI application to enhance RoadRunner maps with traffic lights and traffic signs information

**Like what you see? Star us on GitHub to support the project!**

Paper
-----

If you use CARLA, please cite our CoRL’17 paper.

_CARLA: An Open Urban Driving Simulator_<br>Alexey Dosovitskiy, German Ros,
Felipe Codevilla, Antonio Lopez, Vladlen Koltun; PMLR 78:1-16
[[PDF](http://proceedings.mlr.press/v78/dosovitskiy17a/dosovitskiy17a.pdf)]
[[talk](https://www.youtube.com/watch?v=xfyK03MEZ9Q&feature=youtu.be&t=2h44m30s)]


```
@inproceedings{Dosovitskiy17,
  title = {{CARLA}: {An} Open Urban Driving Simulator},
  author = {Alexey Dosovitskiy and German Ros and Felipe Codevilla and Antonio Lopez and Vladlen Koltun},
  booktitle = {Proceedings of the 1st Annual Conference on Robot Learning},
  pages = {1--16},
  year = {2017}
}
```

Building CARLA
--------------

Use `git clone` or download the project from this page. Note that the master
branch contains the latest fixes and features, for the latest stable code may be
best to switch to the `stable` branch.

Then follow the instruction at [How to build on Linux][buildlinuxlink] or
[How to build on Windows][buildwindowslink].

Unfortunately we don't have official instructions to build on Mac yet, please
check the progress at [issue #150][issue150].

[buildlinuxlink]: https://carla.readthedocs.io/en/latest/build_linux/
[buildwindowslink]: http://carla.readthedocs.io/en/latest/build_windows/
[issue150]: https://github.com/carla-simulator/carla/issues/150

Contributing
------------

Please take a look at our [Contribution guidelines][contriblink].

[contriblink]: http://carla.readthedocs.io/en/latest/CONTRIBUTING

F.A.Q.
------

If you run into problems, check our
[FAQ](http://carla.readthedocs.io/en/latest/faq/).

License
-------

CARLA specific code is distributed under MIT License.

CARLA specific assets are distributed under CC-BY License.

Note that UE4 itself follows its own license terms.
