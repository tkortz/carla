Project Information
===============

This repository was used for the paper ["The Price of Schedulability in Multi-ObjectTracking: The History-vs.-Accuracy Trade-Off"](https://www.cs.unc.edu/~tamert/papers/isorc20.pdf), presented at ISORC 2020.

Other relevant repositories:

* [Tracking-by-Detection](TODO)

In order to reproduce the results in our paper, the following steps are necessary:
1. Record the scenarios, or use our `.rec` files (see instructions [here](TODO)).
2. Replay each scenario in the CARLA client to generate the ground-truth vehicle/pedestrian detections and RGB, depth, and semantic-segmentation images.  Note that this step takes a long time.
3. Post-process the ground-truth detections to filter out invisible agents.
4. (Optional) Process images to detect vehicles/pedestrians.
5. Use our Tracking-by-Detection to track vehicles/pedestrians in each scenario for each history probability-mass function.
6. Process the results to compute tracking metrics.

We now walk through in each step in detail.  These instructions assume the repository root directory is `$CARLA_DIR`.

## 1. Record scenarios

```
TODO: finish
```

The recordings should be saved in `~/.config/Epic/CarlaUE4/Saved/`, with the hero agent ID as the filename  For example, using our pre-recorded scenario files you should have:
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

If you record your own scenarios, be sure to save the recording file with the ego-vehicle ID, and update lines 134-141 of `manual_control_synchronous.py` accordingy.

## 2. Replay each scenario to generate ground-truth data and images

First, launch the CARLA server.  With our setup, we use the following commands:

```
cd $CARLA_DIR
./Dist/CARLA_Shipping_0.9.6-23-g89e329b/LinuxNoEditor/CarlaUE4.sh
```

Next, set up the scenario you want to replay using `config.py`, where `$TOWN` comes from the list below:

* Scenario 1: Town01
* Scenario 2: Town03
* Scenario 3: Town03
* Scenario 4: Town04

```
cd $CARLA_DIR/PythonAPI/util
python3 config.py -m $TOWN --w ClearNoon
```

Then, create the folders `$CARLA_DIR/PythonAPI/examples/isorc/{SCENARIO_NAME}/_ground_truth/` for each scenario you plan to run (e.g., `scenario_1`).  Once the CARLA client has started, navigate to the PythonAPI-examples directory and run the following Python script to enable the synchronous replay:

```
cd $CARLA_DIR/PythonAPI/examples
python3 manual_control_synchronous.py
```

This script will spawn a vehicle (which we'll ignore), and is ready by default to replay scenario 1.  To change scenarios, change line 132 of `manual_control_synchronous.py` to set `SCENARIO_NAME` accordingly.

Once the vehicle spawns, press `ctrl+p` once to replay the scenario.  This causes the following to occur:

* Enables the display of ground-truth bounding boxes for both vehicles and pedestrians.
* Writes the corresponding ground-truth positions of vehicles and pedestrians to files in the directory `$CARLA_DIR/PythonAPI/examples/isorc20/{SCENARIO_NAME}/_ground_truth/`.
* Outputs one RGB, one depth, and one semantic segmentation image per frame from the camera on the front of the ego vehicle to the directory `$CARLA_DIR/PythonAPI/examples/isorc20/{SCENARIO_NAME}/_out/`.

Note that this step is very slow, as it requires running the server synchronously and outputting a lot of data per frame.  If you're just testing the workflow, you can hit `escape` after a couple of frames to close the client.

```
TODO: mention what to do when it's done
```

## 3. Post-process ground-truth detections to filter out invisible agents

```
TODO
```

## 4. (Optional) Detect vehicles/pedestrians in images

```
TODO
```

The RGB images in `isorc/_out/

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

[buildlinuxlink]: http://carla.readthedocs.io/en/latest/how_to_build_on_linux
[buildwindowslink]: http://carla.readthedocs.io/en/latest/how_to_build_on_windows
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
