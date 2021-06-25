# urdf-viz

[![Build Status](https://img.shields.io/github/workflow/status/openrr/urdf-viz/CI/main)](https://github.com/openrr/urdf-viz/actions) [![crates.io](https://img.shields.io/crates/v/urdf-viz.svg)](https://crates.io/crates/urdf-viz) [![docs](https://docs.rs/urdf-viz/badge.svg)](https://docs.rs/urdf-viz)

Visualize [URDF(Unified Robot Description Format)](http://wiki.ros.org/urdf) file.
`urdf-viz` is written in Rust-lang.

## Install

### Install with `cargo`

If you are using rust-lang already and `cargo` is installed, you can install by `cargo install`.

```bash
cargo install urdf-viz
```

If you use mesh other than `.obj`, `.stl`, and `.dae` files, you need to install urdf-viz with the `assimp` feature like below:

```bash
cargo install urdf-viz --features assimp
```

### Pre-requirements for build

#### Common

You need [cmake](https://cmake.org/download) if you use `--features assimp`.

#### On Linux

If you have not installed ROS, you may need `cmake`, `xorg-dev`, `glu` to
compile `assimp-sys` and `glfw-sys`.

```bash
sudo apt-get install cmake xorg-dev libglu1-mesa-dev
```

#### On Windows

You need freetype.lib in your PATH, which is required by `freetype-sys`.
You can find binaries [here](https://github.com/PistonDevelopers/binaries)

#### On MacOS

Install freetype by brew.

```bash
brew install freetype
```

### Download binary

You can download prebuilt binaries from the [release page](https://github.com/openrr/urdf-viz/releases).
Prebuilt binaries are available for macOS and Linux.

<!-- TODO: distribute binary for Windows -->

### Install via Homebrew

You can install urdf-viz using [Homebrew tap on macOS and Linux](https://github.com/openrr/homebrew-tap/blob/main/Formula/urdf-viz.rb):

```sh
brew install openrr/tap/urdf-viz
```

## How to use

`urdf-viz` command will be installed.
It needs `rosrun` and `rospack` to resolve `package://` in `<mesh>` tag, and
it uses `xacro` to convert `.xacro` file into urdf file.
It means you need `$ source ~/catkin_ws/devel/setup.bash` or something before using `urdf-viz`.

```bash
urdf-viz URDF_FILE.urdf
```

It is possible to use xacro file directly.
It will be converted by `rosrun xacro xacro` inside of `urdf-viz`.

```bash
urdf-viz XACRO_FILE.urdf.xacro
```

For other options, please read the output of `-h` option.

```bash
urdf-viz -h
```

If there are no "package://" in mesh tag, and don't use xacro you can skip install of ROS.

## GUI Usage

In the GUI, you can do some operations with keyboard and mouse.

* `l` key to reload the urdf from file
* `c` key to toggle collision model or visual mode
* Move a joint
  * set the angle of a joint by `Up`/`Down` key
  * `Ctrl` + Drag to move the angle of a joint
  * change the joint to be moved by `o` (`[`) and `p` (`]`)
* Inverse kinematics (only positions)
  * `Shift` + Drag to use inverse kinematics(Y and Z axis)
  * `Shift` + `Ctrl` + Drag to use inverse kinematics(X and Z axis)
  * change the move target for inverse kinematics by `,` or `.`
* `r` key to set random joints
* `z` key to reset joint positions and origin
* Move view point
  * Mouse Right Drag to translate view camera position
  * Mouse Left Drag to look around
  * Scroll to zoom in/out

## Web I/O interface

You can set/get the joint angles using http/JSON.
Default port number is 7777. You can change it by `-p` option.
(`jq` is used for JSON formatter in the following examples)

### Set joint angles

POST the JSON data, which format is like below. You have to specify the names of joints and positions (angles).
The length of `names` and `positions` have to be the same. You don't need write
all joint names, it means you can specify a part of the joints.

```json
{
  "names": ["joint_name1", "joint_name2"],
  "positions": [0.5, -0.1]
}
```

You can try it using `curl`.

```bash
$ curl -H "Accept: application/json" -H "Content-type: application/json" -X POST -d '{"names": ["r_shoulder_yaw", "r_shoulder_pitch"], "positions": [0.8, -0.8]}' http://127.0.0.1:7777/set_joint_positions | jq
{
  "is_ok": true,
  "reason": ""
}
```

### Get joint angles as JSON

The result JSON format of getting the joint angles is the same as the *Set* method.

```bash
$ curl http://127.0.0.1:7777/get_joint_positions | jq
{
  "names": [
    "r_shoulder_yaw",
    "r_shoulder_pitch",
    "r_shoulder_roll",
    "r_elbow_pitch",
    "r_wrist_yaw",
    "r_wrist_pitch",
    "l_shoulder_yaw",
    "l_shoulder_pitch",
    "l_shoulder_roll",
    "l_elbow_pitch",
    "l_wrist_yaw",
    "l_wrist_pitch"
  ],
  "positions": [
    0.8,
    -0.8,
    -1.3447506,
    -1.6683152,
    -1.786362,
    -1.0689334,
    0.11638665,
    -0.5987091,
    0.7868867,
    -0.027412653,
    0.019940138,
    -0.6975361
  ]
}
```

### Set Robot Origin

```bash
$ curl -H "Accept: application/json" -H "Content-type: application/json" -X POST -d '{"position":[0.2,0.0,0.0],"quaternion":[0.0,0.0,0.0,1.0]}' http://127.0.0.1:7777/set_robot_origin
{"is_ok":true,"reason":""}
```

The order of the quaternion elements is `w, i, j, k`.

### Get Robot Origin

```bash
$ curl http://127.0.0.1:7777/get_robot_origin
{"position":[0.2,0.0,0.0],"quaternion":[1.0,0.0,0.0,0.0]}
```

## Gallery

![sawyer_1.png](img/sawyer_1.png)
![sawyer_2.png](img/sawyer_2.png)

![nextage_1.png](img/nextage_1.png)
![nextage_2.png](img/nextage_2.png)

![hsr_1.png](img/hsr_1.png)
![hsr_2.png](img/hsr_2.png)

![ubr1_1.png](img/ubr1_1.png)
![ubr1_2.png](img/ubr1_2.png)

![pepper_1.png](img/pepper_1.png)
![pepper_2.png](img/pepper_2.png)

![pr2_1.png](img/pr2_1.png)
![pr2_2.png](img/pr2_2.png)

![thormang3_1.png](img/thormang3_1.png)
![thormang3_2.png](img/thormang3_2.png)

## Dependencies

* [kiss3d](https://github.com/sebcrozet/kiss3d): `urdf-viz` is strongly depend on `kiss3d`, which is super easy to use, great 3D graphic engine.
* [nalgabra](https://github.com/sebcrozet/nalgebra): linear algebra library.
* [k](https://github.com/openrr/k): kinematics library which is based on [nalgabra](https://github.com/sebcrozet/nalgebra). It can load URDF files using `urdf-rs`.
* [assimp-rs](https://github.com/Eljay/assimp-rs): assimp rust interface. `kiss3d` supports `.obj` files natively, but urdf contains `dae` or `stl` files. These files are converted to kiss3d mesh model by `assimp-rs`
* [urdf-rs](https://github.com/openrr/urdf-rs): URDF file loader.
* [structopt](https://github.com/TeXitoi/structopt): super easy command line arguments parser.
