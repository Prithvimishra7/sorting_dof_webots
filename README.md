# 🤖 Vision-Based Robotic Colour-Sorting Cell | Webots

A simulated autonomous pick-and-place system using a **UR5e robotic manipulator**, a camera, and a **Robotiq 3-Finger Gripper** to identify and sort coloured blocks into their corresponding crates.

The system combines computer vision, analytical inverse kinematics, joint-level feedback control, and grasp verification to perform autonomous colour-based sorting.

<p align="center">
  <img src="media/demo.gif" alt="Vision-Based Robotic Colour Sorting Demo" width="750">
</p>

<p align="center">
  <b>Webots R2025a | Python 3 | NumPy | UR5e | Computer Vision</b>
</p>

---

## 📌 Project Overview

This project implements a simulated robotic sorting cell in which a UR5e robot observes blocks arriving at a conveyor using a gripper-mounted camera.

The robot identifies whether a block is red, green, or blue, waits until it becomes stationary, calculates the required joint positions using analytical inverse kinematics, and picks it up using a Robotiq 3-finger gripper.

After successful grasp verification, the robot places the block into the corresponding colour-specific crate.

## ✨ Key Features

- 🎥 **Camera-Based Colour Detection:** Identifies red, green, and blue blocks using image processing.
- 🎯 **Region of Interest (ROI):** Analyses a defined 32 × 32 pixel window around the picking area to avoid false detections.
- ⏸️ **Object Stillness Detection:** Tracks the block centroid and confirms that the object has stopped moving before picking.
- 📐 **Analytical Inverse Kinematics:** Calculates six joint angles for target positions while maintaining a vertically oriented gripper.
- 🔄 **Forward Kinematics Verification:** Checks the calculated inverse kinematics results.
- 🎮 **Closed-Loop Motion Control:** Uses joint position sensors to verify that the robot reaches its target.
- ✋ **Grasp Verification:** Uses finger joint angles to determine whether a block has been successfully grasped.
- 🔁 **Automatic Retry Mechanism:** Retries unsuccessful grasps, with a maximum of three attempts.
- 📦 **Automated Sorting:** Places each detected block into its corresponding colour-specific crate.

---

## ⚙️ System Architecture

The system follows a perception–decision–action workflow.

```text
             ┌─────────────────────────┐
             │     Camera Input        │
             └────────────┬────────────┘
                          ▼
             ┌─────────────────────────┐
             │   Image Processing      │
             │  ROI + Colour Detection │
             └────────────┬────────────┘
                          ▼
             ┌─────────────────────────┐
             │   Object Stillness      │
             │       Detection         │
             └────────────┬────────────┘
                          ▼
             ┌─────────────────────────┐
             │   Inverse Kinematics    │
             │    Six Joint Angles     │
             └────────────┬────────────┘
                          ▼
             ┌─────────────────────────┐
             │   Robot Arm Movement    │
             │   Joint Sensor Feedback │
             └────────────┬────────────┘
                          ▼
             ┌─────────────────────────┐
             │   Grasp Verification    │
             └────────────┬────────────┘
                          ▼
             ┌─────────────────────────┐
             │  Colour-Based Placement │
             └────────────┬────────────┘
                          │
                          └──────► Repeat
```

## 🧠 Technical Implementation

### 1. Camera-Based Colour Detection

The gripper-mounted camera captures a 64 × 64 image.

Webots camera data is converted into a NumPy array using the BGRA pixel format.

To improve detection reliability, the controller analyses only a 32 × 32 pixel Region of Interest (ROI) around the designated picking position.

Example red-colour detection condition:

```python
(r > 100) & (r > g + 50) & (r > b + 50)
```

The controller identifies the colour with the highest number of matching pixels, provided the detection reaches the minimum threshold of `MIN_PIXELS = 40`.

The ROI also prevents interference from the red crate, floor, and fallen objects.

### 2. Object Stillness Detection

Before attempting a grasp, the controller tracks the detected block's image centroid.

- Tracks the centroid for 0.5 seconds.
- Compares its position across consecutive frames.
- Waits if the centroid moves by more than 0.5 pixels.
- Proceeds with picking once the block is stationary.

This prevents the robot from attempting to grasp a moving block.

### 3. Analytical Inverse Kinematics

Analytical inverse kinematics is implemented to calculate the robot's six joint angles for a specified target position.

The calculation is divided into three geometric components:

**Base angle**

The wrist offset is considered when calculating the base joint position.

\[
r=\sqrt{x^2+y^2-L^2}
\]

\[
\theta_1=\operatorname{atan2}(y,x)
-\operatorname{atan2}(L,r)
\]

Where:

\[
L=0.134\text{ m}
\]

**Shoulder and elbow angles**

The upper arm and forearm lengths are:

\[
a_2=0.425\text{ m}
\]

\[
a_3=0.3922\text{ m}
\]

Using the cosine rule:

\[
\cos\theta_3=
\frac{p_x^2+p_y^2-a_2^2-a_3^2}
{2a_2a_3}
\]

The shoulder angle is calculated using:

\[
\theta_2=
\operatorname{atan2}(p_y,p_x)
-
\operatorname{atan2}
(a_3\sin\theta_3,\,
a_2+a_3\cos\theta_3)
\]

**Wrist orientation**

To maintain a vertically oriented gripper:

\[
\theta_2+\theta_3+\theta_4=-\frac{\pi}{2}
\]

Therefore:

\[
\theta_4=-(\theta_2+\theta_3)-\frac{\pi}{2}
\]

The implementation also clamps the cosine value within the valid range of [-1, 1].

The inverse kinematics implementation is verified using forward kinematics:

```python
fk(ik(target))
```

The resulting position is checked against the original target.

### 4. Closed-Loop Motion Control

The `move_to()` function commands the six robot joints and monitors their actual positions through joint position sensors.

Movement is considered complete when all joint errors are below:

\[
0.025\text{ rad}\approx1.4^\circ
\]

A timeout mechanism prevents the controller from waiting indefinitely if the robot cannot reach its target.

The controller also uses different movement speeds:

- Normal movement: 2.0 rad/s
- Slow movement near the block: 0.5 rad/s

This allows faster movement in open space and more controlled movement near the picking position.

### 5. Grasp Verification and Retry Logic

The Robotiq 3-finger gripper uses finger joint angles to determine whether an object has been successfully grasped.

The fingers are commanded to close to 0.9 rad at 1.5 rad/s.

| Finger behaviour | Interpretation |
|---|---|
| All fingers reach approximately 0.9 rad | Empty hand / unsuccessful grasp |
| At least one finger stops around 0.7 rad | Block successfully held |
| Finger moves below 0 rad | Possible collision |

If a grasp fails:

1. The gripper opens.
2. The camera checks whether the block is still present.
3. The robot retries up to three times.
4. Each retry lowers the grasp position by 1 cm.
5. The grasp is verified again after lifting.

This adds a feedback-based verification stage to the pick-and-place process.

---

## 📊 Simulation Results

The robotic sorting cell was tested with five coloured blocks.

| Performance Metric | Result |
|---|---:|
| Total blocks tested | 5 |
| Correctly sorted blocks | 5/5 |
| First-attempt successful grasps | 5/5 |
| Average sorting time | ~10.7 seconds/block |

<badge color="success">5/5 successful sorting operations</badge>

### 🔧 Debugging and Problem Solving

During early simulation tests, the robot experienced unreliable grasping and lifting failures.

Simulation logs revealed:
- Negative finger joint angles.
- Lift operations timing out.
- Finger collisions with the conveyor stopper.

The issue was traced to the stopper wall, which was initially as tall as the blocks. A finger could collide with and hook onto the stopper during lifting.

**Implemented solution:**
- Reduced the stopper height to 3 cm.
- Increased block height.
- Re-tested the grasping and lifting sequence.

These environmental adjustments eliminated the observed collision problem in the tested setup.

---

## 📁 Project Structure

```text
colour_sorting/
│
├── worlds/
│   └── colour_sorting.wbt
│
├── controllers/
│   └── colour_sorter/
│       └── colour_sorter.py
│
├── media/
│   └── demo.gif
│
└── README.md
```

| File / Directory | Description |
|---|---|
| `worlds/` | Webots simulation environment |
| `colour_sorting.wbt` | UR5e, gripper, camera, conveyor, blocks and crates |
| `controllers/` | Robot controller implementation |
| `colour_sorter.py` | Vision, inverse kinematics, motion and grasp logic |
| `media/` | Project demonstration media |
| `README.md` | Project documentation |

---

## 🚀 Installation and Execution

### Requirements

- Webots R2025a
- Python 3
- NumPy
- UR5e robot model
- Robotiq 3-finger gripper model

### Step 1: Install Webots

Download and install:

[Webots Official Website](https://cyberbotics.com/)

### Step 2: Install NumPy

```bash
pip install numpy
```

### Step 3: Clone the Repository

```bash
git clone https://github.com/Prithvimishra7/colour_sorting.git
```

### Step 4: Open the Simulation

1. Launch Webots.
2. Open `worlds/colour_sorting.wbt`.
3. Ensure the controller is assigned to the robot.
4. Press **Play ▶**.
5. Monitor the console for colour detection, grasp verification and sorting progress.

---

## ⚙️ Configuration

The main configuration parameters are defined at the beginning of `colour_sorter.py`.

| Parameter | Default | Description |
|---|---|---|
| `PICK` | `(0.55, 0.02)` | Conveyor picking position (m) |
| `BLOCK_Z` | `0.30` | Block centre height (m) |
| `GRIP_OFFSET` | `0.24` | Gripper flange offset (m) |
| `CRATES` | Per colour | Drop position for each colour |
| `MIN_PIXELS` | `40` | Minimum colour detection pixels |
| `ROI_X`, `ROI_Y` | `(14, 46)` | ROI image window |
| `STILL_TIME` | `0.5` | Required stationary time (s) |
| `SPEED` | `2.0` | Normal arm speed (rad/s) |
| `SLOW_SPEED` | `0.5` | Slow arm speed (rad/s) |
| `FINGER_SPEED` | `1.5` | Gripper closing speed (rad/s) |
| `MAX_TRIES` | `3` | Maximum grasp attempts |

---

## ⚠️ Limitations and Future Improvements

The current implementation has some limitations:

- Picking and crate positions are predefined in the controller.
- The camera identifies the block's colour but does not estimate its world position.
- Colour detection thresholds are tuned for the simulator's lighting conditions.

**Planned improvements:**

- [ ] Add additional colour categories.
- [ ] Implement camera-based object position estimation.
- [ ] Introduce visual servoing for dynamic object positioning.
- [ ] Integrate the sorting cell with ROS 2 using `webots_ros2`.
- [ ] Explore more advanced autonomous manipulation techniques.

---

## 🙏 Acknowledgements

The simulation environment and sorting workflow were designed by me.

The controller code was developed with assistance from an AI coding assistant. I independently debugged and tuned the simulation, analysed controller behaviour, and worked on the vision, kinematics, and motion-control implementation.

---

## 👨‍💻 Author

**Prithvi Mishra**

B.Tech Undergraduate | Robotics & Artificial Intelligence  
Electric Vehicle Specialization  
IILM University, Greater Noida, India

- GitHub: [@Prithvimishra7](https://github.com/Prithvimishra7)
- LinkedIn: [Prithvi Mishra](https://linkedin.com/in/prithvi-mishra-63337036b)

---

⭐ If you find this project interesting, consider giving the repository a star!

