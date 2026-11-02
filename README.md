# Vision-Based Robotic Colour-Sorting Cell (Webots)

A simulated pick-and-place cell where a **UR5e robot arm** watches the end of a conveyor with a camera, recognises the **colour** of each block, picks it up with a **Robotiq 3-finger gripper**, and drops it in the **matching crate**.

![Demo](media/demo.gif)
<!-- Record a short clip in Webots (🎬 button), convert it to a GIF, and save it as media/demo.gif -->

**Built with:** Webots R2025a · Python 3 · NumPy

---

## Features

- 🎥 **Camera-based colour detection:** red, green and blue blocks are recognised from the gripper-mounted camera.
- 🎯 **Region of interest (ROI):** only the pick spot is analysed, so crates, the floor and fallen blocks are ignored.
- ⏸️ **Stillness check:** the robot waits until a block has completely stopped before picking it.
- 📐 **Analytic inverse kinematics:** converts any (x, y, z) target into six joint angles, with the gripper always pointing straight down.
- 🔁 **Closed-loop motion:** every move is confirmed with the joint position sensors, with a timeout if the arm gets blocked.
- ✋ **Grip check from finger angles:** the robot detects whether a block is really in its hand, and retries if it missed.

---

## How it works

```
        ┌──────────────────────────────────────────────┐
        ▼                                              │
  SENSE   camera → crop ROI → colour masks → centroid  │
        ▼                                              │
  CONFIRM block centre unchanged for 0.5 s             │
        ▼                                              │
  THINK   inverse kinematics → 6 joint angles          │
        ▼                                              │
  ACT     move (sensor feedback) → grip → check grip   │
        ▼                                              │
  PLACE   look up crate for that colour → release ─────┘
```

### 1. Colour detection

The 64 × 64 camera image is converted to a NumPy array (Webots stores pixels as **BGRA**). Only a 32 × 32 window around the pick spot is analysed. A pixel counts as red when:

```python
(r > 100) & (r > g + 50) & (r > b + 50)
```

The colour with the most pixels wins, provided it reaches at least `MIN_PIXELS` (40). Without the ROI, the camera saw part of the red crate and "detected" blocks that weren't there.

### 2. Stillness check

The block's centre in the image (the mean pixel position of the colour mask) is tracked for 0.5 s. If it moves by more than half a pixel, the block is still sliding and the robot keeps waiting.

### 3. Inverse kinematics

With the gripper fixed pointing down, the problem splits into three simple parts:

**Base angle (top view).** The UR5e wrist sits $L = 0.134$ m to the side of the arm plane:

$$r = \sqrt{x^2 + y^2 - L^2}, \qquad \theta_1 = \operatorname{atan2}(y, x) - \operatorname{atan2}(L, r)$$

**Shoulder and elbow (side view).** The upper arm $a_2 = 0.425$ m and forearm $a_3 = 0.3922$ m form a triangle with the shoulder-to-wrist line $(p_x, p_y)$. The cosine rule gives:

$$\cos\theta_3 = \frac{p_x^2 + p_y^2 - a_2^2 - a_3^2}{2a_2a_3}$$

$$\theta_2 = \operatorname{atan2}(p_y, p_x) - \operatorname{atan2}(a_3\sin\theta_3,\; a_2 + a_3\cos\theta_3)$$

The cosine is clamped to [−1, 1], so a target outside the workspace cannot crash the program.

**Wrist (keep the gripper vertical).** The shoulder, elbow and wrist-1 axes are parallel, so their angles add up:

$$\theta_2 + \theta_3 + \theta_4 = -\tfrac{\pi}{2} \;\Rightarrow\; \theta_4 = -(\theta_2 + \theta_3) - \tfrac{\pi}{2}$$

The geometry constants match the official UR5e DH parameters (a2, a3, d4, d5, d6). The IK was verified with a forward-kinematics function: `fk(ik(target))` returns the original target.

### 4. Closed-loop motion

`move_to()` sends the six target angles, then reads the joint sensors every time step until all errors are below 0.025 rad (about 1.4°). If the arm cannot get there within the timeout, it reports failure instead of waiting forever. The arm moves fast in open space and slows down for the last 5 cm above a block.

### 5. Grip check (proprioception)

The fingers are commanded to 0.9 rad and close gently (1.5 rad/s):

| Finger angles after closing | Meaning |
|---|---|
| all reach ≈ 0.9 | empty hand, so it missed |
| at least one stops early (≈ 0.7) | block is held ✅ |
| a finger is pushed below 0 | it hit something else |

On a miss, the robot opens, checks with the camera that the block is still there, and retries up to 3 times, 1 cm lower each time. It checks the grip again after lifting.

---

## Results

In a test run, the cell sorted **5 of 5 blocks into the correct crates, each on the first grasp attempt**, at about **10.7 s per block**.

### Debugging story

The first versions gripped unreliably. Simulation logs showed one finger angle going **negative** and the lift **timing out**. The cause was the conveyor's stopper wall: it was as tall as the block, so a finger hit it and hooked on it while lifting. Lowering the stopper to 3 cm and making the blocks taller removed the collision.

---

## Project structure

```
colour_sorting/
├── worlds/
│   └── colour_sorting.wbt          # scene: UR5e, gripper, camera, conveyor, stopper, blocks, crates
├── controllers/
│   └── colour_sorter/
│       └── colour_sorter.py        # vision, IK, motion and grip logic
├── media/
│   └── demo.gif                    # demo recording
└── README.md
```

---

## How to run

1. Install **[Webots R2025a](https://cyberbotics.com/)** and Python 3.
2. Install NumPy:
   ```bash
   pip install numpy
   ```
3. Clone this repository and open `worlds/colour_sorting.wbt` in Webots.
4. Press **Play ▶**. The console shows each detection, grasp and the running sort count.

---

## Configuration

All settings are at the top of `colour_sorter.py`:

| Setting | Default | Purpose |
|---|---|---|
| `PICK` | (0.55, 0.02) | where blocks stop on the conveyor (m) |
| `BLOCK_Z` | 0.30 | height of the block centre (m) |
| `GRIP_OFFSET` | 0.24 | gripper flange height above the block centre when gripping (m) |
| `CRATES` | per colour | (x, y) drop position for each colour |
| `MIN_PIXELS` | 40 | coloured pixels needed to count as a block |
| `ROI_X`, `ROI_Y` | (14, 46) | image window that is analysed |
| `STILL_TIME` | 0.5 | seconds a block must be still before picking |
| `SPEED` / `SLOW_SPEED` | 2.0 / 0.5 | arm speed far from / close to the block (rad/s) |
| `FINGER_SPEED` | 1.5 | finger closing speed (rad/s) |
| `MAX_TRIES` | 3 | grasp attempts per block |

---

## Limitations and next steps

- Pick and crate positions are fixed in code; the camera decides **which** crate, not **where** the block is.
- Colour thresholds are tuned for the simulator's lighting.
- **Next:** add a fourth colour, estimate block position from the camera (visual servoing), and run the cell through **ROS 2** (`webots_ros2`).

---

## Acknowledgements

I designed the cell and the sorting workflow. The controller code was developed with the help of an AI coding assistant; I debugged and tuned it in simulation and can explain the vision, kinematics and control in detail.

## Author

**Prithvi Mishra** · B.Tech Robotics & Artificial Intelligence (EV Specialization), IILM University

[GitHub](https://github.com/Prithvimishra7) · [LinkedIn](https://linkedin.com/in/prithvi-mishra-63337036b)
