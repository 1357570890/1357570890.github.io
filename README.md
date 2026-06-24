# Haozhen's Interactive Portfolio & Algorithm Lab
> **张颢震的个人主页与算法控制系统仿真实验室**

Welcome to my personal developer portfolio and interactive algorithm playground. This repository hosts my GitHub Pages site, demonstrating my foundational capabilities in **robotics control, path planning, 3D point cloud transformation, and edge AI deployment**.

🚀 **Live Site**: [https://1357570890.github.io/](https://1357570890.github.io/)

---

## 🔬 Core Highlights: Interactive Simulation Labs

Unlike typical static portfolios, this project contains three fully functional, lightweight, zero-dependency algorithm simulations implemented in **pure Vanilla JavaScript** directly rendering on HTML5 Canvas. These serve as a direct demonstration of my mathematics and control systems background.

### 1. RRT* (Rapidly-exploring Random Tree Star) Path Planner
* **Description**: Real-time kinematic path planning in a grid map containing user-drawn obstacles.
* **Algorithm Mechanism**:
  * **Random Sampling**: Generates random target coordinates to bias tree growth.
  * **Nearest Node Selection**: Selects the tree node closest to the sample point and extends a fixed-step branch.
  * **Rewire Operator (RRT* Optimization)**: Checks neighboring nodes within a search radius $r = 35px$ to see if a shorter cost path can be achieved through the newly added node. If so, updates the node parents to optimize path length.
  * **Collision Detection**: Implements the **Liang-Barsky Line Clipping Algorithm** to check intersections between tree branches and AABB (Axis-Aligned Bounding Box) obstacles in $O(1)$ time complexity.
* **Interactive Feature**: You can **click and drag your mouse directly on the canvas** to draw custom obstacles and watch the tree adapt in real-time.

### 2. Drone Altitude PID Controller Loop
* **Description**: A 1D vertical altitude hovering simulation for a quadcopter balancing against gravity.
* **Math Formulation**:
  $$u(t) = K_p e(t) + K_i \int_{0}^{t} e(\tau) d\tau + K_d \frac{de(t)}{dt}$$
* **System Characteristics**:
  * **Anti-Windup Clamp**: Integrates anti-windup clamping to prevent the integral accumulator from overflowing.
  * **Real-time Feedback**: Renders a scrolling PV-SP (Process Variable vs Set Point) strip chart.
  * **User Control**: Change the target hovering line instantly by clicking anywhere on the simulator space, and tune $K_p$, $K_i$, and $K_d$ gains using slider inputs.

### 3. 3D LIDAR Point Cloud Projection (Matrix Rotation)
* **Description**: A lightweight 3D wireframe and point-cloud renderer rotating a quadruped robot model.
* **Math Formulation** (Euler Rotation and Perspective Projection):
  * **Rotation Matrix**: Applies yaw and pitch rotations to the 3D coordinate vectors using rotation matrices:
    $$R_y(\theta_y) = \begin{bmatrix} \cos\theta_y & 0 & \sin\theta_y \\ 0 & 1 & 0 \\ -\sin\theta_y & 0 & \cos\theta_y \end{bmatrix}, \quad R_x(\theta_x) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\theta_x & -\sin\theta_x \\ 0 & \sin\theta_x & \cos\theta_x \end{bmatrix}$$
  * **Perspective Projection**: Projects rotated coordinates $(x_{rot}, y_{rot}, z_{rot})$ onto the 2D viewport:
    $$x_{screen} = c_x + \frac{x_{rot} \cdot f}{z_{rot} + d}, \quad y_{screen} = c_y + \frac{y_{rot} \cdot f}{z_{rot} + d}$$
  * **Depth Intensity Rendering**: Simulates LIDAR intensity values by scaling point sizes and opacities inversely proportional to their relative depth coordinates ($z_{rot}$).
* **Interactive Feature**: Drag with the mouse left-button on the canvas to manually rotate the 3D model.

---

## 📂 Repository Structure

```ascii
.
├── index.html        # Main semantic HTML5 layout & SEO tags
├── style.css         # Custom stylesheet (Glassmorphism theme, responsiveness)
├── script.js        # Algorithm simulation loops (RRT*, PID, 3D Matrix Projection)
└── README.md         # Technical documentation & project guide
```

---

## 🛠️ Local Development & Quick Start

1. **Clone the repository**:
   ```bash
   git clone git@github.com:1357570890/1357570890.github.io.git
   cd 1357570890.github.io
   ```

2. **Run locally**:
   Since the project is implemented in pure vanilla JS and CSS, it has **zero package dependencies**. Simply open `index.html` in your web browser, or run a simple local server:
   ```bash
   # Python 3
   python -m http.server 8000
   
   # Node.js / npm
   npx http-server -p 8000
   ```
   Open `http://localhost:8000` to view the page.

---

## 📝 License

This project is open-source and licensed under the [MIT License](LICENSE) (or free for personal/non-commercial portfolio reuse). Feel free to explore the code!
