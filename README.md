# AMR 2D Simulator & Dashboard

ชุดเครื่องมือสำหรับการจำลองและควบคุมหุ่นยนต์ (Autonomous Mobile Robot) ประกอบด้วยตัวจำลองใน ROS 2 และ Dashboard สำหรับการควบคุมผ่านเว็บเบราว์เซอร์

## Required Software

ตรวจสอบให้แน่ใจว่าคุณได้ติดตั้งซอฟต์แวร์เหล่านี้บน **Ubuntu 24.04 (Noble)** หรือ **22.04 (Jammy)**:

* **ROS 2 Jazzy**
* **Python 3.10+**
* **Node.js 24+** & **npm**
* **ROSbridge Suite**

```bash
sudo apt install ros-jazzy-rosbridge-suite git

```

---

## 1. Simulator Installation (ROS 2 Package)

### 1.1 Create ROS 2 Workspace

```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

```

### 1.2 Clone and Build

```bash
git clone https://github.com/Oscars03/amr_2dsim.git
cd ~/robot_ws
colcon build --symlink-install
source install/setup.bash

```

---

## 2. Dashboard Installation

Dashboard สำหรับการควบคุมและแสดงผลการจำลองหุ่นยนต์ พัฒนาด้วย React, Vite และ Electron

### 2.1 Clone the Repository

```bash
git clone https://github.com/Oscars03/amr-sim-dashboard.git
cd amr-sim-dashboard

```

### 2.2 Install Dependencies & Setup Permissions

ติดตั้ง Node.js ผ่าน `nvm`, ติดตั้งแพ็กเกจ และตั้งค่าสิทธิ์ Sandbox:

```bash
# ติดตั้ง nvm
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.5/install.sh | bash
source ~/.bashrc

# ติดตั้งและใช้งาน Node.js
nvm install 24
nvm use 24

# ติดตั้ง dependencies
npm install

# ตั้งค่าสิทธิ์ Sandbox ให้กับ Electron
sudo chown root:root node_modules/electron/dist/chrome-sandbox
sudo chmod 4755 node_modules/electron/dist/chrome-sandbox

```

---

## 3. How to Run

1. **Start ROS 2 Environment:**
```bash
source ~/robot_ws/install/setup.bash

```


2. **Start Dashboard:**
```bash
cd ~/amr-sim-dashboard
npm run dev

```



## Project Links

* **Simulator:** [https://github.com/Oscars03/amr_2dsim](https://github.com/Oscars03/amr_2dsim)
* **Dashboard:** [https://github.com/Oscars03/amr-sim-dashboard](https://github.com/Oscars03/amr-sim-dashboard)

---

*Developed by Phuthanet Phengphan*
