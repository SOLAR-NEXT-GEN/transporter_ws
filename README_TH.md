# transporter_ws

ROS 2 workspace สำหรับหุ่นยนต์ transporter ฐานล้อเป็นแบบ skid-steer ขับด้วย ODrive 4 ตัว และมี manipulator 2 ชุด (ใช้ปั๊มร่วมกัน 1 ตัว แต่ละข้างมีเอาต์พุต ขึ้น/ลง) ควบคุมด้วยบอร์ด STM32 ที่รัน micro-ROS ทั้งหมดบังคับด้วยจอย PS5

🇬🇧 English: [README.md](README.md)

---

## 1. วิธีติดตั้ง

ทดสอบบน **Ubuntu 24.04 + ROS 2 Jazzy**

### 1.1 แพ็กเกจที่ต้องติดตั้ง

| สิ่งที่ต้องมี | ใช้กับ | วิธีติดตั้ง |
|---|---|---|
| ROS 2 Jazzy (desktop หรือ ros-base) | ทั้งหมด | [docs.ros.org](https://docs.ros.org/en/jazzy/Installation.html) |
| `colcon`, `rosdep` | ใช้ build | `sudo apt install python3-colcon-common-extensions python3-rosdep` |
| `ros-jazzy-joy` | อ่านจอย PS5 → `/joy` | `sudo apt install ros-jazzy-joy` |
| `micro_ros_agent` | สื่อสารกับบอร์ด manipulator | build จาก source (ข้อ 1.2) |
| Python package `odrive` (0.6.x) | `transporter_odrive` | `pip install odrive` (ข้อ 1.3) |
| STM32CubeIDE + Docker | ใช้เฉพาะตอน build/แฟลชเฟิร์มแวร์ | [st.com](https://www.st.com/en/development-tools/stm32cubeide.html) |

### 1.2 micro-ROS agent

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/microros_ws/src && cd ~/microros_ws
git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build && source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
echo "source ~/microros_ws/install/local_setup.bash" >> ~/.bashrc
```

### 1.3 สิทธิ์ USB ของ ODrive

```bash
pip install --upgrade odrive          # บน Ubuntu 24.04 ให้เติม --break-system-packages หรือใช้ venv
sudo "$(which odrivetool)" udev-setup  # ให้เข้าถึง USB ได้โดยไม่ต้องใช้ sudo
```
ใช้ `odrivetool` เช็กว่าเห็น ODrive ครบทั้ง 4 ตัว และ serial number ต้องตรงกับ `wheel_serials` ใน [mobile.yaml](src/transporter_bringup/config/mobile.yaml)

### 1.4 สิทธิ์พอร์ตซีเรียลและจอย

```bash
sudo usermod -aG dialout,input $USER   # จากนั้น log out แล้ว log in ใหม่
```
ตั้งชื่อพอร์ตของบอร์ด manipulator ให้คงที่ด้วย udev rule เช่น สร้างไฟล์ `/etc/udev/rules.d/99-transporter.rules` แล้วแทน ID ด้วยค่าที่ได้จาก `udevadm info -a /dev/ttyACM0`
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", ATTRS{serial}=="<board serial>", SYMLINK+="ttyTransporterLeftMani"
```
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```
จับคู่จอย PS5 ผ่าน Bluetooth (กด **PS + Create** ค้างไว้จนไฟกะพริบ) หรือต่อสาย USB ก็ได้

### 1.5 Build workspace

```bash
git clone git@github.com:SOLAR-NEXT-GEN/transporter_ws.git
cd transporter_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

> **ROS_DOMAIN_ID:** เฟิร์มแวร์ manipulator ใช้ domain **124** ทุก terminal ต้อง `export ROS_DOMAIN_ID=124` (หรือใส่ไว้ใน `~/.bashrc`)

### 1.6 เฟิร์มแวร์ (บอร์ด manipulator)

เปิด `firmware/transport_mani` ด้วย STM32CubeIDE แล้ว build และแฟลชลงบอร์ด ไลบรารี micro-ROS แบบ static จะถูกสร้างผ่าน Docker ตอน build ครั้งแรก การตั้งค่าขาและตารางการทำงานของ GPIO ดูได้ที่[หัวข้อ 5](#5-เฟิร์มแวร์-manipulator)

---

## 2. แพ็กเกจใน workspace

| แพ็กเกจ | หน้าที่ |
|---|---|
| `transporter_joy` | แปลงค่าจากจอย PS5 (`/joy`) เป็น `cmd_vel` คำสั่ง manipulator และคำขอ index search ของ ODrive มีระบบ deadman และระบบป้องกันกรณีจอยหลุด |
| `transporter_mobile` | คำนวณ kinematics ของฐานล้อแบบ differential/skid-steer: `cmd_vel` → ความเร็วล้อซ้าย/ขวา (`wheels/cmd`) |
| `transporter_odrive` | ไดรเวอร์ USB ของ ODrive 4 ตัว: index search, arm, ควบคุมความเร็ว, watchdog และจัดการ fault |
| `transporter_manipulator` | ส่งต่อ `manipulator/cmd` ไปยังเฟิร์มแวร์ (`manipulator/hw_cmd`) ด้วยอัตราคงที่ และสั่ง HOLD เมื่อไม่มีคำสั่งเข้ามาตามเวลาที่กำหนด |
| `transporter_bringup` | ไฟล์ launch และไฟล์ตั้งค่าทั้งหมด (`config/*.yaml`) |
| `firmware/transport_mani` | เฟิร์มแวร์ STM32G474 + micro-ROS ควบคุม GPIO ของ manipulator (ปั๊ม, ขึ้น/ลงของแต่ละข้าง) |

### Topic และ Service

| ชื่อ | ชนิด | จาก → ไป |
|---|---|---|
| `/joy` | `sensor_msgs/Joy` | `joy_node` → `transporter_joy` |
| `/cmd_vel` | `geometry_msgs/Twist` | `transporter_joy` → `mobile_controller` |
| `/wheels/cmd` | `std_msgs/Float64MultiArray` `[ซ้าย, ขวา]` rad/s | `mobile_controller` → `odrive_node` |
| `/wheels/state` | `std_msgs/Float64MultiArray` `[ซ้าย, ขวา]` rad/s | `odrive_node` |
| `/odrive/state` | `std_msgs/String` | `odrive_node` (`DISCONNECTED`, `IDLE`, `INDEX_SEARCH`, `ARMING`, `ARMED`) |
| `/odrive/index_search` | `std_srvs/Trigger` | ทำ index search ทุกล้อ แล้ว arm |
| `/odrive/idle`, `/odrive/clear_errors` | `std_srvs/Trigger` | disarm / ล้าง error |
| `/manipulator/cmd` | `std_msgs/Int8MultiArray` `[ซ้าย, ขวา]` | `transporter_joy` → `manipulator_controller` |
| `/manipulator/hw_cmd` | `std_msgs/Int8MultiArray` `[ซ้าย, ขวา]` | `manipulator_controller` → เฟิร์มแวร์ |
| `/manipulator/hw_state` | `std_msgs/Int8MultiArray` `[ซ้าย, ขวา]` | เฟิร์มแวร์ → `manipulator_controller` |

ค่าคำสั่ง manipulator: `1` = ขึ้น, `-1` = ลง, `0` = ค้างไว้ (hold)

```
PS5 ─ joy_node ─/joy─ transporter_joy ─┬─/cmd_vel─ mobile_controller ─/wheels/cmd─ odrive_node ─USB─ ODrive 4 ตัว
                                       └─/manipulator/cmd─ manipulator_controller ─/manipulator/hw_cmd─ micro_ros_agent ─UART─ transport_mani
```

---

## 3. วิธี Launch

| คำสั่ง | สิ่งที่รัน |
|---|---|
| `ros2 launch transporter_bringup mobile.launch.py` | ฐานล้ออย่างเดียว: joy + mobile controller + ODrive |
| `ros2 launch transporter_bringup manipulator.launch.py` | manipulator อย่างเดียว: joy + manipulator controller + micro-ROS agent |
| `ros2 launch transporter_bringup manual.launch.py` | ทั้งสองระบบพร้อมกัน ใช้จอยตัวเดียว |

Launch arguments:

| Argument | ค่าเริ่มต้น | ใช้กับ |
|---|---|---|
| `mani_device` | `/dev/ttyTransporterLeftMani` | manipulator, manual |
| `mani_baudrate` | `115200` | manipulator, manual |
| `use_joy` | `true` | mobile, manipulator (ตั้งเป็น `false` ถ้ามี `joy_node` รันอยู่แล้ว) |

ตัวอย่าง: `ros2 launch transporter_bringup manual.launch.py mani_device:=/dev/ttyACM0`

ไฟล์ตั้งค่าอยู่ที่ [src/transporter_bringup/config](src/transporter_bringup/config)

| ไฟล์ | เนื้อหา |
|---|---|
| `joy.yaml` | การแมปปุ่ม และความเร็ว (0.2 m/s, 0.1 rad/s) |
| `mobile.yaml` | ขนาดล้อ และค่าของ ODrive: serial, ทิศทาง (sign), อัตราทด, ขีดจำกัด |
| `manipulator.yaml` | อัตราการส่งคำสั่ง และ timeout |

---

## 4. วิธีควบคุม (จอย PS5)

**L1 คือ deadman** ถ้าไม่กด L1 ค้างไว้ หุ่นจะไม่ขยับ

| ปุ่ม | การทำงาน |
|---|---|
| **L1** (กดค้าง) | Deadman: เปิดให้ใช้คำสั่งด้านล่างทั้งหมด |
| L1 + **D-pad ↑ / ↓** | เดินหน้า / ถอยหลัง (0.2 m/s) |
| L1 + **D-pad ← / →** | หมุนซ้าย / ขวา (0.1 rad/s) |
| L1 + **PS** | ทำ ODrive index search แล้ว arm **ต้องทำก่อนขับทุกครั้ง** |
| L1 + **△ สามเหลี่ยม** | manipulator ซ้าย **ขึ้น** |
| L1 + **□ สี่เหลี่ยม** | manipulator ซ้าย **ลง** |
| L1 + **○ วงกลม** | manipulator ขวา **ขึ้น** |
| L1 + **✕ กากบาท** | manipulator ขวา **ลง** |
| ไม่กดอะไร / ปล่อย L1 | หยุด และ manipulator ทั้งสองข้าง **ค้างไว้ (hold)** |

### ขั้นตอนเริ่มใช้งาน (ฐานล้อ)
1. สั่ง launch แล้วรอจนขึ้น `All ODrives connected` ถ้าเจอ ODrive ไม่ครบ 4 ตัว หุ่นจะ **ไม่** arm
2. เชื่อมต่อจอย แล้ว **ปล่อย L1 หนึ่งครั้ง** ทุกครั้งที่จอยเชื่อมต่อหรือเชื่อมต่อใหม่ ระบบจะล็อกไว้จนกว่าจะปล่อย L1
3. ยกล้อขึ้นหรือเคลียร์พื้นที่ให้ปลอดภัย แล้วกด **L1 + PS** รอจนขึ้น `Index search done on all wheels` → `ARMED`
4. กด L1 ค้างไว้ แล้วขับด้วย D-pad

### ระบบความปลอดภัย
| เหตุการณ์ | ผลลัพธ์ |
|---|---|
| ODrive เชื่อมต่อไม่ครบ 4 ตัว | ทำ index search หรือ arm ไม่ได้ |
| ยังไม่ได้ทำ index search หรือทำแล้วล้มเหลว | arm ไม่ได้ คำสั่งขับจะถูกเพิกเฉยพร้อมขึ้นคำเตือน |
| มีล้อใดล้อหนึ่งหลุดจาก closed loop | ทุกล้อเข้า idle ต้อง **ทำ index search ใหม่** (L1 + PS) |
| USB ของ ODrive หลุดหรือเชื่อมต่อใหม่ | ทุกล้อเข้า idle เชื่อมต่อใหม่อัตโนมัติ แล้วต้อง **ทำ index search ใหม่** |
| จอยหลุด (ไม่มี `/joy` นานเกิน 0.5 วินาที) | หยุดและ hold เมื่อจอยกลับมาต้องปล่อย L1 ก่อน |
| `transporter_joy` / `mobile_controller` หยุดทำงาน | ความเร็วเป็นศูนย์ภายใน 0.5 วินาที |
| `odrive_node` ดับ | watchdog ของ ODrive หยุดมอเตอร์ภายใน 0.5 วินาที |
| คำสั่ง manipulator ขาดหาย | HOLD ภายใน 0.3 วินาที (ฝั่ง ROS) และ 500 ms (ฝั่งเฟิร์มแวร์) |

> หมายเลขปุ่มเป็นค่าของแพ็กเกจ `joy` ใน ROS 2 (SDL) เช็กได้ด้วย `ros2 topic echo /joy` และแก้ได้ใน `joy.yaml` ถ้าไม่ตรง

---

## 5. เฟิร์มแวร์ manipulator

| คำสั่ง | PUMP_EN | SIDE_UP | SIDE_DOWN |
|---|---|---|---|
| ขึ้น (Up) | HIGH | HIGH | LOW |
| ลง (Down) | HIGH | LOW | HIGH |
| ค้าง (Hold) | LOW | LOW | LOW |

- ปั๊มจะทำงานตลอดเวลาที่มีข้างใดข้างหนึ่งกำลังขยับ และขา UP กับ DOWN ของข้างเดียวกันจะไม่เป็น HIGH พร้อมกันเด็ดขาด
- ถ้าไม่ได้รับคำสั่งนานเกิน 500 ms ทั้งสองข้างจะเข้า HOLD
- ขาที่ใช้ตอนนี้เป็น **ค่าชั่วคราว** ใน `Core/Inc/config.h` (PC0, PC1, PC4, PA10, PB6) ตอนตั้งค่าขาใน CubeMX ให้ตั้ง user label เป็น `PUMP_EN`, `LEFT_UP`, `LEFT_DOWN`, `RIGHT_UP`, `RIGHT_DOWN` แล้ว define ที่ CubeMX สร้างขึ้นจะถูกใช้แทนค่าชั่วคราวโดยอัตโนมัติ
