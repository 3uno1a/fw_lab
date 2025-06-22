# 🧪 firmware laboratory 

### 📟Project
- study

## ⚙️Dev Env
- **Board:** [STM32F407VGT6 Discovery Kit](https://www.st.com/en/evaluation-tools/stm32f4discovery.html)
- **Schematic:** [mb997-f407vgt6-c01_schematic.pdf](https://github.com/user-attachments/files/19399237/mb997-f407vgt6-c01_schematic.pdf)
- **IDE:** STM32CubeIDE v1.16.0  
- **Firmware Package:** STM32CubeF4 v1.28.1

---

## 🌈To Do
### I2C: Audio Playback
- CS43L22 Audio DAC (on board)
- Generate sound via I2S and DMA

### RTOS: Task Structuring
- Refactor monolithic code into:
  - Button Input Task
  - LED Control Task
  - IMU sensor Data Task


### PWM: LED Strip Control
- WS2812 RGB LED (NeoPixel)
- Drive via DMA + PWM for color effects

---

## ✅Done

### GPIO: Button & LED
- Short press: Toggle green LED ON/OFF  
- Long press (2 sec+): Start blue LED blinking 

### UART: Communication & Command Queue
- Supports command parsing:
  - on, off, toggle, blink start, blink stop
- Commands are placed in a msg queue and processed in LED control task

### SPI: MEMS Sensor
- Communicates with onboard **LIS3DSH** accelerometer
- Reads X/Y/Z axes via SPI1
