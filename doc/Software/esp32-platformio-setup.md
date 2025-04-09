# ESP32-S3 Setup with PlatformIO
This guide explains how to set up PlatformIO to develop for the ESP32-S3-DevKitC-1 using the Arduino framework.

---
## Prerequisites

- [Visual Studio Code](https://code.visualstudio.com/)
- Internet connection
---
## 1. Install PlatformIO

1. Open **Visual Studio Code**
2. Go to the **Extensions** tab (or press `Ctrl+Shift+X`)
3. Search for **"PlatformIO IDE"** and install it
4. Restart VS Code if prompted
---
## 2. Open the Project

Open the project by opening the `aetos-micro` folder with PlatformIO:

1. Launch **Visual Studio Code**
2. Click on **File > Open Folder...**
3. Select the `aetos-micro` folder that contains the code and the `platformio.ini` file
4. PlatformIO will automatically recognize and load the project
---
## 3. How to Upload to the ESP32

To flash the code to your ESP32-S3 board:
### Step-by-step:

1. Connect your ESP32-S3 board to your computer via USB
2. Make sure the board is detected by your system
3. In Visual Studio Code, locate the PlatformIO toolbar at the bottom
4. Click the right arrow icon (labeled `Upload`) to compile and flash the code to the board
5. Wait for the upload to complete
---
### platformio.ini Configuration

Ensure your `platformio.ini` file looks like this:

```ini
[env:esp32-s3-devkitc-1]
platform = https://github.com/Jason2866/platform-espressif32.git#Arduino/IDF5
board = esp32-s3-devkitc-1
framework = arduino

monitor_speed = 115200
monitor_raw = true