# Electrical Assembly

## Build of Material

| Item | Description | Quantity | Unit Price | Total Price |
|------|------------|----------|------------|-------------|
| 1    | Freenove ESP32-S3-WROOM Board Lite | 1  | $15.00  | $15.00  |
| 2    | Freenove Breakout Board for ESP32 | 1  | $18.00  | 18.00  |
| 3    |FIT0186 DC Motor| 4  | $26.59 | $106.36 |
| 4    | Talon SRX Motor Controller | 4  | $131.00 | $524.00 |
| 5    | 35A Terminal Block 6 Position Screw | 2  | $7.00 | $14.00 |
| 6    | Emergency Stop Push Button 10A | 1  | $17.00 | $17.00 |

**Total:** 694.00$

**Note:** We are using Talon SRX motor controllers because we had them for free. They are largely overkill for our application, as they offer advanced features like current sensing and CAN bus communication, which are not necessary for our setup. If you are building a similar system from scratch, a simpler and more cost-effective motor driver would be a better choice.

## Wiring Overview

- **Motor Power Supply:**  
  We will be using a DC power source rated for a maximum of **6.4A**, connected via an **XT60 connector**. XT60 was chosen as it was readily available, but any connector rated for at least **6.4A** will suffice.

- **ESP32 Power Supply:**  
  The ESP32 board is powered through its **USB-C port**, which also facilitates serial communication.

- **Grounding:**  
  All the **grounds (GND)** of the devices must be connected together at a single point, typically via a **terminal block**. This ensures a common reference for all components, which is crucial for proper operation and avoids ground loops or voltage differences.

- **12V Power Distribution:**  
  Devices that operate on **12V** (such as the DC motors and the Talon SRX motor controllers) should be connected to a **dedicated 12V terminal block**. This keeps the high-power components separate from the low-power ones (like the ESP32) and helps ensure proper voltage distribution to each device.

- **Emergency Stop Button Placement:**  
  The **Emergency Stop (e-stop)** button should be placed between the **12V power source** and the **12V terminal block**. This ensures that if the e-stop is pressed, it will immediately cut off the power to the 12V components, providing an effective safety measure.

---

### Wiring Steps:
1. **Connect the ESP32 to the Breakout Board:**  
   Begin by securely plugging the ESP32 board into the breakout board.

2. **PWM Motor Control:**  
   Next, connect the **PWM pins** of the **Talon motor controllers** to the appropriate GPIO pins on the ESP32.

3. **Encoder Connections:**  
   Afterward, wire the **encoder pins** to the corresponding GPIO pins on the ESP32.

4. **Connect Grounds:**  
   All the **ground (GND)** pins from the ESP32, motor controllers, and other components must be connected to a **single terminal block** to establish a common ground.

5. **Connect 12V Devices:**  
   The devices that need **12V** power (DC motors, Talon SRX controllers, etc.) should be connected to the **12V terminal block**, ensuring they receive the correct voltage.

6. **Place the Emergency Stop Button:**  
   Position the **e-stop** button between the **12V power source** and the **12V terminal block**. This will allow it to cut power to the 12V components if activated.

---

![Motor Driver](images/schematic.png)

--- 

### Additional Recommendations
- Ensure all connections are properly insulated, particularly the power supply and motor controller pins, to prevent short circuits.