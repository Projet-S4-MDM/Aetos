
# Electrical Assembly

## Build of Material

| Item | Description | Quantity | Unit Price | Total Price |
|------|------------|----------|------------|-------------|
| 1    | Freenove ESP32-S3-WROOM Board Lite | 1  | $15.00  | $15.00  |
| 2    |FIT0186 DC Motor| 4  | $26.59 | $106.36 |
| 3    | Talon SRX Motor Controller | 4  | $131.00 | $524.00 |
| 4    | 35A Terminal Block 6 Position Screw | 2  | $7.00 | $14.00 |
| 5    | Emergency Stop Push Button 10A | 1  | $17.00 | $17.00 |

**Total:** 676.00$

**Note:** We are using Talon SRX motor controllers because we had them for free. They are largely overkill for our application, as they offer advanced features like PID control, current sensing, and CAN bus communication, which are not strictly necessary for our setup. If you are building a similar system from scratch, a simpler and more cost-effective motor driver would be a better choice.

![Motor Driver](images/schematic.png)