# t1tan-strik3rs
The Soldier Health Monitoring and Position Tracking System allows the military personnel to track the current GPS position of a soldier and also checks the health status by detecting heartbeat of a soldier in realtime.


## **_Soldiers Health Monitoring and GPS Tracking System_**

#### **Introduction**

Modern warfare demands real-time solutions for soldier safety. Soldiers risk injury and getting lost during missions. Delayed medical attention or search efforts can be deadly and can jeopardize national security. This project proposes a soldier health and location tracking system. Sensors monitor vitals (heart rate, temperature) and location (GPS). Wireless communication transmits data to a Blynk application accessed on the PC. This unit tracks soldier location and health, triggering alerts for abnormal readings. This system can significantly improve soldier safety by ensuring faster medical response for injured personnel and reducing search times for lost soldiers. However, further considerations include data security, low-power sensor technology, and system scalability for large-scale deployment.

#### **Overview**

The Smart Soldier Gear project, integrated with VSD Squadron technology, enhances soldier safety and operational efficiency through advanced wearable systems. The gear includes a pulse sensor (RC-A-4015) to monitor vital health parameters continuously. Any deviations from predefined thresholds trigger emergency alerts. A GPS module tracks the soldier’s real-time location. Data from these sensors are processed by a microcontroller and transmitted wirelessly via the HC12 RF transceiver directly to the Blynk cloud using the Blynk application. This setup enables real-time data visualization on a customized Blynk dashboard, ensuring continuous health and location monitoring and facilitating immediate responses in emergencies. This system significantly improves the situational awareness and operational readiness of soldiers.

#### **Components Required**

- VSD Squadron Mini developement board with CH32V003F4U6 chip with 32-bit RISC-V core based on RV32EC instruction set
- RC-A-4015 Pulse Sensor
- Neo-6M GPS Receiver
- Bread Board
- Jumper Wires

#### **Software Required**

- Python (libraries like folium, gmplot) - [folium](https://pypi.org/project/folium/), [gmplot](https://pypi.org/project/gmplot/) 
- Arduino IDE - download from [Here](https://www.arduino.cc/en/software)


#### **Circuit Connection Diagram**

![ckt-diagram](https://github.com/BipinRajC/t1tan-strik3rs/assets/112572356/b6ac5f6f-426d-4c1e-9256-e7cd57292ebc)


#### **Table for Pin Connection**

| RC-A-4015 Pulse Sensor | VSD Squadron Mini |
| ---------------------- | ----------------- |
| GND                    | GND               |
| VCC                    | 5V                |
| A0                     | PA1               |

| Neo-6M GPS Receiver | VSD Squadron Mini |
| ------------------- | ----------------- |
| GND                 | GND               |
| VCC                 | 5V                |
| TX                  | PD6 (RX)          |
| RX                  | PD5 (TX)          |

# Demo

This is an example video:

https://github.com/BipinRajC/t1tan-strik3rs/assets/112572356/f391d5fc-bfee-4adb-9523-49ef7cf40934

_Note: GPS module has stability issue to lock onto satellite, was not able to interface it with given time constraint, we are actively working on a fix_ <br>

_Advantages: Keeping in mind the ultimate goal of this hackathon, that is fault injection, we believe that this could see it's applications in military wherein soldiers using such monitoring systems can keep the central unit updated about their positions and health vitals and in case of other countries' intelligence agencies trying to interfere with such systems, our aim is to pentest it and find a viable solution to safeguard such systems and keep our defences non-vulnerable and safe._

## Code

```cpp
// Define constants for EMA filter
const float alpha = 0.2;  // Smoothing factor (adjust as needed, between 0 and 1)
float sensorValueFiltered = 0;  // Initial filtered sensor value

// Define the pin where the pulse sensor is connected
const int pulsePin = A0;

void setup() {
  // Initialize serial communication at 9600 bits per second
  Serial.begin(9600);
}

void loop() {
  // Read the value from the pulse sensor
  int sensorValueRaw = analogRead(pulsePin);

  // Apply EMA filter
  sensorValueFiltered = alpha * sensorValueRaw + (1 - alpha) * sensorValueFiltered;

  // Print the filtered sensor value to the Serial Monitor and Serial Plotter
  Serial.println(sensorValueFiltered);

  // Wait for a short period before reading the value again
  delay(1000); // Adjust the delay as needed
}

```

**_Team Members_** - _Bipin Raj C_ , _B Jnyanadeep_ <br>
**_College_** - _RV College of Engineering_ 




