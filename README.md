| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C6 | ESP32-H2 | ESP32-P4 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- | -------- | -------- |

# WTC

WTC (Wheel Tick Converter) is an innovative solution designed for high-precision speed and distance measurement systems in locomotive applications. Utilising the ESP32 microcontroller, this project bridges real-world wheel tick signals with digital processing, enabling the generation of synthetic wheel ticks compliant with the uBlox ZED-F9R's specifications.

By directly interfacing with opto-isolated inputs, WTC bypasses a software-based odometer input (ADR), offering a more discrete, hardware-centric approach that enhances accuracy, reduces latency, and streamlines integration. Ideal for lab setups, not tested in production.

This document serves as a comprehensive summary of the Wheel Tick Specification for the integration project involving the ESP32 microcontroller and the ZED-F9R module. Our goal is to establish a hardware-based solution for capturing and processing wheel tick signals to accurately measure speed and distance in locomotive applications.

1. Objective

The primary objective of this specification is to provide a clear and effective method for converting real wheel tick signals into synthetic signals that are compatible with the ZED-F9R's input requirements, thereby enhancing accuracy and reliability in speed and distance measurements.

2. System Overview

Components: The system comprises two main components: ESP32 #1 (Wheel Tick Generator) and ESP32 #2 (Wheel Tick Mediator).
Functionality: ESP32 #1 simulates real wheel tick signals, while ESP32 #2 processes these signals and generates synthetic wheel ticks to interface directly with the ZED-F9R module.

3. Signal Requirements

Minimum Pulse Width: 10 microseconds, ensuring the ZED-F9R module can accurately detect each wheel tick.
Maximum Wheel Tick Resolution: Optimal performance is achieved with resolutions less than 5 cm, with a maximum support of up to 40 cm.
Signal Voltage Level: Must not exceed the maximum voltage level specified for the ZED-F9R's WT input pins to prevent damage.
Edge Detection: The system defaults to detecting the rising edge of the wheel tick signal but can be configured to detect both rising and falling edges for higher resolution.

4. Implementation Details

Hardware Integration: Utilizes the existing B2W opto-isolated inputs on the ZED-F9R for direct signal input, bypassing software-based odometer inputs.
Signal Generation: The Wheel Tick Mediator (ESP32 #2) is responsible for adjusting the output based on the real wheel tick intervals received, ensuring compliance with the ZED-F9R specifications.

5. Configuration and Calibration

Adjustable Parameters: Includes the number of synthetic ticks per real tick and the pulse width of synthetic ticks, allowing for fine-tuning and calibration with the ZED-F9R.
Testing and Validation: Emphasizes the importance of thorough testing and calibration to ensure signal integrity and system reliability.

6. Conclusion

The Wheel Tick Specification outlines a robust and accurate approach to wheel tick signal processing, leveraging the capabilities of the ESP32 microcontroller. By adhering to these specifications, the project aims to achieve precise speed and distance measurements, vital for advanced locomotive applications.

This summary encapsulates the essential aspects of the Wheel Tick Specification, serving as a guide for the project's development and implementation phases.
