# MX5 NC Brain ESP32

ESP32-based telemetry system for Mazda MX5 NC using ESP-NOW wireless protocol for real-time vehicle data broadcasting.

## Overview

This is a telemetry transmitter (master) that reads vehicle data from multiple sources and broadcasts it wirelessly using the ESP-NOW protocol. The system collects data from:
- **CAN Bus**: Reading vehicle data directly from the car's CAN network
- **Physical Sensors**: Oil pressure and temperature sensors connected to analog inputs
- **Future ESP-NOW Nodes**: Other ESP32 devices can send additional sensor data to this master unit

The system uses a broadcast architecture, allowing multiple receivers to listen to the same data stream without requiring a Wi-Fi router.

## Features

- **Multi-Source Data Collection**:
  - CAN bus integration for reading vehicle network data
  - Direct analog sensor inputs (oil pressure, oil temperature)
  - ESP-NOW receiver capability for data from remote sensor nodes
- **Wireless Broadcasting**:
  - Low-latency ESP-NOW wireless communication
  - Broadcast architecture supporting multiple receivers
  - No Wi-Fi router required
- **Performance**:
  - Configurable update rate (default: 10Hz)
  - Low power consumption
  - Real-time data aggregation

## Telemetry Data

The system tracks and transmits the following metrics:

### From CAN Bus
- **Water Temperature** (°C) - Engine coolant temperature
- **Engine RPM** - Engine speed
- **Brake Pressure** (bar) - Brake system pressure
- **Brake Percentage** (%) - Brake pedal position
- **Throttle Position** (%) - Throttle body position
- **Vehicle Speed** (km/h) - Vehicle speed from ABS/ECU
- **Accelerator Position** (%) - Accelerator pedal position

### From Physical Sensors
- **Oil Temperature** (°C) - Direct sensor reading
- **Oil Pressure** (bar) - Direct sensor reading

### From Remote ESP-NOW Nodes (Future)
- Additional sensor data from distributed nodes throughout the vehicle

## Hardware Requirements

### Core Components
- ESP32 development board (with CAN bus support recommended)
- CAN bus transceiver module (e.g., MCP2515, TJA1050, or ESP32 with built-in TWAI)
- Power supply (12V vehicle power to 5V/3.3V regulation recommended)

### Sensors
- Oil pressure sensor (analog output)
- Oil temperature sensor (analog output)
- Additional sensors as needed (expandable)

### Optional
- Additional ESP32 nodes for remote sensor data collection via ESP-NOW

**Note**: Receivers can be any ESP32-based device capable of receiving ESP-NOW broadcasts. Receiver implementation is separate from this transmitter project.

## Software Dependencies

### Libraries Required
- `ESP32_NOW` - ESP-NOW protocol implementation
- `WiFi` - Wi-Fi management
- `esp_mac` - MAC address utilities
- CAN bus library (depending on hardware):
  - `ESP32-TWAI-CAN` - For ESP32 built-in TWAI controller
  - `mcp2515` - For MCP2515 CAN controller modules
  - `arduino-CAN` - Generic CAN library

### Installation
Install via Arduino IDE Library Manager or PlatformIO:
```
ESP32_NOW
ESP32-TWAI-CAN (or appropriate CAN library for your hardware)
```

## Configuration

### Wi-Fi Channel
Set the ESP-NOW channel (default: Channel 6). Any receivers must use the same channel:
```cpp
#define ESPNOW_WIFI_CHANNEL 6
```

### Telemetry Structure
The telemetry data structure broadcasts the following format. Receivers must implement matching structure:
```cpp
typedef struct __attribute__((packed)) {
  float oilTemp;
  float waterTemp;
  uint32_t engineRPM;
  float oilPressure;
  float brakePressure;
  int brakePercent;
  float throttlePos;
  float speed;
  float accelPos;
} TelemetryData;
```

## Usage

### Setup
1. **CAN Bus Connection**:
   - Connect CAN transceiver to ESP32 (TX/RX pins or SPI for MCP2515)
   - Connect CAN-H and CAN-L to vehicle's OBD-II port or direct CAN bus tap
   - Configure CAN bus speed to match vehicle (typically 500 kbps for Mazda)

2. **Sensor Connections**:
   - Connect oil pressure sensor to analog input pin
   - Connect oil temperature sensor to analog input pin
   - Ensure sensors output 0-3.3V (use voltage dividers if necessary)

3. **Software Configuration**:
   - Update CAN message IDs for your specific vehicle
   - Configure sensor reading and mapping logic in `loop()` function
   - Adjust ADC resolution if needed (default: 12-bit)
   - Set sensor calibration values for your specific sensors

4. **Upload**:
   - Upload `mth-car-brain-esp32.ino` to ESP32

### Monitoring
Serial output provides debugging information including:
- Broadcast success/failure status
- Current telemetry values
- Connection status

### Integration
Receivers can be built using any ESP32 device configured to listen on the same ESP-NOW channel. The broadcast architecture allows multiple receivers to monitor the same data stream simultaneously.

## Project Structure

```
mx5-nc-brain-esp32/
├── mth-car-brain-esp32.ino          # Main transmitter sketch
└── test-on-device/                   # Test and development files
```

## Customization

### Adding New Sensors
1. Update `TelemetryData` structure to include new fields
2. Add sensor reading code in `loop()`
3. Update sensor mapping and calibration values
4. Ensure any receivers also update their structure to match

### Adjusting Update Rate
Change delay in `loop()`:
```cpp
delay(100); // 10Hz (100ms between transmissions)
delay(50);  // 20Hz (50ms between transmissions)
delay(200); // 5Hz (200ms between transmissions)
```

### Sensor Calibration
Adjust the mapping values in `loop()` to match your specific sensors:
```cpp
myData.oilTemp = map(sensorValue, 0, 3330, 0, 170); // Adjust min/max values
```

## Troubleshooting

### CAN Bus Issues
- Verify CAN-H and CAN-L connections are correct (not swapped)
- Check CAN bus termination resistors (120Ω at each end)
- Confirm CAN bus speed matches vehicle (typically 500 kbps)
- Ensure vehicle ignition is on for CAN bus activity
- Use CAN bus analyzer tool to verify message traffic
- Check for proper ground connection between ESP32 and vehicle

### Transmitter Not Broadcasting
- Check serial output for "Failed to initialize ESP-NOW"
- Verify Wi-Fi channel configuration
- Ensure ESP32 board is properly powered
- Confirm ESP32_NOW library is correctly installed

### Sensor Reading Issues
- Verify sensor connections to analog pins
- Check ADC resolution setting (default: 12-bit)
- Validate sensor voltage ranges match ESP32 input (0-3.3V)
- Use serial output to monitor raw sensor values
- Check sensor power supply voltage
- Verify sensor ground connection

### Communication Issues
- Ensure Wi-Fi channel is not congested
- Check for interference from other 2.4GHz devices
- Verify adequate power supply to ESP32
- For remote ESP-NOW nodes, check distance and line-of-sight

## Performance

- **Update Rate**: 10Hz (configurable)
- **Range**: ~200m line-of-sight (ESP-NOW typical range)
- **Latency**: <10ms typical
- **Data Size**: 36 bytes per transmission

## Future Enhancements

This project is designed to be expandable with the following planned additions:

### Planned Sensors
- Additional temperature sensors (transmission, differential, brakes)
- G-force/accelerometer sensors
- GPS module for location and speed data
- Fuel level and consumption monitoring
- Tire pressure monitoring

### Distributed Sensor Network
- Multiple ESP32 nodes positioned throughout the vehicle
- Each node collecting local sensor data
- Nodes transmit data to the main brain unit via ESP-NOW
- Main unit aggregates and rebroadcasts complete telemetry package

### Architecture Benefits
- **Scalability**: Easy to add new sensors without rewiring
- **Modularity**: Independent sensor nodes can be added/removed
- **Reliability**: Distributed processing reduces single point of failure
- **Flexibility**: Sensor nodes can be positioned optimally without distance constraints

## License

This project is provided as-is for personal use.

## Contributing

Feel free to submit issues or pull requests for improvements.

## Acknowledgments

Built for Mazda MX5 NC telemetry monitoring using ESP32 platform and ESP-NOW protocol.
