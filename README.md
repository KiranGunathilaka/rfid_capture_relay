# RFID Capture Relay - ESP32-S3 Project

This project captures RFID data from a USB HID RFID reader connected to an ESP32-S3 and relays it to another ESP32 via ESP-NOW protocol.

## Project Structure

```
rfid_capture_relay/
├── CMakeLists.txt
├── main/
│   ├── CMakeLists.txt
│   └── main.c
├── components/
│   └── hid/
│       ├── CMakeLists.txt
│       ├── include/
│       │   ├── hid_host.h
│       │   ├── hid_usage_keyboard.h
│       │   └── hid_usage_mouse.h
│       ├── hid_host.c
│       ├── hid_usage_keyboard.c
│       └── hid_usage_mouse.c
├── sdkconfig.defaults
├── .gitignore
└── README.md
```

## Prerequisites

### For Windows Users

1. **ESP-IDF Installation**
   - Download and install ESP-IDF from [Espressif's official installer](https://dl.espressif.com/dl/esp-idf/)
   - The installer should create "ESP-IDF Command Prompt" in your Start Menu

2. **Hardware Requirements**
   - ESP32-S3 development board
   - USB HID RFID reader
   - USB cables for connection

3. **Driver Installation** (if needed)
   - Install [CP210x USB to UART Bridge drivers](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers)

### For Linux/Mac Users

1. **ESP-IDF Installation**
   - Follow the [official ESP-IDF installation guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html)

## Initial Setup (One-time)

### Windows

1. **Clone this repository**
   ```cmd
   git clone <repository-url>
   cd rfid_capture_relay
   ```

2. **Open ESP-IDF Command Prompt**
   - Press Windows Key
   - Search for "ESP-IDF Command Prompt" or "ESP-IDF 5.1 CMD"
   - Click to open

3. **Navigate to project directory**
   ```cmd
   cd C:\path\to\your\rfid_capture_relay
   ```

4. **Set target device**
   ```cmd
   idf.py set-target esp32s3
   ```

5. **Configure project (optional)**
   ```cmd
   idf.py menuconfig
   ```

### Linux/Mac

1. **Clone this repository**
   ```bash
   git clone <repository-url>
   cd rfid_capture_relay
   ```

2. **Set up ESP-IDF environment**
   ```bash
   source $HOME/esp/esp-idf/export.sh
   ```

3. **Set target device**
   ```bash
   idf.py set-target esp32s3
   ```

4. **Configure project (optional)**
   ```bash
   idf.py menuconfig
   ```

## Daily Development Workflow

### Every New Terminal Session (Windows)

**Method 1: Use ESP-IDF Command Prompt (Recommended)**
1. Open "ESP-IDF Command Prompt" from Start Menu
2. Navigate to your project:
   ```cmd
   cd C:\path\to\your\rfid_capture_relay
   ```

**Method 2: Use PowerShell/Command Prompt**
1. Open PowerShell or Command Prompt
2. Navigate to your project directory
3. Set up environment:
   ```cmd
   # For Command Prompt
   ..\esp-idf\export.bat
   
   # For PowerShell (if execution policy is set)
   ..\esp-idf\export.ps1
   ```

### Every New Terminal Session (Linux/Mac)

```bash
# Navigate to project
cd /path/to/your/rfid_capture_relay

# Set up ESP-IDF environment
source $HOME/esp/esp-idf/export.sh
# OR if ESP-IDF is in different location:
# source /path/to/esp-idf/export.sh
```

## Common Development Commands

### Building and Flashing

```bash
# Clean build (removes build directory)
idf.py clean

# Build the project
idf.py build

# Flash to device (device must be connected)
idf.py flash

# Monitor serial output
idf.py monitor

# Flash and immediately start monitoring
idf.py flash monitor

# Build, flash, and monitor in one command
idf.py build flash monitor
```

### Configuration Commands

```bash
# Set target device (run once per project)
idf.py set-target esp32s3

# Open configuration menu
idf.py menuconfig

# Reconfigure project (apply sdkconfig.defaults)
idf.py reconfigure

# Show current configuration
idf.py show-efuse-table
```

### Port and Device Management

```bash
# Flash to specific port (Windows)
idf.py -p COM3 flash

# Flash to specific port (Linux/Mac)
idf.py -p /dev/ttyUSB0 flash

# Monitor specific port
idf.py -p COM3 monitor

# List available ports (Windows)
# Check Device Manager or:
mode

# List available ports (Linux/Mac)
ls /dev/tty*
```

### Debugging and Analysis

```bash
# Generate compilation database for IDEs
idf.py build

# Check project size
idf.py size

# Analyze memory usage
idf.py size-components

# Analyze binary size
idf.py size-files

# Generate documentation
idf.py docs

# Run unit tests (if any)
idf.py test
```

### Advanced Commands

```bash
# Erase flash completely
idf.py erase-flash

# Read MAC address
idf.py read-flash 0x1000 0x6 mac.bin

# Partition table info
idf.py partition-table

# Application info
idf.py app-info

# Bootloader info
idf.py bootloader-info
```

## Monitor Controls

When using `idf.py monitor`:

- **Ctrl + ]** - Exit monitor
- **Ctrl + T, Ctrl + H** - Show help menu
- **Ctrl + T, Ctrl + R** - Reset ESP32
- **Ctrl + T, Ctrl + F** - Toggle log output filtering
- **Ctrl + T, Ctrl + A** - Print available filters

## Project Configuration

### Key Configuration Options

Access via `idf.py menuconfig`:

- **Component config → USB Host Library** - USB host settings
- **Component config → FreeRTOS** - RTOS configuration
- **Component config → Wi-Fi** - ESP-NOW settings
- **Serial flasher config** - Flash settings
- **Partition Table** - Memory layout

### Important sdkconfig Settings

```ini
# USB Host Configuration
CONFIG_USB_HOST_ENABLED=y
CONFIG_USB_HOST_HW_BUFFER_BIAS_IN=y

# FreeRTOS Configuration  
CONFIG_FREERTOS_HZ=1000

# ESP32-S3 USB Configuration
CONFIG_TINYUSB_ENABLED=y
CONFIG_TINYUSB_HOST_ENABLED=y

# Logging Configuration
CONFIG_LOG_DEFAULT_LEVEL_INFO=y
CONFIG_LOG_MAXIMUM_LEVEL_VERBOSE=y
```

## Troubleshooting

### Common Issues

1. **"idf.py not found"**
   - Make sure ESP-IDF environment is set up (`export.sh` or `export.bat`)
   - Use ESP-IDF Command Prompt on Windows

2. **Permission denied on port**
   ```bash
   # Linux/Mac: Add user to dialout group
   sudo usermod -a -G dialout $USER
   # Then logout and login
   ```

3. **Build errors**
   ```bash
   # Clean and rebuild
   idf.py clean
   idf.py build
   ```

4. **Flash errors**
   - Check if ESP32 is properly connected
   - Try different USB cable
   - Verify correct COM port
   - Hold BOOT button while flashing if needed

### Getting Help

```bash
# Show all available commands
idf.py --help

# Show help for specific command
idf.py build --help
idf.py flash --help
```

## Support

For ESP-IDF related issues, check:
- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/)
- [ESP-IDF GitHub Issues](https://github.com/espressif/esp-idf/issues)
- [Espressif Community Forum](https://esp32.com/)