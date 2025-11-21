# RFID Capture Relay - ESP32-S3 Project

This project captures RFID data from a USB HID RFID reader connected to an ESP32-S3 and relays it to another ESP32 via ESP-NOW protocol.

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