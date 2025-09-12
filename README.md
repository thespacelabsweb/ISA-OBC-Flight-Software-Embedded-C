# ISA Flight Software - Team Collaboration

## Project Overview

ISA Flight Software is an embedded system for a precision-guided projectile. The system follows a classic GNC (Guidance, Navigation, Control) architecture and is designed to run on an ATMEL SAM V71 microcontroller (ARM Cortex-M7 with FPU).

The software consists of four primary modules:

1. **Sequencer Module**: Manages mission phases and state transitions
2. **Navigation Module**: Processes sensor data and determines position/attitude
3. **Guidance Module**: Implements trajectory planning and guidance algorithms
4. **Digital Autopilot (DAP)**: Implements control laws and generates actuator commands

## Hardware Requirements

- ATMEL SAM V71 microcontroller (or SAMV71 Xplained Ultra Evaluation Kit)
- PC running Windows 7 or higher
- MPLAB X IDE (for microcontroller deployment)
- XC32 Compiler (for ARM Cortex-M7)
- Debugging tools: EDBG, SAM-ICE (J-Link), or equivalent

## Directory Structure

```
ISA Flight Software/
  - include/            # Header files
    - common/           # Common definitions and types
    - dap/              # Digital Autopilot headers
    - guidance/         # Guidance module headers
    - hal/              # Hardware Abstraction Layer headers
    - math/             # Mathematical utilities headers
    - navigation/       # Navigation module headers
    - sequencer/        # Sequencer module headers
  - src/                # Source code
    - dap/              # Digital Autopilot implementation
    - guidance/         # Guidance module implementation
    - hal/              # Hardware Abstraction Layer implementation
    - math/             # Mathematical utilities implementation
    - navigation/       # Navigation module implementation
    - sequencer/        # Sequencer module implementation
  - test/               # Test files
  - instruction/        # Documentation
  - requirement_doc/    # Requirement documents
  - reference code/     # Reference code (MATLAB, C++)
  - main.c              # Main entry point for full system simulation
  - main_DAP.c          # Main entry point for DAP module test
  - Makefile            # Build script for the full system
  - Makefile.dap        # Build script for DAP module test
  - Makefile.sequencer  # Build script for Sequencer module test
```

## Building and Running on PC

### Prerequisites

- GCC compiler (C99 compatible)
- Make utility
- Math library (libm)

### Building the Full System

```bash
make
```

This will compile all source files and create the executable at `build/isa_flight_software`.

### Running the Full System

```bash
make run
```

This will build (if needed) and then run the executable.

### Running Individual Module Tests

#### Guidance Module Test

```bash
make -f Makefile.guidance
./build/guidance_test
```

#### DAP Module Test

```bash
make -f Makefile.dap
./build/dap_test
```

#### Sequencer Module Test

```bash
make -f Makefile.sequencer
./build/sequencer_test
```

### Running Unit Tests

```bash
make tests
```

This will compile and run all test files in the test directory.

### Cleaning Build Files

```bash
make clean
```

## Deploying to SAM V71 Microcontroller

### Using MPLAB X IDE

1. **Create a new project**:

   - File > New Project
   - Select "Microchip Embedded" > "Standalone Project"
   - Select Device: SAM > ATSAMV71Q21 (or your specific variant)
   - Select Tool: EDBG (or your specific debugger)
   - Select Compiler: XC32
   - Enter project name and location

2. **Add source files**:

   - Right-click on project > "Add Existing Items from Folders"
   - Add all necessary source files from the src/ directory
   - Add main.c or the specific module test file you want to run

3. **Configure project**:

   - Right-click on project > "Properties"
   - Set appropriate compiler and linker options
   - Configure clock settings for SAM V71 (300 MHz)

4. **Build and program**:
   - Click "Build Main Project" (F11)
   - Click "Make and Program Device" (F6)

### Using SAM-BA

1. **Put the SAM V71 in bootloader mode**:

   - Close jumper J200 (ERASE)
   - Power cycle the board
   - Wait 5 seconds
   - Open jumper J200
   - Power cycle the board again

2. **Use SAM-BA to program your compiled binary**:
   ```
   sam-ba -p <port> write <filename> <start-address>
   ```

## Debugging

### Serial Output

The software uses UART for debug output. Connect to the EDBG Virtual COM Port with:

- Baud rate: 115200
- Data bits: 8
- Parity: None
- Stop bits: 1

### LED Indicators

When deployed on the SAM V71 Xplained Ultra board:

- LED0 (PA23): Indicates system status
- LED1 (PC9): Indicates algorithm execution

## Module Details

### Guidance Module

The guidance module implements a proportional navigation guidance law with impact angle control. It calculates acceleration commands based on current state and target position, accounting for drag and gravitational effects.

### DAP Module

The Digital Autopilot implements control laws for roll, pitch, and yaw. It generates actuator commands for the canards based on the guidance commands and current attitude.

### Navigation Module

The navigation module processes IMU and GNSS data to determine position, velocity, and attitude. It performs coordinate transformations between various reference frames.

### Sequencer Module

The sequencer manages mission phases and state transitions from pre-launch to impact, handling timing and sensor inputs to execute the flight sequence.

## Mathematical Utilities

The project includes several mathematical utilities:

- Vector3: 3D vector operations
- Matrix3: 3x3 matrix operations
- Quaternion: Quaternion operations for attitude representation
- Coordinate Transforms: Transformations between reference frames

## Contributing

1. Follow the embedded C programming guidelines in the repository.
2. Maintain consistent code style with the existing codebase.
3. Write unit tests for new functionality.
4. Document your code with Doxygen-style comments.

## License

This software is proprietary and confidential. Unauthorized copying, transfer, or reproduction of the contents is strictly prohibited.
