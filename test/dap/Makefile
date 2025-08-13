# ISA Digital Autopilot (DAP) Makefile
# Enhanced with dual build support: XC32 for PIC32 verification, GCC for host testing

# Build type selection
ifdef PIC32_BUILD
    CC = xc32-gcc
    CFLAGS = -Wall -Wextra -pedantic -std=c99 -O2 -g -mprocessor=32MX470F512H
    LDFLAGS = -lm
    BUILD_SUFFIX = _pic32
    TARGET_DESC = PIC32
else
    CC = gcc
    CFLAGS = -Wall -Wextra -pedantic -std=c99 -O2 -g
    LDFLAGS = -lm
    BUILD_SUFFIX = _host
    TARGET_DESC = Host
endif

# Directories (relative to this Makefile at test/dap)
ROOT_DIR = ../..
SRC_DIR = $(ROOT_DIR)/src
INCLUDE_DIR = $(ROOT_DIR)/include
TEST_DIR = .
BUILD_DIR = build$(BUILD_SUFFIX)

# Source files
MATH_SRC = $(wildcard $(SRC_DIR)/math/*.c)
# All C files in this dir except the test main will be treated as DAP module sources
DAP_SRC = $(filter-out $(TEST_DIR)/test_DAP.c,$(wildcard $(TEST_DIR)/*.c))
MAIN_SRC = $(TEST_DIR)/test_DAP.c

# Object files
MATH_OBJ = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(MATH_SRC))
DAP_OBJ = $(patsubst $(TEST_DIR)/%.c,$(BUILD_DIR)/dap/%.o,$(DAP_SRC))
MAIN_OBJ = $(BUILD_DIR)/test_DAP.o

# All object files
OBJS = $(MATH_OBJ) $(DAP_OBJ) $(MAIN_OBJ)

# Target executable
TARGET = $(BUILD_DIR)/dap_test

# Default target - compile for host
all: directories $(TARGET)
	@echo "✓ $(TARGET_DESC) build completed successfully"

# Check XC32 availability
check-xc32:
	@echo "Checking for XC32 compiler..."
	@if command -v xc32-gcc >/dev/null 2>&1; then \
		echo "✓ xc32-gcc found: $$(which xc32-gcc)"; \
		echo "✓ Version: $$(xc32-gcc --version | head -1)"; \
	else \
		echo "❌ xc32-gcc not found in PATH"; \
		echo "Add to PATH: export PATH=\"/opt/microchip/xc32/v4.60/bin:\$$PATH\""; \
		echo "Or run: echo 'export PATH=\"/opt/microchip/xc32/v4.60/bin:\$$PATH\"' >> ~/.bashrc"; \
		echo "Then: source ~/.bashrc"; \
		exit 1; \
	fi

# PIC32 verification target
pic32-verify: check-xc32 clean
	@echo "Building for PIC32 verification..."
	$(MAKE) PIC32_BUILD=1 all
	@echo "✓ Code verified for PIC32 compatibility"

# Host testing target  
host-test: clean
	@echo "Building for host testing..."
	$(MAKE) all
	@echo "Running tests..."
	$(MAKE) run

# Create build directories
directories:
	@mkdir -p $(BUILD_DIR)
	@mkdir -p $(BUILD_DIR)/math
	@mkdir -p $(BUILD_DIR)/dap

# Link
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

# Compile test main
$(BUILD_DIR)/test_DAP.o: $(MAIN_SRC)
	$(CC) $(CFLAGS) -I$(ROOT_DIR) -I$(INCLUDE_DIR) -I$(TEST_DIR) -c -o $@ $<

# Compile math source files
$(BUILD_DIR)/math/%.o: $(SRC_DIR)/math/%.c
	$(CC) $(CFLAGS) -I$(INCLUDE_DIR) -c -o $@ $<

# Compile DAP source files (in this test directory)
$(BUILD_DIR)/dap/%.o: $(TEST_DIR)/%.c
	$(CC) $(CFLAGS) -I$(ROOT_DIR) -I$(INCLUDE_DIR) -I$(TEST_DIR) -c -o $@ $<

# Clean build files
clean:
	rm -rf build_host build_pic32

# Run the program
run: 
ifdef PIC32_BUILD
	@echo "Cannot run PIC32 binary on host system."
	@echo "✓ PIC32 binary created successfully at $(TARGET)"
	@echo "Use MPLAB X SIM or actual PIC32 hardware to execute."
else
	./$(TARGET)
endif

# Convenience targets
verify-xc32: pic32-verify
test: host-test

# Quick XC32 setup helper
setup-xc32:
	@echo "Setting up XC32 PATH for current session..."
	@echo "Run these commands:"
	@echo "  export PATH=\"/opt/microchip/xc32/v4.60/bin:\$$PATH\""
	@echo "  echo 'export PATH=\"/opt/microchip/xc32/v4.60/bin:\$$PATH\"' >> ~/.bashrc"
	@echo "  source ~/.bashrc"
	@echo "Then run: make check-xc32"

# Show available targets
help:
	@echo "Available targets:"
	@echo "  all          - Build for host (default)"
	@echo "  pic32-verify - Verify code compiles with XC32 for PIC32"
	@echo "  host-test    - Build and test on host system"
	@echo "  verify-xc32  - Alias for pic32-verify"
	@echo "  test         - Alias for host-test"
	@echo "  check-xc32   - Check if XC32 compiler is available"
	@echo "  setup-xc32   - Show XC32 PATH setup instructions"
	@echo "  clean        - Remove all build files"
	@echo "  help         - Show this help"

.PHONY: all clean run directories pic32-verify host-test verify-xc32 test help check-xc32 setup-xc32