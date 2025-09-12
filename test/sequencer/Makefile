# Makefile for ISA Flight Sequencer Test
# MISRA C Compliant Build Configuration

# Compiler and flags
CC = gcc
CFLAGS = -Wall -Wextra -Wpedantic -std=c99 -O0 -g
CFLAGS += -Wstrict-prototypes -Wmissing-prototypes -Wmissing-declarations
CFLAGS += -Wold-style-definition -Wpointer-arith -Wcast-qual -Wcast-align
CFLAGS += -Wwrite-strings -Wconversion -Wsign-conversion -Wformat=2
CFLAGS += -Wformat-security -Wstack-protector -Wunreachable-code
CFLAGS += -Wundef -Wshadow -Wcast-align=strict

# Include directories
INCLUDES = -I. -I../../include/common

# Source files
SEQUENCER_SRC = sequencer.c
TEST_SRC = unit_tests/sequencer_test.c

# Object files
SEQUENCER_OBJ = sequencer.o
TEST_OBJ = unit_tests/sequencer_test.o

# Target executable
TARGET = sequencer_test.exe

# Windows compatibility
TMPDIR = .
export TMPDIR

# Default target
all: $(TARGET)

# Build the test executable
$(TARGET): $(TEST_OBJ) $(SEQUENCER_OBJ)
	@echo "Linking $(TARGET)..."
	$(CC) $(TEST_OBJ) $(SEQUENCER_OBJ) -o $(TARGET)
	@echo "Build complete: $(TARGET)"

# Compile sequencer source
$(SEQUENCER_OBJ): $(SEQUENCER_SRC) sequencer.h
	@echo "Compiling sequencer.c..."
	$(CC) $(CFLAGS) $(INCLUDES) -c $(SEQUENCER_SRC) -o $(SEQUENCER_OBJ)

# Compile test source
$(TEST_OBJ): $(TEST_SRC) sequencer.h
	@echo "Compiling $(TEST_SRC)..."
	$(CC) $(CFLAGS) $(INCLUDES) -c $(TEST_SRC) -o $(TEST_OBJ)

# Run the test
run: $(TARGET)
	@echo "Running sequencer test..."
	@echo "=================================="
	./$(TARGET)

# Clean build artifacts
clean:
	@echo "Cleaning build artifacts..."
	del /Q $(SEQUENCER_OBJ) $(TEST_OBJ) $(TARGET) 2>nul || true
	@echo "Clean complete"

# Force rebuild
rebuild: clean all

# Show help
help:
	@echo "ISA Flight Sequencer Test Makefile"
	@echo "Available targets:"
	@echo "  all      - Build the test executable (default)"
	@echo "  run      - Build and run the test"
	@echo "  clean    - Remove build artifacts"
	@echo "  rebuild  - Clean and build"
	@echo "  help     - Show this help message"

# Declare phony targets
.PHONY: all run clean rebuild help

# Dependencies
$(TEST_OBJ): sequencer.h
$(SEQUENCER_OBJ): sequencer.h