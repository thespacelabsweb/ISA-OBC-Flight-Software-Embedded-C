# Makefile for ISA Flight Sequencer Test
# MISRA C Compliant Build Configuration

# Compiler and flags
CC = gcc
CFLAGS = -Wall -Wextra -Wpedantic -std=c99 -O2 -g
CFLAGS += -Wstrict-prototypes -Wmissing-prototypes -Wmissing-declarations
CFLAGS += -Wold-style-definition -Wpointer-arith -Wcast-qual -Wcast-align
CFLAGS += -Wwrite-strings -Wconversion -Wsign-conversion -Wfloat-equal
CFLAGS += -Wformat=2 -Wundef -Wshadow -Wstrict-overflow=5
CFLAGS += -Wswitch-default -Wswitch-enum -Wunreachable-code -Winit-self

# Windows-specific: Set temporary directory to current directory
TMPDIR = .
export TMPDIR

# Include paths
INCLUDES = -I. -I../../include/common

# Source files
SOURCES = test_sequencer.c sequencer.c
OBJECTS = $(SOURCES:.c=.o)

# Target executable
TARGET = sequencer_test.exe

# Default target
all: $(TARGET)

# Build the executable
$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -o $(TARGET)

# Compile source files to object files
%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Clean build artifacts
clean:
	del /Q $(OBJECTS) $(TARGET) 2>nul || echo "Clean completed"

# Run target - build and execute
run: $(TARGET)
	$(TARGET)

# Memory check with valgrind (if available on Windows)
memcheck: $(TARGET)
	@echo "Memory checking tools may not be available on Windows"
	@echo "Consider using Application Verifier or similar Windows tools"

# Static analysis with cppcheck (if available)
static-check:
	cppcheck --enable=all --std=c99 --suppress=missingIncludeSystem $(SOURCES)

# MISRA check (requires PC-lint or similar tool)
misra-check:
	@echo "Run PC-lint or similar MISRA C checker on source files"
	@echo "Files to check: $(SOURCES)"

# Debug build with extra debugging symbols
debug: CFLAGS += -DDEBUG -g3
debug: $(TARGET)

# Release build with optimizations
release: CFLAGS += -DNDEBUG -O3
release: $(TARGET)

# Phony targets
.PHONY: all clean run memcheck static-check misra-check debug release