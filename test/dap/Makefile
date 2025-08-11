# ISA Digital Autopilot (DAP) Makefile

# Compiler settings
CC = gcc
CFLAGS = -Wall -Wextra -pedantic -std=c99 -O2 -g
LDFLAGS = -lm

# Directories (relative to this Makefile at test/dap)
ROOT_DIR = ../..
SRC_DIR = $(ROOT_DIR)/src
INCLUDE_DIR = $(ROOT_DIR)/include
TEST_DIR = .
BUILD_DIR = build

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

# Default target
all: directories $(TARGET)

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
	rm -rf $(BUILD_DIR)

# Run the program
run: all
	./$(TARGET)

.PHONY: all clean run directories