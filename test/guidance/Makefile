# ISA Flight Software Makefile

# Compiler settings
CC = gcc
CFLAGS = -Wall -Wextra -Werror -pedantic -std=c99 -O2 -g
LDFLAGS = -lm

# Directories
SRC_DIR = ../../src
INCLUDE_DIR = ../../include
BUILD_DIR = build
TEST_DIR = .

# Source files
MATH_SRC = $(wildcard $(SRC_DIR)/math/*.c)
GUIDANCE_SRC = $(wildcard $(SRC_DIR)/guidance/*.c)
NAVIGATION_SRC = $(wildcard $(SRC_DIR)/navigation/*.c)
DAP_SRC = $(wildcard $(SRC_DIR)/dap/*.c)
SEQUENCER_SRC = $(wildcard $(SRC_DIR)/sequencer/*.c)
HAL_SRC = $(wildcard $(SRC_DIR)/hal/*.c)
GUIDANCE_TEST_SRC = guidance.c

# Object files
MATH_OBJ = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(MATH_SRC))
GUIDANCE_OBJ = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(GUIDANCE_SRC))
NAVIGATION_OBJ = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(NAVIGATION_SRC))
DAP_OBJ = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(DAP_SRC))
SEQUENCER_OBJ = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(SEQUENCER_SRC))
HAL_OBJ = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(HAL_SRC))
GUIDANCE_TEST_OBJ = $(BUILD_DIR)/guidance.o

# All object files
OBJS = $(MATH_OBJ) $(GUIDANCE_OBJ) $(NAVIGATION_OBJ) $(DAP_OBJ) $(SEQUENCER_OBJ) $(HAL_OBJ)

# Target executable
TARGET = $(BUILD_DIR)/guidance_test

# Default target
all: directories $(TARGET)

# Create build directories
directories:
	@mkdir -p $(BUILD_DIR)
	@mkdir -p $(BUILD_DIR)/math
	@mkdir -p $(BUILD_DIR)/guidance
	@mkdir -p $(BUILD_DIR)/navigation
	@mkdir -p $(BUILD_DIR)/dap
	@mkdir -p $(BUILD_DIR)/sequencer
	@mkdir -p $(BUILD_DIR)/hal

# Compile guidance test
$(TARGET): $(GUIDANCE_TEST_OBJ) $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

# Compile guidance test file
$(BUILD_DIR)/guidance.o: $(GUIDANCE_TEST_SRC)
	$(CC) $(CFLAGS) -I$(INCLUDE_DIR) -c -o $@ $<

# Compile math source files
$(BUILD_DIR)/math/%.o: $(SRC_DIR)/math/%.c
	$(CC) $(CFLAGS) -I$(INCLUDE_DIR) -c -o $@ $<

# Compile guidance source files
$(BUILD_DIR)/guidance/%.o: $(SRC_DIR)/guidance/%.c
	$(CC) $(CFLAGS) -I$(INCLUDE_DIR) -c -o $@ $<

# Compile navigation source files
$(BUILD_DIR)/navigation/%.o: $(SRC_DIR)/navigation/%.c
	$(CC) $(CFLAGS) -I$(INCLUDE_DIR) -c -o $@ $<

# Compile DAP source files
$(BUILD_DIR)/dap/%.o: $(SRC_DIR)/dap/%.c
	$(CC) $(CFLAGS) -I$(INCLUDE_DIR) -c -o $@ $<

# Compile sequencer source files
$(BUILD_DIR)/sequencer/%.o: $(SRC_DIR)/sequencer/%.c
	$(CC) $(CFLAGS) -I$(INCLUDE_DIR) -c -o $@ $<

# Compile HAL source files
$(BUILD_DIR)/hal/%.o: $(SRC_DIR)/hal/%.c
	$(CC) $(CFLAGS) -I$(INCLUDE_DIR) -c -o $@ $<

# Clean build files
clean:
	rm -rf $(BUILD_DIR)

# Run the guidance test
run: all
	./$(TARGET)

.PHONY: all clean run directories
