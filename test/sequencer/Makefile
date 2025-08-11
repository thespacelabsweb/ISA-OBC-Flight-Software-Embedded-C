# ISA Sequencer Test Makefile

# Compiler settings
CC = gcc
CFLAGS = -Wall -Wextra -pedantic -std=c99 -O2 -g
LDFLAGS = -lm

# Directories (relative to this Makefile at test/sequencer)
ROOT_DIR = ../..
SRC_DIR = $(ROOT_DIR)/src
INCLUDE_DIR = $(ROOT_DIR)/include
TEST_DIR = .
BUILD_DIR = build

# Source files
# Math utilities from shared src
MATH_SRC = $(wildcard $(SRC_DIR)/math/*.c)
# All C files in this dir except the test main will be treated as module sources
# In this setup, the test main and module live in the same file (sequencer.c),
# so exclude it from SEQ_SRC to avoid double-compilation.
SEQ_SRC = $(filter-out $(TEST_DIR)/sequencer.c,$(wildcard $(TEST_DIR)/*.c))
MAIN_SRC = $(TEST_DIR)/sequencer.c

# Object files
MATH_OBJ = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(MATH_SRC))
SEQ_OBJ = $(patsubst $(TEST_DIR)/%.c,$(BUILD_DIR)/sequencer/%.o,$(SEQ_SRC))
MAIN_OBJ = $(BUILD_DIR)/sequencer_main.o

# All object files
OBJS = $(MATH_OBJ) $(SEQ_OBJ) $(MAIN_OBJ)

# Target executable
TARGET = $(BUILD_DIR)/sequencer_test

# Default target
all: directories $(TARGET)

# Create build directories
directories:
	@mkdir -p $(BUILD_DIR)
	@mkdir -p $(BUILD_DIR)/math
	@mkdir -p $(BUILD_DIR)/sequencer

# Link
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

# Compile test main
$(BUILD_DIR)/sequencer_main.o: $(MAIN_SRC)
	$(CC) $(CFLAGS) -I$(ROOT_DIR) -I$(INCLUDE_DIR) -I$(TEST_DIR) -c -o $@ $<

# Compile math source files
$(BUILD_DIR)/math/%.o: $(SRC_DIR)/math/%.c
	$(CC) $(CFLAGS) -I$(INCLUDE_DIR) -c -o $@ $<

# Compile sequencer source files (in this test directory)
$(BUILD_DIR)/sequencer/%.o: $(TEST_DIR)/%.c
	$(CC) $(CFLAGS) -I$(ROOT_DIR) -I$(INCLUDE_DIR) -I$(TEST_DIR) -c -o $@ $<

# Clean build files
clean:
	rm -rf $(BUILD_DIR)

# Run the program
run: all
	./$(TARGET)

.PHONY: all clean run directories


