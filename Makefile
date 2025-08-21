# ISA Flight Software Makefile

# Compiler settings
CC = gcc
CFLAGS = -Wall -Wextra -Werror -pedantic -std=c99 -O2 -g
LDFLAGS = -lm

# Directories
SRC_DIR = src
INCLUDE_DIR = include
BUILD_DIR = build
TEST_DIR = test

# Source files
MATH_SRC = $(wildcard $(SRC_DIR)/math/*.c)
GUIDANCE_SRC = $(wildcard $(SRC_DIR)/guidance/*.c)
NAVIGATION_SRC = $(wildcard $(SRC_DIR)/navigation/*.c)
DAP_SRC = $(wildcard $(SRC_DIR)/dap/*.c)
SEQUENCER_SRC = $(wildcard $(SRC_DIR)/sequencer/*.c)
HAL_SRC = $(wildcard $(SRC_DIR)/hal/*.c)
MAIN_SRC = main.c


# Object files
MATH_OBJ = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(MATH_SRC))
GUIDANCE_OBJ = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(GUIDANCE_SRC))
NAVIGATION_OBJ = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(NAVIGATION_SRC))
DAP_OBJ = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(DAP_SRC))
SEQUENCER_OBJ = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(SEQUENCER_SRC))
HAL_OBJ = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(HAL_SRC))
MAIN_OBJ = $(BUILD_DIR)/main.o      

# All object files
OBJS = $(MATH_OBJ) $(GUIDANCE_OBJ) $(NAVIGATION_OBJ) $(DAP_OBJ) $(SEQUENCER_OBJ) $(HAL_OBJ) $(MAIN_OBJ)

# Test files
TEST_SRC = $(wildcard $(TEST_DIR)/*.c)
TEST_BINS = $(patsubst $(TEST_DIR)/%.c,$(BUILD_DIR)/test_%,$(TEST_SRC))

# Sequencer test (special case)
SEQUENCER_TEST = $(BUILD_DIR)/test_sequencer

# Target executable
TARGET = $(BUILD_DIR)/isa_flight_software

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
	@mkdir -p $(BUILD_DIR)/test

# Compile main program
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

# Compile main.c
$(BUILD_DIR)/main.o: $(MAIN_SRC)
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

# Compile and run tests
tests: directories $(TEST_BINS) $(SEQUENCER_TEST)
	@for test in $(TEST_BINS); do \
		echo "Running $$test"; \
		./$$test; \
	done
	@echo "Running sequencer test"; \
	./$(SEQUENCER_TEST)

# Compile test files
$(BUILD_DIR)/test_%: $(TEST_DIR)/%.c $(filter-out $(BUILD_DIR)/main.o,$(OBJS))
	$(CC) $(CFLAGS) -I$(INCLUDE_DIR) -o $@ $^ $(LDFLAGS)

# Compile sequencer test
$(SEQUENCER_TEST): $(TEST_DIR)/sequencer/test_sequencer.c $(TEST_DIR)/sequencer/sequencer.c
	$(CC) $(CFLAGS) -I$(INCLUDE_DIR) -o $@ $^ $(LDFLAGS)

# Clean build files
clean:
	rm -rf $(BUILD_DIR)

# Run the program
run: all
	./$(TARGET)

.PHONY: all clean run tests directories
