# Makefile for Sequencer Test Application
# Builds the sequencer test executable

# Compiler and flags
CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -O2 -g
INCLUDES = -I.

# Source files
SOURCES = main_sequencer_sim.c sequencer.c
OBJECTS = $(SOURCES:.c=.o)

# Target executable
TARGET = sequencer_test

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
	rm -f $(OBJECTS) $(TARGET)

# Run target - build and execute
run: $(TARGET)
	./$(TARGET)

# Install target (optional)
install: $(TARGET)
	cp $(TARGET) /usr/local/bin/

# Phony targets
.PHONY: all clean install run
