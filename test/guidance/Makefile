# ISA Flight Software - Guidance Module Makefile
# Builds and tests the guidance module with all dependencies

# Compiler settings (GCC for host testing)
CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -O2 -g -lm
INCLUDES = -I. -I../../include/common

# Source files
VECTOR3_SRC = vector3.c
COORDINATE_TRANSFORMS_SRC = coordinate_transforms.c
MATH_UTILS_SRC = math_utils.c
GUIDANCE_SRC = guidance.c
GUIDANCE_TEST_SRC = guidance_test.c

# Object files
VECTOR3_OBJ = vector3.o
COORDINATE_TRANSFORMS_OBJ = coordinate_transforms.o
MATH_UTILS_OBJ = math_utils.o
GUIDANCE_OBJ = guidance.o
GUIDANCE_TEST_OBJ = guidance_test.o

# Target executable
TARGET = guidance_test

# Default target
all: $(TARGET)

# Build the guidance test executable
$(TARGET): $(VECTOR3_OBJ) $(COORDINATE_TRANSFORMS_OBJ) $(MATH_UTILS_OBJ) $(GUIDANCE_OBJ) $(GUIDANCE_TEST_OBJ)
	$(CC) $(CFLAGS) -o $(TARGET) $(VECTOR3_OBJ) $(COORDINATE_TRANSFORMS_OBJ) $(MATH_UTILS_OBJ) $(GUIDANCE_OBJ) $(GUIDANCE_TEST_OBJ) -lm
	@echo "✓ Guidance module build completed successfully"

# Compile vector3 module
$(VECTOR3_OBJ): $(VECTOR3_SRC) vector3.h
	$(CC) $(CFLAGS) $(INCLUDES) -c $(VECTOR3_SRC) -o $(VECTOR3_OBJ)

# Compile coordinate transforms module
$(COORDINATE_TRANSFORMS_OBJ): $(COORDINATE_TRANSFORMS_SRC) coordinate_transforms.h vector3.h
	$(CC) $(CFLAGS) $(INCLUDES) -c $(COORDINATE_TRANSFORMS_SRC) -o $(COORDINATE_TRANSFORMS_OBJ)

# Compile math utils module
$(MATH_UTILS_OBJ): $(MATH_UTILS_SRC) math_utils.h
	$(CC) $(CFLAGS) $(INCLUDES) -c $(MATH_UTILS_SRC) -o $(MATH_UTILS_OBJ)

# Compile guidance module
$(GUIDANCE_OBJ): $(GUIDANCE_SRC) guidance.h vector3.h coordinate_transforms.h math_utils.h
	$(CC) $(CFLAGS) $(INCLUDES) -c $(GUIDANCE_SRC) -o $(GUIDANCE_OBJ)

# Compile guidance test
$(GUIDANCE_TEST_OBJ): $(GUIDANCE_TEST_SRC) guidance.h vector3.h coordinate_transforms.h
	$(CC) $(CFLAGS) $(INCLUDES) -c $(GUIDANCE_TEST_SRC) -o $(GUIDANCE_TEST_OBJ)

# Clean build artifacts
clean:
	rm -f *.o $(TARGET)
	@echo "✓ Build artifacts cleaned"

# Run the test
run: $(TARGET)
	./$(TARGET)

# Build and run (default behavior)
test: all run

# Debug build
debug: CFLAGS += -DDEBUG
debug: clean all

.PHONY: all clean run test debug

