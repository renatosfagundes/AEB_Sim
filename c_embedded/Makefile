# AEB Embedded C - Makefile
# Compiles with GCC (NFR-POR-001: platform-independent)

CC = gcc
CFLAGS = -Wall -Wextra -Werror -std=c99 -pedantic -Iinclude
CFLAGS += -Wshadow -Wconversion -Wdouble-promotion
CFLAGS += -O2

SRC_DIR = src
INC_DIR = include
TEST_DIR = test
BUILD_DIR = build

SRCS = $(wildcard $(SRC_DIR)/*.c)
OBJS = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(SRCS))

TEST_SRCS = $(wildcard $(TEST_DIR)/*.c)
TEST_BINS = $(patsubst $(TEST_DIR)/%.c,$(BUILD_DIR)/%,$(TEST_SRCS))

.PHONY: all clean test

all: $(BUILD_DIR)/libaeb.a

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/libaeb.a: $(OBJS)
	ar rcs $@ $^

# Build and run tests
test: $(TEST_BINS)
	@for t in $(TEST_BINS); do echo "Running $$t..."; $$t; done

$(BUILD_DIR)/%: $(TEST_DIR)/%.c $(BUILD_DIR)/libaeb.a | $(BUILD_DIR)
	$(CC) $(CFLAGS) $< -L$(BUILD_DIR) -laeb -lm -o $@

clean:
	rm -rf $(BUILD_DIR)
