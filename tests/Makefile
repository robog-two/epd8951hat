# SPDX-License-Identifier: GPL-2.0
# Build and run userspace unit tests for the epd8951hat pipeline functions.
#
# The pipeline sources (epd8951hat_pipeline.c) are compiled in TESTING_BUILD
# mode: kernel headers are replaced by tests/compat.h.
#
# Usage:
#   make          – build test binary
#   make run      – build and run
#   make clean    – remove build artefacts

CC      := gcc
CFLAGS  := -std=c11 -Wall -Wextra -Werror \
           -DTESTING_BUILD \
           -I. \
           -I.. \
           -g -O0

SRC     := ../epd8951hat_pipeline.c test_pipeline.c
BIN     := test_pipeline

BENCH_SRC := ../epd8951hat_pipeline.c bench_pipeline.c
BENCH_BIN := bench_pipeline
# The benchmark wants optimisation; -O0 would mask the algorithmic wins.
BENCH_CFLAGS := -std=c11 -Wall -Wextra -DTESTING_BUILD -I. -I.. -O2

.PHONY: all run bench clean

all: $(BIN)

$(BIN): $(SRC) ../epd8951hat_pipeline.h compat.h
	$(CC) $(CFLAGS) $(SRC) -o $@

run: $(BIN)
	./$(BIN)

$(BENCH_BIN): $(BENCH_SRC) ../epd8951hat_pipeline.h compat.h
	$(CC) $(BENCH_CFLAGS) $(BENCH_SRC) -o $@

bench: $(BENCH_BIN)
	./$(BENCH_BIN)

clean:
	rm -f $(BIN) $(BENCH_BIN)
