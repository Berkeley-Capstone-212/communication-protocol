CC ?= cc
CXX ?= c++
AR ?= ar
OPT ?= -Os
SAN ?= -fsanitize=address,undefined -fno-sanitize-recover=all
# Warning set accepted by both GCC and Clang; the build must stay silent.
COMMON = -pedantic -Wall -Wextra -Werror -Wshadow -Wconversion \
	-Wsign-conversion -Wcast-qual -Wcast-align -Wundef -Wfloat-equal \
	-Wdouble-promotion -Wredundant-decls -Wwrite-strings -Wformat=2 \
	-Wunused-macros
CWARN = $(COMMON) -Wbad-function-cast -Wstrict-prototypes \
	-Wmissing-prototypes -Wold-style-definition -Wnested-externs \
	-Wdeclaration-after-statement
C89 = -std=c89 $(CWARN)
TFLAGS = $(CWARN) -Iinclude -O1 -g $(SAN)

all: build/libswarm.a

build/libswarm.a: src/swarm.c include/swarm.h
	@mkdir -p build
	$(CC) $(C89) $(OPT) -ffunction-sections -fdata-sections -Iinclude \
		-c src/swarm.c -o build/swarm.o
	$(AR) rcs $@ build/swarm.o

# default config, a tiny-table config that forces eviction, slot rotation
# and log overflow, and single precision
test: src/swarm.c include/swarm.h test/test_swarm.c
	@mkdir -p build
	$(CC) -std=c89 $(TFLAGS) test/test_swarm.c -lm -o build/test
	$(CC) -std=c89 $(TFLAGS) -DSW_MAX_NODES=6 -DSW_MAX_NBR=3 -DSW_LOG=4 \
		test/test_swarm.c -lm -o build/test_small
	$(CC) -std=c99 $(TFLAGS) -DSW_FLOAT test/test_swarm.c -lm \
		-o build/test_float
	./build/test && ./build/test_small && ./build/test_float

# the library must also build as C++98
cxx: src/swarm.c include/swarm.h
	@mkdir -p build
	$(CXX) -x c++ -std=c++98 $(COMMON) $(OPT) -Iinclude -c src/swarm.c \
		-o build/swarm_cxx.o

# Clang only: every warning Clang has, on the default configuration, plus
# the static analyser. -Wpadded depends on the configuration, so other
# table sizes or SW_FLOAT may report padding there.
LINT = -Weverything -Werror -Wno-poison-system-directories -Iinclude
lint:
	@mkdir -p build
	clang -std=c89 $(LINT) -c src/swarm.c -o build/lint.o
	clang -std=c89 $(LINT) -c test/test_swarm.c -o build/lint_test.o
	printf '#include "swarm.h"\n' | clang++ -x c++ -std=c++98 $(LINT) \
		-fsyntax-only -
	clang --analyze -std=c89 -Iinclude -Xanalyzer -analyzer-werror \
		src/swarm.c -o build/lint.plist
	clang --analyze -std=c89 -Iinclude -Xanalyzer -analyzer-werror \
		test/test_swarm.c -o build/lint_test.plist

size: build/libswarm.a
	size build/swarm.o

clean:
	rm -rf build

.PHONY: all test cxx lint size clean
