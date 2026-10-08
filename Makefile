CC ?= cc
CXX ?= c++
AR ?= ar
OPT ?= -Os
SAN ?= -fsanitize=address,undefined -fno-sanitize-recover=all
WARN = -pedantic -Wall -Wextra -Werror -Wshadow -Wconversion -Wcast-qual
C89 = -std=c89 $(WARN) -Wdeclaration-after-statement -Wstrict-prototypes
TFLAGS = -std=c89 -pedantic -Wall -Wextra -Werror -Iinclude -O1 -g $(SAN)

all: build/libswarm.a

build/libswarm.a: src/swarm.c include/swarm.h
	@mkdir -p build
	$(CC) $(C89) $(OPT) -ffunction-sections -fdata-sections -Iinclude \
		-c src/swarm.c -o build/swarm.o
	$(AR) rcs $@ build/swarm.o

# default config, a tiny-table config that forces eviction and log
# overflow, and single precision
test: src/swarm.c include/swarm.h test/test_swarm.c
	@mkdir -p build
	$(CC) $(TFLAGS) test/test_swarm.c -lm -o build/test
	$(CC) $(TFLAGS) -DSW_MAX_NODES=6 -DSW_MAX_NBR=3 -DSW_LOG=4 \
		test/test_swarm.c -lm -o build/test_small
	$(CC) $(subst c89,c99,$(TFLAGS)) -DSW_FLOAT test/test_swarm.c -lm \
		-o build/test_float
	./build/test && ./build/test_small && ./build/test_float

# the library must also build as C++98
cxx: src/swarm.c include/swarm.h
	@mkdir -p build
	$(CXX) -x c++ -std=c++98 $(WARN) $(OPT) -Iinclude -c src/swarm.c \
		-o build/swarm_cxx.o

size: build/libswarm.a
	size build/swarm.o

clean:
	rm -rf build

.PHONY: all test cxx size clean
