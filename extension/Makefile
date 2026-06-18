CC      ?= gcc
CFLAGS  ?= -std=c99 -g \
	-I../include \
	-D_POSIX_SOURCE -D_DEFAULT_SOURCE \
	-Wall -Werror -Wpedantic -Wshadow -Wconversion
BUILD = ../build
LDLIBS = -lm
SRCFILES = io.c map.c main.c pathfinder.c adt/dynarr.c adt/heap.c adt/bitarr.c
OBJFILES = $(SRCFILES:%.c=$(BUILD)/%.o)

.PHONY: all

all: pathfind

clean:
	rm -rf $(BUILD) pathfind

pathfind: $(OBJFILES)
	$(CC) $(LDFLAGS) $^ $(LOADLIBES) $(LDLIBS) -o $@

$(BUILD)/%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $^

$(OBJFILES): | $(BUILD) $(BUILD)/adt

$(BUILD) $(BUILD)/adt:
	mkdir -p $(BUILD) $(BUILD)/adt
