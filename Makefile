CC      ?= gcc
CFLAGS  ?= -std=c99 -g \
	-I../include \
	-D_POSIX_SOURCE -D_DEFAULT_SOURCE \
	-Wall -Werror -Wpedantic -Wshadow -Wconversion
BUILD_DIR = ../.build

SRCFILES_ANIMATE = animate.c io.c map.c pathfinder.c adt/dynarr.c adt/heap.c adt/bitarr.c smooth.c
OBJFILES_ANIMATE = $(SRCFILES_ANIMATE:%.c=$(BUILD_DIR)/%.o)
SRCFILES = io.c map.c main.c pathfinder.c adt/dynarr.c adt/heap.c adt/bitarr.c smooth.c
OBJFILES = $(SRCFILES:%.c=$(BUILD_DIR)/%.o)
DEPFILES = $(OBJFILES:%.o=%.d)
LDLIBS = -lm

.PHONY: all

all: pathfind animate

clean:
	$(RM) -r $(BUILD_DIR)
	$(RM) pathfind

-include $(DEPFILES)

pathfind: $(OBJFILES)
	$(CC) $(LDFLAGS) $^ $(LOADLIBES) $(LDLIBS) -o $@

animate: $(OBJFILES_ANIMATE)
	$(CC) $(LDFLAGS) $^ $(LOADLIBES) $(LDLIBS) -lX11 -lGL -o $@

$(BUILD_DIR)/%.o: %.c
	$(CC) $(CFLAGS) -MMD -c $< -o $@

$(OBJFILES) $(OBJFILES_ANIMATE): | $(BUILD_DIR) $(BUILD_DIR)/adt

$(BUILD_DIR) $(BUILD_DIR)/adt:
	mkdir -p $(BUILD_DIR) $(BUILD_DIR)/adt
