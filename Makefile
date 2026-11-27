# Makefile for Tetrys (core library + UDP module + TUN/TAP tunnel)
#
# Usage:
#   make            optimised build  -> bin/tetrys_generate_udp, bin/tetrystunnel_udp
#   make debug      -O0 -g3 build   -> bin-debug/
#   make asan       AddressSanitizer + UBSan build -> bin-asan/
#   make figs       regenerate doc/img/*.png from the Xfig sources doc/*.fig (needs fig2dev)
#   make doc        Doxygen HTML documentation -> build-doc/html/index.html
#   make doc-check  same, but fail on any Doxygen warning (undocumented API)
#   make shim       LD_PRELOAD packet-loss shim -> build/loss_shim.so
#   make clean      remove every build*/ and bin*/ directory

CC       ?= gcc
STD      := -std=gnu11
WARN     := -Wall -Wextra -Wpedantic -Wstrict-prototypes
OPT      ?= -O2 -g
CFLAGS   := $(STD) $(WARN) $(OPT) $(EXTRA_CFLAGS)
LDLIBS   := -lpthread -lm

CORE_DIR := src/tetrys_core
UDP_DIR  := src/modules/udp
TUN_DIR  := src/modules/tuntap
INCLUDES := -I$(CORE_DIR) -I$(UDP_DIR)

BUILD    ?= build
BIN      ?= bin

# Doxygen: reuse doc/Doxyfile and override what the sources need (the
# assignments appended on stdin take precedence over the file content).
DOXYGEN  ?= doxygen
DOC_DIR  ?= build-doc
DOXY_OVERRIDES = INPUT="src doc/mainpage.dox" RECURSIVE=YES OUTPUT_DIRECTORY=$(DOC_DIR) \
                 FILE_PATTERNS="*.c *.h *.dox" IMAGE_PATH=doc/img \
                 GENERATE_LATEX=NO GENERATE_HTML=YES EXTRACT_STATIC=YES \
                 WARN_NO_PARAMDOC=YES QUIET=YES

DOXY_WERROR ?= NO

# Figures drawn with Xfig (doc/*.fig) and exported to PNG (doc/img/*.png).
FIG2DEV  ?= fig2dev
FIGS     := tetrysBufferSenderReceiver tetrysBufferSender tetrysBufferReceiver \
            tetrysLegend TetrysDoc
FIG_PNG  := $(patsubst %,doc/img/%.png,$(FIGS))

CORE_SRC := $(wildcard $(CORE_DIR)/*.c)
UDP_SRC  := $(wildcard $(UDP_DIR)/*.c)
TUN_SRC  := $(wildcard $(TUN_DIR)/*.c)

CORE_OBJ := $(patsubst %.c,$(BUILD)/%.o,$(CORE_SRC))
UDP_OBJ  := $(patsubst %.c,$(BUILD)/%.o,$(UDP_SRC))
TUN_OBJ  := $(patsubst %.c,$(BUILD)/%.o,$(TUN_SRC))

.PHONY: all debug asan doc doc-check shim clean

all: $(BIN)/tetrys_generate_udp $(BIN)/tetrystunnel_udp

debug:
	$(MAKE) all BUILD=build-debug BIN=bin-debug OPT="-O0 -g3 -DDEBUG"

asan:
	$(MAKE) all BUILD=build-asan BIN=bin-asan OPT="-O1 -g3 -fno-omit-frame-pointer" \
	    EXTRA_CFLAGS="-fsanitize=address,undefined" \
	    LDLIBS="-lpthread -lm -fsanitize=address,undefined"

figs: $(FIG_PNG)

doc/img/%.png: doc/%.fig
	@mkdir -p doc/img
	$(FIG2DEV) -L png -m 2 $< $@

doc:
	@mkdir -p $(DOC_DIR)
	@( cat doc/Doxyfile; for v in $(DOXY_OVERRIDES); do echo "$$v"; done; echo "WARN_AS_ERROR=$(DOXY_WERROR)" ) | $(DOXYGEN) -
	@echo "Documentation: $(DOC_DIR)/html/index.html"

doc-check:
	$(MAKE) doc DOXY_WERROR=YES

shim: $(BUILD)/loss_shim.so

$(BUILD)/loss_shim.so: test/loss_shim.c
	@mkdir -p $(dir $@)
	$(CC) $(STD) $(WARN) -Werror $(OPT) -shared -fPIC $< -ldl -o $@

$(BIN)/tetrys_generate_udp: $(CORE_OBJ) $(UDP_OBJ) | $(BIN)
	$(CC) $^ $(LDLIBS) -o $@

$(BIN)/tetrystunnel_udp: $(CORE_OBJ) $(TUN_OBJ) | $(BIN)
	$(CC) $^ $(LDLIBS) -o $@

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(INCLUDES) -MMD -MP -c $< -o $@

$(BIN):
	mkdir -p $(BIN)

clean:
	rm -rf build build-debug build-asan build-doc bin bin-debug bin-asan

-include $(CORE_OBJ:.o=.d) $(UDP_OBJ:.o=.d) $(TUN_OBJ:.o=.d)
