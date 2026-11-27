# Superseded by the top-level Makefile: run `make` from the repository root.
ROOT := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)

.PHONY: all debug asan clean
all debug asan clean:
	$(MAKE) -C $(ROOT) $@
