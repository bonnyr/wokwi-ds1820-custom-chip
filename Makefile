# SPDX-FileCopyrightText: © 2022 Bonny Rais <bonnyr@gmail.com>
# SPDX-License-Identifier: MIT

SOURCES = src/ow_signaling_sm.c src/ow_byte_sm.c src/hashmap.c src/ds18b20.chip.c 
INCLUDES = -I . -I include
CHIP_JSON = ds18b20.chip.json

TARBALL  = dist/chip.tar.gz
TARGET  = dist/chip.wasm

.PHONY: all
all: clean $(TARGET)

.PHONY: clean
clean:
		rm -rf dist
dist:
		mkdir -p dist

$(TARBALL): $(TARGET) dist/$(CHIP_JSON)
	tar xzf $(TARBALL) $(TARGET) dist/chip.json

dist/$(CHIP_JSON):
	cp $(CHIP_JSON) dist

$(LIB): dist $(SOURCES)
	  clang --target=wasm32-unknown-wasi --sysroot /opt/wasi-libc -nostartfiles -Wl,--import-memory -Wl,--export-table -Wl,--no-entry -Werror  $(INCLUDES) -o $(LIB) $(SOURCES)
