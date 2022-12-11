# SPDX-FileCopyrightText: © 2022 Bonny Rais <bonnyr@gmail.com>
# SPDX-License-Identifier: MIT

# src/wokwi-api.c 
SOURCES = src/ow_signaling_sm.c src/ow_byte_sm.c src/hashmap.c src/ds18b20-2.chip.c 
TARGET  = dist/chip.wasm
INCLUDE = -I . -I include

.PHONY: all
all: $(TARGET) dist/chip.json

.PHONY: clean
clean:
		rm -rf dist

dist:
		mkdir -p dist

$(TARGET): dist $(SOURCES) # src/wokwi-api.h
	  clang --target=wasm32-unknown-wasi --sysroot /opt/wasi-libc -nostartfiles -Wl,--import-memory -Wl,--export-table -Wl,--no-entry -Werror  $(INCLUDE) -o $(TARGET) $(SOURCES)

dist/chip.json: dist chip.json
	  cp src/ds18b20.chip.json dist/chip.json

