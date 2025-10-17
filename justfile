demo-assemble file: demo-target
    ca65 demos/{{ file }} -o \
        target/demos/{{ file }}.o \
        -l target/demos/{{ file }}.lst
    ld65 target/demos/{{ file }}.o \
        -C demos/memory.cfg \
        -o target/demos/{{ file }}.bin

demo-target:
    mkdir -p target/demos

demo-run file: (demo-assemble file)
    cargo run --package nes-desktop -- target/demos/{{ file }}.bin
