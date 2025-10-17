.segment "code"
start:
    lda #$c0  ;Load the hex value $c0 into the A register
    tax       ;Transfer the value in the A register to X
    inx       ;Increment the value in the X register
    adc #$c4  ;Add the hex value $c4 to the A register

loop:
    jmp loop

.segment "vectors"
    .word 0
    .word start
    .word 0
