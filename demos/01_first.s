screen = $4000

.segment "code"
start:
    lda #$01
    sta screen
    lda #$05
    sta screen + 1
    lda #$08
    sta screen + 2
loop:
    jmp loop

.segment "vectors"
    .word 0
    .word start
    .word 0
