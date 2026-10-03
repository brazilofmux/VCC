* GIME timer semantics probe. Polls the timer latch in $FF92 (reading
* returns and clears it) with interrupts masked, so each poll pass is a
* fixed 18 cycles and the printed counts measure intervals exactly.
*   A: MSB write restarts: 600-line period, 4 intervals
*   B: LSB-only write must NOT restart: first interval still ~600,
*      then the new 767-line period
*   C: value 0 stops the timer; a later LSB-only nonzero write fires
*      at once (count ~0), then a 65-line period
OUTCH   equ     $A002           ; [vector] print char in A
OUTNUM  equ     $BDCC           ; Color BASIC: print D as unsigned decimal

        org     $3000
start   pshs    cc
        orcc    #$50            ; mask IRQ/FIRQ: deterministic polling
        clr     $FF91           ; TINS=0: timer counts scanlines
        lda     #$20
        sta     $FF92           ; latch only the timer interrupt
* --- A ---
        lda     #$58
        sta     $FF95           ; LSB (period-only update)
        lda     #$02
        sta     $FF94           ; MSB: restart with $258 = 600
        lda     $FF92           ; drop any stale latch
        ldu     #table
        lda     #4
        lbsr    measn
* --- B --- (we are just after a fire)
        lda     #$FF
        sta     $FF95           ; LSB only: period $2FF = 767, no restart
        lda     #3
        lbsr    measn
* --- C ---
        clr     $FF95           ; LSB only: counter $200, no restart
        clr     $FF94           ; MSB: restart with 0 -> timer stopped
        lda     $FF92
        ldx     #0              ; ~1.3 s idle: longer than any countdown
cwait   leax    -1,x
        bne     cwait
        lda     $FF92           ; any fire while stopped?
        anda    #$20
        sta     zflag           ; expect 0
        lda     #$40
        sta     $FF95           ; LSB only: 64, countdown overdue
        lda     #2
        lbsr    measn           ; expect ~0, then a 65-line interval
        puls    cc              ; timing done - now print
        lda     #'A
        lbsr    putc
        ldy     #table
        ldb     #4
        lbsr    dump
        lbsr    crlf
        lda     #'B
        lbsr    putc
        ldb     #3
        lbsr    dump
        lbsr    crlf
        lda     #'C
        lbsr    putc
        ldb     zflag
        clra
        lbsr    pnum
        ldb     #2
        lbsr    dump
        lbsr    crlf
        rts

* measn: measure A intervals back to back, recording each count at U
* (no printing between intervals - every count is a pure interval)
measn   sta     left
mloop   ldx     #0
poll    lda     $FF92
        bita    #$20
        bne     fired
        leax    1,x
        bra     poll
fired   stx     ,u++
        dec     left
        bne     mloop
        rts

* dump: print B words from Y
dump    pshs    b
        ldd     ,y++
        lbsr    pnum
        puls    b
        decb
        bne     dump
        rts

pnum    jsr     OUTNUM
        lda     #' 
putc    jmp     [OUTCH]
crlf    lda     #13
        bra     putc
left    fcb     0
zflag   fcb     0
table   rmb     18
        end     start
