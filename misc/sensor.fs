\ *** needs to be on 72mhz!!!
pb11 constant trigger
pb10 constant echo

10 constant measurements

\ some scratch memory ... did not work?
\ create measure-buffer measurements allot
\ try this
measurements buffer: measure-buffer

: trigger-on 1 trigger io! ;
: trigger-off 0 trigger io! ;

: init-sensor 
    omode-pp trigger io-mode!
    imode-pull echo io-mode!
    trigger-off
;

: delay20us 200 0 do loop ;

: send-trigger
    trigger-on
    delay20us
    trigger-off
 ;
   
: wait-echo-comes
    begin
        echo io@  \ until it is 1 which is true value
    until
;

: count-echo
    0
    begin
        echo io@
        while     \ while it is high (true)
        1+
    repeat
;

: measure
    send-trigger
    wait-echo-comes
    count-echo
;

: measure-buffer.
    measurements 0
    do
        measure-buffer
        i cells + @ . cr
    loop
;


: full
    measurements 0
    do
        i
    \    measure
        measure-buffer
        i cells + !
    loop
;
        


: test 0 do measure loop ;

init-sensor
72mhz




