\ some useful tools

\ Lets check out some delays

pb11 constant pin

omode-pp pin io-mode!  

: delay ;

: on 1 pin io! ;
: off 0 pin io! ;


\ calibrated via osci
: d5us 3 0 do loop ;
: d10us 9 0 do loop ;
: d100us 130 0 do loop ;
: d500us 46 0 do d10us loop ;

