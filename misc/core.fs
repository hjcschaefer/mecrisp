
4  constant io-ports  \ A..D
50 constant I2C.DELAY


\ Conditional compilation

\ Idea similar to http://lars.nocrew.org/dpans/dpansa15.htm#A.15.6.2.2532

: nexttoken ( -- addr len )
  begin
    token          \ Fetch new token.
  dup 0= while      \ If length of token is zero, end of line is reached.
    2drop cr query   \ Fetch new line.
  repeat
;

: [else] ( -- )
  1 \ Initial level of nesting
  begin
    nexttoken ( level addr len )

    2dup s" [if]"     compare
 >r 2dup s" [ifdef]"  compare r> or
 >r 2dup s" [ifndef]" compare r> or

    if
      2drop 1+  \ One more level of nesting
    else
      2dup s" [else]" compare
      if
        2drop 1- dup if 1+ then  \ Finished if [else] is reached in level 1. Skip [else] branch otherwise.
      else
        s" [then]" compare if 1- then  \ Level completed.
      then
    then

    ?dup 0=
  until

  immediate 0-foldable
;

: [then] ( -- ) immediate 0-foldable ;

: [if]   ( ? -- )                 0=  if postpone [else] then immediate 1-foldable ;
: [ifdef]  ( -- ) token find drop 0=  if postpone [else] then immediate 0-foldable ;
: [ifndef] ( -- ) token find drop 0<> if postpone [else] then immediate 0-foldable ;

\ hex output and dump utilities
\ adapted from mecrisp 2.0.2 (GPL3)

: .v ( ... -- ... )  \ view stack, this is a slightly cleaner version of .s
  depth 100 u< if
    ." Stack #" depth . ." < "
    \ -1 depth negate ?do sp@ i 2+ cells - @ . loop
    -1 depth negate ?do
      sp@ i 2+ cells - @
      dup $10000 u> if [char] $ emit hex. else . then
    loop
    ." >" cr
  else
    ." Stack underflow (" depth . ." )" cr
  then ;

: u.4 ( u -- ) 0 <# # # # # #> type ;
: u.2 ( u -- ) 0 <# # # #> type ;

: h.4 ( u -- ) base @ hex swap  u.4  base ! ;
: h.2 ( u -- ) base @ hex swap  u.2  base ! ;

$FF variable hex.empty  \ needs to be variable, some flash is zero when empty

: hexdump ( -- ) \ dumps entire flash as Intel hex
  cr
\ STM32F103x8: Complete: $FFFF $0000
\ STM32F103xB: 128 KB would need a somewhat different hex file format
  $FFFF $0000  \ Complete image with Dictionary
  do
    \ Check if this line is entirely empty:
    0                 \ Not worthy to print
    i #16 + i do      \ Scan data
      i c@ hex.empty @ <> or  \ Set flag if there is a non-empty byte
    loop

    if
      ." :10" i h.4 ." 00"  \ Write record-intro with 4 digits.
      $10           \ Begin checksum
      i          +  \ Sum the address bytes
      i 8 rshift +  \ separately into the checksum

      i #16 + i do
        i c@ h.2  \ Print data with 2 digits
        i c@ +    \ Sum it up for Checksum
      loop

      negate h.2  \ Write Checksum
      cr
    then

  #16 +loop
  ." :00000001FF" cr
;

\ adapted from mecrisp-stellaris 2.2.1a (GPL3)

: dump16 ( addr -- )  \ print 16 bytes memory
  $F bic
  dup hex. 2 spaces

  dup #16 + over do
    i c@ h.2 space  \ Print data with 2 digits
    i $F and 7 = if 2 spaces then
  loop

  2 spaces

  dup 16 + swap do
        i c@ $20 u>= i c@ $7F u< and if i c@ else [char] . then emit
        i $F and 7 = if space then
      loop

  cr
;

: dump ( addr len -- )  \ print a memory region
  cr
  over $F and if #16 + then  \ one more line if not aligned on 16
  begin
    swap ( len addr )
    dup dump16
    #16 + ( len addr+16 )
    swap #16 - ( addr+16 len-16 )
    dup 0 <=
  until
  2drop
;

\ I/O pin primitives

$40010800 constant GPIO-BASE
      $00 constant GPIO.CRL   \ reset $44444444 port Conf Register Low
      $04 constant GPIO.CRH   \ reset $44444444 port Conf Register High
      $08 constant GPIO.IDR   \ RO              Input Data Register
      $0C constant GPIO.ODR   \ reset 0         Output Data Register
      $10 constant GPIO.BSRR  \ reset 0         port Bit Set/Reset Reg
      $14 constant GPIO.BRR   \ reset 0         port Bit Reset Register

: bit ( u -- u )  \ turn a bit position into a single-bit mask
  1 swap lshift   ;

: io ( port# pin# -- pin )  \ combine port and pin into single int
  swap 8 lshift or   ;
: io# ( pin -- u )  \ convert pin to bit position
  $1F and   ;
: io-mask ( pin -- u )  \ convert pin to bit mask
  io# bit   ;
: io-port ( pin -- u )  \ convert pin to port number (A=0, B=1, etc)
  8 rshift   ;
: io-base ( pin -- addr )  \ convert pin to GPIO base address
  $F00 and 2 lshift GPIO-BASE +   ;

: 'f ( -- flags ) token find nip ;

: (io@)  (   pin -- pin* addr )
  dup io-mask swap io-base GPIO.IDR  +    ;
: (ioc!) (   pin -- pin* addr )
  dup io-mask swap io-base GPIO.BRR  +    ;
: (ios!) (   pin -- pin* addr )
  dup io-mask swap io-base GPIO.BSRR +    ;
: (iox!) (   pin -- pin* addr )
  dup io-mask swap io-base GPIO.ODR  +    ;
: (io!)  ( f pin -- pin* addr )
  swap 0= $10 and + dup io-mask swap io-base GPIO.BSRR +    ;

: io@ ( pin -- f )  \ get pin value (0 or -1)
  (io@)  bit@ exit [ $1000 setflags 2 h, ' (io@)  ,
  'f (io@)  h, ' bit@ , 'f bit@ h, ] ;
: ioc! ( pin -- )  \ clear pin to low
  (ioc!)    ! exit [ $1000 setflags 2 h, ' (ioc!) ,
  'f (ioc!) h, '    ! , 'f    ! h, ] ;
: ios! ( pin -- )  \ set pin to high
  (ios!)    ! exit [ $1000 setflags 2 h, ' (ios!) ,
  'f (ios!) h, '    ! , 'f    ! h, ] ;
: iox! ( pin -- )  \ toggle pin, not interrupt safe
  (iox!) xor! exit [ $1000 setflags 2 h, ' (iox!) ,
  'f (iox!) h, ' xor! , 'f xor! h, ] ;

: io! ( f pin -- )  \ set pin value
  (io!) ! exit
  [ $1000 setflags
    7 h,
    ' (ios!) , 'f  (ios!) h,
    ' rot    , 'f  rot    h,
    ' 0=     , 'f  0=     h,
      4      ,     $2000  h,
    ' and    , 'f  and    h,
    ' +      , 'f  +      h,
    ' !      , 'f  !      h, ] ;

%0000 constant IMODE-ADC    \ input, analog
%0100 constant IMODE-FLOAT  \ input, floating
%1000 constant IMODE-PULL   \ input, pull-up/down

%0001 constant OMODE-PP     \ output, push-pull
%0101 constant OMODE-OD     \ output, open drain
%1001 constant OMODE-AF-PP  \ alternate function, push-pull
%1101 constant OMODE-AF-OD  \ alternate function, open drain

%01 constant OMODE-SLOW  \ add to OMODE-* for 2 MHz iso 10 MHz drive
%10 constant OMODE-FAST  \ add to OMODE-* for 50 MHz iso 10 MHz drive

: io-mode! ( mode pin -- )  \ set the CNF and MODE bits for a pin
  dup io-base GPIO.CRL + over 8 and shr + >r ( R: crl/crh )
  io# 7 and 4 * ( mode shift )
  $F over lshift not ( mode shift mask )
  r@ @ and -rot lshift or r> ! ;

: io-modes! ( mode pin mask -- )  \ shorthand to config multiple pins of a port
  16 0 do
    i bit over and if
      >r  2dup ( mode pin mode pin R: mask ) $F bic i or io-mode!  r>
    then
  loop 2drop drop ;

: io. ( pin -- )  \ display readable GPIO registers associated with a pin
  cr
    ." PIN " dup io#  dup .  10 < if space then
   ." PORT " dup io-port [char] A + emit
  io-base
  ."   CRL " dup @ hex. 4 +
   ."  CRH " dup @ hex. 4 +
   ."  IDR " dup @ h.4  4 +
  ."   ODR " dup @ h.4 drop ;

\ pin definitions for chips up to 48 pins

0 0  io constant PA0       
0 1  io constant PA1       
0 2  io constant PA2    
0 3  io constant PA3    
0 4  io constant PA4    
0 5  io constant PA5    
0 6  io constant PA6    
0 7  io constant PA7    
0 8  io constant PA8    
0 9  io constant PA9    
0 10 io constant PA10   
0 11 io constant PA11   
0 12 io constant PA12   
0 13 io constant PA13      
0 14 io constant PA14      
0 15 io constant PA15      

2 13 io constant PC13
2 14 io constant PC14
2 15 io constant PC15

3 0  io constant PD0
3 1  io constant PD1

1 0  io constant PB0   
1 1  io constant PB1   
1 2  io constant PB2
1 3  io constant PB3
1 4  io constant PB4
1 5  io constant PB5
1 6  io constant PB6
1 7  io constant PB7
1 8  io constant PB8
1 9  io constant PB9
1 10 io constant PB10
1 11 io constant PB11
1 12 io constant PB12
1 13 io constant PB13  
1 14 io constant PB14  
1 15 io constant PB15  

\ pin definitions for chips up to 64 pins

2 0  io constant PC0
2 1  io constant PC1
2 2  io constant PC2
2 3  io constant PC3
2 4  io constant PC4
2 5  io constant PC5
2 6  io constant PC6
2 7  io constant PC7
2 8  io constant PC8
2 9  io constant PC9
2 10 io constant PC10
2 11 io constant PC11
2 12 io constant PC12

3 2  io constant PD2


\ base definitions for STM32F103
\ adapted from mecrisp-stellaris 2.2.1a (GPL3)
\ needs io.fs

: chipid ( -- u1 u2 u3 3 )  \ unique chip ID as N values on the stack
  $1FFFF7E8 @ $1FFFF7EC @ $1FFFF7F0 @ 3 ;
: hwid ( -- u )  \ a "fairly unique" hardware ID as single 32-bit int
  chipid 1 do xor loop ;
: flash-kb ( -- u )  \ return size of flash memory in KB
  $1FFFF7E0 h@ ;
: flash-pagesize ( addr - u )  \ return size of flash page at given address
  drop flash-kb 128 <= if 1024 else 2048 then ;

: io.all ( -- )  \ display all the readable GPIO registers
  io-ports 0 do i 0 io io. loop ;

$40010000 constant AFIO
     AFIO $4 + constant AFIO-MAPR

$40013800 constant USART1
   USART1 $8 + constant USART1-BRR

$40021000 constant RCC
     RCC $00 + constant RCC-CR
     RCC $04 + constant RCC-CFGR
     RCC $10 + constant RCC-APB1RSTR
     RCC $14 + constant RCC-AHBENR
     RCC $18 + constant RCC-APB2ENR
     RCC $1C + constant RCC-APB1ENR

$40022000 constant FLASH
    FLASH $0 + constant FLASH-ACR

: jtag-deinit ( -- )  \ disable JTAG on PB3 PB4 PA15
  25 bit AFIO-MAPR bis! ;
: swd-deinit ( -- )  \ disable JTAG as well as PA13 and PA14
  AFIO-MAPR @ %111 24 lshift bic 26 bit or AFIO-MAPR ! ;

\ adjusted for STM32F103 @ 72 MHz (original STM32F100 by Igor de om1zz, 2015)

8000000 variable clock-hz  \ the system clock is 8 MHz after reset

: baud ( u -- u )  \ calculate baud rate divider, based on current clock rate
  clock-hz @ swap / ;

: 8MHz ( -- )  \ set the main clock back to 8 MHz, keep baud rate at 115200
  0 RCC-CFGR !                    \ revert to HSI @ 8 MHz, no PLL
  $81 RCC-CR !                    \ turn off HSE and PLL, power-up value
  $18 FLASH-ACR !                 \ zero flash wait, enable half-cycle access
  8000000 clock-hz !  115200 baud USART1-BRR !  \ fix console baud rate
;

: 72MHz ( -- )  \ set the main clock to 72 MHz, keep baud rate at 115200
  8MHz                            \ make sure the PLL is off
  $12 FLASH-ACR !                 \ two flash mem wait states
  16 bit RCC-CR bis!              \ set HSEON
  begin 17 bit RCC-CR bit@ until  \ wait for HSERDY
  1 16 lshift                     \ HSE clock is 8 MHz Xtal source for PLL
  7 18 lshift or                  \ PLL factor: 8 MHz * 9 = 72 MHz = HCLK
  4  8 lshift or                  \ PCLK1 = HCLK/2
  2 14 lshift or                  \ ADCPRE = PCLK2/6
            2 or  RCC-CFGR !      \ PLL is the system clock
  24 bit RCC-CR bis!              \ set PLLON
  begin 25 bit RCC-CR bit@ until  \ wait for PLLRDY
  72000000 clock-hz !  115200 baud USART1-BRR !  \ fix console baud rate
;

0 variable ticks

: ++ticks ( -- ) 1 ticks +! ;  \ for use as systick irq handler

: systick-hz ( u -- )  \ enable systick interrupt at given frequency
  ['] ++ticks irq-systick !
  clock-hz @ swap /  1- $E000E014 !  7 $E000E010 ! ;
: systick-hz? ( -- u ) \ derive current systick frequency from clock
  clock-hz @  $E000E014 @ 1+  / ;

: micros ( -- n )  \ return elapsed microseconds, this wraps after some 2000s
\ assumes systick is running at 1000 Hz, overhead is about 1.8 us @ 72 MHz
\ get current ticks and systick, spinloops if ticks changed while we looked
  begin ticks @ $E000E018 @ over ticks @ <> while 2drop repeat
  $E000E014 @ 1+ swap -  \ convert down-counter to remaining
  clock-hz @ 1000000 / ( ticks systicks mhz )
  / swap 1000 * + ;

: millis ( -- u )  \ return elapsed milliseconds, this wraps after 49 days
  ticks @ ;

: us ( n -- )  \ microsecond delay using a busy loop, this won't switch tasks
  2 -  \ adjust for approximate overhead of this code itself
  micros +  begin dup micros - 0< until  drop ;

: ms ( n -- )  \ millisecond delay, multi-tasker aware (may switch tasks!)
  millis +  begin millis over - 0< while pause repeat  drop ;

\ : j0 micros 1000000 0 do 1 us loop micros swap - . ;
\ : j1 micros 1000000 0 do 5 us loop micros swap - . ;
\ : j2 micros 1000000 0 do 10 us loop micros swap - . ;
\ : j3 micros 1000000 0 do 20 us loop micros swap - . ;
\ : jn j0 j1 j2 j3 ;  \ sample results: 4065044 5988036 10542166 20833317


\ emulate c, which is not available in hardware on some chips.
\ copied from Mecrisp's common/charcomma.txt
0 variable c,collection

: c, ( c -- )  \ emulate c, with h,
  c,collection @ ?dup if $FF and swap 8 lshift or h,
                         0 c,collection !
                      else $100 or c,collection ! then ;

: calign ( -- )  \ must be called to flush after odd number of c, calls
  c,collection @ if 0 c, then ;

: list ( -- )  \ list all words in dictionary, short form
  cr dictionarystart begin
    dup 6 + ctype space
  dictionarynext until drop ;
\ hardware SPI driver

[ifndef] ssel  PA4 variable ssel  [then]  \ can be changed at run time
[ifndef] SCLK  PA5 constant SCLK  [then]
[ifndef] MISO  PA6 constant MISO  [then]
[ifndef] MOSI  PA7 constant MOSI  [then]

$40013000 constant SPI1
     SPI1 $0 + constant SPI1-CR1
     SPI1 $4 + constant SPI1-CR2
     SPI1 $8 + constant SPI1-SR
     SPI1 $C + constant SPI1-DR

: spi. ( -- )  \ display SPI hardware registers
  cr ." CR1 " SPI1-CR1 @ h.4
    ."  CR2 " SPI1-CR2 @ h.4
     ."  SR " SPI1-SR @ h.4 ;

: +spi ( -- ) ssel @ ioc! ;  \ select SPI
: -spi ( -- ) ssel @ ios! ;  \ deselect SPI

: >spi> ( c -- c )  \ hardware SPI, 8 bits
  SPI1-DR !  begin SPI1-SR @ 1 and until  SPI1-DR @ ;

\ single byte transfers
: spi> ( -- c ) 0 >spi> ;  \ read byte from SPI
: >spi ( c -- ) >spi> drop ;  \ write byte to SPI

: spi-init ( -- )  \ set up hardware SPI
  OMODE-PP    ssel @ io-mode! -spi
  OMODE-AF-PP SCLK   io-mode!
  IMODE-FLOAT MISO   io-mode!
  OMODE-AF-PP MOSI   io-mode!

  12 bit RCC-APB2ENR bis!  \ set SPI1EN
  %0000000001010100 SPI1-CR1 !  \ clk/8, i.e. 9 MHz, master
  SPI1-SR @ drop  \ appears to be needed to avoid hang in some cases
  2 bit SPI1-CR2 bis!  \ SS output enable
;
\ bit-banged i2c driver
\ adapted from http://excamera.com/sphinx/article-forth-i2c.html
\
\ This driver is master-only. It supports clock stretching.
\ There have to be 1..10 kΩ resistors on SDA and SCL to pull them up to idle state.

[ifndef] SCL  PB6 constant SCL  [then]
[ifndef] SDA  PB7 constant SDA  [then]

0 variable i2c.adr
0 variable i2c.nak
0 variable i2c.prv
0 variable i2c.cnt

: i2c-init ( -- )  \ initialise bit-banged I2C
  OMODE-OD SCL io-mode!
  OMODE-OD SDA io-mode! ;

: i2c-half ( -- )  \ half-cycle timing delay for I2C
  [ifdef] I2C.DELAY  I2C.DELAY 0 do loop  [then] inline ;

: SCL-ios! ( -- ) \ raise clock
  SCL ios! 
  begin SCL io@ until \ clock stretching to finish
;

: i2c-start ( -- )  \ with SCL high, change SDA from 1 to 0
  SDA ios! i2c-half SCL ios! i2c-half SDA ioc! i2c-half SCL ioc! ;
: i2c-stop  ( -- )  \ with SCL high, change SDA from 0 to 1
  SDA ioc! i2c-half SCL ios! i2c-half SDA ios! i2c-half ;

: b>i2c ( f -- )  \ send one I2C bit
  0<> SDA io! i2c-half SCL-ios! i2c-half SCL ioc! ;
: i2c>b ( -- f )  \ receive one I2C bit
  SDA ios! i2c-half SCL-ios! i2c-half SDA io@ SCL ioc! ;

: x>i2c ( b -- nak )  \ send one byte
  8 0 do dup 128 and b>i2c shl loop drop i2c>b ;
: xi2c> ( nak -- b )  \ read one byte
  0 8 0 do shl i2c>b 1 and + loop swap b>i2c ;

: i2c-flush ( -- )
  i2c.prv @ x>i2c  ?dup if i2c.nak ! then ;

: >i2c ( u -- )  \ send one byte out to the I2C bus
  i2c-flush  i2c.prv ! ;

: i2c> ( -- u )  \ read one byte back from the I2C bus
  i2c.cnt @ dup if
    1- dup i2c.cnt !
    0= xi2c>
  then ;

: i2c>h ( -- u )  i2c> i2c> 8 lshift or ;

: i2c-addr ( u -- )  \ start a new I2C transaction
  shl  dup i2c.adr !  i2c.prv !  0 i2c.nak !  i2c-start ;

: i2c-xfer ( u -- nak )  \ prepare for the reply
  i2c-flush
  dup i2c.cnt !  if
    i2c-start i2c.adr @ 1+ i2c.prv ! i2c-flush
  else
    SCL-ios!  \ i2c-stop
  then
  i2c.nak @
  dup if i2c-stop 0 i2c.cnt ! then  \ ignore reads if we had a nak
;

\ RTC example, this is small enough to leave it in
\ nak's will be silently ignored
\
\ : rtc: ( reg -- ) \ common i2c preamble for RTC
\   $68 i2c-tx drop >i2c drop ;
\ : rtc! ( v reg -- ) \ write v to RTC register
\   rtc: >i2c drop i2c-stop ;
\ : rtc@ ( reg -- v ) \ read RTC register
\   rtc: i2c-start $68 i2c-rx drop 1 i2c> i2c-stop ;

: i2c. ( -- )  \ scan and report all I2C devices on the bus
  128 0 do
    cr i h.2 ." :"
    16 0 do  space
      i j +
      dup $08 < over $77 > or if drop 2 spaces else
        dup i2c-addr  0 i2c-xfer  if drop ." --" else h.2 then
      then
    loop
  16 +loop ;
\ hardware timers

$00 constant TIM.CR1
$04 constant TIM.CR2
$0C constant TIM.DIER
$28 constant TIM.PSC
$2C constant TIM.ARR

create timer-table
  111 c,  \ TIM1  APB2
  0   c,  \ TIM2  APB1
  1   c,  \ TIM3  APB1
  2   c,  \ TIM4  APB1
  3   c,  \ TIM5  APB1
  4   c,  \ TIM6  APB1
  5   c,  \ TIM7  APB1
  113 c,  \ TIM8  APB2
  119 c,  \ TIM9  APB2
  120 c,  \ TIM10 APB2
  121 c,  \ TIM11 APB2
  6   c,  \ TIM12 APB1
  7   c,  \ TIM13 APB1
  8   c,  \ TIM14 APB1
calign

: timer-lookup ( n - pos ) 1- timer-table + c@ ;

: timer-base ( n -- addr )  \ return base address for timer 1..14
  timer-lookup
  dup 100 < if  $400 * $40000000  else  111 - $400 * $40012C00  then + ;

: timer-enabit ( n -- bit addr )  \ return bit and enable address for timer n
  timer-lookup
  dup 100 < if  bit RCC-APB1ENR  else  100 - bit RCC-APB2ENR  then ;

: timer-init ( u n -- )  \ enable timer n as free-running with period u
  dup timer-enabit bis!  \ clock enable
             timer-base >r
  dup 16 rshift TIM.PSC r@ + h!    \ upper 16 bits are used to set prescaler
                TIM.ARR r@ + h!    \ period is auto-reload value
         8 bit TIM.DIER r@ + bis!  \ UDE
  %010 4 lshift TIM.CR2 r@ + !     \ MMS = update
          0 bit TIM.CR1 r> + !     \ CEN
;

: timer-deinit ( n -- )  \ disable timer n
  timer-enabit bic! ;
\ Pulse Width Modulation
\ needs io-stm32f1.fs
\ needs timer-stm32f1.fs

\ The following pins are supported for PWM setup on STM32F1xx:
\   TIM1:   PA8  PA9  PA10 PA11
\   TIM2:   PA0  PA1  PA2  PA3
\   TIM3:   PA6  PA7  PB0  PB1
\   TIM4:   PB6  PB7  PB8  PB9
\ Pins sharing a timer will run at the same repetition rate.
\ Repetition rates which are a divisor of 7200 will be exact.

: p2tim ( pin -- n ) \ convert pin to timer (1..4)
  case
    dup PA4 <                ?of 2 endof
    dup PB1 >                ?of 4 endof
    dup PA7 > over PB0 < and ?of 1 endof
    dup PB6 <                ?of 3 endof
  endcase ;

: p2cmp ( pin -- n ) \ convert pin to output comp-reg# - 1 (0..3)
  dup
  case
    dup PA4 <                ?of 0 endof
    dup PB1 >                ?of 2 endof
    dup PA7 > over PB0 < and ?of 0 endof
    dup PB6 <                ?of 2 endof
  endcase
  + 3 and ;

\ : t dup p2tim . p2cmp . ." : " ;
\ : u                             \ expected output:
\   cr PA8 t PA9 t PA10 t PA11 t  \  1 0 : 1 1 : 1 2 : 1 3 :
\   cr PA0 t PA1 t PA2  t PA3  t  \  2 0 : 2 1 : 2 2 : 2 3 :
\   cr PA6 t PA7 t PB0  t PB1  t  \  3 0 : 3 1 : 3 2 : 3 3 :
\   cr PB6 t PB7 t PB8  t PB9  t  \  4 0 : 4 1 : 4 2 : 4 3 :
\ ;
\ u

: pwm-init ( hz pin -- )  \ set up PWM for pin, using specified repetition rate
  >r  OMODE-PP r@ io-mode!  r@ ioc!  \ start with pwm zero, i.e. fully off
  7200 swap / 1- 16 lshift 10000 or  r@ p2tim timer-init
  $78 r@ p2cmp 1 and 8 * lshift ( $0078 or $7800 )
  r@ p2tim timer-base $18 + r@ p2cmp 2 and 2* + bis!
  r@ p2cmp 4 * bit r> p2tim timer-base $20 + bis! ;

: pwm-deinit ( pin -- )  \ disable PWM, but leave timer running
  dup p2cmp 4 * bit swap p2tim timer-base $20 + bic! ;

\ since zero PWM generates a single blip, set the GPIO pin to normal mode and
\ set its output to "0" - in all other cases, switch the pin to alternate mode

: pwm ( u pin -- )  \ set pwm rate, 0 = full off, 10000 = full on
  over if OMODE-AF-PP else OMODE-PP then over io-mode!  \ for fully-off case
  10000 rot - swap  \ reverse the sense of the PWM count value
  dup p2cmp cells swap p2tim timer-base + $34 + !  \ save to CCR1..4
;
\ simple one-shot ADC

$40012400 constant ADC1
    ADC1 $00 + constant ADC1-SR
    ADC1 $04 + constant ADC1-CR1
    ADC1 $08 + constant ADC1-CR2
    ADC1 $0C + constant ADC1-SMPR1
    ADC1 $10 + constant ADC1-SMPR2
    ADC1 $2C + constant ADC1-SQR1
    ADC1 $30 + constant ADC1-SQR2
    ADC1 $34 + constant ADC1-SQR3
    ADC1 $4C + constant ADC1-DR

$40020000 constant DMA1
    DMA1 $00 + constant DMA1-ISR
    DMA1 $04 + constant DMA1-IFCR
    DMA1 $08 + constant DMA1-CCR1
    DMA1 $0C + constant DMA1-CNDTR1
    DMA1 $10 + constant DMA1-CPAR1
    DMA1 $14 + constant DMA1-CMAR1

: adc-calib ( -- )  \ perform an ADC calibration cycle
  2 bit ADC1-CR2 bis!  begin 2 bit ADC1-CR2 bit@ 0= until ;

: adc-once ( -- u )  \ read ADC value once
  0 bit ADC1-CR2 bis!  \ set ADON to start ADC
  begin 1 bit ADC1-SR bit@ until  \ wait until EOC set
  ADC1-DR @ ;

: adc-init ( -- )  \ initialise ADC
  9 bit RCC-APB2ENR bis!  \ set ADC1EN
  23 bit  \ set TSVREFE for vRefInt use
   0 bit or ADC1-CR2 bis!  \ set ADON to enable ADC
  \ 7.5 cycles sampling time is enough for 18 kΩ to ground, measures as zero
  \ even 239.5 cycles is not enough for 470 kΩ, it still leaves 70 mV residual
  %111 21 lshift ADC1-SMPR1 bis! \ set SMP17 to 239.5 cycles for vRefInt
  %110110110 ADC1-SMPR2 bis! \ set SMP0/1/2 to 71.5 cycles, i.e. 83 µs/conv
  adc-once drop ;

: adc# ( pin -- n )  \ convert pin number to adc index
\ nasty way to map the pins (a "c," table offset lookup might be simpler)
\   PA0..7 => 0..7, PB0..1 => 8..9, PC0..5 => 10..15
  dup io# swap  io-port ?dup if shl + 6 + then ;

: adc ( pin -- u )  \ read ADC value
\ IMODE-ADC over io-mode!
\ nasty way to map the pins (a "c," table offset lookup might be simpler)
\   PA0..7 => 0..7, PB0..1 => 8..9, PC0..5 => 10..15
  adc# ADC1-SQR3 !  adc-once ;

: adc1-dma ( addr count pin rate -- )  \ continuous DMA-based conversion
  3 timer-init        \ set the ADC trigger rate using timer 3
  adc-init  adc drop  \ perform one conversion to set up the ADC
  2dup 0 fill         \ clear sampling buffer

    0 bit RCC-AHBENR bis!  \ DMA1EN clock enable
      2/ DMA1-CNDTR1 !     \ 2-byte entries
          DMA1-CMAR1 !     \ write to address passed as input
  ADC1-DR DMA1-CPAR1 !     \ read from ADC1

                0   \ register settings for CCR1 of DMA1:
  %01 10 lshift or  \ MSIZE = 16-bits
   %01 8 lshift or  \ PSIZE = 16 bits
          7 bit or  \ MINC
          5 bit or  \ CIRC
                    \ DIR = from peripheral to mem
          0 bit or  \ EN
      DMA1-CCR1 !

                 0   \ ADC1 triggers on timer 3 and feeds DMA1:
          23 bit or  \ TSVREFE
          20 bit or  \ EXTTRIG
  %100 17 lshift or  \ timer 3 TRGO event
           8 bit or  \ DMA
           0 bit or  \ ADON
        ADC1-CR2 ! ;

: adc-vcc ( -- mv )  \ return estimated Vcc, based on 1.2V internal bandgap
  4096 1200  17 ADC1-SQR3 !  adc-once  */ ;

: hello ( -- ) flash-kb . ." KB <g6s> " hwid hex.
  $10000 compiletoflash here -  flashvar-here compiletoram here -
  ." ram/flash: " . . ." free " ;

: init ( -- )  \ board initialisation
  jtag-deinit  \ disable JTAG, we only need SWD
  72MHz
  1000 systick-hz
  hello ." ok." cr
;
