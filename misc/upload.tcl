#!/usr/bin/env expect
# ** SEND CORE FILE TO MECRISP!!!

if {$argc != 1} {
    puts "Usage $argv0 <forth-file>"
    exit 0
}
set fname [lindex $argv 0]

# Seems to work!!!! But it takes a while!
spawn /home/pi/bin/uploadfile
expect "Terminal ready"
send "\x01\x13"
expect "*** file: "
send "$fname\r"
#send "core.fs\r"
expect eof

# after this finishes, use mecrisp to watch it finish.


