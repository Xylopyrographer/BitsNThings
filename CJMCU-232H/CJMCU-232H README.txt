The ubiquitous CJMCU-232H board.

Contains the original schematic and revised versions corrected the match the boards most frequently found.

This board is based on the FTDI reference design as found in section 6.1 "USB Bus Powered Configuration" of the "FT232H SINGLE CHANNEL HI-SPEED USB TO MULTIPURPOSE UART/FIFO IC" data sheet (version 1.82).

It was a real bear hunting down the schematic. A. Real. Bear!

Things of note:
- the outputs of the AC and AD busses are 3.3V. Need level shifters if you want ot use it on a 5V system.
- the 4-pin header on the end of the board has a connection to VCORE, which is 1.8V output of the FT232H. Not sure why that was brought out to a header as, per the data sheet, this is a "do not touch" pin so don't try and put any kind of a load on here.
- the FT232H is a very capable bit of hardware. Lot's of fun stuff on the interwebs about its many applications :)

