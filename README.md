# Mega6502

Mega6502 is an experiment implementing a minimal 8 bit platform using an Arduino and a 6502.

# Hardware

Mega6502 requires an Ardunio Mega 2560 development board and CMOS 65c02. The 65c02 should be connected to the Arduino as follows

					
		  5v      GND                                 5v   Arduino 
		  |        |       +------\/------+           |
		  |        +----  1| Vss     /RES |40 -- 3k3 -+--- 52
		  +--- 3k3 -----  2| RDY       ϕ2 |39  
		  |               3| ϕ1       /SO |38  
		  +--- 3k3 -----  4| IRQ       ϕ0 |37 -------- 53
		  |               5| NC        NC |36  
		  +--- 3k3 -----  6| /NMI      NC |35  
		  |               7| SYNC     R/W |34 -------- 51
		  +-------------  8| Vcc       D0 |33 -------- 49
		    22 ---------  9| A0        D1 |32 -------- 48 
		    23 --------- 10| A1        D2 |31 -------- 47
		    24 --------- 11| A2        D3 |30 -------- 46
		    25 --------- 12| A3        D4 |29 -------- 45
		    26 --------- 13| A4        D5 |28 -------- 44
		    27 --------- 14| A5        D6 |27 -------- 43
		    28 --------- 15| A6        D7 |26 -------- 42
		    29 --------- 16| A7       A15 |25 -------- 30
		    37 --------- 17| A8       A14 |24 -------- 31
		    36 --------- 18| A9       A13 |23 -------- 32
		    35 --------- 19| A10      A12 |22 -------- 33
		    34 --------- 20| A11      Vss |21 ---+
		                   +--------------+      |
		                                        GND


