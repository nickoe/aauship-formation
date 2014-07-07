EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:special
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:sssw-cache
EELAYER 24 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Solid State Safety Switch"
Date ""
Rev ""
Comp "AAUSHIP"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MOSFET_N Q1
U 1 1 52F79D2B
P 6100 3350
F 0 "Q1" H 6050 3500 60  0000 R CNN
F 1 "SUP85" H 6050 3150 60  0000 R CNN
F 2 "" H 6100 3350 60  0000 C CNN
F 3 "" H 6100 3350 60  0000 C CNN
	1    6100 3350
	0    1    -1   0   
$EndComp
$Comp
L CONN_2 P1
U 1 1 52F7A037
P 4500 3150
F 0 "P1" V 4450 3150 40  0000 C CNN
F 1 "INPUT" V 4550 3150 40  0000 C CNN
F 2 "" H 4500 3150 60  0000 C CNN
F 3 "" H 4500 3150 60  0000 C CNN
	1    4500 3150
	-1   0    0    -1  
$EndComp
$Comp
L CONN_2 P3
U 1 1 52F7A049
P 6850 3150
F 0 "P3" V 6800 3150 40  0000 C CNN
F 1 "OUTPUT" V 6900 3150 40  0000 C CNN
F 2 "" H 6850 3150 60  0000 C CNN
F 3 "" H 6850 3150 60  0000 C CNN
	1    6850 3150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR1
U 1 1 52F7A074
P 4950 3150
F 0 "#PWR1" H 4950 3150 30  0001 C CNN
F 1 "GND" H 4950 3080 30  0001 C CNN
F 2 "" H 4950 3150 60  0000 C CNN
F 3 "" H 4950 3150 60  0000 C CNN
	1    4950 3150
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 52F7A096
P 5150 3850
F 0 "R1" V 5230 3850 40  0000 C CNN
F 1 "10K" V 5157 3851 40  0000 C CNN
F 2 "" V 5080 3850 30  0000 C CNN
F 3 "" H 5150 3850 30  0000 C CNN
	1    5150 3850
	0    -1   -1   0   
$EndComp
$Comp
L R R2
U 1 1 52F7A0A8
P 5500 4150
F 0 "R2" V 5580 4150 40  0000 C CNN
F 1 "1K" V 5507 4151 40  0000 C CNN
F 2 "" V 5430 4150 30  0000 C CNN
F 3 "" H 5500 4150 30  0000 C CNN
	1    5500 4150
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR3
U 1 1 52F7A0BA
P 5500 4500
F 0 "#PWR3" H 5500 4500 30  0001 C CNN
F 1 "GND" H 5500 4430 30  0001 C CNN
F 2 "" H 5500 4500 60  0000 C CNN
F 3 "" H 5500 4500 60  0000 C CNN
	1    5500 4500
	1    0    0    -1  
$EndComp
$Comp
L CONN_2 P2
U 1 1 52F7A13E
P 4500 3950
F 0 "P2" V 4450 3950 40  0000 C CNN
F 1 "ACTIVATE" V 4550 3950 40  0000 C CNN
F 2 "" H 4500 3950 60  0000 C CNN
F 3 "" H 4500 3950 60  0000 C CNN
	1    4500 3950
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR2
U 1 1 52F7A159
P 4950 4150
F 0 "#PWR2" H 4950 4150 30  0001 C CNN
F 1 "GND" H 4950 4080 30  0001 C CNN
F 2 "" H 4950 4150 60  0000 C CNN
F 3 "" H 4950 4150 60  0000 C CNN
	1    4950 4150
	1    0    0    -1  
$EndComp
Text Notes 5300 2900 0    60   Italic 0
High Power Path
$Comp
L LED D1
U 1 1 52F7A220
P 5750 4650
F 0 "D1" H 5750 4750 50  0000 C CNN
F 1 "LED" H 5750 4550 50  0000 C CNN
F 2 "" H 5750 4650 60  0000 C CNN
F 3 "" H 5750 4650 60  0000 C CNN
	1    5750 4650
	0    1    1    0   
$EndComp
$Comp
L R R3
U 1 1 52F7A23C
P 5750 4150
F 0 "R3" V 5830 4150 40  0000 C CNN
F 1 "330" V 5757 4151 40  0000 C CNN
F 2 "" V 5680 4150 30  0000 C CNN
F 3 "" H 5750 4150 30  0000 C CNN
	1    5750 4150
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR4
U 1 1 52F7A350
P 5750 4950
F 0 "#PWR4" H 5750 4950 30  0001 C CNN
F 1 "GND" H 5750 4880 30  0001 C CNN
F 2 "" H 5750 4950 60  0000 C CNN
F 3 "" H 5750 4950 60  0000 C CNN
	1    5750 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 3250 5900 3250
Wire Wire Line
	4850 3050 6500 3050
Wire Wire Line
	5400 3850 6100 3850
Wire Wire Line
	5500 3900 5500 3850
Connection ~ 5500 3850
Wire Wire Line
	5500 4500 5500 4400
Wire Wire Line
	4950 4050 4950 4150
Wire Wire Line
	4950 4050 4850 4050
Wire Wire Line
	6300 3250 6500 3250
Wire Notes Line
	7050 2900 4300 2900
Wire Notes Line
	4300 2900 4300 3700
Wire Wire Line
	4900 3850 4850 3850
Wire Wire Line
	5750 4450 5750 4400
Wire Wire Line
	5750 3900 5750 3850
Connection ~ 5750 3850
Wire Wire Line
	5750 4950 5750 4850
Wire Notes Line
	7050 2900 7050 3700
Wire Notes Line
	7050 3700 4300 3700
Wire Wire Line
	6100 3850 6100 3550
Wire Wire Line
	4950 3050 4950 3150
Connection ~ 4950 3050
Text Notes 7100 3250 0    60   ~ 0
Female\nbullets
Text Notes 4250 3250 2    60   ~ 0
Male\nbullets
$EndSCHEMATC
