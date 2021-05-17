EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 8
Title "Car bs controller nex mega v3"
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Wire Wire Line
	4300 4750 4650 4750
Wire Wire Line
	5200 4750 4650 4750
Connection ~ 4650 4750
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR027
U 1 1 5FFD6482
P 4650 4950
F 0 "#PWR027" H 4650 4700 50  0001 C CNN
F 1 "GND" H 4655 4777 50  0000 C CNN
F 2 "" H 4650 4950 50  0001 C CNN
F 3 "" H 4650 4950 50  0001 C CNN
	1    4650 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 4750 4650 4950
Text GLabel 1650 6850 0    50   Input ~ 0
SPARE_INPUT_1
Wire Wire Line
	4150 6050 4150 6250
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR022
U 1 1 60016359
P 4150 6250
F 0 "#PWR022" H 4150 6000 50  0001 C CNN
F 1 "GND" H 4155 6077 50  0000 C CNN
F 2 "" H 4150 6250 50  0001 C CNN
F 3 "" H 4150 6250 50  0001 C CNN
	1    4150 6250
	1    0    0    -1  
$EndComp
Connection ~ 4150 6050
Wire Wire Line
	4700 6050 4150 6050
Wire Wire Line
	3800 6050 4150 6050
Wire Wire Line
	3800 5650 4150 5650
Connection ~ 3800 5650
Wire Wire Line
	3600 5650 3800 5650
Wire Wire Line
	3800 5750 3800 5650
Wire Wire Line
	4150 5650 4700 5650
Connection ~ 4150 5650
Wire Wire Line
	4150 5750 4150 5650
Connection ~ 4700 5650
Wire Wire Line
	4700 5750 4700 5650
$Comp
L Car_bs_controller_nex_mega-rescue:C-Device C14
U 1 1 6001636D
P 4700 5900
F 0 "C14" H 4815 5946 50  0000 L CNN
F 1 "10n" H 4815 5855 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 4738 5750 50  0001 C CNN
F 3 "~" H 4700 5900 50  0001 C CNN
	1    4700 5900
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:D_Zener-Device D16
U 1 1 60016373
P 4150 5900
F 0 "D16" V 4104 5980 50  0000 L CNN
F 1 "N" V 4195 5980 50  0000 L CNN
F 2 "Diode_SMD:D_MiniMELF" H 4150 5900 50  0001 C CNN
F 3 "~" H 4150 5900 50  0001 C CNN
	1    4150 5900
	0    1    1    0   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R8
U 1 1 60016379
P 3800 5900
F 0 "R8" H 3870 5946 50  0000 L CNN
F 1 "N" H 3870 5855 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 3730 5900 50  0001 C CNN
F 3 "~" H 3800 5900 50  0001 C CNN
	1    3800 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 5650 3200 5650
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R6
U 1 1 60016386
P 3450 5650
F 0 "R6" V 3243 5650 50  0000 C CNN
F 1 "4k7" V 3334 5650 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 3380 5650 50  0001 C CNN
F 3 "~" H 3450 5650 50  0001 C CNN
	1    3450 5650
	0    1    1    0   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:LL4148-Diode D10
U 1 1 6001638D
P 3050 5650
F 0 "D10" H 3050 5867 50  0000 C CNN
F 1 "LL4148" H 3050 5776 50  0000 C CNN
F 2 "Diode_SMD:D_MiniMELF" H 3050 5475 50  0001 C CNN
F 3 "http://www.vishay.com/docs/85557/ll4148.pdf" H 3050 5650 50  0001 C CNN
	1    3050 5650
	1    0    0    -1  
$EndComp
Text GLabel 6100 6850 2    50   Input ~ 0
MCU_spare_input_1
Wire Wire Line
	4200 7250 4200 7450
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR023
U 1 1 6001941F
P 4200 7450
F 0 "#PWR023" H 4200 7200 50  0001 C CNN
F 1 "GND" H 4205 7277 50  0000 C CNN
F 2 "" H 4200 7450 50  0001 C CNN
F 3 "" H 4200 7450 50  0001 C CNN
	1    4200 7450
	1    0    0    -1  
$EndComp
Connection ~ 4200 7250
Wire Wire Line
	4750 7250 4200 7250
Wire Wire Line
	3850 7250 4200 7250
Wire Wire Line
	3850 6850 4200 6850
Connection ~ 3850 6850
Wire Wire Line
	3650 6850 3850 6850
Wire Wire Line
	3850 6950 3850 6850
Wire Wire Line
	4200 6850 4750 6850
Connection ~ 4200 6850
Wire Wire Line
	4200 6950 4200 6850
Connection ~ 4750 6850
Wire Wire Line
	4750 6950 4750 6850
$Comp
L Car_bs_controller_nex_mega-rescue:C-Device C15
U 1 1 60019433
P 4750 7100
F 0 "C15" H 4865 7146 50  0000 L CNN
F 1 "10n" H 4865 7055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 4788 6950 50  0001 C CNN
F 3 "~" H 4750 7100 50  0001 C CNN
	1    4750 7100
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:D_Zener-Device D17
U 1 1 60019439
P 4200 7100
F 0 "D17" V 4154 7180 50  0000 L CNN
F 1 "4V7" V 4245 7180 50  0000 L CNN
F 2 "Diode_SMD:D_MiniMELF" H 4200 7100 50  0001 C CNN
F 3 "~" H 4200 7100 50  0001 C CNN
	1    4200 7100
	0    1    1    0   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R10
U 1 1 6001943F
P 3850 7100
F 0 "R10" H 3920 7146 50  0000 L CNN
F 1 "10k" H 3920 7055 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 3780 7100 50  0001 C CNN
F 3 "~" H 3850 7100 50  0001 C CNN
	1    3850 7100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 6850 3250 6850
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R7
U 1 1 6001944C
P 3500 6850
F 0 "R7" V 3293 6850 50  0000 C CNN
F 1 "15k" V 3384 6850 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 3430 6850 50  0001 C CNN
F 3 "~" H 3500 6850 50  0001 C CNN
	1    3500 6850
	0    1    1    0   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:LL4148-Diode D11
U 1 1 60019453
P 3100 6850
F 0 "D11" H 3100 7067 50  0000 C CNN
F 1 "LL4148" H 3100 6976 50  0000 C CNN
F 2 "Diode_SMD:D_MiniMELF" H 3100 6675 50  0001 C CNN
F 3 "http://www.vishay.com/docs/85557/ll4148.pdf" H 3100 6850 50  0001 C CNN
	1    3100 6850
	-1   0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR029
U 1 1 6049F500
P 5800 6100
F 0 "#PWR029" H 5800 5850 50  0001 C CNN
F 1 "GND" H 5805 5927 50  0000 C CNN
F 2 "" H 5800 6100 50  0001 C CNN
F 3 "" H 5800 6100 50  0001 C CNN
	1    5800 6100
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR028
U 1 1 604A26F9
P 5750 7350
F 0 "#PWR028" H 5750 7100 50  0001 C CNN
F 1 "GND" H 5755 7177 50  0000 C CNN
F 2 "" H 5750 7350 50  0001 C CNN
F 3 "" H 5750 7350 50  0001 C CNN
	1    5750 7350
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR033
U 1 1 6048CD88
P 6100 4800
F 0 "#PWR033" H 6100 4550 50  0001 C CNN
F 1 "GND" H 6105 4627 50  0000 C CNN
F 2 "" H 6100 4800 50  0001 C CNN
F 3 "" H 6100 4800 50  0001 C CNN
	1    6100 4800
	1    0    0    -1  
$EndComp
Text GLabel 6650 3250 2    50   Input ~ 0
MCU_ignition_switch_input
$Comp
L Car_bs_controller_nex_mega-rescue:LL4148-Diode D12
U 1 1 5FFD4EE7
P 3450 3250
F 0 "D12" H 3450 3467 50  0000 C CNN
F 1 "LL4148" H 3450 3376 50  0000 C CNN
F 2 "Diode_SMD:D_MiniMELF" H 3450 3075 50  0001 C CNN
F 3 "http://www.vishay.com/docs/85557/ll4148.pdf" H 3450 3250 50  0001 C CNN
	1    3450 3250
	-1   0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R9
U 1 1 5FFD4EEE
P 3850 3250
F 0 "R9" V 3643 3250 50  0000 C CNN
F 1 "15k" V 3734 3250 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 3780 3250 50  0001 C CNN
F 3 "~" H 3850 3250 50  0001 C CNN
	1    3850 3250
	0    1    1    0   
$EndComp
Wire Wire Line
	3700 3250 3600 3250
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R14
U 1 1 5FFD4EFB
P 4200 3500
F 0 "R14" H 4270 3546 50  0000 L CNN
F 1 "10k" H 4270 3455 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 4130 3500 50  0001 C CNN
F 3 "~" H 4200 3500 50  0001 C CNN
	1    4200 3500
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:D_Zener-Device D18
U 1 1 5FFD4F01
P 4550 3500
F 0 "D18" V 4504 3580 50  0000 L CNN
F 1 "4V7" V 4595 3580 50  0000 L CNN
F 2 "Diode_SMD:D_MiniMELF" H 4550 3500 50  0001 C CNN
F 3 "~" H 4550 3500 50  0001 C CNN
	1    4550 3500
	0    1    1    0   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:C-Device C16
U 1 1 5FFD4F07
P 5100 3500
F 0 "C16" H 5215 3546 50  0000 L CNN
F 1 "10n" H 5215 3455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 5138 3350 50  0001 C CNN
F 3 "~" H 5100 3500 50  0001 C CNN
	1    5100 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 3350 5100 3250
Connection ~ 5100 3250
Wire Wire Line
	4550 3350 4550 3250
Connection ~ 4550 3250
Wire Wire Line
	4550 3250 5100 3250
Wire Wire Line
	4200 3350 4200 3250
Wire Wire Line
	4000 3250 4200 3250
Connection ~ 4200 3250
Wire Wire Line
	4200 3250 4550 3250
Wire Wire Line
	4200 3650 4550 3650
Wire Wire Line
	5100 3650 4550 3650
Connection ~ 4550 3650
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR024
U 1 1 5FFD4F1B
P 4550 3850
F 0 "#PWR024" H 4550 3600 50  0001 C CNN
F 1 "GND" H 4555 3677 50  0000 C CNN
F 2 "" H 4550 3850 50  0001 C CNN
F 3 "" H 4550 3850 50  0001 C CNN
	1    4550 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 3650 4550 3850
Text GLabel 1950 3250 0    50   Input ~ 0
IGNITION_SWITCH_INPUT
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R15
U 1 1 5FFC4115
P 4250 1200
F 0 "R15" H 4320 1246 50  0000 L CNN
F 1 "N" H 4320 1155 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 4180 1200 50  0001 C CNN
F 3 "~" H 4250 1200 50  0001 C CNN
	1    4250 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 1050 4250 950 
Wire Wire Line
	4250 1350 4600 1350
Wire Wire Line
	3800 4350 3700 4350
$Comp
L Car_bs_controller_nex_mega-rescue:LL4148-Diode D15
U 1 1 5FFD644E
P 3550 4350
F 0 "D15" H 3550 4567 50  0000 C CNN
F 1 "LL4148" H 3550 4476 50  0000 C CNN
F 2 "Diode_SMD:D_MiniMELF" H 3550 4175 50  0001 C CNN
F 3 "http://www.vishay.com/docs/85557/ll4148.pdf" H 3550 4350 50  0001 C CNN
	1    3550 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 2150 3650 2150
Text GLabel 2050 2150 0    50   Input ~ 0
PROXIMITY_SENSOR_INPUT
$Comp
L Car_bs_controller_nex_mega-rescue:LL4148-Diode D14
U 1 1 5FFD29A5
P 3500 2150
F 0 "D14" H 3500 2367 50  0000 C CNN
F 1 "LL4148" H 3500 2276 50  0000 C CNN
F 2 "Diode_SMD:D_MiniMELF" H 3500 1975 50  0001 C CNN
F 3 "http://www.vishay.com/docs/85557/ll4148.pdf" H 3500 2150 50  0001 C CNN
	1    3500 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 4350 4650 4350
Connection ~ 4300 4350
Wire Wire Line
	4100 4350 4300 4350
Wire Wire Line
	4300 4450 4300 4350
Wire Wire Line
	4650 4350 5200 4350
Connection ~ 4650 4350
Wire Wire Line
	4650 4450 4650 4350
Connection ~ 5200 4350
Wire Wire Line
	5200 4450 5200 4350
$Comp
L Car_bs_controller_nex_mega-rescue:C-Device C19
U 1 1 5FFD646E
P 5200 4600
F 0 "C19" H 5315 4646 50  0000 L CNN
F 1 "10n" H 5315 4555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 5238 4450 50  0001 C CNN
F 3 "~" H 5200 4600 50  0001 C CNN
	1    5200 4600
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:D_Zener-Device D21
U 1 1 5FFD6468
P 4650 4600
F 0 "D21" V 4604 4680 50  0000 L CNN
F 1 "N" V 4695 4680 50  0000 L CNN
F 2 "Diode_SMD:D_MiniMELF" H 4650 4600 50  0001 C CNN
F 3 "~" H 4650 4600 50  0001 C CNN
	1    4650 4600
	0    1    1    0   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R17
U 1 1 5FFD6462
P 4300 4600
F 0 "R17" H 4370 4646 50  0000 L CNN
F 1 "N" H 4370 4555 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 4230 4600 50  0001 C CNN
F 3 "~" H 4300 4600 50  0001 C CNN
	1    4300 4600
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R13
U 1 1 5FFD6455
P 3950 4350
F 0 "R13" V 3743 4350 50  0000 C CNN
F 1 "4k7" V 3834 4350 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 3880 4350 50  0001 C CNN
F 3 "~" H 3950 4350 50  0001 C CNN
	1    3950 4350
	0    1    1    0   
$EndComp
Text GLabel 6550 4350 2    50   Input ~ 0
MCU_low_washer_water_level_input
Text GLabel 1950 4350 0    50   Input ~ 0
LOW_WASHER_WATER_LEVEL_INPUT
Wire Wire Line
	4600 2550 4600 2750
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR026
U 1 1 5FFD29D9
P 4600 2750
F 0 "#PWR026" H 4600 2500 50  0001 C CNN
F 1 "GND" H 4605 2577 50  0000 C CNN
F 2 "" H 4600 2750 50  0001 C CNN
F 3 "" H 4600 2750 50  0001 C CNN
	1    4600 2750
	1    0    0    -1  
$EndComp
Connection ~ 4600 2550
Wire Wire Line
	5150 2550 4600 2550
Wire Wire Line
	4250 2550 4600 2550
Wire Wire Line
	4250 2150 4600 2150
Connection ~ 4250 2150
Wire Wire Line
	4050 2150 4250 2150
Wire Wire Line
	4250 2250 4250 2150
Wire Wire Line
	4600 2150 5150 2150
Connection ~ 4600 2150
Wire Wire Line
	4600 2250 4600 2150
Wire Wire Line
	5150 2250 5150 2150
$Comp
L Car_bs_controller_nex_mega-rescue:C-Device C18
U 1 1 5FFD29C5
P 5150 2400
F 0 "C18" H 5265 2446 50  0000 L CNN
F 1 "10n" H 5265 2355 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 5188 2250 50  0001 C CNN
F 3 "~" H 5150 2400 50  0001 C CNN
	1    5150 2400
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:D_Zener-Device D20
U 1 1 5FFD29BF
P 4600 2400
F 0 "D20" V 4554 2480 50  0000 L CNN
F 1 "N" V 4645 2480 50  0000 L CNN
F 2 "Diode_SMD:D_MiniMELF" H 4600 2400 50  0001 C CNN
F 3 "~" H 4600 2400 50  0001 C CNN
	1    4600 2400
	0    1    1    0   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R16
U 1 1 5FFD29B9
P 4250 2400
F 0 "R16" H 4320 2446 50  0000 L CNN
F 1 "N" H 4320 2355 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 4180 2400 50  0001 C CNN
F 3 "~" H 4250 2400 50  0001 C CNN
	1    4250 2400
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R12
U 1 1 5FFD29AC
P 3900 2150
F 0 "R12" V 3693 2150 50  0000 C CNN
F 1 "4k7" V 3784 2150 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 3830 2150 50  0001 C CNN
F 3 "~" H 3900 2150 50  0001 C CNN
	1    3900 2150
	0    1    1    0   
$EndComp
Text GLabel 6950 2150 2    50   Input ~ 0
MCU_proximity_input
Wire Wire Line
	4600 1350 4600 1550
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR025
U 1 1 5FFC99CF
P 4600 1550
F 0 "#PWR025" H 4600 1300 50  0001 C CNN
F 1 "GND" H 4605 1377 50  0000 C CNN
F 2 "" H 4600 1550 50  0001 C CNN
F 3 "" H 4600 1550 50  0001 C CNN
	1    4600 1550
	1    0    0    -1  
$EndComp
Connection ~ 4600 1350
Wire Wire Line
	5150 1350 4600 1350
Wire Wire Line
	4250 950  4600 950 
Connection ~ 4250 950 
Wire Wire Line
	4050 950  4250 950 
Wire Wire Line
	4600 950  5150 950 
Connection ~ 4600 950 
Wire Wire Line
	4600 1050 4600 950 
Connection ~ 5150 950 
Wire Wire Line
	5150 1050 5150 950 
$Comp
L Car_bs_controller_nex_mega-rescue:C-Device C17
U 1 1 5FFC6293
P 5150 1200
F 0 "C17" H 5265 1246 50  0000 L CNN
F 1 "10n" H 5265 1155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 5188 1050 50  0001 C CNN
F 3 "~" H 5150 1200 50  0001 C CNN
	1    5150 1200
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:D_Zener-Device D19
U 1 1 5FFC56A3
P 4600 1200
F 0 "D19" V 4554 1280 50  0000 L CNN
F 1 "N" V 4645 1280 50  0000 L CNN
F 2 "Diode_SMD:D_MiniMELF" H 4600 1200 50  0001 C CNN
F 3 "~" H 4600 1200 50  0001 C CNN
	1    4600 1200
	0    1    1    0   
$EndComp
Wire Wire Line
	3750 950  3650 950 
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R11
U 1 1 5FFC27A0
P 3900 950
F 0 "R11" V 3693 950 50  0000 C CNN
F 1 "4k7" V 3784 950 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 3830 950 50  0001 C CNN
F 3 "~" H 3900 950 50  0001 C CNN
	1    3900 950 
	0    1    1    0   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:LL4148-Diode D13
U 1 1 5FFC21D6
P 3500 950
F 0 "D13" H 3500 1167 50  0000 C CNN
F 1 "LL4148" H 3500 1076 50  0000 C CNN
F 2 "Diode_SMD:D_MiniMELF" H 3500 775 50  0001 C CNN
F 3 "http://www.vishay.com/docs/85557/ll4148.pdf" H 3500 950 50  0001 C CNN
	1    3500 950 
	1    0    0    -1  
$EndComp
Text GLabel 1900 950  0    50   Input ~ 0
DOOR_SWITCH_INPUT
Text GLabel 6350 5650 2    50   Output ~ 0
MCU_water_flow_sensor
Text GLabel 1750 5650 0    50   Input ~ 0
WATER_FLOW_SENSOR_INPUT
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR032
U 1 1 602592B3
P 6100 3700
F 0 "#PWR032" H 6100 3450 50  0001 C CNN
F 1 "GND" H 6105 3527 50  0000 C CNN
F 2 "" H 6100 3700 50  0001 C CNN
F 3 "" H 6100 3700 50  0001 C CNN
	1    6100 3700
	1    0    0    -1  
$EndComp
Text GLabel 6350 950  2    50   Input ~ 0
MCU_door_switch_input
$Comp
L Diode:BAV99 D24
U 1 1 60AD9A0A
P 6050 950
F 0 "D24" V 6096 1029 50  0000 L CNN
F 1 "BAV99" V 6005 1029 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 6050 450 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/BAV99_SER.pdf" H 6050 950 50  0001 C CNN
	1    6050 950 
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5150 950  6250 950 
Connection ~ 6250 950 
Wire Wire Line
	6250 950  6350 950 
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR030
U 1 1 60AE9404
P 6050 1500
F 0 "#PWR030" H 6050 1250 50  0001 C CNN
F 1 "GND" H 6055 1327 50  0000 C CNN
F 2 "" H 6050 1500 50  0001 C CNN
F 3 "" H 6050 1500 50  0001 C CNN
	1    6050 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 1500 6050 1250
Text GLabel 5700 600  0    50   Input ~ 0
+5v_FROM_MCU
Wire Wire Line
	5700 600  6050 600 
Wire Wire Line
	6050 600  6050 650 
$Comp
L Diode:BAV99 D26
U 1 1 60AEFCC9
P 6100 3250
F 0 "D26" V 6146 3329 50  0000 L CNN
F 1 "BAV99" V 6055 3329 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 6100 2750 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/BAV99_SER.pdf" H 6100 3250 50  0001 C CNN
	1    6100 3250
	0    -1   -1   0   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR031
U 1 1 60AFA531
P 6050 2600
F 0 "#PWR031" H 6050 2350 50  0001 C CNN
F 1 "GND" H 6055 2427 50  0000 C CNN
F 2 "" H 6050 2600 50  0001 C CNN
F 3 "" H 6050 2600 50  0001 C CNN
	1    6050 2600
	1    0    0    -1  
$EndComp
Text GLabel 5650 1850 0    50   Input ~ 0
+5v_FROM_MCU
Wire Wire Line
	5650 1850 6050 1850
Wire Wire Line
	5100 3250 6300 3250
Connection ~ 6300 3250
Wire Wire Line
	6300 3250 6650 3250
$Comp
L Diode:BAV99 D25
U 1 1 60B0D13F
P 6050 2150
F 0 "D25" V 6096 2229 50  0000 L CNN
F 1 "BAV99" V 6005 2229 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 6050 1650 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/BAV99_SER.pdf" H 6050 2150 50  0001 C CNN
	1    6050 2150
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5150 2150 6250 2150
Connection ~ 5150 2150
Connection ~ 6250 2150
Wire Wire Line
	6250 2150 6950 2150
Wire Wire Line
	6050 2600 6050 2450
Text GLabel 5800 2900 0    50   Input ~ 0
+5v_FROM_MCU
Wire Wire Line
	5800 2900 6100 2900
Wire Wire Line
	6100 2900 6100 2950
$Comp
L Diode:BAV99 D27
U 1 1 60B1E00F
P 6100 4350
F 0 "D27" V 6146 4429 50  0000 L CNN
F 1 "BAV99" V 6055 4429 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 6100 3850 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/BAV99_SER.pdf" H 6100 4350 50  0001 C CNN
	1    6100 4350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6100 3700 6100 3550
Text GLabel 5900 4050 0    50   Input ~ 0
+5v_FROM_MCU
Wire Wire Line
	5900 4050 6100 4050
Wire Wire Line
	6100 4650 6100 4800
Wire Wire Line
	5200 4350 6300 4350
Connection ~ 6300 4350
Wire Wire Line
	6300 4350 6550 4350
$Comp
L Diode:BAV99 D23
U 1 1 60B32D14
P 5800 5650
F 0 "D23" V 5846 5729 50  0000 L CNN
F 1 "BAV99" V 5755 5729 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5800 5150 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/BAV99_SER.pdf" H 5800 5650 50  0001 C CNN
	1    5800 5650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4700 5650 6000 5650
Connection ~ 6000 5650
Wire Wire Line
	6000 5650 6350 5650
Wire Wire Line
	5800 6100 5800 5950
Text GLabel 5500 5350 0    50   Input ~ 0
+5v_FROM_MCU
Wire Wire Line
	5500 5350 5800 5350
$Comp
L Diode:BAV99 D22
U 1 1 60B3FA15
P 5750 6850
F 0 "D22" V 5796 6929 50  0000 L CNN
F 1 "BAV99" V 5705 6929 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5750 6350 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/BAV99_SER.pdf" H 5750 6850 50  0001 C CNN
	1    5750 6850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4750 6850 5950 6850
Connection ~ 5950 6850
Wire Wire Line
	5950 6850 6100 6850
Wire Wire Line
	5750 7350 5750 7150
Text GLabel 5450 6550 0    50   Input ~ 0
+5v_FROM_MCU
Wire Wire Line
	5450 6550 5750 6550
Wire Wire Line
	1900 950  3350 950 
Wire Wire Line
	2050 2150 3350 2150
Wire Wire Line
	1950 3250 3300 3250
Wire Wire Line
	1950 4350 3400 4350
Wire Wire Line
	1750 5650 2900 5650
Wire Wire Line
	1650 6850 2950 6850
$EndSCHEMATC
