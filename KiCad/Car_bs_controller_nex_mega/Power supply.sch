EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 8
Title "Car bs controller nex mega v3"
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR011
U 1 1 5FFA18F3
P 6600 2750
F 0 "#PWR011" H 6600 2500 50  0001 C CNN
F 1 "GND" H 6605 2577 50  0000 C CNN
F 2 "" H 6600 2750 50  0001 C CNN
F 3 "" H 6600 2750 50  0001 C CNN
	1    6600 2750
	1    0    0    -1  
$EndComp
Text GLabel 10550 1550 2    50   Output ~ 0
7.2V_stab
$Comp
L Car_bs_controller_nex_mega-rescue:D-Device D5
U 1 1 5FFE06F0
P 3700 4350
F 0 "D5" H 3700 4133 50  0000 C CNN
F 1 "1N4148" H 3700 4224 50  0000 C CNN
F 2 "Diode_SMD:D_MiniMELF" H 3700 4350 50  0001 C CNN
F 3 "~" H 3700 4350 50  0001 C CNN
	1    3700 4350
	-1   0    0    1   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:C-Device C10
U 1 1 5FFAF314
P 8950 1800
F 0 "C10" H 9065 1846 50  0000 L CNN
F 1 "100n" H 9000 1700 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 8988 1650 50  0001 C CNN
F 3 "~" H 8950 1800 50  0001 C CNN
	1    8950 1800
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR018
U 1 1 5FFB0B98
P 9400 2250
F 0 "#PWR018" H 9400 2000 50  0001 C CNN
F 1 "GND" H 9405 2077 50  0000 C CNN
F 2 "" H 9400 2250 50  0001 C CNN
F 3 "" H 9400 2250 50  0001 C CNN
	1    9400 2250
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:D-Device D6
U 1 1 5FFCD531
P 3750 3000
F 0 "D6" H 3750 2783 50  0000 C CNN
F 1 "1N4148" H 3750 2874 50  0000 C CNN
F 2 "Diode_SMD:D_MiniMELF" H 3750 3000 50  0001 C CNN
F 3 "~" H 3750 3000 50  0001 C CNN
	1    3750 3000
	-1   0    0    1   
$EndComp
Text GLabel 1950 3000 0    50   Input ~ 10
Power_enable(ACC,BUTTON)
$Comp
L Car_bs_controller_nex_mega-rescue:+12V-power #PWR04
U 1 1 5FFF9F15
P 2950 1150
F 0 "#PWR04" H 2950 1000 50  0001 C CNN
F 1 "+12V" H 2965 1323 50  0000 C CNN
F 2 "" H 2950 1150 50  0001 C CNN
F 3 "" H 2950 1150 50  0001 C CNN
	1    2950 1150
	1    0    0    -1  
$EndComp
Text GLabel 1850 4350 0    50   Input ~ 0
MCU_main_supply_enable
Text GLabel 1550 1600 0    50   Input ~ 10
Power_supply_12V
$Comp
L Car_bs_controller_nex_mega-rescue:D_TVS-Device D2
U 1 1 6003116B
P 2950 1850
F 0 "D2" V 2900 1650 50  0000 L CNN
F 1 "SMBJ18CA" V 3000 1400 50  0000 L CNN
F 2 "Diode_SMD:D_SMB" H 2950 1850 50  0001 C CNN
F 3 "~" H 2950 1850 50  0001 C CNN
	1    2950 1850
	0    1    1    0   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR05
U 1 1 60036567
P 2950 2100
F 0 "#PWR05" H 2950 1850 50  0001 C CNN
F 1 "GND" H 2955 1927 50  0000 C CNN
F 2 "" H 2950 2100 50  0001 C CNN
F 3 "" H 2950 2100 50  0001 C CNN
	1    2950 2100
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:C-Device C3
U 1 1 600389F5
P 4700 1900
F 0 "C3" H 4815 1946 50  0000 L CNN
F 1 "100n" H 4815 1855 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4738 1750 50  0001 C CNN
F 3 "~" H 4700 1900 50  0001 C CNN
	1    4700 1900
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR08
U 1 1 6003B907
P 4700 2100
F 0 "#PWR08" H 4700 1850 50  0001 C CNN
F 1 "GND" H 4705 1927 50  0000 C CNN
F 2 "" H 4700 2100 50  0001 C CNN
F 3 "" H 4700 2100 50  0001 C CNN
	1    4700 2100
	1    0    0    -1  
$EndComp
Text GLabel 10350 4100 2    50   Output ~ 0
5V_sensors_stab
$Comp
L Car_bs_controller_nex_mega-rescue:C-Device C6
U 1 1 5FFCAF91
P 7400 4450
F 0 "C6" H 7500 4400 50  0000 L CNN
F 1 "100n" H 7450 4300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7438 4300 50  0001 C CNN
F 3 "~" H 7400 4450 50  0001 C CNN
	1    7400 4450
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR016
U 1 1 5FFCAF9A
P 8850 4950
F 0 "#PWR016" H 8850 4700 50  0001 C CNN
F 1 "GND" H 8855 4777 50  0000 C CNN
F 2 "" H 8850 4950 50  0001 C CNN
F 3 "" H 8850 4950 50  0001 C CNN
	1    8850 4950
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:LED-Device D1
U 1 1 60143DA7
P 1800 750
F 0 "D1" H 1800 650 50  0000 C CNN
F 1 "PWR" H 1800 850 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric" H 1800 750 50  0001 C CNN
F 3 "~" H 1800 750 50  0001 C CNN
	1    1800 750 
	-1   0    0    1   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR01
U 1 1 6015E304
P 2150 900
F 0 "#PWR01" H 2150 650 50  0001 C CNN
F 1 "GND" H 2155 727 50  0000 C CNN
F 2 "" H 2150 900 50  0001 C CNN
F 3 "" H 2150 900 50  0001 C CNN
	1    2150 900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 900  2150 750 
Wire Wire Line
	2150 750  1950 750 
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R1
U 1 1 60173AF5
P 1450 750
F 0 "R1" V 1350 750 50  0000 C CNN
F 1 "3k9" V 1600 750 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 1380 750 50  0001 C CNN
F 3 "~" H 1450 750 50  0001 C CNN
	1    1450 750 
	0    1    1    0   
$EndComp
Wire Wire Line
	1300 750  1200 750 
Wire Wire Line
	1600 750  1650 750 
Wire Wire Line
	1950 3000 2200 3000
$Comp
L Car_bs_controller_nex_mega-rescue:CP-Device C2
U 1 1 6007A74A
P 4300 1900
F 0 "C2" H 4050 1900 50  0000 L CNN
F 1 "220mk 25v" H 3850 1750 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D8.0mm_P3.80mm" H 4338 1750 50  0001 C CNN
F 3 "~" H 4300 1900 50  0001 C CNN
	1    4300 1900
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR07
U 1 1 6007B2EE
P 4300 2100
F 0 "#PWR07" H 4300 1850 50  0001 C CNN
F 1 "GND" H 4305 1927 50  0000 C CNN
F 2 "" H 4300 2100 50  0001 C CNN
F 3 "" H 4300 2100 50  0001 C CNN
	1    4300 2100
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:D_TVS-Device D3
U 1 1 6022BBB0
P 3200 3250
F 0 "D3" V 3150 3350 50  0000 L CNN
F 1 "SMAJ18CA" V 3250 3350 50  0000 L CNN
F 2 "Diode_SMD:D_SMA" H 3200 3250 50  0001 C CNN
F 3 "~" H 3200 3250 50  0001 C CNN
	1    3200 3250
	0    1    1    0   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR06
U 1 1 6023AE86
P 3200 3600
F 0 "#PWR06" H 3200 3350 50  0001 C CNN
F 1 "GND" H 3205 3427 50  0000 C CNN
F 2 "" H 3200 3600 50  0001 C CNN
F 3 "" H 3200 3600 50  0001 C CNN
	1    3200 3600
	1    0    0    -1  
$EndComp
Text GLabel 1850 5500 0    50   Input ~ 0
MCU_5V_sensors_enable
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R5
U 1 1 5FFCAFA5
P 3150 5500
F 0 "R5" V 2943 5500 50  0000 C CNN
F 1 "2k" V 3034 5500 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 3080 5500 50  0001 C CNN
F 3 "~" H 3150 5500 50  0001 C CNN
	1    3150 5500
	0    -1   1    0   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:D-Device D7
U 1 1 5FFCAF84
P 3750 5500
F 0 "D7" H 3750 5283 50  0000 C CNN
F 1 "1N4148" H 3750 5374 50  0000 C CNN
F 2 "Diode_SMD:D_MiniMELF" H 3750 5500 50  0001 C CNN
F 3 "~" H 3750 5500 50  0001 C CNN
	1    3750 5500
	-1   0    0    1   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:CP-Device C12
U 1 1 603E38BF
P 9800 1800
F 0 "C12" H 9600 1950 50  0000 L CNN
F 1 "220mk 16v" V 10000 1450 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D6.3mm_P2.50mm" H 9838 1650 50  0001 C CNN
F 3 "~" H 9800 1800 50  0001 C CNN
	1    9800 1800
	1    0    0    -1  
$EndComp
Text GLabel 1050 2100 0    50   Input ~ 10
GND
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R2
U 1 1 6008ED16
P 2350 3000
F 0 "R2" V 2143 3000 50  0000 C CNN
F 1 "2k" V 2234 3000 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 2280 3000 50  0001 C CNN
F 3 "~" H 2350 3000 50  0001 C CNN
	1    2350 3000
	0    -1   1    0   
$EndComp
Wire Wire Line
	1550 1600 1700 1600
$Comp
L Car_bs_controller_nex_mega-rescue:Fuse-Device F1
U 1 1 6012347C
P 1850 1600
F 0 "F1" V 1653 1600 50  0000 C CNN
F 1 "3A" V 1744 1600 50  0000 C CNN
F 2 "Fuse:Fuse_2010_5025Metric" V 1780 1600 50  0001 C CNN
F 3 "~" H 1850 1600 50  0001 C CNN
	1    1850 1600
	0    1    1    0   
$EndComp
Wire Wire Line
	10200 1650 10200 1550
Connection ~ 10200 1550
Wire Wire Line
	10200 1550 10550 1550
Wire Wire Line
	6700 1700 6700 1600
Connection ~ 6700 1600
Wire Wire Line
	6700 1600 7150 1600
Wire Wire Line
	7150 1500 6600 1500
Wire Wire Line
	6600 1500 6600 2750
Wire Wire Line
	7150 1800 5950 1800
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R4
U 1 1 6079F2C5
P 3100 4350
F 0 "R4" V 2893 4350 50  0000 C CNN
F 1 "2k" V 2984 4350 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 3030 4350 50  0001 C CNN
F 3 "~" H 3100 4350 50  0001 C CNN
	1    3100 4350
	0    -1   1    0   
$EndComp
Wire Wire Line
	1850 4350 2950 4350
Wire Wire Line
	3250 4350 3550 4350
$Comp
L Car_bs_controller_nex_mega-rescue:C-Device C4
U 1 1 607AF3B5
P 5950 2400
F 0 "C4" H 6065 2446 50  0000 L CNN
F 1 "10mk" H 6050 2300 50  0000 L CNN
F 2 "Capacitor_SMD:C_1210_3225Metric_Pad1.33x2.70mm_HandSolder" H 5988 2250 50  0001 C CNN
F 3 "~" H 5950 2400 50  0001 C CNN
	1    5950 2400
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR010
U 1 1 607AFB42
P 6350 2750
F 0 "#PWR010" H 6350 2500 50  0001 C CNN
F 1 "GND" H 6355 2577 50  0000 C CNN
F 2 "" H 6350 2750 50  0001 C CNN
F 3 "" H 6350 2750 50  0001 C CNN
	1    6350 2750
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR09
U 1 1 607B091D
P 5950 2750
F 0 "#PWR09" H 5950 2500 50  0001 C CNN
F 1 "GND" H 5955 2577 50  0000 C CNN
F 2 "" H 5950 2750 50  0001 C CNN
F 3 "" H 5950 2750 50  0001 C CNN
	1    5950 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	6350 2750 6350 2550
Wire Wire Line
	5950 2550 5950 2750
Wire Wire Line
	5950 2250 5950 1800
Connection ~ 5950 1800
Wire Wire Line
	5950 1800 5350 1800
Wire Wire Line
	1850 5500 3000 5500
Wire Wire Line
	3300 5500 3600 5500
Wire Wire Line
	3850 4350 5350 4350
Wire Wire Line
	3900 5500 5650 5500
Wire Wire Line
	5650 5500 5650 1900
Wire Wire Line
	5650 1900 6350 1900
Wire Wire Line
	6350 2250 6350 1900
Connection ~ 6350 1900
Wire Wire Line
	6350 1900 7150 1900
Wire Wire Line
	8100 1850 8400 1850
Wire Wire Line
	8400 1850 8400 4100
Wire Wire Line
	4700 2050 4700 2100
Wire Wire Line
	4700 1750 4700 1600
Connection ~ 4700 1600
Wire Wire Line
	4700 1600 6700 1600
Wire Wire Line
	10200 2250 10200 1950
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR014
U 1 1 6081775B
P 8250 2300
F 0 "#PWR014" H 8250 2050 50  0001 C CNN
F 1 "GND" H 8255 2127 50  0000 C CNN
F 2 "" H 8250 2300 50  0001 C CNN
F 3 "" H 8250 2300 50  0001 C CNN
	1    8250 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 2300 8250 1750
Wire Wire Line
	8250 1750 8100 1750
Wire Wire Line
	8100 1650 8250 1650
Wire Wire Line
	8250 1650 8250 1750
Connection ~ 8250 1750
$Comp
L Car_bs_controller_nex_mega-rescue:D_TVS-Device D9
U 1 1 60B55E2D
P 9900 4450
F 0 "D9" V 9850 4250 50  0000 L CNN
F 1 "SMBJ5CA" H 9900 4650 50  0000 L CNN
F 2 "Diode_SMD:D_SMA" H 9900 4450 50  0001 C CNN
F 3 "~" H 9900 4450 50  0001 C CNN
	1    9900 4450
	0    1    1    0   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR013
U 1 1 60B5795F
P 7900 4950
F 0 "#PWR013" H 7900 4700 50  0001 C CNN
F 1 "GND" H 7905 4777 50  0000 C CNN
F 2 "" H 7900 4950 50  0001 C CNN
F 3 "" H 7900 4950 50  0001 C CNN
	1    7900 4950
	1    0    0    -1  
$EndComp
Connection ~ 8400 4100
$Comp
L Cbc_power_module:Cbc_power_module D8
U 1 1 607905E0
P 7650 1350
F 0 "D8" H 7625 1425 50  0000 C CNN
F 1 "Cbc_power_module" H 7625 1334 50  0000 C CNN
F 2 "Cbc_power_module:Cbc_power_module" H 7650 1350 50  0001 C CNN
F 3 "" H 7650 1350 50  0001 C CNN
	1    7650 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 1700 6700 1700
$Comp
L Car_bs_controller_nex_mega-rescue:PWR_FLAG-power #FLG02
U 1 1 600CBA9D
P 2650 1100
F 0 "#FLG02" H 2650 1175 50  0001 C CNN
F 1 "PWR_FLAG" H 2650 1273 50  0000 C CNN
F 2 "" H 2650 1100 50  0001 C CNN
F 3 "~" H 2650 1100 50  0001 C CNN
	1    2650 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 1100 2650 1200
Wire Wire Line
	2950 1150 2950 1200
Wire Wire Line
	2650 1200 2950 1200
Wire Wire Line
	1400 2100 2300 2100
Wire Wire Line
	1050 2100 1400 2100
Connection ~ 1400 2100
Wire Wire Line
	1400 2050 1400 2100
$Comp
L Car_bs_controller_nex_mega-rescue:PWR_FLAG-power #FLG01
U 1 1 600D63F4
P 1400 2050
F 0 "#FLG01" H 1400 2125 50  0001 C CNN
F 1 "PWR_FLAG" H 1400 2223 50  0000 C CNN
F 2 "" H 1400 2050 50  0001 C CNN
F 3 "~" H 1400 2050 50  0001 C CNN
	1    1400 2050
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:CP-Device C7
U 1 1 6082FD2B
P 7900 4450
F 0 "C7" H 7700 4600 50  0000 L CNN
F 1 "220mk 10v" V 8100 4100 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D6.3mm_P2.50mm" H 7938 4300 50  0001 C CNN
F 3 "~" H 7900 4450 50  0001 C CNN
	1    7900 4450
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:CP-Device C9
U 1 1 60830AB9
P 8850 4450
F 0 "C9" H 8650 4600 50  0000 L CNN
F 1 "220mk 10v" V 9050 4100 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D6.3mm_P2.50mm" H 8888 4300 50  0001 C CNN
F 3 "~" H 8850 4450 50  0001 C CNN
	1    8850 4450
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR020
U 1 1 6083550C
P 9900 4950
F 0 "#PWR020" H 9900 4700 50  0001 C CNN
F 1 "GND" H 9905 4777 50  0000 C CNN
F 2 "" H 9900 4950 50  0001 C CNN
F 3 "" H 9900 4950 50  0001 C CNN
	1    9900 4950
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR015
U 1 1 6083972F
P 8400 5000
F 0 "#PWR015" H 8400 4750 50  0001 C CNN
F 1 "GND" H 8405 4827 50  0000 C CNN
F 2 "" H 8400 5000 50  0001 C CNN
F 3 "" H 8400 5000 50  0001 C CNN
	1    8400 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	8400 4600 8400 5000
Wire Wire Line
	8400 4300 8400 4100
Wire Wire Line
	8400 4100 8850 4100
$Comp
L Car_bs_controller_nex_mega-rescue:CP-Device C8
U 1 1 60842D6A
P 8400 4450
F 0 "C8" H 8200 4600 50  0000 L CNN
F 1 "220mk 10v" V 8600 4100 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D6.3mm_P2.50mm" H 8438 4300 50  0001 C CNN
F 3 "~" H 8400 4450 50  0001 C CNN
	1    8400 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 4100 7400 4300
Wire Wire Line
	7400 4100 7900 4100
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR012
U 1 1 60849056
P 7400 5000
F 0 "#PWR012" H 7400 4750 50  0001 C CNN
F 1 "GND" H 7405 4827 50  0000 C CNN
F 2 "" H 7400 5000 50  0001 C CNN
F 3 "" H 7400 5000 50  0001 C CNN
	1    7400 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 5000 7400 4600
Wire Wire Line
	9900 4950 9900 4600
Wire Wire Line
	8850 4950 8850 4600
Wire Wire Line
	7900 4950 7900 4600
Wire Wire Line
	7900 4300 7900 4100
Connection ~ 7900 4100
Wire Wire Line
	7900 4100 8400 4100
Wire Wire Line
	8850 4300 8850 4100
Connection ~ 8850 4100
Wire Wire Line
	9900 4300 9900 4100
Connection ~ 9900 4100
Wire Wire Line
	9900 4100 10350 4100
Wire Wire Line
	4300 2100 4300 2050
Wire Wire Line
	4300 1750 4300 1600
Connection ~ 4300 1600
Wire Wire Line
	4300 1600 4700 1600
$Comp
L Device:L L1
U 1 1 607B3569
P 3800 1600
F 0 "L1" V 3990 1600 50  0000 C CNN
F 1 "47uH" V 3899 1600 50  0000 C CNN
F 2 "Inductor_SMD:L_7.3x7.3_H4.5" H 3800 1600 50  0001 C CNN
F 3 "~" H 3800 1600 50  0001 C CNN
	1    3800 1600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2950 1700 2950 1600
Connection ~ 2950 1600
Wire Wire Line
	2950 2000 2950 2100
Wire Wire Line
	2000 1600 2300 1600
Wire Wire Line
	2950 1600 2950 1200
Connection ~ 2950 1200
$Comp
L Car_bs_controller_nex_mega-rescue:C-Device C1
U 1 1 607CC6BD
P 2300 1800
F 0 "C1" H 2100 1850 50  0000 L CNN
F 1 "100n" H 2050 1700 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2338 1650 50  0001 C CNN
F 3 "~" H 2300 1800 50  0001 C CNN
	1    2300 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 1950 2300 2100
Connection ~ 2300 2100
Wire Wire Line
	2300 1650 2300 1600
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR02
U 1 1 607CB561
P 2300 2100
F 0 "#PWR02" H 2300 1850 50  0001 C CNN
F 1 "GND" H 2305 1927 50  0000 C CNN
F 2 "" H 2300 2100 50  0001 C CNN
F 3 "" H 2300 2100 50  0001 C CNN
	1    2300 2100
	1    0    0    -1  
$EndComp
Connection ~ 2300 1600
Wire Wire Line
	9800 1650 9800 1550
Connection ~ 9800 1550
Wire Wire Line
	9800 1550 10200 1550
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR019
U 1 1 607E1224
P 9800 2250
F 0 "#PWR019" H 9800 2000 50  0001 C CNN
F 1 "GND" H 9805 2077 50  0000 C CNN
F 2 "" H 9800 2250 50  0001 C CNN
F 3 "" H 9800 2250 50  0001 C CNN
	1    9800 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	9800 2250 9800 1950
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR021
U 1 1 600FD2BE
P 10200 2250
F 0 "#PWR021" H 10200 2000 50  0001 C CNN
F 1 "GND" H 10205 2077 50  0000 C CNN
F 2 "" H 10200 2250 50  0001 C CNN
F 3 "" H 10200 2250 50  0001 C CNN
	1    10200 2250
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:CP-Device C13
U 1 1 607E9BDB
P 10200 1800
F 0 "C13" H 10000 1950 50  0000 L CNN
F 1 "220mk 16v" V 10400 1450 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D6.3mm_P2.50mm" H 10238 1650 50  0001 C CNN
F 3 "~" H 10200 1800 50  0001 C CNN
	1    10200 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	8850 4100 9900 4100
$Comp
L Car_bs_controller_nex_mega-rescue:CP-Device C11
U 1 1 607EAD87
P 9400 1800
F 0 "C11" H 9200 1950 50  0000 L CNN
F 1 "220mk 16v" V 9600 1450 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D6.3mm_P2.50mm" H 9438 1650 50  0001 C CNN
F 3 "~" H 9400 1800 50  0001 C CNN
	1    9400 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 1550 8950 1550
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR017
U 1 1 607EBEEA
P 8950 2250
F 0 "#PWR017" H 8950 2000 50  0001 C CNN
F 1 "GND" H 8955 2077 50  0000 C CNN
F 2 "" H 8950 2250 50  0001 C CNN
F 3 "" H 8950 2250 50  0001 C CNN
	1    8950 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 1650 9400 1550
Connection ~ 9400 1550
Wire Wire Line
	9400 1550 9800 1550
Wire Wire Line
	9400 1950 9400 2250
Wire Wire Line
	8950 2250 8950 1950
Wire Wire Line
	8950 1650 8950 1550
Connection ~ 8950 1550
Wire Wire Line
	8950 1550 9400 1550
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R3
U 1 1 60819E3D
P 2750 3250
F 0 "R3" V 2543 3250 50  0000 C CNN
F 1 "1k" V 2634 3250 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 2680 3250 50  0001 C CNN
F 3 "~" H 2750 3250 50  0001 C CNN
	1    2750 3250
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5350 1800 5350 3000
Wire Wire Line
	3200 3100 3200 3000
Wire Wire Line
	3200 3000 3600 3000
Wire Wire Line
	3200 3400 3200 3600
Wire Wire Line
	2500 3000 2750 3000
Connection ~ 3200 3000
Wire Wire Line
	2750 3100 2750 3000
Connection ~ 2750 3000
Wire Wire Line
	2750 3000 3200 3000
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR03
U 1 1 6082BA29
P 2750 3600
F 0 "#PWR03" H 2750 3350 50  0001 C CNN
F 1 "GND" H 2755 3427 50  0000 C CNN
F 2 "" H 2750 3600 50  0001 C CNN
F 3 "" H 2750 3600 50  0001 C CNN
	1    2750 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 3600 2750 3400
Wire Wire Line
	3900 3000 5350 3000
Connection ~ 5350 3000
Wire Wire Line
	5350 3000 5350 4350
Wire Wire Line
	2950 1600 3150 1600
Wire Wire Line
	3450 1600 3650 1600
Wire Wire Line
	2300 1600 2950 1600
$Comp
L Car_bs_controller_nex_mega-rescue:C-Device C5
U 1 1 5FFFA29A
P 6350 2400
F 0 "C5" H 6465 2446 50  0000 L CNN
F 1 "1n" H 6465 2355 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 6388 2250 50  0001 C CNN
F 3 "~" H 6350 2400 50  0001 C CNN
	1    6350 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 1600 4300 1600
Wire Wire Line
	1200 750  1200 1300
Wire Wire Line
	1200 1300 2300 1300
Wire Wire Line
	2300 1300 2300 1600
$Comp
L Car_bs_controller_nex_mega-rescue:D-Device D4
U 1 1 5FFE57B9
P 3300 1600
F 0 "D4" H 3300 1817 50  0000 C CNN
F 1 "SS54" H 3300 1726 50  0000 C CNN
F 2 "Diode_SMD:D_SMB" H 3300 1600 50  0001 C CNN
F 3 "~" H 3300 1600 50  0001 C CNN
	1    3300 1600
	-1   0    0    -1  
$EndComp
$EndSCHEMATC
