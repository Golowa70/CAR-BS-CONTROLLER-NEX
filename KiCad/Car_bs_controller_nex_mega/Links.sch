EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 5 8
Title "Car bs controller nex mega v3"
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text GLabel 1250 1500 0    50   Input ~ 0
ONE_WARE_INPUT
Text GLabel 4350 1500 2    50   Output ~ 0
MCU_one_wire_PIN
$Comp
L Car_bs_controller_nex_mega-rescue:BAV99-Diode D28
U 1 1 600F1997
P 2800 1500
F 0 "D28" V 2846 1579 50  0000 L CNN
F 1 "BAV99" V 2755 1579 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 2800 1000 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/BAV99_SER.pdf" H 2800 1500 50  0001 C CNN
	1    2800 1500
	0    -1   -1   0   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR039
U 1 1 600F32F4
P 2800 1950
F 0 "#PWR039" H 2800 1700 50  0001 C CNN
F 1 "GND" H 2805 1777 50  0000 C CNN
F 2 "" H 2800 1950 50  0001 C CNN
F 3 "" H 2800 1950 50  0001 C CNN
	1    2800 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 1950 2800 1800
Text GLabel 1400 1000 0    50   Input ~ 0
+5v_FROM_MCU
Wire Wire Line
	2800 1200 2800 1000
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R23
U 1 1 600F49C9
P 3200 1250
F 0 "R23" H 3270 1296 50  0000 L CNN
F 1 "4k7" H 3270 1205 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 3130 1250 50  0001 C CNN
F 3 "~" H 3200 1250 50  0001 C CNN
	1    3200 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 1400 3200 1500
Connection ~ 3200 1500
Wire Wire Line
	3200 1500 3000 1500
Wire Wire Line
	3200 1100 3200 1000
Wire Wire Line
	3200 1000 2800 1000
Connection ~ 2800 1000
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R20
U 1 1 600783FE
P 1650 3050
F 0 "R20" V 1443 3050 50  0000 C CNN
F 1 "180R" V 1534 3050 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1580 3050 50  0001 C CNN
F 3 "~" H 1650 3050 50  0001 C CNN
	1    1650 3050
	0    1    1    0   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:BAV99-Diode D29
U 1 1 6007840C
P 3200 3050
F 0 "D29" V 3246 3129 50  0000 L CNN
F 1 "BAV99" V 3155 3129 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 3200 2550 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/BAV99_SER.pdf" H 3200 3050 50  0001 C CNN
	1    3200 3050
	0    -1   -1   0   
$EndComp
Text GLabel 1250 2550 0    50   Input ~ 0
+5v_FROM_MCU
Wire Wire Line
	3200 2750 3200 2550
Wire Wire Line
	3400 3050 3650 3050
Text GLabel 4250 3050 2    50   Output ~ 0
MCU_pjon_bus_PIN
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R18
U 1 1 60079F5A
P 1300 3300
F 0 "R18" H 1370 3346 50  0000 L CNN
F 1 "1M" H 1370 3255 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1230 3300 50  0001 C CNN
F 3 "~" H 1300 3300 50  0001 C CNN
	1    1300 3300
	1    0    0    -1  
$EndComp
Text GLabel 1050 3050 0    50   Input ~ 0
PJON_BUS
$Comp
L Car_bs_controller_nex_mega-rescue:MAX485E-Interface_UART U3
U 1 1 60085366
P 7900 1400
F 0 "U3" H 7900 2081 50  0000 C CNN
F 1 "MAX485E" H 7900 1990 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 7900 700 50  0001 C CNN
F 3 "https://datasheets.maximintegrated.com/en/ds/MAX1487E-MAX491E.pdf" H 7900 1450 50  0001 C CNN
	1    7900 1400
	1    0    0    -1  
$EndComp
Text GLabel 10400 1600 2    50   Output ~ 0
RS485_A
Text GLabel 10400 1300 2    50   Output ~ 0
RS485_B
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR045
U 1 1 600873C6
P 7900 2200
F 0 "#PWR045" H 7900 1950 50  0001 C CNN
F 1 "GND" H 7905 2027 50  0000 C CNN
F 2 "" H 7900 2200 50  0001 C CNN
F 3 "" H 7900 2200 50  0001 C CNN
	1    7900 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 2200 7900 2000
Text GLabel 6150 1300 0    50   Output ~ 0
MCU_UART2_RX
Wire Wire Line
	7500 1300 6850 1300
Text GLabel 6150 1600 0    50   Input ~ 0
MCU_UART2_TX
Wire Wire Line
	7500 1600 7250 1600
Wire Wire Line
	6200 800  6400 800 
Wire Wire Line
	7900 800  7900 900 
Wire Wire Line
	7500 1500 7500 1450
Text GLabel 6150 1450 0    50   Input ~ 0
MCU_rs485_de
Wire Wire Line
	6150 1450 6400 1450
Connection ~ 7500 1450
Wire Wire Line
	7500 1450 7500 1400
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R33
U 1 1 6008AE6E
P 6850 1000
F 0 "R33" H 6920 1046 50  0000 L CNN
F 1 "10k" H 6920 955 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 6780 1000 50  0001 C CNN
F 3 "~" H 6850 1000 50  0001 C CNN
	1    6850 1000
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R35
U 1 1 6008B56D
P 7250 1000
F 0 "R35" H 7320 1046 50  0000 L CNN
F 1 "10k" H 7320 955 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 7180 1000 50  0001 C CNN
F 3 "~" H 7250 1000 50  0001 C CNN
	1    7250 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	7250 850  7250 800 
Connection ~ 7250 800 
Wire Wire Line
	6850 850  6850 800 
Connection ~ 6850 800 
Wire Wire Line
	6850 800  7250 800 
Wire Wire Line
	6850 1150 6850 1300
Connection ~ 6850 1300
Wire Wire Line
	6850 1300 6150 1300
Wire Wire Line
	7250 1150 7250 1600
Connection ~ 7250 1600
Wire Wire Line
	7250 1600 6150 1600
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R34
U 1 1 6008D90C
P 6850 2250
F 0 "R34" H 6920 2296 50  0000 L CNN
F 1 "1k" H 6920 2205 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6780 2250 50  0001 C CNN
F 3 "~" H 6850 2250 50  0001 C CNN
	1    6850 2250
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R36
U 1 1 6008DD35
P 7250 2250
F 0 "R36" H 7320 2296 50  0000 L CNN
F 1 "1k" H 7320 2205 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7180 2250 50  0001 C CNN
F 3 "~" H 7250 2250 50  0001 C CNN
	1    7250 2250
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:LED-Device D33
U 1 1 60091218
P 6850 2800
F 0 "D33" V 6797 2880 50  0000 L CNN
F 1 "LED" V 6888 2880 50  0000 L CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 6850 2800 50  0001 C CNN
F 3 "~" H 6850 2800 50  0001 C CNN
	1    6850 2800
	0    1    1    0   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:LED-Device D34
U 1 1 60092102
P 7250 2800
F 0 "D34" V 7197 2880 50  0000 L CNN
F 1 "LED" V 7288 2880 50  0000 L CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 7250 2800 50  0001 C CNN
F 3 "~" H 7250 2800 50  0001 C CNN
	1    7250 2800
	0    1    1    0   
$EndComp
Wire Wire Line
	6850 2100 6850 1300
Wire Wire Line
	6850 2650 6850 2400
Wire Wire Line
	7250 2650 7250 2400
Wire Wire Line
	7250 1600 7250 2100
Wire Wire Line
	7250 2950 6850 2950
Wire Wire Line
	6850 2950 6650 2950
Wire Wire Line
	6650 2950 6650 800 
Connection ~ 6850 2950
Connection ~ 6650 800 
Wire Wire Line
	6650 800  6850 800 
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R31
U 1 1 60098ED3
P 6400 1000
F 0 "R31" H 6470 1046 50  0000 L CNN
F 1 "10k" H 6470 955 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 6330 1000 50  0001 C CNN
F 3 "~" H 6400 1000 50  0001 C CNN
	1    6400 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 850  6400 800 
Connection ~ 6400 800 
Wire Wire Line
	6400 800  6650 800 
Wire Wire Line
	6400 1150 6400 1450
Connection ~ 6400 1450
Wire Wire Line
	6400 1450 7500 1450
$Comp
L Car_bs_controller_nex_mega-rescue:C-Device C24
U 1 1 6009D569
P 8400 1000
F 0 "C24" H 8515 1046 50  0000 L CNN
F 1 "10m" H 8450 900 50  0000 L CNN
F 2 "Capacitor_SMD:C_1210_3225Metric_Pad1.33x2.70mm_HandSolder" H 8438 850 50  0001 C CNN
F 3 "~" H 8400 1000 50  0001 C CNN
	1    8400 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	8400 850  8400 800 
Wire Wire Line
	8400 800  7900 800 
Connection ~ 7900 800 
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR046
U 1 1 6009F1A1
P 8400 2200
F 0 "#PWR046" H 8400 1950 50  0001 C CNN
F 1 "GND" H 8405 2027 50  0000 C CNN
F 2 "" H 8400 2200 50  0001 C CNN
F 3 "" H 8400 2200 50  0001 C CNN
	1    8400 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	8400 1150 8400 2200
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R43
U 1 1 600A0578
P 9350 1300
F 0 "R43" V 9150 1400 50  0000 C CNN
F 1 "10R" V 9234 1300 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 9280 1300 50  0001 C CNN
F 3 "~" H 9350 1300 50  0001 C CNN
	1    9350 1300
	0    1    1    0   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R44
U 1 1 600A0E5E
P 9350 1600
F 0 "R44" V 9143 1600 50  0000 C CNN
F 1 "10R" V 9234 1600 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 9280 1600 50  0001 C CNN
F 3 "~" H 9350 1600 50  0001 C CNN
	1    9350 1600
	0    1    1    0   
$EndComp
Wire Wire Line
	9200 1300 9000 1300
Connection ~ 8400 800 
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR048
U 1 1 600AAF66
P 9000 2200
F 0 "#PWR048" H 9000 1950 50  0001 C CNN
F 1 "GND" H 9005 2027 50  0000 C CNN
F 2 "" H 9000 2200 50  0001 C CNN
F 3 "" H 9000 2200 50  0001 C CNN
	1    9000 2200
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R39
U 1 1 600ABB72
P 9000 1900
F 0 "R39" H 9070 1946 50  0000 L CNN
F 1 "20k" H 9070 1855 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 8930 1900 50  0001 C CNN
F 3 "~" H 9000 1900 50  0001 C CNN
	1    9000 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 1750 9000 1300
Connection ~ 9000 1300
Wire Wire Line
	9000 1300 8300 1300
Wire Wire Line
	9000 2050 9000 2200
$Comp
L Car_bs_controller_nex_mega-rescue:D_TVS-Device D35
U 1 1 600B0557
P 9650 1900
F 0 "D35" V 9600 1700 50  0000 L CNN
F 1 "SMAJ5.0CA" V 9800 1450 50  0000 L CNN
F 2 "Diode_SMD:D_SMA" H 9650 1900 50  0001 C CNN
F 3 "~" H 9650 1900 50  0001 C CNN
	1    9650 1900
	0    1    1    0   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:D_TVS-Device D36
U 1 1 600B2D51
P 10100 1900
F 0 "D36" V 10054 1980 50  0000 L CNN
F 1 "SMAJ5.0CA" V 10145 1980 50  0000 L CNN
F 2 "Diode_SMD:D_SMA" H 10100 1900 50  0001 C CNN
F 3 "~" H 10100 1900 50  0001 C CNN
	1    10100 1900
	0    1    1    0   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR052
U 1 1 600B3535
P 9650 2200
F 0 "#PWR052" H 9650 1950 50  0001 C CNN
F 1 "GND" H 9655 2027 50  0000 C CNN
F 2 "" H 9650 2200 50  0001 C CNN
F 3 "" H 9650 2200 50  0001 C CNN
	1    9650 2200
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR053
U 1 1 600B3AFF
P 10100 2200
F 0 "#PWR053" H 10100 1950 50  0001 C CNN
F 1 "GND" H 10105 2027 50  0000 C CNN
F 2 "" H 10100 2200 50  0001 C CNN
F 3 "" H 10100 2200 50  0001 C CNN
	1    10100 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	10400 1600 10100 1600
Wire Wire Line
	9650 1750 9650 1300
Connection ~ 9650 1300
Wire Wire Line
	9650 1300 9500 1300
Wire Wire Line
	10100 1750 10100 1600
Connection ~ 10100 1600
Wire Wire Line
	10100 1600 9500 1600
Wire Wire Line
	10100 2200 10100 2050
Wire Wire Line
	9650 2200 9650 2050
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R42
U 1 1 600A2E3C
P 9100 1050
F 0 "R42" H 9170 1096 50  0000 L CNN
F 1 "20k" H 9170 1005 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 9030 1050 50  0001 C CNN
F 3 "~" H 9100 1050 50  0001 C CNN
	1    9100 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	8300 1600 9100 1600
Wire Wire Line
	9100 1200 9100 1600
Connection ~ 9100 1600
Wire Wire Line
	9100 1600 9200 1600
Wire Wire Line
	9100 900  9100 800 
Wire Wire Line
	8400 800  8750 800 
$Comp
L Car_bs_controller_nex_mega-rescue:C-Device C25
U 1 1 600C0C4A
P 8750 1000
F 0 "C25" H 8865 1046 50  0000 L CNN
F 1 "100n" H 8850 900 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 8788 850 50  0001 C CNN
F 3 "~" H 8750 1000 50  0001 C CNN
	1    8750 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	8750 850  8750 800 
Connection ~ 8750 800 
Wire Wire Line
	8750 800  9100 800 
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR047
U 1 1 600C2E2A
P 8750 2200
F 0 "#PWR047" H 8750 1950 50  0001 C CNN
F 1 "GND" H 8755 2027 50  0000 C CNN
F 2 "" H 8750 2200 50  0001 C CNN
F 3 "" H 8750 2200 50  0001 C CNN
	1    8750 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	8750 2200 8750 1150
$Comp
L Car_bs_controller_nex_mega-rescue:LED-Device D31
U 1 1 600D3874
P 3650 3600
F 0 "D31" V 3689 3680 50  0000 L CNN
F 1 "LED_PJ" V 3598 3680 50  0000 L CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 3650 3600 50  0001 C CNN
F 3 "~" H 3650 3600 50  0001 C CNN
	1    3650 3600
	0    1    -1   0   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R25
U 1 1 600DC45C
P 3650 3250
F 0 "R25" H 3720 3296 50  0000 L CNN
F 1 "2k2" H 3720 3205 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3580 3250 50  0001 C CNN
F 3 "~" H 3650 3250 50  0001 C CNN
	1    3650 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 3100 3650 3050
Connection ~ 3650 3050
Wire Wire Line
	3650 3450 3650 3400
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR036
U 1 1 6007A5DB
P 1300 3500
F 0 "#PWR036" H 1300 3250 50  0001 C CNN
F 1 "GND" H 1305 3327 50  0000 C CNN
F 2 "" H 1300 3500 50  0001 C CNN
F 3 "" H 1300 3500 50  0001 C CNN
	1    1300 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 3500 3200 3350
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR041
U 1 1 60078414
P 3200 3500
F 0 "#PWR041" H 3200 3250 50  0001 C CNN
F 1 "GND" H 3205 3327 50  0000 C CNN
F 2 "" H 3200 3500 50  0001 C CNN
F 3 "" H 3200 3500 50  0001 C CNN
	1    3200 3500
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR043
U 1 1 600E714B
P 3650 3850
F 0 "#PWR043" H 3650 3600 50  0001 C CNN
F 1 "GND" H 3655 3677 50  0000 C CNN
F 2 "" H 3650 3850 50  0001 C CNN
F 3 "" H 3650 3850 50  0001 C CNN
	1    3650 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 3850 3650 3750
$Comp
L Car_bs_controller_nex_mega-rescue:LED-Device D30
U 1 1 600E9EB0
P 3550 850
F 0 "D30" V 3589 930 50  0000 L CNN
F 1 "LED_1WIRE" V 3498 930 50  0000 L CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 3550 850 50  0001 C CNN
F 3 "~" H 3550 850 50  0001 C CNN
	1    3550 850 
	0    1    -1   0   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R24
U 1 1 600E9EB6
P 3550 1250
F 0 "R24" H 3620 1296 50  0000 L CNN
F 1 "2k2" H 3620 1205 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3480 1250 50  0001 C CNN
F 3 "~" H 3550 1250 50  0001 C CNN
	1    3550 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 1400 3550 1500
Connection ~ 3550 1500
Wire Wire Line
	3550 1500 3200 1500
Wire Wire Line
	3550 700  2800 700 
Wire Wire Line
	2800 700  2800 1000
Wire Wire Line
	3550 1000 3550 1100
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R41
U 1 1 60109215
P 9050 3450
F 0 "R41" V 8843 3450 50  0000 C CNN
F 1 "22R" V 8934 3450 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 8980 3450 50  0001 C CNN
F 3 "~" H 9050 3450 50  0001 C CNN
	1    9050 3450
	0    1    1    0   
$EndComp
Wire Wire Line
	9600 3450 9200 3450
Text GLabel 8450 3450 0    50   Input ~ 0
+5v_FROM_MCU
Wire Wire Line
	8900 3450 8450 3450
Wire Wire Line
	9300 3650 9600 3650
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR050
U 1 1 60115530
P 9300 4400
F 0 "#PWR050" H 9300 4150 50  0001 C CNN
F 1 "GND" H 9305 4227 50  0000 C CNN
F 2 "" H 9300 4400 50  0001 C CNN
F 3 "" H 9300 4400 50  0001 C CNN
	1    9300 4400
	1    0    0    -1  
$EndComp
Text GLabel 8450 3900 0    50   Input ~ 0
MCU_buzzer_out
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR049
U 1 1 601284F7
P 9000 4400
F 0 "#PWR049" H 9000 4150 50  0001 C CNN
F 1 "GND" H 9005 4227 50  0000 C CNN
F 2 "" H 9000 4400 50  0001 C CNN
F 3 "" H 9000 4400 50  0001 C CNN
	1    9000 4400
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R37
U 1 1 60128AD0
P 8700 3900
F 0 "R37" V 8493 3900 50  0000 C CNN
F 1 "1k" V 8584 3900 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 8630 3900 50  0001 C CNN
F 3 "~" H 8700 3900 50  0001 C CNN
	1    8700 3900
	0    1    1    0   
$EndComp
Wire Wire Line
	8550 3900 8450 3900
Wire Wire Line
	9300 3700 9300 3650
Wire Wire Line
	9300 4400 9300 4100
Wire Wire Line
	9000 3900 8850 3900
$Comp
L Car_bs_controller_nex_mega-rescue:BSS138-Transistor_FET Q1
U 1 1 60110C87
P 9200 3900
F 0 "Q1" H 9404 3946 50  0000 L CNN
F 1 "BSS138" H 9404 3855 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 9400 3825 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/BSS138-D.PDF" H 9200 3900 50  0001 L CNN
	1    9200 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 4300 9000 4400
Wire Wire Line
	9000 4000 9000 3900
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R40
U 1 1 60124D5C
P 9000 4150
F 0 "R40" H 9070 4196 50  0000 L CNN
F 1 "10k" H 9070 4105 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 8930 4150 50  0001 C CNN
F 3 "~" H 9000 4150 50  0001 C CNN
	1    9000 4150
	1    0    0    -1  
$EndComp
Connection ~ 9000 3900
Wire Wire Line
	9600 3750 9600 3650
Wire Wire Line
	9600 3350 9600 3450
$Comp
L Car_bs_controller_nex_mega-rescue:ADM705-ADM705 U?
U 1 1 6017DAE3
P 3950 5550
AR Path="/60100EBE/6017DAE3" Ref="U?"  Part="1" 
AR Path="/5FF96CA3/6017DAE3" Ref="U2"  Part="1" 
F 0 "U2" H 3950 5665 50  0000 C CNN
F 1 "ADM705" H 3950 5574 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 3950 5550 50  0001 C CNN
F 3 "" H 3950 5550 50  0001 C CNN
	1    3950 5550
	1    0    0    -1  
$EndComp
Text GLabel 1400 5650 0    50   Input ~ 0
+5v_FROM_MCU
$Comp
L Car_bs_controller_nex_mega-rescue:C-Device C?
U 1 1 6017DAF0
P 2050 5950
AR Path="/5FF96C00/6017DAF0" Ref="C?"  Part="1" 
AR Path="/60100EBE/6017DAF0" Ref="C?"  Part="1" 
AR Path="/5FF96CA3/6017DAF0" Ref="C22"  Part="1" 
F 0 "C22" H 2165 5996 50  0000 L CNN
F 1 "100n" H 2165 5905 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 2088 5800 50  0001 C CNN
F 3 "~" H 2050 5950 50  0001 C CNN
	1    2050 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 5800 2050 5650
Wire Wire Line
	2050 6100 2050 6200
Connection ~ 2050 5650
Wire Wire Line
	3400 5950 2950 5950
Wire Wire Line
	2950 5950 2950 5650
Wire Wire Line
	3400 6150 3000 6150
Wire Wire Line
	3000 6150 3000 6200
Wire Wire Line
	3000 6200 2550 6200
Connection ~ 2550 6200
Wire Wire Line
	2550 6200 2050 6200
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR?
U 1 1 6017DB06
P 2550 6350
AR Path="/60100EBE/6017DB06" Ref="#PWR?"  Part="1" 
AR Path="/5FF96CA3/6017DB06" Ref="#PWR038"  Part="1" 
F 0 "#PWR038" H 2550 6100 50  0001 C CNN
F 1 "GND" H 2555 6177 50  0000 C CNN
F 2 "" H 2550 6350 50  0001 C CNN
F 3 "" H 2550 6350 50  0001 C CNN
	1    2550 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 6350 2550 6200
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R?
U 1 1 6017DB0D
P 5050 6150
AR Path="/5FF96B9E/6017DB0D" Ref="R?"  Part="1" 
AR Path="/60100EBE/6017DB0D" Ref="R?"  Part="1" 
AR Path="/5FF96CA3/6017DB0D" Ref="R29"  Part="1" 
F 0 "R29" V 5150 6150 50  0000 C CNN
F 1 "2k2" V 4934 6150 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 4980 6150 50  0001 C CNN
F 3 "~" H 5050 6150 50  0001 C CNN
	1    5050 6150
	0    1    1    0   
$EndComp
Wire Wire Line
	4900 6150 4500 6150
Text GLabel 6100 5950 2    50   Output ~ 0
MCU_rst
Text GLabel 5450 6150 2    50   Input ~ 0
MCU_wdt_reset_to_ADM705
Wire Wire Line
	5450 6150 5200 6150
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R?
U 1 1 6019B120
P 3900 5200
AR Path="/5FF96B9E/6019B120" Ref="R?"  Part="1" 
AR Path="/60100EBE/6019B120" Ref="R?"  Part="1" 
AR Path="/5FF96CA3/6019B120" Ref="R26"  Part="1" 
F 0 "R26" V 4000 5200 50  0000 C CNN
F 1 "200R" V 3784 5200 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 3830 5200 50  0001 C CNN
F 3 "~" H 3900 5200 50  0001 C CNN
	1    3900 5200
	0    1    1    0   
$EndComp
Wire Wire Line
	3750 5200 3400 5200
Wire Wire Line
	3400 5200 3400 5750
Wire Wire Line
	4500 5750 4500 5200
Wire Wire Line
	4500 5200 4050 5200
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R22
U 1 1 601AD937
P 3100 7300
F 0 "R22" H 3170 7346 50  0000 L CNN
F 1 "2k2" H 3170 7255 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 3030 7300 50  0001 C CNN
F 3 "~" H 3100 7300 50  0001 C CNN
	1    3100 7300
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR?
U 1 1 601B23EA
P 3100 7450
AR Path="/60100EBE/601B23EA" Ref="#PWR?"  Part="1" 
AR Path="/5FF96CA3/601B23EA" Ref="#PWR040"  Part="1" 
F 0 "#PWR040" H 3100 7200 50  0001 C CNN
F 1 "GND" H 3105 7277 50  0000 C CNN
F 2 "" H 3100 7450 50  0001 C CNN
F 3 "" H 3100 7450 50  0001 C CNN
	1    3100 7450
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R?
U 1 1 60085833
P 1950 6650
AR Path="/5FF96B9E/60085833" Ref="R?"  Part="1" 
AR Path="/60100EBE/60085833" Ref="R?"  Part="1" 
AR Path="/5FF96CA3/60085833" Ref="R21"  Part="1" 
F 0 "R21" V 2050 6650 50  0000 C CNN
F 1 "22k" V 1834 6650 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 1880 6650 50  0001 C CNN
F 3 "~" H 1950 6650 50  0001 C CNN
	1    1950 6650
	0    1    1    0   
$EndComp
Wire Wire Line
	2100 6650 3100 6650
Connection ~ 3100 6650
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R?
U 1 1 600A19E2
P 5050 6550
AR Path="/5FF96B9E/600A19E2" Ref="R?"  Part="1" 
AR Path="/60100EBE/600A19E2" Ref="R?"  Part="1" 
AR Path="/5FF96CA3/600A19E2" Ref="R30"  Part="1" 
F 0 "R30" V 5150 6550 50  0000 C CNN
F 1 "1k" V 4934 6550 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 4980 6550 50  0001 C CNN
F 3 "~" H 5050 6550 50  0001 C CNN
	1    5050 6550
	0    1    1    0   
$EndComp
Wire Wire Line
	4900 6550 4750 6550
Wire Wire Line
	4750 6550 4750 6350
Wire Wire Line
	4500 6350 4600 6350
Text GLabel 5450 6550 2    50   Output ~ 0
MCU_power_OK_from_ADM705
Wire Wire Line
	5450 6550 5200 6550
$Comp
L Car_bs_controller_nex_mega-rescue:R_POT-Device RV1
U 1 1 600CE8E4
P 3100 7000
F 0 "RV1" H 3030 7046 50  0000 R CNN
F 1 "10k" H 3030 6955 50  0000 R CNN
F 2 "Potentiometer_SMD:Potentiometer_Bourns_3314J_Vertical" H 3100 7000 50  0001 C CNN
F 3 "~" H 3100 7000 50  0001 C CNN
	1    3100 7000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 6650 3100 6850
Wire Wire Line
	3250 7000 3250 6650
Wire Wire Line
	3250 6650 3100 6650
Wire Wire Line
	3100 6650 3100 6350
Wire Wire Line
	3100 6350 3350 6350
$Comp
L Car_bs_controller_nex_mega-rescue:SW_Push-Switch SW1
U 1 1 6019F31B
P 9350 5400
F 0 "SW1" H 9350 5685 50  0000 C CNN
F 1 "SW_Push" H 9350 5594 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_Push_SPST_NO_Alps_SKRK" H 9350 5600 50  0001 C CNN
F 3 "~" H 9350 5600 50  0001 C CNN
	1    9350 5400
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR051
U 1 1 6019FDAB
P 9600 5650
F 0 "#PWR051" H 9600 5400 50  0001 C CNN
F 1 "GND" H 9605 5477 50  0000 C CNN
F 2 "" H 9600 5650 50  0001 C CNN
F 3 "" H 9600 5650 50  0001 C CNN
	1    9600 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	9550 5400 9600 5400
Wire Wire Line
	9600 5400 9600 5650
Text GLabel 8350 5400 0    50   Input ~ 0
MCU_button_on_board
$Comp
L Car_bs_controller_nex_mega-rescue:LED-Device D32
U 1 1 601BAB6B
P 6750 4350
F 0 "D32" V 6789 4430 50  0000 L CNN
F 1 "LED_PJ" V 6698 4430 50  0000 L CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 6750 4350 50  0001 C CNN
F 3 "~" H 6750 4350 50  0001 C CNN
	1    6750 4350
	0    1    -1   0   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R32
U 1 1 601BAB71
P 6750 4000
F 0 "R32" H 6820 4046 50  0000 L CNN
F 1 "2k2" H 6820 3955 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6680 4000 50  0001 C CNN
F 3 "~" H 6750 4000 50  0001 C CNN
	1    6750 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 3850 6750 3800
Wire Wire Line
	6750 4200 6750 4150
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR044
U 1 1 601BAB79
P 6750 4600
F 0 "#PWR044" H 6750 4350 50  0001 C CNN
F 1 "GND" H 6755 4427 50  0000 C CNN
F 2 "" H 6750 4600 50  0001 C CNN
F 3 "" H 6750 4600 50  0001 C CNN
	1    6750 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 4600 6750 4500
Text GLabel 6500 3800 0    50   Input ~ 0
MCU_system_status_LED
Wire Wire Line
	6500 3800 6750 3800
Connection ~ 3000 1500
Connection ~ 3400 3050
$Comp
L Car_bs_controller_nex_mega-rescue:+12V-power #PWR037
U 1 1 600B23CB
P 1350 6500
F 0 "#PWR037" H 1350 6350 50  0001 C CNN
F 1 "+12V" H 1365 6673 50  0000 C CNN
F 2 "" H 1350 6500 50  0001 C CNN
F 3 "" H 1350 6500 50  0001 C CNN
	1    1350 6500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1350 6650 1350 6500
Wire Wire Line
	1350 6650 1800 6650
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R19
U 1 1 600EEF92
P 1500 1500
F 0 "R19" V 1293 1500 50  0000 C CNN
F 1 "22R" V 1384 1500 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1430 1500 50  0001 C CNN
F 3 "~" H 1500 1500 50  0001 C CNN
	1    1500 1500
	0    1    1    0   
$EndComp
$Comp
L Device:L L2
U 1 1 6084ED75
P 2000 1500
F 0 "L2" V 2190 1500 50  0000 C CNN
F 1 "BLN" V 2099 1500 50  0000 C CNN
F 2 "Inductor_SMD:L_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2000 1500 50  0001 C CNN
F 3 "~" H 2000 1500 50  0001 C CNN
	1    2000 1500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2150 1500 3000 1500
Wire Wire Line
	1650 1500 1850 1500
Wire Wire Line
	1350 1500 1250 1500
$Comp
L Device:L L3
U 1 1 60861E30
P 2150 3050
F 0 "L3" V 2340 3050 50  0000 C CNN
F 1 "BLN" V 2249 3050 50  0000 C CNN
F 2 "Inductor_SMD:L_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2150 3050 50  0001 C CNN
F 3 "~" H 2150 3050 50  0001 C CNN
	1    2150 3050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1400 1000 2800 1000
Wire Wire Line
	3550 1500 4350 1500
Wire Wire Line
	3650 3050 4250 3050
Wire Wire Line
	1300 3500 1300 3450
Wire Wire Line
	1050 3050 1300 3050
Wire Wire Line
	1300 3150 1300 3050
Wire Wire Line
	1500 3050 1300 3050
Connection ~ 1300 3050
Wire Wire Line
	1800 3050 2000 3050
Wire Wire Line
	2300 3050 3400 3050
Wire Wire Line
	1250 2550 3200 2550
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R?
U 1 1 608BD806
P 3950 6800
AR Path="/5FF96B9E/608BD806" Ref="R?"  Part="1" 
AR Path="/60100EBE/608BD806" Ref="R?"  Part="1" 
AR Path="/5FF96CA3/608BD806" Ref="R27"  Part="1" 
F 0 "R27" V 4050 6800 50  0000 C CNN
F 1 "(not used) 1k" V 3834 6800 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 3880 6800 50  0001 C CNN
F 3 "~" H 3950 6800 50  0001 C CNN
	1    3950 6800
	0    1    1    0   
$EndComp
Wire Wire Line
	4600 6800 4600 6350
Wire Wire Line
	4100 6800 4600 6800
Connection ~ 4600 6350
Wire Wire Line
	4600 6350 4750 6350
Wire Wire Line
	3800 6800 3600 6800
Wire Wire Line
	3350 6800 3350 6350
Connection ~ 3350 6350
Wire Wire Line
	3350 6350 3400 6350
$Comp
L Car_bs_controller_nex_mega-rescue:C-Device C?
U 1 1 608D047B
P 3600 7250
AR Path="/5FF96C00/608D047B" Ref="C?"  Part="1" 
AR Path="/60100EBE/608D047B" Ref="C?"  Part="1" 
AR Path="/5FF96CA3/608D047B" Ref="C23"  Part="1" 
F 0 "C23" H 3715 7296 50  0000 L CNN
F 1 "1n" H 3715 7205 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 3638 7100 50  0001 C CNN
F 3 "~" H 3600 7250 50  0001 C CNN
	1    3600 7250
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR?
U 1 1 608D12EF
P 3600 7450
AR Path="/60100EBE/608D12EF" Ref="#PWR?"  Part="1" 
AR Path="/5FF96CA3/608D12EF" Ref="#PWR042"  Part="1" 
F 0 "#PWR042" H 3600 7200 50  0001 C CNN
F 1 "GND" H 3605 7277 50  0000 C CNN
F 2 "" H 3600 7450 50  0001 C CNN
F 3 "" H 3600 7450 50  0001 C CNN
	1    3600 7450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 7450 3600 7400
Wire Wire Line
	3600 7100 3600 6800
Connection ~ 3600 6800
Wire Wire Line
	3600 6800 3350 6800
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R?
U 1 1 608DEDBC
P 4800 5950
AR Path="/5FF96B9E/608DEDBC" Ref="R?"  Part="1" 
AR Path="/60100EBE/608DEDBC" Ref="R?"  Part="1" 
AR Path="/5FF96CA3/608DEDBC" Ref="R28"  Part="1" 
F 0 "R28" V 4900 5950 50  0000 C CNN
F 1 "220R" V 4684 5950 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 4730 5950 50  0001 C CNN
F 3 "~" H 4800 5950 50  0001 C CNN
	1    4800 5950
	0    1    1    0   
$EndComp
Wire Wire Line
	4650 5950 4500 5950
$Comp
L Jumper:SolderJumper_2_Open JP1
U 1 1 608E7439
P 5500 5950
F 0 "JP1" H 5500 6155 50  0000 C CNN
F 1 "SolderJumper_2_Open" H 5650 6100 50  0000 C CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_RoundedPad1.0x1.5mm" H 5500 5950 50  0001 C CNN
F 3 "~" H 5500 5950 50  0001 C CNN
	1    5500 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 5950 5350 5950
Wire Wire Line
	5650 5950 6100 5950
Text GLabel 6200 800  0    50   Input ~ 0
+5v_FROM_MCU
Wire Wire Line
	1400 5650 1600 5650
$Comp
L Car_bs_controller_nex_mega-rescue:C-Device C?
U 1 1 6091A653
P 1600 5950
AR Path="/5FF96C00/6091A653" Ref="C?"  Part="1" 
AR Path="/60100EBE/6091A653" Ref="C?"  Part="1" 
AR Path="/5FF96CA3/6091A653" Ref="C21"  Part="1" 
F 0 "C21" H 1715 5996 50  0000 L CNN
F 1 "10mk" H 1715 5905 50  0000 L CNN
F 2 "Capacitor_Tantalum_SMD:CP_EIA-7343-15_Kemet-W_Pad2.25x2.55mm_HandSolder" H 1638 5800 50  0001 C CNN
F 3 "~" H 1600 5950 50  0001 C CNN
	1    1600 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 6100 1600 6200
Wire Wire Line
	1600 6200 2050 6200
Connection ~ 2050 6200
Wire Wire Line
	1600 5800 1600 5650
Connection ~ 1600 5650
Wire Wire Line
	1600 5650 2050 5650
Wire Wire Line
	9600 3750 9900 3750
Wire Wire Line
	9600 3350 9900 3350
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R45
U 1 1 60938CF2
P 9900 3550
F 0 "R45" H 9970 3596 50  0000 L CNN
F 1 "1k" H 9970 3505 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 9830 3550 50  0001 C CNN
F 3 "~" H 9900 3550 50  0001 C CNN
	1    9900 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	9900 3700 9900 3750
Connection ~ 9900 3750
Wire Wire Line
	9900 3750 10250 3750
Wire Wire Line
	9900 3400 9900 3350
Connection ~ 9900 3350
Wire Wire Line
	9900 3350 10250 3350
$Comp
L Car_bs_controller_nex_mega-rescue:PKLCS1212E4001-R1-PKLCS1212E4001-R1 LS1
U 1 1 60150DF8
P 10250 3550
F 0 "LS1" H 10354 3596 50  0000 L CNN
F 1 "PKLCS1212E4001-R1" H 10354 3505 50  0000 L CNN
F 2 "PKLCS1212E4001-R1:SPKR_PKLCS1212E4001-R1" H 10250 3550 50  0001 L BNN
F 3 "" H 10250 3550 50  0001 L BNN
F 4 "Manufacturer Recommendations" H 10250 3550 50  0001 L BNN "STANDARD"
F 5 "3.0 mm" H 10250 3550 50  0001 L BNN "MAXIMUM_PACKAGE_HEIGHT"
F 6 "Murata Electronics" H 10250 3550 50  0001 L BNN "MANUFACTURER"
F 7 "January 22, 2018" H 10250 3550 50  0001 L BNN "PARTREV"
	1    10250 3550
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R38
U 1 1 6094A492
P 8700 5400
F 0 "R38" V 8493 5400 50  0000 C CNN
F 1 "470R" V 8584 5400 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 8630 5400 50  0001 C CNN
F 3 "~" H 8700 5400 50  0001 C CNN
	1    8700 5400
	0    1    1    0   
$EndComp
Wire Wire Line
	8550 5400 8350 5400
Wire Wire Line
	8850 5400 9150 5400
Wire Wire Line
	7250 800  7900 800 
Wire Wire Line
	9650 1300 10400 1300
Wire Wire Line
	2050 5650 2950 5650
Wire Notes Line
	6700 4900 6700 7700
Wire Notes Line
	6700 7700 600  7700
Wire Notes Line
	600  7700 600  4900
Wire Notes Line
	600  4900 6700 4900
Text Notes 1150 7350 0    79   ~ 16
Optional
$EndSCHEMATC
