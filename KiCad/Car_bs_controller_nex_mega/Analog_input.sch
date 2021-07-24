EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 8 8
Title "Car bs controller nex mega v3"
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text GLabel 6050 3600 2    50   Output ~ 0
MCU_resistive_sensor_0_190
$Comp
L Car_bs_controller_nex_mega-rescue:TL431DBZ-Reference_Voltage U?
U 1 1 6010F6C7
P 2850 1300
AR Path="/5FF96C00/6010F6C7" Ref="U?"  Part="1" 
AR Path="/60100EBE/6010F6C7" Ref="U5"  Part="1" 
F 0 "U5" V 2896 1230 50  0000 R CNN
F 1 "TL431DBZ" V 2805 1230 50  0000 R CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 2850 1150 50  0001 C CIN
F 3 "http://www.ti.com/lit/ds/symlink/tl431.pdf" H 2850 1300 50  0001 C CIN
	1    2850 1300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2850 1200 2850 1150
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R?
U 1 1 6010F6CE
P 2500 1050
AR Path="/5FF96C00/6010F6CE" Ref="R?"  Part="1" 
AR Path="/60100EBE/6010F6CE" Ref="R58"  Part="1" 
F 0 "R58" V 2293 1050 50  0000 C CNN
F 1 "330R" V 2384 1050 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2430 1050 50  0001 C CNN
F 3 "~" H 2500 1050 50  0001 C CNN
	1    2500 1050
	0    1    1    0   
$EndComp
Connection ~ 2850 1050
Wire Wire Line
	2750 1300 2750 1150
Wire Wire Line
	2750 1150 2850 1150
Connection ~ 2850 1150
Wire Wire Line
	2850 1150 2850 1050
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR?
U 1 1 6010F6DA
P 2850 1500
AR Path="/5FF96C00/6010F6DA" Ref="#PWR?"  Part="1" 
AR Path="/60100EBE/6010F6DA" Ref="#PWR061"  Part="1" 
F 0 "#PWR061" H 2850 1250 50  0001 C CNN
F 1 "GND" H 2855 1327 50  0000 C CNN
F 2 "" H 2850 1500 50  0001 C CNN
F 3 "" H 2850 1500 50  0001 C CNN
	1    2850 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 1500 2850 1400
$Comp
L Car_bs_controller_nex_mega-rescue:C-Device C?
U 1 1 6010F6E1
P 3400 1300
AR Path="/5FF96C00/6010F6E1" Ref="C?"  Part="1" 
AR Path="/60100EBE/6010F6E1" Ref="C31"  Part="1" 
F 0 "C31" H 3515 1346 50  0000 L CNN
F 1 "100n" H 3515 1255 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 3438 1150 50  0001 C CNN
F 3 "~" H 3400 1300 50  0001 C CNN
	1    3400 1300
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR?
U 1 1 6010F6E7
P 3400 1500
AR Path="/5FF96C00/6010F6E7" Ref="#PWR?"  Part="1" 
AR Path="/60100EBE/6010F6E7" Ref="#PWR063"  Part="1" 
F 0 "#PWR063" H 3400 1250 50  0001 C CNN
F 1 "GND" H 3405 1327 50  0000 C CNN
F 2 "" H 3400 1500 50  0001 C CNN
F 3 "" H 3400 1500 50  0001 C CNN
	1    3400 1500
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:C-Device C?
U 1 1 6010F6EE
P 2200 1300
AR Path="/5FF96C00/6010F6EE" Ref="C?"  Part="1" 
AR Path="/60100EBE/6010F6EE" Ref="C29"  Part="1" 
F 0 "C29" H 2315 1346 50  0000 L CNN
F 1 "100n" H 2315 1255 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 2238 1150 50  0001 C CNN
F 3 "~" H 2200 1300 50  0001 C CNN
	1    2200 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 1150 2200 1050
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR?
U 1 1 6010F6F6
P 2200 1500
AR Path="/5FF96C00/6010F6F6" Ref="#PWR?"  Part="1" 
AR Path="/60100EBE/6010F6F6" Ref="#PWR058"  Part="1" 
F 0 "#PWR058" H 2200 1250 50  0001 C CNN
F 1 "GND" H 2205 1327 50  0000 C CNN
F 2 "" H 2200 1500 50  0001 C CNN
F 3 "" H 2200 1500 50  0001 C CNN
	1    2200 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 1500 2200 1450
Wire Wire Line
	3400 1150 3400 1050
Wire Wire Line
	2650 1050 2850 1050
Text GLabel 1300 1050 0    50   Input ~ 0
+5v_FROM_MCU
Wire Wire Line
	2850 1050 3400 1050
Wire Wire Line
	2350 1050 2200 1050
Connection ~ 2200 1050
Wire Wire Line
	4000 1050 3400 1050
Connection ~ 3400 1050
Text GLabel 4000 1050 2    50   Input ~ 0
MCU_Uref_2.5v
Text GLabel 1300 5300 0    50   Input ~ 0
+5v_FROM_MCU
Wire Wire Line
	2400 5550 2400 5300
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR?
U 1 1 6007B31F
P 2400 6500
AR Path="/5FF96C00/6007B31F" Ref="#PWR?"  Part="1" 
AR Path="/60100EBE/6007B31F" Ref="#PWR059"  Part="1" 
F 0 "#PWR059" H 2400 6250 50  0001 C CNN
F 1 "GND" H 2405 6327 50  0000 C CNN
F 2 "" H 2400 6500 50  0001 C CNN
F 3 "" H 2400 6500 50  0001 C CNN
	1    2400 6500
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R?
U 1 1 6007B2F5
P 1450 5850
AR Path="/5FF96B9E/6007B2F5" Ref="R?"  Part="1" 
AR Path="/60100EBE/6007B2F5" Ref="R54"  Part="1" 
F 0 "R54" V 1243 5850 50  0000 C CNN
F 1 "39k" V 1334 5850 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 1380 5850 50  0001 C CNN
F 3 "~" H 1450 5850 50  0001 C CNN
	1    1450 5850
	0    1    1    0   
$EndComp
Wire Wire Line
	1800 5950 1800 5850
Wire Wire Line
	1600 5850 1800 5850
Connection ~ 1800 5850
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR?
U 1 1 6008F098
P 1800 6500
AR Path="/5FF96C00/6008F098" Ref="#PWR?"  Part="1" 
AR Path="/60100EBE/6008F098" Ref="#PWR057"  Part="1" 
F 0 "#PWR057" H 1800 6250 50  0001 C CNN
F 1 "GND" H 1805 6327 50  0000 C CNN
F 2 "" H 1800 6500 50  0001 C CNN
F 3 "" H 1800 6500 50  0001 C CNN
	1    1800 6500
	1    0    0    -1  
$EndComp
Text GLabel 4600 5950 2    50   Output ~ 0
MCU_supply_voltage_input
Text GLabel 6250 1150 0    50   Input ~ 0
+5v_FROM_MCU
Wire Wire Line
	7550 1400 7550 1150
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR?
U 1 1 600A0120
P 7550 2250
AR Path="/5FF96C00/600A0120" Ref="#PWR?"  Part="1" 
AR Path="/60100EBE/600A0120" Ref="#PWR070"  Part="1" 
F 0 "#PWR070" H 7550 2000 50  0001 C CNN
F 1 "GND" H 7555 2077 50  0000 C CNN
F 2 "" H 7550 2250 50  0001 C CNN
F 3 "" H 7550 2250 50  0001 C CNN
	1    7550 2250
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R?
U 1 1 600A0129
P 6650 1700
AR Path="/5FF96B9E/600A0129" Ref="R?"  Part="1" 
AR Path="/60100EBE/600A0129" Ref="R65"  Part="1" 
F 0 "R65" V 6443 1700 50  0000 C CNN
F 1 "22k" V 6534 1700 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 6580 1700 50  0001 C CNN
F 3 "~" H 6650 1700 50  0001 C CNN
	1    6650 1700
	0    1    1    0   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R?
U 1 1 600A012F
P 6950 1950
AR Path="/5FF96B9E/600A012F" Ref="R?"  Part="1" 
AR Path="/60100EBE/600A012F" Ref="R66"  Part="1" 
F 0 "R66" H 6880 1904 50  0000 R CNN
F 1 "10k" H 6880 1995 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 6880 1950 50  0001 C CNN
F 3 "~" H 6950 1950 50  0001 C CNN
	1    6950 1950
	-1   0    0    1   
$EndComp
Wire Wire Line
	6950 1800 6950 1700
Wire Wire Line
	6800 1700 6950 1700
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR?
U 1 1 600A0139
P 6950 2200
AR Path="/5FF96C00/600A0139" Ref="#PWR?"  Part="1" 
AR Path="/60100EBE/600A0139" Ref="#PWR068"  Part="1" 
F 0 "#PWR068" H 6950 1950 50  0001 C CNN
F 1 "GND" H 6955 2027 50  0000 C CNN
F 2 "" H 6950 2200 50  0001 C CNN
F 3 "" H 6950 2200 50  0001 C CNN
	1    6950 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6950 2200 6950 2100
Wire Wire Line
	6250 1700 6500 1700
Text GLabel 6250 1700 0    50   Input ~ 0
5V_sensors_stab
Text GLabel 9750 1800 2    50   Output ~ 0
MCU_sensors_voltage_test_input
$Comp
L Car_bs_controller_nex_mega-rescue:BAV99-Diode D46
U 1 1 600F6CEA
P 7550 1700
F 0 "D46" V 7596 1779 50  0000 L CNN
F 1 "BAV99" V 7505 1779 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 7550 1200 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/BAV99_SER.pdf" H 7550 1700 50  0001 C CNN
	1    7550 1700
	0    -1   -1   0   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:BAV99-Diode D44
U 1 1 6006A159
P 2400 5850
F 0 "D44" V 2446 5929 50  0000 L CNN
F 1 "BAV99" V 2355 5929 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 2400 5350 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/BAV99_SER.pdf" H 2400 5850 50  0001 C CNN
	1    2400 5850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1800 5850 2600 5850
Wire Wire Line
	3100 5850 2800 5850
Connection ~ 2600 5850
Wire Wire Line
	6950 1700 7750 1700
Connection ~ 6950 1700
Connection ~ 7750 1700
$Comp
L Car_bs_controller_nex_mega-rescue:+12V-power #PWR056
U 1 1 600BDE44
P 750 5750
F 0 "#PWR056" H 750 5600 50  0001 C CNN
F 1 "+12V" H 765 5923 50  0000 C CNN
F 2 "" H 750 5750 50  0001 C CNN
F 3 "" H 750 5750 50  0001 C CNN
	1    750  5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	750  5850 750  5750
Wire Wire Line
	750  5850 1300 5850
$Comp
L Amplifier_Operational:MCP6004 U6
U 1 1 60A45791
P 4500 3600
F 0 "U6" H 4500 3967 50  0000 C CNN
F 1 "MCP6004(LM324)" H 4500 3876 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 4450 3700 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21733j.pdf" H 4550 3800 50  0001 C CNN
	1    4500 3600
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:BAV99-Diode D45
U 1 1 6005FCC4
P 2850 3500
F 0 "D45" V 2896 3579 50  0000 L CNN
F 1 "BAV99" V 2805 3579 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 2850 3000 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/BAV99_SER.pdf" H 2850 3500 50  0001 C CNN
	1    2850 3500
	0    -1   -1   0   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR?
U 1 1 6011AD65
P 2850 4100
AR Path="/5FF96C00/6011AD65" Ref="#PWR?"  Part="1" 
AR Path="/60100EBE/6011AD65" Ref="#PWR062"  Part="1" 
F 0 "#PWR062" H 2850 3850 50  0001 C CNN
F 1 "GND" H 2855 3927 50  0000 C CNN
F 2 "" H 2850 4100 50  0001 C CNN
F 3 "" H 2850 4100 50  0001 C CNN
	1    2850 4100
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R?
U 1 1 60104916
P 2300 3500
AR Path="/5FF96B9E/60104916" Ref="R?"  Part="1" 
AR Path="/60100EBE/60104916" Ref="R57"  Part="1" 
F 0 "R57" V 2093 3500 50  0000 C CNN
F 1 "10k" V 2184 3500 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 2230 3500 50  0001 C CNN
F 3 "~" H 2300 3500 50  0001 C CNN
	1    2300 3500
	0    1    1    0   
$EndComp
Text GLabel 1400 2700 0    50   Input ~ 0
+5v_FROM_MCU
Text GLabel 1750 3500 0    50   Input ~ 0
RESISTIVE_SENSOR_0_190
Wire Wire Line
	1750 3500 1950 3500
Wire Wire Line
	1950 3200 1950 3500
Connection ~ 1950 3500
Wire Wire Line
	1950 3500 2150 3500
Wire Wire Line
	1950 2900 1950 2700
Wire Wire Line
	1950 2700 1400 2700
Wire Wire Line
	1950 2700 2850 2700
Wire Wire Line
	2850 2700 2850 3200
Connection ~ 1950 2700
Wire Wire Line
	2450 3500 3050 3500
Connection ~ 3050 3500
$Comp
L Car_bs_controller_nex_mega-rescue:C-Device C?
U 1 1 60A60B4D
P 3600 3950
AR Path="/5FF96C00/60A60B4D" Ref="C?"  Part="1" 
AR Path="/60100EBE/60A60B4D" Ref="C32"  Part="1" 
F 0 "C32" H 3715 3996 50  0000 L CNN
F 1 "10n" H 3715 3905 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 3638 3800 50  0001 C CNN
F 3 "~" H 3600 3950 50  0001 C CNN
	1    3600 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 3800 3600 3500
Connection ~ 3600 3500
Wire Wire Line
	3600 3500 4200 3500
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR?
U 1 1 60A62ADF
P 3600 4100
AR Path="/5FF96C00/60A62ADF" Ref="#PWR?"  Part="1" 
AR Path="/60100EBE/60A62ADF" Ref="#PWR064"  Part="1" 
F 0 "#PWR064" H 3600 3850 50  0001 C CNN
F 1 "GND" H 3605 3927 50  0000 C CNN
F 2 "" H 3600 4100 50  0001 C CNN
F 3 "" H 3600 4100 50  0001 C CNN
	1    3600 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 3800 2850 4100
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R?
U 1 1 60A64790
P 4450 4050
AR Path="/5FF96B9E/60A64790" Ref="R?"  Part="1" 
AR Path="/60100EBE/60A64790" Ref="R62"  Part="1" 
F 0 "R62" V 4243 4050 50  0000 C CNN
F 1 "10k*" V 4334 4050 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 4380 4050 50  0001 C CNN
F 3 "~" H 4450 4050 50  0001 C CNN
	1    4450 4050
	0    1    1    0   
$EndComp
Wire Wire Line
	4300 4050 4100 4050
Wire Wire Line
	4100 4050 4100 3700
Wire Wire Line
	4100 3700 4200 3700
Wire Wire Line
	4600 4050 4950 4050
Wire Wire Line
	4950 4050 4950 3600
Wire Wire Line
	4950 3600 4800 3600
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R?
U 1 1 60A6B34E
P 4100 4300
AR Path="/5FF96B9E/60A6B34E" Ref="R?"  Part="1" 
AR Path="/60100EBE/60A6B34E" Ref="R60"  Part="1" 
F 0 "R60" V 3893 4300 50  0000 C CNN
F 1 "2k2" V 3984 4300 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 4030 4300 50  0001 C CNN
F 3 "~" H 4100 4300 50  0001 C CNN
	1    4100 4300
	-1   0    0    1   
$EndComp
Wire Wire Line
	4100 4150 4100 4050
Connection ~ 4100 4050
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR?
U 1 1 60A6E04C
P 4100 4550
AR Path="/5FF96C00/60A6E04C" Ref="#PWR?"  Part="1" 
AR Path="/60100EBE/60A6E04C" Ref="#PWR065"  Part="1" 
F 0 "#PWR065" H 4100 4300 50  0001 C CNN
F 1 "GND" H 4105 4377 50  0000 C CNN
F 2 "" H 4100 4550 50  0001 C CNN
F 3 "" H 4100 4550 50  0001 C CNN
	1    4100 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 4550 4100 4450
Wire Wire Line
	3050 3500 3600 3500
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R?
U 1 1 60A77AB5
P 5250 3600
AR Path="/5FF96B9E/60A77AB5" Ref="R?"  Part="1" 
AR Path="/60100EBE/60A77AB5" Ref="R63"  Part="1" 
F 0 "R63" V 5043 3600 50  0000 C CNN
F 1 "6k8" V 5134 3600 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 5180 3600 50  0001 C CNN
F 3 "~" H 5250 3600 50  0001 C CNN
	1    5250 3600
	0    1    1    0   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R?
U 1 1 60A78379
P 5600 3900
AR Path="/5FF96B9E/60A78379" Ref="R?"  Part="1" 
AR Path="/60100EBE/60A78379" Ref="R64"  Part="1" 
F 0 "R64" V 5393 3900 50  0000 C CNN
F 1 "15k*" V 5484 3900 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 5530 3900 50  0001 C CNN
F 3 "~" H 5600 3900 50  0001 C CNN
	1    5600 3900
	-1   0    0    1   
$EndComp
Wire Wire Line
	5100 3600 4950 3600
Connection ~ 4950 3600
Wire Wire Line
	5400 3600 5600 3600
Wire Wire Line
	5600 3600 5600 3750
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR?
U 1 1 60A7BA8D
P 5600 4500
AR Path="/5FF96C00/60A7BA8D" Ref="#PWR?"  Part="1" 
AR Path="/60100EBE/60A7BA8D" Ref="#PWR067"  Part="1" 
F 0 "#PWR067" H 5600 4250 50  0001 C CNN
F 1 "GND" H 5605 4327 50  0000 C CNN
F 2 "" H 5600 4500 50  0001 C CNN
F 3 "" H 5600 4500 50  0001 C CNN
	1    5600 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 4500 5600 4050
Wire Wire Line
	6050 3600 5600 3600
Connection ~ 5600 3600
Wire Wire Line
	1300 5300 2400 5300
$Comp
L Amplifier_Operational:MCP6004 U6
U 2 1 60A8BA22
P 3400 5950
F 0 "U6" H 3400 6317 50  0000 C CNN
F 1 "MCP6004(LM324)" H 3400 6226 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 3350 6050 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21733j.pdf" H 3450 6150 50  0001 C CNN
	2    3400 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 6050 3100 6250
Wire Wire Line
	3100 6250 3700 6250
Wire Wire Line
	3700 6250 3700 5950
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R?
U 1 1 60A9331F
P 3950 5950
AR Path="/5FF96B9E/60A9331F" Ref="R?"  Part="1" 
AR Path="/60100EBE/60A9331F" Ref="R59"  Part="1" 
F 0 "R59" V 3743 5950 50  0000 C CNN
F 1 "6k8" V 3834 5950 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 3880 5950 50  0001 C CNN
F 3 "~" H 3950 5950 50  0001 C CNN
	1    3950 5950
	0    1    1    0   
$EndComp
Wire Wire Line
	3800 5950 3700 5950
Connection ~ 3700 5950
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R?
U 1 1 60A95310
P 4300 6250
AR Path="/5FF96B9E/60A95310" Ref="R?"  Part="1" 
AR Path="/60100EBE/60A95310" Ref="R61"  Part="1" 
F 0 "R61" V 4093 6250 50  0000 C CNN
F 1 "10k" V 4184 6250 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 4230 6250 50  0001 C CNN
F 3 "~" H 4300 6250 50  0001 C CNN
	1    4300 6250
	-1   0    0    1   
$EndComp
Wire Wire Line
	4300 6100 4300 5950
Wire Wire Line
	4300 5950 4100 5950
Wire Wire Line
	4300 5950 4600 5950
Connection ~ 4300 5950
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR?
U 1 1 60A98CA4
P 4300 6550
AR Path="/5FF96C00/60A98CA4" Ref="#PWR?"  Part="1" 
AR Path="/60100EBE/60A98CA4" Ref="#PWR066"  Part="1" 
F 0 "#PWR066" H 4300 6300 50  0001 C CNN
F 1 "GND" H 4305 6377 50  0000 C CNN
F 2 "" H 4300 6550 50  0001 C CNN
F 3 "" H 4300 6550 50  0001 C CNN
	1    4300 6550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 6550 4300 6400
Wire Wire Line
	2400 6150 2400 6500
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R?
U 1 1 6008DAC0
P 1800 6100
AR Path="/5FF96B9E/6008DAC0" Ref="R?"  Part="1" 
AR Path="/60100EBE/6008DAC0" Ref="R55"  Part="1" 
F 0 "R55" H 1730 6054 50  0000 R CNN
F 1 "10k" H 1730 6145 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 1730 6100 50  0001 C CNN
F 3 "~" H 1800 6100 50  0001 C CNN
	1    1800 6100
	-1   0    0    1   
$EndComp
Wire Wire Line
	1800 6250 1800 6500
Wire Wire Line
	6250 1150 7550 1150
Wire Wire Line
	7550 2000 7550 2250
$Comp
L Car_bs_controller_nex_mega-rescue:C-Device C?
U 1 1 60AA5579
P 2800 6150
AR Path="/5FF96C00/60AA5579" Ref="C?"  Part="1" 
AR Path="/60100EBE/60AA5579" Ref="C30"  Part="1" 
F 0 "C30" H 2915 6196 50  0000 L CNN
F 1 "10n" H 2915 6105 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 2838 6000 50  0001 C CNN
F 3 "~" H 2800 6150 50  0001 C CNN
	1    2800 6150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 6000 2800 5850
Connection ~ 2800 5850
Wire Wire Line
	2800 5850 2600 5850
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR?
U 1 1 60AA73A6
P 2800 6500
AR Path="/5FF96C00/60AA73A6" Ref="#PWR?"  Part="1" 
AR Path="/60100EBE/60AA73A6" Ref="#PWR060"  Part="1" 
F 0 "#PWR060" H 2800 6250 50  0001 C CNN
F 1 "GND" H 2805 6327 50  0000 C CNN
F 2 "" H 2800 6500 50  0001 C CNN
F 3 "" H 2800 6500 50  0001 C CNN
	1    2800 6500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 6500 2800 6300
$Comp
L Car_bs_controller_nex_mega-rescue:C-Device C?
U 1 1 60AA933E
P 8050 2000
AR Path="/5FF96C00/60AA933E" Ref="C?"  Part="1" 
AR Path="/60100EBE/60AA933E" Ref="C35"  Part="1" 
F 0 "C35" H 8165 2046 50  0000 L CNN
F 1 "10n" H 8165 1955 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 8088 1850 50  0001 C CNN
F 3 "~" H 8050 2000 50  0001 C CNN
	1    8050 2000
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:MCP6004 U6
U 3 1 60AAC493
P 8700 1800
F 0 "U6" H 8700 2167 50  0000 C CNN
F 1 "MCP6004(LM324)" H 8700 2076 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 8650 1900 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21733j.pdf" H 8750 2000 50  0001 C CNN
	3    8700 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 1700 8050 1700
Wire Wire Line
	8400 1900 8400 2100
Wire Wire Line
	8400 2100 9000 2100
Wire Wire Line
	9000 2100 9000 1800
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R?
U 1 1 60AB2049
P 9200 1800
AR Path="/5FF96B9E/60AB2049" Ref="R?"  Part="1" 
AR Path="/60100EBE/60AB2049" Ref="R67"  Part="1" 
F 0 "R67" V 8993 1800 50  0000 C CNN
F 1 "6k8" V 9084 1800 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 9130 1800 50  0001 C CNN
F 3 "~" H 9200 1800 50  0001 C CNN
	1    9200 1800
	0    1    1    0   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R?
U 1 1 60AB3277
P 9450 2100
AR Path="/5FF96B9E/60AB3277" Ref="R?"  Part="1" 
AR Path="/60100EBE/60AB3277" Ref="R68"  Part="1" 
F 0 "R68" V 9243 2100 50  0000 C CNN
F 1 "10k" V 9334 2100 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 9380 2100 50  0001 C CNN
F 3 "~" H 9450 2100 50  0001 C CNN
	1    9450 2100
	-1   0    0    1   
$EndComp
Wire Wire Line
	9450 1950 9450 1800
Wire Wire Line
	9450 1800 9350 1800
Wire Wire Line
	9050 1800 9000 1800
Connection ~ 9000 1800
Wire Wire Line
	9450 1800 9750 1800
Connection ~ 9450 1800
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR?
U 1 1 60ABA33E
P 9450 2300
AR Path="/5FF96C00/60ABA33E" Ref="#PWR?"  Part="1" 
AR Path="/60100EBE/60ABA33E" Ref="#PWR072"  Part="1" 
F 0 "#PWR072" H 9450 2050 50  0001 C CNN
F 1 "GND" H 9455 2127 50  0000 C CNN
F 2 "" H 9450 2300 50  0001 C CNN
F 3 "" H 9450 2300 50  0001 C CNN
	1    9450 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	9450 2300 9450 2250
Wire Wire Line
	8050 1850 8050 1700
Connection ~ 8050 1700
Wire Wire Line
	8050 1700 8400 1700
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR?
U 1 1 60ABEDC2
P 8050 2250
AR Path="/5FF96C00/60ABEDC2" Ref="#PWR?"  Part="1" 
AR Path="/60100EBE/60ABEDC2" Ref="#PWR071"  Part="1" 
F 0 "#PWR071" H 8050 2000 50  0001 C CNN
F 1 "GND" H 8055 2077 50  0000 C CNN
F 2 "" H 8050 2250 50  0001 C CNN
F 3 "" H 8050 2250 50  0001 C CNN
	1    8050 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	8050 2250 8050 2150
$Comp
L Amplifier_Operational:MCP6004 U6
U 5 1 60AC1C82
P 7200 5250
F 0 "U6" H 6800 5300 50  0000 L CNN
F 1 "MCP6004(LM324)" H 6650 5200 50  0000 L CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 7150 5350 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21733j.pdf" H 7250 5450 50  0001 C CNN
	5    7200 5250
	1    0    0    -1  
$EndComp
Text GLabel 6950 4850 0    50   Input ~ 0
+5v_FROM_MCU
Wire Wire Line
	6950 4850 7100 4850
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR?
U 1 1 60AC5A47
P 7100 5650
AR Path="/5FF96C00/60AC5A47" Ref="#PWR?"  Part="1" 
AR Path="/60100EBE/60AC5A47" Ref="#PWR069"  Part="1" 
F 0 "#PWR069" H 7100 5400 50  0001 C CNN
F 1 "GND" H 7105 5477 50  0000 C CNN
F 2 "" H 7100 5650 50  0001 C CNN
F 3 "" H 7100 5650 50  0001 C CNN
	1    7100 5650
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:MCP6004 U6
U 4 1 60AC89DB
P 9000 4050
F 0 "U6" H 9000 4417 50  0000 C CNN
F 1 "MCP6004(LM324)" H 9000 4326 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 8950 4150 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21733j.pdf" H 9050 4250 50  0001 C CNN
	4    9000 4050
	1    0    0    -1  
$EndComp
NoConn ~ 8700 3950
NoConn ~ 8700 4150
NoConn ~ 9300 4050
Wire Wire Line
	3400 1500 3400 1450
$Comp
L Car_bs_controller_nex_mega-rescue:C-Device C?
U 1 1 607E2C1C
P 7400 5250
AR Path="/5FF96C00/607E2C1C" Ref="C?"  Part="1" 
AR Path="/60100EBE/607E2C1C" Ref="C33"  Part="1" 
F 0 "C33" H 7515 5296 50  0000 L CNN
F 1 "100n" H 7515 5205 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 7438 5100 50  0001 C CNN
F 3 "~" H 7400 5250 50  0001 C CNN
	1    7400 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 5600 7400 5600
Wire Wire Line
	7400 5600 7400 5400
Wire Wire Line
	7100 5650 7100 5600
Wire Wire Line
	7100 5600 7100 5550
Connection ~ 7100 5600
Wire Wire Line
	7400 5100 7400 4900
Wire Wire Line
	7400 4900 7100 4900
Wire Wire Line
	7100 4850 7100 4900
Wire Wire Line
	7100 4900 7100 4950
Connection ~ 7100 4900
Wire Wire Line
	7900 5400 7900 5600
Wire Wire Line
	7900 5600 7400 5600
Connection ~ 7400 5600
Wire Wire Line
	7900 5100 7900 4900
Wire Wire Line
	7900 4900 7400 4900
Connection ~ 7400 4900
$Comp
L Device:CP C34
U 1 1 607E9B4B
P 7900 5250
F 0 "C34" H 8018 5296 50  0000 L CNN
F 1 "10mk" H 8018 5205 50  0000 L CNN
F 2 "Capacitor_Tantalum_SMD:CP_EIA-6032-15_Kemet-U" H 7938 5100 50  0001 C CNN
F 3 "~" H 7900 5250 50  0001 C CNN
	1    7900 5250
	1    0    0    -1  
$EndComp
$Comp
L Device:L L4
U 1 1 607FD4F7
P 1600 1050
F 0 "L4" V 1419 1050 50  0000 C CNN
F 1 "FB" V 1510 1050 50  0000 C CNN
F 2 "Inductor_SMD:L_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 1600 1050 50  0001 C CNN
F 3 "~" H 1600 1050 50  0001 C CNN
	1    1600 1050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1750 1050 2200 1050
Wire Wire Line
	1450 1050 1300 1050
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R?
U 1 1 60104923
P 1950 3050
AR Path="/5FF96B9E/60104923" Ref="R?"  Part="1" 
AR Path="/60100EBE/60104923" Ref="R56"  Part="1" 
F 0 "R56" V 1743 3050 50  0000 C CNN
F 1 "2k*" V 1834 3050 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1880 3050 50  0001 C CNN
F 3 "~" H 1950 3050 50  0001 C CNN
	1    1950 3050
	-1   0    0    1   
$EndComp
Text Notes 4000 3100 0    79   ~ 0
k=1+(R62/R60)
Text Notes 5900 3950 0    50   ~ 0
15k(LM324)\n10k(MCP6004)
Text Notes 5500 3550 0    50   ~ 0
2.5V
Wire Notes Line
	550  750  4700 750 
Wire Notes Line
	4700 750  4700 1950
Wire Notes Line
	4700 1950 550  1950
Wire Notes Line
	550  1950 550  750 
Text Notes 800  1650 0    79   ~ 16
Optional
$EndSCHEMATC
