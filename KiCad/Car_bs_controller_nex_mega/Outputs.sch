EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 6 8
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
L Car_bs_controller_nex_mega-rescue:ULN2003A-Transistor_Array U4
U 1 1 6003689D
P 3900 2650
F 0 "U4" H 3900 3317 50  0000 C CNN
F 1 "ULN2003A" H 3900 3226 50  0000 C CNN
F 2 "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm" H 3950 2100 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/uln2003a.pdf" H 4000 2450 50  0001 C CNN
	1    3900 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 3750 6200 2550
Wire Wire Line
	6200 2550 5450 2550
Wire Wire Line
	6100 2650 5800 2650
$Comp
L Car_bs_controller_nex_mega-rescue:+12V-power #PWR055
U 1 1 6003D5D2
P 4300 1850
F 0 "#PWR055" H 4300 1700 50  0001 C CNN
F 1 "+12V" H 4315 2023 50  0000 C CNN
F 2 "" H 4300 1850 50  0001 C CNN
F 3 "" H 4300 1850 50  0001 C CNN
	1    4300 1850
	1    0    0    -1  
$EndComp
Text GLabel 2550 2450 0    50   Input ~ 0
MCU_water_pump_out
Text GLabel 2300 2650 0    50   Input ~ 0
MCU_light_out
Text GLabel 2450 2850 0    50   Input ~ 0
MCU_converter_out
Wire Wire Line
	2550 2450 3500 2450
Wire Wire Line
	3500 2550 3000 2550
Wire Wire Line
	3000 2550 3000 2650
Wire Wire Line
	3000 2650 2300 2650
Wire Wire Line
	3500 2650 3100 2650
Wire Wire Line
	3100 2650 3100 2850
Wire Wire Line
	3100 2850 2450 2850
Text GLabel 8950 2450 2    50   Output ~ 0
PUMP_RELAY_NO
Text GLabel 8950 2150 2    50   Output ~ 0
PUMP_RELAY_COM
Text GLabel 8950 3750 2    50   Output ~ 0
LIGHT_RELAY_NO
Text GLabel 8950 3450 2    50   Output ~ 0
LIGHT_RELAY_COM
Text GLabel 8850 4750 2    50   Output ~ 0
CONVERTER_RELAY_COM
Text GLabel 8850 5050 2    50   Output ~ 0
CONVERTER_RELAY_NO
Wire Wire Line
	6750 3750 6200 3750
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR054
U 1 1 60076725
P 3900 3400
F 0 "#PWR054" H 3900 3150 50  0001 C CNN
F 1 "GND" H 3905 3227 50  0000 C CNN
F 2 "" H 3900 3400 50  0001 C CNN
F 3 "" H 3900 3400 50  0001 C CNN
	1    3900 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 3400 3900 3250
Text GLabel 2400 3050 0    50   Input ~ 0
MCU_spare_out_1
Text GLabel 2400 3250 0    50   Input ~ 0
MCU_spare_out_2
Text GLabel 2400 3450 0    50   Input ~ 0
MCU_spare_out_3
Text GLabel 2400 3650 0    50   Input ~ 0
MCU_spare_out_4
Wire Wire Line
	3500 2750 3200 2750
Wire Wire Line
	3200 2750 3200 3050
Wire Wire Line
	3200 3050 2400 3050
Wire Wire Line
	3500 2850 3300 2850
Wire Wire Line
	3300 2850 3300 3250
Wire Wire Line
	3300 3250 2400 3250
Wire Wire Line
	3500 2950 3400 2950
Wire Wire Line
	3400 2950 3400 3450
Wire Wire Line
	3400 3450 2400 3450
Wire Wire Line
	3500 3050 3500 3650
Wire Wire Line
	3500 3650 2400 3650
Connection ~ 4300 2050
Wire Wire Line
	4300 2050 4300 2250
Wire Wire Line
	6750 3350 6350 3350
Wire Wire Line
	8950 3750 8200 3750
Wire Wire Line
	6350 4650 6350 3350
Wire Wire Line
	6350 4650 6750 4650
Wire Wire Line
	6750 5050 6100 5050
Wire Wire Line
	8850 5050 8200 5050
Wire Wire Line
	6100 2650 6100 5050
Wire Wire Line
	6350 3350 6350 2050
Connection ~ 6350 3350
Connection ~ 6350 2050
$Comp
L Car_bs_controller_nex_mega-rescue:Varistor-Device RV4
U 1 1 6001EA98
P 8250 2300
F 0 "RV4" H 8353 2346 50  0000 L CNN
F 1 "S07K17" H 8353 2255 50  0000 L CNN
F 2 "Varistor:RV_Disc_D9mm_W3.7mm_P5mm" V 8180 2300 50  0001 C CNN
F 3 "~" H 8250 2300 50  0001 C CNN
	1    8250 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 2450 8950 2450
Wire Wire Line
	8950 2150 8250 2150
Connection ~ 8250 2150
$Comp
L Car_bs_controller_nex_mega-rescue:Varistor-Device RV2
U 1 1 60020EDF
P 8200 3600
F 0 "RV2" H 8303 3646 50  0000 L CNN
F 1 "S07K17" H 8303 3555 50  0000 L CNN
F 2 "Varistor:RV_Disc_D9mm_W3.7mm_P5mm" V 8130 3600 50  0001 C CNN
F 3 "~" H 8200 3600 50  0001 C CNN
	1    8200 3600
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:Varistor-Device RV3
U 1 1 60021A69
P 8200 4900
F 0 "RV3" H 8303 4946 50  0000 L CNN
F 1 "S07K17" H 8303 4855 50  0000 L CNN
F 2 "Varistor:RV_Disc_D9mm_W3.7mm_P5mm" V 8130 4900 50  0001 C CNN
F 3 "~" H 8200 4900 50  0001 C CNN
	1    8200 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	8950 3450 8200 3450
Connection ~ 8200 3450
Connection ~ 8200 5050
Wire Wire Line
	8200 5050 7450 5050
Wire Wire Line
	8850 4750 8200 4750
Wire Wire Line
	8200 4750 7450 4750
Connection ~ 8200 4750
Wire Wire Line
	4600 4350 4600 2750
Wire Wire Line
	4600 2750 4300 2750
Wire Wire Line
	4300 2850 4650 2850
Wire Wire Line
	4650 2850 4650 4450
Wire Wire Line
	4300 2950 4700 2950
Wire Wire Line
	4700 2950 4700 4550
Wire Wire Line
	4750 4650 4750 3050
Wire Wire Line
	4750 3050 4300 3050
Wire Wire Line
	4300 2050 4900 2050
$Comp
L Car_bs_controller_nex_mega-rescue:DIODE-Simulation_SPICE D37
U 1 1 60039BB9
P 4550 4750
F 0 "D37" H 4550 4850 50  0000 C CNN
F 1 "SS14" H 4500 4950 50  0000 C CNN
F 2 "Diode_SMD:D_SMA" H 4550 4750 50  0001 C CNN
F 3 "~" H 4550 4750 50  0001 C CNN
F 4 "Y" H 4550 4750 50  0001 L CNN "Spice_Netlist_Enabled"
F 5 "D" H 4550 4750 50  0001 L CNN "Spice_Primitive"
	1    4550 4750
	-1   0    0    1   
$EndComp
Wire Wire Line
	4700 4750 4900 4750
Wire Wire Line
	4900 4750 4900 3500
Connection ~ 4900 2050
Wire Wire Line
	4900 2050 6350 2050
Text GLabel 4050 4350 0    50   Output ~ 0
Socket_spare_out_1
Text GLabel 4050 4450 0    50   Output ~ 0
Socket_spare_out_2
Text GLabel 4050 4550 0    50   Output ~ 0
Socket_spare_out_3
Text GLabel 4050 4650 0    50   Output ~ 0
Socket_spare_out_4
Text GLabel 4050 4750 0    50   Output ~ 0
Socket_spare_out_+12v
Wire Wire Line
	4050 4350 4600 4350
Wire Wire Line
	4050 4450 4650 4450
Wire Wire Line
	4050 4550 4700 4550
Wire Wire Line
	4050 4650 4750 4650
Wire Wire Line
	4050 4750 4400 4750
NoConn ~ 2750 3050
NoConn ~ 2750 3250
NoConn ~ 2750 3450
NoConn ~ 2750 3650
Wire Wire Line
	4300 1850 4300 2050
Wire Wire Line
	4300 2450 5100 2450
$Comp
L Car_bs_controller_nex_mega-rescue:LED-Device D?
U 1 1 6011C2E4
P 5100 3250
AR Path="/5FF96CA3/6011C2E4" Ref="D?"  Part="1" 
AR Path="/5FF96DB0/6011C2E4" Ref="D38"  Part="1" 
F 0 "D38" V 5200 3300 50  0000 L CNN
F 1 "LED_K1" V 5450 3100 50  0000 L CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 5100 3250 50  0001 C CNN
F 3 "~" H 5100 3250 50  0001 C CNN
	1    5100 3250
	0    -1   1    0   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R?
U 1 1 6011C2EA
P 5100 2900
AR Path="/5FF96CA3/6011C2EA" Ref="R?"  Part="1" 
AR Path="/5FF96DB0/6011C2EA" Ref="R46"  Part="1" 
F 0 "R46" H 5170 2946 50  0000 L CNN
F 1 "10k" H 5170 2855 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5030 2900 50  0001 C CNN
F 3 "~" H 5100 2900 50  0001 C CNN
	1    5100 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 3100 5100 3050
Wire Wire Line
	5100 3500 5100 3400
$Comp
L Car_bs_controller_nex_mega-rescue:LED-Device D?
U 1 1 60124D70
P 5450 3250
AR Path="/5FF96CA3/60124D70" Ref="D?"  Part="1" 
AR Path="/5FF96DB0/60124D70" Ref="D39"  Part="1" 
F 0 "D39" V 5550 3300 50  0000 L CNN
F 1 "LED_K2" V 5800 3100 50  0000 L CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 5450 3250 50  0001 C CNN
F 3 "~" H 5450 3250 50  0001 C CNN
	1    5450 3250
	0    -1   1    0   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R?
U 1 1 60124D76
P 5450 2900
AR Path="/5FF96CA3/60124D76" Ref="R?"  Part="1" 
AR Path="/5FF96DB0/60124D76" Ref="R47"  Part="1" 
F 0 "R47" H 5520 2946 50  0000 L CNN
F 1 "10k" H 5520 2855 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5380 2900 50  0001 C CNN
F 3 "~" H 5450 2900 50  0001 C CNN
	1    5450 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 3100 5450 3050
Wire Wire Line
	5450 3500 5450 3400
$Comp
L Car_bs_controller_nex_mega-rescue:LED-Device D?
U 1 1 6012B195
P 5800 3250
AR Path="/5FF96CA3/6012B195" Ref="D?"  Part="1" 
AR Path="/5FF96DB0/6012B195" Ref="D40"  Part="1" 
F 0 "D40" V 5900 3300 50  0000 L CNN
F 1 "LED_K3" V 6150 3100 50  0000 L CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 5800 3250 50  0001 C CNN
F 3 "~" H 5800 3250 50  0001 C CNN
	1    5800 3250
	0    -1   1    0   
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:R-Device R?
U 1 1 6012B19B
P 5800 2900
AR Path="/5FF96CA3/6012B19B" Ref="R?"  Part="1" 
AR Path="/5FF96DB0/6012B19B" Ref="R48"  Part="1" 
F 0 "R48" H 5870 2946 50  0000 L CNN
F 1 "10k" H 5870 2855 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5730 2900 50  0001 C CNN
F 3 "~" H 5800 2900 50  0001 C CNN
	1    5800 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5800 3100 5800 3050
Wire Wire Line
	5800 3500 5800 3400
Wire Wire Line
	5800 3500 5450 3500
Wire Wire Line
	5100 3500 5450 3500
Connection ~ 5450 3500
Wire Wire Line
	5100 3500 4900 3500
Connection ~ 5100 3500
Connection ~ 4900 3500
Wire Wire Line
	4900 3500 4900 2050
Wire Wire Line
	5100 2450 5100 2750
Connection ~ 5100 2450
Wire Wire Line
	5100 2450 6750 2450
Wire Wire Line
	5450 2550 5450 2750
Connection ~ 5450 2550
Wire Wire Line
	5450 2550 4300 2550
Wire Wire Line
	5800 2650 5800 2750
Connection ~ 5800 2650
Wire Wire Line
	5800 2650 4300 2650
Wire Wire Line
	8250 2150 7550 2150
Wire Wire Line
	8200 3450 7550 3450
Wire Wire Line
	6350 2050 6750 2050
$Comp
L ab2_relay:SRA-12VDC-CL REL2
U 1 1 609D3C8A
P 7100 4900
F 0 "REL2" H 7100 5277 60  0000 C CNN
F 1 "SRA-12VDC-CL" V 6700 4950 60  0000 C CNN
F 2 "ab2_relay:AB2_SRA-12VDC-CL" V 6550 4850 60  0000 C CNN
F 3 "" H 7205 4835 60  0000 C CNN
	1    7100 4900
	0    -1   1    0   
$EndComp
Wire Wire Line
	7450 4750 7450 4650
Wire Wire Line
	7450 4650 7150 4650
Wire Wire Line
	7450 5050 7450 5150
Wire Wire Line
	7450 5150 7200 5150
Wire Wire Line
	7000 5150 6750 5150
Wire Wire Line
	6750 5150 6750 5050
Wire Wire Line
	7000 4650 7000 4600
Wire Wire Line
	7000 4600 6750 4600
Wire Wire Line
	6750 4600 6750 4650
NoConn ~ 7100 5150
$Comp
L ab2_relay:SRA-12VDC-CL REL1
U 1 1 609F4C66
P 7100 3600
F 0 "REL1" H 7100 3977 60  0000 C CNN
F 1 "SRA-12VDC-CL" V 6700 3650 60  0000 C CNN
F 2 "ab2_relay:AB2_SRA-12VDC-CL" V 6550 3500 60  0000 C CNN
F 3 "" H 7205 3535 60  0000 C CNN
	1    7100 3600
	0    -1   1    0   
$EndComp
$Comp
L ab2_relay:SRA-12VDC-CL REL3
U 1 1 609F846A
P 7150 2250
F 0 "REL3" H 7150 2627 60  0000 C CNN
F 1 "SRA-12VDC-CL" V 6750 2300 60  0000 C CNN
F 2 "ab2_relay:AB2_SRA-12VDC-CL" V 6600 2250 60  0000 C CNN
F 3 "" H 7255 2185 60  0000 C CNN
	1    7150 2250
	0    -1   1    0   
$EndComp
Wire Wire Line
	7550 2150 7550 2000
Wire Wire Line
	7550 2000 7200 2000
Wire Wire Line
	8250 2450 7550 2450
Wire Wire Line
	7550 2450 7550 2500
Wire Wire Line
	7550 2500 7250 2500
Connection ~ 8250 2450
Wire Wire Line
	7050 2000 6750 2000
Wire Wire Line
	6750 2000 6750 2050
Wire Wire Line
	7050 2500 6750 2500
Wire Wire Line
	6750 2500 6750 2450
NoConn ~ 7150 2500
Wire Wire Line
	7550 3450 7550 3350
Wire Wire Line
	7550 3350 7150 3350
Wire Wire Line
	8200 3750 7550 3750
Wire Wire Line
	7550 3750 7550 3850
Wire Wire Line
	7550 3850 7200 3850
Connection ~ 8200 3750
Wire Wire Line
	7000 3850 6750 3850
Wire Wire Line
	6750 3850 6750 3750
Wire Wire Line
	7000 3350 7000 3300
Wire Wire Line
	7000 3300 6750 3300
Wire Wire Line
	6750 3300 6750 3350
NoConn ~ 7100 3850
$EndSCHEMATC
