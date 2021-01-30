EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 7 8
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector_Generic:Conn_01x03 J3
U 1 1 6001F76C
P 2650 2600
F 0 "J3" H 2730 2642 50  0000 L CNN
F 1 "Proxy " H 2730 2551 50  0000 L CNB
F 2 "Connector_Phoenix_MC:PhoenixContact_MC_1,5_3-G-3.81_1x03_P3.81mm_Horizontal" H 2650 2600 50  0001 C CNN
F 3 "~" H 2650 2600 50  0001 C CNN
	1    2650 2600
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J1
U 1 1 6002058B
P 2650 1100
F 0 "J1" H 2730 1142 50  0000 L CNN
F 1 "Power" H 2730 1051 50  0000 L CNB
F 2 "Connector_Phoenix_MSTB:PhoenixContact_MSTBA_2,5_3-G_1x03_P5.00mm_Horizontal" H 2650 1100 50  0001 C CNN
F 3 "~" H 2650 1100 50  0001 C CNN
	1    2650 1100
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J12
U 1 1 6003D07B
P 9950 1550
F 0 "J12" H 10030 1542 50  0000 L CNN
F 1 "PUMP_RELAY" H 10030 1451 50  0000 L CNN
F 2 "Connector_Phoenix_MSTB:PhoenixContact_MSTBA_2,5_2-G_1x02_P5.00mm_Horizontal" H 9950 1550 50  0001 C CNN
F 3 "~" H 9950 1550 50  0001 C CNN
	1    9950 1550
	1    0    0    -1  
$EndComp
Text GLabel 6250 950  0    50   Input ~ 0
Socket_spare_out_1
$Comp
L Connector_Generic:Conn_01x05 J8
U 1 1 6003ABD7
P 6600 1150
F 0 "J8" H 6680 1192 50  0000 L CNN
F 1 "Outputs" H 6680 1101 50  0000 L CNN
F 2 "Connector_Hirose:Hirose_DF13-05P-1.25DS_1x05_P1.25mm_Horizontal" H 6600 1150 50  0001 C CNN
F 3 "~" H 6600 1150 50  0001 C CNN
	1    6600 1150
	1    0    0    -1  
$EndComp
Text GLabel 6250 1050 0    50   Input ~ 0
Socket_spare_out_2
Text GLabel 6250 1150 0    50   Input ~ 0
Socket_spare_out_3
Text GLabel 6250 1250 0    50   Input ~ 0
Socket_spare_out_4
Text GLabel 6250 1350 0    50   Input ~ 0
Socket_spare_out_+12v
Wire Wire Line
	6400 950  6250 950 
Wire Wire Line
	6400 1050 6250 1050
Wire Wire Line
	6400 1150 6250 1150
Wire Wire Line
	6400 1250 6250 1250
Wire Wire Line
	6400 1350 6250 1350
Text GLabel 9750 1550 0    50   Input ~ 0
PUMP_RELAY_COM
Text GLabel 9750 1650 0    50   Input ~ 0
PUMP_RELAY_NO
$Comp
L Connector_Generic:Conn_01x02 J13
U 1 1 60022055
P 9950 1950
F 0 "J13" H 10030 1942 50  0000 L CNN
F 1 "LIGHT_RELAY" H 10030 1851 50  0000 L CNN
F 2 "Connector_Phoenix_MSTB:PhoenixContact_MSTBA_2,5_2-G_1x02_P5.00mm_Horizontal" H 9950 1950 50  0001 C CNN
F 3 "~" H 9950 1950 50  0001 C CNN
	1    9950 1950
	1    0    0    -1  
$EndComp
Text GLabel 9750 1950 0    50   Input ~ 0
LIGHT_RELAY_COM
Text GLabel 9750 2050 0    50   Input ~ 0
LIGHT_RELAY_NO
$Comp
L Connector_Generic:Conn_01x02 J14
U 1 1 60022F3B
P 9950 2350
F 0 "J14" H 10030 2342 50  0000 L CNN
F 1 "CONVERTER_RELAY" H 10030 2251 50  0000 L CNN
F 2 "Connector_Phoenix_MSTB:PhoenixContact_MSTBA_2,5_2-G_1x02_P5.00mm_Horizontal" H 9950 2350 50  0001 C CNN
F 3 "~" H 9950 2350 50  0001 C CNN
	1    9950 2350
	1    0    0    -1  
$EndComp
Text GLabel 9750 2350 0    50   Input ~ 0
CONVERTER_RELAY_COM
Text GLabel 9750 2450 0    50   Input ~ 0
CONVERTER_RELAY_NO
Text GLabel 2250 1000 0    50   Input ~ 0
Power_enable(ACC,BUTTON)
Wire Wire Line
	2450 1000 2250 1000
Text GLabel 2250 1100 0    50   Input ~ 0
Power_supply_12V
Text GLabel 2250 1200 0    50   Input ~ 0
GND
Wire Wire Line
	2250 1100 2450 1100
Wire Wire Line
	2250 1200 2450 1200
$Comp
L Connector_Generic:Conn_01x05 J2
U 1 1 60030305
P 2650 1650
F 0 "J2" H 2730 1692 50  0000 L CNN
F 1 "Inputs" H 2730 1601 50  0000 L CNB
F 2 "Connector_Phoenix_MC:PhoenixContact_MC_1,5_5-G-3.81_1x05_P3.81mm_Horizontal" H 2650 1650 50  0001 C CNN
F 3 "~" H 2650 1650 50  0001 C CNN
	1    2650 1650
	1    0    0    -1  
$EndComp
Text GLabel 2250 1450 0    50   Input ~ 0
DOOR_SWITCH_INPUT
Wire Wire Line
	2450 1450 2250 1450
Text GLabel 2250 1550 0    50   Input ~ 0
IGNITION_SWITCH_INPUT
Wire Wire Line
	2450 1550 2250 1550
Text GLabel 2250 1650 0    50   Input ~ 0
LOW_WASHER_WATER_LEVEL_INPUT
Wire Wire Line
	2450 1650 2250 1650
Wire Wire Line
	2450 1750 2250 1750
Wire Wire Line
	2450 1850 2250 1850
Text GLabel 1350 2500 0    50   Input ~ 0
5V_sensors_stab
$Comp
L Simulation_SPICE:DIODE D43
U 1 1 600A3DD8
P 2000 2500
F 0 "D43" H 2000 2717 50  0000 C CNN
F 1 "SS14" H 2000 2626 50  0000 C CNN
F 2 "Diode_SMD:D_SMA" H 2000 2500 50  0001 C CNN
F 3 "~" H 2000 2500 50  0001 C CNN
F 4 "Y" H 2000 2500 50  0001 L CNN "Spice_Netlist_Enabled"
F 5 "D" H 2000 2500 50  0001 L CNN "Spice_Primitive"
	1    2000 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 2500 1350 2500
$Comp
L Device:Fuse F3
U 1 1 600A579A
P 2300 2500
F 0 "F3" V 2103 2500 50  0000 C CNN
F 1 "200mA" V 2194 2500 50  0000 C CNN
F 2 "Fuse:Fuse_1210_3225Metric" V 2230 2500 50  0001 C CNN
F 3 "~" H 2300 2500 50  0001 C CNN
	1    2300 2500
	0    1    1    0   
$EndComp
Text GLabel 1700 2650 0    50   Input ~ 0
PROXIMITY_SENSOR_INPUT
Wire Wire Line
	1700 2650 2150 2650
Wire Wire Line
	2150 2650 2150 2600
Wire Wire Line
	2150 2600 2450 2600
Text GLabel 1350 2800 0    50   Input ~ 0
GND
Wire Wire Line
	2450 2700 2250 2700
Wire Wire Line
	2250 2700 2250 2800
$Comp
L Connector_Generic:Conn_01x03 J4
U 1 1 600AB02C
P 2650 3850
F 0 "J4" H 2730 3892 50  0000 L CNN
F 1 "PJON" H 2730 3801 50  0000 L CNB
F 2 "Connector_Phoenix_MC:PhoenixContact_MC_1,5_3-G-3.81_1x03_P3.81mm_Horizontal" H 2650 3850 50  0001 C CNN
F 3 "~" H 2650 3850 50  0001 C CNN
	1    2650 3850
	1    0    0    -1  
$EndComp
Text GLabel 1350 3750 0    50   Input ~ 0
5V_sensors_stab
$Comp
L Simulation_SPICE:DIODE D42
U 1 1 600AB035
P 1850 3750
F 0 "D42" H 1850 3967 50  0000 C CNN
F 1 "SS14" H 1850 3876 50  0000 C CNN
F 2 "Diode_SMD:D_SMA" H 1850 3750 50  0001 C CNN
F 3 "~" H 1850 3750 50  0001 C CNN
F 4 "Y" H 1850 3750 50  0001 L CNN "Spice_Netlist_Enabled"
F 5 "D" H 1850 3750 50  0001 L CNN "Spice_Primitive"
	1    1850 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 3750 2350 3750
Wire Wire Line
	1700 3750 1650 3750
$Comp
L Device:Fuse F2
U 1 1 600AB03D
P 2200 3750
F 0 "F2" V 2003 3750 50  0000 C CNN
F 1 "200mA 5v" V 2094 3750 50  0000 C CNN
F 2 "Fuse:Fuse_1210_3225Metric" V 2130 3750 50  0001 C CNN
F 3 "~" H 2200 3750 50  0001 C CNN
	1    2200 3750
	0    1    1    0   
$EndComp
Wire Wire Line
	2050 3750 2000 3750
Wire Wire Line
	2150 3900 2150 3850
Wire Wire Line
	2150 3850 2450 3850
Text GLabel 1350 4050 0    50   Input ~ 0
GND
Wire Wire Line
	2450 3950 2250 3950
Wire Wire Line
	2250 3950 2250 4050
Wire Wire Line
	2250 4050 1350 4050
$Comp
L Connector_Generic:Conn_01x03 J7
U 1 1 600AC237
P 3000 5050
F 0 "J7" H 3080 5092 50  0000 L CNN
F 1 "1_WARE" H 3080 5001 50  0000 L CNB
F 2 "Connector_Phoenix_MC:PhoenixContact_MC_1,5_3-G-3.81_1x03_P3.81mm_Horizontal" H 3000 5050 50  0001 C CNN
F 3 "~" H 3000 5050 50  0001 C CNN
	1    3000 5050
	1    0    0    -1  
$EndComp
Text GLabel 1700 4950 0    50   Input ~ 0
5V_sensors_stab
$Comp
L Simulation_SPICE:DIODE D44
U 1 1 600AC240
P 2250 4950
F 0 "D44" H 2250 5167 50  0000 C CNN
F 1 "SS14" H 2250 5076 50  0000 C CNN
F 2 "Diode_SMD:D_SMA" H 2250 4950 50  0001 C CNN
F 3 "~" H 2250 4950 50  0001 C CNN
F 4 "Y" H 2250 4950 50  0001 L CNN "Spice_Netlist_Enabled"
F 5 "D" H 2250 4950 50  0001 L CNN "Spice_Primitive"
	1    2250 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 4950 2700 4950
Wire Wire Line
	2100 4950 1700 4950
$Comp
L Device:Fuse F4
U 1 1 600AC248
P 2550 4950
F 0 "F4" V 2353 4950 50  0000 C CNN
F 1 "200mA" V 2444 4950 50  0000 C CNN
F 2 "Fuse:Fuse_1210_3225Metric" V 2480 4950 50  0001 C CNN
F 3 "~" H 2550 4950 50  0001 C CNN
	1    2550 4950
	0    1    1    0   
$EndComp
Wire Wire Line
	2500 5100 2500 5050
Wire Wire Line
	2500 5050 2800 5050
Text GLabel 1700 5250 0    50   Input ~ 0
GND
Wire Wire Line
	2800 5150 2600 5150
Wire Wire Line
	2600 5150 2600 5250
Wire Wire Line
	2600 5250 1700 5250
Text GLabel 1350 3900 0    50   Input ~ 0
PJON_BUS
Wire Wire Line
	1350 3900 2150 3900
Text GLabel 1700 5100 0    50   Input ~ 0
ONE_WARE_INPUT
Wire Wire Line
	1700 5100 2500 5100
$Comp
L Connector_Generic:Conn_01x02 J5
U 1 1 600B6D74
P 2750 6200
F 0 "J5" H 2830 6192 50  0000 L CNN
F 1 "RS485" H 2830 6101 50  0000 L CNN
F 2 "Connector_Phoenix_MC:PhoenixContact_MC_1,5_2-G-3.81_1x02_P3.81mm_Horizontal" H 2750 6200 50  0001 C CNN
F 3 "~" H 2750 6200 50  0001 C CNN
	1    2750 6200
	1    0    0    -1  
$EndComp
Text GLabel 2300 6200 0    50   Input ~ 0
RS485_A
Text GLabel 2300 6300 0    50   Input ~ 0
RS485_B
Wire Wire Line
	2550 6200 2300 6200
Wire Wire Line
	2550 6300 2300 6300
$Comp
L Connector_Generic:Conn_01x05 J10
U 1 1 600B9818
P 6700 2150
F 0 "J10" H 6780 2192 50  0000 L CNN
F 1 "Encoder" H 6780 2101 50  0000 L CNN
F 2 "Connector_Hirose:Hirose_DF13-05P-1.25DSA_1x05_P1.25mm_Vertical" H 6700 2150 50  0001 C CNN
F 3 "~" H 6700 2150 50  0001 C CNN
	1    6700 2150
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J6
U 1 1 6002F5B7
P 2750 6900
F 0 "J6" H 2830 6892 50  0000 L CNN
F 1 "RES_SENS" H 2830 6801 50  0000 L CNN
F 2 "Connector_Phoenix_MC:PhoenixContact_MC_1,5_2-G-3.81_1x02_P3.81mm_Horizontal" H 2750 6900 50  0001 C CNN
F 3 "~" H 2750 6900 50  0001 C CNN
	1    2750 6900
	1    0    0    -1  
$EndComp
Text GLabel 2350 6900 0    50   Input ~ 0
RESISTIVE_SENSOR_0_190
Text GLabel 2350 7000 0    50   Input ~ 0
GND
Wire Wire Line
	2350 6900 2550 6900
Wire Wire Line
	2550 7000 2350 7000
Text GLabel 5800 1950 0    50   Input ~ 0
GND
Text GLabel 5800 2050 0    50   Input ~ 0
+5v_FROM_MCU
Text GLabel 5800 2150 0    50   Input ~ 0
MCU_encoder_SW
Text GLabel 5800 2250 0    50   Input ~ 0
MCU_encoder_DT
Wire Wire Line
	5800 1950 6500 1950
Wire Wire Line
	5800 2050 5850 2050
Wire Wire Line
	5800 2150 6500 2150
Wire Wire Line
	5800 2250 6500 2250
Wire Wire Line
	5800 2350 6500 2350
$Comp
L Connector_Generic:Conn_01x04 J11
U 1 1 6004C8D2
P 7150 3250
F 0 "J11" H 7230 3242 50  0000 L CNN
F 1 "LCD_Nextion" H 7230 3151 50  0000 L CNN
F 2 "Connector_Hirose:Hirose_DF13-04P-1.25DSA_1x04_P1.25mm_Vertical" H 7150 3250 50  0001 C CNN
F 3 "~" H 7150 3250 50  0001 C CNN
	1    7150 3250
	1    0    0    -1  
$EndComp
Text GLabel 5400 3250 0    50   Input ~ 0
MCU_UART1_RX
Text GLabel 5400 3350 0    50   Input ~ 0
MCU_UART1_TX
Text GLabel 5000 3450 0    50   Input ~ 0
GND
Wire Wire Line
	6950 3250 5400 3250
Wire Wire Line
	6950 3450 6350 3450
$Comp
L Connector_Generic:Conn_01x08 J9
U 1 1 6005D984
P 6600 4700
F 0 "J9" H 6680 4692 50  0000 L CNN
F 1 "LCD_ST7920" H 6680 4601 50  0000 L CNN
F 2 "Connector_Hirose:Hirose_DF13-08P-1.25DSA_1x08_P1.25mm_Vertical" H 6600 4700 50  0001 C CNN
F 3 "~" H 6600 4700 50  0001 C CNN
	1    6600 4700
	1    0    0    -1  
$EndComp
Text GLabel 5800 4700 0    50   Input ~ 0
LCD_DI_MCU_CS
Text GLabel 5800 4800 0    50   Input ~ 0
LCD_RW_MCU_MOSI
Text GLabel 5800 4900 0    50   Input ~ 0
LCD_E_MCU_SCLK
Text GLabel 5800 5000 0    50   Input ~ 0
LCD_RST
Text GLabel 5800 5100 0    50   Input ~ 0
LCD_BLA
Wire Wire Line
	6400 4500 6250 4500
Wire Wire Line
	6400 4400 6350 4400
Wire Wire Line
	6400 4700 5800 4700
Wire Wire Line
	6400 4800 5800 4800
Wire Wire Line
	6400 4900 5800 4900
Wire Wire Line
	6400 5000 5800 5000
$Comp
L Device:R R66
U 1 1 600468C7
P 6250 5350
F 0 "R66" V 6200 5550 50  0000 C CNN
F 1 "220R" V 6350 5350 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 6180 5350 50  0001 C CNN
F 3 "~" H 6250 5350 50  0001 C CNN
	1    6250 5350
	-1   0    0    1   
$EndComp
Wire Wire Line
	6400 5100 6250 5100
$Comp
L Device:R_POT RV5
U 1 1 60052726
P 5700 4200
F 0 "RV5" H 5630 4246 50  0000 R CNN
F 1 "10k" H 5630 4155 50  0000 R CNN
F 2 "Potentiometer_SMD:Potentiometer_Bourns_3314J_Vertical" H 5700 4200 50  0001 C CNN
F 3 "~" H 5700 4200 50  0001 C CNN
	1    5700 4200
	1    0    0    -1  
$EndComp
Connection ~ 6350 3450
Wire Wire Line
	6350 3450 5000 3450
Wire Wire Line
	6350 4400 5700 4400
Wire Wire Line
	5700 4400 5700 4350
Connection ~ 6350 4400
Wire Wire Line
	5700 4050 5700 4000
Wire Wire Line
	5700 4000 6250 4000
Wire Wire Line
	6250 4000 6250 4500
Wire Wire Line
	5850 4200 6050 4200
Wire Wire Line
	6050 4200 6050 4600
Wire Wire Line
	6050 4600 6400 4600
$Comp
L Device:Fuse F6
U 1 1 600838D1
P 6000 2050
F 0 "F6" V 5750 2100 50  0000 C CNN
F 1 "200mA 5v" V 5850 2100 50  0000 C CNN
F 2 "Fuse:Fuse_1210_3225Metric" V 5930 2050 50  0001 C CNN
F 3 "~" H 6000 2050 50  0001 C CNN
	1    6000 2050
	0    1    1    0   
$EndComp
Text GLabel 5800 2350 0    50   Input ~ 0
MCU_encoder_CLK
Connection ~ 1850 2500
Wire Wire Line
	2250 2800 1350 2800
Wire Wire Line
	1850 2900 1850 2500
$Comp
L Device:C C31
U 1 1 600C6520
P 1850 3050
F 0 "C31" H 1965 3096 50  0000 L CNN
F 1 "100n" H 1965 3005 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1888 2900 50  0001 C CNN
F 3 "~" H 1850 3050 50  0001 C CNN
	1    1850 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 3200 2250 3200
Wire Wire Line
	2250 3200 2250 2800
Connection ~ 2250 2800
$Comp
L Device:C C30
U 1 1 600DE72B
P 1650 4250
F 0 "C30" H 1765 4296 50  0000 L CNN
F 1 "100n" H 1765 4205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1688 4100 50  0001 C CNN
F 3 "~" H 1650 4250 50  0001 C CNN
	1    1650 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 4100 1650 3750
Connection ~ 1650 3750
Wire Wire Line
	1650 3750 1350 3750
Wire Wire Line
	1650 4400 2250 4400
Wire Wire Line
	2250 4400 2250 4050
Connection ~ 2250 4050
$Comp
L Device:C C32
U 1 1 600E6537
P 2100 5550
F 0 "C32" H 2215 5596 50  0000 L CNN
F 1 "100n" H 2215 5505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2138 5400 50  0001 C CNN
F 3 "~" H 2100 5550 50  0001 C CNN
	1    2100 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 5400 2100 4950
Connection ~ 2100 4950
Wire Wire Line
	2100 5700 2600 5700
Wire Wire Line
	2600 5700 2600 5250
Connection ~ 2600 5250
Text GLabel 2250 1750 0    50   Input ~ 0
WATER_FLOW_SENSOR_INPUT
Text GLabel 2250 1850 0    50   Input ~ 0
SPARE_INPUT_1
Wire Wire Line
	6950 3350 5400 3350
Wire Wire Line
	6150 2050 6250 2050
Wire Wire Line
	6250 2050 6250 2750
Connection ~ 6250 2050
Wire Wire Line
	6250 2050 6500 2050
$Comp
L Jumper:SolderJumper_3_Open JP1
U 1 1 601EDF36
P 6550 2950
F 0 "JP1" V 6596 3017 50  0000 L CNN
F 1 "SolderJumper_3_Open" V 6750 2050 50  0000 L CNN
F 2 "Jumper:SolderJumper-3_P1.3mm_Open_RoundedPad1.0x1.5mm_NumberLabels" H 6550 2950 50  0001 C CNN
F 3 "~" H 6550 2950 50  0001 C CNN
	1    6550 2950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6700 2950 6950 2950
Wire Wire Line
	6950 2950 6950 3150
Wire Wire Line
	6550 2750 6250 2750
Connection ~ 6250 2750
Connection ~ 6250 4000
$Comp
L Device:Fuse F5
U 1 1 6020080D
P 5700 3150
F 0 "F5" V 5450 3200 50  0000 C CNN
F 1 "200mA 5v" V 5550 3200 50  0000 C CNN
F 2 "Fuse:Fuse_1210_3225Metric" V 5630 3150 50  0001 C CNN
F 3 "~" H 5700 3150 50  0001 C CNN
	1    5700 3150
	0    1    1    0   
$EndComp
Text GLabel 5450 3150 0    50   Input ~ 0
+3v3_from_MCU
Wire Wire Line
	5450 3150 5500 3150
Wire Wire Line
	5850 3150 6550 3150
$Comp
L power:PWR_FLAG #FLG05
U 1 1 600E9BEB
P 5250 2950
F 0 "#FLG05" H 5250 3025 50  0001 C CNN
F 1 "PWR_FLAG" H 5250 3123 50  0000 C CNN
F 2 "" H 5250 2950 50  0001 C CNN
F 3 "~" H 5250 2950 50  0001 C CNN
	1    5250 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	5250 2950 5250 3000
Wire Wire Line
	5250 3000 5500 3000
Wire Wire Line
	5500 3000 5500 3150
Connection ~ 5500 3150
Wire Wire Line
	5500 3150 5550 3150
Wire Wire Line
	6250 5200 6250 5100
$Comp
L Transistor_BJT:BC857 Q2
U 1 1 601A9050
P 6150 5850
F 0 "Q2" H 6341 5896 50  0000 L CNN
F 1 "BC857" H 6341 5805 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 6350 5775 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/BC860-D.pdf" H 6150 5850 50  0001 L CNN
	1    6150 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 5650 6250 5500
$Comp
L Device:R R64
U 1 1 601AD207
P 5750 5600
F 0 "R64" H 5820 5646 50  0000 L CNN
F 1 "1k" H 5820 5555 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5680 5600 50  0001 C CNN
F 3 "~" H 5750 5600 50  0001 C CNN
	1    5750 5600
	1    0    0    -1  
$EndComp
$Comp
L Device:R R65
U 1 1 601AE134
P 5750 6100
F 0 "R65" H 5820 6146 50  0000 L CNN
F 1 "10k" H 5820 6055 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5680 6100 50  0001 C CNN
F 3 "~" H 5750 6100 50  0001 C CNN
	1    5750 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 6050 6250 6350
Wire Wire Line
	6250 6350 5750 6350
Wire Wire Line
	5750 6350 5750 6250
Wire Wire Line
	5950 5850 5750 5850
Wire Wire Line
	5750 5850 5750 5750
Wire Wire Line
	5750 5950 5750 5850
Connection ~ 5750 5850
Wire Wire Line
	6250 2750 6250 4000
Wire Wire Line
	5750 6350 4950 6350
Wire Wire Line
	4950 6350 4950 4500
Wire Wire Line
	4950 4500 6250 4500
Connection ~ 5750 6350
Connection ~ 6250 4500
Wire Wire Line
	5750 5450 5900 5450
Wire Wire Line
	5900 5450 5900 5100
Wire Wire Line
	5900 5100 5800 5100
$Comp
L Connector_Generic:Conn_01x04 J15
U 1 1 600F65D0
P 9450 3900
F 0 "J15" H 9530 3892 50  0000 L CNN
F 1 "I2C" H 9530 3801 50  0000 L CNN
F 2 "Connector_Hirose:Hirose_DF13-04P-1.25DSA_1x04_P1.25mm_Vertical" H 9450 3900 50  0001 C CNN
F 3 "~" H 9450 3900 50  0001 C CNN
	1    9450 3900
	1    0    0    -1  
$EndComp
Text GLabel 8600 3900 0    50   BiDi ~ 0
MCU_I2C_SDA
Wire Wire Line
	9250 3900 9150 3900
Text GLabel 8600 4000 0    50   BiDi ~ 0
MCU_I2C_SCL
Wire Wire Line
	9250 4000 8900 4000
$Comp
L Device:R R77
U 1 1 60111A1C
P 8900 4350
F 0 "R77" H 8970 4396 50  0000 L CNN
F 1 "2k2" H 8970 4305 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8830 4350 50  0001 C CNN
F 3 "~" H 8900 4350 50  0001 C CNN
	1    8900 4350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R78
U 1 1 601122AB
P 9150 4350
F 0 "R78" H 9220 4396 50  0000 L CNN
F 1 "2k2" H 9220 4305 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9080 4350 50  0001 C CNN
F 3 "~" H 9150 4350 50  0001 C CNN
	1    9150 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	8900 4200 8900 4000
Connection ~ 8900 4000
Wire Wire Line
	8900 4000 8600 4000
Wire Wire Line
	9150 4200 9150 3900
Connection ~ 9150 3900
Wire Wire Line
	9150 3900 8600 3900
Wire Wire Line
	9150 4500 8900 4500
Text GLabel 8600 3800 0    50   BiDi ~ 0
+5v_FROM_MCU
Wire Wire Line
	8700 4500 8900 4500
Connection ~ 8900 4500
Wire Wire Line
	8700 4500 8700 3800
Wire Wire Line
	8700 3800 8600 3800
Wire Wire Line
	9250 3800 8700 3800
Connection ~ 8700 3800
Wire Wire Line
	6350 3450 6350 4400
Text GLabel 8600 4100 0    50   Input ~ 0
GND
Wire Wire Line
	9250 4100 8600 4100
$EndSCHEMATC
