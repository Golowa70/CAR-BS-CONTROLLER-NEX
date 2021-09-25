EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 8
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
L Car_bs_controller_nex_mega-rescue:MEGA_PRO_EMBED_CH340G___ATMEGA2560-MEGA_PRO_EMBED_CH340G___ATMEGA2560 U1
U 1 1 6002A64B
P 4950 3650
F 0 "U1" H 4950 6317 50  0000 C CNN
F 1 "MEGA_PRO_EMBED_CH340G___ATMEGA2560" H 4950 6226 50  0000 C CNN
F 2 "MODULE_MEGA_PRO_EMBED_CH340G_:_ATMEGA2560" H 4950 3650 50  0001 L BNN
F 3 "" H 4950 3650 50  0001 L BNN
F 4 "" H 4950 3650 50  0001 L BNN "MAXIMUM_PACKAGE_HIEGHT"
F 5 "12/May/2017" H 4950 3650 50  0001 L BNN "PARTREV"
F 6 "Manufacturer Recommendations" H 4950 3650 50  0001 L BNN "STANDARD"
F 7 "Robotdyn" H 4950 3650 50  0001 L BNN "MANUFACTURER"
	1    4950 3650
	1    0    0    -1  
$EndComp
$Comp
L Car_bs_controller_nex_mega-rescue:GND-power #PWR034
U 1 1 6005869D
P 5650 6200
F 0 "#PWR034" H 5650 5950 50  0001 C CNN
F 1 "GND" H 5655 6027 50  0000 C CNN
F 2 "" H 5650 6200 50  0001 C CNN
F 3 "" H 5650 6200 50  0001 C CNN
	1    5650 6200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 6200 5650 6050
Text GLabel 5650 1250 2    50   Input ~ 0
7.2V_stab
Text GLabel 6950 1350 2    50   Output ~ 0
+5v_FROM_MCU
Wire Wire Line
	6650 2650 5650 2650
Text GLabel 5700 1650 2    50   Input ~ 0
MCU_Uref_2.5v
Wire Wire Line
	5650 1650 5700 1650
Text GLabel 5750 1450 2    50   Output ~ 0
+3v3_from_MCU
NoConn ~ 5650 1750
NoConn ~ 5650 1850
Wire Wire Line
	4250 3050 3700 3050
Text GLabel 5900 5150 2    50   Input ~ 0
MCU_door_switch_input
Wire Wire Line
	5900 5150 5650 5150
Text GLabel 3650 5150 0    50   Input ~ 0
MCU_proximity_input
Wire Wire Line
	4250 5150 3650 5150
Text GLabel 5900 5250 2    50   Input ~ 0
MCU_ignition_switch_input
Wire Wire Line
	5900 5250 5650 5250
Text GLabel 3650 5250 0    50   Input ~ 0
MCU_low_washer_water_level_input
Wire Wire Line
	4250 5250 3650 5250
Text GLabel 5900 5350 2    50   Input ~ 0
MCU_supply_voltage_input
Wire Wire Line
	5900 5350 5650 5350
Wire Wire Line
	4250 2550 3700 2550
Wire Wire Line
	4250 4150 3600 4150
Wire Wire Line
	6150 4250 5650 4250
Wire Wire Line
	4250 4250 3600 4250
Text GLabel 3600 4250 0    50   Input ~ 0
MCU_encoder_CLK
Text GLabel 3600 4150 0    50   Input ~ 0
MCU_encoder_SW
Text GLabel 6150 4250 2    50   Input ~ 0
MCU_encoder_DT
Text GLabel 3700 2550 0    50   Input ~ 0
MCU_pjon_bus_PIN
Text GLabel 3700 3050 0    50   Input ~ 0
MCU_system_status_LED
Text GLabel 6650 2650 2    50   Input ~ 0
MCU_one_wire_PIN
NoConn ~ 4250 2150
NoConn ~ 4250 2050
NoConn ~ 4250 1950
NoConn ~ 4250 1850
Text GLabel 6650 3550 2    50   Output ~ 0
MCU_water_pump_out
Wire Wire Line
	6650 3550 5650 3550
Text GLabel 3300 3550 0    50   Output ~ 0
MCU_light_out
Wire Wire Line
	4250 3550 3300 3550
Text GLabel 6650 3650 2    50   Output ~ 0
MCU_converter_out
Wire Wire Line
	6650 3650 5650 3650
Text GLabel 3750 2750 0    50   Output ~ 0
MCU_5V_sensors_enable
Wire Wire Line
	4250 2750 3750 2750
Text GLabel 3750 2850 0    50   Output ~ 0
MCU_main_supply_enable
Wire Wire Line
	4250 2850 3750 2850
Text GLabel 6650 2550 2    50   Output ~ 0
MCU_buzzer_out
Wire Wire Line
	6650 2550 5650 2550
Wire Wire Line
	6650 2850 5650 2850
Text GLabel 6650 2750 2    50   Output ~ 0
MCU_rs485_de
Wire Wire Line
	6650 2750 5650 2750
NoConn ~ 4250 3150
NoConn ~ 5650 3150
NoConn ~ 5650 3050
NoConn ~ 4250 2950
Text GLabel 3700 2650 0    50   Input ~ 0
MCU_water_flow_sensor
Wire Wire Line
	4250 2650 3700 2650
Text GLabel 6600 2950 2    50   Input ~ 0
MCU_button_on_board
Wire Wire Line
	6600 2950 5650 2950
Text GLabel 3750 1650 0    50   Input ~ 0
MCU_rst
Wire Wire Line
	4250 1650 3750 1650
Text GLabel 3650 5350 0    50   Input ~ 0
MCU_sensors_voltage_test_input
Wire Wire Line
	4250 5350 3650 5350
Text GLabel 6650 3350 2    50   Output ~ 0
MCU_UART1_TX
Wire Wire Line
	6650 3350 5650 3350
Text GLabel 3700 3350 0    50   Input ~ 0
MCU_UART1_RX
Wire Wire Line
	4250 3350 3700 3350
Text GLabel 6650 3250 2    50   Output ~ 0
MCU_UART2_TX
Wire Wire Line
	6650 3250 5650 3250
Text GLabel 3700 3250 0    50   Input ~ 0
MCU_UART2_RX
Wire Wire Line
	4250 3250 3700 3250
Text GLabel 3700 4450 0    50   Input ~ 0
MCU_power_OK_from_ADM705
Wire Wire Line
	4250 4450 3700 4450
Text GLabel 6650 2850 2    50   Output ~ 0
MCU_wdt_reset_to_ADM705
Text GLabel 6200 4650 2    50   Output ~ 0
LCD_BLA
Wire Wire Line
	6200 4650 5650 4650
Text GLabel 3600 5050 0    50   Output ~ 0
LCD_DI_MCU_CS
Wire Wire Line
	3600 5050 4250 5050
Text GLabel 3600 4950 0    50   Output ~ 0
LCD_RW_MCU_MOSI
Wire Wire Line
	4250 4950 3600 4950
Text GLabel 5900 5050 2    50   Output ~ 0
LCD_E_MCU_SCLK
Wire Wire Line
	5900 5050 5650 5050
Text GLabel 3700 4550 0    50   Output ~ 0
LCD_RST
Wire Wire Line
	4250 4550 3700 4550
NoConn ~ 4250 4350
NoConn ~ 4250 3850
NoConn ~ 4250 3750
NoConn ~ 4250 3650
NoConn ~ 4250 4650
NoConn ~ 4250 4750
NoConn ~ 4250 4850
NoConn ~ 4250 5450
NoConn ~ 4250 5550
NoConn ~ 4250 5650
NoConn ~ 4250 5750
NoConn ~ 4250 5850
NoConn ~ 5650 5850
NoConn ~ 5650 5750
NoConn ~ 5650 5650
NoConn ~ 5650 5550
NoConn ~ 5650 4950
NoConn ~ 5650 4850
NoConn ~ 5650 4750
NoConn ~ 5650 4450
NoConn ~ 5650 4350
NoConn ~ 5650 3950
NoConn ~ 5650 3850
NoConn ~ 5650 3750
Text GLabel 6150 4550 2    50   Input ~ 0
MCU_spare_input_1
Wire Wire Line
	6150 4550 5650 4550
Wire Wire Line
	5750 1450 5650 1450
Text GLabel 5900 5450 2    50   Input ~ 0
MCU_resistive_sensor_0_190
Wire Wire Line
	5900 5450 5650 5450
Text GLabel 3600 3950 0    50   Output ~ 0
MCU_spare_out_1
Text GLabel 3600 4050 0    50   Output ~ 0
MCU_spare_out_3
Wire Wire Line
	4250 3950 3600 3950
Wire Wire Line
	4250 4050 3600 4050
Text GLabel 6150 4050 2    50   Output ~ 0
MCU_spare_out_2
Wire Wire Line
	6150 4050 5650 4050
Text GLabel 6150 4150 2    50   Output ~ 0
MCU_spare_out_4
Wire Wire Line
	6150 4150 5650 4150
Text GLabel 5650 3450 2    50   BiDi ~ 0
MCU_I2C_SDA
Text GLabel 4250 3450 0    50   BiDi ~ 0
MCU_I2C_SCL
Wire Wire Line
	5650 1350 6750 1350
$Comp
L Device:CP C20
U 1 1 6082B25A
P 6750 1650
F 0 "C20" H 6868 1696 50  0000 L CNN
F 1 "220mk 10v" H 6868 1605 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D6.3mm_P2.50mm" H 6788 1500 50  0001 C CNN
F 3 "~" H 6750 1650 50  0001 C CNN
	1    6750 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 1500 6750 1350
Connection ~ 6750 1350
Wire Wire Line
	6750 1350 6950 1350
$Comp
L power:GND #PWR035
U 1 1 6082CBB1
P 6750 1900
F 0 "#PWR035" H 6750 1650 50  0001 C CNN
F 1 "GND" H 6755 1727 50  0000 C CNN
F 2 "" H 6750 1900 50  0001 C CNN
F 3 "" H 6750 1900 50  0001 C CNN
	1    6750 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 1900 6750 1800
$EndSCHEMATC
