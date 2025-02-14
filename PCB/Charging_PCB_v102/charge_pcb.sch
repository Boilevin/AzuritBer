EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
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
L Device:Battery BT1
U 1 1 60702E26
P 650 2450
F 0 "BT1" H 758 2496 50  0000 L CNN
F 1 "Charge" H 758 2405 50  0000 L CNN
F 2 "Zimprich:Anschlussklemme_2P_RM5,08" V 650 2510 50  0001 C CNN
F 3 "~" V 650 2510 50  0001 C CNN
	1    650  2450
	1    0    0    -1  
$EndComp
$Comp
L charge_pcb-rescue:FUSE-Zimprich F1
U 1 1 60704371
P 1300 2250
F 0 "F1" H 1300 2490 50  0000 C CNN
F 1 "5A" H 1300 2399 50  0000 C CNN
F 2 "Zimprich:Fuseholder_Reichelt_PL112000" H 1300 2250 60  0001 C CNN
F 3 "" H 1300 2250 60  0000 C CNN
	1    1300 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	650  3700 650  2650
$Comp
L Device:R R1
U 1 1 6070A5C5
P 2210 2550
F 0 "R1" H 2280 2596 50  0000 L CNN
F 1 "100k" H 2280 2505 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 2140 2550 50  0001 C CNN
F 3 "~" H 2210 2550 50  0001 C CNN
	1    2210 2550
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 6070AC52
P 2210 3350
F 0 "R2" H 2280 3396 50  0000 L CNN
F 1 "10k" H 2280 3305 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 2140 3350 50  0001 C CNN
F 3 "~" H 2210 3350 50  0001 C CNN
	1    2210 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2210 2250 2210 2400
Wire Wire Line
	2210 3500 2210 3700
$Comp
L charge_pcb-rescue:BC548BTA-dk_Transistors-Bipolar-BJT-Single Q1
U 1 1 60711074
P 3400 2900
F 0 "Q1" H 3588 2953 60  0000 L CNN
F 1 "BC547C" H 3588 2847 60  0000 L CNN
F 2 "digikey-footprints:TO-92-3_Formed_Leads" H 3600 3100 60  0001 L CNN
F 3 "https://media.digikey.com/pdf/Data%20Sheets/ON%20Semiconductor%20PDFs/BC546-50.pdf" H 3600 3200 60  0001 L CNN
F 4 "BC548BTACT-ND" H 3600 3300 60  0001 L CNN "Digi-Key_PN"
F 5 "BC548BTA" H 3600 3400 60  0001 L CNN "MPN"
F 6 "Discrete Semiconductor Products" H 3600 3500 60  0001 L CNN "Category"
F 7 "Transistors - Bipolar (BJT) - Single" H 3600 3600 60  0001 L CNN "Family"
F 8 "https://media.digikey.com/pdf/Data%20Sheets/ON%20Semiconductor%20PDFs/BC546-50.pdf" H 3600 3700 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/on-semiconductor/BC548BTA/BC548BTACT-ND/4553029" H 3600 3800 60  0001 L CNN "DK_Detail_Page"
F 10 "TRANS NPN 30V 0.1A TO-92" H 3600 3900 60  0001 L CNN "Description"
F 11 "ON Semiconductor" H 3600 4000 60  0001 L CNN "Manufacturer"
F 12 "Active" H 3600 4100 60  0001 L CNN "Status"
	1    3400 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 3700 3500 3100
Wire Wire Line
	3500 2250 3500 2300
$Comp
L Device:R R4
U 1 1 60712C09
P 3500 2450
F 0 "R4" H 3570 2496 50  0000 L CNN
F 1 "4k7" H 3570 2405 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 3430 2450 50  0001 C CNN
F 3 "~" H 3500 2450 50  0001 C CNN
	1    3500 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 2600 3500 2650
$Comp
L Device:R R3
U 1 1 60713C5E
P 3050 2900
F 0 "R3" H 3120 2946 50  0000 L CNN
F 1 "4k7" H 3120 2855 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 2980 2900 50  0001 C CNN
F 3 "~" H 3050 2900 50  0001 C CNN
	1    3050 2900
	0    1    1    0   
$EndComp
Text GLabel 2520 2985 3    50   Input ~ 0
pinChargeEnable
Connection ~ 3500 2650
Wire Wire Line
	3500 2650 3500 2700
$Comp
L charge_pcb-rescue:ACS712-Lötpad_2,5mm U1
U 1 1 60718D60
P 5600 3150
F 0 "U1" V 5554 3378 50  0000 L CNN
F 1 "INA169" V 5645 3378 50  0000 L CNN
F 2 "Zimprich:INA169_mit_Pinnummern" H 5600 3150 60  0001 C CNN
F 3 "" H 5600 3150 60  0000 C CNN
	1    5600 3150
	0    1    1    0   
$EndComp
Connection ~ 3500 3700
Wire Wire Line
	4400 2750 4850 2750
Wire Wire Line
	5500 3700 5500 3550
Text GLabel 5600 3850 3    50   Output ~ 0
pinChargeCurrent
Wire Wire Line
	5600 3550 5600 3850
$Comp
L power:+3.3V #PWR03
U 1 1 6071F590
P 5950 3850
F 0 "#PWR03" H 5950 3700 50  0001 C CNN
F 1 "+3.3V" V 5965 3978 50  0000 L CNN
F 2 "" H 5950 3850 50  0001 C CNN
F 3 "" H 5950 3850 50  0001 C CNN
	1    5950 3850
	0    1    1    0   
$EndComp
Wire Wire Line
	5700 3550 5700 3850
Wire Wire Line
	5700 3850 5950 3850
$Comp
L charge_pcb-rescue:HEADER_2-Zimprich J3
U 1 1 607207B7
P 6450 1550
F 0 "J3" V 6495 1422 60  0000 R CNN
F 1 "Power Switch" V 6600 1850 60  0000 R CNN
F 2 "TerminalBlock:TerminalBlock_bornier-2_P5.08mm" H 6450 1550 60  0001 C CNN
F 3 "" H 6450 1550 60  0000 C CNN
	1    6450 1550
	0    -1   -1   0   
$EndComp
$Comp
L charge_pcb-rescue:HEADER_2-Zimprich J2
U 1 1 60721ABE
P 5850 1550
F 0 "J2" V 5895 1422 60  0000 R CNN
F 1 "Battery" V 6000 1700 60  0000 R CNN
F 2 "TerminalBlock:TerminalBlock_bornier-2_P5.08mm" H 5850 1550 60  0001 C CNN
F 3 "" H 5850 1550 60  0000 C CNN
	1    5850 1550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5700 2750 5700 2400
$Comp
L charge_pcb-rescue:FUSE-Zimprich F2
U 1 1 60732011
P 6500 1900
F 0 "F2" V 6400 1700 50  0000 L CNN
F 1 "10A" V 6500 1700 50  0000 L CNN
F 2 "Zimprich:Fuseholder_Reichelt_PL112000" H 6500 1900 60  0001 C CNN
F 3 "" H 6500 1900 60  0000 C CNN
	1    6500 1900
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR02
U 1 1 607340F0
P 5900 1650
F 0 "#PWR02" H 5900 1400 50  0001 C CNN
F 1 "GND" H 5905 1477 50  0000 C CNN
F 2 "" H 5900 1650 50  0001 C CNN
F 3 "" H 5900 1650 50  0001 C CNN
	1    5900 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR01
U 1 1 60734664
P 3500 3700
F 0 "#PWR01" H 3500 3450 50  0001 C CNN
F 1 "GND" H 3505 3527 50  0000 C CNN
F 2 "" H 3500 3700 50  0001 C CNN
F 3 "" H 3500 3700 50  0001 C CNN
	1    3500 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 3700 4850 3700
$Comp
L Diode:MBR745 D4
U 1 1 60735AE0
P 6550 3200
F 0 "D4" V 6504 3280 50  0000 L CNN
F 1 "MBR1045" V 6595 3280 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-2_Vertical" H 6550 3025 50  0001 C CNN
F 3 "http://www.onsemi.com/pub_link/Collateral/MBR735-D.PDF" H 6550 3200 50  0001 C CNN
	1    6550 3200
	0    1    1    0   
$EndComp
Wire Wire Line
	5200 3700 5200 4650
Wire Wire Line
	5200 4650 6550 4650
Wire Wire Line
	6550 4650 6550 3700
Connection ~ 5200 3700
Wire Wire Line
	5200 3700 5500 3700
$Comp
L Diode:1N4148 D3
U 1 1 60738085
P 4850 3150
F 0 "D3" V 4804 3230 50  0000 L CNN
F 1 "1N4148" V 4895 3230 50  0000 L CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 4850 2975 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/1N4148_1N4448.pdf" H 4850 3150 50  0001 C CNN
	1    4850 3150
	0    1    1    0   
$EndComp
Wire Wire Line
	4850 3000 4850 2750
Wire Wire Line
	4850 2750 5500 2750
Wire Wire Line
	4850 3300 4850 3700
Connection ~ 4850 3700
Wire Wire Line
	4850 3700 5200 3700
$Comp
L charge_pcb-rescue:BC548BTA-dk_Transistors-Bipolar-BJT-Single Q3
U 1 1 6073AD5E
P 7850 2850
F 0 "Q3" H 8038 2903 60  0000 L CNN
F 1 "BC547C" H 8038 2797 60  0000 L CNN
F 2 "digikey-footprints:TO-92-3_Formed_Leads" H 8050 3050 60  0001 L CNN
F 3 "https://media.digikey.com/pdf/Data%20Sheets/ON%20Semiconductor%20PDFs/BC546-50.pdf" H 8050 3150 60  0001 L CNN
F 4 "BC548BTACT-ND" H 8050 3250 60  0001 L CNN "Digi-Key_PN"
F 5 "BC548BTA" H 8050 3350 60  0001 L CNN "MPN"
F 6 "Discrete Semiconductor Products" H 8050 3450 60  0001 L CNN "Category"
F 7 "Transistors - Bipolar (BJT) - Single" H 8050 3550 60  0001 L CNN "Family"
F 8 "https://media.digikey.com/pdf/Data%20Sheets/ON%20Semiconductor%20PDFs/BC546-50.pdf" H 8050 3650 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/on-semiconductor/BC548BTA/BC548BTACT-ND/4553029" H 8050 3750 60  0001 L CNN "DK_Detail_Page"
F 10 "TRANS NPN 30V 0.1A TO-92" H 8050 3850 60  0001 L CNN "Description"
F 11 "ON Semiconductor" H 8050 3950 60  0001 L CNN "Manufacturer"
F 12 "Active" H 8050 4050 60  0001 L CNN "Status"
	1    7850 2850
	1    0    0    -1  
$EndComp
Text GLabel 7075 3020 3    50   Input ~ 0
pinBatterySwitch
$Comp
L Device:R R5
U 1 1 6073CDAC
P 7500 2850
F 0 "R5" V 7707 2850 50  0000 C CNN
F 1 "4k7" V 7616 2850 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 7430 2850 50  0001 C CNN
F 3 "~" H 7500 2850 50  0001 C CNN
	1    7500 2850
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R6
U 1 1 60740775
P 7950 2400
F 0 "R6" H 8020 2446 50  0000 L CNN
F 1 "4k7" H 8020 2355 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 7880 2400 50  0001 C CNN
F 3 "~" H 7950 2400 50  0001 C CNN
	1    7950 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 2250 7300 2250
Wire Wire Line
	6550 3700 7325 3700
Connection ~ 6550 3700
Wire Wire Line
	6550 3700 6550 3350
Wire Wire Line
	7950 2550 7950 2600
Connection ~ 7950 2600
Wire Wire Line
	7950 2600 7950 2650
$Comp
L charge_pcb-rescue:HEADER_2-Zimprich J5
U 1 1 60749513
P 11450 3000
F 0 "J5" V 11495 2872 60  0000 R CNN
F 1 "Motor_Drive" V 11600 3150 60  0000 R CNN
F 2 "TerminalBlock:TerminalBlock_bornier-2_P5.08mm" H 11450 3000 60  0001 C CNN
F 3 "" H 11450 3000 60  0000 C CNN
	1    11450 3000
	1    0    0    -1  
$EndComp
$Comp
L charge_pcb-rescue:HEADER_2-Zimprich J7
U 1 1 6074E736
P 10185 3000
F 0 "J7" V 10230 2872 60  0000 R CNN
F 1 "24V Out" V 10335 3150 60  0000 R CNN
F 2 "TerminalBlock:TerminalBlock_bornier-2_P5.08mm" H 10185 3000 60  0001 C CNN
F 3 "" H 10185 3000 60  0000 C CNN
	1    10185 3000
	1    0    0    -1  
$EndComp
$Comp
L charge_pcb-rescue:HEADER_2-Zimprich J6
U 1 1 6074E527
P 11000 3000
F 0 "J6" V 11045 2872 60  0000 R CNN
F 1 "Motor_Cutter" V 11150 3150 60  0000 R CNN
F 2 "TerminalBlock:TerminalBlock_bornier-2_P5.08mm" H 11000 3000 60  0001 C CNN
F 3 "" H 11000 3000 60  0000 C CNN
	1    11000 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	10085 3700 10085 3050
Connection ~ 7950 3700
Wire Wire Line
	10900 3050 10900 3700
Connection ~ 10900 3700
Wire Wire Line
	10900 3700 11350 3700
Wire Wire Line
	11350 3050 11350 3700
Wire Wire Line
	10085 2250 10085 2950
$Comp
L Device:R R7
U 1 1 6075C810
P 9250 2550
F 0 "R7" H 9320 2596 50  0000 L CNN
F 1 "100k" H 9320 2505 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 9180 2550 50  0001 C CNN
F 3 "~" H 9250 2550 50  0001 C CNN
	1    9250 2550
	1    0    0    -1  
$EndComp
$Comp
L Device:R R8
U 1 1 6075CAE4
P 9250 3350
F 0 "R8" H 9320 3396 50  0000 L CNN
F 1 "10k" H 9320 3305 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 9180 3350 50  0001 C CNN
F 3 "~" H 9250 3350 50  0001 C CNN
	1    9250 3350
	1    0    0    -1  
$EndComp
Text GLabel 9075 2970 3    50   Output ~ 0
pinBatteryVoltage
Wire Wire Line
	9250 2250 9250 2400
Wire Wire Line
	9250 2700 9250 2900
Wire Wire Line
	9250 3500 9250 3700
Connection ~ 9250 2900
Wire Wire Line
	9250 2900 9250 3200
$Comp
L Device:CP C1
U 1 1 607607AB
P 9515 3000
F 0 "C1" H 9633 3046 50  0000 L CNN
F 1 "100uF" H 9633 2955 50  0000 L CNN
F 2 "Zimprich:Elko_vert_11.2x6.3mm_RM2.5" H 9553 2850 50  0001 C CNN
F 3 "~" H 9515 3000 50  0001 C CNN
	1    9515 3000
	1    0    0    -1  
$EndComp
Connection ~ 9250 3700
Wire Wire Line
	9515 2850 9515 2250
Wire Wire Line
	9515 3150 9515 3700
$Comp
L Connector:Screw_Terminal_01x06 J1
U 1 1 607FC1A2
P 5995 5640
F 0 "J1" V 5867 5920 50  0000 L CNN
F 1 "PCB_1" V 5958 5920 50  0000 L CNN
F 2 "Connector_JST:JST_XH_B6B-XH-A_1x06_P2.50mm_Vertical" H 5995 5640 50  0001 C CNN
F 3 "~" H 5995 5640 50  0001 C CNN
	1    5995 5640
	1    0    0    -1  
$EndComp
Text GLabel 5795 5640 0    50   Output ~ 0
pinChargeVoltage
Text GLabel 5795 5540 0    50   Input ~ 0
pinBatterySwitch
Text GLabel 5795 5740 0    50   Output ~ 0
pinChargeCurrent
Text GLabel 5795 5840 0    50   Input ~ 0
pinChargeEnable
Text GLabel 5795 5440 0    50   Output ~ 0
pinBatteryVoltage
Wire Wire Line
	650  2250 1050 2250
$Comp
L charge_pcb-rescue:DIODE-Lötpad_2,5mm D5
U 1 1 608CEAE9
P 7500 2250
F 0 "D5" H 7500 2442 40  0000 C CNN
F 1 "SB1240" H 7500 2366 40  0000 C CNN
F 2 "Diode_THT:D_DO-41_SOD81_P10.16mm_Horizontal" H 7500 2250 60  0001 C CNN
F 3 "" H 7500 2250 60  0000 C CNN
	1    7500 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 2250 7950 2250
Wire Wire Line
	7050 2400 7050 2250
Wire Wire Line
	5800 1650 5800 1950
Wire Wire Line
	5700 2400 5800 2400
Wire Wire Line
	6400 1650 6400 1950
Wire Wire Line
	6400 1950 5800 1950
Connection ~ 5800 1950
Wire Wire Line
	5800 1950 5800 2400
Wire Wire Line
	6500 2150 6500 2400
Wire Wire Line
	6500 2400 7050 2400
Wire Wire Line
	6500 2400 6500 3050
Wire Wire Line
	6500 3050 6550 3050
Connection ~ 6500 2400
Text GLabel 2035 2965 3    50   Output ~ 0
pinChargeVoltage
$Comp
L Diode:MBR745 D1
U 1 1 60702097
P 1145 3060
F 0 "D1" V 1099 3140 50  0000 L CNN
F 1 "MBR1045" V 1190 3140 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-2_Vertical" H 1145 2885 50  0001 C CNN
F 3 "http://www.onsemi.com/pub_link/Collateral/MBR735-D.PDF" H 1145 3060 50  0001 C CNN
	1    1145 3060
	0    1    1    0   
$EndComp
$Comp
L charge_pcb-rescue:HEADER_2-Zimprich J4
U 1 1 609318F2
P 9855 3000
F 0 "J4" V 9900 2872 60  0000 R CNN
F 1 "24V Out" V 10005 3150 60  0000 R CNN
F 2 "TerminalBlock:TerminalBlock_bornier-2_P5.08mm" H 9855 3000 60  0001 C CNN
F 3 "" H 9855 3000 60  0000 C CNN
	1    9855 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	9755 3050 9755 3700
Wire Wire Line
	9755 2950 9755 2250
$Comp
L Device:R R9
U 1 1 60730CD6
P 1670 2740
F 0 "R9" H 1740 2786 50  0000 L CNN
F 1 "4.7kO" H 1740 2695 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 1600 2740 50  0001 C CNN
F 3 "~" H 1670 2740 50  0001 C CNN
	1    1670 2740
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D6
U 1 1 6073C265
P 1670 3040
F 0 "D6" V 1709 2922 50  0000 R CNN
F 1 "LED" V 1618 2922 50  0000 R CNN
F 2 "LED_THT:LED_D1.8mm_W3.3mm_H2.4mm" H 1670 3040 50  0001 C CNN
F 3 "~" H 1670 3040 50  0001 C CNN
	1    1670 3040
	0    -1   -1   0   
$EndComp
Connection ~ 3500 2250
Wire Wire Line
	3500 2250 4400 2250
Wire Wire Line
	9075 2900 9075 2970
Wire Wire Line
	9075 2900 9250 2900
Wire Wire Line
	7350 2850 7325 2850
Wire Wire Line
	7075 2850 7075 3020
$Comp
L Device:R R10
U 1 1 60AB9B49
P 7325 3235
F 0 "R10" V 7532 3235 50  0000 C CNN
F 1 "2k2" V 7441 3235 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 7255 3235 50  0001 C CNN
F 3 "~" H 7325 3235 50  0001 C CNN
	1    7325 3235
	1    0    0    -1  
$EndComp
Wire Wire Line
	7325 3385 7325 3700
Connection ~ 7325 3700
Wire Wire Line
	7325 3700 7950 3700
Wire Wire Line
	7325 3085 7325 2850
Connection ~ 7325 2850
Wire Wire Line
	7325 2850 7075 2850
Wire Wire Line
	8455 3125 7950 3125
Wire Wire Line
	7950 3050 7950 3125
Connection ~ 7950 3125
Wire Wire Line
	7950 3125 7950 3700
$Comp
L power:+3.3V #PWR04
U 1 1 60AA8912
P 5795 5940
F 0 "#PWR04" H 5795 5790 50  0001 C CNN
F 1 "+3.3V" V 5810 6068 50  0000 L CNN
F 2 "" H 5795 5940 50  0001 C CNN
F 3 "" H 5795 5940 50  0001 C CNN
	1    5795 5940
	0    -1   -1   0   
$EndComp
$Comp
L Connector:Screw_Terminal_01x02 J8
U 1 1 60AA2A4C
P 8555 3325
F 0 "J8" V 8427 3405 50  0000 L CNN
F 1 "Start" V 8518 3405 50  0000 L CNN
F 2 "Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical" H 8555 3325 50  0001 C CNN
F 3 "~" H 8555 3325 50  0001 C CNN
	1    8555 3325
	0    1    1    0   
$EndComp
Wire Wire Line
	2900 2250 3500 2250
$Comp
L charge_pcb-rescue:DIODE-Lötpad_2,5mm D8
U 1 1 60ABC1BF
P 10635 2250
F 0 "D8" H 10635 2442 40  0000 C CNN
F 1 "SB1240" H 10635 2366 40  0000 C CNN
F 2 "Diode_THT:D_DO-41_SOD81_P10.16mm_Horizontal" H 10635 2250 60  0001 C CNN
F 3 "" H 10635 2250 60  0000 C CNN
	1    10635 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	7950 2600 8555 2600
Connection ~ 9250 2250
Wire Wire Line
	8555 2600 8555 3125
$Comp
L Transistor_FET:IRF9540N Q4
U 1 1 60B1B32A
P 8775 2050
F 0 "Q4" H 8979 2096 50  0000 L CNN
F 1 "IRF9540N" H 8979 2005 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 8975 1975 50  0001 L CIN
F 3 "http://www.irf.com/product-info/datasheets/data/irf9540n.pdf" H 8775 2050 50  0001 L CNN
	1    8775 2050
	1    0    0    1   
$EndComp
Wire Wire Line
	8875 2250 9250 2250
Wire Wire Line
	8575 2050 8555 2050
Wire Wire Line
	8555 2050 8555 2600
Connection ~ 8555 2600
Wire Wire Line
	8875 1850 7950 1850
Wire Wire Line
	7950 1850 7950 2250
Connection ~ 7950 2250
Wire Wire Line
	11350 2950 11350 2250
Wire Wire Line
	10835 2250 10900 2250
Wire Wire Line
	10900 2950 10900 2250
Connection ~ 10900 2250
Wire Wire Line
	10900 2250 11350 2250
Wire Wire Line
	9250 3700 9515 3700
Wire Wire Line
	9250 2250 9515 2250
Connection ~ 9755 2250
Connection ~ 9755 3700
Wire Wire Line
	9755 2250 10085 2250
Wire Wire Line
	10445 3700 10445 3050
Wire Wire Line
	9755 3700 10085 3700
Connection ~ 9515 2250
Wire Wire Line
	9515 2250 9755 2250
Connection ~ 9515 3700
Wire Wire Line
	9515 3700 9755 3700
Connection ~ 10085 2250
Wire Wire Line
	10085 2250 10340 2250
Connection ~ 10085 3700
Wire Wire Line
	10085 3700 10445 3700
Connection ~ 10445 3700
Wire Wire Line
	10445 3700 10900 3700
$Comp
L charge_pcb-rescue:HEADER_2-Zimprich J9
U 1 1 60BDF668
P 10545 3000
F 0 "J9" V 10590 2872 60  0000 R CNN
F 1 "24V out" V 10695 3150 60  0000 R CNN
F 2 "TerminalBlock:TerminalBlock_bornier-2_P5.08mm" H 10545 3000 60  0001 C CNN
F 3 "" H 10545 3000 60  0000 C CNN
	1    10545 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	7950 3700 9250 3700
$Comp
L Device:R R11
U 1 1 60C5823D
P 2665 3050
F 0 "R11" V 2872 3050 50  0000 C CNN
F 1 "2k2" V 2781 3050 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 2595 3050 50  0001 C CNN
F 3 "~" H 2665 3050 50  0001 C CNN
	1    2665 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 2250 1670 2250
Wire Wire Line
	1550 2250 1550 2495
Wire Wire Line
	1550 2495 1145 2495
Wire Wire Line
	1145 2495 1145 2910
Connection ~ 1550 2250
Wire Wire Line
	1145 3210 1145 3700
Wire Wire Line
	650  3700 1145 3700
Connection ~ 1145 3700
Wire Wire Line
	1670 2590 1670 2250
Connection ~ 1670 2250
Wire Wire Line
	1670 2250 2210 2250
Wire Wire Line
	1670 3190 1670 3700
Wire Wire Line
	1145 3700 1670 3700
Connection ~ 1670 3700
Wire Wire Line
	1670 3700 2210 3700
Connection ~ 2210 2250
Wire Wire Line
	2210 2250 2500 2250
Connection ~ 2210 3700
Wire Wire Line
	2210 3700 2665 3700
Wire Wire Line
	2210 2700 2210 2850
Wire Wire Line
	2035 2965 2035 2850
Wire Wire Line
	2035 2850 2210 2850
Connection ~ 2210 2850
Wire Wire Line
	2210 2850 2210 3200
Wire Wire Line
	2520 2900 2520 2985
Connection ~ 2665 2900
Wire Wire Line
	2665 2900 2520 2900
Wire Wire Line
	2665 2900 2900 2900
Wire Wire Line
	2665 3200 2665 3700
Connection ~ 2665 3700
Wire Wire Line
	2665 3700 3500 3700
Wire Wire Line
	10445 2950 10445 2410
Wire Wire Line
	10445 2410 10340 2410
Wire Wire Line
	10340 2410 10340 2250
Connection ~ 10340 2250
Wire Wire Line
	10340 2250 10435 2250
$Comp
L Mechanical:MountingHole_Pad H2
U 1 1 6073E1F7
P 2600 5700
F 0 "H2" H 2700 5749 50  0000 L CNN
F 1 "MountingHole_Pad" H 2700 5658 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_DIN965_Pad_TopBottom" H 2600 5700 50  0001 C CNN
F 3 "~" H 2600 5700 50  0001 C CNN
	1    2600 5700
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H4
U 1 1 6073E5C5
P 2600 6650
F 0 "H4" H 2700 6699 50  0000 L CNN
F 1 "MountingHole_Pad" H 2700 6608 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_DIN965_Pad_TopBottom" H 2600 6650 50  0001 C CNN
F 3 "~" H 2600 6650 50  0001 C CNN
	1    2600 6650
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H3
U 1 1 6073E399
P 2600 6200
F 0 "H3" H 2700 6249 50  0000 L CNN
F 1 "MountingHole_Pad" H 2700 6158 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_DIN965_Pad_TopBottom" H 2600 6200 50  0001 C CNN
F 3 "~" H 2600 6200 50  0001 C CNN
	1    2600 6200
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H1
U 1 1 6073B4AE
P 2600 5200
F 0 "H1" H 2700 5249 50  0000 L CNN
F 1 "MountingHole_Pad" H 2700 5158 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_DIN965_Pad_TopBottom" H 2600 5200 50  0001 C CNN
F 3 "~" H 2600 5200 50  0001 C CNN
	1    2600 5200
	1    0    0    -1  
$EndComp
$Comp
L charge_pcb-rescue:GND-Lötpad_2,5mm #PWR0104
U 1 1 60746896
P 2600 6750
F 0 "#PWR0104" H 2600 6500 60  0001 C CNN
F 1 "GND" H 2605 6569 60  0000 C CNN
F 2 "" H 2600 6750 60  0000 C CNN
F 3 "" H 2600 6750 60  0000 C CNN
	1    2600 6750
	1    0    0    -1  
$EndComp
$Comp
L charge_pcb-rescue:GND-Lötpad_2,5mm #PWR0103
U 1 1 60746674
P 2600 6300
F 0 "#PWR0103" H 2600 6050 60  0001 C CNN
F 1 "GND" H 2605 6119 60  0000 C CNN
F 2 "" H 2600 6300 60  0000 C CNN
F 3 "" H 2600 6300 60  0000 C CNN
	1    2600 6300
	1    0    0    -1  
$EndComp
$Comp
L charge_pcb-rescue:GND-Lötpad_2,5mm #PWR0102
U 1 1 607463A7
P 2600 5800
F 0 "#PWR0102" H 2600 5550 60  0001 C CNN
F 1 "GND" H 2605 5619 60  0000 C CNN
F 2 "" H 2600 5800 60  0000 C CNN
F 3 "" H 2600 5800 60  0000 C CNN
	1    2600 5800
	1    0    0    -1  
$EndComp
$Comp
L charge_pcb-rescue:GND-Lötpad_2,5mm #PWR0101
U 1 1 6073E8B8
P 2600 5300
F 0 "#PWR0101" H 2600 5050 60  0001 C CNN
F 1 "GND" H 2605 5119 60  0000 C CNN
F 2 "" H 2600 5300 60  0000 C CNN
F 3 "" H 2600 5300 60  0000 C CNN
	1    2600 5300
	1    0    0    -1  
$EndComp
Connection ~ 4850 2750
Wire Wire Line
	4400 2250 4400 2350
Wire Wire Line
	3500 2650 4100 2650
$Comp
L Transistor_FET:IRF9540N Q2
U 1 1 60B519B7
P 4300 2550
F 0 "Q2" H 4504 2596 50  0000 L CNN
F 1 "IRF9540N" H 4504 2505 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 4500 2475 50  0001 L CIN
F 3 "http://www.irf.com/product-info/datasheets/data/irf9540n.pdf" H 4300 2550 50  0001 L CNN
	1    4300 2550
	1    0    0    1   
$EndComp
Wire Wire Line
	4100 2650 4100 2550
$Comp
L charge_pcb-rescue:DIODE-Lötpad_2,5mm D2
U 1 1 6071023D
P 2700 2250
F 0 "D2" H 2700 2442 40  0000 C CNN
F 1 "SB1240" H 2700 2366 40  0000 C CNN
F 2 "Diode_THT:D_DO-41_SOD81_P10.16mm_Horizontal" H 2700 2250 60  0001 C CNN
F 3 "" H 2700 2250 60  0000 C CNN
	1    2700 2250
	1    0    0    -1  
$EndComp
$EndSCHEMATC
