## Loosely Collected Notes I took as I learned circuits and designed this board

Servo Controller (ala Adafruit) 
	PCA9685
	tlc59116
	use a pi pico or similar, but more programming work
Servo Battery
	ETA9740, HM5907
	6V 4A regulated power supply to protect servo
		Boost converter or regulator (5V to 6V with a resistor)  (TI has a bunch)
	Include 5V power
	Includes 3.3 V power
Servo battery charger
	TPS61230 (5A)
	TPS61090 (2A)
	https://wiki.seeedstudio.com/Lipo-Rider-Plus/
	2S BQ25887 2S Balance charging IC
USB-c wiring:
	the correct pulldowns for CC1 and CC2 are 5K1, You do need separate pulldowns for each CC pin
	DP and DN should go straight to the data controller (potentially with ESD protection diodes).
	There should be no more than 10uF of capacitance on VBus, and you have to make sure your circuit draws no more than 100mA until:
    1. you negotiate more through power descriptors (up to 500mA)
    2. you see appropriate voltages on the CC lines
    3. you detect either a DCP or CDP using BCS
	https://dubiouscreations.com/2021/04/06/designing-with-usb-c-lessons-learned/
	It looks like I should have fed CC1 and CC2 to two adc pins to handle connector polarity.
	Basically, if you see >1.23V on either CC1 or CC2, you are all set and can draw 3A. If it's between 0.66V and 1.23V you can draw 1.5A. Put a filter on the CC lines because there will be some fluctuations (up to 10ms long) that you are supposed to ignore.
	You need to also verify the IC connected to the UDP pin has an internal pull up to 3.3V, or add an external resistance.
	Suggestion seemed to be capacitor placed as close as physically possible to power output pins.
	Shield should not be connected to the ground, but the chassis.
	USB 2.0-only SMT Korean Hroparts TYPE-C-31-M-12 (LCSC part number C165948)
	Pico USB_DM and USB_DP A 27Ω series termination resistor is required on each pin, but bus pullups and pulldowns are provided internally
Voltage Regulation
	USB SHIELD: 1MΩ resistor and 4.7 nF capacitor in parallel  OR (330 Ω resistor and a 0.1 µF capacitor in parallel)
	213716 Molex USB C
	AP2112  (input to 3.3V)  OR identical Torex XC6210B332MR-G

For 1S Lipo: MCP73871

MCP73213 or MCP73223 would require 2 LDO (one for USB one for battery)
JST-XH 2S
bq241xx  (rather complicated wiring)



Ultra Librarian or SNAPEDA

4.7K resistor on SDA and SCL for high speed operations  (2K fine, probably better if some are expected on upstream board)
Capacitors. Near each power line for IC.The value it is not very critical and usually it is at the range of 0.001μF to 4.7μF, but the combination of values helps to keep the impedance low and to avoid resonance spikes. 100nF and 10uF

Absolute Orientation Sensor
	BNO055 BNO08X (not in stock?)  (2.4 to 3.6V)
		0x29 (Com3 adjusts to 0x28)

qwiic connector  (JST SH 4-pin connector and will only use it for I2C)
	servo and absolute orientation sensor

Altimeter
	Bosch BMP390  (1.2 to 3.6V)
		SDO to GND results in slave address 1110110 (0x76); connection it to VDDIO results in slave address 1110111 (0x77)

https://www.nxp.com/design/designs/kv-series-quad-motor-control:KINETIS-DRONE-REFERENCE-DESIGN
https://www.nxp.com/design/designs/ai-robot-platform-based-on-i-mx-8m-plus:AI-ROBOT-PLATFORM
https://www.nxp.com/design/designs/px4-robotic-drone-vehicle-flight-management-unit-vmu-fmu-rddrone-fmuk66:RDDRONE-FMUK66
https://www.ti.com/solution/drone-propeller-esc?variantid=17140&subsystemid=17313
https://www.ti.com/applications/industrial/motor-drives/overview.html
https://www.ti.com/solution/drone-propeller-esc?variantid=17141&subsystemid=17228


N-channel Mosfet to PWM (terminal block)  (high current switch)
	D is the thing being used (Power source -> thing used -> Mosfet -> sink)
	Gate is the PWM signal (R1 1k, C1 100u, R2 10k)
	https://www.instructables.com/How-to-Control-a-MOSFET-With-Arduino-PWM/
	https://forum.arduino.cc/t/mosfet-control-with-pwm/912521/7
5 V Mosfet for powering on Vision board
Mosfet with no voltage (connecting RUN or GLOBAL_EN to ground)

Voltage of all on I2C ??
Addresses of all

Fiducial markers (for pick and place)

Include a charging LED (off when charged)

Wireless Receiver: SX1280
Add wifi ESP module

Battery linked to power switch
Microcontroller power on Vision Processor (what signal?) (GPIO pin 3, or serial message, RUN to ground)

RP2040 Chip
	USB_VDD should be decoupled with a 100nF capacitor close to the chip’s USB_VDD pin
	To reduce the number of external power supplies, ADC_AVDD can use from the same power source as the digital IO supply (IOVDD)
	ADC_AVDD should be decoupled with a 100nF capacitor close to the chip’s ADC_AVDD pin
	(all of the above and more to the same 3.3V)
	IOVDD should be decoupled with a 100nF capacitor close to each of the chip’s IOVDD pins.
	DVDD should be decoupled with a 100nF capacitor close to each of the chip’s DVDD pins.
	A 1μF capacitor should be connected between VREG_VIN and ground close to the chip’s VREG_VIN pin
	USB 2.0 compatible Full Speed device (12Mbps)

Mount points (for vibration dampening)

Microcontoller Connections:
	Two buttons (any pin, pwm)
		servo latch
	1 serial to MCU
		send data, receive updated target
	8 MB SPI FLASH (or larger) (QSPI NOR memory)
		Winbound W25Q16  W25Q16JVSSIM
	12 MHz crystal for perfect timing
	USB (same both for charging lipo and serial)
	BOOTSEL button while plugging it into USB (or pulling down the RUN/Reset pin to ground)

Pico Alternatives:
	STM32F779AIY6TR
	MSP432P411xT / MSP432P401xT
	nRF5340
	Kinetis® KV4x family - MCU for motor control
		MK20DX32VLH5 MKV42F128VL


Future Directions:
LoRa
DC input to Charge Lipo and to Power board (if connected)
	5V 2A switching regulator for robust power supply (8V -18V input voltage)


Suppliers:
	JLCPCB
	PCB Shopper
	PCBWay
	SeeedStudio
	Smart Prototyping
	PCB Shopper - see all

Reflow Oven:
	Breville Smart Oven
	T-962 https://www.aliexpress.com/item/2251832137689249.html?gatewayAdapt=4itemAdapt 
	ZH2520HL (needs 110V) https://zbdz.en.alibaba.com/product/62345546771-915115390/Smt_Reflow_Oven_For_IC_Chip_Mini_Reflow_Soldering_And_Welding_Oven.html?spm=a2700.shop_cp.16319.20.3e387e82t42EEZ 

BGA makes assembly much more expensive

Actual Part List:
	USB4085-GF-A
	JST B3B-XH-AM(LF)(SN)  (right angle S3B-XH-A(LF)(SN) out of stock)
	Resistors 0603 (larger allows routing UNDER)
		5.1K  - 2
		R0 (no resistance)

	BATT CHARGER 
reqs: 
	2S or 3S Lipo (3s preferred)
	VIN Min < 5V
	VIN Max > 20V
	Current > 5 Amps
options:
	MP2650/MP2760  (unavailable)
	MP2762A (2s only)

	MP2639AGR-Z (QFN)
	MP2672A or MP2639A (QFN) or MAX14748 (non stocked)0x0A, integrates CC negotiation) or MAX77962 (0xD2h/0xD3h) or TI bq25792 (QFN) 		
	bq24179 (not stock)
	bq25887 (best stock, psel for detection), bq25883, bq25882, bq25886 (~886 best but not stocking, detects BC1.2)
	ACT2861QI301 (good for 3 cell, QFN, available)
	BQ25792
	MAX77961 (3 cell, sys connection, QFN) (61 is 6A, 60 is 3A)

	STUSB4500 (WLSCP, no stock) or STUSB4500L (3A Max) or MAX77958 or TPS65987D or PTN5110 or UPD301C
		MAX77958 is good but not QFN
	TPS65987D (QFN) (and similar tps65983 but external switch for 5A)
	XPD319BP25 (36W)

	BHI260AP (LGA 44 pin, 1.8V)  BHI160 (LGA 24 pin, 1.6 to 3.6V	)
	BMP390/BMP384/BMP581 (LGA 10 pin, 1.6 to 3.6V)
	BMM150 (WLCSP-12, 1.6-3.6V)
	BNO086, FSP200 (similar or same as Bosch, low stock)
	ICM-42688-V(QFN package, no sensor fusion)
	ICM-20948 (no stock, QFN, sensor fusion included but questionable)
	ISM330DHCX + LIS3MDL (6+3 axis, LGA, industrial, low stock) - add magnetometer and rp2040 for Fusion calculation


	PWM Controller PCA9685 (), TLC59116F (VQFN or TSSOP), TLC5947 (RGB LED Drivers)TLC59711, TLC5940, TLC5946, TLC694x, TLC59711 
		TLC5971, LED6001, MAX16803, RT9288A, LT3595A and LT8500, tlc59282 (all at once driver)
		LT8500 and TLC5940 are confirmed motor control,
		Dedicated Pi Pico (16 bit) - https://www.waveshare.com/w/upload/0/06/Pico_Servo_Driver_Sch.pdf 
		STNRG388A



	MOSFET: PMPB10XNE PMPB11EN CSD18542KTT

	BUCK
		All in one: lm5127, ADP1034/ADP1031, ltc7811/7818/7817, 
		tps65310a (2A limit, no stock),
		tps65287 (low stock, uncertain voltage setting but likely 3 buck),
		MAX20028 (only one > 4V, moderate stock), 
		MPM54304 (non over 5.5V, LGA, moderate stock)
		NCV97200 (3.3 and 5V only, no stock)
		ADP5055 (high current, LGA, moderate stock, 18V max)
		MC34VR500 (QFN, moderate stock)
	Plain BUCK
		MAX17504 (3.5A wide voltage)
		MAX17541G (0.5A)
		LMR16006
		TPS65320D-Q1 (buck + loo)
		MP2223 (dual output buck)
	Tps54331, MAX20028, MPM54304, TPS54620 (6A, in stock)

	DC Motor Driver: TB6612, DRV8871, L293D, L9110H, MX1508 (make pi), DRV8840, DRV8313, UCC27200A
	L298N, L293D, SmartElex 15S, VNH2SP30, VNH3ASP30, TB6612FNG, TB67H450FNG, L6258, HR8825
	FROM 3D Printers: TMC2209, TMC2130, TMC2225
	https://learn.adafruit.com/assets/9536
	https://www.ti.com/lit/ug/tiducn7/tiducn7.pdf?ts=1659019146617&ref_url=https%253A%252F%252Fwww.ti.com%252Fsolution%252Fdrone-payload-control
	https://www.ti.com/tool/TIDA-00827#description 

https://www.mouser.com/manufacturer/amphenol-ltw/

JCLPCB In Stock:
	RP2040
	BMP388
	TMC2209
	BMM150
	BHI160B
	BQ7693003DBTR (Cell balancing)
	Bucks: TPS54331DR, MC34063ADR2G (1.5A), ICL7660AIBAZA-T
	LDO AMS1117-3.3 (3.3V fixed)
	Mosfets: AO3401A (max 12 V on gate, 4A), AO3400A (5.7A) P-channel,
	N-channel AO3402 (4A)
	USB PD-controller: XPD319 (DRP only), CYPD2120-24LQXIT
	PWM Controller: PCA9635, TLC5940 (no stock), TM1640, HT1621B
	Charger: MP26123DR
External:
	tps65987d
	TLC59401
	MAX77961EFV06+
	MAX17205


Used Datasheets:
y


Connectors:
neutrik
Phoenix Contact

Reqs:
47μF (1206 or 1210), 16V  GRM31CR61C476ME44, EMK325ABJ476MM8P
10 uF (0805), 16V  (Murata GRM32ER7YA106KA12)
10 uF (1210), 35V
220 nF   (0402), 6.3V
4.7 uF  (0402), 6.3V (10V on batt EVAL)
47 nF (0402) (25V on battery inrush) (Murata GRM31CR61C476ME44)
100 nF (0402), 10V  (>12V for batt)
22 uF (), 6V  GRM21BR61C226ME44 (overkill)
10 nF (25V on USB)
220 nF (on Inductor, low V, EVAL 0402)
22 pF (crystal, stable C0G class)


Mostly MLCC
X5R or X7R (especially on BATT charge and Converter for low ESR), although Class 1 is best
bigger size footprints generally are more stable with same temp coef
Anything on SYS, and BATT can be 16V MAX:   22UF (1210)
6V: 4.7uF (0603), 220 pF, 
33 pF (6V, Class 1)
47 uF (Sys, 2220)
VBUS, VD (PP_HV1) can be 30V MAX: 10 nF (0402), 1uF (0603), 10uF (1210), 47nF (0603)
Also 10uF 0603

3.3 uH  (16.3 mΩ resistance DC, 15 Amp, 10 Amp (+ 40C rise) (Pulse PA5007.332NLT)

0.01 Ω (1206), 3W
10 ohm (0402 on datasheet) 0.1W or higher
4.7 Ω (0402)
200K Ω (0402)

17800 (inlim)
86600 (iset)
178000 (ITO top off)
226000 (ITO top off)
178000 (set)
226000 (set 12.0 V)

Diodes
PMEG4050EP

LED: 19-217/GHC-YR1S2/3T 
Mosfet
10A DMN3016LFDE DMN3020

Gate to Source should be 12V
for 5V: 5A is needed
for Sys: 10 A is needed
for Batt: 30A is needed

NOTES:
	Don’t unplug under load, either battery or DC power
	GP and GPIO are used interchangeably
	Use the batt removal switch
	This switch can be removed if there are concerns about accidentally sliding it to removal as the default is to charge (or remove what is currently R43, the pull up to AVL)
	Using a thermosistor - the R(resistance at room temp) value must match the value of the resistor R50 (currently 10K). As setup, the pull down of ground, 110K on R41 will need to be removed for best accuracy, but should work with (reading a bit hot). Beware if that resistor is removed a THM to GND must be present. See datasheet for more info and example thermistor choices. Use a smaller Beta 10K NTC for a wider temperature cutoff.
	Remove 10K on GP15 and bridge
	Fuses can be removed to slightly lower quiescent current or diodes can be desoldered
	can place heatsinks on square test pads
	avoid placing directly on conductive sheets as it will short test pads
	can use the vbus header to provide DC in if no USB power or USB out
	batt should be connected, it acts as an extra source of power for spikes
	batt can be kept in protective sleeve, but default charge is low current
	could use the places for capacitors on batt out and sys as locations for terminal blocks (caps on batt should have connections for high-ish amps, others maybe not so high)
	no batt balancing as default charge is slow
	no battery gauge!
	three sets of sda and scl i2c pins, all with pull ups to +3V3
	PP_HV2 has 5V supply theoretically capable of supplying/sourcing devices over USB, but by default this is disabled and on power loss will reset to sink mode
	battery low voltage
	all power chips have built in thermal shutdowns and overcurrent/undervoltage protections
	could replace the CNFG Resistor (currently R43) from 8660 (3S) to 86k6 (2s)
	Use on lead acid: https://www.ti.com/lit/an/slua992/slua992.pdf
	2 issues with RP2040: no floating point unit, no code security (use another board if those are needed for that and use this as a relay)
	INOKB is attached to GPIO 16 of the RP2040. This means this can be used to detect the presence of DC power on CHGIN when this is pulled low (and is pulled high when disconnected). Note that due to the default pull down of the RP2040, the test point for this will always register ground, just ground high resistance (>200K) with no chargin, and to low resistance to ground with charging present
	solar panel goes to high voltage (shutting off input from usb) even when there is little current available. The charger can sometime deal with this by regulating system voltage, but only if it has a battery connected for backup power.
	set i2c lines high to reduce current loss slightly (as they have 3.3V pull up resistors)
	set 5v logic pin high to reduce current loss
	swap current limiting resistor on 5V GP15 to get a high power data line (it’s bigger than its neighbors, an 0603 rather than 0402)
	Note the sum of current limiting resistors, if all were on, would still exceed the total current producible by the RP2040
	there is a lot of capacitance on this board, expect it to take a while (seconds) to fully turn off
	A high quality 3S lipo can theoretically produce something like 500 Amps, that is power that can be quite dangerous
	MOSFET max on DMP3007SCG is 50A continuous current and TPH3R704PL is 90A, Mini fuses are available up to 30A and the fuse block holder is rated to 20 Amps @ 500V AC, with all four layers (2x power, 2x ground) exclusively with power planes to hold the current on that corner of the PCB
	What is a safe temperature? In general components here have ratings provided for 70C with max ratings usually being higher. So that should be a guidance for what a safe ‘hot’ limit is. Lower, however, is definitely preferred as hot temperatures will likely result in a much short product lifespan
	The MPM will get less hot in 2S lipo mode (lower input voltage)
	Use temperature sensor to shut off charging if too hot
	can use a solar panel as dc input, keep in mind that panels rated voltage is often exceeded at open circuit, so 12V safest. Same with batteries, a ‘24V’ battery often has a nominal voltage as high as 30V
	So how much above 24V is a problem? See if the TVS diode near the barrel jack starts to get hot. TVS can repeated handle current in small amounts, but heat can damage it over time.
	square test points for heat sinks as needed
	Remove the voltage sensing resistor next to INA219 to further reduce power loss
	having inline resistors or PTC fuses in power going off board is a good idea, especially on the 3V3 line which controls the MCU
	LEDs are of varying brightness, batt mosfet is very bright (warning) while vsys mosfet is very dim
	Zeners may slightly alter the voltage and therefore ADC usage of GP28 and GP29
	The diode by the Vsys MOSFET could be removed for adding items there if desired
	Note on the two plated through holes, which should be preferred for connecting as they are more resilient. One connected to USB shield with option to connect to ground, the other connected to ground with 1M
	i2c line 1 (sensors) doesn’t have full ESD protection, although i2c0 does have 5V tvs (power ic qwiic)
	an LED and resistor (0805) may be placed on the bottom near DC in if a connection indicator is desired, or to pull down open circuit of a solar panel
	Place a diode (currently D15) between VBAT and VSyS to increase the possible current maximum
	Careful about using metal screws on the mounting holes as there are traces and planes fairly close to some of them. If screws are connecting to another board or metal enclosure, something conductive, keep an eye on them.
	Thinking about the supercap is that in the event of a power loss (probably a thermal overheat or overcurrent protection engagement) the MCU could continue to function (check for loss of connection to 5V powered INA219 for power loss)
	Theoretically MAX can be used to send reverse 5V OTG, if DC is not connected.
	Battery helps absorb inductive loads
Low Battery:
	Remove Diode and let Sys shut down automatically, adjust Sys min
	Set UVLO on MPM to 8.5 volts, and/or turn off 5V bucks (en1/2) but 
Note about Zeners and how they don’t hold exactly a stable 3V3 like you might hope. Yes, you can safely remove them (carefully, of course).
keep 3V3
	could pass 12V directly into VSYS (or indeed 5V to 16V)
	Stop performing actions on all outputs
	Use a timer since chg-in last connected
	DC in barrel has a TVS with a minimum possible breakdown of 24.4V
Possible substitutions:
	MAX 77960 (3A max charge in, 6A max sys)
	MPM54304GMN -XXX other suffixes (5V may be 2 separate nets, 3A max each)
	TPS25750 (would require some rework)
	Different RP2040 flash memory sizes
	Different switches, diodes, mosfets, inductor
	Any BMP38X, BMP581
Not populated if no equivalents available (all should be hand solderable):
	Fuse blocks
	SD card mount
	JST connectors
	Mounting holes changed
Pair with a ATECC608 breakout for code security, or use a code security feature on the Xiao/QTpy breakout spot

0x23 =TPS probably
0x3C = display, if connected
0x40 = INA219
0x68 = MPM
0x68 = RTC
0x69 = MAX probably

0x10 = BMM150
0x68 = RTC
0x68/Ox69 = BMI270
Ox77/Ox76 = BMP390



Uses:
	Uninterrupted backup power supply
	Boost power supply (where occasional spikes above USB power capability, legacy won’t cut it)
	Robots and automation of all sorts (ideal as sensors, unified power and mcu)

21.4 47.1
STEPPER BOARD
2 wire 16AWG standard cable
	either a AMP Mate-n-Lok 1-480424-0 power connector
	or PCIE or ATX 12V-only
	JST VH 4 pin (5v red, gnd, gnd, 12v yellow)
	molex sata 675820000
8 pin connector 
3x 5 screw terminal
Diodes:
https://learn.watterott.com/silentstepstick/protector/
https://www.autodesk.com/products/fusion-360/blog/trace-width/

Lessons:
	Use planes more than wires (although wires override planes and can help manage lane prioritization)
	Use the grid
	Use ‘Vendor’ for JCLPCB number in Kicad to export BOM easily
	voltage rating on capacitors should be 2x average expected
	use lots of capacitors (except on very high speed lines)



i2c pull-ups (those for I2C1 are not shown in image but are present) are to 3V3 power external and not to LDO_3V3 (assume this doesn’t matter but perhaps they need to be in a valid state at boot?)
LDO 1V8 has higher capacitance than recommend (max 6 uF, 10 uF used)
5V on PP_HV2
Pins 57, 58, and 59 for drain and grounds are labeled differently than data sheet (matching 59, 58, 57) but are correctly matched in layout

3V3 and 5VD come from a regulator and become available slightly after system receives power