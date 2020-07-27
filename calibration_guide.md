# Calibration guide


The TriSonica default setting is to display:

S  00.26 D  080 U -00.23 V -00.04 W  00.12 T  23.58 H  36.37 P  857.30 PI  001.8 RO -000.9 MD  110 

where 

•	S is 3D wind speed
•	D is Horizontal Wind Direction
•	U, V, W are vectors
•	T is temperature
•	H is humidity
•	P is pressure
•	PI is pitch
•	RO is roll
•	MD is compass heading

and the units are m/s, hPa, Celsius, and Degrees, where appropriate. 

The default data output rate is 5 Hz at a baudrate of 115200. 


The following calibrations must be performed for each sensor:

1)	Compass (performed in the field)
2)	Tilt (performed in the lab)
3)	Temperature (performed in lab)


The calibrations are performed as follows:

Compass:

1)	Enter “compasscalibrate YES”  ( “…” signifies calibration)
2)	Rotate and tilt the unit in a three-dimensional figure-eight until calibration is finished
3)	Enter “nvwrite” into terminal to save calibration


Tilt:

1)	Place TSM on known level surface and enter “levelcalibrate YES” 
2)	Wait until calibration is finished
3)	Enter “nvwrite” into terminal to save calibration

Temperature

1)	Loosely wrap TSM in coat/towel to create a zero-wind environment
2)	Enter “calibrate <temp> [<rh>]” where temp is xx.x. in C and relative humidity is xx.x % (must know the calibration temp. If humidity is not supplied, then 50% is assumed.) 
3)	Wait until calibration is finished
4)	Enter “nvwrite” into terminal to save calibration



Below is a table of useful CLI commands:


help <command>	Display a list of CLI commands
exit	Leave the CLI and return to sampling
nvwrite	Write changes to non-volatile memory
baudrate [<baud>[now]]	Show/set the current baudrate
show [<parameter>]	Display parameters that can be shown; or show [] parameter
hide [<parameters>]	Display parameters that can be hidden; or hide [] parameter
display	Display 
(name/what is tagged/what the tag is/significant figures/enabled/units)
outputrate [value]	Set/show the data output rate of the sampled data (Hz)
units [<parameter>[units]]	Sets/display the unit value for all adjustable parameters
compasscalibrate Yes	Start a compass calibration cycle
levelcalibrate Yes	Start a level calibration cycle
calibrate <temp> [<rh>]	Start a temp calibration cycle
decimals [<param>]	Set the number of decimal places of a Display parameter or a Group of Parameters
tag[<param>]	Display/set the ID Tag of a Display Parameter or a Group of Parameters
untag[<param>]	Remove the ID Tag of a Display Parameter or a Group of Parameters











-----------------------------------------------------------------------------------------
|     Name |            Description |  Tagged |      Tag | Decimals | Enabled |  Units  |
-----------------------------------------------------------------------------------------
|    IDTag |                 ID Tag |   Yes   |          |          |   Yes   |         |
|        S |          Wind Speed 3D |   Yes   |        S |     2    |   Yes   | m/s     |
|      S2D |          Wind Speed 2D |   Yes   |       S2 |     2    |   Yes   | m/s     |
|        D |   Horiz Wind Direction |   Yes   |        D |     0    |   Yes   | Degrees |
|       DV |    Vert Wind Direction |   Yes   |       DV |     0    |   Yes   | Degrees |
|        U |               U Vector |   Yes   |        U |     3    |   Yes   | m/s     |
|        V |               V Vector |   Yes   |        V |     3    |   Yes   | m/s     |
|        W |               W Vector |   Yes   |        W |     3    |   Yes   | m/s     |
|        T |            Temperature |   Yes   |        T |     2    |   Yes   | C       |
|       Cs |         Speed of Sound |   Yes   |        C |     2    |   Yes   | m/s     |
|   RHTemp |         RH Temp Sensor |   Yes   |     RHST |     2    |   Yes   | C       |
|       RH |     RH Humidity Sensor |   Yes   |     RHSH |     2    |   Yes   | %       |
|        H |               Humidity |   Yes   |        H |     2    |   Yes   | %       |
|       DP |               DewPoint |   Yes   |       DP |     2    |   Yes   | C       |
|    PTemp |   Pressure Temp Sensor |   Yes   |      PST |     2    |   Yes   | C       |
|        P |        Pressure Sensor |   Yes   |        P |          |   Yes   | hPa     |
|  Density |            Air Density |   Yes   |       AD |          |   Yes   | kg/m^3  |
|   LevelX |                Level X |   Yes   |       AX |          |   Yes   |         |
|   LevelY |                Level Y |   Yes   |       AY |          |   Yes   |         |
|   LevelZ |                Level Z |   Yes   |       AZ |          |   Yes   |         |
|    Pitch |                  Pitch |   Yes   |       PI |     1    |   Yes   | Degrees |
|     Roll |                   Roll |   Yes   |       RO |     1    |   Yes   | Degrees |
|    CTemp |           Compass Temp |   Yes   |       MT |     1    |   Yes   | C       |
|     MagX |              Compass X |   Yes   |       MX |          |   Yes   |         |
|     MagY |              Compass Y |   Yes   |       MY |          |   Yes   |         |
|     MagZ |              Compass Z |   Yes   |       MZ |          |   Yes   |         |
|  Heading |        Compass Heading |   Yes   |       MD |     0    |   Yes   | Degrees |
| TrueHead |           True Heading |   Yes   |       TD |     0    |   Yes   | Degrees |
-----------------------------------------------------------------------------------------




Minicom guide:


1.	Install minicom (mac: http://macappstore.org/minicom/)
2.	Run “minicom -s”
    a.	Go to serial port setup
    b.	Press F (turns off hardware flow control)
    c.	Press A and enter the appropriate port (go to /dev, see which device appears and disappears when you plug/unplug device, mine looks like: tty.usbserial-D307LICF)
    d.	Press enter to return to the main window
    e.	Go to exit, press enter
3.	You should see data streaming
4.	To change settings, hit ctrl c, brings you to a “>” command prompt, run the following commands to save calibration:
    •	show all
    •	decimals U 3
    •	decimals W 3
    •	decimals V 3
    •	decimals S 3
    •	decimals S2D 3
    •	outputrate 20
    •	nvwrite

    •	calibrate temp rh
    •	levelcalibrate YES
    •	nvwrite

    •	compasscalibrate YES (rotate in figure 8)
    •	nvwrite




