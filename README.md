# Adafruit-CLUE-Compass
Tilt-compensated compass for the Adafruit CLUE implemented in CircuitPython

# Description
A simple tilt-compensated compass for the [Adafruit Clue](https://www.adafruit.com/product/4500), implementated in CircuitPython. 

# Usage

Note that I used the CircuitPython 6.x libraries for the CLUE.  While I've not checked, I suspect the application requires 6.x or above. To install, copy the contents of the fonts/ directory into a directory of the same name in the root of your CIRCUITPY volume and then copy clue-compass.py to code.py in the root of that volume.  

The first time you run the program the compass won't be properly calibrated and the accuracy can be poor. Instead of using a distinct calibration phase or step, I just calibrate on-the-fly.  Articulate the compass in 3 dimensions for a minute or two.  The accuracy should improve markedly.  Once you're satisfied, click the *left* button to save the calibration data into the CLUE's non-volatile memory.  If you're using the non-volatile memory for other storage, such as barometric calibration, this step will overwrite that data.  And equivalently, if other applications write into the non-volatile storage, they'll likely wipe out the compass calibration data.
See the CLUE [altimeter example code](https://github.com/adafruit/Adafruit_Learning_System_Guides/blob/master/CLUE_Altimeter/clue_altimeter.py) for another application that uses NVM.  

Subsequently, when you start the application, it will attempt to load the calibration data from the non-volatile memory.   If you don't like the current calibration, touch capacitive pad number 2.  The front white LEDs will blink briefly and you'll hear a short tone.  This resets the working calibration data in RAM.  As usual, move the compass around until you're satisfied, and then press the *left* button to save the working calibration data into the non-volatile memory.  

Calibration is important.  Without calibration you'll be disappointed in the accuracy.  Typically, you'll only need to calibrate and save once.  


# Remarks
* Tilt-compensated compass with a simple graphical interface
* Provides hard and soft iron calibration and correction (compensation) with storage of calibration data in the CLUE's non-volatile memory.  
* Makes use of the CLUE's on-board accelerometer and magnetometer.  The implementation doesn't currently provide full **sensor fusion**.  In particular we don't use any filters (Kalman or otherwise) or make use of the gyroscopes. 
* I've also tested the code with breakout IMUs connected over i2C STEMMA, such as the FXOS8700, ICM209248, and BNO085.  Those work fine but require minor adaptations to import additional libraries and deal with different axis orientations on those PCBs and chips.  
* At some point I'll likely switch the tilt-compensation computation from the current 3-vector form to quaternions.  
* Currently does not support explicit magnetic declination adjustments
