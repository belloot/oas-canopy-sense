# Mobile Baseline Test Plan (Field)

## Purpose
The purpose of this test is to see if the sensor system can accurately and reliably  
produce specific measurements at fixed, known heights above the canopy while moving.

## Test Description
The radar system will be mounted on the Amiga tractor. We'll set it at a fixed height  
(12, 16, 20 in.) above the general top canopy using a measuring tape. For optional  
visual observation, we will use a ruler or stick taped onto the side of the radar setup.  
The distance from the radar to the bottom of the ruler or stick should be approximately  
the same as the height we're aiming for (12, 16, 20 in.).

## Test Setup
### Location
In the field with thick, dense, level, uniform canopy. This doesn't have to be a full bed.  
A portion of a bed works too. 
Let's choose Bed 20 (7/30/26) on far strawberry field.

### Equipment
Radar system, laptop, Amiga tractor, measuring tape, hex key.

### Sensor Configuration
UPDATE WITH RADAR CONFIG WE'RE USING.

## Procedure
1. Mount radar system onto Amiga tractor's platform.
2. Using hex key, adjust the height of the sensor system until it is x inches above the general top canopy, where x is the distance we're testing.  
Use a measuring tape to confirm. Do not worry about what the radar is reading.
3. Optional: For visual observation, consider taping a stick/ruler on the side 
of the radar setup to establish a fixed length from sensor-canopy.
4. Start the radar system.
5. When ready, start the Python script (CSV logger)
6. Move Amiga forward, making sure to not scrape the beds (1.1 mph setting)
7. Keep moving for a set distance (determined by red flag on far right bed)
8. Optional: Move Amiga backwards to beginning while still logging (be careful of scraping bed)
9. Unplug the radar system, terminate the Python script.
10. This concludes one trial. Repeat from step 2 for the next trial.
Note: The target distances we're testing for are 12, 16, and 20 inches.

## Data Collection
We're using the Python script to log CSV data. The main thing we're looking at is  
the filtered distance in inches.
We have a jupyter notebook for field plotting and debugging.

## Expected Results
This test is successful if the filtered distance in inches remain reasonably close  
to the distance we're aiming for (12, 16, 20 in.). We can establish what "reasonably"  
means later when doing summary statistics on the CSV data.

If the test is successful, we have strong indication that the radar can accurately measure  
sensor-canopy distance while moving over thick, dense, level, uniform canopies. Depending on  
the percentage of no peaks, standard deviation, and mean absolute error, we can determine if  
the readings are reliable and the radar's consistency.

## Control
The control test will have the same setup as the field test above, but we will operate  
the Amiga on parking lot concrete. We will do trials for 12, 16, 20 inches  
as the target heights. The Amiga will move forward for 30 seconds at 1.1 mph, and CSV  
will be logged for each trial. This will then be compared with the field tests.

In addition, we will explore how calibration affects performance in the field.  
We will first call the radar's calibrate function in the parking lot and do the  
control test. Then, going into the field, we will not recalibrate it on  
startup. After collecting one complete round of field tests (multiple trials,  
various heights), we will repeat the same round in the field but with recalibration.