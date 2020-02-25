2020-02-24, rb

To get more insight I decided to implement my own MMA8451Q library, which is more comprehensive and explanatory.

I have been merging several programming styles and methods from several different libraries.

An excellent library to start with is the MMA845XQ[.cpp, .h] from mbed! It is object oriented and widely uses enumumerated register descriptions which define types. This allows the compiler to check the functions' input parameter types.

Example:

The function prototype the include file (.h):

    enum RANGE {
        RANGE_2G = 0,  // The total range is +- 2G, resol. 1G / 4096 counts (0.25 mg)
        RANGE_4G = 1,  // The total range is +- 4G, resol. 1G / 2048 counts (0.49 mg)
        RANGE_8G = 2   // The total range is +- 8G, resol. 1G / 1024 counts (0.98 mg)
    };

	uint8_t setCommonParameters(RANGE range, RESOLUTION resolution, LOW_NOISE lo_noise, DATA_RATE data_rate, OVERSAMPLE_MODE os_mode, HPF_MODE hpf_mode);

The function call in the main program (.ino):

	accel.setCommonParameters(accel.RANGE_2G, accel.RES_MAX, accel.LN_OFF, accel.DR_100, accel.OS_NORMAL, accel.HPF_OFF);


I am using the main MMA8451Q datasheet and a few application notes, mainly AN4070 for basic motion detection based on FF_MT_CFG and related registers.

It turns out that the Freefall-Motion method IS PROBABLY NOT THE BEST for our purpose. It uses fixed threshold values. Thus tilting the accelerometer can cause a permanent interrupt when the threshold is too low. It has to be low to be sensitive!

I will try to use the Transient method. It reveals a high pass filter to supress a constant gravitational acceleration bias.

 

2020-02-25, rb

Dear All,
I implemented my own library to learn more about the accelerometer. Especially the INTERRUPT mechanism I want to understand.
You will find two folders with two versions:V001: Motion Detection (FF_MT_CFG), similar to Hung's approach (have a look at the Excel sheet)V002: Transient Detection (TRANSIENT_CFG) with DC component removal by highpass filtering. This removes the const accel bias caused by to gravity.

To V001: 
The motion detection causes headaches because it takes the DC into account. When the system should wake up due to small accelerations (makes sense with only slightly oscillating light poles) the motion event accel threshold should be low. There is only one threshold for all axes! The z axis has to be deactivated, since it exceeds the threshold immediately due to gravitation. It can happen that an interrupt is fired just by rotating the vibration sensor such that the x or y axis is subjected to gravitation! We cannot know in advance how the sensor is mounted! It is the customer's choice. It is an unacceptable design if it will work only in very specific mounting conditions!

To V002: 
The transient detection also causes a headache. The DC is nicely removed, but the high-pass filter (HPF) characteristics is dependent on the output data rate ODR. The smallest cutoff frequency is 0.5 Hz. (0.25 Hz can be reached with ODR = 50 samples per sec. This is sampling rate too slow!). In high resolution with oversampling it can go up to 4 Hz! This means an oscillation has to be faster than 4 Hz to be detected! This is again an unacceptable limitation. A pole can be very high and slowly moving. 
Furthermore the accel thresholds are coarse. The accel threshold setting is independent of the chosen dynamic range (+-2g, +-4g, +-8g). It is always relative to 8g. The resolution is 7 bits only, i.e. 0...127 for 0 to 8g. The resolution (for 1 LSB) is 8g /128 = 0.0625g = 62.5 mg. With this limitation it is e.g. not possible to fire an interrupt caused by an acceleration below 62.5 mg. Additionally the HPF has to be considered, too.
We have to learn much more and search for alternatives.
The interrupt idea might(!) not work for mast excitation in normal wind conditions. We have to test it!
Rolf

2020-02-25, rb

ATTENTION!!!
1) INT2 and probably INT1 of MMA8451 are OPEN COLLECTOR! 
2) The interrupt signal is LOW ACTIVE! => use INPUT_PULLUP and trigger on FALLING!

