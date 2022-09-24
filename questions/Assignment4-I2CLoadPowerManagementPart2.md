Please include your answers to the questions below with your submission, entering into the space below each question
See [Mastering Markdown](https://guides.github.com/features/mastering-markdown/) for github markdown formatting if desired.

*Be sure to take measurements with logging disabled to ensure your logging logic is not impacting current/time measurements.*

*Please include screenshots of the profiler window detailing each current measurement captured.  See the file Instructions to add screenshots in assignment.docx in the ECEN 5823 Student Public Folder.* 

1. What is the average current per period?
   Answer:***14.15uA***
   <br>Screenshot:  
   ![Avg_current_per_period](../Screenshots/Assignment_4/Avg_current_period.png)  

2. What is the average current when the Si7021 is Powered Off?
   Answer:***2.58uA***
   <br>Screenshot:  
   ![Avg_current_LPM_Off](../Screenshots/Assignment_4/Avg_current_off.png)  

3. What is the average current when the Si7021 is Powered On?
   Answer:
   <br>Screenshot:***205.81***  
   ![Avg_current_LPM_Off](../Screenshots/Assignment_4/Avg_current_on.png)  

4. How long is the Si7021 Powered On for 1 temperature reading?
   Answer:***97.50ms***
   <br>Screenshot:  
   ![duration_lpm_on](../Screenshots/Assignment_4/Avg_current_on.png)  

5. Compute what the total operating time of your design for assignment 4 would be in hours, assuming a 1000mAh battery power supply?
   Answer:(1000 * 1000)/14.15 Ah/A = ***70671 Hours***. Hence the total operating time of this design is 70671 Hours.
   
6. How has the power consumption performance of your design changed since the previous assignment?
   Answer: The power consumption has decreased considerably between Assignment 3 and 4. For instance
		The average power consumed is 494.08uW for Assignment 3 and 47.24uW for Assignment 4 which is approximately 10 times lesser.
		The reason for this difference in power consumption is mainly due to the non-blocking mechanism used in Assignment 4. When the 
		system is waiting for delay time, it is sleeping in EM3 rather than EM0 which was the case in the previous assignment which adds 
		to one of the major reasons of power consumption.
		The second reason is the reduced power consumption by letting the system to sleep in EM1 when waiting for the I2C transfers to happen.

   
7. Describe how you tested your code for EM1 during I2C transfers.
   Answer:  For testing the code in EM1, the energy profiler was majorly used. From the screenshot below we can see that the after the I2C transfer
		the system sleeps in EM1 and then tries to sleep for 11ms in EM3 and wakes upto EM0 for the I2C read.
 ![EM1_test](../Screenshots/Assignment_4/Em1.png) 

