Please include your answers to the questions below with your submission, entering into the space below each question
See [Mastering Markdown](https://guides.github.com/features/mastering-markdown/) for github markdown formatting if desired.

**1. How much current does the system draw (instantaneous measurement) when a single LED is on with the GPIO pin set to StrongAlternateStrong?**
   Answer: LED0 was used for the purpose of measuring current in this question. It is observed that the Instantaneous current when the **LED0 is ON is 5.37mA** and **4.74mA when the LED0 is OFF**. From these measurements we can infer that the current drawn by the LED0 is **0.63mA** over one on-off cycle.


**2. How much current does the system draw (instantaneous measurement) when a single LED is on with the GPIO pin set to WeakAlternateWeak?**
   Answer: LED0 was used for this question as well. It is observed that the Instantaneous current when the **LED0 is ON is 5.42mA** and **4.84mA when the LED0 is OFF**. Hence the current drawn by the LED0 is **0.58mA** over one on-off cycle.


**3. Is there a meaningful difference in current between the answers for question 1 and 2? Please explain your answer, 
referencing the [Mainboard Schematic](https://www.silabs.com/documents/public/schematic-files/WSTK-Main-BRD4001A-A01-schematic.pdf) and [AEM Accuracy](https://www.silabs.com/documents/login/user-guides/ug279-brd4104a-user-guide.pdf) section of the user's guide where appropriate. Extra credit is avilable for this question and depends on your answer.**
   Answer: The AEC has an accuracy of 0.1mA when measuring currents above 250uA and the accuracy increases to 0.1uA when measuring currents below 250uA. Keeping this in mind when we observe the difference between the above current values we observe:	
			difference in current drawn = (0.63 - 0.58)mA = 0.05mA. This value is lower than the resolution hence the this difference does not provide any meaningful information.
		Furthermore, when we look at the individual differences their values are 0.05 when LED is ON and 0.1 when the LED is OFF and these values are also approximately less than the resolution.

Conclusion these differences do not provide any meaningful information.	


**4. With the WeakAlternateWeak drive strength setting, what is the average current for 1 complete on-off cycle for 1 LED with an on-off duty cycle of 50% (approximately 1 sec on, 1 sec off)?**
   Answer: The average current drawn by LED0 is    
		Instantaneous power = (Instantaneous current * Voltage)  				
					  = (0.58mA * 3.33 V)  	
					  = 1.93mW  
		Average power 	  = (Instantaneous power * duty cycle)  
		  			  = (1.93 * 0.5)mW = 0.96mW  
		Average current 	  = (Average power / Voltage)  
					  = 0.96mW / 3.33V  
					  = **0.29mA**  


**5. With the WeakAlternateWeak drive strength setting, what is the average current for 1 complete on-off cycle for 2 LEDs (both on at the time same and both off at the same time) with an on-off duty cycle of 50% (approximately 1 sec on, 1 sec off)?**
   Answer: The Instantaneous current when both LED0 and LED1 are ON is as follows:  					
					  = (5.91 - 4.86)mA = 1.05mA  
		Instantaneous power = (Instantaneous current * Voltage)  				
					  = (1.05mA * 3.33 V)  
				        = 3.49mW  
		Average power 	  = (Instantaneous power * duty cycle)  
		  			  = (3.49 * 0.5)mW = 1.745mW  
		Average current 	  = (Average power / Voltage)  
					  = 1.745mW / 3.33V  
					  = **0.52mA**  


