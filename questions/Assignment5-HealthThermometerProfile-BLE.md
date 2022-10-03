Please include your answers to the questions below with your submission, entering into the space below each question
See [Mastering Markdown](https://guides.github.com/features/mastering-markdown/) for github markdown formatting if desired.

*Be sure to take measurements with logging disabled to ensure your logging logic is not impacting current/time measurements.*

*Please include screenshots of the profiler window detailing each current measurement captured.  See the file Instructions to add screenshots in assignment.docx in the ECEN 5823 Student Public Folder.*

1. Provide screen shot verifying the Advertising period matches the values required for the assignment.
   <br>Screenshot:  250ms
   ![advertising_period](../Screenshots/Assignment_5/Adv_period.png)  

2. What is the average current between advertisements?
   Answer: 2.21uA
   <br>Screenshot:  
   ![avg_current_between_advertisements](../Screenshots/Assignment_5/Avg_cur_btw_adv.png)  

3. What is the peak current of an advertisement? 
   Answer: 4.18mA
   <br>Screenshot:  
   ![peak_current_of_advertisement](../Screenshots/Assignment_5/Adv_peak_cur.png)  

4. Provide screen shot showing the connection interval settings. Do they match the values you set in your slave(server) code or the master's(client) values?.
Answer: 33ms, No they don't match.
   <br>Screenshot: 
   ![connection_interval](../Screenshots/Assignment_5/con_inv_period.png)  

5. What is the average current between connection intervals?
   Answer: 1.49uA
   <br>Screenshot:  
   ![avg_current_between_connection_intervals](../Screenshots/Assignment_5/con_avg_curr.png)  

6. If possible, provide screen shot verifying the slave latency matches what was reported when you logged the values from event = gecko_evt_le_connection_parameters_id. 
   <br>Screenshot: The latency obtained from the gecko_evt_le_connection_parameters_id is 0. The following screenshot proves the same.
   ![slave_latency](../Screenshots/Assignment_5/slave_latency.png)  

7. What is the peak current of a data transmission when the phone is connected and placed next to the Blue Gecko? 
   Answer:9.56mA  
   <br>Screenshot: 
   ![peak_current_phone_next_to](../Screenshots/Assignment_5/data_peak_curr_n.png)  
   
8. What is the peak current of a data transmission when the phone is connected and placed approximately 20 feet away from the Blue Gecko? 
   Answer:18.13mA
   <br>Screenshot:  
   ![peak_current_phone_20ft_away](../Screenshots/Assignment_5/data_peak_curr_20.png)  
   