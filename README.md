# Fall Detection Smart Bracelet Development  
## Division of Labor  
| Name      | Task                                      |  
|:--------- |:----------------------------------------- |  
| SharkY.Y | Algorithm design, debugging, testing, Line Bot setup |  
| ChenShenWei | Hardware assembly, 3D printing        |  
| ASRIEL | Documentation, record-keeping, testing personnel |  

## Motivation  
In today's aging society, the safety of elderly individuals has become a significant concern. Commercially available smartwatches are often expensive and include features that may not be useful for seniors. Therefore, I aim to create a fall detection smart bracelet specifically designed for the elderly. This bracelet will send notifications to family members when a fall is detected, allowing young people working away from home to stay informed about the safety of their elderly loved ones. This proactive approach can help prevent regrets caused by delayed responses.  

## Expected Benefits  
When a fall is detected, and the elderly person does not press the button within 10 seconds, the bracelet will automatically send a notification via Discord, requesting a callback to confirm their condition. This ensures timely assistance and helps determine whether an ambulance is needed.  

# Step-by-Step Diagram - Fall Detection System Design Process  
:::spoiler Design Process  
## **1. Requirement Analysis**  
   - Understand the application scenarios and goals of fall detection.  
   - Confirm the sensors to be used: accelerometers (acc), gyroscopes (gyro), etc.  
   - Define detectable types of falls: forward, lateral, backward, etc.  

## **2. Data Collection and Understanding**  
   - Extract training and testing data from the **MobiFall Dataset**:  
     - **Accelerometer**: Acceleration data on the x, y, z axes.  
     - **Gyroscope**: Angular velocity data on the x, y, z axes.  
     - **Activity type**: Labeled as ADL (Activities of Daily Living) or Falls.  
   - Analyze the differences between fall data and daily activity data (visualize data changes).  

## **3. Data Preprocessing**  
   - **Data Cleaning**:  
     - Remove noisy data.  
     - Correct outliers.  
   - **Smoothing**:  
     - Apply moving average or low-pass filters to reduce transient noise.  
   - **Normalization**:  
     - Convert data from different sensors into comparable ranges to reduce inconsistencies.  

## **4. Feature Extraction**  
   - **Peak Acceleration Analysis**:  
     - Calculate the composite acceleration on the x, y, z axes to check for sudden spikes.  
   - **Angular Velocity Change Analysis**:  
     - Examine angular velocity peaks and rapid changes, especially body rotations during falls.  
   - **Duration Analysis**:  
     - Ensure changes in acceleration and angular velocity are sustained, not transient noise.  

## **5. Fall Condition Settings**  
   - Set **acceleration threshold**: For example, trigger the first alarm when acceleration exceeds 12 m/s².  
   - Set **angular velocity threshold**: Trigger the second alarm when angular velocity exceeds 3 rad/s.  
   - Combine **duration threshold**: Confirm a fall event if changes exceed 1 second.  

## **6. Algorithm Design**  
   - Design a classifier (e.g., Decision Tree, SVM, KNN) based on extracted features to distinguish daily activities from falls.  
   - Detection Logic Workflow:  
     1. **Monitor changes in acceleration and angular velocity**.  
     2. **Check if changes meet the duration threshold**.  
     3. **Determine whether to trigger a fall alarm**.  

## **7. Model Training and Testing**  
   - Split data into training and testing sets for model training.  
   - Evaluate model accuracy using a **confusion matrix** to calculate precision, recall, and F1 scores.  
   - Fine-tune model parameters (e.g., thresholds and feature weights).  

## **8. Data Validation**  
   - Validate with real-world fall data from the **MobiFall Dataset**.  
   - Test if the model can correctly distinguish normal activities and falls.  
   - Adjust for false positives or missed detections.  

## **9. System Integration**  
   - Integrate the detection algorithm into the ESP32 and build the bracelet.  
   - Ensure real-time sensor data transmission for analysis.  

## **10. Alarm System Design**  
   - Design an alert system based on fall detection results:  
     - Emit sound or vibration alarms.  
     - Notify family members or medical personnel via Line Bot.  

## **11. System Testing and Iteration**  
   - Perform real-world tests to evaluate system performance under various conditions (e.g., different surfaces, actions).  
   - Optimize and improve based on test results.  
:::  



## Contributor
[ASRIEL](https://github.com/Asriel20080305)
[ChenShenWei](https://github.com/csw321214)
