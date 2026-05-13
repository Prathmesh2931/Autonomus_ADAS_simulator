1. Camera sensor pose issue not showing correct raw image even with correct joint and link pose ?

2. Launch error not able to recognise install of world.sdf and model.sdf

3. Dealing with late response for controlling steering 

4. Some part of wheel is been grounded in sim looking to solve it ?

5. image view from the robots rgb camera if i want to control robot cmd vel such that it should be within white and yellow line so do i have to train some custom model for it and have a real time control of bot control 

6. If u look around just tell me like yellow is detect correction with mask but white is been faint out why can u give me reason for it 

7. Hough Line is working well for line segment detection according to slope but mainly detect the straight line for curve fitting i tried polynomial fit by getting endpoint but i guess they are eventually failing.

8. I will try another approach with taking input-> ROI -> perspective transform(Birds view )->Image threshold-> Histogram -> Sliding Window 

9. working with below pipeline :
                                Camera Image
                                ↓
                                Gaussian Blur
                                ↓
                                HSV Thresholding
                                ↓
                                Morphological Filtering
                                ↓
                                ROI Masking
                                ↓
                                Bird's Eye Transform
                                ↓
                                Histogram Peak Detection
                                ↓
                                Sliding Windows
                                ↓
                                Polynomial Curve Fit
                                ↓
                                Inverse Perspective Transform
                                ↓
                                Final Lane Overlay

10. There is one fault thing where lets suppose i am going too left on road then red curve is dominating but with that if it is not able to detect some of the yellow lines then probably blue line gets merge into red one which i dont want , in that case it should not go in that direction like we should conclude that it is going out of bounding road it need steer in the opposite direction.