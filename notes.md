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

11. System can break in below things :shadows, glare, overexposure, faded lanes, road reflections, nighttime, different asphalt textures

12. Histogram can fail at : intersections, lane splits, merges, sharp exits (Because of 2 vertical dominant line)

13. Can upgrade to Semantic segmentation.Example:ENet, SCNN, LaneNet, UltraFast Lane Detection

14. Next Upgrade is done with below : 1. TEMPORAL SMOOTHING , 2. LOOKAHEAD STEERING , 3. SOBEL + HSV COMBINED MASK , 4. DYNAMIC LANE WIDTH , 5. KALMAN FILTER  ,6. PURE PURSUIT / STANLEY ,7. SEMANTIC SEGMENTATION

15. Problem in my newer code base : confidence is always 90 for both lane detection but they were overlapped in that case too my concern is that if these is situation then whatever side car is sterring it need to also  look out for opposite direction in these case. 

16. While there is turn in that case polynomial curve will eventually tell these to us then why the speed of my car is not reduced. In the case if both curve are drastically fluctuating at that point bot should slow up dont u think so . There can be such more cases with we can make our algo robust 

17. Then why in the temporal sobel lookahead there is issue where if only edge is been detected then why vehicle is following that same edge technically it should steer away a bit and try to fit in the middle of both left and right edge and tell me if white lets suppose is blue then both of edge are pointing same curve then these is flaw of it dont u think. And I Dont get sudden fluctuation of curve edge how is it possible there is some point that we are missing check it . And if there are curve that are like so curvy then make confidense a bit less it will help me out like  50 to 60 .which will slow out speed in these case . 

18. Almost good one but while taking a right turn it is sterring left like little which lead to lane chnage and also part of vehicle then go  out of lane like it goes some what straight while taking turn and after that as it goes beyond lane then at that time only one white lane is detected . These again caught us in  problem which takes vehicle out of lane but rest of things works pretty well . 

19. Like Technically speaking it should never cross the lane while taking a turn that is causing me a problem , same thing not correct if only one lane is there should look for other one like either to its left or right how we will have a check of these LMK
