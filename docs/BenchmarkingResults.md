# Benchmarking Odometry and Error Calculation in RRC Lab  

### Overview  
A 2x2 square was marked in the RRC lab to benchmark odometry and calculate errors. The benchmarking and error calculation process was performed twice: once using ROS1 and then with ROS2. The errors from both setups were compared.  

### Procedure  
1. **Initial Pose:** The wheelchair started at (0, 0).  
2. **Trajectory:**  
   - Moved straight for **2m**, then performed a **90° clockwise turn** at (2, 0).  
   - Continued straight for another **2m**, followed by a **90° clockwise turn** at (2, -2).  
   - Proceeded straight for another **2m**, making a **90° clockwise turn** at (0, -2).  
   - Finally, moved straight for **2m** to return to the initial pose (0, 0), completing the 2x2 square.  

### Odometry Readings  

#### ROS1  
The odometry readings recorded using the encoder for ROS1 were:  
- (0, 0) → 0°  
- (1.87, 0) → 0.002°  
- (1.87, 0) → 91.63°  
- (1.83, -1.94) → 90.81°  
- (1.83, -1.94) → 182°  
- (0.06, -1.88) → 182°  
- (0.07, -1.88) → 279°  
- (0.37, 0.07) → 280°  
- (0.37, 0.07) → 343°  

#### ROS2  
The odometry readings recorded using the encoder for ROS2 were:  
- (0, 0) → 0°  
- (2.08, 0) → 0°  
- (2.07, 0) → 87.56°  
- (2.13, -2.02) → 87.56°  
- (2.14, -2.02) → 175.37°  
- (0.20, -2.17) → 175.37°  
- (0.20, -2.18) → 89.18°  
- (0.21, 0.06) → 90°  
- (0.20, 0.06) → 4.6°  

### Error Analysis  

#### ROS1 Errors  
- (0, 0) → 0°  
- (0.13, 0) → 0.002°  
- (0.13, 0) → 1.63°  
- (0.17, 0.06) → 0.81°  
- (0.17, 0.06) → 2°  
- (0.06, 0.12) → 2°  
- (0.07, 0.12) → 9°  
- (0.37, 0.07) → 10°  
- (0.37, 0.07) → 17°  

#### ROS2 Errors  
- (0, 0) → 0°  
- (0.08, 0) → 0°  
- (0.07, 0) → 2.44°  
- (0.13, 0.02) → 2.44°  
- (0.14, 0.01) → 4.63°  
- (0.20, 0.17) → 4.63°  
- (0.20, 0.18) → 0.82°  
- (0.21, 0.06) → 0°  
- (0.20, 0.06) → 4.6°  

### Error Comparison Between ROS1 and ROS2  
- (0, 0) → 0°  
- (0.05, 0) → 0.002°  
- (0.06, 0) → 0.81°  
- (0.04, 0.04) → 1.63°  
- (0.03, 0.05) → 2.63°  
- (0.14, 0.05) → 2.63°  
- (0.13, 0.06) → 8.18°  
- (0.16, 0.01) → 10°  
- (0.17, 0.01) → 12.4°  
