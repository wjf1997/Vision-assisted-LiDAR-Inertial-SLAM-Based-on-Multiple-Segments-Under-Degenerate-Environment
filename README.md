
To mitigate the impact of geometric degradation environments on LiDAR SLAM, a vision-assisted method based on multiple segments has been designed.
The key technology lies in two aspects, the former is the buffer mechanism for degenerate detection and a pose graph that merges segments by considering the degenerate segment as an individual fixed pose.
The latter is the integration of visual projection residuals and LiDAR odometry through dual sliding windows within degenerate segment.
This repository contains core code for buffer mechanism and dual sliding window. The full project will be released upon the acceptance of mansucript.
We have tested our project in [TartanAir, SubT-MARS](https://superodometry.com/iccv23_challenge_LiI) dataset, [Hilti](https://hilti-challenge.com/dataset-2022.html) dataset and self-made UAV dataset.
![3](https://github.com/user-attachments/assets/4b71c7f6-b034-43da-b2eb-95b60cfe6bd9)

![2](https://github.com/user-attachments/assets/14da6f90-6b5a-4ff9-8610-00c72e010047)

# Dependency
Our project is developed based on [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM). Therefore, the dependencies of our method are consistent with it, and there are no additional dependency requirements.







