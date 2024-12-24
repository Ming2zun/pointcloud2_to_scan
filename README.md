# pointcloud2_to_scan
A py node of ros2 realizes the conversion from point cloud to laser
1.Accept /points_raw topic publishing /scan topic.
2.What you need to pay attention to is the 27th line code of point_to_scan.py, and the value of scan _ msg.angle _ increment = 0.0266666666666667 depends on the actual situation of your 3D laser. For example, if your laser has 360 lines in the horizontal direction, the scanning angle will be π/360.
3.As described in urdf about gazebo lidar below, I chose 120 lines in the horizontal direction and 16 lines in the vertical direction. So the value is π/120= 0.0266666666666667.
<scan>
            <horizontal>
              <samples>120</samples>
              <resolution>1.000000</resolution>
              <min_angle>0.000000</min_angle>
              <max_angle>6.280000</max_angle>
            </horizontal>
            <vertical>
              <samples>16</samples>
              <resolution>1.000000</resolution>
              <min_angle>0.000000</min_angle>
              <max_angle>0.400000</max_angle>
            </vertical>
          </scan>
    ![image](https://github.com/user-attachments/assets/d6e475f0-7ef8-47ff-a105-1f40ae784df9)
    4.If you have any questions, please contact clibang2022@163.com
