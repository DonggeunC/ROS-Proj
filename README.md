# ROS-Proj

- 차선 인식 : HSV를 통하여 노란 차선과 하얀 차선을 구함. 구한 차선들에 ROI를 지정한 마스크를 씌우고 마스크 영역에서의 Canny Edge를 구함. 구한 차선의 Edge를 통해 Hough Line을 구하고 평균선을 찾음. 평균선의 vanishing point를 찾아 주행 영역과 주행방향을 구함
