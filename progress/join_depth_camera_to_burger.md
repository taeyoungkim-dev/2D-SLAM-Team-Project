### 2026.05.18 시작

#### 2d-slam taeyoung_explorer fail 영상 촬영

<video width="100%" height="auto" controls>
    <source src="https://github.com/user-attachments/assets/6c2ecfc2-dbb0-4788-b89d-950f6b7a8b7a" type="video/webm">
    Your browser does not support the video tag.
</video>

> **실험 환경 세팅:** TurtleBot3 Burger에 Intel RealSense Depth Camera를 장착하는 작업 진행 중 발생한 2D-SLAM 탐색 실패 기록 영상입니다.



#### 실험 환경 세팅을 위해 turtlebot3 burger에 real sense depth camera를 설치하는 작업

커스텀 한 파일
/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf
/opt/ros/humble/share/turtlebot3_description/urdf/turtlebot3_burger.urdf

두 파일을 수정한 새로운 custom_burger 패키지를 생성하여 로봇을 만들었음.

현재 프로젝트에서는 depth camera를 사용한 3d-slam이 필요하기 때문에

기존 2d-slam에서 사용했던 cartographer를 사용하기 힘들어짐

따라서 RTAB-Map를 사용하여 3d-slam을 진행해야 함.

