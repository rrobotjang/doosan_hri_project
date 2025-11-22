# doosan_hri_project

컨테이너화 및 실행 순서
Docker Compose 설정: app.py 서버와 ROS 2 시스템을 별도의 Docker 컨테이너 또는 통합된 컨테이너 내에서 실행하도록 구성합니다.
ROS 2 시스템 실행: ros2 launch dsr_bringup integrated_system.launch.py (또는 해당 실행 명령어)를 실행하여 로봇 노드를 시작합니다.
API 서버 실행: python3 app.py를 실행하여 Flask 서버를 시작합니다. (모델 로드 시간이 소요됩니다.)
프론트엔드 접속: 웹 브라우저에서 http://<서버_IP>:5000에 접속하여 음성 명령을 내립니다.
통합 동작 확인: 명령이 성공적으로 처리되면, ROS 2 시스템의 integrated_robot_system 노드가 명령 JSON을 수신하고 로봇 동작을 시작합니다.
