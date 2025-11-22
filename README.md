# doosan_hri_project

컨테이너화 및 실행 순서
Docker Compose 설정: app.py 서버와 ROS 2 시스템을 별도의 Docker 컨테이너 또는 통합된 컨테이너 내에서 실행하도록 구성합니다.
ROS 2 시스템 실행: ros2 launch dsr_bringup integrated_system.launch.py (또는 해당 실행 명령어)를 실행하여 로봇 노드를 시작합니다.
API 서버 실행: python3 app.py를 실행하여 Flask 서버를 시작합니다. (모델 로드 시간이 소요됩니다.)
프론트엔드 접속: 웹 브라우저에서 http://<서버_IP>:5000에 접속하여 음성 명령을 내립니다.
통합 동작 확인: 명령이 성공적으로 처리되면, ROS 2 시스템의 integrated_robot_system 노드가 명령 JSON을 수신하고 로봇 동작을 시작합니다.


##
Flask 대신 FastAPI를 사용하여 백엔드 API 서버를 구현하고, 이 서버와 ROS 2 시스템(워커)이 포함된 Docker Compose 파일을 작성하면 시스템 전체를 컨테이너화하여 쉽게 배포하고 관리할 수 있습니다.
시스템 구성 요소 요약
FastAPI 서버: 음성 데이터를 처리하고 ROS 2로 명령을 전달하는 역할 (이전 app.py 대체).
ROS 2 워커: 로봇 제어, 비전 처리, MoveIt! 실행을 담당하는 ROS 2 패키지 (integrated_robot_system.py).


###서버리스^^
Dockerfile: 두 시스템을 실행할 수 있는 통합된 환경 정의.
docker-compose.yml: 두 서비스를 오케스트레이션.
