표준 ROS 2 Docker 설정 방식
1. 기본 이미지 선택
가장 표준적인 접근 방식은 Docker Hub에 있는 공식 ROS 2 이미지를 베이스로 사용하는 것입니다. Ubuntu 버전과 ROS 배포판에 맞게 선택합니다.
osrf/ros:humble-desktop (데스크톱 환경 포함)
osrf/ros:humble-ros-base (ROS 핵심 기능만 포함, 권장)
2. Multi-Stage Build 전략
개발 환경(빌드)과 런타임 환경을 분리하여 최종 실행 이미지를 가볍게 만듭니다.
builder Stage: 모든 코드와 라이브러리를 설치하고 컴파일합니다.
runtime Stage: 빌드된 결과물(.rosinstall, .repos, colcon build 결과물)만 복사하여 실행에 필요한 최소한의 환경만 구성합니다.
