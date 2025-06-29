<!--
# SLAM
SLAM(위치추정 및 공간지도생성) 기반 자율주행 로봇 시스템 구현

[link](https://www.notion.so/SLAM-1ee37748e6e780bb9662dc6e9c799230)
-->

# ArtGuard - 자율 로봇 기반 미술품 상태 점검 시스템

ROS2를 활용한 미술품 상태 점검 로봇 시스템 구현 프로젝트

## 프로젝트 개요

- **목표**: 박물관 작품 보존을 위해, 사람보다 효율적으로 변화 감지와 점검이 가능한 동적 장애물 회피 자율주행 로봇 개발
- **주요 기능**: 작품 위치 순찰, 이미지 촬영 및 이상 탐지, 관람객 회피 주행, 이상 탐지 시 UI 알림
- **사용 장비**: TurtleBot4(RPLIDAR A1M8, OAK-D Pro)
- **개발 환경**: Ubuntu 22.04, ROS2 Humble, RTX3060, Colab
- **주요 기술 스택**: ROS2, SLAM, Nav2, OpenCV, YOLO
- **기간**: 2025.05.09 ~ 2025.05.15

## 시연 영상

- 영상 링크: [Demo Video](https://youtu.be/oX_34NGaoFc)

<div align="center">
  
[Demo Video](https://github.com/user-attachments/assets/1fd10935-5177-4a5d-925a-c975939630fb)

</div>

## 플로우차트

<div align="center">
  
![지능-B 1주차 플로우차트](https://github.com/user-attachments/assets/9d84f606-79b5-47a0-b4e9-e80827d44ce8)

</div>

## 상세 설명

### 문제 정의

- 박물관 작품 보존을 위해, 사람보다 효율적으로 변화 감지와 점검이 가능한 동적 장애물 회피 자율주행 로봇이 필요하다.

### 해결 방안

- 로봇이 관람객과 충돌을 피하면서 지정된 경로를 따라 순찰하며 카메라로 작품의 상태를 촬영
- 사전에 저장된 이미지와 비교해 손상, 위치 변화 등의 이상 여부를 자동 감지
- 이상 감지 시 UI를 통해 즉시 경고 전송

### 주요 기능

- **작품 순찰 자율주행**: SLAM 기반 경로 생성 및 주행
- **정밀 이미지 촬영**: 작품 앞 정해진 거리·각도에서 이미지 캡처
- **비교 분석**: 기준 이미지와 비교해 손상/이동 여부 판단
- **이상 감지 시 UI 알림**: 작품 정보와 함께 경고 메시지 표시
- **동적 장애물 회피**: 관람객 감지 시 회피 주행 경로 재계산

## 프로젝트 기여자

- 김재권: kimjg1219@gmail.com
- 이호준: hojun7889@gmail.com
- 위석환: llaayy.kr@gmail.com
- 최초인: choinchoi0331@gmail.com

## 교육과정 및 참고자

### 교육과정

<div align="center">

| 주차 | 기간 | 구분 | 강의실 |
| --- | --- | --- | --- |
| <1주차> | 2025.05.09(금) ~ 2025.05.15(목) | 협동-3 | 별관 B-2호 |

| 차시 | 학습내용 | 실습항목 |
| --- | --- | --- |
| 1 | SLAM 및 ROS2 기본 이론 | 이론 교육, SLAM 맵 작성 실습 |
| 2~4 | TurtleBot4 기반 ROS2 Navigation | Nav2, 경로 생성 및 실행 |
| 5~6 | 영상처리 및 OpenCV 활용 | 영상 비교 알고리즘 |
| 7~8 | 프로젝트 개발 및 통합 | 로봇 순찰 및 이상 감지 시스템 구현 |
| 9 | 프로젝트 시연 및 발표 | 프로젝트 발표 |

</div>

### 참고 자료

- [Rokey-3-D-2/rokey_pjt](https://github.com/Rokey-3-D-2/rokey_pjt)
- [Doosan Robotics Manual](https://manual.doosanrobotics.com/ko/programming-manual/3.3.0/publish/)
