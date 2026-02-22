# 자율주행차 센서 상태 모니터링 모듈
### Fail-Safe Monitoring Module for Autonomous Vehicles


### 📈 Project Metrics
![Visitors](https://visitor-badge.laobi.icu/badge?page_id=utsi09.autonomous_vehicle_monitor&color=blue)
![Stars](https://img.shields.io/github/stars/utsi09/autonomous_vehicle_monitor?style=flat-square&color=yellow)
![Forks](https://img.shields.io/github/forks/utsi09/autonomous_vehicle_monitor?style=flat-square&color=lightgrey)


본 프로젝트는 자율주행 시스템의 안전성 확보를 위한 실시간 센서 상태 모니터링 및 고장 판단 알고리즘을 구현합니다.
* This project implements real-time sensor status monitoring and fault diagnosis algorithms to ensure the safety of autonomous driving systems.

## 주요 구성 (Key Components)
* **센서 구성**: 라이다, 카메라, IMU, GPS
    * Sensor Configuration: LiDAR, Camera, IMU, and GPS
* **ROS 2 데이터 수집**: 센서 데이터 및 예상 경로 생성을 위한 차량 상태(Heading, Velocity) 구독
    * ROS 2 Data Acquisition: Subscribing to sensor data and vehicle state (heading, velocity) for trajectory prediction

---

## 핵심 기능 (Core Features)

### 1. 주행 중요 센서 동적 선택 (Dynamic Sensor Selection)
<img width="1895" height="1062" alt="Image" src="https://github.com/user-attachments/assets/b9a0ac44-75c4-4da0-b93d-1f5a647bb6ca" />

* **센서 FOV와 예상 경로 비교**: 실시간으로 예측된 주행 경로(kinematic bicycle model 기반)와 각 센서의 시야각을 분석하여, 현재 주행 상황에서 가장 핵심적인 역할을 수행하는 센서를 동적으로 선택합니다.
    * **정상 센서 (Green)**: 정상 동작 중인 센서
    * **중요 센서 (Orange)**: 주행 경로상 핵심 정보를 제공하는 우선순위 센서
    * **고장 센서 (Red)**: 결함이 감지된 센서

* **Path-based Sensor Selection**: Dynamically selects critical sensors by comparing the predicted vehicle trajectory(based on kinematic bicycle model) with each sensor's Field of View (FOV).
    * **Normal (Green)**: Sensors operating within normal parameters.
    * **Critical (Orange)**: Priority sensors providing essential data for the current path.
    * **Faulty (Red)**: Sensors where failures have been detected.

### 2. 비헤이비어 트리 기반 의사결정 (Decision Making via Behavior Tree)
<img width="1095" height="722" alt="Image" src="https://github.com/user-attachments/assets/b0e7ae72-5a8d-4aeb-9aec-52898cf62de9" />

* **Fail-Safe 전략 수립**: 시스템의 최종적인 판단 및 제어 전략은 비헤이비어 트리(Behavior Tree) 아키텍처를 통해 결정됩니다.
    * The final fail-safe judgment and strategy are determined via Behavior Tree.

### 3. ISO 26262 기능 안전 규격 준수 (Functional Safety)
* **FTTI 기반 고장 진단**: ISO 26262 표준에 따른 정적 및 동적 FTTI(Fault Tolerant Time Interval)를 생성하여 고장 판단에 활용합니다.
    * Utilizing static and dynamic FTTI for fault diagnosis in accordance with ISO 26262 standards.

---
**Researcher**: Eum Taewook (SKKU CNR Lab)
