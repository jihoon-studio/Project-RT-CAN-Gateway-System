# RT-CAN Gateway System
**FreeRTOS 기반 CAN 네트워크 제어 및 임베디드 게이트웨이 프로젝트**

> **STM32 + FreeRTOS 환경에서 다수의 임베디드 노드가 생성하는 CAN 데이터를
중앙 게이트웨이에서 제어·통합하여 이더넷 기반 상위 시스템으로 전달하는 RTOS 기반 네트워크 프로젝트**

---

## 프로젝트 개요

RT-CAN Gateway System은
여러 임베디드 보드에서 발생하는 실시간 센서 데이터를 CAN Bus로 수집하고,
중앙 제어 보드에서 이를 통합·가공하고 데이터 흐름을 제어한 뒤 이더넷 네트워크로 전달하는 시스템입니다.

이 프로젝트의 핵심 목표는 다음과 같습니다.

- FreeRTOS 기반 멀티 태스크 구조 설계
- CAN 통신 + RTOS 이벤트 기반 동기화
- 임베디드 시스템에서 네트워크 계층 개념 적용
- 실시간성과 안정성을 고려한 게이트웨이 아키텍처 구현

---

## 시스템 아키텍처

**분산 제어 네트워크**

<img width="761" height="573" alt="image" src="https://github.com/user-attachments/assets/15ef9630-0f89-49f8-8daa-2f088613e7ea" />

- **Board A / B**

  - 센서 데이터 생성
  - CAN 메시지 송신
  
- **Board C (Gateway)**

  - CAN 메시지 수신 및 ID 기반 필터링
  - 메시지 유형에 따른 데이터 분기 처리
  - RTOS 기반 제어 흐름 관리
  - Ethernet을 통한 상위 시스템 연동

- **Raspberry Pi**
  - UDP 수신
  - 최종 데이터 출력

---

## 담당 역할

- FreeRTOS 기반 태스크 구조 설계
- CAN 통신 송·수신 로직 구현
- CAN 메시지 프레임 패킹 구조 설계
- RTOS 동기화 메커니즘 (Semaphore / Event) 적용
- 시스템 전체 데이터 흐름 정리 및 문서화

---

## 수행 목표

- RTOS 환경에서 CAN 통신을 안정적으로 처리
- 멀티 태스크 환경에서 데이터 충돌 없이 통신
- 임베디드 시스템에 네트워크 계층 개념 적용
- 확장 가능한 Gateway 구조 설계

---

## 사용 기술

### Embedded / RTOS
![STM32](https://img.shields.io/badge/STM32-Nucleo--F429ZI-blue?style=plastic&logo)
![FreeRTOS](https://img.shields.io/badge/FreeRTOS-RTOS-green?style=plastic&logo)
![CMSIS](https://img.shields.io/badge/CMSIS-RTOS-lightgrey?style=plastic&logo)
![HAL](https://img.shields.io/badge/STM32-HAL-orange?style=plastic&logo)

### Communication
![CAN](https://img.shields.io/badge/CAN-Bus-red?style=plastic&logo)
![CAN Filter](https://img.shields.io/badge/CAN-Filter_Implemented-critical?style=plastic&logo)
![Frame](https://img.shields.io/badge/Data-Frame_Packing-yellow?style=plastic&logo)

### Toolchain
![CubeIDE](https://img.shields.io/badge/STM32-CubeIDE-blueviolet?style=plastic&logo)
![CubeMX](https://img.shields.io/badge/STM32-CubeMX_.ioc-informational?style=plastic&logo)

---

## 주요 구현 내용

1️⃣ FreeRTOS 태스크 설계

- CAN 송신 Task
- CAN 수신 Task
- 데이터 처리 Task
- 통신 이벤트 기반 Task 동기화

👉 Event / Semaphore를 활용해
CAN 수신 시 대기 중인 Task를 깨우는 구조로 설계

2️⃣ CAN 통신 구현

- HAL CAN Driver 기반 송·수신
- CAN Filter 직접 설정
- 메시지 ID 기반 데이터 분기 처리
- 프레임 패킹을 통한 데이터 구조화

```c
CAN_TxHeaderTypeDef TxHeader;
TxHeader.StdId = 0x201;
TxHeader.DLC = 8;
TxHeader.IDE = CAN_ID_STD;
TxHeader.RTR = CAN_RTR_DATA;
```

3️⃣ RTOS + CAN 동기화 구조

- CAN Interrupt 발생
- Event / Semaphore로 Task 깨움
- Queue를 통해 데이터 전달
- Task 간 자원 충돌 방지

👉 RTOS 환경에서 실시간성 + 안정성 확보

4️⃣ 네트워크 개념 적용

- Board C를 Gateway 노드로 설정
- CAN → Ethernet 방향의 데이터 흐름 설계
- 스위치 및 게이트웨이의 데이터 처리·제어 구조를 임베디드 환경에 적용
- 중앙 제어 노드를 통한 데이터 필터링 및 분기 구조 설계

---

## 데이터 통합 및 DB 로그 결과

CAN 기반 분산 노드에서 수집된 데이터를 중앙 게이트웨이(Board C)에서
시간 기준으로 통합하고, Ethernet을 통해 상위 시스템(Raspberry Pi)으로 전달하여
DB에 스냅샷 형태로 저장한 결과입니다.

<img width="1149" height="600" alt="image" src="https://github.com/user-attachments/assets/4a31a15e-729a-4e88-9bee-45bcf1fcf332" />

- Board A(환경 센서)와 Board B(구동/거리 센서)의 데이터를 CAN으로 수신
- Gateway(Board C)에서 메시지 ID 기준 필터링 및 데이터 통합
- Ethernet(UDP) 기반으로 Raspberry Pi에 전달 후 SQLite DB에 저장

### DB 스냅샷 로그 예시

<img width="431" height="458" alt="image" src="https://github.com/user-attachments/assets/5935a76b-fc99-4860-b657-f8dda5a4eef5" />

- 1초 주기 스냅샷 방식으로 데이터 저장
- 센서/구동 데이터가 동일 타임스탬프로 정합성 있게 기록됨
- 분산 노드 데이터를 중앙에서 제어·관리하는 게이트웨이 구조 검증

---

## 트러블슈팅

### 1) Ethernet Task 스택 오버플로우 문제

- **문제**  
  Ethernet 통신 Task 실행 중 시스템 불안정 및 HardFault 발생

- **원인**  
  - Ethernet Task에 할당된 스택 크기(2048 words)가 MCU의 가용 메모리 한계를 초과  
  - LwIP 초기화 및 네트워크 버퍼 사용으로 스택 사용량 급증

- **해결**  
  - FreeRTOS Stack High Water Mark를 통해 태스크별 실제 스택 사용량 분석  
  - Ethernet Task 스택 크기를 1024 words로 최적화  
  - 불필요한 지역 변수 제거 및 네트워크 버퍼를 전역/동적 영역으로 분리

- **결과**  
  - 장시간 동작 시에도 안정적인 Ethernet 통신 확보

### 2) FreeRTOS–HAL SysTick 자원 충돌로 인한 데드락

- **문제**  
  RTOS 구동 후 Ethernet(LwIP) 초기화 과정에서 시스템이 멈추며 태스크 전환이 발생하지 않음

- **원인**  
  - FreeRTOS와 STM32 HAL이 동일한 SysTick 타이머를 시간 기준으로 공유  
  - RTOS 스케줄링과 HAL/LwIP 초기화 로직이 동시에 SysTick을 사용하며 자원 충돌 발생  
  - 특정 모듈 문제가 아닌 RTOS–HAL 전반의 타이밍 자원 충돌

- **해결**  
  - HAL 타임베이스를 SysTick에서 하드웨어 타이머(TIM6) 기반으로 변경  
  - RTOS 스케줄러와 HAL 시간 관리 로직을 물리적으로 분리  
  - FreeRTOS tick과 HAL delay 함수 간 간섭 제거

- **결과**  
  - 데드락 문제 해결  
  - CAN 및 Ethernet Task 병행 실행 시에도 RTOS 정상 동작

### 3) CAN 수신 지연으로 인한 메시지 누락 문제

- **문제**  
  다수 노드에서 CAN 메시지를 송신할 경우 일부 메시지가 수신되지 않음

- **원인**  
  - CAN 수신을 polling 방식으로 처리하여 수신 처리 지연 발생  
  - FIFO 적재 속도를 Task가 따라가지 못해 메시지 손실 발생

- **해결**  
  - CAN 수신을 Interrupt 기반 구조로 전환  
  - 수신 인터럽트 발생 시 Event/Semaphore로 전용 처리 Task를 즉시 활성화  
  - 메시지 처리와 통신 로직을 분리하여 수신 지연 최소화

- **결과**  
  - CAN 메시지 누락 현상 제거  
  - 다중 노드 환경에서도 안정적인 데이터 수신 확보
  - Queue 및 Semaphore를 활용해 Task 간 공유 자원 접근을 분리

---

## 프로젝트를 진행하며 느낀점

- RTOS 기반 환경에서 통신 제어 로직을 설계하는 경험
- CAN 통신 필터링 및 프레임 구조에 대한 이해
- 이벤트 기반 동기화를 통한 실시간 시스템 안정화
- 스위치 및 게이트웨이가 수행하는 데이터 전달·제어 흐름(Control Plane)을 임베디드 시스템 관점에서 이해
- 라우터를 포함한 네트워크 장비의 역할을 구조 중심으로 이해
- 네트워크 토폴로지, 게이트웨이, 데이터 경로 분리에 대한 구조적 사고 능력 향상
