# DT_RCcar — Isaac Sim Limo Nav2 Digital Twin

Isaac Sim 안에 스캔한 사무실 맵(Gaussian Splat) 위에 AgileX Limo(4륜 스키드조향 차동구동) 로봇을 배치해서:

1. **WASD 키보드 텔레옵**으로 수동 조작
2. **RViz에서 클릭한 목표 지점까지 Nav2 자율주행**

둘 다 구현 완료 (2026-09-07).

## 저장소 구조

```
0907.usd                          # 메인 스테이지 (맵 + Limo 로봇)
usd/                              # 맵 메시 원본 — 0907.usd가 상대경로로 참조
├── lcc-usdz-result/              # ⚠️ *.usdz는 100MB+라 이 repo엔 없음(.gitignore) — 별도로 옮길 것
└── mesh-files/
limo_wasd_teleop_behavior.py      # WASD 전용 텔레옵 BehaviorScript
limo_nav2_behavior.py             # 현재 /limo에 부착된 스크립트 — WASD+Nav2 겸용
prepare_nav2_assets_limo.py       # occupancy map 생성 + 라이다 부착용 1회 실행 스크립트 (headless)
nav2/
├── limo_map.png / limo_map.yaml  # 생성된 occupancy map
├── nav2_params_limo.yaml         # Nav2 파라미터 (Limo 크기에 맞게 튜닝됨)
├── limo_navigation.launch.py     # Nav2 브링업 launch 파일
└── limo_view.rviz                # RViz 설정
run_isaac.sh / run_nav2.sh / run_rviz.sh   # 실행 스크립트
```

## 필요 환경

- Isaac Sim 5.1
- ROS2 Humble + Nav2 (`ros-humble-navigation2`, `ros-humble-nav2-bringup`)
- 인터넷 연결 (`/limo`, `/limo/lidar`가 NVIDIA 서버의 https:// 에셋을 직접 참조)

## ⚠️ 이 repo에 없는 것 (별도로 준비해야 함)

`usd/lcc-usdz-result/*.usdz` (맵 메시 원본, 400MB+ x2)는 GitHub 파일 크기 제한 때문에 포함되지 않았습니다.
0907.usd를 실행하려면 이 파일들을 **별도 경로(zip 전송 등)로** 같은 상대 위치에 갖다 놔야 합니다.

## 하드코딩된 경로 (다른 PC에서 실행 시 수정 필요)

`run_isaac.sh`, `run_nav2.sh`, `run_rviz.sh`, `prepare_nav2_assets_limo.py` 안에 이 PC 기준 절대경로(`/home/jeon/...`)가 박혀있습니다. 다른 환경에서 쓰려면 Isaac Sim 설치 경로와 이 프로젝트 경로에 맞게 고쳐야 합니다.

## 실행 순서

```bash
./run_isaac.sh     # Isaac Sim 실행 → Play (스크립트 실행 허용 팝업 뜨면 Yes)
./run_nav2.sh       # 별도 터미널
./run_rviz.sh       # 별도 터미널
```
RViz에서 "Set Goal" 도구로 지도 위 목표 지점 클릭 → Limo 자율주행.

## 겪은 문제와 해결 (트러블슈팅 참고용)

1. **BehaviorScript(`omni:scripting:scripts`)가 조용히 로드 안 됨**: 속성만으론 부족하고, 프림에 `OmniScriptingAPI` 싱글-apply 스키마가 **함께** apiSchemas에 있어야 하고, 속성 variability가 반드시 **uniform**이어야 함 (`uniform asset[] omni:scripting:scripts`). 둘 중 하나라도 안 맞으면 에러 로그도 없이 조용히 무시됨.
2. **RTX Lidar 생성 커맨드 함정**: `omni.kit.commands.execute("IsaacSensorCreateRtxLidar", path="/limo/lidar", parent=None, ...)`처럼 절대경로+`parent=None`을 주면 엉뚱한 위치(`/World/limo/lidar_02`)에 생성됨. `path="lidar", parent="/limo"` 형태로 분리해서 호출해야 함.
3. **occupancy map 노이즈**: 이 씬의 콜리전 메시(Gaussian Splat 재구성)는 바닥이 world z=0.0이 아니라 z≈-0.89. 바닥 바로 위(2~70cm) 밴드로 스캔하면 실내 전체가 OCCUPIED로 나옴. 실측 비교 결과 "바닥 위 0.8m~2.0m" 밴드가 가장 결과가 좋았음.
4. **GPU VRAM 8GB 제약**: headless map 생성 스크립트를 Isaac Sim GUI와 동시 실행하면 크래시 위험 — GUI 먼저 닫고 실행할 것.
5. **RViz "Lookup would require extrapolation into the future" 경고**: `/scan`(Isaac Sim 렌더 시각)과 `/tf`(behavior script가 물리 스텝 누적한 시각)가 다른 시간 소스라 나는 경고. 동작엔 지장 없음.
