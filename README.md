# dof2-trajectory-planner

2자유도 로봇(평면 x-z)의 `z1` 이동 trajectory planning + IK + 50Hz 실행 시각화를 위한 GUI 패키지입니다.

## Features
- Quintic profile 기반 `z1` trajectory 생성
- 시점별 IK(`q1`, `q2`) 계산
- 50Hz 실행 루프(모의 로봇 인터페이스)
- 실시간 상태/타겟 테이블/그래프 표시
- x-z 평면 로봇 시각화(지면, 바퀴, 링크, 베이스 바 포함)

## Default Parameters
- `L1 = 0.26`
- `L2 = 0.37`
- `wheel diameter = 0.145` (`R = 0.0725`)
- `fixed x2 = 0.10`
- `z1 start = 0.35`
- `z1 target = 0.55`
- `elbow mode = down`

## Coordinate/Angle Convention
- 베이스 조인트: `(x1, z1)`, `x1 = 0`
- 바퀴 중심: `(x2, z2)`, `z2 = wheel_radius`
- `q1 = 0`이면 첫 링크가 왼쪽(`-x`)을 향함
- 반시계방향(CCW)이 양(+) 방향
- GUI의 각도 단위는 라디안(`rad`)

## Install
```bash
python -m pip install -e .
```

## Run
```bash
dof2-traj-gui
```

또는:
```bash
python run_gui.py
```

## Project Structure
```text
dof2-trajectory-planner/
  src/
    dof2_trajectory_gui/
      controller.py
      gui.py
      kinematics.py
      main.py
      robot_interface.py
      trajectory.py
  pyproject.toml
  requirements.txt
  run_gui.py
  README.md
```

## Notes
- 현재는 `MockRobotInterface` 기반 시뮬레이션 모드입니다.
- 실제 모터/엔코더 연동은 `src/dof2_trajectory_gui/robot_interface.py`의 `RealRobotInterface` 구현으로 확장할 수 있습니다.

