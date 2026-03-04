from __future__ import annotations

from typing import Optional

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
    QAbstractItemView,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from .controller import StepCommand, TrajectoryController
from .kinematics import ArmConfig, forward_kinematics
from .robot_interface import MockRobotInterface
from .trajectory import Trajectory, plan_z1_trajectory

pg.setConfigOptions(antialias=True)


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("2-DOF Trajectory Planner (x-z Plane)")
        self.resize(1400, 900)

        self.fs = 50.0
        self.robot = MockRobotInterface()
        self.controller = TrajectoryController(fs=self.fs)
        self.trajectory: Optional[Trajectory] = None
        self.cfg: Optional[ArmConfig] = None

        self._build_ui()
        self.robot.connect()

        self.timer = QTimer(self)
        self.timer.setInterval(int(round(1000.0 / self.fs)))
        self.timer.timeout.connect(self.on_timer)
        self.timer.start()

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)

        left = QVBoxLayout()
        right = QVBoxLayout()
        root.addLayout(left, stretch=0)
        root.addLayout(right, stretch=1)

        param_group = QGroupBox("Parameters")
        param_layout = QFormLayout(param_group)

        self.sb_l1 = self._spin(0.01, 5.0, 0.26, 0.01)
        self.sb_l2 = self._spin(0.01, 5.0, 0.37, 0.01)
        self.sb_wheel_d = self._spin(0.01, 2.0, 0.0725 * 2.0, 0.01)
        self.sb_x2 = self._spin(-5.0, 5.0, 0.10, 0.01)
        self.sb_z1_start = self._spin(-5.0, 5.0, 0.35, 0.01)
        self.sb_z1_target = self._spin(-5.0, 5.0, 0.55, 0.01)
        self.sb_duration = self._spin(0.1, 30.0, 3.0, 0.1)

        self.cb_elbow = QComboBox()
        self.cb_elbow.addItems(["down", "up"])
        self.cb_elbow.setCurrentText("down")

        param_layout.addRow("L1 [m]", self.sb_l1)
        param_layout.addRow("L2 [m]", self.sb_l2)
        param_layout.addRow("Wheel diameter [m]", self.sb_wheel_d)
        param_layout.addRow("Fixed x2 [m]", self.sb_x2)
        param_layout.addRow("z1 start [m]", self.sb_z1_start)
        param_layout.addRow("z1 target [m]", self.sb_z1_target)
        param_layout.addRow("Duration [s]", self.sb_duration)
        param_layout.addRow("Elbow mode", self.cb_elbow)
        left.addWidget(param_group)

        button_row = QHBoxLayout()
        self.btn_generate = QPushButton("Generate")
        self.btn_start = QPushButton("Start")
        self.btn_stop = QPushButton("Stop")
        button_row.addWidget(self.btn_generate)
        button_row.addWidget(self.btn_start)
        button_row.addWidget(self.btn_stop)
        left.addLayout(button_row)

        status_group = QGroupBox("Live Status")
        status_layout = QFormLayout(status_group)
        self.lbl_status = QLabel("Ready")
        self.lbl_step = QLabel("-")
        self.lbl_time = QLabel("-")
        self.lbl_target_z1 = QLabel("-")
        self.lbl_target_q1 = QLabel("-")
        self.lbl_target_q2 = QLabel("-")
        self.lbl_actual_q1 = QLabel("-")
        self.lbl_actual_q2 = QLabel("-")
        status_layout.addRow("System", self.lbl_status)
        status_layout.addRow("Step", self.lbl_step)
        status_layout.addRow("Time", self.lbl_time)
        status_layout.addRow("Target z1", self.lbl_target_z1)
        status_layout.addRow("Target q1", self.lbl_target_q1)
        status_layout.addRow("Target q2", self.lbl_target_q2)
        status_layout.addRow("Actual q1", self.lbl_actual_q1)
        status_layout.addRow("Actual q2", self.lbl_actual_q2)
        left.addWidget(status_group)

        table_group = QGroupBox("Target Joint Positions (time-step)")
        table_layout = QVBoxLayout(table_group)
        self.target_table = QTableWidget(0, 4)
        self.target_table.setHorizontalHeaderLabels(["t [s]", "z1 [m]", "q1 [rad]", "q2 [rad]"])
        self.target_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.target_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.target_table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.target_table.verticalHeader().setVisible(False)
        self.target_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        table_layout.addWidget(self.target_table)
        left.addWidget(table_group, stretch=1)

        self.robot_plot = pg.PlotWidget(title="Robot Pose in x-z Plane")
        self.robot_plot.setLabel("bottom", "x [m]")
        self.robot_plot.setLabel("left", "z [m]")
        self.robot_plot.showGrid(x=True, y=True, alpha=0.3)
        self.robot_plot.setAspectLocked(True)
        self.robot_curve = self.robot_plot.plot(
            [],
            [],
            pen=pg.mkPen("#f4b400", width=4),
            symbol="o",
            symbolBrush="w",
            symbolSize=8,
        )
        self.base_point = self.robot_plot.plot(
            [], [], pen=None, symbol="o", symbolBrush="#66bb6a", symbolSize=12
        )
        self.base_bar = self.robot_plot.plot([], [], pen=pg.mkPen("#cfd8dc", width=10))
        self.goal_point = self.robot_plot.plot(
            [], [], pen=None, symbol="x", symbolPen=pg.mkPen("#ff5252", width=2), symbolSize=12
        )
        self.ground_line = self.robot_plot.plot([], [], pen=pg.mkPen("#6d4c41", width=2))
        self.wheel_circle = self.robot_plot.plot([], [], pen=pg.mkPen("#90caf9", width=2))

        # Draw order: green base point always on top.
        self.ground_line.setZValue(0)
        self.wheel_circle.setZValue(1)
        self.robot_curve.setZValue(2)
        self.base_bar.setZValue(3)
        self.goal_point.setZValue(4)
        self.base_point.setZValue(10)
        right.addWidget(self.robot_plot, stretch=3)

        self.z_plot = pg.PlotWidget(title="z1 Trajectory")
        self.z_plot.setLabel("bottom", "time [s]")
        self.z_plot.setLabel("left", "z1 [m]")
        self.z_plot.showGrid(x=True, y=True, alpha=0.3)
        self.z_curve = self.z_plot.plot([], [], pen=pg.mkPen("#00bcd4", width=2))
        self.z_cursor = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen("#ffc107", width=1))
        self.z_plot.addItem(self.z_cursor)
        right.addWidget(self.z_plot, stretch=2)

        self.q_plot = pg.PlotWidget(title="Joint Targets")
        self.q_plot.setLabel("bottom", "time [s]")
        self.q_plot.setLabel("left", "angle [rad]")
        self.q_plot.showGrid(x=True, y=True, alpha=0.3)
        self.q_plot.addLegend(offset=(8, 8))
        self.q1_curve = self.q_plot.plot([], [], pen=pg.mkPen("#ff7043", width=2), name="q1")
        self.q2_curve = self.q_plot.plot([], [], pen=pg.mkPen("#42a5f5", width=2), name="q2")
        self.q_cursor = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen("#ffc107", width=1))
        self.q_plot.addItem(self.q_cursor)
        right.addWidget(self.q_plot, stretch=2)

        self.btn_generate.clicked.connect(self.on_generate)
        self.btn_start.clicked.connect(self.on_start)
        self.btn_stop.clicked.connect(self.on_stop)

    @staticmethod
    def _spin(min_v: float, max_v: float, value: float, step: float) -> QDoubleSpinBox:
        sb = QDoubleSpinBox()
        sb.setRange(min_v, max_v)
        sb.setDecimals(4)
        sb.setSingleStep(step)
        sb.setValue(value)
        return sb

    def _read_config(self) -> ArmConfig:
        cfg = ArmConfig(
            l1=self.sb_l1.value(),
            l2=self.sb_l2.value(),
            x2=self.sb_x2.value(),
            wheel_diameter=self.sb_wheel_d.value(),
            x1=0.0,
        )
        if cfg.l1 <= 0.0 or cfg.l2 <= 0.0:
            raise ValueError("L1 and L2 must be > 0.")
        if cfg.wheel_diameter <= 0.0:
            raise ValueError("Wheel diameter must be > 0.")
        return cfg

    def _set_robot_view(self, cfg: ArmConfig) -> None:
        reach = cfg.l1 + cfg.l2 + 0.05
        xmin = min(cfg.x1, cfg.x2 - cfg.wheel_radius) - 0.2 * reach
        xmax = max(cfg.x1, cfg.x2 + cfg.wheel_radius) + 0.2 * reach
        zmin = min(-0.05, cfg.z2 - reach)
        zmax = cfg.z2 + reach
        self.robot_plot.setXRange(xmin, xmax, padding=0.05)
        self.robot_plot.setYRange(zmin, zmax, padding=0.05)
        self.ground_line.setData([xmin, xmax], [0.0, 0.0])

        theta = np.linspace(0.0, 2.0 * np.pi, 120)
        wheel_x = cfg.x2 + cfg.wheel_radius * np.cos(theta)
        wheel_z = cfg.z2 + cfg.wheel_radius * np.sin(theta)
        self.wheel_circle.setData(wheel_x, wheel_z)

    def _fill_target_table(self, traj: Trajectory) -> None:
        self.target_table.setRowCount(traj.n)
        for i in range(traj.n):
            row_values = (
                f"{traj.t[i]:.3f}",
                f"{traj.z1[i]:.4f}",
                f"{traj.q1[i]:.4f}",
                f"{traj.q2[i]:.4f}",
            )
            for c, v in enumerate(row_values):
                self.target_table.setItem(i, c, QTableWidgetItem(v))

    def _update_trajectory_plots(self, traj: Trajectory) -> None:
        self.z_curve.setData(traj.t, traj.z1)
        self.q1_curve.setData(traj.t, traj.q1)
        self.q2_curve.setData(traj.t, traj.q2)
        self.z_cursor.setValue(0.0)
        self.q_cursor.setValue(0.0)

    def _render_step(self, cmd: StepCommand) -> None:
        if self.cfg is None or self.trajectory is None:
            return

        state = self.robot.get_joint_state()

        self.lbl_step.setText(f"{cmd.index + 1} / {self.trajectory.n}")
        self.lbl_time.setText(f"{cmd.t:.3f} s")
        self.lbl_target_z1.setText(f"{cmd.z1:.4f} m")
        self.lbl_target_q1.setText(f"{cmd.q1:.4f} rad")
        self.lbl_target_q2.setText(f"{cmd.q2:.4f} rad")
        self.lbl_actual_q1.setText(f"{state.q1:.4f} rad")
        self.lbl_actual_q2.setText(f"{state.q2:.4f} rad")

        base, joint2, wheel = forward_kinematics(cmd.z1, state.q1, state.q2, self.cfg)
        self.robot_curve.setData(
            [base[0], joint2[0], wheel[0]],
            [base[1], joint2[1], wheel[1]],
        )
        self.base_point.setData([base[0]], [base[1]])
        bar_len = max(0.12, 0.5 * (self.cfg.l1 + self.cfg.l2))
        half = 0.5 * bar_len
        self.base_bar.setData([base[0] - half, base[0] + half], [base[1], base[1]])
        self.goal_point.setData([self.cfg.x2], [self.cfg.z2])

        self.z_cursor.setValue(cmd.t)
        self.q_cursor.setValue(cmd.t)
        if 0 <= cmd.index < self.target_table.rowCount():
            self.target_table.selectRow(cmd.index)

    def on_generate(self) -> None:
        try:
            cfg = self._read_config()
            traj = plan_z1_trajectory(
                cfg=cfg,
                z1_start=self.sb_z1_start.value(),
                z1_target=self.sb_z1_target.value(),
                duration=self.sb_duration.value(),
                fs=self.fs,
                elbow=self.cb_elbow.currentText(),
            )
        except Exception as exc:
            QMessageBox.critical(self, "Trajectory Error", str(exc))
            self.lbl_status.setText("Generate failed")
            return

        self.cfg = cfg
        self.trajectory = traj
        self.controller.load(traj)

        self._set_robot_view(cfg)
        self._update_trajectory_plots(traj)
        self._fill_target_table(traj)

        first = StepCommand(
            index=0,
            t=float(traj.t[0]),
            z1=float(traj.z1[0]),
            q1=float(traj.q1[0]),
            q2=float(traj.q2[0]),
        )
        self.robot.send_joint_target(first.q1, first.q2)
        self._render_step(first)

        self.lbl_status.setText(f"Generated {traj.n} points ({traj.duration:.2f}s @ {traj.fs:.1f}Hz)")

    def on_start(self) -> None:
        if self.trajectory is None:
            QMessageBox.information(self, "Info", "Generate trajectory first.")
            return

        try:
            self.controller.start()
        except RuntimeError as exc:
            QMessageBox.warning(self, "Error", str(exc))
            return
        self.lbl_status.setText("Running")

    def on_stop(self) -> None:
        self.controller.stop()
        self.lbl_status.setText("Stopped")

    def on_timer(self) -> None:
        cmd = self.controller.step()
        if cmd is None:
            return

        self.robot.send_joint_target(cmd.q1, cmd.q2)
        self._render_step(cmd)

        if self.trajectory is not None and cmd.index >= self.trajectory.n - 1:
            self.lbl_status.setText("Trajectory complete")
