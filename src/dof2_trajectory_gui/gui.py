from __future__ import annotations

from typing import Optional

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import QEvent, QTimer, Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
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
    QSizePolicy,
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
        self.base_trail_x: list[float] = []
        self.base_trail_z: list[float] = []

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
        self.cb_base_trail = QCheckBox("Base trail")
        self.cb_base_trail.setChecked(True)
        button_row.addWidget(self.btn_generate)
        button_row.addWidget(self.btn_start)
        button_row.addWidget(self.btn_stop)
        button_row.addWidget(self.cb_base_trail)
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
        self.lbl_hover = QLabel("-")
        hover_font = QFont("Consolas")
        hover_font.setStyleHint(QFont.Monospace)
        self.lbl_hover.setFont(hover_font)
        self.lbl_hover.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Preferred)
        self.lbl_hover.setFixedWidth(430)
        status_layout.addRow("System", self.lbl_status)
        status_layout.addRow("Step", self.lbl_step)
        status_layout.addRow("Time", self.lbl_time)
        status_layout.addRow("Target z1", self.lbl_target_z1)
        status_layout.addRow("Target q1", self.lbl_target_q1)
        status_layout.addRow("Target q2", self.lbl_target_q2)
        status_layout.addRow("Actual q1", self.lbl_actual_q1)
        status_layout.addRow("Actual q2", self.lbl_actual_q2)
        status_layout.addRow("Hover value", self.lbl_hover)
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
        self.base_trail_curve = self.robot_plot.plot(
            [],
            [],
            pen=pg.mkPen("#66bb6a", width=2, style=Qt.PenStyle.DashLine),
        )
        self.base_trail_dots = self.robot_plot.plot(
            [],
            [],
            pen=None,
            symbol="o",
            symbolSize=7,
            symbolBrush=pg.mkBrush(102, 187, 106, 85),
            symbolPen=pg.mkPen(102, 187, 106, 130, width=1),
        )
        self.hover_robot_curve = self.robot_plot.plot(
            [],
            [],
            pen=pg.mkPen(244, 180, 0, 110, width=3),
            symbol="o",
            symbolBrush=pg.mkBrush(255, 255, 255, 90),
            symbolPen=pg.mkPen(255, 255, 255, 120, width=1),
            symbolSize=7,
        )
        self.hover_base_point = self.robot_plot.plot(
            [],
            [],
            pen=None,
            symbol="o",
            symbolSize=12,
            symbolBrush=pg.mkBrush(102, 187, 106, 110),
            symbolPen=pg.mkPen(102, 187, 106, 140, width=1),
        )
        self.hover_base_bar = self.robot_plot.plot([], [], pen=pg.mkPen(207, 216, 220, 110, width=10))
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
        self.base_trail_curve.setZValue(3)
        self.base_trail_dots.setZValue(4)
        self.hover_robot_curve.setZValue(6)
        self.hover_base_bar.setZValue(7)
        self.base_bar.setZValue(8)
        self.hover_base_point.setZValue(9)
        self.goal_point.setZValue(10)
        self.base_point.setZValue(10)
        right.addWidget(self.robot_plot, stretch=3)

        self.z_plot = pg.PlotWidget(title="z1 Trajectory")
        self.z_plot.setLabel("bottom", "time [s]")
        self.z_plot.setLabel("left", "z1 [m]")
        self.z_plot.showGrid(x=True, y=True, alpha=0.3)
        self.z_curve = self.z_plot.plot([], [], pen=pg.mkPen("#00bcd4", width=2))
        self.z_cursor = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen("#ffc107", width=1))
        self.z_plot.addItem(self.z_cursor)
        self.z_hover_marker = self.z_plot.plot(
            [],
            [],
            pen=None,
            symbol="o",
            symbolSize=10,
            symbolBrush=pg.mkBrush(0, 188, 212, 220),
            symbolPen=pg.mkPen("#e0f7fa", width=2),
        )
        self.z_hover_value = pg.TextItem(
            text="",
            color="#eceff1",
            anchor=(0, 1),
            border=pg.mkPen("#90a4ae"),
            fill=pg.mkBrush(0, 0, 0, 180),
        )
        self.z_plot.addItem(self.z_hover_value, ignoreBounds=True)
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
        self.q1_hover_marker = self.q_plot.plot(
            [],
            [],
            pen=None,
            symbol="o",
            symbolSize=9,
            symbolBrush=pg.mkBrush(255, 112, 67, 220),
            symbolPen=pg.mkPen("#ffe0b2", width=2),
        )
        self.q2_hover_marker = self.q_plot.plot(
            [],
            [],
            pen=None,
            symbol="o",
            symbolSize=9,
            symbolBrush=pg.mkBrush(66, 165, 245, 220),
            symbolPen=pg.mkPen("#bbdefb", width=2),
        )
        self.q_hover_value = pg.TextItem(
            text="",
            color="#eceff1",
            anchor=(0, 1),
            border=pg.mkPen("#90a4ae"),
            fill=pg.mkBrush(0, 0, 0, 180),
        )
        self.q_plot.addItem(self.q_hover_value, ignoreBounds=True)
        right.addWidget(self.q_plot, stretch=2)

        self._z_mouse_proxy = pg.SignalProxy(
            self.z_plot.scene().sigMouseMoved, rateLimit=60, slot=self._on_z_plot_hover
        )
        self._q_mouse_proxy = pg.SignalProxy(
            self.q_plot.scene().sigMouseMoved, rateLimit=60, slot=self._on_q_plot_hover
        )
        self.z_plot.viewport().installEventFilter(self)
        self.q_plot.viewport().installEventFilter(self)

        self.btn_generate.clicked.connect(self.on_generate)
        self.btn_start.clicked.connect(self.on_start)
        self.btn_stop.clicked.connect(self.on_stop)
        self.cb_base_trail.stateChanged.connect(self.on_base_trail_toggled)

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

    def _set_robot_view(self, cfg: ArmConfig, traj: Optional[Trajectory] = None) -> None:
        # Fit view to trajectory geometry so the robot plot starts zoomed-in and centered.
        if traj is not None and traj.n > 0:
            x_vals: list[float] = [cfg.x2 - cfg.wheel_radius, cfg.x2 + cfg.wheel_radius]
            z_vals: list[float] = [cfg.z2 - cfg.wheel_radius, cfg.z2 + cfg.wheel_radius]

            bar_len = max(0.12, 0.5 * (cfg.l1 + cfg.l2))
            half = 0.5 * bar_len

            for z1, q1, q2 in zip(traj.z1, traj.q1, traj.q2):
                base, joint2, wheel = forward_kinematics(float(z1), float(q1), float(q2), cfg)
                x_vals.extend([base[0], joint2[0], wheel[0], base[0] - half, base[0] + half])
                z_vals.extend([base[1], joint2[1], wheel[1], base[1], base[1]])

            xmin_data, xmax_data = min(x_vals), max(x_vals)
            zmin_data, zmax_data = min(z_vals), max(z_vals)
            x_span = max(1e-3, xmax_data - xmin_data)
            z_span = max(1e-3, zmax_data - zmin_data)
            x_pad = 0.10 * x_span
            z_pad = 0.12 * z_span

            xmin = xmin_data - x_pad
            xmax = xmax_data + x_pad
            zmin = zmin_data - z_pad
            zmax = zmax_data + z_pad
        else:
            reach = cfg.l1 + cfg.l2
            xmin = min(cfg.x1, cfg.x2 - cfg.wheel_radius) - 0.25 * reach
            xmax = max(cfg.x1, cfg.x2 + cfg.wheel_radius) + 0.25 * reach
            zmin = cfg.z2 - 0.25 * reach
            zmax = cfg.z2 + 0.75 * reach

        # If everything is above ground, keep ground line as the lower bound.
        if zmin >= 0.0:
            zmin = 0.0

        self.robot_plot.setXRange(xmin, xmax, padding=0.0)
        self.robot_plot.setYRange(zmin, zmax, padding=0.0)
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

    def _reset_base_trail(self) -> None:
        self.base_trail_x = []
        self.base_trail_z = []
        self.base_trail_curve.setData([], [])
        self.base_trail_dots.setData([], [])

    def _append_base_trail(self, x: float, z: float) -> None:
        if self.base_trail_x:
            if abs(self.base_trail_x[-1] - x) < 1e-9 and abs(self.base_trail_z[-1] - z) < 1e-9:
                return
        self.base_trail_x.append(x)
        self.base_trail_z.append(z)
        if self.cb_base_trail.isChecked():
            self.base_trail_curve.setData(self.base_trail_x, self.base_trail_z)
            self.base_trail_dots.setData(self.base_trail_x, self.base_trail_z)

    def on_base_trail_toggled(self, _: int) -> None:
        enabled = self.cb_base_trail.isChecked()
        self.base_trail_curve.setVisible(enabled)
        self.base_trail_dots.setVisible(enabled)
        if enabled:
            self.base_trail_curve.setData(self.base_trail_x, self.base_trail_z)
            self.base_trail_dots.setData(self.base_trail_x, self.base_trail_z)

    def _clear_hover_robot_pose(self) -> None:
        self.hover_robot_curve.setData([], [])
        self.hover_base_point.setData([], [])
        self.hover_base_bar.setData([], [])

    def _update_hover_robot_pose(self, z1: float, q1: float, q2: float) -> None:
        if self.cfg is None:
            self._clear_hover_robot_pose()
            return

        base, joint2, wheel = forward_kinematics(z1, q1, q2, self.cfg)
        self.hover_robot_curve.setData(
            [base[0], joint2[0], wheel[0]],
            [base[1], joint2[1], wheel[1]],
        )
        self.hover_base_point.setData([base[0]], [base[1]])

        bar_len = max(0.12, 0.5 * (self.cfg.l1 + self.cfg.l2))
        half = 0.5 * bar_len
        self.hover_base_bar.setData([base[0] - half, base[0] + half], [base[1], base[1]])

    def eventFilter(self, watched: object, event: object) -> bool:
        if event is not None and hasattr(event, "type"):
            if event.type() == QEvent.Type.Leave:
                if watched is self.z_plot.viewport() or watched is self.q_plot.viewport():
                    self._clear_hover_overlay()
        return super().eventFilter(watched, event)

    def _update_trajectory_plots(self, traj: Trajectory) -> None:
        self.z_curve.setData(traj.t, traj.z1)
        self.q1_curve.setData(traj.t, traj.q1)
        self.q2_curve.setData(traj.t, traj.q2)
        self.z_cursor.setValue(0.0)
        self.q_cursor.setValue(0.0)
        self._lock_data_plot_ranges(traj)
        self._clear_hover_overlay()

    def _clear_hover_overlay(self) -> None:
        self.lbl_hover.setText("-")
        self.z_hover_marker.setData([], [])
        self.q1_hover_marker.setData([], [])
        self.q2_hover_marker.setData([], [])
        self.z_hover_value.setText("")
        self.q_hover_value.setText("")
        self._clear_hover_robot_pose()

    def _lock_data_plot_ranges(self, traj: Trajectory) -> None:
        t_min = float(traj.t[0])
        t_max = float(traj.t[-1])
        if t_max <= t_min:
            t_max = t_min + 1e-3
        t_span = t_max - t_min
        t_pad = max(0.05, 0.03 * t_span)

        z_min = float(np.min(traj.z1))
        z_max = float(np.max(traj.z1))
        z_span = z_max - z_min
        z_pad = max(2e-3, 0.15 * (z_span if z_span > 0.0 else 0.05))

        q_min = float(min(np.min(traj.q1), np.min(traj.q2)))
        q_max = float(max(np.max(traj.q1), np.max(traj.q2)))
        q_span = q_max - q_min
        q_pad = max(2e-3, 0.15 * (q_span if q_span > 0.0 else 0.05))

        self.z_plot.enableAutoRange(x=False, y=False)
        self.q_plot.enableAutoRange(x=False, y=False)
        self.z_plot.setXRange(t_min - t_pad, t_max + t_pad, padding=0.0)
        self.z_plot.setYRange(z_min - z_pad, z_max + z_pad, padding=0.0)
        self.q_plot.setXRange(t_min - t_pad, t_max + t_pad, padding=0.0)
        self.q_plot.setYRange(q_min - q_pad, q_max + q_pad, padding=0.0)

    def _place_hover_text(
        self,
        plot: pg.PlotWidget,
        text_item: pg.TextItem,
        x: float,
        y: float,
        text: str,
    ) -> None:
        x_range, y_range = plot.plotItem.vb.viewRange()
        x_min, x_max = x_range
        y_min, y_max = y_range
        x_span = max(1e-9, x_max - x_min)
        y_span = max(1e-9, y_max - y_min)

        dx = 0.02 * x_span
        dy = 0.06 * y_span

        near_right = x >= (x_min + 0.75 * x_span)
        near_top = y >= (y_min + 0.75 * y_span)

        anchor_x = 1.0 if near_right else 0.0
        anchor_y = 0.0 if near_top else 1.0
        tx = x - dx if near_right else x + dx
        ty = y - dy if near_top else y + dy

        tx = min(max(tx, x_min + 0.01 * x_span), x_max - 0.01 * x_span)
        ty = min(max(ty, y_min + 0.01 * y_span), y_max - 0.01 * y_span)

        text_item.setAnchor((anchor_x, anchor_y))
        text_item.setPos(tx, ty)
        text_item.setText(text)

    def _nearest_index_from_time(self, t_query: float) -> Optional[int]:
        if self.trajectory is None or self.trajectory.n == 0:
            return None

        t_arr = self.trajectory.t
        idx = int(np.searchsorted(t_arr, t_query))
        if idx <= 0:
            return 0
        if idx >= self.trajectory.n:
            return self.trajectory.n - 1

        prev_i = idx - 1
        return prev_i if abs(t_query - t_arr[prev_i]) <= abs(t_arr[idx] - t_query) else idx

    def _update_hover_from_index(self, idx: int) -> None:
        if self.trajectory is None:
            self._clear_hover_overlay()
            return

        t = float(self.trajectory.t[idx])
        z1 = float(self.trajectory.z1[idx])
        q1 = float(self.trajectory.q1[idx])
        q2 = float(self.trajectory.q2[idx])

        self.lbl_hover.setText(
            f"k={idx:03d}  t={t:6.3f}  z1={z1:+.4f}  q1={q1:+.4f}  q2={q2:+.4f}"
        )
        self.z_cursor.setValue(t)
        self.q_cursor.setValue(t)
        self.z_hover_marker.setData([t], [z1])
        self.q1_hover_marker.setData([t], [q1])
        self.q2_hover_marker.setData([t], [q2])
        self._place_hover_text(
            self.z_plot,
            self.z_hover_value,
            t,
            z1,
            f"t={t:.3f}\nz1={z1:.4f}",
        )
        q_text_y = max(q1, q2)
        self._place_hover_text(
            self.q_plot,
            self.q_hover_value,
            t,
            q_text_y,
            f"t={t:.3f}\nq1={q1:.4f}, q2={q2:.4f}",
        )
        self._update_hover_robot_pose(z1, q1, q2)

    def _on_z_plot_hover(self, evt: tuple[object]) -> None:
        if self.trajectory is None:
            self._clear_hover_overlay()
            return

        pos = evt[0]
        if not self.z_plot.sceneBoundingRect().contains(pos):
            self._clear_hover_overlay()
            return

        mouse = self.z_plot.plotItem.vb.mapSceneToView(pos)
        idx = self._nearest_index_from_time(float(mouse.x()))
        if idx is None:
            self._clear_hover_overlay()
            return

        self._update_hover_from_index(idx)

    def _on_q_plot_hover(self, evt: tuple[object]) -> None:
        if self.trajectory is None:
            self._clear_hover_overlay()
            return

        pos = evt[0]
        if not self.q_plot.sceneBoundingRect().contains(pos):
            self._clear_hover_overlay()
            return

        mouse = self.q_plot.plotItem.vb.mapSceneToView(pos)
        idx = self._nearest_index_from_time(float(mouse.x()))
        if idx is None:
            self._clear_hover_overlay()
            return

        self._update_hover_from_index(idx)

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
        self._append_base_trail(base[0], base[1])
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

        self._set_robot_view(cfg, traj)
        self._update_trajectory_plots(traj)
        self._fill_target_table(traj)
        self._reset_base_trail()

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
