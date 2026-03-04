from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from .kinematics import ArmConfig, forward_kinematics, inverse_kinematics

pg.setConfigOptions(antialias=True)


@dataclass
class ReachInfo:
    intervals: list[tuple[float, float]]
    z_min: float
    z_max: float


class PoseOnlyWindow(QMainWindow):
    SLIDER_STEPS = 2000

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("2-DOF Robot Pose (Parameter + Slider)")
        self.resize(1280, 820)

        self.cfg: Optional[ArmConfig] = None
        self.reach: Optional[ReachInfo] = None
        self._syncing = False
        self._slider_z_min = 0.0
        self._slider_z_max = 1.0

        self._build_ui()
        self.on_params_changed()

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
        self.cb_elbow = QComboBox()
        self.cb_elbow.addItems(["down", "up"])
        self.cb_elbow.setCurrentText("down")
        param_layout.addRow("L1 [m]", self.sb_l1)
        param_layout.addRow("L2 [m]", self.sb_l2)
        param_layout.addRow("Wheel diameter [m]", self.sb_wheel_d)
        param_layout.addRow("Fixed x2 [m]", self.sb_x2)
        param_layout.addRow("Elbow mode", self.cb_elbow)
        left.addWidget(param_group)

        z_group = QGroupBox("Position Control")
        z_layout = QFormLayout(z_group)
        self.sb_z1 = self._spin(0.0, 5.0, 0.35, 0.001)
        self.sl_z1 = QSlider(Qt.Orientation.Horizontal)
        self.sl_z1.setRange(0, self.SLIDER_STEPS)
        self.lbl_reach = QLabel("-")
        z_layout.addRow("z1 [m]", self.sb_z1)
        z_layout.addRow("z1 slider", self.sl_z1)
        z_layout.addRow("Reachable z1", self.lbl_reach)
        left.addWidget(z_group)

        state_group = QGroupBox("Current State")
        state_layout = QFormLayout(state_group)
        self.lbl_q1 = QLabel("-")
        self.lbl_q2 = QLabel("-")
        self.lbl_z2 = QLabel("-")
        self.lbl_status = QLabel("Ready")
        state_layout.addRow("q1 [rad]", self.lbl_q1)
        state_layout.addRow("q2 [rad]", self.lbl_q2)
        state_layout.addRow("z2 [m]", self.lbl_z2)
        state_layout.addRow("Status", self.lbl_status)
        left.addWidget(state_group)
        left.addStretch(1)

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
            [],
            [],
            pen=None,
            symbol="o",
            symbolBrush="#66bb6a",
            symbolSize=12,
        )
        self.base_bar = self.robot_plot.plot([], [], pen=pg.mkPen("#cfd8dc", width=10))
        self.l1_extension_curve = self.robot_plot.plot(
            [],
            [],
            pen=pg.mkPen(220, 220, 220, 120, width=1, style=Qt.PenStyle.DashLine),
        )
        self.goal_point = self.robot_plot.plot(
            [],
            [],
            pen=None,
            symbol="x",
            symbolPen=pg.mkPen("#ff5252", width=2),
            symbolSize=12,
        )
        self.ground_line = self.robot_plot.plot([], [], pen=pg.mkPen("#6d4c41", width=2))
        self.wheel_circle = self.robot_plot.plot([], [], pen=pg.mkPen("#90caf9", width=2))

        self.txt_z1 = pg.TextItem(
            color="#66bb6a",
            anchor=(0, 1),
            border=pg.mkPen(255, 255, 255, 220, width=1),
            fill=pg.mkBrush(0, 0, 0, 150),
        )
        self.txt_q1 = pg.TextItem(
            color="#ff7043",
            anchor=(0, 1),
            border=pg.mkPen(255, 255, 255, 220, width=1),
            fill=pg.mkBrush(0, 0, 0, 150),
        )
        self.txt_q2 = pg.TextItem(
            color="#42a5f5",
            anchor=(0, 1),
            border=pg.mkPen(255, 255, 255, 220, width=1),
            fill=pg.mkBrush(0, 0, 0, 150),
        )
        self.q1_arc_curve = self.robot_plot.plot([], [], pen=pg.mkPen("#ff7043", width=3))
        self.q2_arc_curve = self.robot_plot.plot([], [], pen=pg.mkPen("#42a5f5", width=3))
        self.robot_plot.addItem(self.txt_z1, ignoreBounds=True)
        self.robot_plot.addItem(self.txt_q1, ignoreBounds=True)
        self.robot_plot.addItem(self.txt_q2, ignoreBounds=True)

        self.ground_line.setZValue(0)
        self.wheel_circle.setZValue(1)
        self.robot_curve.setZValue(2)
        self.base_bar.setZValue(3)
        self.goal_point.setZValue(4)
        self.l1_extension_curve.setZValue(5)
        self.q1_arc_curve.setZValue(6)
        self.q2_arc_curve.setZValue(6)
        self.base_point.setZValue(10)
        self.txt_z1.setZValue(100)
        self.txt_q1.setZValue(100)
        self.txt_q2.setZValue(100)

        right.addWidget(self.robot_plot, stretch=1)

        self.sb_l1.valueChanged.connect(self.on_params_changed)
        self.sb_l2.valueChanged.connect(self.on_params_changed)
        self.sb_wheel_d.valueChanged.connect(self.on_params_changed)
        self.sb_x2.valueChanged.connect(self.on_params_changed)
        self.cb_elbow.currentTextChanged.connect(self.on_params_changed)
        self.sb_z1.valueChanged.connect(self.on_z1_spin_changed)
        self.sl_z1.valueChanged.connect(self.on_slider_changed)

    @staticmethod
    def _spin(min_v: float, max_v: float, value: float, step: float) -> QDoubleSpinBox:
        sb = QDoubleSpinBox()
        sb.setRange(min_v, max_v)
        sb.setDecimals(4)
        sb.setSingleStep(step)
        sb.setValue(value)
        return sb

    def _read_config(self) -> ArmConfig:
        return ArmConfig(
            l1=self.sb_l1.value(),
            l2=self.sb_l2.value(),
            x2=self.sb_x2.value(),
            wheel_diameter=self.sb_wheel_d.value(),
            x1=0.0,
        )

    @staticmethod
    def _compute_reach(cfg: ArmConfig) -> ReachInfo:
        dx = abs(cfg.x2 - cfg.x1)
        max_r = cfg.l1 + cfg.l2
        min_r = abs(cfg.l1 - cfg.l2)

        if dx > max_r:
            return ReachInfo(intervals=[], z_min=cfg.z2, z_max=cfg.z2)

        dz_max = float(np.sqrt(max(0.0, max_r * max_r - dx * dx)))
        dz_min = float(np.sqrt(max(0.0, min_r * min_r - dx * dx))) if dx < min_r else 0.0

        if dz_min <= 1e-10:
            intervals = [(cfg.z2 - dz_max, cfg.z2 + dz_max)]
        else:
            intervals = [
                (cfg.z2 - dz_max, cfg.z2 - dz_min),
                (cfg.z2 + dz_min, cfg.z2 + dz_max),
            ]
        return ReachInfo(intervals=intervals, z_min=intervals[0][0], z_max=intervals[-1][1])

    @staticmethod
    def _clip_reach_nonnegative(reach: ReachInfo) -> ReachInfo:
        clipped: list[tuple[float, float]] = []
        for a, b in reach.intervals:
            a2 = max(0.0, a)
            b2 = max(0.0, b)
            if b2 >= a2:
                clipped.append((a2, b2))

        if not clipped:
            return ReachInfo(intervals=[], z_min=0.0, z_max=0.0)
        return ReachInfo(intervals=clipped, z_min=clipped[0][0], z_max=clipped[-1][1])

    def _format_intervals(self, reach: ReachInfo) -> str:
        if not reach.intervals:
            return "No reachable z1 >= 0 interval"
        if len(reach.intervals) == 1:
            a, b = reach.intervals[0]
            return f"[{a:.4f}, {b:.4f}]"
        low = reach.intervals[0]
        high = reach.intervals[1]
        return f"[{low[0]:.4f}, {low[1]:.4f}] U [{high[0]:.4f}, {high[1]:.4f}]"

    def _snap_to_reachable(self, z1: float) -> float:
        z1 = max(0.0, z1)
        if self.reach is None or not self.reach.intervals:
            return z1
        for a, b in self.reach.intervals:
            if a <= z1 <= b:
                return z1
        candidates: list[float] = []
        for a, b in self.reach.intervals:
            candidates.extend([a, b])
        return min(candidates, key=lambda c: abs(c - z1))

    def _z_to_slider(self, z1: float) -> int:
        if self._slider_z_max <= self._slider_z_min:
            return 0
        ratio = (z1 - self._slider_z_min) / (self._slider_z_max - self._slider_z_min)
        ratio = float(np.clip(ratio, 0.0, 1.0))
        return int(round(ratio * self.SLIDER_STEPS))

    def _slider_to_z(self, slider_val: int) -> float:
        ratio = float(np.clip(slider_val / self.SLIDER_STEPS, 0.0, 1.0))
        return self._slider_z_min + ratio * (self._slider_z_max - self._slider_z_min)

    def _compute_robot_view(self, cfg: ArmConfig, reach: ReachInfo) -> tuple[float, float, float, float]:
        elbow = self.cb_elbow.currentText()
        x_vals: list[float] = [cfg.x2 - cfg.wheel_radius, cfg.x2 + cfg.wheel_radius, cfg.x1]
        z_vals: list[float] = [0.0, cfg.z2 - cfg.wheel_radius, cfg.z2 + cfg.wheel_radius]
        bar_len = max(0.12, 0.5 * (cfg.l1 + cfg.l2))
        half = 0.5 * bar_len

        if reach.intervals:
            for a, b in reach.intervals:
                for z1 in np.linspace(a, b, 30):
                    try:
                        q1, q2 = inverse_kinematics(float(z1), cfg, elbow=elbow)
                    except ValueError:
                        continue
                    base, joint2, wheel = forward_kinematics(float(z1), q1, q2, cfg)
                    x_vals.extend([base[0], joint2[0], wheel[0], base[0] - half, base[0] + half])
                    z_vals.extend([base[1], joint2[1], wheel[1], base[1], base[1]])

        x_min = min(x_vals)
        x_max = max(x_vals)
        z_min = min(z_vals)
        z_max = max(z_vals)

        x_span = max(1e-3, x_max - x_min)
        z_span = max(1e-3, z_max - z_min)
        x_pad = 0.10 * x_span
        z_pad = 0.12 * z_span

        x_min -= x_pad
        x_max += x_pad
        z_min -= z_pad
        z_max += z_pad
        if z_min >= 0.0:
            z_min = 0.0
        return x_min, x_max, z_min, z_max

    @staticmethod
    def _arc_points(
        cx: float,
        cz: float,
        start_angle: float,
        delta_angle: float,
        radius: float,
        n: int = 80,
    ) -> tuple[np.ndarray, np.ndarray]:
        if abs(delta_angle) < 1e-6:
            return np.array([], dtype=float), np.array([], dtype=float)
        angles = np.linspace(start_angle, start_angle + delta_angle, n)
        x = cx + radius * np.cos(angles)
        z = cz + radius * np.sin(angles)
        return x, z

    @staticmethod
    def _rects_overlap(
        r1: tuple[float, float, float, float],
        r2: tuple[float, float, float, float],
        pad_x: float,
        pad_y: float,
    ) -> bool:
        return not (
            r1[2] + pad_x < r2[0]
            or r2[2] + pad_x < r1[0]
            or r1[3] + pad_y < r2[1]
            or r2[3] + pad_y < r1[1]
        )

    def _estimate_text_size_data(self, item: pg.TextItem) -> tuple[float, float]:
        vb = self.robot_plot.plotItem.vb
        x_range, y_range = vb.viewRange()
        x_span = max(1e-9, x_range[1] - x_range[0])
        y_span = max(1e-9, y_range[1] - y_range[0])
        scene_rect = vb.sceneBoundingRect()
        br = item.boundingRect()
        px_w = max(1.0, scene_rect.width())
        px_h = max(1.0, scene_rect.height())
        w_data = max(1e-4, br.width() * x_span / px_w)
        h_data = max(1e-4, br.height() * y_span / px_h)
        return w_data, h_data

    def _place_text_item(
        self,
        item: pg.TextItem,
        anchor_x: float,
        anchor_y: float,
        offsets: list[tuple[float, float]],
        placed_rects: list[tuple[float, float, float, float]],
    ) -> None:
        vb = self.robot_plot.plotItem.vb
        x_range, y_range = vb.viewRange()
        x_min, x_max = x_range
        y_min, y_max = y_range
        x_span = max(1e-9, x_max - x_min)
        y_span = max(1e-9, y_max - y_min)
        margin_x = 0.01 * x_span
        margin_y = 0.01 * y_span
        sep_x = 0.012 * x_span
        sep_y = 0.015 * y_span

        w, h = self._estimate_text_size_data(item)

        best_rect: Optional[tuple[float, float, float, float]] = None
        best_pos: Optional[tuple[float, float]] = None

        for ox, oy in offsets:
            px = anchor_x + ox
            py = anchor_y + oy

            px = min(max(px, x_min + margin_x), x_max - w - margin_x)
            py = min(max(py, y_min + h + margin_y), y_max - margin_y)
            rect = (px, py - h, px + w, py)

            is_ok = True
            for prev in placed_rects:
                if self._rects_overlap(rect, prev, sep_x, sep_y):
                    is_ok = False
                    break
            if is_ok:
                best_pos = (px, py)
                best_rect = rect
                break

        if best_pos is None:
            px = min(max(anchor_x + offsets[0][0], x_min + margin_x), x_max - w - margin_x)
            py = min(max(anchor_y + offsets[0][1], y_min + h + margin_y), y_max - margin_y)
            best_pos = (px, py)
            best_rect = (px, py - h, px + w, py)

        item.setPos(best_pos[0], best_pos[1])
        placed_rects.append(best_rect)

    def _update_pose(self, z1: float) -> None:
        if self.cfg is None:
            return

        try:
            q1, q2 = inverse_kinematics(z1, self.cfg, elbow=self.cb_elbow.currentText())
        except ValueError:
            self.lbl_status.setText("Unreachable pose")
            self.q1_arc_curve.setData([], [])
            self.q2_arc_curve.setData([], [])
            self.l1_extension_curve.setData([], [])
            return

        base, joint2, wheel = forward_kinematics(z1, q1, q2, self.cfg)
        self.robot_curve.setData(
            [base[0], joint2[0], wheel[0]],
            [base[1], joint2[1], wheel[1]],
        )
        self.base_point.setData([base[0]], [base[1]])
        bar_len = max(0.12, 0.5 * (self.cfg.l1 + self.cfg.l2))
        half = 0.5 * bar_len
        self.base_bar.setData([base[0] - half, base[0] + half], [base[1], base[1]])
        self.goal_point.setData([self.cfg.x2], [self.cfg.z2])

        theta1 = q1 + np.pi
        theta2 = theta1 + q2
        ext_len = max(0.04, 0.22 * self.cfg.l2)
        self.l1_extension_curve.setData(
            [joint2[0], joint2[0] + ext_len * np.cos(theta1)],
            [joint2[1], joint2[1] + ext_len * np.sin(theta1)],
        )

        arc_r1 = max(0.03, 0.22 * min(self.cfg.l1, self.cfg.l2))
        arc_r2 = max(0.035, 0.28 * min(self.cfg.l1, self.cfg.l2))
        q1_ax, q1_az = self._arc_points(base[0], base[1], np.pi, q1, arc_r1)
        q2_ax, q2_az = self._arc_points(joint2[0], joint2[1], theta1, q2, arc_r2)
        self.q1_arc_curve.setData(q1_ax, q1_az)
        self.q2_arc_curve.setData(q2_ax, q2_az)

        x_range, y_range = self.robot_plot.plotItem.vb.viewRange()
        dx = 0.02 * (x_range[1] - x_range[0])
        dy = 0.03 * (y_range[1] - y_range[0])

        q1_mid = np.pi + 0.5 * q1
        q2_mid = theta1 + 0.5 * q2
        q1_tx = base[0] + (arc_r1 + 0.02) * np.cos(q1_mid)
        q1_tz = base[1] + (arc_r1 + 0.02) * np.sin(q1_mid)
        q2_tx = joint2[0] + (arc_r2 + 0.02) * np.cos(q2_mid)
        q2_tz = joint2[1] + (arc_r2 + 0.02) * np.sin(q2_mid)

        self.txt_z1.setHtml(
            f"<span style='font-size:12pt; font-weight:700; color:#66bb6a;'>z1={z1:+.4f}</span>"
        )
        self.txt_q1.setHtml(
            f"<span style='font-size:12pt; font-weight:700; color:#ff7043;'>q1={q1:+.4f}</span>"
        )
        self.txt_q2.setHtml(
            f"<span style='font-size:12pt; font-weight:700; color:#42a5f5;'>q2={q2:+.4f}</span>"
        )

        placed: list[tuple[float, float, float, float]] = []
        self._place_text_item(
            self.txt_q2,
            q2_tx,
            q2_tz,
            offsets=[
                (dx * 0.5, dy * 0.5),
                (dx * 0.5, -dy * 1.2),
                (-dx * 1.6, dy * 0.5),
                (-dx * 1.6, -dy * 1.2),
                (dx * 1.8, dy * 0.1),
                (-dx * 2.0, dy * 0.1),
            ],
            placed_rects=placed,
        )
        self._place_text_item(
            self.txt_q1,
            q1_tx,
            q1_tz,
            offsets=[
                (dx * 0.3, dy * 0.4),
                (dx * 0.3, -dy * 1.1),
                (-dx * 1.5, dy * 0.4),
                (-dx * 1.5, -dy * 1.1),
                (dx * 1.6, dy * 0.1),
            ],
            placed_rects=placed,
        )
        self._place_text_item(
            self.txt_z1,
            base[0],
            base[1],
            offsets=[
                (dx * 0.8, dy * 0.8),
                (dx * 0.8, -dy * 1.4),
                (-dx * 2.2, dy * 0.8),
                (-dx * 2.2, -dy * 1.4),
                (dx * 2.0, dy * 0.2),
            ],
            placed_rects=placed,
        )

        self.lbl_q1.setText(f"{q1:.4f}")
        self.lbl_q2.setText(f"{q2:.4f}")
        self.lbl_z2.setText(f"{self.cfg.z2:.4f}")
        self.lbl_status.setText("Pose updated")

    def on_params_changed(self) -> None:
        if self._syncing:
            return
        self.cfg = self._read_config()
        raw_reach = self._compute_reach(self.cfg)
        self.reach = self._clip_reach_nonnegative(raw_reach)
        self.lbl_reach.setText(self._format_intervals(self.reach))

        if not self.reach.intervals:
            self.lbl_status.setText("No reachable interval for z1 >= 0")
            self._syncing = True
            self.sb_z1.setRange(0.0, 0.0)
            self.sb_z1.setValue(0.0)
            self.sl_z1.setValue(0)
            self._syncing = False
            self.robot_curve.setData([], [])
            self.base_point.setData([], [])
            self.base_bar.setData([], [])
            self.l1_extension_curve.setData([], [])
            self.q1_arc_curve.setData([], [])
            self.q2_arc_curve.setData([], [])
            self.txt_z1.setText("")
            self.txt_q1.setText("")
            self.txt_q2.setText("")
            return

        self._slider_z_min = self.reach.z_min
        self._slider_z_max = self.reach.z_max

        z1_cur = self._snap_to_reachable(max(0.0, self.sb_z1.value()))
        self._syncing = True
        self.sb_z1.setRange(self._slider_z_min, self._slider_z_max)
        self.sb_z1.setValue(z1_cur)
        self.sl_z1.setValue(self._z_to_slider(z1_cur))
        self._syncing = False

        x_min, x_max, z_min, z_max = self._compute_robot_view(self.cfg, self.reach)
        self.robot_plot.enableAutoRange(x=False, y=False)
        self.robot_plot.setXRange(x_min, x_max, padding=0.0)
        self.robot_plot.setYRange(z_min, z_max, padding=0.0)
        self.ground_line.setData([x_min, x_max], [0.0, 0.0])

        theta = np.linspace(0.0, 2.0 * np.pi, 120)
        wheel_x = self.cfg.x2 + self.cfg.wheel_radius * np.cos(theta)
        wheel_z = self.cfg.z2 + self.cfg.wheel_radius * np.sin(theta)
        self.wheel_circle.setData(wheel_x, wheel_z)

        self._update_pose(z1_cur)

    def on_z1_spin_changed(self, z1: float) -> None:
        if self._syncing:
            return
        z1_snap = self._snap_to_reachable(float(z1))
        self._syncing = True
        self.sb_z1.setValue(z1_snap)
        self.sl_z1.setValue(self._z_to_slider(z1_snap))
        self._syncing = False
        self._update_pose(z1_snap)

    def on_slider_changed(self, slider_val: int) -> None:
        if self._syncing:
            return
        z1 = self._slider_to_z(slider_val)
        z1_snap = self._snap_to_reachable(z1)
        self._syncing = True
        self.sb_z1.setValue(z1_snap)
        self.sl_z1.setValue(self._z_to_slider(z1_snap))
        self._syncing = False
        self._update_pose(z1_snap)
