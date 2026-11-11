#!/usr/bin/python3
"""Hold-to-drive PyQt5 controller for the ROBOCON mecanum chassis."""

from dataclasses import dataclass
import signal
import sys

from geometry_msgs.msg import Twist
from PyQt5.QtCore import QEvent
from PyQt5.QtCore import QTimer
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)
import rclpy
from rclpy.node import Node


@dataclass(frozen=True)
class SpeedSpec:
    key: str
    label: str
    unit: str
    minimum: float
    maximum: float
    initial: float


FORWARD = 'forward'
LATERAL = 'lateral'
ROTATION = 'rotation'
SLIDER_STEPS = 1000


class ChassisGui(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.publisher = node.create_publisher(Twist, '/cmd_vel', 10)
        self.publish_rate = max(
            1.0, float(node.get_parameter('publish_rate').value)
        )
        linear_limit = max(
            0.05, float(node.get_parameter('linear_limit').value)
        )
        angular_limit = max(
            0.05, float(node.get_parameter('angular_limit').value)
        )
        self.specs = {
            FORWARD: SpeedSpec(
                FORWARD, '前后速度', 'm/s',
                0.05, linear_limit, 0.35,
            ),
            LATERAL: SpeedSpec(
                LATERAL, '横向速度', 'm/s',
                0.05, linear_limit, 0.30,
            ),
            ROTATION: SpeedSpec(
                ROTATION, '旋转速度', 'rad/s',
                0.10, angular_limit, 0.65,
            ),
        }
        self.sliders = {}
        self.spinboxes = {}
        self.direction_buttons = {}
        self.active_direction = None
        self.closing = False

        self.setWindowTitle('RC2027 麦克纳姆底盘按住控制')
        self.setMinimumWidth(820)
        self._build_ui()

        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self._spin_ros)
        self.spin_timer.start(20)

        self.publish_timer = QTimer(self)
        self.publish_timer.timeout.connect(self._publish_active_command)
        self.publish_interval_ms = max(
            1, round(1000.0 / self.publish_rate)
        )

    def _build_ui(self):
        root = QWidget()
        layout = QVBoxLayout(root)
        layout.setContentsMargins(22, 20, 22, 20)
        layout.setSpacing(14)

        title = QLabel('麦克纳姆底盘按住控制')
        title_font = QFont()
        title_font.setPointSize(18)
        title_font.setBold(True)
        title.setFont(title_font)
        layout.addWidget(title)

        subtitle = QLabel(
            '按住方向按钮时才会发布速度，松开立即停车。'
            '空闲时不发送 /cmd_vel。'
        )
        subtitle.setWordWrap(True)
        subtitle.setStyleSheet('color: #59636f;')
        layout.addWidget(subtitle)

        panel = QFrame()
        panel.setFrameShape(QFrame.StyledPanel)
        panel.setStyleSheet(
            'QFrame { background: #f4f6f8; border: 1px solid #d8dde3; '
            'border-radius: 8px; }'
        )
        grid = QGridLayout(panel)
        grid.setContentsMargins(16, 14, 16, 14)
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(14)

        headers = ('速度设置', '速度滑块', '精确值', '按住控制')
        for column, text in enumerate(headers):
            label = QLabel(text)
            label.setStyleSheet('font-weight: 600; color: #303841;')
            grid.addWidget(label, 0, column)

        button_rows = {
            FORWARD: (
                ('forward', '后退', 'backward', -1.0),
                ('forward', '前进', 'forward', 1.0),
            ),
            LATERAL: (
                ('lateral', '右移', 'right', -1.0),
                ('lateral', '左移', 'left', 1.0),
            ),
            ROTATION: (
                ('rotation', '右转', 'clockwise', -1.0),
                ('rotation', '左转', 'counterclockwise', 1.0),
            ),
        }

        for row, spec in enumerate(self.specs.values(), start=1):
            name = QLabel(f'{spec.label}\n{spec.unit}')
            name.setMinimumWidth(100)
            grid.addWidget(name, row, 0)

            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, SLIDER_STEPS)
            slider.setSingleStep(1)
            slider.setPageStep(25)
            slider.setTickInterval(SLIDER_STEPS // 10)
            slider.setTickPosition(QSlider.TicksBelow)
            slider.setMinimumWidth(300)
            slider.setValue(self._speed_to_slider(spec, spec.initial))
            grid.addWidget(slider, row, 1)
            self.sliders[spec.key] = slider

            spinbox = QDoubleSpinBox()
            spinbox.setRange(spec.minimum, spec.maximum)
            spinbox.setSingleStep(0.05)
            spinbox.setDecimals(2)
            spinbox.setSuffix(f' {spec.unit}')
            spinbox.setValue(spec.initial)
            spinbox.setMinimumWidth(135)
            grid.addWidget(spinbox, row, 2)
            self.spinboxes[spec.key] = spinbox

            slider.valueChanged.connect(
                lambda value, axis=spec: self._slider_changed(axis, value)
            )
            spinbox.valueChanged.connect(
                lambda value, axis=spec: self._spinbox_changed(axis, value)
            )

            button_box = QGridLayout()
            button_box.setHorizontalSpacing(8)
            for column, button_spec in enumerate(button_rows[spec.key]):
                _, text, direction, sign = button_spec
                button = QPushButton(text)
                button.setMinimumHeight(44)
                button.setMinimumWidth(82)
                button.setToolTip('按住运动，松开立即停止')
                button.pressed.connect(
                    lambda s=sign, a=spec.key: self._start_motion(a, s)
                )
                button.released.connect(self._stop_motion)
                button_box.addWidget(button, 0, column)
                self.direction_buttons[direction] = button
            grid.addLayout(button_box, row, 3)

        layout.addWidget(panel)

        stop_button = QPushButton('停止 / 全部归零')
        stop_button.setMinimumHeight(54)
        stop_button.setStyleSheet(
            'QPushButton { background: #b83232; color: white; '
            'border-radius: 7px; font-size: 15px; font-weight: 700; } '
            'QPushButton:hover { background: #982727; }'
        )
        stop_button.clicked.connect(self._stop_motion)
        layout.addWidget(stop_button)

        self.status_label = QLabel()
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet(
            'QLabel { background: white; border: 1px solid #d8dde3; '
            'border-radius: 6px; padding: 10px; color: #37424d; }'
        )
        layout.addWidget(self.status_label)
        self._update_idle_status()

        self.setCentralWidget(root)

    def _speed_to_slider(self, spec, speed):
        bounded = min(max(speed, spec.minimum), spec.maximum)
        ratio = (bounded - spec.minimum) / (spec.maximum - spec.minimum)
        return round(ratio * SLIDER_STEPS)

    def _slider_to_speed(self, spec, value):
        ratio = value / SLIDER_STEPS
        return spec.minimum + ratio * (spec.maximum - spec.minimum)

    def _slider_changed(self, spec, value):
        spinbox = self.spinboxes[spec.key]
        spinbox.blockSignals(True)
        spinbox.setValue(self._slider_to_speed(spec, value))
        spinbox.blockSignals(False)
        self._refresh_active_status()

    def _spinbox_changed(self, spec, value):
        slider = self.sliders[spec.key]
        slider.blockSignals(True)
        slider.setValue(self._speed_to_slider(spec, value))
        slider.blockSignals(False)
        self._refresh_active_status()

    def _command_for(self, axis, sign):
        speed = self.spinboxes[axis].value() * sign
        if axis == FORWARD:
            return speed, 0.0, 0.0
        if axis == LATERAL:
            return 0.0, speed, 0.0
        return 0.0, 0.0, speed

    def _start_motion(self, axis, sign):
        if self.closing or self.node is None or not rclpy.ok():
            return
        self.active_direction = (axis, sign)
        self._publish_active_command()
        self.publish_timer.start(self.publish_interval_ms)
        self._refresh_active_status()

    def _stop_motion(self):
        was_active = self.active_direction is not None
        self.active_direction = None
        self.publish_timer.stop()
        if was_active:
            self._publish_twist(Twist())
        self._update_idle_status()

    def _publish_active_command(self):
        if (
            self.active_direction is None
            or self.closing
            or self.node is None
            or not rclpy.ok()
        ):
            return
        forward, lateral, rotation = self._command_for(
            *self.active_direction
        )
        message = Twist()
        message.linear.x = forward
        message.linear.y = lateral
        message.angular.z = rotation
        self._publish_twist(message)

    def _publish_twist(self, message):
        if self.node is not None and rclpy.ok():
            self.publisher.publish(message)

    def _refresh_active_status(self):
        if self.active_direction is None:
            self._update_idle_status()
            return
        forward, lateral, rotation = self._command_for(
            *self.active_direction
        )
        self.status_label.setText(
            f'正在按住控制（{self.publish_rate:.0f} Hz）：'
            f'vx={forward:+.2f} m/s，'
            f'vy={lateral:+.2f} m/s，'
            f'wz={rotation:+.2f} rad/s'
        )

    def _update_idle_status(self):
        if not hasattr(self, 'status_label'):
            return
        self.status_label.setText(
            '未按键：底盘停止，不发送 /cmd_vel。'
            '按住方向按钮开始运动，松开立即停车。'
        )

    def _spin_ros(self):
        if self.closing or self.node is None or not rclpy.ok():
            self.spin_timer.stop()
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except Exception:
            self.spin_timer.stop()

    def keyPressEvent(self, event):
        if event.key() in (Qt.Key_Escape, Qt.Key_Space):
            self._stop_motion()
            event.accept()
            return
        super().keyPressEvent(event)

    def changeEvent(self, event):
        if event.type() == QEvent.WindowDeactivate:
            self._stop_motion()
        super().changeEvent(event)

    def closeEvent(self, event):
        self._stop_motion()
        self.closing = True
        self.spin_timer.stop()
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if rclpy.ok():
            rclpy.shutdown()
        event.accept()


def main(args=None):
    rclpy.init(args=args)
    node = Node('chassis_gui')
    node.declare_parameter('publish_rate', 20.0)
    node.declare_parameter('linear_limit', 1.0)
    node.declare_parameter('angular_limit', 1.5)

    app = QApplication(sys.argv)
    signal.signal(signal.SIGINT, lambda *_args: app.quit())
    window = ChassisGui(node)
    window.show()
    exit_code = app.exec_()
    window._stop_motion()
    if window.node is not None:
        window.node.destroy_node()
        window.node = None
    if rclpy.ok():
        rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
