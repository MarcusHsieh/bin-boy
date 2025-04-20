#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from PyQt5.QtWidgets import (QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLabel, QSizePolicy)
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QPolygonF
from PyQt5.QtCore import Qt, QPointF, QTimer, QRectF, pyqtSignal

class JoystickWidget(QWidget):
    joystickMoved = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 200)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._knob_pos = QPointF(0.0, 0.0) # CENTER (-1 to 1 range)
        self._mouse_pressed = False
        self._center = QPointF(0.0, 0.0)
        self._radius = 0.0

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        size = min(self.width(), self.height())
        self._radius = (size / 2.0) * 0.9
        self._center = QPointF(self.width() / 2.0, self.height() / 2.0)
        outer_radius = self._radius
        inner_radius = outer_radius * 0.3

        painter.setPen(QPen(Qt.gray, 2))
        painter.setBrush(QBrush(QColor(240, 240, 240)))
        painter.drawEllipse(self._center, outer_radius, outer_radius)

        painter.setPen(QPen(Qt.lightGray, 1))
        painter.drawLine(QPointF(self._center.x(), self._center.y() - outer_radius),
                         QPointF(self._center.x(), self._center.y() + outer_radius))
        painter.drawLine(QPointF(self._center.x() - outer_radius, self._center.y()),
                         QPointF(self._center.x() + outer_radius, self._center.y()))

        knob_widget_pos = QPointF(
            self._center.x() + self._knob_pos.x() * outer_radius,
            self._center.y() + self._knob_pos.y() * outer_radius
        )

        painter.setPen(QPen(Qt.black, 1))
        painter.setBrush(QBrush(Qt.darkGray))
        painter.drawEllipse(knob_widget_pos, inner_radius, inner_radius)

    def mousePressEvent(self, event):
        if self._is_within_radius(event.pos()):
            self._mouse_pressed = True
            self._update_knob_pos(event.pos())
        else:
            self._mouse_pressed = False

    def mouseMoveEvent(self, event):
        if self._mouse_pressed:
            self._update_knob_pos(event.pos())

    def mouseReleaseEvent(self, event):
        if self._mouse_pressed:
            self._mouse_pressed = False
            self._reset_knob()

    def resizeEvent(self, event):
        self.update()

    def _is_within_radius(self, pos):
        # move joystick check
        delta = pos - self._center
        return (delta.x()**2 + delta.y()**2) <= self._radius**2

    def _update_knob_pos(self, pos):
        delta = pos - self._center
        distance = (delta.x()**2 + delta.y()**2)**0.5

        # normal + clamp
        if distance <= 1e-6:
             norm_x = 0.0
             norm_y = 0.0
        elif distance <= self._radius:
            norm_x = delta.x() / self._radius
            norm_y = delta.y() / self._radius
        else:
            norm_x = delta.x() / distance
            norm_y = delta.y() / distance

        self._knob_pos = QPointF(norm_x, norm_y) #negative y?

        self.update()
        self.joystickMoved.emit(self._knob_pos.x(), self._knob_pos.y())

    def _reset_knob(self):
        self._knob_pos = QPointF(0.0, 0.0)
        self.update()
        self.joystickMoved.emit(0.0, 0.0)


class KiwiDriveGui(Node, QWidget):
    def __init__(self):
        QWidget.__init__(self)
        Node.__init__(self, 'kiwi_drive_gui')

        self.setWindowTitle('Kiwi Drive Joystick')
        self.setGeometry(100, 100, 350, 400) # x, y, width, height

        # ROS publisher
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Publishing Twist commands to /cmd_vel")

        # curr velocity state
        self._linear_x = 0.0
        self._linear_y = 0.0
        self._angular_z = 0.0
        self._max_linear_speed = 0.5 # m/s TUNE
        self._max_angular_speed = 1.0 # rad/s TUNE

        # --- GUI Elements ---
        self.layout = QVBoxLayout(self)

        # joystick
        self.joystick = JoystickWidget()
        self.joystick.joystickMoved.connect(self.update_linear_velocity)
        self.layout.addWidget(self.joystick)

        # rotation buttons
        self.rotate_layout = QHBoxLayout()
        self.btn_rotate_left = QPushButton("Rotate Left (<<)")
        self.btn_rotate_right = QPushButton("Rotate Right (>>)")
        self.btn_rotate_left.pressed.connect(self.start_rotate_left)
        self.btn_rotate_left.released.connect(self.stop_rotate)
        self.btn_rotate_right.pressed.connect(self.start_rotate_right)
        self.btn_rotate_right.released.connect(self.stop_rotate)
        self.rotate_layout.addWidget(self.btn_rotate_left)
        self.rotate_layout.addWidget(self.btn_rotate_right)
        self.layout.addLayout(self.rotate_layout)

        # status labels
        self.status_label = QLabel("Status: Idle")
        self.layout.addWidget(self.status_label)
        self.vel_label = QLabel("Vx: 0.00, Vy: 0.00, Wz: 0.00")
        self.layout.addWidget(self.vel_label)

        # --- ROS Timer for Qt Event Loop ---
        # Use a ROS timer to periodically process Qt events
        self.qt_timer_period = 0.02 # seconds (50 Hz)
        self.qt_timer = self.create_timer(self.qt_timer_period, self.process_qt_events)
        self.get_logger().info(f"Using ROS Timer for Qt event processing ({1.0/self.qt_timer_period:.1f} Hz)")

    def process_qt_events(self):
        """Process pending Qt events."""
        QApplication.instance().processEvents()

    def update_linear_velocity(self, joy_x, joy_y):
        """Update linear velocity based on joystick signal."""
        # Map joystick (-1 to 1) to linear velocities
        # Note the swap: joy_x controls Vy (side), joy_y controls Vx (fwd/back)
        self._linear_x = joy_y * self._max_linear_speed
        self._linear_y = joy_x * self._max_linear_speed # Negative if you want left stick = positive Y
        self.publish_command()
        self.update_status_labels()

    def start_rotate_left(self):
        self._angular_z = self._max_angular_speed
        self.publish_command()
        self.update_status_labels()

    def start_rotate_right(self):
        self._angular_z = -self._max_angular_speed
        self.publish_command()
        self.update_status_labels()

    def stop_rotate(self):
        self._angular_z = 0.0
        self.publish_command()
        self.update_status_labels()

    def publish_command(self):
        """Publish the current combined velocity as a Twist message."""
        msg = Twist()
        msg.linear.x = self._linear_x
        msg.linear.y = self._linear_y
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self._angular_z
        self.publisher_.publish(msg)

    def update_status_labels(self):
        """Update the GUI labels."""
        status = "Driving"
        if abs(self._linear_x) < 0.01 and abs(self._linear_y) < 0.01 and abs(self._angular_z) < 0.01:
            status = "Idle"
        self.status_label.setText(f"Status: {status}")
        self.vel_label.setText(f"Vx: {self._linear_x:.2f}, Vy: {self._linear_y:.2f}, Wz: {self._angular_z:.2f}")

    def closeEvent(self, event):
        """Ensure ROS node cleanup on window close."""
        self.get_logger().info("GUI closing, shutting down ROS node.")
        self.destroy_node()
        QWidget.closeEvent(self, event)


def main(args=None):
    rclpy.init(args=args)

    # Need a QApplication instance
    app = QApplication(sys.argv)

    # Create and show the GUI/Node
    gui_node = KiwiDriveGui()
    gui_node.show()

    try:
        rclpy.spin(gui_node)
    except KeyboardInterrupt:
        gui_node.get_logger().info("KeyboardInterrupt, shutting down.")
    except Exception as e:
        gui_node.get_logger().error(f"Error during spin: {e}", exc_info=True)
    finally:
        if rclpy.ok() and gui_node.is_valid():
            gui_node.destroy_node()
        if rclpy.ok():
             rclpy.shutdown()
        sys.exit()


if __name__ == '__main__':
    main()