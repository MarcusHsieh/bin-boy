#!/usr/bin/env python3
"""
Person Tracker Widget
Qt-based dashboard for person tracking visualization
"""

from python_qt_binding.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout,
                                          QLabel, QTableWidget, QTableWidgetItem,
                                          QProgressBar, QGroupBox, QPushButton,
                                          QHeaderView)
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QFont, QColor
from bin_boy_interfaces.msg import PersonTrackingArray, CameraPerformance
import math


class PersonTrackerWidget(QWidget):
    """Main widget for person tracking dashboard"""

    def __init__(self, node):
        super(PersonTrackerWidget, self).__init__()
        self.setObjectName('PersonTrackerWidget')
        self.setWindowTitle('Person Tracker Dashboard')

        self.node = node
        self.latest_data = None
        self.latest_camera_perf = None

        # Setup UI
        self.setup_ui()

        # Subscribe to tracking data
        self.subscription = self.node.create_subscription(
            PersonTrackingArray,
            'person/tracking_data',
            self.tracking_callback,
            10
        )

        # Subscribe to camera performance data
        self.camera_perf_subscription = self.node.create_subscription(
            CameraPerformance,
            '/camera/performance',
            self.camera_perf_callback,
            10
        )

        # Timer to update UI
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_ui)
        self.update_timer.start(100)  # 10 Hz

    def setup_ui(self):
        """Setup the Qt UI layout"""
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)

        # === Header Section ===
        header_layout = QHBoxLayout()

        # Title
        title_label = QLabel("BIN-BOY PERSON TRACKING")
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title_label.setFont(title_font)
        header_layout.addWidget(title_label)

        header_layout.addStretch()

        # Status indicator
        self.status_label = QLabel("State: IDLE")
        status_font = QFont()
        status_font.setPointSize(12)
        self.status_label.setFont(status_font)
        header_layout.addWidget(self.status_label)

        main_layout.addLayout(header_layout)

        # === Camera Performance Section ===
        camera_group = QGroupBox("Camera Performance (TensorRT)")
        camera_layout = QHBoxLayout()

        self.inference_time_label = QLabel("Inference: -")
        self.fps_label = QLabel("FPS: -")
        self.gpu_detections_label = QLabel("Detections: -")

        camera_layout.addWidget(self.inference_time_label)
        camera_layout.addWidget(self.fps_label)
        camera_layout.addWidget(self.gpu_detections_label)
        camera_layout.addStretch()

        camera_group.setLayout(camera_layout)
        main_layout.addWidget(camera_group)

        # === Target Info Section ===
        target_group = QGroupBox("Target Person")
        target_layout = QVBoxLayout()

        target_info_layout = QHBoxLayout()
        self.target_id_label = QLabel("ID: -")
        self.target_distance_label = QLabel("Distance: -")
        self.target_angle_label = QLabel("Angle: -")

        target_info_layout.addWidget(self.target_id_label)
        target_info_layout.addWidget(self.target_distance_label)
        target_info_layout.addWidget(self.target_angle_label)
        target_info_layout.addStretch()

        target_layout.addLayout(target_info_layout)
        target_group.setLayout(target_layout)
        main_layout.addWidget(target_group)

        # === People Table ===
        people_group = QGroupBox("Detected People")
        people_layout = QVBoxLayout()

        self.people_table = QTableWidget()
        self.people_table.setColumnCount(6)
        self.people_table.setHorizontalHeaderLabels([
            "ID", "Target", "Confidence", "Color Match", "Score", "BBox"
        ])

        # Table formatting
        header = self.people_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)  # ID
        header.setSectionResizeMode(1, QHeaderView.ResizeToContents)  # Target
        header.setSectionResizeMode(2, QHeaderView.Stretch)           # Confidence
        header.setSectionResizeMode(3, QHeaderView.Stretch)           # Color
        header.setSectionResizeMode(4, QHeaderView.Stretch)           # Score
        header.setSectionResizeMode(5, QHeaderView.ResizeToContents)  # BBox

        self.people_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.people_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.people_table.setAlternatingRowColors(True)

        people_layout.addWidget(self.people_table)
        people_group.setLayout(people_layout)
        main_layout.addWidget(people_group)

        # === Controls ===
        controls_layout = QHBoxLayout()

        self.reset_button = QPushButton("Reset Tracking")
        self.reset_button.clicked.connect(self.reset_tracking)
        controls_layout.addWidget(self.reset_button)

        controls_layout.addStretch()

        self.person_count_label = QLabel("People: 0")
        controls_layout.addWidget(self.person_count_label)

        main_layout.addLayout(controls_layout)

        self.setLayout(main_layout)
        self.resize(800, 600)

    def tracking_callback(self, msg):
        """Receive tracking data from ROS topic"""
        self.latest_data = msg

    def camera_perf_callback(self, msg):
        """Receive camera performance data from ROS topic"""
        self.latest_camera_perf = msg

    def update_ui(self):
        """Update UI with latest tracking data"""
        if self.latest_data is None:
            return

        msg = self.latest_data

        # Update status
        self.status_label.setText(f"State: {msg.tracking_state}")
        state_color = self.get_state_color(msg.tracking_state)
        self.status_label.setStyleSheet(f"color: {state_color}; font-weight: bold;")

        # Update target info
        target_person = None
        for person in msg.persons:
            if person.is_target:
                target_person = person
                break

        if target_person:
            self.target_id_label.setText(f"ID: #{target_person.id}")

            if target_person.distance >= 0:
                self.target_distance_label.setText(f"Distance: {target_person.distance:.2f}m")
            else:
                self.target_distance_label.setText("Distance: -")

            if target_person.distance >= 0:
                angle_deg = math.degrees(target_person.angle)
                self.target_angle_label.setText(f"Angle: {angle_deg:.0f}°")
            else:
                self.target_angle_label.setText("Angle: -")
        else:
            self.target_id_label.setText("ID: -")
            self.target_distance_label.setText("Distance: -")
            self.target_angle_label.setText("Angle: -")

        # Update people count
        self.person_count_label.setText(f"People: {len(msg.persons)}")

        # Update camera performance metrics
        if self.latest_camera_perf:
            perf = self.latest_camera_perf
            self.inference_time_label.setText(f"Inference: {perf.inference_time_ms:.1f} ms")
            self.fps_label.setText(f"FPS: {perf.fps:.1f}")
            self.gpu_detections_label.setText(f"Detections: {perf.gpu_detections}")
        else:
            self.inference_time_label.setText("Inference: -")
            self.fps_label.setText("FPS: -")
            self.gpu_detections_label.setText("Detections: -")

        # Update people table
        self.people_table.setRowCount(len(msg.persons))

        for i, person in enumerate(msg.persons):
            # ID
            id_item = QTableWidgetItem(f"#{person.id}")
            if person.is_target:
                id_item.setForeground(QColor(200, 0, 0))
                font = id_item.font()
                font.setBold(True)
                id_item.setFont(font)
            self.people_table.setItem(i, 0, id_item)

            # Target indicator
            target_item = QTableWidgetItem("★" if person.is_target else "")
            target_item.setTextAlignment(Qt.AlignCenter)
            if person.is_target:
                target_item.setForeground(QColor(200, 0, 0))
            self.people_table.setItem(i, 1, target_item)

            # Confidence bar
            conf_widget = self.create_progress_bar(person.confidence, "green")
            self.people_table.setCellWidget(i, 2, conf_widget)

            # Color match bar
            if person.color_similarity >= 0:
                color = "green" if person.color_similarity >= 0.6 else "orange"
                color_widget = self.create_progress_bar(person.color_similarity, color)
            else:
                color_widget = QLabel("-")
                color_widget.setAlignment(Qt.AlignCenter)
            self.people_table.setCellWidget(i, 3, color_widget)

            # Match score bar
            if person.match_score >= 0:
                score_widget = self.create_progress_bar(person.match_score, "blue")
            else:
                score_widget = QLabel("-")
                score_widget.setAlignment(Qt.AlignCenter)
            self.people_table.setCellWidget(i, 4, score_widget)

            # BBox
            bbox_text = f"[{person.x1:.0f}, {person.y1:.0f}, {person.x2:.0f}, {person.y2:.0f}]"
            bbox_item = QTableWidgetItem(bbox_text)
            bbox_item.setFont(QFont("Monospace", 8))
            self.people_table.setItem(i, 5, bbox_item)

    def create_progress_bar(self, value, color_name):
        """Create a progress bar widget with value and color"""
        container = QWidget()
        layout = QHBoxLayout()
        layout.setContentsMargins(2, 2, 2, 2)

        bar = QProgressBar()
        bar.setMinimum(0)
        bar.setMaximum(100)
        bar.setValue(int(value * 100))
        bar.setTextVisible(True)
        bar.setFormat(f"{value:.2f}")

        # Color stylesheet
        if color_name == "green":
            bar.setStyleSheet("QProgressBar::chunk { background-color: #4CAF50; }")
        elif color_name == "blue":
            bar.setStyleSheet("QProgressBar::chunk { background-color: #2196F3; }")
        elif color_name == "orange":
            bar.setStyleSheet("QProgressBar::chunk { background-color: #FF9800; }")
        elif color_name == "red":
            bar.setStyleSheet("QProgressBar::chunk { background-color: #F44336; }")

        layout.addWidget(bar)
        container.setLayout(layout)
        return container

    def get_state_color(self, state):
        """Get color for tracking state"""
        state_colors = {
            "IDLE": "gray",
            "TRACKING": "green",
            "LOST": "orange",
            "REACQUIRING": "red",
            "LOCKED": "blue"
        }
        return state_colors.get(state, "black")

    def reset_tracking(self):
        """Reset tracking (publish a reset command)"""
        # TODO: Implement reset service call if needed
        self.node.get_logger().info("Reset tracking requested")

    def shutdown(self):
        """Cleanup on shutdown"""
        self.update_timer.stop()
        if self.subscription:
            self.node.destroy_subscription(self.subscription)
        if self.camera_perf_subscription:
            self.node.destroy_subscription(self.camera_perf_subscription)
