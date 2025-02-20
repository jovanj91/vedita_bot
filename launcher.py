import sys
import subprocess
import os
import signal
import re
from PyQt6.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QLabel, QTextEdit, QLineEdit, QHBoxLayout, QCheckBox
)
from PyQt6.QtCore import QThread, pyqtSignal


class ROS2Launcher(QWidget):
    def __init__(self):
        super().__init__()

        self.ros_ws_path = os.path.expanduser("~/ros2_ws")

        self.setWindowTitle("ROS 2 Launcher")
        self.setGeometry(100, 100, 700, 600)

        layout = QVBoxLayout()

        self.status_label = QLabel("Set parameters and launch ROS 2 processes")
        layout.addWidget(self.status_label)

        # Use Sim Time checkbox (set default to checked)
        self.sim_time_checkbox = QCheckBox("Use Sim Time")
        self.sim_time_checkbox.setChecked(True)
        self.sim_time_checkbox.stateChanged.connect(self.toggle_world_input)
        layout.addWidget(self.sim_time_checkbox)

        # World file input (hidden if use_sim_time is False)
        self.world_layout = QHBoxLayout()
        self.world_label = QLabel("World File:")
        self.world_input = QLineEdit("./src/vedita_bot/worlds/test.world")
        self.world_layout.addWidget(self.world_label)
        self.world_layout.addWidget(self.world_input)
        layout.addLayout(self.world_layout)

        # Map file input
        map_layout = QHBoxLayout()
        map_layout.addWidget(QLabel("Map File:"))
        self.map_input = QLineEdit("./src/vedita_bot/maps/test.yaml")
        map_layout.addWidget(self.map_input)
        layout.addLayout(map_layout)

        # Buttons
        self.launch_sim_button = QPushButton("Launch Simulation")
        self.launch_sim_button.clicked.connect(self.launch_sim)
        layout.addWidget(self.launch_sim_button)

        self.stop_sim_button = QPushButton("Stop Simulation")
        self.stop_sim_button.clicked.connect(self.stop_sim)
        self.stop_sim_button.setDisabled(True)  # Initially disabled
        layout.addWidget(self.stop_sim_button)

        self.launch_localization_button = QPushButton("Launch Localization")
        self.launch_localization_button.clicked.connect(self.launch_localization)
        self.launch_localization_button.setDisabled(True)  # Initially disabled
        layout.addWidget(self.launch_localization_button)

        self.stop_localization_button = QPushButton("Stop Localization")
        self.stop_localization_button.clicked.connect(self.stop_localization)
        self.stop_localization_button.setDisabled(True)  # Initially disabled
        layout.addWidget(self.stop_localization_button)

        self.launch_navigation_button = QPushButton("Launch Navigation")
        self.launch_navigation_button.clicked.connect(self.launch_navigation)
        self.launch_navigation_button.setDisabled(True)  # Initially disabled
        layout.addWidget(self.launch_navigation_button)

        self.stop_navigation_button = QPushButton("Stop Navigation")
        self.stop_navigation_button.clicked.connect(self.stop_navigation)
        self.stop_navigation_button.setDisabled(True)  # Initially disabled
        layout.addWidget(self.stop_navigation_button)

        self.log_output = QTextEdit()
        self.log_output.setReadOnly(True)
        layout.addWidget(self.log_output)

        self.setLayout(layout)

        # Store ROS processes
        self.sim_process = None
        self.localization_process = None
        self.navigation_process = None
        self.log_thread = None

    def toggle_world_input(self):
        """ Hide world input if use_sim_time is unchecked """
        is_checked = self.sim_time_checkbox.isChecked()
        self.world_label.setVisible(is_checked)
        self.world_input.setVisible(is_checked)

    def get_use_sim_time(self):
        return "true" if self.sim_time_checkbox.isChecked() else "false"

    def launch_sim(self):
        if self.sim_process is None:
            world_file = self.world_input.text()
            command = f"ros2 launch vedita_bot launch_sim.launch.py world:={world_file}"
            self.sim_process = self.start_ros_process(command)
            self.status_label.setText("Simulation running...")

            # Disable launch button, enable stop button, enable next step
            self.launch_sim_button.setDisabled(True)
            self.stop_sim_button.setDisabled(False)
            self.launch_localization_button.setDisabled(False)

    def stop_sim(self):
        self.stop_ros_process(self.sim_process, "Simulation stopped.")
        self.sim_process = None

        # Disable stop button, disable next steps
        self.stop_sim_button.setDisabled(True)
        self.launch_sim_button.setDisabled(False)
        self.launch_localization_button.setDisabled(True)

    def launch_localization(self):
        if self.localization_process is None:
            map_file = self.map_input.text()
            use_sim_time = self.get_use_sim_time()
            command = f"ros2 launch vedita_bot localization_launch.py map:={map_file} use_sim_time:={use_sim_time}"
            self.localization_process = self.start_ros_process(command)
            self.status_label.setText("Localization running...")

            # Disable launch button, enable stop button, enable next step
            self.launch_localization_button.setDisabled(True)
            self.stop_localization_button.setDisabled(False)
            self.launch_navigation_button.setDisabled(False)

    def stop_localization(self):
        self.stop_ros_process(self.localization_process, "Localization stopped.")
        self.localization_process = None

        # Disable stop button, disable next steps
        self.stop_localization_button.setDisabled(True)
        self.launch_navigation_button.setDisabled(True)

    def launch_navigation(self):
        if self.navigation_process is None:
            use_sim_time = self.get_use_sim_time()
            command = f"ros2 launch vedita_bot navigation_launch.py use_sim_time:={use_sim_time} map_subscribe_transient_local:=true"
            self.navigation_process = self.start_ros_process(command)
            self.status_label.setText("Navigation running...")

            # Disable launch button, enable stop button
            self.launch_navigation_button.setDisabled(True)
            self.stop_navigation_button.setDisabled(False)

    def stop_navigation(self):
        self.stop_ros_process(self.navigation_process, "Navigation stopped.")
        self.navigation_process = None

        # Disable stop button, enable previous step stop button
        self.stop_navigation_button.setDisabled(True)
        self.stop_localization_button.setDisabled(False)

    def start_ros_process(self, command):
        process = subprocess.Popen(
            ["bash", "-c", f"source {self.ros_ws_path}/install/setup.bash && {command}"],
            cwd=self.ros_ws_path,
            executable="/bin/bash",
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            preexec_fn=os.setsid
        )
        self.log_thread = LogReader(process)
        self.log_thread.new_log.connect(self.update_log)
        self.log_thread.start()
        return process

    def stop_ros_process(self, process, message):
        if process is not None:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            process.wait()
            self.status_label.setText(message)

    def ansi_to_html(self, text):
        """Convert ANSI escape codes to HTML for colored logs."""
        ansi_colors = {
            '37': 'white',
            '90': 'gray',
            '91': 'red',
            '92': 'green',
            '93': 'yellow',
            '94': 'blue',
            '95': 'magenta',
            '96': 'cyan',
            '97': 'white',
        }

        # Regex to match ANSI escape codes
        ansi_regex = re.compile(r'\x1b\[([0-9;]+)m')

        # Replace ANSI codes with HTML tags
        def replace_ansi(match):
            codes = match.group(1).split(';')
            html_tags = []
            for code in codes:
                if code in ansi_colors:
                    html_tags.append(f'<span style="color: {ansi_colors[code]};">')
            return ''.join(html_tags)

        # Replace ANSI codes and close spans
        html_text = ansi_regex.sub(replace_ansi, text)
        html_text = html_text.replace('\x1b[0m', '</span>')  # Reset code
        return html_text

    def update_log(self, log_text):
        """Update the log output with colored text."""
        html_text = self.ansi_to_html(log_text)
        self.log_output.append(html_text)

    def closeEvent(self, event):
        """Ensure all ROS 2 processes are stopped before closing the application."""
        self.stop_navigation()
        self.stop_localization()
        self.stop_sim()
        event.accept()  # Allow window to close


class LogReader(QThread):
    new_log = pyqtSignal(str)

    def __init__(self, process):
        super().__init__()
        self.process = process

    def run(self):
        while True:
            output = self.process.stdout.readline()
            if output:
                self.new_log.emit(output.strip())
            if self.process.poll() is not None:
                break


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ROS2Launcher()
    window.show()
    sys.exit(app.exec())