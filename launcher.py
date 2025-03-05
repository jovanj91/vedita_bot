import sys
import subprocess
import os
import signal
from dotenv import load_dotenv
import re
from PyQt6.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QLabel, QTextEdit, QLineEdit, QHBoxLayout, QCheckBox
)
from PyQt6.QtCore import QThread, pyqtSignal
import paramiko, atexit


class ROS2Launcher(QWidget):
    def __init__(self):
        super().__init__()

        self.ros_ws_path = os.path.expanduser("~/ros2_ws")

        self.setWindowTitle("Vedita Launcher")
        self.setGeometry(100, 100, 700, 600)

        layout = QVBoxLayout()

        self.status_label = QLabel("Set parameters and launch ROS 2 processes")
        layout.addWidget(self.status_label)

        # Use Sim Time checkbox (set default to checked)
        self.sim_time_checkbox = QCheckBox("Running in simulation mode")
        self.sim_time_checkbox.setChecked(True)
        self.sim_time_checkbox.stateChanged.connect(self.toggle_use_sim_time)
        layout.addWidget(self.sim_time_checkbox)

        # World file input (hidden if use_sim_time is False)
        self.world_layout = QHBoxLayout()
        self.world_label = QLabel("World File:")
        self.world_input = QLineEdit("./src/vedita_bot/worlds/test.world")
        self.world_layout.addWidget(self.world_label)
        self.world_layout.addWidget(self.world_input)
        layout.addLayout(self.world_layout)

        # Buttons
        self.launch_sim_button = QPushButton("Launch Simulation")
        self.launch_sim_button.clicked.connect(self.launch_sim)
        layout.addWidget(self.launch_sim_button)

        self.stop_sim_button = QPushButton("Stop Simulation")
        self.stop_sim_button.clicked.connect(self.stop_sim)
        self.stop_sim_button.setDisabled(True)  # Initially disabled
        layout.addWidget(self.stop_sim_button)

        # SSH Buttons
        self.launch_robot_button = QPushButton("Launch Robot")
        self.launch_robot_button.clicked.connect(self.launch_robot)
        self.launch_robot_button.setVisible(False)  # Initially hidden
        layout.addWidget(self.launch_robot_button)

        self.stop_robot_button = QPushButton("Stop Robot")
        self.stop_robot_button.clicked.connect(self.stop_robot)
        self.stop_robot_button.setDisabled(True)  # Initially disabled
        self.stop_robot_button.setVisible(False)  # Initially hidden
        layout.addWidget(self.stop_robot_button)

        # Map file input
        map_layout = QHBoxLayout()
        map_layout.addWidget(QLabel("Map File:"))
        self.map_input = QLineEdit("./src/vedita_bot/maps/lantai8baru.yaml")
        map_layout.addWidget(self.map_input)
        layout.addLayout(map_layout)

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

        self.launch_person_follow_button = QPushButton("Launch Person Follower")
        self.launch_person_follow_button.clicked.connect(self.launch_person_follower)
        self.launch_person_follow_button.setDisabled(True)  # Initially disabled
        self.launch_person_follow_button.setVisible(False)
        layout.addWidget(self.launch_person_follow_button)

        self.stop_person_follow_button = QPushButton("Stop Person Follower")
        self.stop_person_follow_button.clicked.connect(self.stop_person_follower)
        self.stop_person_follow_button.setDisabled(True)  # Initially disabled
        self.stop_person_follow_button.setVisible(False)
        layout.addWidget(self.stop_person_follow_button)

        self.log_output = QTextEdit()
        self.log_output.setReadOnly(True)
        layout.addWidget(self.log_output)
        
        self.launch_joystick_button = QPushButton("Launch Joystick Control")
        self.launch_joystick_button.clicked.connect(self.launch_joystick)
        self.launch_joystick_button.setVisible(False)  # Initially hidden
        layout.addWidget(self.launch_joystick_button)

        self.stop_joystick_button = QPushButton("Stop Joystick Control")
        self.stop_joystick_button.clicked.connect(self.stop_joystick)
        self.stop_joystick_button.setDisabled(True)  # Initially disabled
        self.stop_joystick_button.setVisible(False)  # Initially hidden
        layout.addWidget(self.stop_joystick_button)

        self.activate_camera_button = QPushButton("Activate Camera")
        self.activate_camera_button.clicked.connect(self.launch_camera)
        self.activate_camera_button.setVisible(False)  # Initially hidden
        layout.addWidget(self.activate_camera_button)

        self.deactivate_camera_button = QPushButton("Deactivate Camera")
        self.deactivate_camera_button.clicked.connect(self.stop_camera)
        self.deactivate_camera_button.setDisabled(True)  # Initially disabled
        self.deactivate_camera_button.setVisible(False)  # Initially hidden
        layout.addWidget(self.deactivate_camera_button)


        self.setLayout(layout)
        load_dotenv()
        # Store ROS processes
        self.sim_process = None
        self.localization_process = None
        self.navigation_process = None
        self.joystick_process = None
        self.camera_process = None
        self.person_follower_process = None

        self.raspi_ip=os.getenv("RASPI_IP")
        self.raspi_username=os.getenv("RASPI_USERNAME")
        self.raspi_password=os.getenv("RASPI_PASSWORD")

        self.ssh_client = None
        self.robot_process = None
        self.log_thread = None

    def toggle_use_sim_time(self):
        is_checked = self.sim_time_checkbox.isChecked()
        self.world_label.setVisible(is_checked)
        self.world_input.setVisible(is_checked)
        self.launch_localization_button.setDisabled(is_checked)
        self.launch_sim_button.setVisible(is_checked)
        self.stop_sim_button.setVisible(is_checked)
        self.launch_joystick_button.setVisible(not is_checked)
        self.stop_joystick_button.setVisible(not is_checked)
        self.activate_camera_button.setVisible(not is_checked)
        self.deactivate_camera_button.setVisible(not is_checked)
        self.launch_person_follow_button.setVisible(not is_checked)
        self.stop_person_follow_button.setVisible(not is_checked)

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
            self.launch_joystick_button.setDisabled(False)
            self.launch_localization_button.setDisabled(False)
            

    def stop_sim(self):
        self.stop_ros_process(self.sim_process, "Simulation stopped.")
        self.sim_process = None

        # Disable stop button, disable next steps
        self.stop_sim_button.setDisabled(True)
        self.launch_sim_button.setDisabled(False)
        self.launch_joystick_button.setDisabled(True)
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
        self.launch_localization_button.setDisabled(False)
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
    
    def launch_joystick(self):
        if self.joystick_process is None:
            command = f"ros2 launch vedita_bot joystick.launch.py"
            self.joystick_process = self.start_ros_process(command)
            self.status_label.setText("Joystick running...")

            # Disable launch button, enable stop button
            self.launch_joystick_button.setDisabled(True)
            self.stop_joystick_button.setDisabled(False)

    def stop_joystick(self):
        self.stop_ros_process(self.joystick_process, "Joystick stopped.")
        self.joystick_process = None

        # Disable stop button, enable previous step stop button
        self.stop_joystick_button.setDisabled(True)
        self.launch_joystick_button.setDisabled(False)

    def launch_camera(self):
        if self.camera_process is None:
            command = f"ros2 launch vedita_bot camera.launch.py"
            self.camera_process = self.start_ros_process(command)
            self.status_label.setText("Camera activated...")

            # Disable launch button, enable stop button
            self.activate_camera_button.setDisabled(True)
            self.deactivate_camera_button.setDisabled(False)
            self.launch_person_follow_button.setDisabled(False)


    def stop_camera(self):
        self.stop_ros_process(self.camera_process, "Camera deactivated.")
        self.camera_process= None

        # Disable stop button, enable previous step stop button
        self.deactivate_camera_button.setDisabled(True)
        self.activate_camera_button.setDisabled(False)
        self.launch_person_follow_button.setDisabled(True)

    def launch_person_follower(self):
        if self.person_follower_process is None:
            command = f"ros2 launch vedita_bot person_follower_yolo.launch.py"
            self.person_follower_process = self.start_ros_process(command)
            self.status_label.setText("Person Follower Running..")

            # Disable launch button, enable stop button
            self.launch_person_follow_button.setDisabled(True)
            self.stop_person_follow_button.setDisabled(False)
            self.deactivate_camera_button.setDisabled(True)

    def stop_person_follower(self):
        self.stop_ros_process(self.person_follower_process, "Person Follower stopped.")
        self.person_follower_process = None

        # Disable stop button, enable previous step stop button
        self.stop_person_follow_button.setDisabled(True)
        self.launch_person_follow_button.setDisabled(False)
        self.deactivate_camera_button.setDisabled(False)

    def launch_robot(self):
        if self.robot_process is None:
            self.ssh_client = paramiko.SSHClient()
            self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            try:
                # Connect to the Raspberry Pi
                self.ssh_client.connect(self.raspi_ip, username=self.raspi_username, password=self.raspi_password)

                # Command to launch the robot
                command = "ros2 launch vedita_bot launch_robot.launch.py"

                # Execute the command in a shell to allow for streaming output
                transport = self.ssh_client.get_transport()
                channel = transport.open_session()
                channel.exec_command(f"bash -c '{command}'")

                # Create a thread to read the output from the SSH channel
                self.ssh_log_thread = SSHLogReader(channel)
                self.ssh_log_thread.new_log.connect(self.update_log)
                self.ssh_log_thread.start()

                self.status_label.setText("Robot running...")

                # Disable launch button, enable stop button
                self.launch_robot_button.setDisabled(True)
                self.stop_robot_button.setDisabled(False)

                # Store the channel for later use
                self.robot_process = channel
            except Exception as e:
                self.status_label.setText(f"Failed to connect to Raspberry Pi: {e}")

    def stop_robot(self):
        if self.robot_process is not None:
            try:
                # Send SIGINT to the remote process
                self.robot_process.send(b'\x03')  # Ctrl+C
                self.robot_process.close()
                self.ssh_client.close()
                self.status_label.setText("Robot stopped.")
                self.robot_process = None

                # Disable stop button, enable launch button
                self.stop_robot_button.setDisabled(True)
                self.launch_robot_button.setDisabled(False)
            except Exception as e:
                self.status_label.setText(f"Failed to stop robot: {e}")

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
        self.stop_joystick()
        self.stop_camera()
        self.stop_person_follower()
        #self.stop_robot()
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


class SSHLogReader(QThread):
    new_log = pyqtSignal(str)

    def __init__(self, channel):
        super().__init__()
        self.channel = channel

    def run(self):
        while True:
            if self.channel.recv_ready():
                output = self.channel.recv(1024).decode('utf-8')
                if output:
                    self.new_log.emit(output.strip())
            if self.channel.exit_status_ready():
                break

def cleanup():
    """Ensure all ROS 2 processes are stopped before exiting."""
    if window.sim_process is not None:
        window.stop_ros_process(window.sim_process, "Simulation stopped.")
    if window.localization_process is not None:
        window.stop_ros_process(window.localization_process, "Localization stopped.")
    if window.navigation_process is not None:
        window.stop_ros_process(window.navigation_process, "Navigation stopped.")
    if window.joystick_process is not None:
        window.stop_ros_process(window.joystick_process, "Joystick stopped.")
    if window.camera_process is not None:
        window.stop_ros_process(window.camera_process, "Camera stopped.")
    if window.person_follower_process is not None:
        window.stop_ros_process(window.person_follower_process, "Person Follower stopped.")
    if window.robot_process is not None:
        window.stop_robot()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ROS2Launcher()
    window.show()
    atexit.register(cleanup)
    sys.exit(app.exec())