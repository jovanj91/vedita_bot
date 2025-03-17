import sys
import subprocess
import os
import signal
from dotenv import load_dotenv
import re
import pymysql
from PyQt6.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QLabel, QTextEdit, QLineEdit, QHBoxLayout, QCheckBox
)
from PyQt6.QtCore import QThread, pyqtSignal
import paramiko, atexit


class ROS2Launcher(QWidget):
    def __init__(self):
        super().__init__()

        load_dotenv()
        # Store ROS processes
        self.robot_process = None
        self.web_server = None
        self.joystick_process_process = None
        

        self.log_thread = None

        self.ros_ws_path = os.path.expanduser("~/ros2_ws")

        self.setWindowTitle("Vedita Launcher")
        self.setGeometry(100, 100, 700, 600)

        layout = QVBoxLayout()

        self.status_label = QLabel("Set parameters and launch ROS 2 processes")
        layout.addWidget(self.status_label)

        # Buttons
        self.launch_robot_button = QPushButton("Launch Robot")
        self.launch_robot_button.clicked.connect(self.launch_robot)
        self.launch_robot_button.setVisible(False) 
        layout.addWidget(self.launch_robot_button)

        self.stop_robot_button = QPushButton("Stop Robot")
        self.stop_robot_button.clicked.connect(self.stop_robot)
        self.stop_robot_button.setDisabled(True)  
        layout.addWidget(self.stop_robot_button)

        self.launch_camera_ws_button = QPushButton("Launch Camera Web-Server")
        self.launch_camera_ws_button.clicked.connect(self.launch_camera_ws)
        layout.addWidget(self.launch_camera_ws_button)

        self.stop_camera_ws_button = QPushButton("Stop Camera Web-Server")
        self.stop_camera_ws_button.clicked.connect(self.stop_camera_ws)
        self.stop_camera_ws_button.setDisabled(True)  # Initially disabled
        layout.addWidget(self.stop_camera_ws_button)

        self.log_output = QTextEdit()
        self.log_output.setReadOnly(True)
        layout.addWidget(self.log_output)
        
        self.launch_joystick_button = QPushButton("Launch Joystick Control")
        self.launch_joystick_button.clicked.connect(self.launch_joystick)
        layout.addWidget(self.launch_joystick_button)

        self.stop_joystick_button = QPushButton("Stop Joystick Control")
        self.stop_joystick_button.clicked.connect(self.stop_joystick)
        self.stop_joystick_button.setDisabled(True)  # Initially disabled
        layout.addWidget(self.stop_joystick_button)
    
        self.setLayout(layout)
        
    def launch_robot(self):
        if self.navigation_process is None:
            command = f"ros2 launch vedita_bot launch_robot.launch.py"
            self.navigation_process = self.start_ros_process(command)
            self.status_label.setText("Robot System running...")

            # Disable launch button, enable stop button
            self.launch_robot_button.setDisabled(True)
            self.stop_robot_button.setDisabled(False)
    
    def stop_robot(self):
        self.stop_ros_process(self.navigation_process, "Navigation stopped.")
        self.navigation_process = None

        # Disable stop button, enable previous step stop button
        self.stop_robot_button.setDisabled(True)
        self.launch_robot_button.setDisabled(False)
        
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

    def launch_camera_ws(self):
        if self.camera_ws_process is None:
            command = f"ros2 run web_video_server web_video_server"
            self.camera_ws_process = self.start_ros_process(command)
            self.status_label.setText("Camera WebServer Running..")

            # Disable launch button, enable stop button
            self.launch_camera_ws_button.setDisabled(True)
            self.stop_camera_ws_button.setDisabled(False)

    def stop_camera_ws(self):
        self.stop_ros_process(self.camera_ws_process, "Camera WebServer Stopped.")
        self.camera_ws_process= None

        # Disable stop button, enable previous step stop button
        self.stop_camera_ws_button.setDisabled(True)
        self.launch_camera_ws_button.setDisabled(False)


    def launch_person_follower(self):
        if self.person_follower_process is None:
            command = f"ros2 launch vedita_bot person_follower_yolo.launch.py"
            self.person_follower_process = self.start_ros_process(command)
            self.status_label.setText("Person Follower Running..")

            # Disable launch button, enable stop button
            self.launch_person_follow_button.setDisabled(True)
            self.stop_person_follow_button.setDisabled(False)
    
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

    def insert_data(self, table, data):
        conn = None
        try:
            # Connect to MySQL database
            conn = pymysql.connect(
                host=self.DB_HOST,
                port=self.DB_PORT,
                user=self.DB_USER,
                password=self.DB_PASSWORD,
                database=self.DB_NAME,
                cursorclass=pymysql.cursors.DictCursor
            )
            
            cursor = conn.cursor()

            # Prepare SQL query dynamically
            columns = ', '.join(data.keys())
            placeholders = ', '.join(['%s'] * len(data))
            sql = f"REPLACE INTO {table} ({columns}) VALUES ({placeholders})"

            cursor.execute(sql, tuple(data.values()))
            conn.commit()
            return True

        except pymysql.MySQLError as e:
            return False

        finally:
            if conn:
                conn.close()

    def update_log(self, log_text):
        """Update the log output with colored text."""
        html_text = self.ansi_to_html(log_text)
        self.log_output.append(html_text)

    def closeEvent(self, event):
        """Ensure all ROS 2 processes are stopped before closing the application."""
        self.stop_joystick()
        self.stop_camera_ws()
        self.stop_robot()
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

def cleanup():
    """Ensure all ROS 2 processes are stopped before exiting."""
    if window.robot_process is not None:
        window.stop_ros_process(window.robot_process, "Robot Stopped.")
    if window.joystick_process is not None:
        window.stop_ros_process(window.camera_ws_process, "Camera Web Server Stopped.")
    if window.camera_ws_process is not None:
        window.stop_ros_process(window.joystick_process, "Joystick stopped.")
    

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ROS2Launcher()
    window.show()
    atexit.register(cleanup)
    sys.exit(app.exec())