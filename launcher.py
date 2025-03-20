import sys
import subprocess
import os
import signal
from dotenv import load_dotenv
import re
import mysql.connector
from PyQt6.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QLabel, QTextEdit, QLineEdit, QHBoxLayout, QCheckBox
)
from PyQt6.QtCore import QThread, pyqtSignal
import paramiko, atexit

import requests


class ROS2Launcher(QWidget):
    def __init__(self):
        super().__init__()

        load_dotenv()
        # Store ROS processes
        self.sim_process = None
        self.localization_process = None
        self.navigation_process = None
        self.joystick_process = None
        self.camera_ws_process = None
        self.person_follower_process = None
        self.move_point_A_process = None
        self.move_point_B_process = None
        self.move_point_C_process = None
        self.move_point_D_process = None
        self.move_point_E_process = None

        self.DB_HOST=os.getenv("DB_HOST")
        self.DB_PORT=int(os.getenv("DB_PORT"))
        self.DB_USER=os.getenv("DB_USER")
        self.DB_PASSWORD=os.getenv("DB_PASSWORD")
        self.DB_NAME=os.getenv("DB_NAME")

        self.DB_HOST_EMS=os.getenv("DB_HOST_EMS")
        self.DB_PORT_EMS=int(os.getenv("DB_PORT_EMS"))
        self.DB_USER_EMS=os.getenv("DB_USER_EMS")
        self.DB_PASSWORD_EMS=os.getenv("DB_PASSWORD_EMS")
        self.DB_NAME_EMS=os.getenv("DB_NAME_EMS")

        self.ws_url=os.getenv("WS_VEDITA")

        self.raspi_ip=os.getenv("RASPI_IP")
        self.raspi_username=os.getenv("RASPI_USERNAME")
        self.raspi_password=os.getenv("RASPI_PASSWORD")

        self.ssh_client = None
        self.robot_process = None
        self.log_thread = None

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
        self.stop_sim_button.setDisabled(True) 
        layout.addWidget(self.stop_sim_button)

        self.launch_robot_button = QPushButton("Launch Robot")
        self.launch_robot_button.clicked.connect(self.launch_robot)
        self.launch_robot_button.setVisible(False) 
        layout.addWidget(self.launch_robot_button)

        self.stop_robot_button = QPushButton("Stop Robot")
        self.stop_robot_button.clicked.connect(self.stop_robot)
        self.stop_robot_button.setDisabled(True)  
        self.stop_robot_button.setVisible(False)  
        layout.addWidget(self.stop_robot_button)

        map_layout = QHBoxLayout()
        map_layout.addWidget(QLabel("Map File:"))
        self.map_input = QLineEdit("./src/vedita_bot/maps/gayungan.yaml")
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

        self.launch_camera_ws_button = QPushButton("Launch Camera Web-Server")
        self.launch_camera_ws_button.clicked.connect(self.launch_camera_ws)
        self.launch_camera_ws_button.setVisible(False)  # Initially hidden
        layout.addWidget(self.launch_camera_ws_button)

        self.stop_camera_ws_button = QPushButton("Stop Camera Web-Server")
        self.stop_camera_ws_button.clicked.connect(self.stop_camera_ws)
        self.stop_camera_ws_button.setDisabled(True)  # Initially disabled
        self.stop_camera_ws_button.setVisible(False)  # Initially hidden
        layout.addWidget(self.stop_camera_ws_button)
    
        self.move_pointA_layout = QHBoxLayout()
        self.launch_pointA_button = QPushButton("F3")
        self.stop_pointA_button = QPushButton("Stop F3")
        self.stop_pointA_button.setDisabled(True)  # Initially disabled
        self.launch_pointA_button.clicked.connect(lambda: self.launch_move_point(
            self.move_point_A_process, 
            'ros2 run vedita_bot_addon send_goal --ros-args '
            '-p x:=1.55 '
            '-p y:=3.37 '
            '-p z:=0.0 '
            '-p qx:=0.0 '
            '-p qy:=0.0 '
            '-p qz:=0.73 '
            '-p qw:=0.68 ', 
            self.launch_pointA_button, self.stop_pointA_button,
            table='sari_additionalData', data=(1, 'F3'),
            room='F3'
        ))
        self.move_pointA_layout.addWidget(self.launch_pointA_button)
        
        self.stop_pointA_button.clicked.connect(lambda: self.stop_move_point(
            self.move_point_A_process, 
            self.launch_pointA_button, self.stop_pointA_button
        ))
        self.move_pointA_layout.addWidget(self.stop_pointA_button)

        self.move_pointB_layout = QHBoxLayout()

        self.launch_pointB_button = QPushButton("F4")
        self.stop_pointB_button = QPushButton("Stop F4")
        self.launch_pointB_button.clicked.connect(lambda: self.launch_move_point(
            self.move_point_B_process, 
            'ros2 run vedita_bot_addon send_goal --ros-args '
            '-p x:=-4.04 '
            '-p y:=2.96 '
            '-p z:=0.0 '
            '-p qx:=0.0 '
            '-p qy:=0.0 '
            '-p qz:=0.96 '
            '-p qw:=0.27 ', 
            self.launch_pointB_button, self.stop_pointB_button,
            table='sari_additionalData', data=(1, 'F4'),
            room='F4'
        ))
        self.move_pointB_layout.addWidget(self.launch_pointB_button)
        
        self.stop_pointB_button.clicked.connect(lambda: self.stop_move_point(
            self.move_point_B_process, 
            self.launch_pointB_button, self.stop_pointB_button
        ))
        self.stop_pointB_button.setDisabled(True)  # Initially disabled
        self.move_pointB_layout.addWidget(self.stop_pointB_button)

        self.move_pointC_layout = QHBoxLayout()

        self.launch_pointC_button = QPushButton("F5")
        self.stop_pointC_button = QPushButton("Stop F5")
        self.launch_pointC_button.clicked.connect(lambda:self.launch_move_point(
            self.move_point_C_process, 
            'ros2 run vedita_bot_addon send_goal --ros-args '
            '-p x:=-3.97 '
            '-p y:=4.6 '
            '-p z:=0.0 '
            '-p qx:=0.0 '
            '-p qy:=0.0 '
            '-p qz:=-0.67 '
            '-p qw:=0.73 ', 
            self.launch_pointC_button, self.stop_pointC_button,
            table='sari_additionalData', data=(1, 'F5'),
            room='F5'
        ))
        self.move_pointC_layout.addWidget(self.launch_pointC_button)
        
        self.stop_pointC_button.clicked.connect(lambda: self.stop_move_point(
            self.move_point_C_process, 
            self.launch_pointC_button, self.stop_pointC_button
        ))
        self.stop_pointC_button.setDisabled(True)  # Initially disabled
        self.move_pointC_layout.addWidget(self.stop_pointC_button)
        
        self.move_pointD_layout = QHBoxLayout()
        self.launch_pointD_button = QPushButton("F6")
        self.stop_pointD_button = QPushButton("Stop F6")
        self.stop_pointD_button.setDisabled(True)  # Initially disabled
        self.launch_pointD_button.clicked.connect(lambda: self.launch_move_point(
            self.move_point_D_process, 
            'ros2 run vedita_bot_addon send_goal --ros-args '
            '-p x:=0.0 '
            '-p y:=0.0 '
            '-p z:=0.0 '
            '-p qx:=0.0 '
            '-p qy:=0.0 '
            '-p qz:=0.44 '
            '-p qw:=0.89 ', 
            self.launch_pointD_button, self.stop_pointD_button,
            table='sari_additionalData', data=(1, 'F6'),
            room='F6'
        ))
        self.move_pointD_layout.addWidget(self.launch_pointD_button)
        
        self.stop_pointD_button.clicked.connect(lambda: self.stop_move_point(
            self.move_point_D_process, 
            self.launch_pointD_button, self.stop_pointD_button
        ))
        self.move_pointD_layout.addWidget(self.stop_pointD_button)

        self.move_pointE_layout = QHBoxLayout()
        self.launch_pointE_button = QPushButton("Docking")
        self.stop_pointE_button = QPushButton("Docking")
        self.stop_pointE_button.setDisabled(True)  # Initially disabled
        self.launch_pointE_button.clicked.connect(lambda: self.launch_move_point(
            self.move_point_E_process, 
            'ros2 run vedita_bot_addon send_goal --ros-args '
            '-p x:=0.0 '
            '-p y:=0.0 '
            '-p z:=0.0 '
            '-p qx:=0.0 '
            '-p qy:=0.0 '
            '-p qz:=0.0 '
            '-p qw:=1.0 ', 
            self.launch_pointE_button, self.stop_pointE_button,
            table='sari_additionalData', data=(1, 'Restroom F6'),
            room='F6'
        ))
        self.move_pointE_layout.addWidget(self.launch_pointE_button)
        
        self.stop_pointE_button.clicked.connect(lambda: self.stop_move_point(
            self.move_point_E_process, 
            self.launch_pointE_button, self.stop_pointE_button
        ))
        self.move_pointE_layout.addWidget(self.stop_pointE_button)

        layout.addLayout(self.move_pointA_layout)
        layout.addLayout(self.move_pointB_layout)
        layout.addLayout(self.move_pointC_layout)
        layout.addLayout(self.move_pointD_layout)
        layout.addLayout(self.move_pointE_layout)

        self.setLayout(layout)

    def toggle_use_sim_time(self):
        is_checked = self.sim_time_checkbox.isChecked()
        self.world_label.setVisible(is_checked)
        self.world_input.setVisible(is_checked)
        self.launch_localization_button.setDisabled(is_checked)
        self.launch_sim_button.setVisible(is_checked)
        self.stop_sim_button.setVisible(is_checked)
        self.launch_joystick_button.setVisible(not is_checked)
        self.stop_joystick_button.setVisible(not is_checked)
        # self.launch_camera_ws_button.setVisible(not is_checked)
        # self.stop_camera_ws_button.setVisible(not is_checked)
        # self.launch_person_follow_button.setVisible(not is_checked)
        # self.stop_person_follow_button.setVisible(not is_checked)
        # self.launch_robot_button.setVisible(not is_checked)
        # self.stop_robot_button.setVisible(not is_checked)

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
            self.stop_localization_button.setDisabled(True)
    
    def stop_navigation(self):
        self.stop_ros_process(self.navigation_process, "Navigation stopped.")
        self.navigation_process = None

        # Disable stop button, enable previous step stop button
        self.stop_navigation_button.setDisabled(True)
        self.launch_navigation_button.setDisabled(False)
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
            

    def stop_person_follower(self):
        self.stop_ros_process(self.person_follower_process, "Person Follower stopped.")
        self.person_follower_process = None

        # Disable stop button, enable previous step stop button
        self.stop_person_follow_button.setDisabled(True)
        self.launch_person_follow_button.setDisabled(False)
        
    def launch_move_point(self, point_process, command, button_launch, button_stop, table, data, room):
        if point_process is None:
            point_process = self.start_ros_process(command)
            self.status_label.setText("Robot Moving...")
            
            self.insert_data(table, data)
            # Disable launch button, enable stop button, enable next step
            # data = self.fetch_data_sensor(room)
            # self.send_message(room, data)
            button_launch.setDisabled(True)
            button_stop.setDisabled(False)

    def stop_move_point(self, point_process, button_launch, button_stop):
        self.stop_ros_process(point_process, "Move Canceled")
        point_process = None

        # Disable stop button, disable next steps
        button_stop.setDisabled(True)
        button_launch.setDisabled(False)

    def launch_robot(self):
        if self.robot_process is None:
            self.ssh_client = paramiko.SSHClient()
            self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            try:
                # Connect to the Raspberry Pi
                self.ssh_client.connect(self.raspi_ip, username=self.raspi_username, password=self.raspi_password)

                # Command to launch the robot
                command = "ros2 launch vedita_bot camera.launch.py"

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
                self.launch_robot_button.setDisabled(False)
                self.stop_robot_button.setDisabled(True)

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
        try:
            # Connect to MySQL database
            connection = mysql.connector.connect(
                host=self.DB_HOST,
                user=self.DB_USER,
                password=self.DB_PASSWORD,
                database=self.DB_NAME,
                port=self.DB_PORT
            )
            cursor = connection.cursor()

            # Execute REPLACE INTO query
            query = f"REPLACE INTO `{table}` (`id`, `room_name`) VALUES (%s, %s)"
            cursor.execute(query, data)

            # Commit and close connection
            connection.commit()
            cursor.close()
            connection.close()
            print(f"Position Updated")

        except mysql.connector.Error as e:
            print(f" Database error: {e}")

    def fetch_data_sensor(self, room):
        try:
            # Connect to MySQL database
            connection = mysql.connector.connect(
                host=self.DB_HOST_EMS,
                user=self.DB_USER_EMS,
                password=self.DB_PASSWORD_EMS,
                database=self.DB_NAME_EMS,
                port=self.DB_PORT_EMS
            )
            cursor = connection.cursor()

            # Execute SELECT query
            query = "SELECT humidity, temperature FROM AVG_DHT where room_name= %s;"
            cursor.execute(query,(room))

            # Fetch all rows_sensor
            results = cursor.fetchall()

            # Close connection
            cursor.close()
            connection.close()

            return results  # Return fetched data

        except mysql.connector.Error as e:
            print(f"Database error: {e}")
            return None  # Return None in case of error

    def send_message(self, room, data):
        room_condition=""
        temperature = 0
        humidity = 0
        if data:
            for row in data:
                humidity, temperature = row  # Unpack tuple
            room_condition = f" Saat ini, kondisi ruangan {room} berada pada \
                suhu {temperature} derajat celcius dan kelembaban {humidity} persen."
            
        url = f"{self.ws_url}/sendmessage"
        params = {"user": "vedita", 
                  "text": f"Baik kak, Vedita menuju ke ruangan {room}. {room_condition}"}
        try:
            response = requests.get(url, params=params, verify=False)
            if response.status_code == 200:
                self.status_label.setText("Message sent successfully!")
            else:
                self.status_label.setText(f"Error: {response.status_code}")
        except requests.exceptions.RequestException as e:
            print(f"Request failed: {e}")

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
        self.stop_camera_ws()
        self.stop_person_follower()
        self.stop_robot()
        if(self.move_point_A_process is not None):
            self.stop_move_point(point_process=self.move_point_A_process, button_launch=self.launch_pointA_button, button_stop=self.stop_pointA_button)
        if(self.move_point_B_process is not None):
            self.stop_move_point(point_process=self.move_point_B_process, button_launch=self.launch_pointB_button, button_stop=self.stop_pointB_button)
        if(self.move_point_C_process is not None):
            self.stop_move_point(point_process=self.move_point_C_process, button_launch=self.launch_pointC_button, button_stop=self.stop_pointC_button)
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
    if window.camera_ws_process is not None:
        window.stop_ros_process(window.camera_ws_process, "Camera Web Server stopped.")
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