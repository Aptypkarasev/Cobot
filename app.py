import tkinter as tk
import tkinter.messagebox as mb
from tkinter import ttk
from tkinter import filedialog
import subprocess
import socket
import signal
import os
from time import sleep
import time
from datetime import datetime
import urx
import asyncio
from threading import Thread
import threading
import multiprocessing as mp
from math import sin, cos, tan, pi
import asyncio
from pathlib import Path
import logging
from logging.handlers import RotatingFileHandler
import csv

import Align_D
import Align_H
import Power_On_H
import camDiagn
import camHirurg
import ESTOP_RESET_D
import ESTOP_RESET_H
import Joystick_diagnost
import Joystick_hirurg
import Power_On_D
import Power_Off_H
import Power_Off_D

rob_us_data = []
us_lock = False
stop_route = threading.Event()
starting_pose = []

auto = mp.Value("i", 1)
force_lock = mp.Value("i", 0)
control_lock = mp.Value("i", 0)
program_lock = mp.Value("i", 0)


def setup_status_logging(log_dir="logs", max_bytes=10 * 1024 * 1024, backup_count=5):
    log_dir = Path(log_dir)
    log_dir.mkdir(exist_ok=True)

    log_file = log_dir / f'processes_status_log_{datetime.now().strftime("%Y-%m-%d")}.log'

    logger = logging.getLogger("ProcessStatus")
    logger.setLevel(logging.DEBUG)

    if logger.handlers:
        return logger

    formatter = logging.Formatter(
        fmt='%(asctime)s.%(msecs)03d | %(levelname)-8s | %(processName)-15s | %(threadName)-15s | %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )

    file_handler = RotatingFileHandler(
        filename=log_file,
        maxBytes=max_bytes,
        backupCount=backup_count,
        encoding='utf-8'
    )

    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(formatter)

    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    console_handler.setFormatter(formatter)

    logger.addHandler(file_handler)
    logger.addHandler(console_handler)

    logger.info("=" * 80)
    logger.info(f'ЗАПУСК ЕППУИХ | PID: {os.getpid()}')
    logger.info("=" * 80)

    return logger


class CSVLogger:
    def __init__(self, filename="logs/telemetry_log.csv"):
        self.filename = filename
        self.enabled = False
        self._ensure_header()

    def _ensure_header(self):
        if not os.path.exists(self.filename):
            with open(self.filename, "w", newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(["timestamp","robot", "temperature", "X", "Y", "Z", "J1","J2","J3","J4", "J5", "J6", "FX", "FY", "FZ", "MX", "MY", "MZ"])

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False

    def log_data(self, data):
        if not self.enabled:
            return

        try:
            timestamp = datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]
            with open(self.filename, "a", newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([timestamp]+[data])
        except Exception as e:
            logger.error(f'Ошибка логгирования телеметрии: {type(e).__name__}:{str(e)[:100]}')



def force_control(threshold):
    rob = urx.Robot("192.168.8.3", use_rt=True)
    global stop_route
    while True:
        forces = rob.get_tcp_force()
        if forces[2] > threshold:
            force_lock.value = 1
            rob.speedl_tool([0] * 6, 0.5, 1)
            stop_route.set()
            time.sleep(1)
            rob.movel_tool([0, 0, -0.1, 0, 0, 0], 0.5, 0.5)
            time.sleep(10)
            force_lock.value = 0
        else:
            force_lock.value = 0


def route_follow(nsteps, S, pause_time, vel):
    rob = urx.Robot("192.168.8.3", use_rt=True)
    global rob_us_data, us_lock, stop_route, starting_pose
    force = [rob.get_tcp_force()[2]]
    t = [0]
    coordinates = [0]
    start = time.time()
    us_lock = True
    starting_pose = rob.getl()
    for x in range(nsteps):
        if stop_route.is_set():
            stop_route.clear()
            rob_us_data = [t, coordinates, force]
            app.show_warning("Маршрут остановлен")
            rob.close()
            break
        rob.translate((S[0], 0, 0), 0.1, vel)
        while rob.is_program_running():
            if stop_route.is_set():
                stop_route.clear()
                rob_us_data = [t, coordinates, force]
                app.show_warning("Маршрут остановлен")
                rob.close()
                break
            pass
        rob.translate((0, S[1], 0), 0.1, vel)
        while rob.is_program_running():
            if stop_route.is_set():
                stop_route.clear()
                rob_us_data = [t, coordinates, force]
                app.show_warning("Маршрут остановлен")
                rob.close()
                break
            pass
        rob.translate((0, 0, S[2]), 0.1, vel)
        while rob.is_program_running():
            if stop_route.is_set():
                stop_route.clear()
                rob_us_data = [t, coordinates, force]
                app.show_warning("Маршрут остановлен")
                rob.close()
                break
            pass
        t.append(time.time() - start)
        time.sleep(pause_time)
        coordinates.append(rob.getl())
        force.append(rob.get_tcp_force()[2])

        # for x in range(nsteps):
        #     # rob.translate((0, 0, -S[2]), 0.1, vel)
        #     while rob.is_program_running():
        #         pass
        #     rob.translate((0, -S[1], 0), 0.1, vel)
        #     while rob.is_program_running():
        #         pass
        #     rob.translate((-S[0], 0, 0), 0.1, vel)
        #     while rob.is_program_running():
        #         pass
        if stop_route.is_set():
            rob_us_data = [t, coordinates, force]
            app.show_warning("Маршрут остановлен")
            rob.close()
            us_lock = False
            stop_route.clear()
            return
    rob_us_data = [t, coordinates, force]
    # rob.translate((0, -S[1]*nsteps, 0), 0.1, 0.05)
    # while rob.is_program_running():
    #     pass
    # rob.translate((-S[0]*nsteps, 0, 0), 0.1, 0.05)
    # while rob.is_program_running():
    #     pass
    us_lock = False
    rob.close()


def aphi(angle, nsteps, step, pause_time, vel, tool_length=0.645):
    global us_lock, stop_route

    rob = urx.Robot("192.168.8.4", use_rt=True)
    pose = rob.getl()
    # rob.movel((pose[0], pose[1], pose[2], 3.14 / 2, 0, 0), acc=0.2, vel=0.2)
    # while rob.is_program_running():
    #     pass
    # angle *= (pi / 180)
    # r = tool_length + 0.195
    # h = r * sin(angle) + 0.005
    # delta = r - r * cos(angle)
    # rob.translate((0, 0, h), 0.1, 0.5)
    # while rob.is_program_running():
    #     pass
    # rob.translate_tool((0, 0, delta), 0.1, 0.1)
    # while rob.is_program_running():
    #     pass
    # o = rob.get_orientation()
    # o.rotate_xt(angle)
    # rob.set_orientation(o, 0.1, 0.1)
    # while rob.is_program_running():
    #     pass
    # rob.translate_tool((0, 0, nsteps * step), 0.1, vel)
    # time.sleep(90)

    for s in range(nsteps):

        rob.translate_tool((0, 0, -step), 0.1, vel)
        while rob.is_program_running():
            pass

        time.sleep(pause_time)
    while rob.is_program_running():
        pass
    us_lock = False
    rob.close()


def ashido_init(nsteps, step, angle=0, tool_length=0.645):
    global us_lock
    rob = urx.Robot("192.168.8.4", use_rt=True)
    pose = rob.getl()
    if angle != 0:
        rob.movel((pose[0], pose[1], pose[2], 3.14 / 2, 0, 0), acc=0.2, vel=0.2)
        while rob.is_program_running():
            pass
        angle *= (pi / 180)
        r = tool_length + 0.195
        h = r * sin(angle) + 0.005
        delta = r - r * cos(angle)
        rob.translate((0, 0, h), 0.1, 0.5)
        while rob.is_program_running():
            pass
        rob.translate_tool((0, 0, delta), 0.1, 0.1)
        while rob.is_program_running():
            pass
        o = rob.get_orientation()
        o.rotate_xt(angle)
        rob.set_orientation(o, 0.1, 0.1)
        while rob.is_program_running():
            pass
    rob.translate_tool((0, 0, nsteps * step), 0.1, 0.1)

    us_lock = False
    rob.close()
    program_lock.value = 0


async def ashido(nsteps, step, pause_time, vel, angle=0, is_hirurg=0, tool_length=0.645):
    global rob_us_data, us_lock, stop_route
    loop = asyncio.get_running_loop()

    hirurg = urx.Robot("192.168.8.4", use_rt=True)
    diagnost = urx.Robot("192.168.8.3", use_rt=True)

    time.sleep(5)

    if (not (hirurg.is_running() or diagnost.is_running())):
        return -1
    if angle != 0:
        dv = vel * cos(angle * pi / 180) if vel * cos(angle * pi / 180) >= 0.001 else 0.001
        ds = step * cos(angle * pi / 180) if step * cos(angle * pi / 180) >= 0.001 else 0.001
    else:
        dv = vel
        ds = step

    t = [0]
    coordinates = [0]
    start = time.time()
    us_lock = True
    starting_pose = diagnost.getl()

    for s in range(nsteps):
        await loop.run_in_executor(None, hirurg.translate_tool, ([0, 0, -step], vel, vel))
        await loop.run_in_executor(None, diagnost.translate, ([0, -ds, 0], dv, dv))
        while hirurg.is_program_running() or diagnost.is_program_running():
            pass
        t.append(time.time() - start)
        coordinates.append(diagnost.getl())

    rob_us_data = [t, coordinates]
    us_lock = False
    hirurg.close()
    diagnost.close()
    program_lock.value = 0


class ScriptRunnerApp:
    attempt_d_counter = 0
    attempt_h_counter = 0
    cam_h_state = False
    cam_d_state = False

    def __init__(self, root):
        self.root = root
        self.root.title('ЕППУИХ')

        self.threads = []
        self.processes = {}
        self.heartbeat = mp.Queue()
        self.heartbeat_timeout = 10.0
        self.restart_delays = {}

        tabs = ttk.Notebook(root)
        control_tab = ttk.Frame(tabs)
        cam_tab = ttk.Frame(tabs)
        telemetry_tab = ttk.Frame(tabs)
        route_tab = ttk.Frame(tabs)
        aphi_tab = ttk.Frame(tabs)
        ashido_tab = ttk.Frame(tabs)

        tabs.add(control_tab, text='РПК')
        tabs.add(route_tab, text='ППУИ')
        tabs.add(aphi_tab, text="АПХИ")
        tabs.add(ashido_tab, text="АСХИДО")
        tabs.add(cam_tab, text="Камеры")
        tabs.add(telemetry_tab, text="Телеметрия")
        tabs.pack(expand=1, fill="both")

        # self.label = tk.Label(root, text= "Запуск системы управления")

        self.system_launch_btn = tk.Button(control_tab, text='Запуск системы', command=self.system_launch,
                                           state=tk.NORMAL)
        self.system_launch_btn.grid(column=0, row=0, padx=5, pady=15)

        self.system_stop_btn = tk.Button(control_tab, text='Остановка системы', command=self.system_stop,
                                         state=tk.DISABLED)
        self.system_stop_btn.grid(column=1, row=0, padx=5, pady=15)

        self.control_launch_btn = tk.Button(control_tab, text='Запуск управления', command=self.control_launch,
                                            state=tk.DISABLED)
        self.control_launch_btn.grid(column=0, row=1, padx=10, pady=10)

        self.control_stop_btn = tk.Button(control_tab, text='Остановка управления', command=self.control_stop,
                                          state=tk.DISABLED)
        self.control_stop_btn.grid(column=1, row=1, padx=10, pady=10)

        self.align_btn = tk.Button(control_tab, text='Выравнивание роботов', command=self.align, state=tk.DISABLED)
        self.align_btn.grid(column=0, row=2, padx=10, pady=10)

        self.unlock_btn = tk.Button(control_tab, text='Разблокировать роботов', command=self.unlock)
        self.unlock_btn.grid(column=1, row=2, padx=10, pady=10)

        self.save_d_route_btn = tk.Button(control_tab, text='Сохранить маршрут диагноста', command=self.save_d_route)
        self.save_d_route_btn.grid(column=2, row=0, padx=10, pady=10)

        self.save_h_route_btn = tk.Button(control_tab, text='Сохранить маршрут хирурга', command=self.save_h_route)
        self.save_h_route_btn.grid(column=3, row=0, padx=10, pady=10)

        self.launch_d_route_btn = tk.Button(control_tab, text='Запуск маршрута диагноста из файла',
                                            command=self.launch_d_route)
        self.launch_d_route_btn.grid(column=2, row=1, padx=10, pady=10)

        self.launch_h_route_btn = tk.Button(control_tab, text='Запуск маршрута хирурга из файла',
                                            command=self.launch_h_route)
        self.launch_h_route_btn.grid(column=3, row=1, padx=10, pady=10)

        self.cam_d_btn = tk.Button(cam_tab, text='Камера диагноста', command=self.cam_d)
        self.cam_d_btn.grid(column=0, row=0, padx=10, pady=10)

        self.cam_d_btn = tk.Button(cam_tab, text='Камера хирурга', command=self.cam_h)
        self.cam_d_btn.grid(column=1, row=0, padx=10, pady=10)

        self.num_steps_label = tk.Label(route_tab, text="Количество шагов")
        self.num_steps_label.grid(column=0, row=0)

        self.step_val_label = tk.Label(route_tab, text="Величина шага, м")
        self.step_val_label.grid(column=0, row=1, padx=1, pady=10)

        self.step_x_label = tk.Label(route_tab, text="x:")
        self.step_x_label.grid(column=1, row=1, padx=1, pady=10)

        self.step_x_entry = tk.Entry(route_tab)
        self.step_x_entry.grid(column=2, row=1, padx=2, pady=10)

        self.step_y_label = tk.Label(route_tab, text="y:")
        self.step_y_label.grid(column=3, row=1, padx=1, pady=10)

        self.step_y_entry = tk.Entry(route_tab)
        self.step_y_entry.grid(column=4, row=1, padx=2, pady=10)

        self.step_z_label = tk.Label(route_tab, text="z:")
        self.step_z_label.grid(column=5, row=1, padx=1, pady=10)

        self.step_z_entry = tk.Entry(route_tab)
        self.step_z_entry.grid(column=6, row=1, padx=2, pady=10)

        self.pause_time_label = tk.Label(route_tab, text="Время остановки")
        self.pause_time_label.grid(column=0, row=2, padx=10, pady=10)

        self.velocity_label = tk.Label(route_tab, text="Скорость, м/с")
        self.velocity_label.grid(column=0, row=3, padx=1, pady=10)

        self.num_steps_entry = tk.Entry(route_tab)
        self.num_steps_entry.grid(column=1, row=0, padx=1, pady=10)

        self.pause_time_entry = tk.Entry(route_tab)
        self.pause_time_entry.grid(column=1, row=2, padx=1, pady=10)

        self.velocity_entry = tk.Entry(route_tab)
        self.velocity_entry.grid(column=1, row=3, padx=10, pady=10)

        self.start_route_btn = tk.Button(route_tab, text="Начать маршрут", command=self.start_route)
        self.start_route_btn.grid(column=0, row=4)

        self.stop_route_btn = tk.Button(route_tab, text="Остановить маршрут", command=self.stop_routef)
        self.stop_route_btn.grid(column=1, row=4)

        self.return_to_start_btn = tk.Button(route_tab, text="Вернуться в исходную точку", command=self.return_to_start)
        self.return_to_start_btn.grid(column=2, row=4)

        self.save_route_btn = tk.Button(route_tab, text="Сохранить данные маршрута", command=self.save_route)
        self.save_route_btn.grid(column=0, row=5)

        self.move_angle_label = tk.Label(aphi_tab, text="Угол продвижения в градусах")
        self.move_angle_label.grid(column=0, row=0, padx=1, pady=10)

        self.move_angle_entry = tk.Entry(aphi_tab)
        self.move_angle_entry.grid(column=1, row=0, padx=1, pady=10)

        self.aphi_num_steps_label = tk.Label(aphi_tab, text="Количество шагов")
        self.aphi_num_steps_label.grid(column=0, row=1)

        self.aphi_num_steps_entry = tk.Entry(aphi_tab)
        self.aphi_num_steps_entry.grid(column=1, row=1)

        self.aphi_step_val_label = tk.Label(aphi_tab, text="Величина шага, м")
        self.aphi_step_val_label.grid(column=0, row=2, padx=1, pady=10)

        self.aphi_step_val_entry = tk.Entry(aphi_tab)
        self.aphi_step_val_entry.grid(column=1, row=2)

        self.aphi_pause_time_label = tk.Label(aphi_tab, text="Время остановки")
        self.aphi_pause_time_label.grid(column=0, row=3, padx=10, pady=10)

        self.aphi_pause_time_entry = tk.Entry(aphi_tab)
        self.aphi_pause_time_entry.grid(column=1, row=3)

        self.aphi_velocity_label = tk.Label(aphi_tab, text="Скорость, м/с")
        self.aphi_velocity_label.grid(column=0, row=4, padx=1, pady=10)

        self.aphi_velocity_entry = tk.Entry(aphi_tab)
        self.aphi_velocity_entry.grid(column=1, row=4)

        self.start_aphi_btn = tk.Button(aphi_tab, text="Начать продвижение", command=self.start_aphi)
        self.start_aphi_btn.grid(column=0, row=5)

        self.stop_aphi_btn = tk.Button(aphi_tab, text="Остановить продвижение", command=self.stop_aphi)
        self.stop_aphi_btn.grid(column=0, row=6)

        self.return_to_zero_btn = tk.Button(aphi_tab, text="Вернуться в исходную точку", command=self.fuck_go_back)
        self.return_to_zero_btn.grid(column=1, row=5)

        self.save_aphi_btn = tk.Button(aphi_tab, text="Сохранить данные пути", command=self.save_aphi)
        self.save_aphi_btn.grid(column=1, row=6)

        self.ashido_position_btn = tk.Button(ashido_tab, text="Установить хирурга в начальную точку",
                                             command=self.ashido_pos)
        self.ashido_position_btn.grid(column=0, row=0)
        self.ashido_start_btn = tk.Button(ashido_tab, text="НАЧАТЬ", command=self.ashido_start)
        self.ashido_start_btn.grid(column=0, row=1)

        self.is_diagnost_connected_label = tk.Label(telemetry_tab, text="Статус подключения диагноста:")
        self.is_diagnost_connected_label.grid(column=0, row=0)

        self.diagnost_status_label = tk.Label(telemetry_tab, text=" ")
        self.diagnost_status_label.grid(column=1, row=0)

        self.is_hirurg_connected_label = tk.Label(telemetry_tab, text="Статус подключения хирурга:")
        self.is_hirurg_connected_label.grid(column=0, row=1)

        self.hirurg_status_label = tk.Label(telemetry_tab, text=" ")
        self.hirurg_status_label.grid(column=1, row=1)

        self.diagnost_las_label = tk.Label(telemetry_tab, text="Показания лазерного датчика диагноста:")
        self.diagnost_las_label.grid(column=0, row=2)

        self.diagnost_las_data_label = tk.Label(telemetry_tab, text=" ")
        self.diagnost_las_data_label.grid(column=1, row=2)

        self.hirurg_las_label = tk.Label(telemetry_tab, text="Показания лазерного датчика хирурга:")
        self.hirurg_las_label.grid(column=0, row=3)

        self.hirurg_las_data_label = tk.Label(telemetry_tab, text=" ")
        self.hirurg_las_data_label.grid(column=1, row=3)

        self.diagnost_force_label = tk.Label(telemetry_tab, text="Показания силового датчика диагноста:")
        self.diagnost_force_label.grid(column=0, row=4)

        self.diagnost_force_data_label = tk.Label(telemetry_tab, text=" ")
        self.diagnost_force_data_label.grid(column=1, row=4)

        self.hirurg_force_label = tk.Label(telemetry_tab, text="Показания силового датчика хирурга:")
        self.hirurg_force_label.grid(column=0, row=5)

        self.hirurg_force_data_entry = tk.Entry(telemetry_tab, textvariable="0", state="disabled")
        self.hirurg_force_data_entry.grid(column=1, row=5)

        self.control_h_process = mp.Process(target=Joystick_hirurg.main,
                                            args=(self.heartbeat, auto, program_lock, hirurg_path), daemon = True, name="hirurg_control")
        self.control_d_process = mp.Process(target=Joystick_diagnost.main,
                                            args=(self.heartbeat, auto, shared_path, force_lock, program_lock), daemon = True, name ="diagnost_control")
        self.align_h_process = mp.Process(target=Align_H.main)
        self.power_on_h_process = mp.Process(target=Power_On_H.main)
        self.power_on_d_process = mp.Process(target=Power_On_D.main)
        self.power_off_h_process = mp.Process(target=Power_Off_H.main)
        self.power_off_d_process = mp.Process(target=Power_Off_D.main)
        self.align_d_process = mp.Process(target=Align_D.main)
        self.unlock_h_process = mp.Process(target=ESTOP_RESET_H.main)
        self.unlock_d_process = mp.Process(target=ESTOP_RESET_D.main)
        self.cam_d_process = mp.Process(target=camDiagn.main)
        self.cam_h_process = mp.Process(target=camHirurg.main)

        self.telemetry_logger = CSVLogger("logs/telemetry_log.csv")
        self.tel_logging_var = tk.BooleanVar(value=False)

        self.toggle_telemetry_log_label = ttk.Label(telemetry_tab, text="Управление логированием")
        self.toggle_telemetry_log_label.grid(column =0, row=6)

        self.toggle_telemetry_log_btn = ttk.Checkbutton(telemetry_tab, text="Включить логирование в csv", variable=self.tel_logging_var, command=self.toggle_telemetry_logging)
        self.toggle_telemetry_log_btn.grid(column=0,row=7)

        self.telemetry_logging_status_label = ttk.Label(telemetry_tab, text="Логирование телеметрии: ОТКЛЮЧЕНО", foreground="red")
        self.telemetry_logging_status_label.grid(column=1, row=7)


        self.monitor = Thread(target=self.system_monitor, daemon=True)
        self.monitor.start()

        self.watchdog_thread = Thread(target = self.watchdog, daemon=True, name= "Watchdog")
        self.watchdog_thread.start()
        logger.info(f"Запущен фоновый монитор сердцебиения (поток: {self.watchdog_thread.name})")

        # self.force_control_thread = Thread(target=force_control, args = (17,))
        # self.force_control_thread.start()

    def toggle_telemetry_logging(self):
        if self.tel_logging_var.get():
            self.telemetry_logger.enable()
            self.telemetry_logging_status_label.config(text="Логирование: ВКЛЮЧЕНО", foreground="green")
        else:
            self.telemetry_logger.disable()
            self.telemetry_logging_status_label.config(text="Логирование: ВЫКЛЮЧЕНО", foreground="red")

    def system_launch(self):
        print('System launched')
        self.system_launch_btn.configure(state=tk.DISABLED)
        self.system_stop_btn.configure(state=tk.NORMAL)
        self.control_launch_btn.configure(state=tk.NORMAL)
        self.align_btn.configure(state=tk.NORMAL)
        self.power_on_h_process.start()
        self.power_on_d_process.start()

        self.monitor.start()

    def system_stop(self):
        print('system_stopped')
        self.system_launch_btn.configure(state=tk.NORMAL)
        self.system_stop_btn.configure(state=tk.DISABLED)
        self.control_stop_btn.configure(state=tk.DISABLED)
        self.control_launch_btn.configure(state=tk.DISABLED)
        self.align_btn.configure(state=tk.DISABLED)
        self.power_off_h_process = mp.Process(target=Power_Off_H.main)
        self.power_off_d_process = mp.Process(target=Power_Off_D.main)

    def control_launch(self):
        sleep(1)
        global us_lock
        print(self.control_d_process.exitcode)
        if not us_lock:
            if (not self.control_h_process.is_alive()) or (not self.control_d_process.is_alive()):
                self.control_h_launch()
                self.control_d_launch()

            else:
                program_lock.value = 0
            self.control_stop_btn.configure(state=tk.NORMAL)
            self.control_launch_btn.configure(state=tk.DISABLED)
        else:
            self.show_error("ППУИ запущено!")

    def control_d_launch(self):
        if not self.control_d_process.is_alive():
            self.control_d_process.start()
            sleep(5)
            if self.control_d_process.is_alive():
                print("diagnost control launched")
                self.attempt_d_counter = 0
                params = (self.heartbeat, auto, shared_path, force_lock, program_lock)
                self.processes['diagnost_control'] = {
                    'process': self.control_d_process,
                    'last_heartbeat': time.time(),
                    'start_time': time.time(),
                    'params': params,
                    'state': 'STARTING',
                    'pid': self.control_d_process.pid,
                    'target': Joystick_diagnost.main
                }
                logger.info(
                    f'Запущен процесс {"diagnost_control"} | PID {self.control_d_process.pid} | Taймаут сердцебиения: {self.heartbeat_timeout}s'
                )
                self._update_status('diagnost_control', "STARTING", "orange")
            else:
                print("Error while initiating diagnost control")
                self.control_stop(D=1)
                sleep(5)
                if self.attempt_d_counter <= 5:
                    print("Retrying to initiate diagnost control")
                    self.attempt_d_counter += 1
                    self.control_d_launch()
                else:
                    self.show_error(
                        "Не удаётся запустить управление диагностом\nПроверьте состояние робота, он должен быть включен и разблокирован.")
                    self.attempt_d_counter = 0

    def control_h_launch(self):
        if not self.control_h_process.is_alive():
            self.control_h_process.start()
            sleep(5)
            if self.control_h_process.is_alive():
                print("hirurg control launched")
                self.attempt_h_counter = 0
                params = (self.heartbeat, auto, program_lock, hirurg_path)
                self.processes['hirurg_control'] = {
                    'process': self.control_h_process,
                    'last_heartbeat': time.time(),
                    'start_time': time.time(),
                    'params': params,
                    'state': 'STARTING',
                    'pid': self.control_h_process.pid,
                    'target': Joystick_hirurg.main
                }
                logger.info(
                    f'Запущен процесс {"hirurg_control"} | PID {self.control_d_process.pid} | Taймаут сердцебиения: {self.heartbeat_timeout}s'
                )
                self._update_status('hirurg_control', "STARTING", "orange")
            else:
                print("Error while initiating hirurg control")
                self.control_stop(H=1)
                sleep(10)
                if self.attempt_h_counter <= 5:
                    print("Retrying to initiate hirurg control")
                    self.attempt_h_counter += 1
                    self.control_h_launch()
                else:
                    self.show_error(
                        "Не удаётся запустить управление хирургом\nПроверьте состояние робота, он должен быть включен и разблокирован.")
                    self.attempt_h_counter = 0

    def control_stop(self, H=0, D=0):
        print('control stopped')
        self.control_stop_btn.configure(state=tk.DISABLED)
        self.control_launch_btn.configure(state=tk.NORMAL)
        print(self.control_d_process)
        print(self.control_h_process)
        if D:
            print("Reinitiating diagnost control")
            self.control_d_process.terminate()
            self.control_d_process.join(timeout=3.0)
            print(self.control_d_process)
            sleep(1)
            self.control_d_process = mp.Process(target=Joystick_diagnost.main,
                                                args=(self.heartbeat, auto, shared_path, force_lock, program_lock),
                                                daemon=True, name="diagnost_control")
            print(self.control_d_process)
        elif H:
            self.control_h_process.terminate()
            self.control_h_process.join(timeout=3.0)
            print("Reinitiating hirurg control")
            print(self.control_h_process)
            sleep(1)
            self.control_h_process = mp.Process(target=Joystick_hirurg.main,
                                                args=(self.heartbeat, auto, program_lock, hirurg_path), daemon=True, name="hirurg_control")
            print(self.control_h_process)
        else:
            program_lock.value = 1

    def align(self):
        print('aligned')
        self.align_d_process = subprocess.Popen(['python', 'Align_D.py'])

    def unlock(self):
        print('unlocked')
        self.unlock_h_process = subprocess.Popen(['python', 'ESTOP_RESET_D.py'])
        self.unlock_d_process = subprocess.Popen(['python', 'ESTOP_RESET_H.py'])

    def cam_d(self):
        # if not self.cam_d_state:
        #     print('Diagnost cam on')
        #     self.cam_d_state = True
        #     self.cam_d_process = subprocess.Popen(['python', 'camDiagn.py'])
        # else:
        #     self.cam_d_process.kill()
        #     print('Diagnost cam off')
        #     self.cam_d_state = False
        self.show_error("Превышен порог силового давления!")
        self.show_error("Начальная точка маршрута отсутствует")
        self.show_warning('Управление диагноста отключено')
        self.show_error("Робот в движении")
        self.show_error("Данные отсутствуют")
        self.show_error(
            "Не удаётся запустить управление хирургом\nПроверьте состояние робота, он должен быть включен и разблокирован.")
        self.show_error("ППУИ запущено!")

    def cam_h(self):
        if not self.cam_h_state:
            print('Hirurg cam on')
            self.cam_d_state = True
            self.cam_h_process = subprocess.Popen(['python', 'camHirurg.py'])
        else:
            self.cam_h_process.kill()
            print('Hirurg cam off')
            self.cam_h_state = False

    def start_route(self):
        if (program_lock.value == 0):
            program_lock.value = 1
            self.show_warning('Управление диагноста отключено')
            sleep(5)
        nsteps = int(self.num_steps_entry.get())
        SX = float(self.step_x_entry.get())
        SY = float(self.step_y_entry.get())
        SZ = float(self.step_z_entry.get())
        pause = int(self.pause_time_entry.get())
        vel = float(self.velocity_entry.get())
        us_thread = Thread(target=route_follow, args=(nsteps, [-SX, -SY, - SZ], pause, vel))
        us_thread.start()

    def start_aphi(self):
        if program_lock.value == 0:
            program_lock.value = 1
            self.show_warning('Управление диагноста отключено')
            sleep(5)
        angle = float(self.move_angle_entry.get())
        nsteps = int(self.aphi_num_steps_entry.get())
        step = float(self.aphi_step_val_entry.get())
        pause = int(self.aphi_pause_time_entry.get())
        vel = float(self.aphi_velocity_entry.get())
        aphi_thread = Thread(target=aphi, args=(angle, nsteps, step, pause, vel))
        aphi_thread.start()

    def stop_aphi(self):
        pass

    def fuck_go_back(self):
        pass

    def save_aphi(self):
        pass

    def ashido_pos(self):
        if program_lock.value == 0:
            program_lock.value = 1
            self.show_warning('Управление диагноста отключено')
            sleep(5)
        angle = float(self.move_angle_entry.get()) if self.aphi_velocity_entry.get() != "" else 0
        nsteps = int(self.aphi_num_steps_entry.get())
        step = float(self.aphi_step_val_entry.get())
        pause = int(self.aphi_pause_time_entry.get())
        vel = float(self.aphi_velocity_entry.get())
        ashido_thread = Thread(target=ashido_init, args=(nsteps, step, pause, angle))
        ashido_thread.start()

    def ashido_start(self):
        if program_lock.value == 0:
            program_lock.value = 1
            self.show_warning('Управление диагноста отключено')
            sleep(5)
        angle = float(self.move_angle_entry.get()) if self.aphi_velocity_entry.get() != "" else 0
        nsteps = int(self.aphi_num_steps_entry.get())
        step = float(self.aphi_step_val_entry.get())
        pause = int(self.aphi_pause_time_entry.get())
        vel = float(self.aphi_velocity_entry.get())
        ashido_thread = Thread(target=ashido, args=(nsteps, step, pause, vel, angle))
        ashido_thread.start()

    def save_route(self):
        if us_lock and False:
            self.show_error("Робот в движении")
            return 228
        else:
            file_path = filedialog.asksaveasfilename(defaultextension=".txt")
            if file_path:
                with open(file_path, "w") as file:
                    file.write("Step | Time,s | Coordinate | Force, N\n")
                    try:
                        for x in range(len(rob_us_data[0])):
                            file.write(f'{x} {rob_us_data[0][x]} {rob_us_data[1][x]} {rob_us_data[2][x]}\n')
                    except IndexError:
                        self.show_error("Данные отсутствуют")

    def save_d_route(self):
        print(shared_path)
        if (len(shared_path[0])):
            file_path = filedialog.asksaveasfilename(defaultextension=".txt")
            if file_path:
                with open(file_path, "w") as file:
                    for pos in shared_path[0]:
                        file.write(str(pos[0]) + "|" + str(pos[1]) + "|" + str(pos[2]) + "\n")
                shared_path[0] = []
        else:
            self.show_error("Путь отсутствует")

    def save_h_route(self):
        if (len(hirurg_path[0])):
            file_path = filedialog.asksaveasfilename(defaultextension=".txt")
            if file_path:
                with open(file_path, "w") as file:
                    for pos in hirurg_path[0]:
                        file.write(str(pos[0]) + "|" + str(pos[1]) + "|" + str(pos[2]) + "\n")
                hirurg_path[0] = []
        else:
            self.show_error("Путь отсутствует")

    def launch_h_route(self):
        file_path = filedialog.askopenfile(title="Выберите файл маршрута хирурга", filetypes=[("Text files", "*.txt")])

        pass

    def stop_routef(self):
        global stop_route
        stop_route.set()

    def return_to_start(self):
        global starting_pose
        if starting_pose:
            rob = urx.Robot("192.168.8.3", use_rt=True)
            rob.movel(starting_pose)
        else:
            self.show_error("Начальная точка маршрута отсутствует")

    def launch_d_route(self):
        pass

    def system_monitor(self):
        diagnost = urx.Robot("192.168.8.3", use_rt=True)
        hirurg = urx.Robot("192.168.8.4", use_rt=True)
        while True:
            diagnost_status = diagnost.is_running()
            hirurg_status = hirurg.is_running()

            diagnost_force = diagnost.get_tcp_force()
            hirurg_force = hirurg.get_tcp_force()

            diagnost_pose = diagnost.getl()
            hirurg_pose = hirurg.getl()

            diagnost_joints = diagnost.getj()
            hirurg_joints = hirurg.getj()

            # print(diagnost_force)
            self.diagnost_force_data_label.config(
                text=f'X:{diagnost_force[0]:.2f}, Y: {diagnost_force[1]:.2f}, Z: {diagnost_force[2]:.2f}')

            self.telemetry_logger.log_data(["диагност"]+list(diagnost_pose[:3]) +list(diagnost_joints) + list(diagnost_force))
            self.telemetry_logger.log_data(["хирург"] + list(hirurg_pose[:3]) + list(hirurg_joints) + list(hirurg_force))
            time.sleep(1)

            # self.diagnost_force_data_entry.set(diagnost_force)

    def watchdog(self):
        logger.debug("Запущен цикл мониторинга сердцебиения")
        while True:
            current_time = time.time()
            while not self.heartbeat.empty():
                msg = self.heartbeat.get()
                name = msg[0]
                if name not in self.processes:
                    logger.debug(f"Получено сообщение от неизвестного процесса {name}: {msg}")
                    continue

                proc = self.processes[name]
                msg_type = msg[1]

                if msg_type == "READY":
                    proc['state'] = "READY"
                    proc['last_heartbeat'] = msg[2]
                    logger.info(f"Процесс {name} (PID: {proc['pid']}) готов к работе")
                    self._update_status(name, "READY", "lightgreen")

                elif msg_type == "ALIVE":
                    prev_state = proc["state"]
                    proc["state"] = "RUNNING"
                    proc["last_heartbeat"] = msg[2]

                    if prev_state != "RUNNING" or (current_time - getattr(proc, "last_log_time", 0)) > 30:
                        uptime = current_time - proc["start_time"]
                        logger.debug(
                            f"Процесс {name} активен | PID: {proc['pid']} | Аптайм: {uptime:.1f}s | Последнее сердцебиение: {current_time - proc['last_heartbeat']:.2f}s назад")
                        proc["last_log_time"] = current_time

                    self._update_status(name, "RUNNING", "green")

                elif msg_type == "ERROR":
                    error_msg = msg[2]
                    logger.error(f"Процесс {name} (PID: {proc['pid']}) ошибка: {error_msg}")
                    self._force_restart(name, reason=f"Ошибка: {error_msg}")

                elif msg_type == "CRASH":
                    error_msg = msg[2]
                    logger.critical(f"Процесс {name} (PID: {proc['pid']}) аварийно завершился: {error_msg}")
                    self._force_restart(name, reason=f"Аварийное завершение: {error_msg}")

            for name in list(self.processes.keys()):
                proc = self.processes[name]
                time_since_hb = current_time - proc["last_heartbeat"]

                if time_since_hb > self.heartbeat_timeout:
                    if not proc['process'].is_alive():
                        logger.warning(
                            f"Процесс {name} (PID: {proc['pid']}) завершился без уведомления. Последнее сердцебиение: {time_since_hb:.1f}s назад")
                        self._force_restart(name, reason="Неожиданное завершение процесса")
                    else:
                        logger.critical(
                            f"ОБНАРУЖЕНО ЗАВИСАНИЕ: процесс {name} (PID: {proc.get('pid', 'N/A')}) не отвечает "
                            f"в течение {time_since_hb:.1f}s (тайм: {self.heartbeat_timeout}s) "
                            f"Состояние: {proc['state']}"
                        )
                        self._force_restart(name, reason="Зависание процесса (таймаут сердцебиения)")

            time.sleep(1.0)

    def _update_status(self, name, state, color):
        pass

    def _schedule_restart(self, name, reason="Плановый перезапуск"):
        pass

    def _actual_restart(self, name, target_func, params, reason=""):
        self.restart_delays[name] = 1.0

        logger.info(f"Перезапуск процесса {name} | Причина: {reason}")

        p = mp.Process(target=target_func, args=params, daemon=True, name=name)
        p.start()

        self.processes[name] = {
            'process': p,
            'last_heartbeat': time.time(),
            'start_time': time.time(),
            'params': params,
            'state': 'STARTING',
            'pid': p.pid,
            'target': target_func
        }
        self._update_status(name, "RESTARTING", "orange")

    def _force_restart(self, name, reason="Неизвестная причина"):
        proc = self.processes[name]
        if not proc:
            logger.warning(f"Попытка перезапуска несуществующего процесса {name}")
            return

        p = proc["process"]
        pid = proc.get('pid', 'N/A')

        logger.warning(f"Начало принудительного перезапуска {name} (PID: {pid}) | Причина: {reason}")

        if p.is_alive():
            logger.debug(f"Отправка SIGTERM процессу {name} (PID: {pid})")
            p.terminate()
            p.join(timeout=3.0)

        if p.is_alive():
            logger.error(f"Процесс {name} (PID: {pid}) не отвечает на SIGTERM, отправка SIGKILL")
            p.kill()
            p.join(timeout=2.0)

        if p.is_alive():
            logger.critical(f"НЕВОЗМОЖНО ЗАВЕРШИТЬ процесс {name} даже после SIGKILL!")
        else:
            logger.info(f"Процесс {name} (PID: {pid}) успешно завершён")

        if name in self.processes:
            del self.processes[name]

        delay = self.restart_delays.get(name, 1.0)
        self.restart_delays[name] = min(delay * 2, 30.0)

        logger.info(f"Планирование перезапуска {name} через {delay:.1f}с (экспоненциальная задержка)")

        threading.Timer(delay, self._actual_restart, args=(name, proc['target'], proc['params'], reason)).start()

    def kill_all(self):
        pass

    def show_error(self, msg):
        mb.showerror("ОШИБКА!", msg)

    def show_warning(self, msg):
        mb.showwarning("ВНИМАНИЕ!", msg)

def on_closing():
    logger.info(f"Получен сигнал закрытия приложения")
    for name in list(app.processes.keys()):
        app.processes[name]["process"].kill()
        app.processes[name]["process"].join(timeout=2.0)
    logger.info("Все процессы остановлены. Завершение работы.")
    root.destroy()

if __name__ == '__main__':
    logger = setup_status_logging()
    mp.freeze_support()
    proxy = mp.Manager()
    shared_path = proxy.list()
    shared_path.append([])
    hirurg_path = proxy.list()
    hirurg_path.append([])
    root = tk.Tk()
    app = ScriptRunnerApp(root)
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()
