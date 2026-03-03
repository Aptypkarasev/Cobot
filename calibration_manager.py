import math3d as m3d
import numpy as np
from scipy.linalg import svd
import json
import os
from scipy.spatial.transform import Rotation as R_scipy


class CalibrationManager:
    def __init__(self, filename="calibration_data.json"):
        self.filename = filename
        self.transform = None  # m3d.Transform (Base1 -> Base2)

    def calculate_transform_from_points(self, points1, points2):
        """
        Вычисляет трансформацию между двумя наборами точек.
        points1/2: список m3d.Vector или списков [x,y,z]
        Возвращает: m3d.Transform
        """
        # Конвертируем в numpy для SVD
        if isinstance(points1[0], m3d.Vector):
            pts1 = np.array([p.get_array() for p in points1])
            pts2 = np.array([p.get_array() for p in points2])
        else:
            pts1 = np.array(points1)
            pts2 = np.array(points2)

        # 1. Центроиды
        c1 = np.mean(pts1, axis=0)
        c2 = np.mean(pts2, axis=0)

        # 2. Центрирование
        Q1 = pts1 - c1
        Q2 = pts2 - c2

        # 3. Матрица ковариации
        H = np.dot(Q1.T, Q2)

        # 4. SVD разложение
        U, S, Vt = svd(H)
        R = np.dot(Vt.T, U.T)

        # 5. Проверка на отражение
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = np.dot(Vt.T, U.T)

        # 6. Вектор переноса
        t = c2 - np.dot(R, c1)

        # 7. Создаём m3d.Transform
        self.transform = m3d.Transform(
            m3d.Orientation(R),
            m3d.Vector(t)
        )
        return self.transform

    def save(self, metadata=None):
        """Сохраняет трансформацию в JSON"""
        if self.transform is None:
            raise ValueError("Нет данных для сохранения")

        R = self.transform.orient.get_array()
        t = self.transform.pos.get_array()

        data = {
            "R": R.tolist(),
            "t": t.tolist()
        }
        if metadata:
            data["metadata"] = metadata

        with open(self.filename, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=4)
        print(f"✓ Калибровка сохранена в {self.filename}")

    def load(self):
        """Загружает трансформацию из файла"""
        if not os.path.exists(self.filename):
            raise FileNotFoundError(f"Файл {self.filename} не найден!")

        with open(self.filename, 'r', encoding='utf-8') as f:
            data = json.load(f)

        R = np.array(data['R'])
        t = np.array(data['t'])

        self.transform = m3d.Transform(
            m3d.Orientation(R),
            m3d.Vector(t)
        )
        print(f"✓ Калибровка загружена из {self.filename}")
        return self.transform

    def transform_pose(self, pose_list):
        """
        Трансформирует позу из Base1 в Base2.

        Args:
            pose_list: [x,y,z, rx,ry,rz] от rob.getl() (в метрах и радианах)

        Returns:
            [x,y,z, rx,ry,rz] для второго робота
        """
        if self.transform is None:
            raise ValueError("Сначала загрузите данные калибровки (load)!")

        # Создаём трансформацию из позы Робота 1
        T1 = m3d.Transform(
            m3d.Orientation(pose_list[3:]),
            m3d.Vector(pose_list[:3])
        )

        # Применяем калибровочную трансформацию: T2 = T_calib * T1
        T2 = self.transform * T1

        # Конвертируем ориентацию (матрица 3x3) обратно в вектор поворота
        orient_matrix = T2.orient.get_array()
        rot_vec = R_scipy.from_matrix(orient_matrix).as_rotvec()

        # Собираем итоговый список для urx.movel()
        pos = T2.pos.get_array()
        pose_out = list(pos) + list(rot_vec)

        return pose_out

    def verify(self):
        """Проверяет корректность матрицы поворота"""
        if self.transform is None:
            return False
        R = self.transform.orient.get_array()
        det = np.linalg.det(R)
        ortho = np.allclose(np.dot(R, R.T), np.eye(3), atol=1e-6)
        return abs(det - 1.0) < 0.01 and ortho

    def get_translation_distance(self):
        """Возвращает расстояние между базами в метрах"""
        if self.transform is None:
            return 0
        return self.transform.pos.length

    def transform_position_only(self, pose_list, orientation_offset=None):
        """
        Трансформирует ТОЛЬКО ПОЗИЦИЮ из Base1 в Base2.
        Ориентация задаётся независимо для безопасности.

        Args:
            pose_list: [x,y,z, rx,ry,rz] от rob.getl()
            orientation_offset: [rx,ry,rz] в радианах - дополнение к ориентации
                               (например, [0, 0, 3.14] для разворота на 180°)

        Returns:
            [x,y,z, rx,ry,rz] для второго робота
        """
        if self.transform is None:
            raise ValueError("Сначала загрузите данные калибровки (load)!")

        # 1. Трансформируем позицию через калибровочную матрицу
        pos1 = np.array(pose_list[:3])
        pos2 = np.dot(self.transform.orient.get_array(), pos1) + self.transform.pos.get_array()

        # 2. Ориентацию задаём независимо (для безопасности)
        if orientation_offset is None:
            # По умолчанию: разворот на 180° вокруг Z (роботы смотрят друг на друга)
            orientation_offset = [0, 0, np.pi]

        # Базовая ориентация (можно настроить под вашу задачу)
        base_orientation = [0, 0, 0]  # Или [np.pi, 0, 0] для вертикального подхода

        # Итоговая ориентация = базовая + оффсет
        final_orientation = list(np.array(base_orientation) + np.array(orientation_offset))

        return list(pos2) + final_orientation