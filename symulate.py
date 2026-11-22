
import os
import time
import math

import pybullet as p
import pybullet_data


URDF_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "robot_arm.urdf"
)

TIME_STEP = 1.0 / 240.0
MAX_SIMULATION_TIME = 30.0


def main():
    physics_client = p.connect(p.GUI)

    if physics_client < 0:
        raise RuntimeError("Nie udało się uruchomić PyBullet GUI")

    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(TIME_STEP)
        p.setRealTimeSimulation(0)

        p.loadURDF("plane.urdf")

        robot = p.loadURDF(
            URDF_PATH,
            basePosition=[0, 0, 0],
            useFixedBase=True
        )

        p.resetDebugVisualizerCamera(
            cameraDistance=35,
            cameraYaw=45,
            cameraPitch=-25,
            cameraTargetPosition=[0, 0, 12]
        )

        joint_ids = {}

        for i in range(p.getNumJoints(robot)):
            info = p.getJointInfo(robot, i)
            name = info[1].decode("utf-8")
            joint_type = info[2]

            print(f"Joint {i}: {name}, type={joint_type}")

            if joint_type in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
                joint_ids[name] = i

        print("Dostępne ruchome przeguby:", joint_ids)

        # Sterujemy wyłącznie przegubami istniejącymi jako ruchome w URDF.
        requested_joints = [
            "test_joint",
            "sfera1_walec1",
            "sfera2_walec2",
            "walec2_midjoint_walec2_sfera3",
        ]

        controlled_joints = [
            name for name in requested_joints
            if name in joint_ids
        ]

        if not controlled_joints:
            raise RuntimeError(
                "Nie znaleziono przegubów do sterowania. "
                "Sprawdź nazwy w URDF."
            )

        print("Sterowane przeguby:", controlled_joints)

        start_time = time.monotonic()
        step_count = 0
        max_steps = int(MAX_SIMULATION_TIME / TIME_STEP)

        while p.isConnected() and step_count < max_steps:
            t = time.monotonic() - start_time

            targets = {
                "test_joint": 0.5 * math.sin(0.5 * t),
                "sfera1_walec1": 0.8 * math.sin(0.7 * t),
                "sfera2_walec2": 0.6 * math.sin(0.4 * t),
                "walec2_midjoint_walec2_sfera3":
                    0.8 * math.sin(0.6 * t),
            }

            for name in controlled_joints:
                p.setJointMotorControl2(
                    bodyUniqueId=robot,
                    jointIndex=joint_ids[name],
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=targets[name],
                    force=10,
                    maxVelocity=1.0
                )

            p.stepSimulation()
            step_count += 1

            # Nie usypiaj na dłużej, jeśli użytkownik zamknął GUI.
            time.sleep(TIME_STEP)

        print("Koniec symulacji.")

        # Pozostaw GUI otwarte, ale nie wykonuj kolejnych kroków fizyki.
        while p.isConnected():
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Przerwano symulację.")

    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    main()
