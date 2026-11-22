
import os
import time
import math
import pybullet as p
import pybullet_data

# Ścieżka do pliku URDF
URDF_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "robot_arm.urdf"
)

TIME_STEP = 1.0 / 240.0


def main():
    physics_client = p.connect(p.GUI)

    if physics_client < 0:
        raise RuntimeError("Nie udało się uruchomić PyBullet.")

    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(TIME_STEP)
        p.setRealTimeSimulation(0)

        # Podłoże
        p.loadURDF("plane.urdf")

        # Wczytanie robota
        robot = p.loadURDF(
            URDF_PATH,
            basePosition=[0, 0, 0.02],
            useFixedBase=True,
            flags=p.URDF_USE_INERTIA_FROM_FILE
        )

        # Kamera
        p.resetDebugVisualizerCamera(
            cameraDistance=0.8,
            cameraYaw=45,
            cameraPitch=-25,
            cameraTargetPosition=[0, 0, 0.2]
        )

        # Odczyt przegubów
        joints = {}

        print("\n--- PRZEGUBY MODELU ---")

        for i in range(p.getNumJoints(robot)):
            info = p.getJointInfo(robot, i)
            name = info[1].decode("utf-8")
            joint_type = info[2]

            print(f"{i}: {name}, typ={joint_type}")

            if joint_type != p.JOINT_FIXED:
                joints[name] = i

        # Przeguby, które mają być testowane
        test_joints = [
            "cylinder_sfera1",
            "sfera1_walec1",
            "sfera2_walec2",
        ]

        for name in test_joints:
            if name not in joints:
                raise RuntimeError(
                    f"Nie znaleziono przegubu '{name}'. "
                    "Sprawdź nazwy w pliku URDF."
                )

        print("\nRozpoczynam test ruchu.")
        print("Zamknij okno PyBullet lub naciśnij Ctrl+C, aby zakończyć.")

        # Wyłączenie domyślnych napędów
        for joint_id in joints.values():
            p.setJointMotorControl2(
                robot,
                joint_id,
                p.VELOCITY_CONTROL,
                force=0
            )

        start = time.time()

        while p.isConnected():
            t = time.time() - start

            # Kąty docelowe w radianach
            targets = {
                # Cylinder -> sfera 1: oś Z, zakres +/- 1.4 rad
                "cylinder_sfera1":
                    0.5 * math.sin(0.7 * t),

                # Bark: oś Y, zakres +/- 1.4 rad
                "sfera1_walec1":
                    0.6 * math.sin(0.5 * t),

                # Łokieć: oś Y, zakres od -2.0944 do 0
                "sfera2_walec2":
                    -1.0 + 0.7 * math.sin(0.6 * t),
            }

            for name, target in targets.items():
                p.setJointMotorControl2(
                    bodyUniqueId=robot,
                    jointIndex=joints[name],
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=target,
                    force=20,
                    maxVelocity=0.5,
                    positionGain=0.15,
                    velocityGain=1.0
                )

            p.stepSimulation()
            time.sleep(TIME_STEP)

    except KeyboardInterrupt:
        print("\nTest przerwany przez użytkownika.")

    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    main()
