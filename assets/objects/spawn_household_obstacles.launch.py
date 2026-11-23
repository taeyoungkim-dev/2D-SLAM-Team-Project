import atexit
import xml.etree.ElementTree as ET
from pathlib import Path
from tempfile import TemporaryDirectory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 어느 디렉터리에서 실행해도 이 파일 옆의 에셋을 쓴다(통합 launch 에서도 include 할 수 있게).
    asset_dir = Path(__file__).resolve().parent
    cable_dir = asset_dir / "tangled_power_cable"

    # 케이블 SDF의 두 메시 경로를 절대 경로로 바꿉니다.
    cable_model = ET.parse(cable_dir / "model.sdf")
    for uri in cable_model.findall(".//mesh/uri"):
        mesh_path = cable_dir / "meshes" / Path(uri.text).name
        uri.text = str(mesh_path.resolve(strict=True))

    # 스폰 노드에 전달할 임시 SDF를 만들고, 런치 종료 시 정리합니다.
    temp_dir = TemporaryDirectory(prefix="gazebo_cable_")
    atexit.register(temp_dir.cleanup)
    cable_sdf = Path(temp_dir.name) / "power_cable.sdf"
    cable_model.write(cable_sdf, encoding="utf-8", xml_declaration=False)

    # 모델 이름, SDF 경로, x, y, yaw
    objects = (
        ("power_cable", cable_sdf, "-0.737", "-1.52", "-0.323"),
        ("cardboard_box", asset_dir / "cardboard_box.sdf", "0.58", "-0.56", "-0.66"),
        ("urine_puddle", asset_dir / "urine_puddle.sdf", "0.60", "-1.53", "2.81"),
    )

    return LaunchDescription([
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity", name,
                "-file", str(sdf_path),
                "-x", x,
                "-y", y,
                "-z", "0",
                "-Y", yaw,
            ],
            output="screen",
        )
        for name, sdf_path, x, y, yaw in objects
    ])