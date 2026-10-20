import tempfile
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from raicom_control.lightweight_urdf import build_lightweight_urdf


ASCII_STL = """solid offset_box
facet normal 0 0 1
  outer loop
    vertex 1 2 3
    vertex 3 2 3
    vertex 1 6 3
  endloop
endfacet
facet normal 0 0 -1
  outer loop
    vertex 1 2 8
    vertex 3 2 8
    vertex 1 6 8
  endloop
endfacet
endsolid offset_box
"""


class LightweightUrdfTest(unittest.TestCase):
    def test_replaces_meshes_without_changing_joint_definition(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            mesh = root / "meshes" / "arm" / "link.STL"
            mesh.parent.mkdir(parents=True)
            mesh.write_text(ASCII_STL, encoding="utf-8")
            source = root / "source.urdf"
            source.write_text(
                """<?xml version='1.0'?>
<robot name='test'>
  <link name='base'/>
  <link name='link1'>
    <inertial><mass value='1'/><inertia ixx='1' ixy='0' ixz='0' iyy='1' iyz='0' izz='1'/></inertial>
    <visual><origin xyz='0 0 0' rpy='0 0 0'/><geometry>
      <mesh filename='package://landerpi_description/meshes/arm/link.STL'/>
    </geometry></visual>
    <collision><geometry>
      <mesh filename='package://landerpi_description/meshes/arm/link.STL'/>
    </geometry></collision>
  </link>
  <joint name='joint1' type='fixed'>
    <origin xyz='0.1 0.2 0.3' rpy='0 0 0'/>
    <parent link='base'/><child link='link1'/>
  </joint>
</robot>
""",
                encoding="utf-8",
            )
            output = root / "light.urdf"

            manifest = build_lightweight_urdf(source, output, root)
            tree = ET.parse(output)

            self.assertEqual(manifest["links"], 2)
            self.assertEqual(manifest["joints"], 1)
            self.assertEqual(manifest["inertials"], 1)
            self.assertEqual(manifest["replacements"], 2)
            self.assertEqual(manifest["remaining_meshes"], 0)
            self.assertFalse(tree.findall(".//mesh"))
            self.assertEqual(
                tree.find("./joint/origin").attrib,
                {"xyz": "0.1 0.2 0.3", "rpy": "0 0 0"},
            )
            visual = tree.find("./link[@name='link1']/visual")
            self.assertEqual(visual.find("origin").get("xyz"), "2 4 5.5")
            self.assertEqual(visual.find("geometry/box").get("size"), "2 4 5")

    def test_wheel_becomes_cylinder(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            mesh = root / "meshes" / "wheel.STL"
            mesh.parent.mkdir(parents=True)
            mesh.write_text(ASCII_STL, encoding="utf-8")
            source = root / "source.urdf"
            source.write_text(
                """<robot name='test'><link name='wheel_left_link'><visual>
<geometry><mesh filename='package://landerpi_description/meshes/wheel.STL'/></geometry>
</visual></link></robot>""",
                encoding="utf-8",
            )
            output = root / "light.urdf"

            manifest = build_lightweight_urdf(source, output, root)
            tree = ET.parse(output)

            cylinder = tree.find(".//cylinder")
            self.assertIsNotNone(cylinder)
            self.assertEqual(cylinder.get("length"), "2")
            self.assertEqual(cylinder.get("radius"), "2.5")
            self.assertEqual(manifest["remaining_meshes"], 0)


if __name__ == "__main__":
    unittest.main()
