# SPDX-FileCopyrightText: 2025, FANUC America Corporation
# SPDX-FileCopyrightText: 2025, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0

"""URDF validation tests for all FANUC robot description packages.

Tests validate:
1. Xacro -> URDF compilation succeeds
2. URDF XML structure is well-formed
3. Kinematic chain has expected links and joints
4. All mesh files referenced in URDF exist
5. Joint limits are physically valid (lower < upper)
6. Inertial parameters are valid where present
"""

import math
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

# Repository root
REPO_ROOT = Path(__file__).parent.parent

# All robot models organized by package
ROBOT_MODELS = {
    "fanuc_crx_description": [
        "crx5ia",
        "crx10ia",
        "crx10ia_l",
        "crx10ia_lp",
        "crx20ia_l",
        "crx30ia",
    ],
    "fanuc_lrmate_description": [
        "lrmate200id",
        "lrmate200id7l",
    ],
    "fanuc_m10_description": [
        "m10_12-14d",
    ],
    "fanuc_m20_description": [
        "m20_25-18d",
        "m20_35-18d",
    ],
    "fanuc_r1000ia_description": [
        "r1000ia_100f",
    ],
    "fanuc_r2000_description": [
        "r2000ic_125l",
    ],
}

# Expected structure for all 6-axis FANUC robots
EXPECTED_JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"]
EXPECTED_JOINT_TYPES = ["revolute"] * 6
EXPECTED_JOINT_AXES = [
    (0, 0, 1),    # J1: Z-axis
    (0, 1, 0),    # J2: Y-axis
    (0, -1, 0),   # J3: -Y-axis
    (-1, 0, 0),   # J4: -X-axis
    (0, -1, 0),   # J5: -Y-axis
    (-1, 0, 0),   # J6: -X-axis
]
EXPECTED_LINK_NAMES = [
    "base_link",
    "J1_link",
    "J2_link",
    "J3_link",
    "J4_link",
    "J5_link",
    "J6_link",
]
EXPECTED_AUX_LINKS = ["wbase", "flange"]
EXPECTED_AUX_JOINTS = ["base_link-wbase", "J6-flange", "base_joint", "child_joint"]


def _get_all_model_params():
    """Generate pytest parameters for all models."""
    params = []
    for package, models in ROBOT_MODELS.items():
        for model in models:
            params.append(
                pytest.param(package, model, id=f"{package}/{model}")
            )
    return params


def _xacro_to_urdf(package: str, model: str) -> tuple:
    """Process a robot xacro file and return (stdout, stderr, returncode).

    Creates a temporary wrapper xacro that uses filesystem paths instead of
    $(find ...) substitutions, enabling tests to run without a full ROS2
    workspace installation.
    """
    macro_file = REPO_ROOT / package / "urdf" / f"{model}_urdf_macro.xacro"
    assert macro_file.exists(), f"Macro file not found: {macro_file}"

    # Read the top-level xacro to extract the macro call pattern
    robot_file = REPO_ROOT / package / "robot" / f"{model}.urdf.xacro"
    assert robot_file.exists(), f"Robot file not found: {robot_file}"

    # Read original content and replace $(find ...) with absolute paths
    content = robot_file.read_text()
    content = content.replace(
        f"$(find {package})",
        str(REPO_ROOT / package),
    )

    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".urdf.xacro", delete=False
    ) as tmp:
        tmp.write(content)
        tmp_path = tmp.name

    try:
        result = subprocess.run(
            ["xacro", tmp_path],
            capture_output=True,
            text=True,
            timeout=30,
        )
        return result.stdout, result.stderr, result.returncode
    finally:
        os.unlink(tmp_path)


def _parse_urdf(urdf_xml: str) -> ET.Element:
    """Parse URDF XML string into ElementTree."""
    return ET.fromstring(urdf_xml)


def _parse_axis(axis_elem):
    """Parse axis xyz attribute into a tuple of floats."""
    if axis_elem is None:
        return (1, 0, 0)  # URDF default axis
    xyz = axis_elem.get("xyz", "1 0 0").split()
    return tuple(float(v) for v in xyz)


class TestXacroCompilation:
    """Test that all xacro files compile to valid URDF."""

    @pytest.mark.parametrize("package,model", _get_all_model_params())
    def test_xacro_file_exists(self, package, model):
        """Verify the xacro source file exists."""
        xacro_file = REPO_ROOT / package / "robot" / f"{model}.urdf.xacro"
        assert xacro_file.exists(), f"Missing xacro file: {xacro_file}"

    @pytest.mark.parametrize("package,model", _get_all_model_params())
    def test_macro_file_exists(self, package, model):
        """Verify the macro xacro file exists."""
        macro_file = REPO_ROOT / package / "urdf" / f"{model}_urdf_macro.xacro"
        assert macro_file.exists(), f"Missing macro file: {macro_file}"

    @pytest.mark.parametrize("package,model", _get_all_model_params())
    def test_xacro_compiles(self, package, model):
        """Verify xacro processing succeeds."""
        stdout, stderr, returncode = _xacro_to_urdf(package, model)
        assert returncode == 0, (
            f"Xacro compilation failed for {package}/{model}:\n{stderr}"
        )
        assert len(stdout) > 0, "Xacro produced empty output"

    @pytest.mark.parametrize("package,model", _get_all_model_params())
    def test_urdf_is_valid_xml(self, package, model):
        """Verify the output is valid XML."""
        stdout, stderr, returncode = _xacro_to_urdf(package, model)
        if returncode != 0:
            pytest.skip(f"Xacro compilation failed: {stderr}")
        try:
            ET.fromstring(stdout)
        except ET.ParseError as e:
            pytest.fail(f"Invalid XML from {package}/{model}: {e}")


class TestKinematicChain:
    """Test the kinematic chain structure."""

    @pytest.mark.parametrize("package,model", _get_all_model_params())
    def test_expected_links_exist(self, package, model):
        """Verify all expected links are present."""
        stdout, _, returncode = _xacro_to_urdf(package, model)
        if returncode != 0:
            pytest.skip("Xacro compilation failed")

        root = _parse_urdf(stdout)
        link_names = [link.get("name") for link in root.findall("link")]

        for expected in EXPECTED_LINK_NAMES:
            assert expected in link_names, (
                f"Missing link '{expected}' in {model}. Found: {link_names}"
            )

    @pytest.mark.parametrize("package,model", _get_all_model_params())
    def test_auxiliary_links_exist(self, package, model):
        """Verify auxiliary frames (wbase, flange) are present."""
        stdout, _, returncode = _xacro_to_urdf(package, model)
        if returncode != 0:
            pytest.skip("Xacro compilation failed")

        root = _parse_urdf(stdout)
        link_names = [link.get("name") for link in root.findall("link")]

        for expected in EXPECTED_AUX_LINKS:
            assert expected in link_names, (
                f"Missing auxiliary link '{expected}' in {model}"
            )

    @pytest.mark.parametrize("package,model", _get_all_model_params())
    def test_world_and_ee_links_exist(self, package, model):
        """Verify world and ee_link are present (from top-level xacro)."""
        stdout, _, returncode = _xacro_to_urdf(package, model)
        if returncode != 0:
            pytest.skip("Xacro compilation failed")

        root = _parse_urdf(stdout)
        link_names = [link.get("name") for link in root.findall("link")]

        assert "world" in link_names, f"Missing 'world' link in {model}"
        assert "ee_link" in link_names, f"Missing 'ee_link' link in {model}"

    @pytest.mark.parametrize("package,model", _get_all_model_params())
    def test_six_revolute_joints(self, package, model):
        """Verify exactly 6 revolute joints J1-J6 exist."""
        stdout, _, returncode = _xacro_to_urdf(package, model)
        if returncode != 0:
            pytest.skip("Xacro compilation failed")

        root = _parse_urdf(stdout)
        joints = {j.get("name"): j for j in root.findall("joint")}

        for name in EXPECTED_JOINT_NAMES:
            assert name in joints, f"Missing joint '{name}' in {model}"
            assert joints[name].get("type") == "revolute", (
                f"Joint '{name}' should be revolute, got '{joints[name].get('type')}'"
            )

    @pytest.mark.parametrize("package,model", _get_all_model_params())
    def test_joint_axes(self, package, model):
        """Verify joint rotation axes match expected FANUC configuration."""
        stdout, _, returncode = _xacro_to_urdf(package, model)
        if returncode != 0:
            pytest.skip("Xacro compilation failed")

        root = _parse_urdf(stdout)
        joints = {j.get("name"): j for j in root.findall("joint")}

        for name, expected_axis in zip(EXPECTED_JOINT_NAMES, EXPECTED_JOINT_AXES):
            joint = joints[name]
            axis = _parse_axis(joint.find("axis"))
            assert axis == expected_axis, (
                f"Joint '{name}' axis mismatch in {model}: "
                f"expected {expected_axis}, got {axis}"
            )

    @pytest.mark.parametrize("package,model", _get_all_model_params())
    def test_kinematic_chain_connectivity(self, package, model):
        """Verify the kinematic chain is properly connected base_link -> J1 -> ... -> J6."""
        stdout, _, returncode = _xacro_to_urdf(package, model)
        if returncode != 0:
            pytest.skip("Xacro compilation failed")

        root = _parse_urdf(stdout)
        joints = {j.get("name"): j for j in root.findall("joint")}

        expected_chain = [
            ("J1", "base_link", "J1_link"),
            ("J2", "J1_link", "J2_link"),
            ("J3", "J2_link", "J3_link"),
            ("J4", "J3_link", "J4_link"),
            ("J5", "J4_link", "J5_link"),
            ("J6", "J5_link", "J6_link"),
        ]

        for joint_name, expected_parent, expected_child in expected_chain:
            joint = joints[joint_name]
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            assert parent == expected_parent, (
                f"Joint '{joint_name}' parent mismatch: "
                f"expected '{expected_parent}', got '{parent}'"
            )
            assert child == expected_child, (
                f"Joint '{joint_name}' child mismatch: "
                f"expected '{expected_child}', got '{child}'"
            )

    @pytest.mark.parametrize("package,model", _get_all_model_params())
    def test_auxiliary_joint_connectivity(self, package, model):
        """Verify auxiliary frame connections."""
        stdout, _, returncode = _xacro_to_urdf(package, model)
        if returncode != 0:
            pytest.skip("Xacro compilation failed")

        root = _parse_urdf(stdout)
        joints = {j.get("name"): j for j in root.findall("joint")}

        # wbase: base_link -> wbase (fixed)
        assert "base_link-wbase" in joints
        wbase_joint = joints["base_link-wbase"]
        assert wbase_joint.get("type") == "fixed"
        assert wbase_joint.find("parent").get("link") == "base_link"
        assert wbase_joint.find("child").get("link") == "wbase"

        # flange: J6_link -> flange (fixed)
        assert "J6-flange" in joints
        flange_joint = joints["J6-flange"]
        assert flange_joint.get("type") == "fixed"
        assert flange_joint.find("parent").get("link") == "J6_link"
        assert flange_joint.find("child").get("link") == "flange"

        # child_joint: flange -> ee_link (fixed)
        assert "child_joint" in joints
        child_joint = joints["child_joint"]
        assert child_joint.get("type") == "fixed"
        assert child_joint.find("parent").get("link") == "flange"
        assert child_joint.find("child").get("link") == "ee_link"


class TestMeshReferences:
    """Test that all mesh file references are valid."""

    @pytest.mark.parametrize("package,model", _get_all_model_params())
    def test_visual_meshes_exist(self, package, model):
        """Verify all visual mesh files referenced in URDF exist."""
        stdout, _, returncode = _xacro_to_urdf(package, model)
        if returncode != 0:
            pytest.skip("Xacro compilation failed")

        root = _parse_urdf(stdout)
        missing = []

        for visual in root.iter("visual"):
            mesh = visual.find(".//mesh")
            if mesh is not None:
                filename = mesh.get("filename", "")
                if filename.startswith("package://"):
                    # Convert package:// URI to filesystem path
                    rel_path = filename.replace("package://", "")
                    abs_path = REPO_ROOT / rel_path
                    if not abs_path.exists():
                        missing.append(str(abs_path))

        assert len(missing) == 0, (
            f"Missing visual meshes for {model}:\n" + "\n".join(missing)
        )

    @pytest.mark.parametrize("package,model", _get_all_model_params())
    def test_collision_meshes_exist(self, package, model):
        """Verify all collision mesh files referenced in URDF exist."""
        stdout, _, returncode = _xacro_to_urdf(package, model)
        if returncode != 0:
            pytest.skip("Xacro compilation failed")

        root = _parse_urdf(stdout)
        missing = []

        for collision in root.iter("collision"):
            mesh = collision.find(".//mesh")
            if mesh is not None:
                filename = mesh.get("filename", "")
                if filename.startswith("package://"):
                    rel_path = filename.replace("package://", "")
                    abs_path = REPO_ROOT / rel_path
                    if not abs_path.exists():
                        missing.append(str(abs_path))

        assert len(missing) == 0, (
            f"Missing collision meshes for {model}:\n" + "\n".join(missing)
        )

    @pytest.mark.parametrize("package,model", _get_all_model_params())
    def test_mesh_count(self, package, model):
        """Verify each model has 7 visual and 7 collision meshes (base + J1-J6)."""
        stdout, _, returncode = _xacro_to_urdf(package, model)
        if returncode != 0:
            pytest.skip("Xacro compilation failed")

        root = _parse_urdf(stdout)

        visual_meshes = [
            v.find(".//mesh") for v in root.iter("visual")
            if v.find(".//mesh") is not None
        ]
        collision_meshes = [
            c.find(".//mesh") for c in root.iter("collision")
            if c.find(".//mesh") is not None
        ]

        assert len(visual_meshes) == 7, (
            f"Expected 7 visual meshes for {model}, got {len(visual_meshes)}"
        )
        assert len(collision_meshes) == 7, (
            f"Expected 7 collision meshes for {model}, got {len(collision_meshes)}"
        )


class TestJointLimits:
    """Test joint limit validity."""

    @pytest.mark.parametrize("package,model", _get_all_model_params())
    def test_lower_less_than_upper(self, package, model):
        """Verify lower limit < upper limit for all joints."""
        stdout, _, returncode = _xacro_to_urdf(package, model)
        if returncode != 0:
            pytest.skip("Xacro compilation failed")

        root = _parse_urdf(stdout)
        joints = {j.get("name"): j for j in root.findall("joint")}

        for name in EXPECTED_JOINT_NAMES:
            joint = joints[name]
            limit = joint.find("limit")
            assert limit is not None, f"No limit element for joint '{name}'"

            lower = float(limit.get("lower"))
            upper = float(limit.get("upper"))
            assert lower < upper, (
                f"Joint '{name}' in {model}: lower ({lower}) >= upper ({upper})"
            )

    @pytest.mark.parametrize("package,model", _get_all_model_params())
    def test_non_negative_effort_limits(self, package, model):
        """Verify effort limits are non-negative.

        Note: Some models use 0.0 as a placeholder when effort limits are
        not specified. This is valid URDF but may indicate incomplete data.
        """
        stdout, _, returncode = _xacro_to_urdf(package, model)
        if returncode != 0:
            pytest.skip("Xacro compilation failed")

        root = _parse_urdf(stdout)
        joints = {j.get("name"): j for j in root.findall("joint")}

        for name in EXPECTED_JOINT_NAMES:
            limit = joints[name].find("limit")
            effort = float(limit.get("effort"))
            assert effort >= 0, (
                f"Joint '{name}' in {model}: effort limit ({effort}) must be non-negative"
            )

    @pytest.mark.parametrize("package,model", _get_all_model_params())
    def test_positive_velocity_limits(self, package, model):
        """Verify velocity limits are positive."""
        stdout, _, returncode = _xacro_to_urdf(package, model)
        if returncode != 0:
            pytest.skip("Xacro compilation failed")

        root = _parse_urdf(stdout)
        joints = {j.get("name"): j for j in root.findall("joint")}

        for name in EXPECTED_JOINT_NAMES:
            limit = joints[name].find("limit")
            velocity = float(limit.get("velocity"))
            assert velocity > 0, (
                f"Joint '{name}' in {model}: velocity limit ({velocity}) must be positive"
            )

    @pytest.mark.parametrize("package,model", _get_all_model_params())
    def test_joint_range_reasonable(self, package, model):
        """Verify joint ranges are within physically reasonable bounds.

        Industrial robots can have wrist joints (J4, J5, J6) with ranges
        exceeding 720 deg (e.g., M-20 J6 has +/-450 deg = 900 deg range).
        We use 1080 deg (3 full rotations) as the upper sanity bound.
        """
        stdout, _, returncode = _xacro_to_urdf(package, model)
        if returncode != 0:
            pytest.skip("Xacro compilation failed")

        root = _parse_urdf(stdout)
        joints = {j.get("name"): j for j in root.findall("joint")}

        max_range_rad = math.radians(1080)  # 3 full rotations

        for name in EXPECTED_JOINT_NAMES:
            limit = joints[name].find("limit")
            lower = float(limit.get("lower"))
            upper = float(limit.get("upper"))
            joint_range = upper - lower

            assert joint_range <= max_range_rad, (
                f"Joint '{name}' in {model}: range ({math.degrees(joint_range):.1f} deg) "
                f"exceeds maximum (1080 deg)"
            )


class TestInertialParameters:
    """Test inertial parameter validity where present."""

    @pytest.mark.parametrize("package,model", _get_all_model_params())
    def test_positive_mass(self, package, model):
        """Verify all link masses are non-negative where inertial is defined."""
        stdout, _, returncode = _xacro_to_urdf(package, model)
        if returncode != 0:
            pytest.skip("Xacro compilation failed")

        root = _parse_urdf(stdout)

        for link in root.findall("link"):
            inertial = link.find("inertial")
            if inertial is not None:
                mass_elem = inertial.find("mass")
                if mass_elem is not None:
                    mass = float(mass_elem.get("value"))
                    assert mass >= 0, (
                        f"Link '{link.get('name')}' in {model}: "
                        f"negative mass ({mass})"
                    )

    @pytest.mark.parametrize("package,model", _get_all_model_params())
    def test_inertia_diagonal_non_negative(self, package, model):
        """Verify inertia tensor diagonal elements are non-negative."""
        stdout, _, returncode = _xacro_to_urdf(package, model)
        if returncode != 0:
            pytest.skip("Xacro compilation failed")

        root = _parse_urdf(stdout)

        for link in root.findall("link"):
            inertial = link.find("inertial")
            if inertial is not None:
                inertia = inertial.find("inertia")
                if inertia is not None:
                    ixx = float(inertia.get("ixx", "0"))
                    iyy = float(inertia.get("iyy", "0"))
                    izz = float(inertia.get("izz", "0"))

                    link_name = link.get("name")
                    assert ixx >= 0, (
                        f"Link '{link_name}' in {model}: negative Ixx ({ixx})"
                    )
                    assert iyy >= 0, (
                        f"Link '{link_name}' in {model}: negative Iyy ({iyy})"
                    )
                    assert izz >= 0, (
                        f"Link '{link_name}' in {model}: negative Izz ({izz})"
                    )


class TestPackageStructure:
    """Test package-level structural requirements."""

    @pytest.mark.parametrize("package", ROBOT_MODELS.keys())
    def test_package_xml_exists(self, package):
        """Verify package.xml exists."""
        assert (REPO_ROOT / package / "package.xml").exists()

    @pytest.mark.parametrize("package", ROBOT_MODELS.keys())
    def test_cmakelists_exists(self, package):
        """Verify CMakeLists.txt exists."""
        assert (REPO_ROOT / package / "CMakeLists.txt").exists()

    @pytest.mark.parametrize("package", ROBOT_MODELS.keys())
    def test_launch_dir_exists(self, package):
        """Verify launch directory exists."""
        assert (REPO_ROOT / package / "launch").is_dir()

    @pytest.mark.parametrize("package", ROBOT_MODELS.keys())
    def test_rviz_dir_exists(self, package):
        """Verify rviz directory exists."""
        assert (REPO_ROOT / package / "rviz").is_dir()

    @pytest.mark.parametrize("package", ROBOT_MODELS.keys())
    def test_meshes_dir_exists(self, package):
        """Verify meshes directory exists."""
        assert (REPO_ROOT / package / "meshes").is_dir()

    @pytest.mark.parametrize("package,model", _get_all_model_params())
    def test_mesh_subdirs_exist(self, package, model):
        """Verify visual and collision mesh directories exist for each model."""
        visual_dir = REPO_ROOT / package / "meshes" / model / "visual"
        collision_dir = REPO_ROOT / package / "meshes" / model / "collision"

        assert visual_dir.is_dir(), f"Missing visual mesh dir: {visual_dir}"
        assert collision_dir.is_dir(), f"Missing collision mesh dir: {collision_dir}"

    @pytest.mark.parametrize("package", ROBOT_MODELS.keys())
    def test_package_xml_format(self, package):
        """Verify package.xml is format 3."""
        pkg_xml = REPO_ROOT / package / "package.xml"
        tree = ET.parse(pkg_xml)
        root = tree.getroot()
        assert root.get("format") == "3", (
            f"{package}/package.xml is not format 3"
        )

    @pytest.mark.parametrize("package", ROBOT_MODELS.keys())
    def test_package_xml_has_required_deps(self, package):
        """Verify package.xml has required exec dependencies."""
        required_deps = {
            "joint_state_publisher_gui",
            "launch",
            "launch_ros",
            "robot_state_publisher",
            "rviz2",
            "urdf",
            "xacro",
        }

        pkg_xml = REPO_ROOT / package / "package.xml"
        tree = ET.parse(pkg_xml)
        root = tree.getroot()

        exec_deps = {dep.text for dep in root.findall("exec_depend")}
        missing = required_deps - exec_deps
        assert len(missing) == 0, (
            f"{package} missing exec dependencies: {missing}"
        )
