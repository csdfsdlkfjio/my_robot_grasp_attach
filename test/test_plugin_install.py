"""The Gazebo plugin must be built and installed where gzserver loads it."""

import os
import subprocess

from ament_index_python.packages import get_package_prefix


def plugin_library():
    """Path of the installed plugin shared library."""
    prefix = get_package_prefix('my_robot_grasp_attach')
    return os.path.join(prefix, 'lib', 'libgazebo_grasp_attach.so')


def test_plugin_library_is_installed():
    """gzserver finds plugins by file name on GAZEBO_PLUGIN_PATH."""
    assert os.path.isfile(plugin_library())


def test_plugin_library_contains_the_model_plugin_class():
    """The class has to be compiled in, not just an empty library."""
    result = subprocess.run(
        ['nm', '-D', '--defined-only', plugin_library()],
        capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert 'GraspAttach' in result.stdout


def test_plugin_library_links_gazebo_and_ros():
    """An unresolved dependency would only show up inside gzserver."""
    result = subprocess.run(
        ['ldd', plugin_library()], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert 'libgazebo_physics.so' in result.stdout
    assert 'libgazebo_ros_node.so' in result.stdout
