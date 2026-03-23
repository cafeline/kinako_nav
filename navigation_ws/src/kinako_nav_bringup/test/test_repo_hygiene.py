from pathlib import Path


def test_no_parent_share_install_paths_in_cmake():
    workspace = Path(__file__).resolve().parents[3]
    targets = [
        workspace / 'src' / 'emcl2_ros2' / 'CMakeLists.txt',
        workspace / 'src' / 'pointcloud2_cutter' / 'CMakeLists.txt',
        workspace / 'src' / 'vq_server' / 'CMakeLists.txt',
        workspace / 'src' / 'waypoint_follower' / 'CMakeLists.txt',
    ]

    for cmake_file in targets:
        text = cmake_file.read_text(encoding='utf-8')
        assert '../share/' not in text, f'Found legacy install path in {cmake_file}'
