from kinako_nav_bringup.scenario_loader import build_default_overrides


def test_build_default_overrides_uses_assets_layout():
    defaults = build_default_overrides('/tmp/kinako_nav_assets', 'tsukuba')

    assert defaults['map_hdf5_file'] == '/tmp/kinako_nav_assets/maps3d/tsukuba/localization.h5'
    assert defaults['vq_map_file'] == '/tmp/kinako_nav_assets/maps3d/tsukuba/visualization.h5'
    assert defaults['map_yaml_file'] == '/tmp/kinako_nav_assets/maps2d/tsukuba/navigation_map.yaml'
    assert defaults['waypoint_csv_file'] == '/tmp/kinako_nav_assets/waypoints/tsukuba/waypoints.csv'
    assert defaults['regions_config_file'] == '/tmp/kinako_nav_assets/regions/tsukuba/regions.yaml'
