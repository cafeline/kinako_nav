from kinako_nav_bringup.scenario_loader import merge_scenario_overrides


def test_merge_scenario_overrides_updates_nested_values():
    base = {
        'emcl2': {'ros__parameters': {'map_hdf5_path': 'old.h5'}},
        'waypoint_follower_node': {'ros__parameters': {'waypoint_csv_path': 'old.csv'}},
    }
    overrides = {
        'map_hdf5_file': '/maps/new.h5',
        'waypoint_csv_file': '/waypoints/new.csv',
    }

    merged = merge_scenario_overrides(base, overrides)

    assert merged['emcl2']['ros__parameters']['map_hdf5_path'] == '/maps/new.h5'
    assert merged['waypoint_follower_node']['ros__parameters']['waypoint_csv_path'] == '/waypoints/new.csv'
