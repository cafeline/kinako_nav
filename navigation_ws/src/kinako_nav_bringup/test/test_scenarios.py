from kinako_nav_bringup.scenario_loader import list_available_scenarios


def test_list_available_scenarios_returns_core_scenarios():
    scenarios = list_available_scenarios('/tmp/kinako_nav_bringup/config/scenarios')
    assert {'tsukuba', 'tsudanuma', '19f'}.issubset(set(scenarios))
