"""
Unit tests for parse_sim_config() in simulator_node.

Run:  pytest src/amr_2dsim/amr_2dsim/test_parse_sim_config.py -v
"""
import textwrap
import pytest
from amr_2dsim.simulator_node import parse_sim_config, _DEFAULTS


def _write_urdf(tmp_path, body: str) -> str:
    p = tmp_path / "test.urdf"
    p.write_text(textwrap.dedent(f'<?xml version="1.0"?><robot name="t">{body}</robot>'))
    return str(p)


def test_full_config(tmp_path):
    """All fields present → values from file, not defaults."""
    urdf = _write_urdf(tmp_path, """
        <amr_sim_config>
          <kinematic_model>diff_drive</kinematic_model>
          <wheel_base>0.325</wheel_base>
          <robot_radius>0.212</robot_radius>
          <laser_range_max>8.0</laser_range_max>
          <ticks_per_meter>500.0</ticks_per_meter>
        </amr_sim_config>
    """)
    cfg = parse_sim_config(urdf)
    assert cfg["kinematic_model"] == "diff_drive"
    assert cfg["wheel_base"] == pytest.approx(0.325)
    assert cfg["robot_radius"] == pytest.approx(0.212)
    assert cfg["laser_range_max"] == pytest.approx(8.0)
    assert cfg["ticks_per_meter"] == pytest.approx(500.0)


def test_no_config_tag(tmp_path):
    """No <amr_sim_config> → all defaults, no crash."""
    urdf = _write_urdf(tmp_path, '<link name="base_link"/>')
    cfg = parse_sim_config(urdf)
    assert cfg == _DEFAULTS


def test_partial_config(tmp_path):
    """Only wheel_base present → that field updated, rest stay default."""
    urdf = _write_urdf(tmp_path, """
        <amr_sim_config>
          <wheel_base>0.325</wheel_base>
        </amr_sim_config>
    """)
    cfg = parse_sim_config(urdf)
    assert cfg["wheel_base"] == pytest.approx(0.325)
    assert cfg["robot_radius"] == pytest.approx(_DEFAULTS["robot_radius"])
    assert cfg["kinematic_model"] == _DEFAULTS["kinematic_model"]


def test_bad_float_field(tmp_path):
    """Bad float value in one field → that field falls back to default, no crash."""
    urdf = _write_urdf(tmp_path, """
        <amr_sim_config>
          <wheel_base>NOT_A_NUMBER</wheel_base>
          <robot_radius>0.212</robot_radius>
        </amr_sim_config>
    """)
    cfg = parse_sim_config(urdf)
    assert cfg["wheel_base"] == pytest.approx(_DEFAULTS["wheel_base"])  # fallback
    assert cfg["robot_radius"] == pytest.approx(0.212)                  # still parsed


def test_missing_file():
    """Non-existent file → defaults, no crash."""
    cfg = parse_sim_config("/tmp/nonexistent_robot.urdf")
    assert cfg == _DEFAULTS
