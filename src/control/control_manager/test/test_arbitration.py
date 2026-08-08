import pytest

from control_manager.arbitration import Source, select_source


@pytest.mark.parametrize(
    ("kwargs", "expected"),
    [
        ({"emergency_stop": True, "effective_source": Source.AUTO, "auto_permitted": True,
          "manual_command_fresh": True, "nav_command_fresh": True}, Source.ZERO),
        ({"emergency_stop": False, "effective_source": Source.MANUAL, "auto_permitted": False,
          "manual_command_fresh": True, "nav_command_fresh": False}, Source.MANUAL),
        ({"emergency_stop": False, "effective_source": Source.AUTO, "auto_permitted": True,
          "manual_command_fresh": False, "nav_command_fresh": True}, Source.AUTO),
        ({"emergency_stop": False, "effective_source": Source.AUTO, "auto_permitted": True,
          "manual_command_fresh": False, "nav_command_fresh": False}, Source.ZERO),
        ({"emergency_stop": False, "effective_source": Source.AUTO, "auto_permitted": False,
          "manual_command_fresh": False, "nav_command_fresh": True}, Source.ZERO),
    ],
)
def test_only_authorized_fresh_source_is_forwarded(kwargs, expected):
    assert select_source(**kwargs) == expected
