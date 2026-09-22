"""kill_engaged must read the kill channel exactly as PX4's rc_update does (threshold sign = polarity)."""
from airframe_designer.px4.killtest import kill_engaged


class FakeLink:
    def __init__(self, **params):
        self.params = {k: {"value": v} for k, v in params.items()}


def test_positive_threshold_kills_above():
    link = FakeLink(RC_MAP_KILL_SW=5, RC_KILLSWITCH_TH=0.5, RC5_MIN=1011, RC5_TRIM=1499, RC5_MAX=1988, RC5_REV=1)
    assert kill_engaged(link, 1988) is True
    assert kill_engaged(link, 1011) is False


def test_negative_threshold_kills_below():
    link = FakeLink(RC_MAP_KILL_SW=9, RC_KILLSWITCH_TH=-0.5)
    assert kill_engaged(link, 1011) is True
    assert kill_engaged(link, 1988) is False


def test_reversed_channel_and_unmapped():
    link = FakeLink(RC_MAP_KILL_SW=6, RC_KILLSWITCH_TH=0.5, RC6_REV=-1)
    assert kill_engaged(link, 1011) is True                  # reversed: the low end reads high
    assert kill_engaged(FakeLink(RC_MAP_KILL_SW=0), 1988) is None
    assert kill_engaged(link, 0) is None                      # no signal
