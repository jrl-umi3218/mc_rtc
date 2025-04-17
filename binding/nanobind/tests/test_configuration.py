import numpy as np
import mc_rtc

# import pytest


def test_configuration_manual():
    c = mc_rtc.Configuration()
    c.add_null("null_key")
    assert c.has("null_key")

    arr = c.array("empty_array")
    assert c.has("empty_array")
    arr.push(3.5)
    assert arr.isArray()

    np_vec2 = np.array([1.0, 2.0])
    c.add("vec2", np_vec2)
    assert c.has("vec2")

    float_ref = 3.0
    c.add("float", float_ref)
    assert c.has("float")
    assert c["float"].isNumeric()
    assert c.get_double("float") == float_ref

    # get non-existing element with default_value
    assert not c.has("non_existing")
    assert c.get_double("non_existing", 42.0) == 42.0
    c2_ref = mc_rtc.Configuration()
    c.add("config", c2_ref)
    c["config"].add("float", 42.0)
    assert c.has("config")
    c2 = c.get_Configuration("config")
    assert c2.get_double("float") == 42.0

    true_bool = True
    c.add("bool", true_bool)
    assert c.get_bool("bool")
