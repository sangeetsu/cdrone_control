from drone_light_pkg.light_utils import duty_from_intensity


def test_duty_from_intensity_clamped():
    assert duty_from_intensity(-1.0) == 0.0
    assert duty_from_intensity(0.5) == 50.0
    assert duty_from_intensity(2.0) == 100.0
