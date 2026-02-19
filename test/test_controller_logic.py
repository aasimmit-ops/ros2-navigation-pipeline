def compute_curvature(y_r, lookahead_distance):
    if lookahead_distance <= 0:
        return 0.0
    return (2 * y_r) / (lookahead_distance ** 2)


def test_curvature_positive():
    curvature = compute_curvature(0.5, 1.0)
    assert curvature > 0


def test_curvature_zero():
    curvature = compute_curvature(0.0, 1.0)
    assert curvature == 0.0


def test_invalid_lookahead():
    curvature = compute_curvature(1.0, 0.0)
    assert curvature == 0.0
