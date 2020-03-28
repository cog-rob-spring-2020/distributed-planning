from solutions.rrt import distance

def test_distance():
    epsilon = 0.001
    test_cases = [
        {
            "p1": (3, 3),
            "p2": (4, 4),
            "expected": 1.414,
        }
    ]

    for tc in test_cases:
        found = distance(tc["p1"], tc["p2"])
        assert abs(found - tc["expected"]) <= epsilon
