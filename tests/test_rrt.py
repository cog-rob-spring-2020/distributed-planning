from solutions.rrt import distance


def test_generated_paths():
    # create a loop with like 10 trials
    # use a known feasible environment
    # run RRT
    # run check_path to see if it worked

    epsilon = 0.001
    test_cases = [{"p1": (3, 3), "p2": (4, 4), "expected": 1.414,}]

    for tc in test_cases:
        found = distance(tc["p1"], tc["p2"])
        assert abs(found - tc["expected"]) <= epsilon
