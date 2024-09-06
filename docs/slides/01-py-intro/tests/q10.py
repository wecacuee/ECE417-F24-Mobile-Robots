from otter.test_files import test_case

OK_FORMAT = False

name = "q10"
points = 2

@test_case(points=None, hidden=False)
def test_first_last(first_last):
    import random
    a = list(range(random.randint(1, 100)))
    fst_last = first_last(a)
    assert fst_last[0] == a[0]
    assert fst_last[-1] == a[-1]
