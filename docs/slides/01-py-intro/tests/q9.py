from otter.test_files import test_case

OK_FORMAT = False

name = "q9"
points = 20

@test_case(points=None, hidden=False)
def test_FIB20(FIB20):
    assert len(FIB20) == 20
    assert FIB20[0] == 0
    assert FIB20[1] == 1
