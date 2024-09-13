from otter.test_files import test_case

OK_FORMAT = False

name = "q4"
points = 10

@test_case(points=None, hidden=False)
def test_floating_point(your_answer):
    assert abs(0.1+0.2-0.3) <= your_answer

