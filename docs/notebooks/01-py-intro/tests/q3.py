from otter.test_files import test_case

OK_FORMAT = False

name = "q3"
points = 2

@test_case(points=None, hidden=False)
def test_q3output(q3output):
    expected_q3output = """\
0.1 = 0.10000
0.1 = 0.10000000000000000555
"""
    assert q3output.stdout == expected_q3output
