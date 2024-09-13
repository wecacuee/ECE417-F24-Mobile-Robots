from otter.test_files import test_case

OK_FORMAT = False

name = "q6"
points = 2

@test_case(points=None, hidden=False)
import random

def test_length_of_colors(length_of_colors):
    n = random.randint(1, 100)
    colors = ['red'] * n
    assert n == len(colors)
