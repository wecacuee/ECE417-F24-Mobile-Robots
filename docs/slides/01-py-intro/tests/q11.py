from otter.test_files import test_case

OK_FORMAT = False

name = "q11"
points = 20

@test_case(points=None, hidden=False)
def test_sqrt(sqrt, x=2):
    z = sqrt(x)
    assert abs(z*z - x) < 1e-8, f'z*z={z*z} not equals x={x}'
def test_sqrt_3(sqrt, x=3):
    test_sqrt(sqrt, x=x)
def test_sqrt_4(sqrt, x=4):
    test_sqrt(sqrt, x=x)
def test_sqrt_rnd(sqrt):
    import random
    for _ in range(10):
        x = random.randint(1, 100)
        test_sqrt(sqrt, x=x)
