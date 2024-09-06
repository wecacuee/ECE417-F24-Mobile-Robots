from otter.test_files import test_case

OK_FORMAT = False

name = "q1"
points = 2

@test_case(points=None, hidden=False)
def test_ls_output(lsoutput):
    expected_output = 'bye.txt  alive.txt  hi.txt'
    assert lsoutput.stdout.strip() == expected_output.strip(), lsoutput.stdout
