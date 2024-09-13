from otter.test_files import test_case

OK_FORMAT = False

name = "q8"
points = 2

@test_case(points=None, hidden=False)
def test_ele_7th_to_3rd_list(ele_7th_to_3rd_list):
    A = list(range(1,10,1)) # start,stop,step
    assert ele_7th_to_3rd_list(A) == A[slice(6, 1, -2)]
