from test_generator import find_data_directories, build_test_class, TestMetaclass
import random

if __name__ == '__main__':
    TestMetaclass.create_test_data_from_solutions(
        inputs = map(str, [0, 3, 5] + [
            random.randint(0, 20) for _ in range(10)]))
