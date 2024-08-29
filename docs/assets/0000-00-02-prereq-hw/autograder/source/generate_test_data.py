import random
import os
import importlib.util
import sys

def imp_load_source(module_name, module_path):
    modulespec = importlib.util.spec_from_file_location(module_name,
                                                        module_path)
    module = importlib.util.module_from_spec(modulespec)
    sys.modules[module_name] = module
    modulespec.loader.load_module()
    return module

def generate_test_factorial():
    inputs = list(map(str, [0, 3, 5] + [
        random.randint(0, 20) for _ in range(10)]))
    msgs = ["Incorrect factorial for inp={inp}".format(inp=inp)
            for inp in inputs]
    setting1 = dict(weight=1, visibility="visible")
    setting2 = dict(weight=1, visibility="visible")
    settings = [dict(msg=msg, **setting1) 
                for msg in msgs[:3]
                ] + [dict(msg=msg, **setting2)
                     for msg in msgs[3:]]
    test_generator.TestMetaclass.create_test_data_from_solutions(
            inputs=inputs,
            settings=settings,
            command = ["./test_factorial"],
            dir_name_format="factorial_{i:02d}_{inp:>02s}")

def generate_test_prime():
    inputs = list(map(str,
                      [11, 13, 17, 19, 23, 15877 ]
                      +
                      [
        random.randint(0, 99_999) for _ in range(7)]))
    msgs = ["Incorrect factorial for inp={inp}".format(inp=inp)
            for inp in inputs]
    setting2 = dict(weight=1, visibility="visible")
    settings = [dict(msg=msg, **setting2) for msg in msgs]
    test_generator.TestMetaclass.create_test_data_from_solutions(
            inputs=inputs,
            settings=settings,
            command = ["./test_prime"],
            dir_name_format="prime_{i:02d}_{inp:>05s}")


if __name__ == '__main__':
    random.seed(13)
    if os.path.exists("/autograder"):
        os.chdir("/autograder")
    ROOTDIR = os.getcwd()
    FILES = ["test_factorial.c", "test_prime.c"]
    global test_generator
    test_generator = imp_load_source("test_generator",
                                     f"{ROOTDIR}/source/test_generator.py")
    for file in FILES:
        test_generator.prepare_and_compile_submission(ROOTDIR, file)
    ## Run from inside source
    os.chdir(f"{ROOTDIR}/source")
    generate_test_factorial()
    generate_test_prime()

