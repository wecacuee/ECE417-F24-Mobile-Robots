import random
import unittest
import os
import os.path
import shutil
import subprocess32 as subprocess
from subprocess32 import PIPE
import time
from gradescope_utils.autograder_utils.decorators import weight, visibility
import yaml

BASE_DIR = './test_data'

def check_correct_filenames(root_dir, expecting_files):
    tests = []
    for file in expecting_files:
        submitted_files = [os.path.basename(f)
                           for f in os.listdir("submission")]
        if file in submitted_files:
            tests.append(
                dict(name=f"Found submitted file {file}",
                     status="success",
                     score=2,
                     max_score=2,
                     visibility="visible"))
        else:
            tests.append(
                dict(name=f"Not submitted file {file}",
                     status="failed",
                     score=0,
                     max_score=2,
                     visibility="visible",
                     output=f"""You submitted {submitted_files}; none of """
                     f"""which is {file}. Cannot run further tests."""))

    return tests

def prepare_and_compile_submission(root_dir, file):
    # Prepare submission
    tests = check_correct_filenames(root_dir, [file])
    if tests[0]['status'] != "success":
        os.chdir(f"{root_dir}")
        return tests
    shutil.copy2(f"submission/{file}",  f"source/{file}")

    ## Compile
    os.chdir(f"{root_dir}/source")
    executable, _ = os.path.splitext(file)
    process = subprocess.run(["gcc", file,  "-o",  executable], capture_output=True)
    if process.returncode != 0:
        tests.append(
            dict(name=f"Failed to compile {file}",
                 status="failed",
                 score=0,
                 max_score=5,
                 visibility="visible",
                 output=f"""Output: {process.stdout}\n\n\n"""
                 f"""Errors: {process.stderr}"""))
    else:
        tests.append(
            dict(name=f"Succefully compiled {file}",
                 status="success",
                 score=5,
                 max_score=5,
                 visibility="visible",
                 output=f"""Output: {process.stdout}\n\n\n"""
                 f"""Errors: {process.stderr}"""))

    os.chdir(f"{root_dir}")
    return tests


def load_test_file(dir_name, path):
    full_path = os.path.join(BASE_DIR, dir_name, path)
    if os.path.isfile(full_path):
        with open(full_path, 'rb') as f:
            return f.read()
    return None

def load_settings(dir_name):
    settings_yml = load_test_file(dir_name, 'settings.yml')

    if settings_yml is not None:
        return yaml.safe_load(settings_yml) or {}
    else:
        return {}

def dump_settings(dir_name, settings):
    full_path = os.path.join(BASE_DIR, dir_name, 'settings.yml')
    with open(full_path, "w") as f:
        f.write(yaml.dump(settings))


class TestMetaclass(type):
    """
    Metaclass that allows generating tests based on a directory.
    """
    def __new__(cls, name, bases, attrs):
        data_dir = attrs['data_dir']
        attrs[cls.test_name(data_dir)] = cls.generate_test(data_dir)
        return super(TestMetaclass, cls).__new__(cls, name, bases, attrs)

    @classmethod
    def create_test_data_from_solutions(cls, inputs=None,
                                        write_expected_err=False,
                                        settings=None,
                                        dir_name_format="{i:03d}_{inp:>05s}",
                                        command=["../run.sh"],
                                        ):

        if inputs is None:
            test_inputs = [load_test_file(dir_name, 'input')
                      for dir_name in find_data_directories()]
        else:
            test_inputs = inputs
        if settings is None:
            loaded_settings = [load_settings('')]*len(test_inputs)
        else:
            loaded_settings = settings

        for i, (inp, sett) in enumerate(zip(test_inputs, loaded_settings)):
            dir_name =  dir_name_format.format(i=i, inp=inp)
            dir_path = os.path.join(BASE_DIR, dir_name)
            os.makedirs(dir_path, exist_ok=True)
            fpath_test_specific_command = os.path.join(BASE_DIR, dir_name, 'run.yaml')
            with open(fpath_test_specific_command, "w") as f:
                f.write(yaml.dump(command))
            proc = subprocess.Popen(command, stdin=PIPE, stdout=PIPE, stderr=PIPE)
            if inputs is not None:
                fpath_input = os.path.join(BASE_DIR, dir_name, 'input')
                with open(fpath_input, 'w') as f:
                    f.write(inp)
            dump_settings(dir_name, sett)

            output, err = proc.communicate(bytes(inp, 'utf-8'), sett.get('timeout', 1))
            print(f"Creating test case {i}, f({inp}) == {output.decode('utf-8')}")

            fpath_expected_output = os.path.join(BASE_DIR, dir_name, 'output')
            fpath_expected_err = os.path.join(BASE_DIR, dir_name, 'err')
            with open(fpath_expected_output, 'wb') as f:
                f.write(output)
            if write_expected_err:
                with open(fpath_expected_err, 'wb') as f:
                    f.write(err)



    @classmethod
    def generate_test(cls, dir_name):
        """ Returns a testcase for the given directory """
        command = cls.generate_command(dir_name)

        settings = load_settings(dir_name)

        @weight(settings.get('weight', 1))
        @visibility(settings.get('visibility', 'visible'))
        def fn(self):
            proc = subprocess.Popen(command, stdin=PIPE, stdout=PIPE, stderr=PIPE)
            stdin = load_test_file(dir_name, 'input')

            startTime = time.time()
            output, err = proc.communicate(stdin, settings.get('timeout', 1))
            stopTime = time.time()
            timeTaken = stopTime - startTime

            expected_output = load_test_file(dir_name, 'output')
            expected_err = load_test_file(dir_name, 'err')

            msg = settings.get('msg', "Output did not match expected")
            self.assertEqual(expected_output, output, msg=msg)
            if expected_err is not None:
                self.assertEqual(expected_err, err, msg=msg)
        fn.__doc__ = settings.get('doc', 'Test {0}'.format(dir_name))
        fn.__number__ = settings.get('number', None)
        fn.__hide_errors__ = settings.get('hide_errors', None)
        fn.__leaderboard_column__ = settings.get('leaderboard_column',
                                                 None)
        fn.__leaderboard_sort_order__ = settings.get('leaderboard_sort_order',
                                                     None)
        fn.__leaderboard_value__ = settings.get('leaderboard_value', None)
        return fn

    @staticmethod
    def generate_command(dir_name):
        """Generates the command passed to Popen"""
        test_specific_script = os.path.join(BASE_DIR, dir_name, 'run.yaml')
        if os.path.isfile(test_specific_script):
            with open(test_specific_script, "r") as f:
                return yaml.safe_load(f)
        return ["bash", "./run.sh"]

    @staticmethod
    def klass_name(dir_name):
        return 'Test{0}'.format(''.join([x.capitalize() for x in dir_name.split('_')]))

    @staticmethod
    def test_name(dir_name):
        return 'test_{0}'.format(dir_name)


def build_test_class(data_dir):
    klass = TestMetaclass(
        TestMetaclass.klass_name(data_dir),
        (unittest.TestCase,),
        {
            'data_dir': data_dir
        }
    )
    return klass


def find_data_directories():
    return filter(
        lambda x: os.path.isdir(os.path.join(BASE_DIR, x)),
        os.listdir(BASE_DIR)
    )
