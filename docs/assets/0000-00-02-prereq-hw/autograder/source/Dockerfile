FROM ubuntu:22.04
RUN mkdir -p /autograder/source
WORKDIR /autograder/source
COPY requirements.txt \
    run_autograder \
    run.sh \
    run_tests.py \
    setup.sh \
    test_data/ \
    test_generator.py .
WORKDIR /autograder
COPY run_autograder .
RUN bash setup.sh
RUN mkdir -p /autograder/results
RUN run_autograder
