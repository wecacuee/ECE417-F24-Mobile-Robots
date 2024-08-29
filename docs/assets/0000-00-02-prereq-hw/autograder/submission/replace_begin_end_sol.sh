#sed -e "/^\/\/\/ BEGIN SOLUTION/,/^\/\/\/ END SOLUTION/{ /^\/\/\/ BEGIN SOLUTION/{p; r YOUR ANSWER HERE }; /^\/\/\/ END SOLUTION/p; d }"  test_date_template.c
#perl -0777 -i -pe "s#(/// BEGIN SOLUTION\\n).*(\\n/// END SOLUTION)#YOUR ANSWER HERE#g" test_date_template.c
sed "/\/\/\/ BEGIN SOLUTION/,/\/\/\/ END SOLUTION/c\\/\/YOUR ANSWER HERE" test_date_sol.c > test_date_template.c
sed "/\/\/\/ BEGIN SOLUTION/,/\/\/\/ END SOLUTION/c\\/\/YOUR ANSWER HERE" test_factorial_sol.c > test_factorial_template.c
sed "/\/\/\/ BEGIN SOLUTION/,/\/\/\/ END SOLUTION/c\\/\/YOUR ANSWER HERE" test_prime_sol.c > test_prime_template.c
