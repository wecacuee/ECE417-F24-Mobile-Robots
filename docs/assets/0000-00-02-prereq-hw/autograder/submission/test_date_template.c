// This file provides a template to implement Gregorian calendar date
// difference.
//
// You have to edit the parts that say YOUR ANSWER HERE to make sure that all
// the tests pass.
#include <stdio.h>
#include <assert.h>

struct date {
    // Define a structure with month, day and year as int types
    //
    // Make sure the date structure matches its usage elsewhere
//YOUR ANSWER HERE
};
// Short name for struct date is date_t
typedef struct date date_t;

// Array defining the days of the month
unsigned short GREG_CAL_DAYSINMONTH[12] = {31, 28, 31, 30, 31, 30,
                                           31, 31, 30, 31, 30, 31};

unsigned short greg_is_leap_year(int year) {
    // Logic determining of a leap year according to
    // https://en.wikipedia.org/wiki/Proleptic_Gregorian_calendar
    // https://en.wikipedia.org/wiki/Leap_year
    //
    // Every year that is 
    // 1. exactly divisible by four is a leap year,
    // 2. except for years that are exactly divisible by 100,
    // 3. but these centurial years are leap years if they are exactly
    // divisible by 400.
    //
    // For example, the years 1700, 1800, and 1900
    // are not leap years, but the years 1600 and 2000 are
    // 
    // Return 1 if the year is a leap year and 0 if the year is not a leap
    // year
//YOUR ANSWER HERE
}

// Count the number of Proleptic Gregorian calendar leap years since
// 0001-01-01 AD
unsigned int greg_leap_years(int year) {
    // Every year that is exactly divisible by four is a leap year,
    // except for years that are exactly divisible by 100,
    // but these centurial years are leap years if they are exactly
    // divisible by 400. For example, the years 1700, 1800, and 1900
    // are not leap years, but the years 1600 and 2000 are
    int julian_leap = (year-1) / 4; // Leap years divisible by 4
    int gregorian_leap = julian_leap
                - ((year-1) / 100) // Not leap years divisible by 100
                + ((year-1) / 400); // But leap years divisble by 400
    return gregorian_leap;
}

// Count the number of days since the start of this year
unsigned int date_greg_days_since_start_of_year(date_t date) {
    int month_days = 0;
    for (int month = 0; month < date.month-1; ++month) {
        // Add the number of days each month to month_days, but take into
        // account the month of February (month == 1) for the case of leap
        // year.
//YOUR ANSWER HERE
    }
    // Return days in this month and month_days so far.
    return month_days + (date.day-1);
}

// Return the total number of days since 0001-01-01 AD by
// Proleptic Gregorian calendar
unsigned int date_greg_days(date_t dt) {
    // Use the functions greg_leap_years and
    // date_greg_days_since_start_of_year to find the total number of
    // days since 0001-01-01 AD.
    // 
    // Return the total number of days
//YOUR ANSWER HERE
}

// Return the days between dt1 - dt2
unsigned int date_days_diff(date_t dt1, date_t dt2) {
    return date_greg_days(dt1) - date_greg_days(dt2);
}

int main() {
    // Test leap years 
    // Test 1 AD
    fprintf(stderr,
        (!greg_is_leap_year(1)) ?
        "Test passed; 1 is not leap\n"
        : "Test failed;; 1 is not leap\n");

    // Test 4 AD
    fprintf(stderr,
        (greg_is_leap_year(4)) ?
        "Test passed; 4 is leap\n"
        : "Test failed;; 4 is leap\n");

    // Test 100 AD
    fprintf(stderr,
        (!greg_is_leap_year(100)) ?
        "Test passed; 100 is not leap\n"
        : "Test failed;; 100 is not leap\n");
    // Test 400 AD
    fprintf(stderr,
        (greg_is_leap_year(400)) ?
        "Test passed; 400 is leap\n"
        : "Test failed;; 400 is leap\n");

    // Test 2000 AD
    fprintf(stderr,
        (greg_is_leap_year(2000)) ?
        "Test passed; 2000 is leap\n"
        : "Test failed;; 2000 is leap\n");

    // Test 1900 AD
    fprintf(stderr,
        (!greg_is_leap_year(1900)) ?
        "Test passed; 1900 is not leap\n"
        : "Test failed;; 1900 is not leap\n");

    // Test 1
    date_t check_days = { .month = 1, .day = 23, .year = 1 };
    unsigned int correct = 22;
    unsigned int found = date_greg_days(check_days);
    fprintf(stderr,
        (found == correct) ?
        "Test passed; days to 0001-01-23=%u=%u\n"
        : "Test failed; days to 0001-01-23=%u!=%u\n",
        correct, found);

    // Test 2
    date_t check_leap_month = { .month = 3, .day = 1, .year = 4 };
    correct = 1155;
    found = date_greg_days(check_leap_month);
    fprintf(stderr,
        (found == correct) ?
        "Test passed; days to 0004-03-01=%u=%u\n"
        : "Test failed; days to 0004-03-01=%u!=%u\n",
        correct, found
    );

    // Test 3
    date_t check_non_leap_month_st = { .month = 2, .day = 1, .year = 1 };
    date_t check_non_leap_month = { .month = 3, .day = 1, .year = 1 };
    correct = 28;
    found = date_days_diff(check_non_leap_month, check_non_leap_month_st);
    fprintf(stderr,
        (found == correct) ?
        "Test passed; days: 0001-03-01 - 0001-02-01=%u=%u\n"
        : "Test failed; days: 0001-03-01 - 0001-02-01=%u!=%u\n",
        correct, found);

    // Test 4
    date_t check_100_non_leap_month_st = { .month = 2, .day = 1, .year = 100 };
    date_t check_100_non_leap_month = { .month = 3, .day = 1, .year = 100 };
    correct = 28;
    found = date_days_diff(check_100_non_leap_month, check_100_non_leap_month_st);
    fprintf(stderr,
        (found == correct) ?
        "Test passed; days: 0100-03-01 - 0100-02-01=%u=%u\n"
        :  "Test failed; days: 0100-03-01 - 0100-02-01=%u=%u\n",
        correct, found);

    // Test 5
    date_t check_400_leap_month_st = { .month = 2, .day = 1, .year = 2000 };
    date_t check_400_leap_month = { .month = 3, .day = 1, .year = 2000 };
    correct = 29;
    found =  date_days_diff(check_400_leap_month, check_400_leap_month_st);
    fprintf(stderr,
        (found == correct) ?
        "Test passed; days: 2000-03-01 - 2000-02-01=%u=%u\n"
        : "Test failed; days: 2000-03-01 - 2000-02-01=%u=%u\n",
        correct, found);

    // Test 6
    date_t start_of_the_course = { .month = 9, .day = 4, .year = 2024 };
    correct = 739132;
    found = date_greg_days(start_of_the_course);
    fprintf(stderr,
        (found == correct) ?
        "Test passed; days to the start of the course=%u=%u\n"
        : "Test failed; days to the start of the course=%u=%u\n",
        correct, found);

    // Testing through input output
    // Wait on command line for an input
    fprintf(stderr, "Enter a date in yyyy-mm-dd format: \n");
    date_t user_inp;
    int ret = scanf("%04d-%02d-%04d",
        &user_inp.year, &user_inp.month, &user_inp.day);
    if (ret != 3) {
    	fprintf(stderr, "Unable to read the date in yyyy-mm-dd format\n");
    	return 1;
    }
    printf("%u\n", date_greg_days(user_inp));
    return 0;
}
