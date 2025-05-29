#include "unity.h"

// Run all registered test cases
extern void unity_run_all_tests(void);

void app_main(void)
{
    UNITY_BEGIN();
    unity_run_all_tests();
    UNITY_END();
}