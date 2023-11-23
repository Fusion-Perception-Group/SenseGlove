#include <iostream>
#include <vector>
#include "unity.h"
#include "CLK_CFG.h"

void setUp()
{
}

void tearDown()
{
}

void test_test()
{
    TEST_ASSERT_EQUAL(1, 1);
}

int main()
{
    HAL_Init();
    SystemClock_Config();
    UNITY_BEGIN();
    RUN_TEST(test_test);
    return UNITY_END();
}
