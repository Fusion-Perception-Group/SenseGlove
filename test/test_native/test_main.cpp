#include "ffmt.hpp"
#include "unity.h"
#include <string>

void setUp()
{
}

void tearDown()
{
}

void test_ffmt_parse()
{
    using namespace vermils::ffmt;
    int a = 123;
    double d = 123.321;
    HolderContainer phs;
    char fmt[] = "a = {{ {0:^+020.3}, {{} {1:.e}, {}";
    parse(phs, fmt);

    TEST_ASSERT_EQUAL(0, phs[0].index);
    TEST_ASSERT_TRUE(phs[0].escape);

    TEST_ASSERT_EQUAL(0, phs[1].index);
    TEST_ASSERT_FALSE(phs[1].escape);
    TEST_ASSERT_EQUAL(20, phs[1].padding);
    TEST_ASSERT_EQUAL(3, phs[1].precision);
    TEST_ASSERT_EQUAL(Center, phs[1].align);
    TEST_ASSERT_EQUAL(DefaultFormat, phs[1].format);
    TEST_ASSERT_EQUAL('0', phs[1].fill);
    TEST_ASSERT_TRUE(phs[1].show_positive);
    TEST_ASSERT_EQUAL(7, phs[1].begin);
    TEST_ASSERT_EQUAL(18, phs[1].end);

    TEST_ASSERT_EQUAL(1, phs[3].index);
    TEST_ASSERT_FALSE(phs[3].escape);
    TEST_ASSERT_EQUAL(0, phs[3].padding);
    TEST_ASSERT_EQUAL(6, phs[3].precision);
    TEST_ASSERT_EQUAL(Right, phs[3].align);
    TEST_ASSERT_EQUAL(Scientific, phs[3].format);
    TEST_ASSERT_EQUAL(' ', phs[3].fill);
    TEST_ASSERT_FALSE(phs[3].show_positive);
    TEST_ASSERT_EQUAL(24, phs[3].begin);
    TEST_ASSERT_EQUAL(30, phs[3].end);
}

void test_ffmt()
{
    using namespace vermils::ffmt;
    int a = 123;
    double d = 123.321;
    
    TEST_ASSERT_EQUAL_STRING("{}", format("{{}").data());

    TEST_ASSERT_EQUAL_STRING("a = +00123",  format("a = {:+06}", a).data());
    TEST_ASSERT_EQUAL_STRING("a = +00123",  format("a = {:+06}", 123).data());
    TEST_ASSERT_EQUAL_STRING("a = 123  ",  format("a = {:<05}", a).data());
    TEST_ASSERT_EQUAL_STRING("a = 123  ",  format("a = {:<05}", 123).data());
    TEST_ASSERT_EQUAL_STRING("a = -123 ",  format("a = {:<05}", -123).data());
    TEST_ASSERT_EQUAL_STRING("a = +0", format("a = {:+}", 0).data());
    TEST_ASSERT_EQUAL_STRING("a = 0", format("a = {:}", 0).data());
    TEST_ASSERT_EQUAL_STRING("0x7b", format("0x{:.x}", a).data());
    TEST_ASSERT_EQUAL_STRING("deadbeef", format("{:.x}", 0xdeadbeef).data());

    TEST_ASSERT_EQUAL_STRING("true", format("{}", true).data());
    TEST_ASSERT_EQUAL_STRING("false", format("{}", false).data());

    TEST_ASSERT_EQUAL_STRING("d = +00123.321", format("d = {:+010.3}", d).data());
    TEST_ASSERT_EQUAL_STRING("d = 123.321000", format("d = {:<010.3}", 123.321).data());
    TEST_ASSERT_EQUAL_STRING("1.23321e2", format("{:.5e}", d).data());

    uint32_t ptr = 0x00005678;
    TEST_ASSERT_EQUAL_STRING("00005678", format("{:.p}", ptr).data());

    auto equivalent_ptr_fmt_str = format("{{:0{}}", sizeof(void*) * 2);
    TEST_ASSERT_EQUAL_STRING(format(equivalent_ptr_fmt_str, (void*)0x12345678).data(), format("{}", (void*)0x12345678).data());

    TEST_ASSERT_EQUAL_STRING("*****YES*****", format("{:*^13}", "YES").data());
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_ffmt_parse);
    RUN_TEST(test_ffmt);
    return UNITY_END();
}