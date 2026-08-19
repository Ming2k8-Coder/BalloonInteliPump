#include "unity.h"
#include "bip_storage.h"

void test_nvs_storage_struct_alignment(void) {
    TEST_ASSERT_LESS_OR_EQUAL(sizeof(SystemConfig), 256);
}

void test_nvs_storage_crc32_and_write_counter(void) {
    uint32_t writes_before = get_nvs_write_count();
    save_system_config();
    uint32_t writes_after = get_nvs_write_count();
    TEST_ASSERT_EQUAL_UINT32(writes_before + 1, writes_after);
}

void run_storage_tests(void) {
    RUN_TEST(test_nvs_storage_struct_alignment);
    RUN_TEST(test_nvs_storage_crc32_and_write_counter);
}

