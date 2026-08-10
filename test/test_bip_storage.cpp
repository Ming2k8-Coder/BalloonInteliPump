#include "unity.h"
#include "bip_storage.h"

void test_nvs_storage_struct_alignment(void) {
    // Verify SystemConfig struct size remains safe for storage
    TEST_ASSERT_LESS_OR_EQUAL(sizeof(SystemConfig), 256);
}

void run_storage_tests(void) {
    RUN_TEST(test_nvs_storage_struct_alignment);
}
