cmake_minimum_required(VERSION 3.16)
project(espidf_gattserver_contract_tests LANGUAGES CXX)

enable_testing()

find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(Threads REQUIRED)
add_test(
    NAME gatt_custom_notify_contract_test
    COMMAND
        "${Python3_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/custom_notify_contract_test.py"
)
add_test(
    NAME gatt_zephyr_backend_contract_test
    COMMAND
        "${Python3_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/zephyr_backend_contract_test.py"
)
add_test(
    NAME gatt_notify_buffer_contract_test
    COMMAND
        "${Python3_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/notify_buffer_contract_test.py"
)

add_test(
    NAME gatt_notify_log_contract_test
    COMMAND
        "${Python3_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/notify_log_contract_test.py"
)

add_executable(gatt_flags_contract_test flags_contract_test.cpp)
target_compile_features(gatt_flags_contract_test PRIVATE cxx_std_17)
target_include_directories(gatt_flags_contract_test PRIVATE
    ../include
    stubs
)

add_test(NAME gatt_flags_contract_test COMMAND gatt_flags_contract_test)

add_executable(gatt_connection_state_test connection_state_test.cpp)
target_compile_features(gatt_connection_state_test PRIVATE cxx_std_17)
target_include_directories(gatt_connection_state_test PRIVATE ..)

add_test(NAME gatt_connection_state_test COMMAND gatt_connection_state_test)

add_executable(gatt_notify_log_limiter_test notify_log_limiter_test.cpp)
target_compile_features(gatt_notify_log_limiter_test PRIVATE cxx_std_17)
target_include_directories(gatt_notify_log_limiter_test PRIVATE ..)
target_link_libraries(gatt_notify_log_limiter_test PRIVATE Threads::Threads)

add_test(NAME gatt_notify_log_limiter_test COMMAND gatt_notify_log_limiter_test)

add_executable(gatt_service_change_test service_change_test.cpp)
target_compile_features(gatt_service_change_test PRIVATE cxx_std_17)
target_include_directories(gatt_service_change_test PRIVATE ..)

add_test(NAME gatt_service_change_test COMMAND gatt_service_change_test)

add_executable(gatt_value_storage_test value_storage_test.cpp)
target_compile_features(gatt_value_storage_test PRIVATE cxx_std_17)
target_include_directories(gatt_value_storage_test PRIVATE ..)

add_test(NAME gatt_value_storage_test COMMAND gatt_value_storage_test)

add_executable(gatt_zephyr_table_builder_test zephyr_table_builder_test.cpp)
target_compile_features(gatt_zephyr_table_builder_test PRIVATE cxx_std_17)
target_include_directories(gatt_zephyr_table_builder_test PRIVATE
    ..
    ../include
    stubs
)

add_test(NAME gatt_zephyr_table_builder_test COMMAND gatt_zephyr_table_builder_test)

add_executable(
    gatt_zephyr_write_assembler_test
    zephyr_write_assembler_test.cpp
    ../backends/zephyr/gatt_write_assembler.cpp
)
target_compile_features(gatt_zephyr_write_assembler_test PRIVATE cxx_std_17)
target_include_directories(gatt_zephyr_write_assembler_test PRIVATE
    ../backends/zephyr
)

add_test(NAME gatt_zephyr_write_assembler_test COMMAND gatt_zephyr_write_assembler_test)

add_executable(gatt_zephyr_value_arena_test zephyr_value_arena_test.cpp)
target_compile_features(gatt_zephyr_value_arena_test PRIVATE cxx_std_17)
target_include_directories(gatt_zephyr_value_arena_test PRIVATE
    ../backends/zephyr
)

add_test(NAME gatt_zephyr_value_arena_test COMMAND gatt_zephyr_value_arena_test)

add_executable(gatt_zephyr_notify_retry_test zephyr_notify_retry_test.cpp)
target_compile_features(gatt_zephyr_notify_retry_test PRIVATE cxx_std_17)
target_include_directories(gatt_zephyr_notify_retry_test PRIVATE
    ../backends/zephyr
)

add_test(NAME gatt_zephyr_notify_retry_test COMMAND gatt_zephyr_notify_retry_test)
