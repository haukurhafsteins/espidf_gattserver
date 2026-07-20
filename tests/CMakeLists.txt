cmake_minimum_required(VERSION 3.16)
project(espidf_gattserver_contract_tests LANGUAGES CXX)

enable_testing()

add_executable(gatt_flags_contract_test flags_contract_test.cpp)
target_compile_features(gatt_flags_contract_test PRIVATE cxx_std_17)
target_include_directories(gatt_flags_contract_test PRIVATE
    ../include
    stubs
)

add_test(NAME gatt_flags_contract_test COMMAND gatt_flags_contract_test)
