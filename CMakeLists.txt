idf_component_register(SRCS "gattserver.cpp" "gap.cpp" "gatt.cpp" "print.cpp"
INCLUDE_DIRS "include"
REQUIRES bt esp_event nvs_flash)