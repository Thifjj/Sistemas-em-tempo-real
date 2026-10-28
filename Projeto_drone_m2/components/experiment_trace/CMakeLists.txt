idf_component_register(SRCS "experiment_trace.c" INCLUDE_DIRS "include" REQUIRES freertos esp_timer esp_trace WHOLE_ARCHIVE)
idf_component_get_property(freertos_lib freertos COMPONENT_LIB)
target_include_directories(${freertos_lib} PUBLIC "${CMAKE_CURRENT_LIST_DIR}/include")
