# Use C++ source files
idf_component_register(SRCS "main.cpp" "fram.cpp"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES driver esp_event nvs_flash 
                       )

# Set C and C++ standards to C99 and C++20
target_compile_options(${COMPONENT_LIB} PRIVATE
    $<$<COMPILE_LANGUAGE:C>:-std=c99>
    $<$<COMPILE_LANGUAGE:CXX>:-std=c++20>
)