idf_component_register(
    SRCS 
    "platform/vl6180x_i2c.c"
    "core/src/vl6180x_api.c"
    "vl6180x.cpp"
    INCLUDE_DIRS
    "."
    "platform/inc"
    "core/inc"
    # REQUIRES ""
)

target_compile_options(${COMPONENT_LIB} PUBLIC 
    "-Wno-maybe-uninitialized"
)
