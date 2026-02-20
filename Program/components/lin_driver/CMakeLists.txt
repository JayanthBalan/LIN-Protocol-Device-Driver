
idf_component_register(
    SRCS "lin_master.c" "lin_slave.c"
    INCLUDE_DIRS "inc"
    REQUIRES driver freertos log esp_timer
)
