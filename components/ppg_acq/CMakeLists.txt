idf_component_register(
    SRCS "fir.c" "ppg_stream.c" "ppg_acq.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer ads131m02
)
