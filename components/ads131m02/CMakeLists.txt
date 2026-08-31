idf_component_register(
    SRCS "ads131m02.c" "ads131m02_frame.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer
)
