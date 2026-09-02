idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES ppg_acq ads131m02 servo state_machine
)
