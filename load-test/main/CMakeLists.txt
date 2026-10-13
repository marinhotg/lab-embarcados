# Reuses the pin map from firmware/main/config.h instead of duplicating it.
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "." "../../firmware/main"
)
