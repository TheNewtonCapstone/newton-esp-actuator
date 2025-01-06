FILE(GLOB_RECURSE app_sources src/*.*)

idf_component_register(SRCS ${app_sources}
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES esp_timer esp_driver_pcnt esp_driver_gpio esp_driver_mcpwm)