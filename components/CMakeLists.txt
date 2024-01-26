idf_component_register(SRCS "DW1000.cpp" 
                            "DW1000Mac.cpp"
                            "DW1000Device.cpp"
                            "DW1000Ranging.cpp"
                            "DW1000Time.cpp"
                            "spi_config.cpp"
                            INCLUDE_DIRS "include"
                            REQUIRES driver esp_timer)
