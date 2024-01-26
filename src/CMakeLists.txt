cmake_minimum_required(VERSION 3.5)

# Register the main application component
idf_component_register(
    SRCS "main.cpp"  # List all your source files here
    INCLUDE_DIRS "."  # Include directory for the main component
)