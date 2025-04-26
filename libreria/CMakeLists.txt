cmake_minimum_required(VERSION 3.12)
project(DWM_Master C)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

# --- Libreria condivisa ---

# creazione libreria condivisa
add_library(dwm_master_shared SHARED src/dwm_master.c)

#  directory header pubblici
target_include_directories(dwm_master_shared PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# impostazioni libreria condivisa
set_target_properties(dwm_master_shared PROPERTIES
        OUTPUT_NAME DWM_Master
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib # per .so/.dll
        ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib # per Windows
        PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/include/dwm_master.h 
)

# --- Libreria statica ---

add_library(dwm_master_static STATIC
        src/dwm_master.c
)

target_include_directories(dwm_master_static PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

set_target_properties(dwm_master_static PROPERTIES
        OUTPUT_NAME DWM_Master
        ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
        PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/include/dwm_master.h 
)