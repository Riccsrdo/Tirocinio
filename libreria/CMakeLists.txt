cmake_minimum_required(VERSION 3.12)
project(DWM_Master C)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(BUILD_SHARED "Build shared libraries instead of static ones" ON)
set(DEBUG_FLAG "Allow debug prints" OFF)

if(DEBUG_FLAG)
    add_compile_definitions(Debug)
endif()

set(LIB_TYPE STATIC)
if(BUILD_SHARED)
    set(LIB_TYPE SHARED)
endif()

# --- Libreria dinamica ---

# creazione libreria 
add_library(dwm_master ${LIB_TYPE} src/dwm_master.c)

if(BUILD_SHARED)
        # impostazioni libreria dinamica
        set_target_properties(dwm_master_shared PROPERTIES
                OUTPUT_NAME DWM_Master
                ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib # per Windows
                PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/include/dwm_master.h 
        )
else()
        # impostazioni libreria statica
        set_target_properties(dwm_master_static PROPERTIES
                OUTPUT_NAME DWM_Master
                ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
                PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/include/dwm_master.h 
        )
endif()

#  directory header
target_include_directories(dwm_master PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)






