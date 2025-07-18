cmake_minimum_required(VERSION 3.12)
project(DWM_Master C)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(BUILD_SHARED "Build shared libraries instead of static ones" ON)
set(DEBUG_FLAG "Allow debug prints" OFF)

if(DEBUG_FLAG)
    add_compile_definitions(Debug)
endif()

# --- Definizione della libreria ---
if(BUILD_SHARED)
    # Crea la libreria direttamente come SHARED
    add_library(dwm_master SHARED src/dwm_master.c src/dwm_utilities.c)
else()
    # Crea la libreria direttamente come STATIC
    add_library(dwm_master STATIC src/dwm_master.c src/dwm_utilities.c)
endif()

set_target_properties(dwm_master PROPERTIES OUTPUT_NAME DWM_Master)

# --- Header directory ---

target_include_directories(dwm_master
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# per librerie condivise in unix
set_target_properties(dwm_master PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Per creare eseguibile di test:
#
# add_executable(test main.c)
#
# collego libreria all'eseguibile di test:
# target_link_libraries(test dwm_master)






