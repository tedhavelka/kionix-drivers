#
# Copyright (c) 2021 Neela Nurseries
#

# Point to this project's root directory.
set(KIONIX_WORK_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE PATH "Kionix driver work root directory")

# Subdirectories
add_subdirectory(drivers)
# add_subdirectory_ifdef(CONFIG_KX132_1211 lib)  . . . 'lib' directory not yet created in this project - TMH

# Include headers
# zephyr_include_directories(include)            . . . also not yet created - TMH
zephyr_include_directories(drivers)
zephyr_include_directories(drivers/kionix/kx132-1211)
