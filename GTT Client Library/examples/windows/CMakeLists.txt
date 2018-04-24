set(SRC 
			demo.c
			serial_gtt.c
			serial.c
			GTTProject17.c
)
include_directories(${CMAKE_SOURCE_DIR}/gttclient)
add_executable(demo ${SRC})
target_link_libraries(demo gttclient)