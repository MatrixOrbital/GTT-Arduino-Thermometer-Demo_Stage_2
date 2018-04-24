set(SRC 
	demo.c
)
include_directories(${CMAKE_SOURCE_DIR}/gttclient)
add_executable(generic_demo ${SRC})
target_link_libraries(generic_demo gttclient)