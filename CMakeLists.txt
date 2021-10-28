cmake_minimum_required(VERSION 2.8)
project( auto_parking )

find_package( OpenCV REQUIRED )

set(CMAKE_CXX_STANDARD 17)

include_directories( 
    ${OpenCV_INCLUDE_DIRS} 
    ${CMAKE_SOURCE_DIR}
    /usr/local/include
)

add_library(readosm SHARED IMPORTED)
set_target_properties(
    readosm
    PROPERTIES IMPORTED_LOCATION
    /usr/local/lib/libreadosm.dylib
)

# add_library(hdmap_client SHARED IMPORTED)
# set_target_properties(
#     hdmap_client
#     PROPERTIES IMPORTED_LOCATION
#     ${CMAKE_SOURCE_DIR}/lib/libhdmap_client.so
# )


add_executable( auto_parking  ${CMAKE_SOURCE_DIR}/read_osm.cpp ${CMAKE_SOURCE_DIR}/helper.cpp)


target_link_libraries( auto_parking ${OpenCV_LIBS} readosm expat z)
