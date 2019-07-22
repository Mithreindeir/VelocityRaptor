cmake_minimum_required (VERSION 3.1)
project (VELOCITYRAPTOR C)
file (GLOB SOURCES "src/*.c")

add_library(VelocityRaptor STATIC ${SOURCES})
target_link_libraries(VelocityRaptor m)
set_property(TARGET VelocityRaptor PROPERTY POSITION_INDEPENDENT_CODE ON)
