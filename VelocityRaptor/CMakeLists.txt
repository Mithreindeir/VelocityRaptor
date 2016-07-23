cmake_minimum_required (VERSION 2.8.7)
project (VELOCITYRAPTOR)
file (GLOB SOURCES "src/*.c")

add_library(VelocityRaptor ${SOURCES})
set_property(TARGET VelocityRaptor PROPERTY C_STANDARD 99)
set_property(TARGET VelocityRaptor PROPERTY lm)
