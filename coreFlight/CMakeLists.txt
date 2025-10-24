cmake_minimum_required(VERSION 4.0)
project(coreFlight C)

set(CMAKE_C_STANDARD 11)

add_executable(coreFlight main.c
        lib/ccsds/spp.c
        lib/ccsds/spp.h
        lib/filesystem/filesystem.c
        lib/filesystem/filesystem.h
        lib/task/task.c
        lib/task/task.h)
