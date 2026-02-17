cmake_minimum_required(VERSION 3.16)
project(gr-felica LANGUAGES CXX)

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Gnuradio 3.10 REQUIRED COMPONENTS runtime)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

set(FELICA_PYTHON_DIR
    "${CMAKE_INSTALL_LIBDIR}/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/dist-packages")

add_subdirectory(lib)
add_subdirectory(python)
add_subdirectory(grc)
