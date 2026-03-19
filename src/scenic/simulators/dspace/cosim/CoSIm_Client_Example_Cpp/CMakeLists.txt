cmake_minimum_required(VERSION 3.13)

project(DsVeosCoSimDemo VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

add_subdirectory(ThirdParty/veos-cosim-client)

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE DsVeosCoSim)