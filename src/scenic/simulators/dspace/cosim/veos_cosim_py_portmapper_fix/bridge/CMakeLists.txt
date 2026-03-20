cmake_minimum_required(VERSION 3.20)
project(veos_cosim_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(_candidate_roots
  "${CMAKE_CURRENT_LIST_DIR}/../../VeosCoSim_Client/client/x64/Release"
  "${CMAKE_CURRENT_LIST_DIR}/../../VeosCoSim_Client/VeosCoSim_Client/client/x64/Release"
  "${CMAKE_CURRENT_LIST_DIR}/../VeosCoSim_Client/client/x64/Release"
  "${CMAKE_CURRENT_LIST_DIR}/../VeosCoSim_Client/VeosCoSim_Client/client/x64/Release"
)

set(VENDOR_ROOT "")
foreach(candidate IN LISTS _candidate_roots)
  if(EXISTS "${candidate}/include/VeosCoSim.h" AND EXISTS "${candidate}/lib/VeosCoSimApplStatic.lib")
    set(VENDOR_ROOT "${candidate}")
    break()
  endif()
endforeach()

if(VENDOR_ROOT STREQUAL "")
  message(FATAL_ERROR
    "Could not find VeosCoSim release folder. Searched candidates:\n"
    "  ${CMAKE_CURRENT_LIST_DIR}/../../VeosCoSim_Client/client/x64/Release\n"
    "  ${CMAKE_CURRENT_LIST_DIR}/../../VeosCoSim_Client/VeosCoSim_Client/client/x64/Release\n"
    "  ${CMAKE_CURRENT_LIST_DIR}/../VeosCoSim_Client/client/x64/Release\n"
    "  ${CMAKE_CURRENT_LIST_DIR}/../VeosCoSim_Client/VeosCoSim_Client/client/x64/Release\n"
  )
endif()

message(STATUS "Using VENDOR_ROOT=${VENDOR_ROOT}")
set(VENDOR_INCLUDE "${VENDOR_ROOT}/include")
set(VENDOR_LIB_DIR "${VENDOR_ROOT}/lib")
set(VENDOR_STATIC_LIB "${VENDOR_LIB_DIR}/VeosCoSimApplStatic.lib")

add_library(veos_cosim_bridge SHARED veos_cosim_bridge.cpp)
target_include_directories(veos_cosim_bridge PRIVATE "${VENDOR_INCLUDE}")
target_link_libraries(veos_cosim_bridge PRIVATE "${VENDOR_STATIC_LIB}")
target_compile_definitions(veos_cosim_bridge PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
