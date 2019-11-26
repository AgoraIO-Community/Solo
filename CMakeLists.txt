cmake_minimum_required(VERSION 3.1)
project(solo VERSION 1.0.0 LANGUAGES C)

option(BUILD_ARM "Build ARM lib and test" OFF)

if (BUILD_ARM)
    set(CODE_PATH ${PROJECT_SOURCE_DIR}/JC1_SDK_SRC_ARM)
else()
    set(CODE_PATH ${PROJECT_SOURCE_DIR}/JC1_SDK_SRC_FLP)
endif()

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

include_directories(
    ${CODE_PATH}/interface
    ${CODE_PATH}/src/libBWE
    ${CODE_PATH}/src/libSATECodec
    )

file(GLOB SRC_LIST
    ${CODE_PATH}/interface/*.h
    ${CODE_PATH}/src/libBWE/*.c
    ${CODE_PATH}/src/libBWE/*.h
    ${CODE_PATH}/src/libSATECodec/*.c
    ${CODE_PATH}/src/libSATECodec/*.h
    )

add_library(JC1Codec STATIC ${SRC_LIST})

add_executable(JC1Encoder ${CODE_PATH}/test/enc_main.c)
target_link_libraries(JC1Encoder JC1Codec)

add_executable(JC1Decoder ${CODE_PATH}/test/dec_main.c)
target_link_libraries(JC1Decoder JC1Codec)
