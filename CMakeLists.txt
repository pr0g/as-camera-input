cmake_minimum_required(VERSION 3.15)
project(as-camera-input LANGUAGES CXX)

include(FetchContent)

FetchContent_Declare(
    as-camera
    GIT_REPOSITORY https://github.com/pr0g/as-camera.git
    GIT_TAG        2c4be372017e0e3a7a74e05cb135429c75eb2d4a)
FetchContent_Declare(
    bec
    GIT_REPOSITORY https://github.com/pr0g/Bit-Field-Enum-Class.git
    GIT_TAG        origin/main)
FetchContent_Declare(
    cmake-helpers
    GIT_REPOSITORY https://github.com/pr0g/cmake-helpers.git
    GIT_TAG        origin/main)

FetchContent_MakeAvailable(as-camera bec cmake-helpers)
    
list(APPEND CMAKE_MODULE_PATH ${cmake-helpers_SOURCE_DIR})

add_library(${PROJECT_NAME})
target_sources(${PROJECT_NAME} PRIVATE src/as-camera-input.cpp)
target_include_directories(
    ${PROJECT_NAME}
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "d")
target_link_libraries(${PROJECT_NAME} PUBLIC as-camera PRIVATE bec)
target_compile_definitions(
    ${PROJECT_NAME}
    PRIVATE $<$<BOOL:${AS_PRECISION_FLOAT}>:AS_PRECISION_FLOAT>
            $<$<BOOL:${AS_PRECISION_DOUBLE}>:AS_PRECISION_DOUBLE>
            $<$<BOOL:${AS_COL_MAJOR}>:AS_COL_MAJOR>
            $<$<BOOL:${AS_ROW_MAJOR}>:AS_ROW_MAJOR>)

include(thh-install)
thh_install_static_with_dependencies(${PROJECT_NAME} as-camera-input)
