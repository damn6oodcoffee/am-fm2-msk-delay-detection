cmake_minimum_required(VERSION 3.27)
set(CMAKE_CXX_STANDARD 17 REQUIRED)
project(imgui_test)

set(GLFW_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../glfw-3.4")
set(GLFW_DIR_LIB ${GLFW_DIR}/build/src)
set(GLAD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../glad")
set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/imgui)
set(IMPLOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/implot)

file(GLOB imgui_files 
     LIST_DIRECTORIES false 
     RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
     "${IMGUI_DIR}/*.cpp"
     "${IMGUI_DIR}/*.h"
     "${IMPLOT_DIR}/*cpp"
     "${IMPLOT_DIR}/*h")

message("========================================================")
message(${imgui_files})
message("========================================================")

find_package(OpenGL REQUIRED)

add_executable(${PROJECT_NAME}
               MainWindow.cpp
               SignalModel.cpp
               ${GLAD_DIR}/src/glad.c
               ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
               ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
               ${imgui_files})        

target_include_directories(${PROJECT_NAME} PRIVATE 
                           ${GLFW_DIR}/include
                           ${GLAD_DIR}/include 
                           ${IMGUI_DIR}
                           ${IMGUI_DIR}/backends
                           ${IMPLOT_DIR})

target_link_libraries(${PROJECT_NAME} PRIVATE 
                      ${OpenGL_LIBRARIES} 
                      ${GLFW_DIR_LIB}/Debug/glfw3.lib)