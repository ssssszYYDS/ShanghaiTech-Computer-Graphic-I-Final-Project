cmake_minimum_required(VERSION 3.5)
project(mass-spring-cloth-simulation)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS  "-O2")

# Flags for compiling
if(NO_OMP STREQUAL 1) 
    message(STATUS "OpenMP disabled.")
	add_definitions(-DNO_OMP)
else()
    message(STATUS "OpenMP enabled.")
    set(NO_OMP 0)
    remove_definitions(-DNO_OMP)
endif()

# Find OpenMP
find_package(OpenMP)
if(NO_OMP STREQUAL 0 AND OpenMP_FOUND)
	message(STATUS "OpenMP Found.")
    include_directories(${OpenMP_CXX_INCLUDE_DIRS})
endif()

# Add include path
include_directories(
	src
	extern
	include
)

# glfw
option(GLFW_BUILD_TESTS off)
option(GLFW_BUILD_EXAMPLES off)
option(GLFW_BUILD_DOCS off)
add_subdirectory(extern/glfw)
include_directories(extern/glfw/include)
list(APPEND DEPENDENCIES glfw ${glfw_LIBRARIES})

# glad
set(GLAD_INCLUDE extern/glad/include)
set(GLAD_SRC extern/glad/src/glad.c)
include_directories(${GLAD_INCLUDE})

# glm
add_subdirectory(extern/glm)
list(APPEND DEPENDENCIES glm::glm)

# Find source files
file(GLOB_RECURSE SOURCE_FILES 
    src/*.cpp
	include/*.hpp
	${GLAD_SRC}
)

add_library(libmain
    src/camera.cpp
    src/cloth.cpp
    src/cloth_renderer.cpp
    src/cloth_simulator.cpp
    src/shader.cpp
    ${GLAD_SRC}
)
target_include_directories(libmain
    PUBLIC include
)
target_link_libraries(libmain PUBLIC ${DEPENDENCIES})


# Specific flags
if(WIN32)
	add_definitions(-D_AMD64_)
endif()

# Executables
add_executable(main ${SOURCE_FILES})
target_link_libraries(main PUBLIC libmain OpenMP::OpenMP_CXX)


