# Created by the script cgal_create_CMakeLists
# This is the CMake script for compiling a set of CGAL applications.

cmake_minimum_required(VERSION 3.1...3.23)

project(Polygonal_surface_reconstruction_Examples)

#set(CMAKE_C_COMPILER "gcc-13")
#set(CMAKE_CXX_COMPILER "g++-13")

#add_compile_options(-std=c++20)
#add_compile_options(-std=c++14)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fsanitize=undefined")

set(CMAKE_THREAD_LIBS_INIT "-lpthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)

# OPENMP variables
set(OpenMP_CXX_FLAGS "-fopenmp=lomp")
set(OpenMP_CXX_LIB_NAMES "libomp")
#set(OpenMP_libomp_LIBRARY "/usr/local/Cellar/libomp/18.1.3/lib/libomp.dylib")
#set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp /usr/local/Cellar/libomp/18.1.3/lib/libomp.dylib -I/usr/local/Cellar/libomp/18.1.3/include")

# Vulkan
find_package(Vulkan)

## Threads
#find_package(Threads REQUIRED)

# OpenMP
find_package(OpenMP)
#if (OPENMP_FOUND)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
#endif ()

# PCL and its components
find_package(PCL 1.12.1 REQUIRED common io search kdtree octree surface filters)
list(APPEND ADDITIONAL_OPTIONS
        -VTK_MODULE_ENABLE_VTK_ViewsQt=YES
        -VTK_MODULE_ENABLE_VTK_GuiSupportQt=YES
        )

# VTK
find_package(VTK)

# OPENCV
find_package(OpenCV REQUIRED)

# Boost and its components
find_package(Boost REQUIRED COMPONENTS json)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
endif()


if (NOT Boost_FOUND)

    message(
            STATUS
            "NOTICE: This project requires the Boost library, and will not be compiled."
    )

    return()

endif ()

# Eigen3
find_package(Eigen3 3.1.0) #(requires 3.1.0 or greater)

# PCL
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OPENCV
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR})

# Creating entries for all C++ files with "main" routine
# ##########################################################

add_library(iopcl STATIC IO/Io_pcl.h IO/Io_pcl.cpp)
add_library(geometry STATIC Geometry/Geometry_pcl.h Geometry/Geometry_pcl.cpp)
add_library(algo_reconstruction STATIC Algo_reconstruction.h Algo_reconstruction.cpp)
add_library(shift STATIC PointClouds/Shift.h PointClouds/Shift.cpp)
add_library(string_utils STATIC StringUtils/PathUtils.h StringUtils/PathUtils.cpp)
add_library(file_utils STATIC FileUtils/FileUtils.h FileUtils/FileUtils.cpp)
add_library(texturing_mapping STATIC Texturing/TexturingMapping.h Texturing/TexturingMapping.cpp
        Texturing/IO/Image_txt_reading.h Texturing/IO/Image_txt_reading.cpp
        Texturing/Image/Panorama2cubemap.h Texturing/Image/Panorama2cubemap.cpp
        Texturing/Image/Lidar2depth.h Texturing/Image/Lidar2depth.cpp
        Texturing/MtlUtils.h Texturing/MtlUtils.cpp
        Texturing/FaceIndexMaps.h Texturing/FaceIndexMaps.cpp)
add_library(metrics STATIC Metrics/Metrics.h Metrics/Metrics.cpp
            Metrics/BlurEstimation.h Metrics/BlurEstimation.cpp)
add_library(color_transfer STATIC ColorTransfer/ColorTransferMeanLocal.h ColorTransfer/ColorTransferMeanLocal.cpp
            ColorTransfer/ColorTransferMeanSamePolygons.h ColorTransfer/ColorTransferMeanSamePolygons.cpp)
add_library(best_fitting_texturing STATIC BestFittingTexturing/BestFittingTexturingQuality.h BestFittingTexturing/BestFittingTexturingQuality.cpp)
add_executable(texturing_map_example texturing_map_example.cpp)
add_executable(metric_image_compare_example metric_image_compare_example.cpp)
add_executable(metrics_examples metrics_examples.cpp)
add_library(building_reconstruction STATIC Building_reconstruction.h Building_reconstruction.cpp)
add_executable(building_reconstruction_example building_reconstruction_example.cpp)

foreach (
        target
        metrics
        texturing_mapping
        color_transfer
)
    target_link_libraries(${target} PUBLIC ${OpenCV_LIBS})
endforeach ()

foreach (
        target
        iopcl
        geometry
        algo_reconstruction
        shift
        string_utils
        texturing_mapping
        metrics
        building_reconstruction
        color_transfer
        best_fitting_texturing
)
    target_link_libraries(${target} PUBLIC Eigen3::Eigen ${PCL_LIBRARIES})
endforeach ()

target_link_libraries(metric_image_compare_example PUBLIC metrics)
target_link_libraries(metrics_examples PUBLIC metrics)
target_link_libraries(texturing_mapping PUBLIC color_transfer)

foreach (
        target
        texturing_map_example
)
    target_link_libraries(${target} PUBLIC iopcl)
    target_link_libraries(${target} PUBLIC geometry)
    target_link_libraries(${target} PUBLIC algo_reconstruction)
    target_link_libraries(${target} PUBLIC shift)
    target_link_libraries(${target} PUBLIC string_utils)
    target_link_libraries(${target} PUBLIC file_utils)
    target_link_libraries(${target} PUBLIC building_reconstruction)
    target_link_libraries(${target} PUBLIC texturing_mapping)
    target_link_libraries(${target} PUBLIC color_transfer)
    target_link_libraries(${target} PUBLIC best_fitting_texturing)
    target_link_libraries(${target} PUBLIC Eigen3::Eigen ${PCL_LIBRARIES} ${Boost_LIBRARIES})
endforeach ()

foreach (
        target
        building_reconstruction_example
)
    target_link_libraries(${target} PUBLIC iopcl)
    target_link_libraries(${target} PUBLIC geometry)
    target_link_libraries(${target} PUBLIC algo_reconstruction)
    target_link_libraries(${target} PUBLIC shift)
    target_link_libraries(${target} PUBLIC string_utils)
    target_link_libraries(${target} PUBLIC building_reconstruction)
    target_link_libraries(${target} PUBLIC texturing_mapping)
    target_link_libraries(${target} PUBLIC Eigen3::Eigen ${PCL_LIBRARIES})
endforeach ()

target_link_libraries(texturing_mapping PUBLIC geometry)
target_link_libraries(texturing_mapping PUBLIC string_utils)
target_link_libraries(texturing_mapping PUBLIC file_utils)

target_link_libraries(color_transfer PUBLIC texturing_mapping)
target_link_libraries(color_transfer PUBLIC string_utils)
target_link_libraries(color_transfer PUBLIC ${Boost_LIBRARIES})

target_link_libraries(best_fitting_texturing PUBLIC texturing_mapping)


