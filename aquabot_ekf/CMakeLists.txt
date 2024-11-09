cmake_minimum_required(VERSION 3.8)
project(aquabot_ekf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)
ament_auto_find_build_dependencies()


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")


#LES TROIS LIGNES SONT A AJOUTER POUR QUE BUILD MARCHE ET PAS D'ERREUR AU LANCEMENT DE LA NODE
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr/share/cmake/geographiclib/")
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
#---------------------------------------------------------------------------------------------

find_package(GeographicLib REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories(include)
ament_auto_add_executable(gps2pose src/gps2pose.cpp)
target_link_libraries(gps2pose ${GeographicLib_LIBRARIES})

install(DIRECTORY launch params
        DESTINATION share/${PROJECT_NAME})


ament_auto_package()
