cmake_minimum_required(VERSION 3.5)
project(bcr_bot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes models urdf worlds rviz
    DESTINATION share/${PROJECT_NAME}/
)

install( DIRECTORY config/
         DESTINATION share/${PROJECT_NAME}/config)
         
         
install(PROGRAMS scripts/remapper.py
        DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/localization_wrapper.py
        DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/scan_matching.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
