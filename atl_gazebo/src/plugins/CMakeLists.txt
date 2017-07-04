SET(
    atl_GAZEBO_DEPS
    atl_gazebo_msgs
    wave
    ${GAZEBO_LIBRARIES}
    ${OpenCV_LIBS}
    yaml-cpp
    pthread
)

# PLUGINS
ADD_LIBRARY(box_gplugin SHARED box_gplugin.cpp)
ADD_DEPENDENCIES(box_gplugin atl_gazebo_msgs)
TARGET_LINK_LIBRARIES(box_gplugin ${atl_GAZEBO_DEPS})

ADD_LIBRARY(camera_gplugin SHARED camera_gplugin.cpp)
ADD_DEPENDENCIES(camera_gplugin atl_gazebo_msgs)
TARGET_LINK_LIBRARIES(camera_gplugin ${atl_GAZEBO_DEPS})

ADD_LIBRARY(df_camera_gplugin SHARED df_camera_gplugin.cpp)
ADD_DEPENDENCIES(df_camera_gplugin atl_gazebo_msgs)
TARGET_LINK_LIBRARIES(df_camera_gplugin ${atl_GAZEBO_DEPS})

ADD_LIBRARY(gimbal_gplugin SHARED gimbal_gplugin.cpp)
ADD_DEPENDENCIES(gimbal_gplugin atl_gazebo_msgs)
TARGET_LINK_LIBRARIES(gimbal_gplugin ${atl_GAZEBO_DEPS})

ADD_LIBRARY(lz_gplugin SHARED lz_gplugin.cpp)
ADD_DEPENDENCIES(lz_gplugin atl_gazebo_msgs)
TARGET_LINK_LIBRARIES(lz_gplugin ${atl_GAZEBO_DEPS})

ADD_LIBRARY(quadrotor_gplugin SHARED quadrotor_gplugin.cpp)
ADD_DEPENDENCIES(quadrotor_gplugin atl_gazebo_msgs)
TARGET_LINK_LIBRARIES(quadrotor_gplugin ${atl_GAZEBO_DEPS})

# ADD_LIBRARY(world_gplugin SHARED world_gplugin.cpp)
# ADD_DEPENDENCIES(world_gplugin atl_gazebo_msgs)
# TARGET_LINK_LIBRARIES(world_gplugin ${atl_GAZEBO_DEPS})

# INSTALL
# INSTALL(
#   TARGETS
#   box_gplugin
#   camera_gplugin
#   df_camera_gplugin
#   gimbal_gplugin
#   lz_gplugin
#   quadrotor_gplugin
#   # world_gplugin
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# # INSTALL LAUNCH FILES
# SET(SHARE_DEST ${CATKIN_PACKAGE_SHARE_DESTINATION})
# INSTALL(DIRECTORY launch/ DESTINATION ${SHARE_DEST}/launch)
