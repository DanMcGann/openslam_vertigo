# Extra Boost Libraries needed for program options
FIND_PACKAGE(Boost COMPONENTS program_options REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

add_executable(robustISAM2 robustISAM2.cpp)
target_link_libraries(robustISAM2 vertigo gtsam ${Boost_LIBRARIES})
