cmake_minimum_required(VERSION 2.8.3)
project(grasp_planning_graspit)

# Set BUILD_GRASPIT to true if you would like to 
# downloads (git clone) and build the fork of GraspIt! which
# is required for this package (https://github.com/JenniferBuehler/graspit).
# If BUILD_GRASPIT is set to false, it will search for the package
# "graspit' instead.
# If set to true, you can also use GRASPIT_BRANCH_NAME to select a git branch.
# IMPORTANT: If you set this to false and are compiling
# with catkin, uncomment the <*_depend> tags relating to
# graspit in package.xml!
if (NOT DEFINED BUILD_GRASPIT)
    set (BUILD_GRASPIT true)
endif (NOT DEFINED BUILD_GRASPIT)

###################################################
## Define cmake variables to build with/without catkin 
###################################################


# By default, compiling with catkin is
# enabled. It will only be disabled again
# if CATKIN_DEVEL_PREFIX is not set.
# To enforce compiling *with* catkin
# even if CATKIN_DEVEL_PREFIX is not defined,
# compile with the flag -DENFORCE_CATKIN=true
# This is for example required when compiling on jenkins,
# when it seems that CATKIN_DEVEL_PREFIX is not set.
# So set ENFORCE_CATKIN to true only for the commits
# to jenkins.
set (ENFORCE_CATKIN false)
   
# Private flag: compiling with catkin is prioritized by default.
# This flag will be set to false if CATKIN_DEVEL_PREFIX
# is empty/not defined, **and** ENFORCE_CATKIN is false.
set (COMPILE_WITH_CATKIN true)

if (NOT ENFORCE_CATKIN AND NOT CATKIN_DEVEL_PREFIX)
    message(STATUS "Compiling grasp_planning_graspit: CATKIN_DEVEL_PREFIX is not defined, so **enforcing compilation with cmake**.")
    set (COMPILE_WITH_CATKIN false)
endif (NOT ENFORCE_CATKIN AND NOT CATKIN_DEVEL_PREFIX)

if (COMPILE_WITH_CATKIN)
    if (NOT BUILD_GRASPIT)
        find_package(catkin REQUIRED COMPONENTS eigen_conversions roslint graspit) 
    else (NOT BUILD_GRASPIT)
        find_package(catkin REQUIRED COMPONENTS eigen_conversions roslint) 
    endif (NOT BUILD_GRASPIT)
endif (COMPILE_WITH_CATKIN)
   
# Configuration file for graspit
set (GRASP_PLANNNIG_GRASPIT_CONFIG_CMAKE grasp_planning_graspitConfig.cmake)

# CMake module path
set (CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeMacros/")

# if catkin is not used, or graspit built locally,
# we have to get the Eigen/graspit dependencies separately
if (NOT COMPILE_WITH_CATKIN OR BUILD_GRASPIT)
    if (NOT BUILD_GRASPIT)
        message(STATUS "Finding graspit...")
        find_package(graspit REQUIRED)
    endif (NOT BUILD_GRASPIT)
    find_package(Eigen3 REQUIRED)
endif (NOT COMPILE_WITH_CATKIN OR BUILD_GRASPIT)

find_package(Boost REQUIRED COMPONENTS filesystem system thread program_options)

# add a target to generate API documentation with Doxygen
find_package(Doxygen)
if(DOXYGEN_FOUND)
    # can check with if (TARGET doc) whether the target 'doc' was already used for another package.
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    add_custom_target(grasp_planning_graspit_doc
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen" VERBATIM
    )
endif(DOXYGEN_FOUND)
   
# Target dependencies to graspit: defaults to whole
# graspit package and can be overwritten if graspit is build locally.
set(GRASPIT_TARGET_DEPS graspit)

# if the graspit fork should be build with this package, pull graspit 
# from external and build locally without installing
if (BUILD_GRASPIT)
    if (NOT DEFINED GRASPIT_BRANCH_NAME)
        set (GRASPIT_BRANCH_NAME master)
    endif (NOT DEFINED GRASPIT_BRANCH_NAME)
    message(STATUS "Building graspit library, will clone git repository. Branch name: ${GRASPIT_BRANCH_NAME}")

    ####
    # Download and unpack graspit at configure time.
    # The advantage of doing this at configure time, and then build
    # the graspit source as sub-package, is that the requierd variables
    # (include/link directories and libraries) can be used. Only
    # adding graspit as external package will not allow us to access the
    # config files (as for example resulting from find_package())
    ####

    # set the environment variables for the graspit_external/CMakeLists-external-graspit.txt
    set (GRASPIT_REPO_NAME graspit)
    set (GRASPIT_GIT "https://github.com/belledon/graspit")
    set (GRASPIT_BRANCH_NAME "master")
    set (GRASPIT_DL_DIR ${CMAKE_CURRENT_BINARY_DIR}/graspit_external)
    set (GRASPIT_INSTALL_DIR ${GRASPIT_DL_DIR}/install)
    set (GRASPIT_BUILD_DIR ${GRASPIT_DL_DIR}/graspit-build)
    
    # message("Using binary dir ${CMAKE_CURRENT_BINARY_DIR}")

    # Copy the external cmake file to the build directory for graspit.
    # first generate the cmakelists: this copies from the source directory to the build directory
    configure_file(graspit_external/CMakeLists-external-graspit.txt ${GRASPIT_DL_DIR}/CMakeLists.txt)
    # This clones the repository and runs the cmake configure step.
    execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
        WORKING_DIRECTORY ${GRASPIT_DL_DIR})
    execute_process(COMMAND ${CMAKE_COMMAND} --build .
        WORKING_DIRECTORY ${GRASPIT_DL_DIR})
  
    # convenience variable: this is where all the graspit source is after download step. 
    set (GRASPIT_SRC_DIR ${GRASPIT_DL_DIR}/repo/${GRASPIT_REPO_NAME})

    # Back up variables and change them for add_subdirectory().
    # It would be nice to completely build graspit entirely by
    # an external project, but in order to avoid building
    # at configure time, and still have access to cmake
    # variables as graspit_LIBRARIES, this solution was found best.
    set (_CMAKE_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX}) 
    set (_CATKIN_DEVEL_PREFIX ${CATKIN_DEVEL_PREFIX})
    set (_BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS})
    set (_ENFORCE_CATKIN ${ENFORCE_CATKIN})
    set (_LIBRARY_OUTPUT_PATH ${LIBRARY_OUTPUT_PATH})
    set (_EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH})
    # change the values for the subdirectory
    set (CMAKE_INSTALL_PREFIX ${GRASPIT_INSTALL_DIR})
    set (CATKIN_DEVEL_PREFIX false)
    # output paths need to be set as well to prevent installation into catkin devel space
    set (LIBRARY_OUTPUT_PATH ${GRASPIT_BUILD_DIR})
    set (EXECUTABLE_OUTPUT_PATH ${GRASPIT_BUILD_DIR})
    set (BUILD_SHARED_LIBS false) # we will compile the code into the grasp_planning_graspit library
    set (BUILD_TESTS false)
    set (ENFORCE_CATKIN false)

    # Add the external graspit build directory to our build, and set the build directory.
    add_subdirectory(${GRASPIT_SRC_DIR}
        ${GRASPIT_BUILD_DIR} EXCLUDE_FROM_ALL)
   
    # restore old variables 
    set (CMAKE_INSTALL_PREFIX ${_CMAKE_INSTALL_PREFIX}) 
    set (CATKIN_DEVEL_PREFIX ${_CATKIN_DEVEL_PREFIX})
    set (BUILD_SHARED_LIBS ${_BUILD_SHARED_LIBS})
    set (LIBRARY_OUTPUT_PATH ${_LIBRARY_OUTPUT_PATH})
    set (EXECUTABLE_OUTPUT_PATH ${_EXECUTABLE_OUTPUT_PATH})
    set (ENFORCE_CATKIN ${_ENFORCE_CATKIN})
    
    # get the relevant cmake variables from the subdirectory
    get_directory_property(graspit_INCLUDE_DIRS
            DIRECTORY ${GRASPIT_SRC_DIR} 
            DEFINITION graspit_INCLUDE_DIRS)
    get_directory_property(graspit_CXXFLAGS
            DIRECTORY ${GRASPIT_SRC_DIR} 
            DEFINITION graspit_CXXFLAGS)
    get_directory_property(graspit_LINK_DIRS
            DIRECTORY ${GRASPIT_SRC_DIR} 
            DEFINITION graspit_LINK_DIRS)
    get_directory_property(graspit_LIBRARIES
            DIRECTORY ${GRASPIT_SRC_DIR} 
            DEFINITION graspit_LIBRARIES)
    get_directory_property(graspit_SYS_INCLUDES
            DIRECTORY ${GRASPIT_SRC_DIR} 
            DEFINITION graspit_SYS_INCLUDES)
    get_directory_property(graspit_SYS_LIBRARIES
            DIRECTORY ${GRASPIT_SRC_DIR} 
            DEFINITION graspit_SYS_LIBRARIES)

   # overwrite the target dependencies to include the external target 
   set(GRASPIT_TARGET_DEPS graspit_ext graspit graspit_simulator)
endif (BUILD_GRASPIT)

# Find Qt package separately because we'll need it for the wrapper,
# regardless whether it is required by graspit or not
SET( QT_USE_QT3SUPPORT TRUE )
find_package(Qt4 COMPONENTS QtCore REQUIRED)
include (${QT_USE_FILE})

# Qt MOC files are needed ONLY because QT slots are used at the moment in EigenGraspPlanner.h
# Qt include files should be covered by the implicit dependency of graspit libraries
# if compiled with catkin
set (MOC_HEADERS include/grasp_planning_graspit/EigenGraspPlanner.h) 
QT4_WRAP_CPP(MOC_OUTFILES ${MOC_HEADERS})
    
###############################
# Build catkin_package() command
###############################
if (COMPILE_WITH_CATKIN)
    # INCLUDE_DIRS for catkin_package() command
    set(CATKIN_DEPEND_INCLUDE_DIRS
        include
        ${graspit_SYS_INCLUDES}
    )    
    
    # CATKIN_DEPENDS for catkin_package() command
    set (CATKIN_DEPEND_PKGS eigen_conversions)
    if (NOT BUILD_GRASPIT)
        set (CATKIN_DEPEND_PKGS ${CATKIN_DEPEND_PKGS} graspit)
    endif (NOT BUILD_GRASPIT)

    # DEPENDS for catkin_package()
    set(CATKIN_DEPENDS_LIBS Boost SoQt4 Coin Qt4 LAPACK Qhull)

    ###################################
    ## catkin specific configuration ##
    ###################################
    ## The catkin_package macro generates cmake config files for your package
    ## Declare things to be passed to dependent projects
    ## INCLUDE_DIRS: uncomment this if you package contains header files
    ## LIBRARIES: libraries you create in this project that dependent projects also need
    ## CATKIN_DEPENDS: catkin_packages dependent projects also need
    ## DEPENDS: system dependencies of this project that dependent projects also need
    catkin_package(
      INCLUDE_DIRS ${CATKIN_DEPEND_INCLUDE_DIRS}
      LIBRARIES grasp_planning_graspit ${graspit_SYS_LIBRARIES}  # need to add graspit_SYS_LIBRARIES here for bloom binary builds
      CATKIN_DEPENDS ${CATKIN_DEPEND_PKGS} 
      DEPENDS ${CATKIN_DEPENDS_LIBS}
    )
endif (COMPILE_WITH_CATKIN)


###########
## Build ##
###########

# add compiler definitions
add_definitions(${graspit_CXXFLAGS})

# include all directories, inc. catkin related - if any is empty,
# it doesn't matter
set(ALL_INCLUDE_DIRECTORIES
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${graspit_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}

)

add_definitions(
    ${graspit_CXXFLAGS}
    # -DGRASPITDBG
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${ALL_INCLUDE_DIRECTORIES}
)
INCLUDE_DIRECTORIES("${Boost_INCLUDE_DIRS}" "/usr/include/python3.5")
set (INCDIR include/${PROJECT_NAME})
set(MAIN_HEADER_FILES
    ${INCDIR}/GraspItSceneManager.h
    ${INCDIR}/GraspItSceneManagerHeadless.h
    ${INCDIR}/GraspItAccessor.h
    ${INCDIR}/GraspItDatabaseManager.h
    ${INCDIR}/GraspItSimpleDBManager.h
    ${INCDIR}/EigenGraspPlanner.h
    ${INCDIR}/ContactGetter.h
    #    ${INCDIR}/EigenGraspPlannerNoQt.h
    ${INCDIR}/EigenGraspResult.h
    ${INCDIR}/GraspItTypes.h
    ${INCDIR}/SharedPtr.h
    ${INCDIR}/ThreadImpl.h
    ${INCDIR}/PrintHelpers.h
    ${INCDIR}/LogBinding.h
)

set(MAIN_SOURCE_FILES 
    ${MOC_OUTFILES} 
    src/GraspItSceneManager.cpp 
    src/GraspItSceneManagerHeadless.cpp 
    src/GraspItAccessor.cpp 
    src/EigenGraspPlanner.cpp 
    src/ContactGetter.cpp
    #    src/EigenGraspPlannerNoQt.cpp 
    src/EigenGraspResult.cpp 
    src/LogBinding.cpp 
    src/PrintHelpers.cpp 
    src/GraspItHelpers.cpp
    src/GraspItDatabaseManager.cpp
    src/GraspItSimpleDBManager.cpp
)

## Declare a C++ library
add_library(grasp_planning_graspit SHARED
 ${MAIN_SOURCE_FILES} 
)

## Add cmake target dependencies of the library
if (COMPILE_WITH_CATKIN)
    add_dependencies(grasp_planning_graspit ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif (COMPILE_WITH_CATKIN)

if (BUILD_GRASPIT)
    add_dependencies(grasp_planning_graspit ${GRASPIT_TARGET_DEPS})
endif (BUILD_GRASPIT)

## Declare C++ executables
add_executable(grasp_planning src/grasp_planning.cpp)
add_executable(om_grasp_planning om/om_grasp_planning.cpp)

add_executable(simple_graspit_database test/simple_graspit_database.cpp)
add_executable(simple_planning_tutorial tutorial/src/simple_planning_tutorial.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
if (COMPILE_WITH_CATKIN)
    add_dependencies(grasp_planning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    add_dependencies(simple_graspit_database ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    add_dependencies(simple_planning_tutorial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif (COMPILE_WITH_CATKIN)

# Add depending libraries if they don't come with catkin dependencies already.
# At least one of the variables must be non-empty at this point.
set (DEPEND_LIBRARIES ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${graspit_LIBRARIES} ${DEPEND_LIBRARIES})

## Specify libraries to link a library or executable target against

target_link_libraries(grasp_planning_graspit
  ${DEPEND_LIBRARIES}
)

target_link_libraries(grasp_planning
  grasp_planning_graspit
  ${DEPEND_LIBRARIES}
)

target_link_libraries(om_grasp_planning
    grasp_planning_graspit
    ${DEPEND_LIBRARIES}
)


target_link_libraries(simple_planning_tutorial
  grasp_planning_graspit
  ${DEPEND_LIBRARIES}
)

target_link_libraries(simple_graspit_database
  grasp_planning_graspit
  ${DEPEND_LIBRARIES}
)

# style checking
if (COMPILE_WITH_CATKIN)
    roslint_cpp()
endif (COMPILE_WITH_CATKIN)

#############
## Install ##
#############

if (COMPILE_WITH_CATKIN) 
    # all install targets should use catkin DESTINATION variables
    # See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html
    set ( INCLUDE_DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
    set ( LIB_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    set ( BIN_DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    set ( SHARE_DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
else (COMPILE_WITH_CATKIN) 
    set ( INCLUDE_DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/)
    set ( LIB_DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/)
    set ( BIN_DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/)
    set ( SHARE_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/)
endif (COMPILE_WITH_CATKIN) 

install(TARGETS grasp_planning_graspit
   EXPORT grasp_planning_graspit-targets
   ARCHIVE DESTINATION ${LIB_DESTINATION}
   LIBRARY DESTINATION ${LIB_DESTINATION}
   RUNTIME DESTINATION ${LIB_DESTINATION}
)

install(TARGETS grasp_planning
   EXPORT grasp_planning-targets
   ARCHIVE DESTINATION ${BIN_DESTINATION}
   LIBRARY DESTINATION ${BIN_DESTINATION}
   RUNTIME DESTINATION ${BIN_DESTINATION}
)

install(TARGETS om_grasp_planning
    EXPORT om_grasp_planning-targets
    ARCHIVE DESTINATION ${BIN_DESTINATION}
    LIBRARY DESTINATION ${BIN_DESTINATION}
    RUNTIME DESTINATION ${BIN_DESTINATION}
)

install(EXPORT grasp_planning_graspit-targets
    DESTINATION ${LIB_DESTINATION}/${PROJECT_NAME}
)
install(EXPORT grasp_planning-targets
    DESTINATION ${LIB_DESTINATION}/${PROJECT_NAME}
)
install(EXPORT om_grasp_planning-targets
    DESTINATION ${LIB_DESTINATION}/${PROJECT_NAME}
)
# install(EXPORT autograsp_planning-targets
#     DESTINATION ${LIB_DESTINATION}/${PROJECT_NAME}
# )
install(FILES
    cmake/${GRASP_PLANNNIG_GRASPIT_CONFIG_CMAKE}
    DESTINATION ${LIB_DESTINATION}/${PROJECT_NAME}
)

if (BUILD_GRASPIT)
    set (graspit_EXECUTABLE ${GRASPIT_BUILD_DIR}/graspit_simulator)
    if (COMPILE_WITH_CATKIN)
        # make the GraspIt! simulator available also in the catkin devel space
        add_custom_target(copy_simulator_exe ALL 
            COMMAND ${CMAKE_COMMAND} -E copy
            ${graspit_EXECUTABLE}
            ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/${PROJECT_NAME}
            DEPENDS graspit_simulator
        )
    endif (COMPILE_WITH_CATKIN)
    message (STATUS "Will install graspit executable '${graspit_EXECUTABLE}' as ${BIN_DESTINATION}/graspit_simulator_standalone")
    install(PROGRAMS ${graspit_EXECUTABLE} 
       DESTINATION ${BIN_DESTINATION}
       RENAME graspit_simulator_standalone
    )
endif (BUILD_GRASPIT)

install(FILES
    ${MAIN_HEADER_FILES}
    DESTINATION ${INCLUDE_DESTINATION}
)

install(FILES 
    CMakeMacros/FindEigen3.cmake 
    DESTINATION ${LIB_DESTINATION}/${PROJECT_NAME}
)

install(DIRECTORY 
    tutorial 
    DESTINATION ${SHARE_DESTINATION}
)

if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/docs/html)
    install(DIRECTORY 
        ${CMAKE_CURRENT_BINARY_DIR}/docs/html 
        DESTINATION ${SHARE_DESTINATION}/doc
    )
endif (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/docs/html)



add_subdirectory(pybind11)
pybind11_add_module(autograsp_planning om/autograsp_planning.cpp)
target_link_libraries(autograsp_planning PRIVATE grasp_planning_graspit)
install(TARGETS autograsp_planning
    EXPORT autograsp_planning-targets
    ARCHIVE DESTINATION ${LIB_DESTINATION}
    LIBRARY DESTINATION ${LIB_DESTINATION}
    RUNTIME DESTINATION ${LIB_DESTINATION}
)
# target_link_libraries(autograsp_planning PRIVATE ${DEPEND_LIBRARIES})

# target_link_libraries(include/grasp_planning_graspit/LogBinding.h
#     include/grasp_planning_graspit/GraspItSceneManagerHeadless.h
#     include/grasp_planning_graspit/ContactGetter.h)

#############
## Testing ##
#############

if (COMPILE_WITH_CATKIN)
    ## Add gtest based cpp test target and link libraries
    # catkin_add_gtest(${PROJECT_NAME}-test test/test_grasp_planning_graspit.cpp)
    # if(TARGET ${PROJECT_NAME}-test)
    #   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
    # endif()

    ## Add folders to be run by python nosetests
    # catkin_add_nosetests(test)
endif (COMPILE_WITH_CATKIN)