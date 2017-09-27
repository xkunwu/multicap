project(test)
set(proj_deps calib reconstruction freenect2 ${OpenCV_LIBS} ${Boost_LIBRARIES} glfw ${OPENGL_glu_LIBRARY} ${GLFW_LIBRARIES} ${OPENGL_gl_LIBRARY})

#set(CALIB_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include)
set(proj_hdrs
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${LIBFREENECT2_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${CALIB_INCLUDE_DIRS}
    ${RECON_INCLUDE_DIRS}
)
file(GLOB proj_incs ${CMAKE_CURRENT_LIST_DIR}/src/*.hpp )
file(GLOB proj_srcs ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp )

#file(GLOB_RECURSE cpp_files RELATIVE ${CMAKE_CURRENT_LIST_DIR} *.cpp)
file(GLOB cpp_files ${CMAKE_CURRENT_LIST_DIR}/*.cpp)
foreach(cpp_f ${cpp_files})
    get_filename_component(cpp_fwe ${cpp_f} NAME_WE)
    set(the_target "${PROJECT_NAME}-${cpp_fwe}")
    add_exe_target(${the_target} "${proj_srcs};${cpp_f}" "${proj_incs}" "${proj_hdrs}" "${proj_deps}")
endforeach()
