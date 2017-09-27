project(calib)
set(proj_deps ${OpenCV_LIBS})

set(proj_hdrs
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${PCL_INCLUDE_DIRS}
    )
set(proj_ldrs
)
file(GLOB proj_incs ${CMAKE_CURRENT_LIST_DIR}/src/*.hpp )
file(GLOB proj_srcs ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp )

add_lib_target(${PROJECT_NAME} "SHARED" "${proj_srcs}" "${proj_incs}" "${proj_hdrs}" "${proj_ldrs}" "${proj_deps}")
