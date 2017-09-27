
macro(debug_message msg)
    message(STATUS "${msg}")
endmacro()

macro(add_exe_target target srcs incs hdrs deps)
    include_directories( ${hdrs} )
    #debug_message("${PROJECT_NAME}: ${hdrs}")
    add_executable(${target} ${srcs} ${incs})
    link_directories( ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} )
    target_link_libraries(${target} ${deps})
endmacro()

# type: STATIC | SHARED | MODULE
macro(add_lib_target target type srcs incs hdrs ldrs deps)
    include_directories( ${hdrs} )
    #debug_message("${PROJECT_NAME}: ${deps}")
    add_library(${target} ${type} ${srcs} ${incs})
    list(APPEND ldrs ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
    link_directories( ${ldrs} )
    target_link_libraries(${target} ${deps})
endmacro()

macro(add_module module_name_0)
    debug_message("add_module(" ${module_name_0} ${ARGN} ")")
    string(TOLOWER "${module_name}" module_name_0)
    #set(the_module "project_${module_name}")
    set(the_module "${PROJECT_NAME}-${module_name}")
endmacro()

macro(glob_module_sources)
  debug_message("glob_module_sources(" ${ARGN} ")")
  set(_argn ${ARGN})

  file(GLOB_RECURSE lib_srcs
       "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp"
  )
  file(GLOB lib_incs
       "${CMAKE_CURRENT_LIST_DIR}/include/opencv2/*.hpp"
       "${CMAKE_CURRENT_LIST_DIR}/include/opencv2/${name}/*.hpp"
       "${CMAKE_CURRENT_LIST_DIR}/include/opencv2/${name}/*.h"
       "${CMAKE_CURRENT_LIST_DIR}/include/opencv2/${name}/hal/*.hpp"
       "${CMAKE_CURRENT_LIST_DIR}/include/opencv2/${name}/hal/*.h"
  )
endmacro()

macro(define_module module_name)
    debug_message("define_module(" ${module_name} ${ARGN} ")")
    set(_argn ${ARGN})
    set(exclude_cuda "")
    foreach(arg ${_argn})
        if("${arg}" STREQUAL "EXCLUDE_CUDA")
            set(exclude_cuda "${arg}")
            list(REMOVE_ITEM _argn ${arg})
        endif()
    endforeach()

    add_module(${module_name} ${_argn})
    glob_module_sources()
    module_include_directories()
    cv_create_module()

endmacro()
