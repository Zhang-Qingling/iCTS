if(DEBUG_ICTS_SCG)
  message(STATUS "CTS: DEBUG_ICTS_SCG")
  set(CMAKE_BUILD_TYPE "Debug")
else()
  message(STATUS "CTS: RELEASE_ICTS_SCG")
  set(CMAKE_BUILD_TYPE "Release")
endif()

add_library(icts_SCG
  ${CMAKE_CURRENT_LIST_DIR}/SCG_builder.cc
)

target_include_directories(icts_SCG PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}
  ${ICTS_SOLVER}/database
  ${ICTS_SOLVER}/tools/timing_propagator
)

target_link_libraries(icts_SCG PUBLIC
  icts_solver_database
  icts_timing_propagator
  icts_api
)
