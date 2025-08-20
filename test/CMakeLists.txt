if (DEBUG_ICTS_TEST)
  message(STATUS "CTS: DEBUG_ICTS_TEST")
else()
  message(STATUS "CTS: RELEASE_ICTS_TEST")
endif()

find_package(GTest REQUIRED)

if (PY_MODEL)
  add_executable(icts_py_test ${ICTS_TEST}/PyTest.cc)
  target_link_libraries(icts_py_test
    PRIVATE
      icts_source
      icts_test_external_libs
      GTest::gtest
      GTest::gtest_main
  )
endif()

add_executable(icts_anneal_opt_test ${ICTS_TEST}/AnnealOptTest.cc)

target_link_libraries(icts_anneal_opt_test
  PRIVATE
    icts_source
    icts_api
    icts_test_external_libs
    GTest::gtest
    GTest::gtest_main
    -u_ZN3ipl17SteinerWirelength25updateAllNetWorkPointPairEv
    -u_ZN3ipl13NesterovPlace21printNesterovDatabaseEv
    -u_ZN3ito3iTO5runTOEv
    -u_ZN3idm11DataManager7saveDefENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE
    -u_ZN4icts9GDSPloter10plotDesignERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE
)

target_include_directories(icts_anneal_opt_test PUBLIC ${ICTS_TEST})
