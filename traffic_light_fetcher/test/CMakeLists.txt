find_package(GTest REQUIRED)

set(TEST_NAME simple_traffic_light_detector_test)

file(GLOB TEST_SRC CONFIGURE_DEPENDS *.cpp)

add_executable(${TEST_NAME}
  ${TEST_SRC}
)

target_link_libraries(${TEST_NAME} PRIVATE
  traffic_light_detector
  GTest::Main
)

gtest_discover_tests(${TEST_NAME})