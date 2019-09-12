set(TEST_TARGET TrackSim_s1_1a_6cs_mtp)
add_executable(${TEST_TARGET} ${TEST_TARGET}.cpp)
target_link_libraries(${TEST_TARGET} osimSimulation)
set_target_properties(${TEST_TARGET} PROPERTIES
    FOLDER "OCPs")
		