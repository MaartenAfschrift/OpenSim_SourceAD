set(TEST_TARGET TrackSim_subject1_exe_2a_6cs)
add_executable(${TEST_TARGET} ${TEST_TARGET}.cpp)
target_link_libraries(${TEST_TARGET} osimSimulation)
set_target_properties(${TEST_TARGET} PROPERTIES
    FOLDER "OCPs")
		