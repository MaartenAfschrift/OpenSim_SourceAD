set(TEST_TARGET PredSim_s1_6cs_const_cm4)
add_executable(${TEST_TARGET} ${TEST_TARGET}.cpp)
target_link_libraries(${TEST_TARGET} osimSimulation)
set_target_properties(${TEST_TARGET} PROPERTIES
    FOLDER "OCPs")
		