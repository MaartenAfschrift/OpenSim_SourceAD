set(TEST_TARGET SimExo_2D_Pog_s1_mtp_pp)
add_executable(${TEST_TARGET} ${TEST_TARGET}.cpp)
target_link_libraries(${TEST_TARGET} osimSimulation)
set_target_properties(${TEST_TARGET} PROPERTIES
    FOLDER "External_Functions")
		