
file(GLOB TEST_PROGS "test*.cpp")

OpenSimAddTests(
    TESTPROGRAMS ${TEST_PROGS}
    LINKLIBS osimSimulation ${ADOLC_LIBRARIES}
    )
