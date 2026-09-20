#   serialization_roundtrip - wire structs through cereal, no I/O
#   unit_tests              - pure connector logic, no I/O
#   sim_integration         - full API against a live simulator; exits 77

add_executable(serialization_roundtrip serialization_roundtrip.cpp)
target_include_directories(serialization_roundtrip PRIVATE ${PROJECT_SOURCE_DIR}/include)
add_test(NAME serialization_roundtrip COMMAND serialization_roundtrip)

add_executable(unit_tests unit_tests.cpp)
target_include_directories(unit_tests PRIVATE ${PROJECT_SOURCE_DIR}/include)
add_test(NAME unit_tests COMMAND unit_tests)

add_executable(sim_integration sim_integration.cpp)
target_link_libraries(sim_integration PRIVATE ${LIBRARY_NAME})
add_test(NAME sim_integration COMMAND sim_integration)
set_tests_properties(sim_integration PROPERTIES SKIP_RETURN_CODE 77)

set(PLUGIN_HEADERS ${PROJECT_SOURCE_DIR}/../FlightForgePlugin/Source/MessageSerialization/Public)
foreach (HEADER serializable_shared.h serializable_schema.h)
    if (EXISTS ${PLUGIN_HEADERS}/${HEADER})
        add_test(NAME header_mirror_${HEADER}
                 COMMAND ${CMAKE_COMMAND} -E compare_files
                         ${PLUGIN_HEADERS}/${HEADER}
                         ${PROJECT_SOURCE_DIR}/include/flight_forge_connector/serialization/${HEADER})
    endif()
endforeach()
