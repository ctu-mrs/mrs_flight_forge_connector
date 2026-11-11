#   serialization_roundtrip - wire structs through cereal, no I/O
#   wire_roundtrip          - the FlatBuffers 1.0 messages built, verified and read back, no I/O
#   unit_tests              - pure connector logic, no I/O
#   sim_integration         - full API against a live simulator; exits 77

add_executable(serialization_roundtrip serialization_roundtrip.cpp)
target_include_directories(serialization_roundtrip PRIVATE ${PROJECT_SOURCE_DIR}/include)
add_test(NAME serialization_roundtrip COMMAND serialization_roundtrip)

add_executable(wire_roundtrip wire_roundtrip.cpp)
target_include_directories(wire_roundtrip PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_compile_definitions(wire_roundtrip PRIVATE FF_WIRE_BFBS="${PROJECT_SOURCE_DIR}/include/flight_forge_connector/wire/flightforge.bfbs")
add_test(NAME wire_roundtrip COMMAND wire_roundtrip)

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

set(PLUGIN_ROOT ${PROJECT_SOURCE_DIR}/..)
if (EXISTS ${PLUGIN_ROOT}/tools/generate_wire.sh)
    add_test(NAME wire_mirror_generated
             COMMAND ${CMAKE_COMMAND} -DA=${PLUGIN_HEADERS}/Generated -DB=${PROJECT_SOURCE_DIR}/include/flight_forge_connector/wire
                     -P ${CMAKE_CURRENT_SOURCE_DIR}/compare_dirs.cmake)
    add_test(NAME wire_mirror_flatbuffers
             COMMAND ${CMAKE_COMMAND} -DA=${PLUGIN_ROOT}/FlightForgePlugin/Source/ThirdParty/include/flatbuffers
                     -DB=${PROJECT_SOURCE_DIR}/include/flatbuffers -P ${CMAKE_CURRENT_SOURCE_DIR}/compare_dirs.cmake)
    # builds the pinned flatc into ~/.cache on first run
    add_test(NAME wire_generated_fresh COMMAND ${PLUGIN_ROOT}/tools/generate_wire.sh)
    set_tests_properties(wire_generated_fresh PROPERTIES ENVIRONMENT "FF_WIRE_CHECK=1;FF_CONNECTOR_DIR=${PROJECT_SOURCE_DIR}" TIMEOUT 900)
endif()
