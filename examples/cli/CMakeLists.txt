add_executable(debug_cli debug-cli.cpp)
target_link_libraries(debug_cli PRIVATE ${LIBRARY_NAME})

add_executable(debug_game_mode_cli debug_game_mode_cli.cpp)
target_link_libraries(debug_game_mode_cli PRIVATE ${LIBRARY_NAME})

# offline ground truth: world mesh as PLY, exact distances for a batch of points
add_executable(extract_gt extract_gt.cpp)
target_link_libraries(extract_gt PRIVATE ${LIBRARY_NAME})

# one frame from every camera, written out as PNGs for eyeballing
add_executable(capture_all capture_all.cpp)
target_link_libraries(capture_all PRIVATE ${LIBRARY_NAME} fpng)

# per-sensor timing through the C++ API, which is the path real consumers take
add_executable(profile_sensors profile_sensors.cpp)
target_link_libraries(profile_sensors PRIVATE ${LIBRARY_NAME})
