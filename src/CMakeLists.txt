set(SOURCES socket_client.cpp flight_forge_connector.cpp game_mode_controller.cpp)

add_library(${LIBRARY_NAME} OBJECT ${SOURCES})
target_include_directories(${LIBRARY_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)