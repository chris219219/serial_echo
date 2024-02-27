cmake_minimum_required(VERSION 3.0)
project(SERIALECHO)

add_compile_options(-Og -pipe)

add_executable(SERIALECHOEX ${CMAKE_CURRENT_SOURCE_DIR}/serialecho.c)
target_link_libraries(SERIALECHOEX -static-libgcc -static-libstdc++)
target_include_directories(SERIALECHOEX BEFORE PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/inc)
set_target_properties(SERIALECHOEX PROPERTIES OUTPUT_NAME "serialecho")

add_executable(SERIALSENDEX ${CMAKE_CURRENT_SOURCE_DIR}/serialsend.c)
target_link_libraries(SERIALSENDEX -static-libgcc -static-libstdc++)
target_include_directories(SERIALSENDEX BEFORE PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/inc)
set_target_properties(SERIALSENDEX PROPERTIES OUTPUT_NAME "serialsend")