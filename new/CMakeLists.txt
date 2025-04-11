cmake_minimum_required(VERSION 3.16.3)
project(main)
set(PJT_SRC_DIR ${PROJECT_SOURCE_DIR}/Src)
set(PJT_INC_DIR ${PROJECT_SOURCE_DIR}/Inc)
set(PJT_LIB_DIR ${PROJECT_SOURCE_DIR}/Lib)
set(MOTOR_SRC_DIR ${PROJECT_SOURCE_DIR}/motor/Src)
set(MOTOR_INC_DIR ${PROJECT_SOURCE_DIR}/motor/Inc)
set(EIGEN_INC_DIR ${PROJECT_SOURCE_DIR}/eigen-3.4.0)
aux_source_directory(${PJT_SRC_DIR} SRC_LIST)
aux_source_directory(${MOTOR_SRC_DIR} SRC_LIST)
include_directories(
    ${PJT_INC_DIR} ${MOTOR_INC_DIR} ${EIGEN_INC_DIR}
)
link_directories(${PJT_LIB_DIR})
add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} pthread)
target_link_libraries(${PROJECT_NAME} unitreeMotorSDK_Linux64)

