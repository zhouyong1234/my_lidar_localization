set(WORK_SPACE_PATH ${PROJECT_SOURCE_DIR})

configure_file(
    ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/common/common.h.in
    ${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/common/common.h
)

include_directories(${PROJECT_BINARY_DIR}/include)