# Enable code coverage

# Generating coverage information is only useful for host builds using GCC:
if (BUILD_HOST AND TOOLCHAIN MATCHES "gcc")
    option(ENABLE_COVERAGE "Enable generating code coverage information" ON)

    if (ENABLE_COVERAGE)
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
    endif ()
endif ()

# Coverage reporting is currently only supported for Unix hosts:
if (ENABLE_COVERAGE)
    option(ENABLE_COVERAGE_REPORT "Enable generating coverage reports" ON)

    if (ENABLE_COVERAGE_REPORT)
        find_program(GCOV_EXECUTABLE    gcov)
        if (CMAKE_HOST_UNIX)
            find_program(LCOV_EXECUTABLE    lcov)
            find_program(GENHTML_EXECUTABLE genhtml)
            set(USER_EXCLUDE_DIRS '/usr/*')
        else ()
            get_filename_component(lcov_path ${PROJECT_SOURCE_DIR}/scripts/lcov REALPATH)
            set(LCOV_EXECUTABLE    perl ${lcov_path}/lcov.perl)
            set(GENHTML_EXECUTABLE perl ${lcov_path}/genhtml.perl)
            set(USER_EXCLUDE_DIRS 'C:/MinGW/*')
        endif ()

        if (NOT GCOV_EXECUTABLE OR NOT LCOV_EXECUTABLE OR NOT GENHTML_EXECUTABLE)
            message(WARNING "Coverage reporting was requested but not all requirements (gcov, lcov, genhtml) were found")
            set(ENABLE_COVERAGE_REPORT OFF)
        else ()

            # Add a 'coverage' target for gathering code coverage information:
            separate_arguments(test_command UNIX_COMMAND "ctest -Q || true")
            set(coverage_info_filename coverage.info)
            set(coverage_info_cleaned_filename ${coverage_info_filename}.cleaned)
            get_filename_component(cmock_path ${CMOCK_ROOT} REALPATH)
            get_filename_component(unity_path ${UNITY_ROOT} REALPATH)
            get_filename_component(mock_path ${CMOCK_BIN} REALPATH)

            add_custom_target(coverage
                ${LCOV_EXECUTABLE} --directory . --zerocounters --quiet
                COMMAND ${test_command} # Run tests, ignore failures
                COMMAND ${LCOV_EXECUTABLE} --directory . --capture --output-file ${coverage_info_filename} --quiet
                COMMAND ${LCOV_EXECUTABLE} --remove ${coverage_info_filename} '${mock_path}/*' 'test/*' '${unity_path}/*' '${cmock_path}/*' ${USER_EXCLUDE_DIRS} --output-file ${coverage_info_cleaned_filename} --quiet
                COMMAND ${GENHTML_EXECUTABLE} -o coverage ${coverage_info_cleaned_filename} --quiet
                COMMAND ${CMAKE_COMMAND} -E remove ${coverage_info_cleaned_filename}
                WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
                COMMENT "Generating code coverage information..."
            )

            add_custom_command(TARGET coverage POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E echo \"Open ${CMAKE_BINARY_DIR}/coverage/index.html in your browser to view the coverage report\"
            )
        endif ()
    endif ()
endif ()

