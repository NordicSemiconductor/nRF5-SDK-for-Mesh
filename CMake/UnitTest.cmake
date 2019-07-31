# Setup unit test build
find_path(CMOCK_ROOT
    "src/cmock.c"
    HINTS "${CMAKE_SOURCE_DIR}/../CMock" ENV CMOCK_ROOT
    DOC "Path to CMock repository"
    )

find_path(UNITY_ROOT
    "src/unity.c"
    HINTS "${CMOCK_ROOT}/vendor/unity" ENV UNITY_ROOT
    DOC "Path to Unity repository"
    )

if (NOT CMOCK_ROOT OR NOT UNITY_ROOT)
    message(FATAL_ERROR "Path(s) to CMock and/or Unity not found. "
        "Looked in:\n"
        "CMock: ${CMOCK_ROOT}\n"
        "Unity: ${UNITY_ROOT}\n"
        "Please call \n\tgit clone https://github.com/ThrowTheSwitch/CMock.git --recursive\n"
        "in the parent directory of ${CMAKE_SOURCE_DIR}.")
endif ()

find_program(RUBY_EXECUTABLE "ruby"
    DOC "Path to ruby executable")

if (NOT RUBY_EXECUTABLE)
    message(FATAL_ERROR "Ruby not found")
endif()

if (NOT CMOCK_BIN)
    set(CMOCK_BIN "${CMAKE_CURRENT_BINARY_DIR}/mocks")
    message(STATUS "CMock binary directory not set. Defaults to ${CMOCK_BIN}.")
endif (NOT CMOCK_BIN)

if (NOT EXISTS ${CMOCK_BIN}/tmp)
    make_directory(${CMOCK_BIN}/tmp)
endif ()

# Adds a base common unit test library
# Extend to these source files (and include directories etc.)
# using the target_sources(unit_test_common ...) command
# specific for your project.
add_library(unit_test_common STATIC
    "${UNITY_ROOT}/src/unity.c"
    "${CMOCK_ROOT}/src/cmock.c")

target_include_directories(unit_test_common PUBLIC
    "${CMOCK_ROOT}/src"
    "${UNITY_ROOT}/src")

# Generate targets for mock files for all headers matched by `INCLUDE_GLOB_EXPRESSION`
# filtered by `EXCLUDE_REGEX`.
# For each header, we make a target outputting ${header}_mock.c and ${header}_mock.h.
# Any target that depends on these files will trigger this target.
function (generate_mock_targets INCLUDE_GLOB_EXPRESSION EXCLUDE_REGEX MOCK_DEPS)
    if (CMOCK_SETTINGS_FILE)
        set(settings_arg "-o${CMOCK_SETTINGS_FILE}")
    else ()
        set(settings_arg "")
        message("WARNING: `CMOCK_SETTINGS_FILE` is not defined. Default CMock configuration is used.")
    endif (CMOCK_SETTINGS_FILE)

    # Find mock candidates
    file(GLOB MOCK_SRC
        ${INCLUDE_GLOB_EXPRESSION})

    foreach(headerpath IN ITEMS ${MOCK_SRC})
        get_filename_component(header ${headerpath} NAME_WE)
        if (${header} MATCHES "${EXCLUDE_REGEX}")
            continue()
        elseif (${headerpath} MATCHES "softdevice")
            add_custom_command(OUTPUT ${CMOCK_BIN}/${header}_mock.c ${CMOCK_BIN}/${header}_mock.h
                COMMAND ${RUBY_EXECUTABLE} ${CMOCK_ROOT}/lib/cmock.rb ${settings_arg} ${CMOCK_BIN}/tmp/${header}.h
                DEPENDS ${CMOCK_BIN}/tmp/${header}.h
                VERBATIM
                COMMENT "Generating SoftDevice mock for ${header}.h...")

            add_custom_command(OUTPUT ${CMOCK_BIN}/tmp/${header}.h
                COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CONFIG_DIR}/svcall2func.py ${headerpath} ${CMOCK_BIN}/tmp/${header}.h
                DEPENDS ${headerpath} ${MOCK_DEPS}
                COMMENT "Preprocessing SoftDevice header")
        else ()
            add_custom_command(OUTPUT ${CMOCK_BIN}/${header}_mock.c ${CMOCK_BIN}/${header}_mock.h
                COMMAND ${RUBY_EXECUTABLE} ${CMOCK_ROOT}/lib/cmock.rb ${settings_arg} ${headerpath}
                DEPENDS ${headerpath} ${MOCK_DEPS}
                VERBATIM
                COMMENT "Generating mock for ${header}.h...")
        endif()
    endforeach(headerpath)
endfunction (generate_mock_targets)

# Add a unit test
# NOTE: This assumes the test is named on the form ut_<NAME>.c
function (add_unit_test NAME SOURCES INCLUDE_DIRS COMPILE_OPTIONS)
    add_executable(ut_${NAME}
        ${CMAKE_CURRENT_BINARY_DIR}/ut_${NAME}_runner.c)

    target_link_libraries(ut_${NAME} PUBLIC unit_test_common)
    target_compile_options(ut_${NAME} PUBLIC ${COMPILE_OPTIONS})
    target_include_directories(ut_${NAME} PUBLIC ${INCLUDE_DIRS})

    foreach (source IN ITEMS ${SOURCES})
        get_filename_component(source ${source} ABSOLUTE)

        if (source MATCHES ut_)

            # Add custom command to generate unit test runner
            add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/ut_${NAME}_runner.c
                COMMAND ${RUBY_EXECUTABLE} ${UNITY_ROOT}/auto/generate_test_runner.rb ${source} ${CMAKE_CURRENT_BINARY_DIR}/ut_${NAME}_runner.c
                COMMENT "Generating runner for ${NAME}"
                DEPENDS ${source})

            target_sources(ut_${NAME} PUBLIC ${source})
        elseif (source MATCHES mock)
            get_filename_component(base_name ${source} NAME_WE)
            if (NOT TARGET ${base_name})
                # If there is no target to generate the mock object yet,
                # let's create it!
                add_library(${base_name} OBJECT ${source})
                target_include_directories(${base_name} PUBLIC ${INCLUDE_DIRS})
                target_compile_options(${base_name} PUBLIC ${COMPILE_OPTIONS})
            endif (NOT TARGET ${base_name})

            target_sources(ut_${NAME} PUBLIC $<TARGET_OBJECTS:${base_name}>)
        else ()
            target_sources(ut_${NAME} PUBLIC ${source})
        endif ()
    endforeach (source IN ITEMS ${SOURCES})
    add_test(${NAME} ut_${NAME})
endfunction (add_unit_test)

enable_testing()
