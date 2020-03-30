find_program(PC_LINT_EXECUTABLE
    lint-nt.exe
    PATHS "C:/lint" ENV PATH)

if (PC_LINT_EXECUTABLE)
    set(PC_LINT_FLAGS "-b" CACHE STRING "Additional PC-Lint command line options")
    set(PC_LINT_SETTINGS_FILE "${CMAKE_CONFIG_DIR}/mesh.lnt" CACHE FILEPATH "PC-Lint configuration file")
    add_custom_target(lint)
    function (add_pc_lint target sources include_dirs defines)
        file(TO_NATIVE_PATH "${PC_LINT_SETTINGS_FILE}" __filedata)

        foreach (include IN LISTS include_dirs)
            file(TO_NATIVE_PATH "${include}" include)
            set(__filedata "${__filedata}\n-i\"${include}\"")
        endforeach ()

        string(REPLACE "-D" "-d" defines "${defines}")
        foreach (define IN LISTS defines)
            string(FIND "${define}" "-d" def_pos)
            if(def_pos GREATER_EQUAL 0)
                set(__filedata "${__filedata}\n${define}")
            else(def_pos GREATER_EQUAL 0)
                set(__filedata "${__filedata}\n-d${define}")
            endif(def_pos GREATER_EQUAL 0)
        endforeach ()

        foreach (source IN LISTS sources)
            file(TO_NATIVE_PATH "${source}" source)
            set(__filedata "${__filedata}\n${source}")
        endforeach ()

        file(WRITE "${CMAKE_BINARY_DIR}/${target}.lnt" ${__filedata})

        file(TO_NATIVE_PATH "${CMAKE_BINARY_DIR}/${target}.lnt" __target_settings_file)
        add_custom_target(${target}_lint
            COMMAND ${PC_LINT_EXECUTABLE} ${__target_settings_file} -u "${PC_LINT_FLAGS}")
        add_dependencies(lint ${target}_lint)
    endfunction ()
else ()
    message(STATUS "PC-Lint executable not found. Linting disabled.")
    function (add_pc_lint target sources include_dirs defines)
    endfunction ()
endif ()
