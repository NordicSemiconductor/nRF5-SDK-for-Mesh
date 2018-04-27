#! find_dependency : Resolves external dependencies.
#
# This function will try to resolve external dependencies, e.g., the Unity
# framework path.
#
# \arg:var          Variable to store dependency in.
# \arg:description  Variable description.
# \arg:default_path Default path to check for the dependency.
# \arg:file_check   File that should be found in the default path.
#
macro (find_dependency var description default_path file_check)
    if (NOT ${var})
        if (DEFINED ENV{${var}})
            file(TO_CMAKE_PATH "$ENV{${var}}" ${var})
            set(var_source "set with system ENVIRONMENT")
        else ()
            set(${var} ${default_path})
            set(var_source "set with default PATH")
        endif ()
    else()
        set(var_source "set with command line ARG")
    endif ()

    set(${var} ${${var}} CACHE PATH ${description})

    if (EXISTS "${${var}}/${file_check}")
        # Convert to absolute path
        if (NOT IS_ABSOLUTE ${${var}})
            set(${var} "${CMAKE_CURRENT_BINARY_DIR}/${${var}}")
        endif ()

        message(STATUS "${var}=${${var}} --- ${var_source}")
    else ()
        set(${var} ${var}-NOTFOUND)
    endif()
endmacro()
