# Library for writing multithreaded tests, used to test for IRQ-safety problems.
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads)

if(CMAKE_USE_PTHREADS_INIT)
    add_library(mttest STATIC mttest.c)
    target_link_libraries(mttest ${CMAKE_THREAD_LIBS_INIT})
    target_include_directories(mttest PUBLIC "." ${${SOFTDEVICE}_INCLUDE_DIRS} ${${PLATFORM}_INCLUDE_DIRS})

    # Adds a multi-threaded unit test, with automatic test runner generation.
    function(add_mtt_test NAME SOURCES INCLUDE_DIRS COMPILE_OPTIONS)
        add_executable(ut_${NAME} ${SOURCES})
        target_compile_options(ut_${NAME} PUBLIC
            ${COMPILE_OPTIONS})

        target_include_directories(ut_${NAME} PUBLIC
            ${INCLUDE_DIRS})

        # Link to the pthread library explicitly, since CMake doesn't do this automatically on all platforms:
        target_link_libraries(ut_${NAME} PUBLIC mttest ${CMAKE_THREAD_LIBS_INIT} pthread)
        add_test(${NAME} ut_${NAME})
    endfunction(add_mtt_test)
else ()
    message("Warning: Multi-threaded test not supported.")
    function(add_mtt_test NAME SOURCES INCLUDE_DIRS COMPILE_OPTIONS)
        message("Warning: Multi-threading test \"${NAME}\" not supported")
    endfunction(add_mtt_test)
endif()
