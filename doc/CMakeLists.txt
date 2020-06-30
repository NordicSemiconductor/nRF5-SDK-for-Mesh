# Add doc target for generating doxygen documentation
if (DOT_EXECUTABLE)
    set(HAVE_DOT "YES")
else ()
    set(HAVE_DOT "NO")
    message(STATUS "Graphviz Dot not found. Cannot generate dot figures in documentation.")
endif (DOT_EXECUTABLE)

if (MSCGEN_EXECUTABLE)
    set(HAVE_MSCGEN "YES")
else ()
    set(HAVE_MSCGEN "NO")
    message(STATUS "Mscgen not found. Cannot generate message sequence charts in documentation.")
endif (MSCGEN_EXECUTABLE)

string(TIMESTAMP TODAY "%Y-%m-%d")
set(PROJECT_DOC_NAME "nRF5 SDK for Mesh v${VERSION_STRING}")

if (EXISTS "${CMAKE_SOURCE_DIR}/.git")
    # Get latest commit hash for the working branch
    execute_process(
        COMMAND git rev-parse --short HEAD
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_HEAD_COMMIT_ID
        OUTPUT_STRIP_TRAILING_WHITESPACE)
else ()
    set(GIT_HEAD_COMMIT_ID "")
endif ()

set(CURRENT_USER $ENV{USER})
set(DOCUMENTATION_DIR offline)
set(ONLY_OFFLINE YES)
set(ONLY_ONLINE NO)
set(ONLINE_DOC "")
set(INFOCENTER "http://infocenter.nordicsemi.com/topic")

set(PROJECT_REVISION ${GIT_HEAD_COMMIT_ID})
# Configure Doxyfiles
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/doxygen/Doxyfile.in
    ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    )

add_custom_target(doc
    COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/doxygen/Doxyfile.in
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating Doxygen documentation..." VERBATIM
    )

# Configure Doxyfile for building the Infocenter documentation:
set(PROJECT_REVISION "")
set(DOCUMENTATION_DIR infocenter)
set(ONLY_OFFLINE NO)
set(ONLY_ONLINE YES)
set(ONLINE_DOC "_infocenter")
set(INFOCENTER "PLUGINS_ROOT")
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/doxygen/Doxyfile.in
    ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile-infocenter
    )
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/doxygen/MANIFEST.MF.meshsdk.in
    ${CMAKE_CURRENT_BINARY_DIR}/infocenter/MANIFEST.MF.meshsdk
    )
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/doxygen/buildjar.xml.in
    ${CMAKE_CURRENT_BINARY_DIR}/infocenter/buildjar.xml
    )

# Build infocenter documentation:
add_custom_target(doc_online
    COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile-infocenter
    SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/doxygen/Doxyfile.in
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating online Doxygen documentation..." VERBATIM
    )

# Add dependencies for the serial documentation:
add_dependencies(doc serial_doc_markdown)
add_dependencies(doc_online serial_doc_markdown)
