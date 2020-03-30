set(SERIAL_DOC_GEN_INPUT_FILES
    ${CMAKE_SOURCE_DIR}/mesh/serial/api/serial_types.h
    ${CMAKE_SOURCE_DIR}/mesh/serial/include/serial_cmd.h
    ${CMAKE_SOURCE_DIR}/mesh/serial/include/serial_evt.h
    ${CMAKE_SOURCE_DIR}/mesh/serial/include/serial_cmd_rsp.h)

set(PYACI_DIR
    ${CMAKE_SOURCE_DIR}/scripts/interactive_pyaci/aci)

add_custom_target(serial_doc_asciidoc
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/serial_doc_gen_asciidoc.py ${SERIAL_DOC_GEN_INPUT_FILES} -o ${MBTLE_SOURCE_DIR}/doc/user_guide/modules
    DEPENDS ${SERIAL_DOC_GEN_INPUT_FILES}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "-- Generating serial doc (asciidoc)")

add_custom_target(serial_doc_markdown
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/serial_doc_gen_markdown.py ${SERIAL_DOC_GEN_INPUT_FILES} -o  ${MBTLE_SOURCE_DIR}/doc/user_guide/modules
    DEPENDS ${SERIAL_DOC_GEN_INPUT_FILES}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "-- Generating serial doc (markdown)")

add_custom_target(serial_pyaci
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/serial_doc_gen_pyaci.py ${SERIAL_DOC_GEN_INPUT_FILES} -o ${PYACI_DIR}
    DEPENDS ${SERIAL_DOC_GEN_INPUT_FILES}
    # BYPRODUCTS ${PYACI_DIR}/aci_cmd.py ${PYACI_DIR}/aci_evt.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "-- Generating PyACI")
