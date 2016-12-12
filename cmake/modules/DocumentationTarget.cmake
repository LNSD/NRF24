#====================================================================#
#  Setup Documentation generation target                             #
#====================================================================#

# Find Doxygen and NodeJS/NPM required packages
find_package (Doxygen) # sudo apt install doxygen
find_package (LibXslt) # sudo apt install libxslt1-dev xsltproc
find_package (Node)

option (BUILD_DOCUMENTATION "generate the HTML based API documentation (requires Doxygen and Node.js)" ${DOXYGEN_FOUND})

if (BUILD_DOCUMENTATION)

    # Check if Doxygen is available
    if (NOT DOXYGEN_FOUND)
        message (FATAL_ERROR "Doxygen is needed to build the documentation.")
    endif (NOT DOXYGEN_FOUND)

    # Configure doxyfile and copy the output doxyfile to binary dir
    set (doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/docs/doxyfile.in)
    set (doxyfile ${CMAKE_CURRENT_BINARY_DIR}/doxyfile)

    configure_file (${doxyfile_in} ${doxyfile})

    # Add a target to generate API documentation with Doxygen (xml and html)
    add_custom_target(docs
            COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen (xml and html)"
            VERBATIM)


    # Check if xsltproc is available
    if (NOT LIBXSLT_XSLTPROC_EXECUTABLE)
      message (FATAL_ERROR "'xsltproc' is needed to build the documentation.")
    endif (NOT LIBXSLT_XSLTPROC_EXECUTABLE)

    # Combine xml output with xsltproc tool
    set (template ${CMAKE_CURRENT_BINARY_DIR}/docs/xml/combine.xslt)
    set (index ${CMAKE_CURRENT_BINARY_DIR}/docs/xml/index.xml)
    set (output ${CMAKE_CURRENT_BINARY_DIR}/docs/xml/doxygen.xml)

    add_custom_command(TARGET docs
            POST_BUILD
            COMMAND ${LIBXSLT_XSLTPROC_EXECUTABLE} ${template} ${index} > ${output}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/docs/xml
            COMMENT "Combining Doxygen XML output with 'xsltproc' into ${output}"
            VERBATIM)


    # Check if NodeJS and NPM are available
    if (NOT NODE_FOUND)
        message (FATAL_ERROR "Node.js is needed to build the documentation.")
    endif()

    if (NOT NPM_FOUND)
        message (FATAL_ERROR "NPM is needed to build the documentation.")
    endif()

    # Copy docs submodule to build directory
    file (COPY ${CMAKE_CURRENT_SOURCE_DIR}/docs DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

    add_custom_command(TARGET docs
            POST_BUILD
            COMMAND ${NPM_EXECUTABLE} install
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/docs
            COMMENT "Install all node dependencies"
            VERBATIM)

    add_custom_command(TARGET docs
            POST_BUILD
            COMMAND ${NPM_EXECUTABLE} run build
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/docs
            COMMENT "Generating documentation with gulp"
            VERBATIM)
endif()
