#.rst:
# FindNode
# -------
#
# This module looks for Node.js and NPM executables
#

include (FindPackageHandleStandardArgs)

# Look for 'Node'
#
find_program (NODE_EXECUTABLE
  NAMES node
  PATHS
    /home/$ENV{USER}/.nvm/versions/node/v* #.nvm
  DOC "JavaScript runtime environment Node.js interpreter"
)

if(NODE_EXECUTABLE)
  execute_process(COMMAND ${NODE_EXECUTABLE} "--version" OUTPUT_VARIABLE NODE_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()

find_package_handle_standard_args(Node
                                  REQUIRED_VARS NODE_EXECUTABLE
                                  VERSION_VAR NODE_VERSION)


# Look for 'NPM'
#
find_program (NPM_EXECUTABLE
  NAMES npm
  PATHS
    /home/$ENV{USER}/.nvm/versions/node/v* #.nvm
  DOC "Node.js default package manager"
)

if(NPM_EXECUTABLE)
  execute_process(COMMAND ${NPM_EXECUTABLE} "--version" OUTPUT_VARIABLE NPM_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()

find_package_handle_standard_args(NPM
                                  REQUIRED_VARS NPM_EXECUTABLE
                                  VERSION_VAR NPM_VERSION)


mark_as_advanced (NODE_FOUND NODE_EXECUTABLE NPM_FOUND NPM_EXECUTABLE)
