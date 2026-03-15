build_lib_example(
  NAME leosim-example
  SOURCE_FILES leosim-example.cc
  LIBRARIES_TO_LINK
    ${libleosim}
    ${libcore}
)

