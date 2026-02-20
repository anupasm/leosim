build_lib_example(
  NAME leosim-example
  SOURCE_FILES leosim-example.cc
  LIBRARIES_TO_LINK
    ${libleosim}
    ${libcore}
)

build_lib_example(
  NAME leosim-routing-example
  SOURCE_FILES leosim-routing-example.cc
  LIBRARIES_TO_LINK
    ${libleosim}
    ${libcore}
    ${libnetwork}
    ${libmobility}
)
build_lib_example(
  NAME leosim-device-installer-example
  SOURCE_FILES leosim-device-installer-example.cc
  LIBRARIES_TO_LINK
    ${libleosim}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libinternet}
    ${libapplications}
)