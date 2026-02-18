build_lib_example(
  NAME leosim-example
  SOURCE_FILES leosim-example.cc
  LIBRARIES_TO_LINK
    ${libleosim}
    ${libcore}
)

build_lib_example(
  NAME leosim-loader-example
  SOURCE_FILES leosim-loader-example.cc
  LIBRARIES_TO_LINK
    ${libleosim}
    ${libcore}
    ${libnetwork}
    ${libmobility}
)

build_lib_example(
  NAME leosim-mobility-example
  SOURCE_FILES leosim-mobility-example.cc
  LIBRARIES_TO_LINK
    ${libleosim}
    ${libcore}
    ${libnetwork}
    ${libmobility}
)

build_lib_example(
  NAME leosim-visualization-example
  SOURCE_FILES leosim-visualization-example.cc
  LIBRARIES_TO_LINK
    ${libleosim}
    ${libcore}
    ${libnetwork}
    ${libmobility}
)

build_lib_example(
  NAME leosim-channel-example
  SOURCE_FILES leosim-channel-example.cc
  LIBRARIES_TO_LINK
    ${libleosim}
    ${libcore}
    ${libnetwork}
    ${libmobility}
)

build_lib_example(
  NAME leosim-tcp-example
  SOURCE_FILES leosim-tcp-example.cc
  LIBRARIES_TO_LINK
    ${libleosim}
    ${libcore}
    ${libnetwork}
    ${libinternet}
    ${libapplications}
    ${libmobility}
)
