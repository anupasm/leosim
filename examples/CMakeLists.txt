build_lib_example(
  NAME leosim-example
  SOURCE_FILES leosim-example.cc
  LIBRARIES_TO_LINK
    ${libleosim}
    ${libcore}
)

build_lib_example(
  NAME leosim-routing-research-example
  SOURCE_FILES leosim-routing-research-example.cc
  LIBRARIES_TO_LINK
    ${libleosim}
    ${libcore}
)

build_lib_example(
  NAME leosim-param-scenario
  SOURCE_FILES leosim-param-scenario.cc
  LIBRARIES_TO_LINK
    ${libleosim}
    ${libapplications}
    ${libcore}
    ${libflow-monitor}
    ${libinternet}
    ${libmobility}
    ${libnetwork}
    ${libpoint-to-point}
)
