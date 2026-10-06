build_lib_example(
  NAME leosim-experiments
  SOURCE_FILES leosim-experiments.cc
  LIBRARIES_TO_LINK
    ${libleosim}
    ${libapplications}
    ${libcore}
    ${libflow-monitor}
    ${libinternet}
    ${libinternet-apps}
    ${libmobility}
    ${libnetwork}
    ${libpoint-to-point}
)
