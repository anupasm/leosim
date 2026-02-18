build_lib(
  LIBNAME leosim
  SOURCE_FILES
    model/leosim-channel.cc
    model/leosim-loader.cc
    model/leosim-mobility-model.cc
    model/leosim-channel-model.cc
    model/leosim-isl-routing-model.cc
    helper/leosim-helper.cc
    helper/leosim-loader-helper.cc
    helper/leosim-mobility-helper.cc
    helper/leosim-channel-helper.cc
    helper/leosim-visualization-helper.cc
    
  HEADER_FILES
    model/leosim-channel.h
    model/leosim-loader.h
    model/leosim-mobility-model.h
    model/leosim-channel-model.h
    model/leosim-isl-routing-model.h
    helper/leosim-helper.h
    helper/leosim-loader-helper.h
    helper/leosim-mobility-helper.h
    helper/leosim-channel-helper.h
    helper/leosim-visualization-helper.h
    
  LIBRARIES_TO_LINK
    ${libcore}
    ${libnetwork}
    ${libinternet}
    ${libapplications}
    ${libmobility}
    ${libpropagation}
    ${libflow-monitor}
  TEST_SOURCES
    test/leosim-test-suite.cc
)
