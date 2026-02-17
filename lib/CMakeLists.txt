add_library(gnuradio-felica SHARED
  iq_to_pdu_impl.cc
)

target_compile_definitions(gnuradio-felica PRIVATE felica_EXPORTS)

target_include_directories(gnuradio-felica
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(gnuradio-felica
  PUBLIC
    gnuradio::gnuradio-runtime
)

set_target_properties(gnuradio-felica PROPERTIES
  OUTPUT_NAME gnuradio-felica
)

install(TARGETS gnuradio-felica
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(FILES
  ${CMAKE_SOURCE_DIR}/include/gnuradio/felica/api.h
  ${CMAKE_SOURCE_DIR}/include/gnuradio/felica/iq_to_pdu.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gnuradio/felica
)
