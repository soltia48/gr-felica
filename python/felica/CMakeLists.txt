pybind11_add_module(felica_python
  bindings/iq_to_pdu_python.cc
)

target_link_libraries(felica_python
  PRIVATE
    gnuradio-felica
    gnuradio::gnuradio-runtime
)

set_target_properties(felica_python PROPERTIES
  INSTALL_RPATH "$ORIGIN/../../.."
)

install(TARGETS felica_python
  LIBRARY DESTINATION ${FELICA_PYTHON_DIR}/felica
)

install(FILES
  __init__.py
  DESTINATION ${FELICA_PYTHON_DIR}/felica
)
