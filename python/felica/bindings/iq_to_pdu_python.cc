#include <pybind11/pybind11.h>

#include <gnuradio/basic_block.h>
#include <gnuradio/block.h>
#include <gnuradio/felica/iq_to_pdu.h>

namespace py = pybind11;

PYBIND11_MODULE(felica_python, m) {
  py::module::import("gnuradio.gr");

  using iq_to_pdu = gr::felica::iq_to_pdu;

  py::class_<iq_to_pdu, gr::block, gr::basic_block, std::shared_ptr<iq_to_pdu>>(
      m, "iq_to_pdu")
      .def(py::init(&iq_to_pdu::make), py::arg("sample_rate"),
           py::arg("bitrate") = 212000, py::arg("check_crc") = true,
           py::arg("min_contrast") = 0.08f, py::arg("include_len_byte") = false,
           py::arg("use_agc") = true, py::arg("agc_tau_bits") = 64.0f,
           R"pbdoc(
Decode JIS X 6319-4 (FeliCa) frames from complex I/Q and emit payload PDUs.
)pbdoc");
}
