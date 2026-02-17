#ifndef INCLUDED_GNURADIO_FELICA_IQ_TO_PDU_H
#define INCLUDED_GNURADIO_FELICA_IQ_TO_PDU_H

#include <gnuradio/block.h>
#include <gnuradio/felica/api.h>

namespace gr {
namespace felica {

class FELICA_API iq_to_pdu : virtual public gr::block {
public:
  using sptr = std::shared_ptr<iq_to_pdu>;

  static sptr make(float sample_rate, int bitrate = 212000,
                   bool check_crc = true, float min_contrast = 0.08f,
                   bool include_len_byte = false, bool use_agc = true,
                   float agc_tau_bits = 64.0f);
};

} // namespace felica
} // namespace gr

#endif
