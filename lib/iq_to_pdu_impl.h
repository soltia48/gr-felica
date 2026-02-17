#ifndef INCLUDED_GNURADIO_FELICA_IQ_TO_PDU_IMPL_H
#define INCLUDED_GNURADIO_FELICA_IQ_TO_PDU_IMPL_H

#include <gnuradio/felica/iq_to_pdu.h>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace gr {
namespace felica {

class iq_to_pdu_impl : public iq_to_pdu {
public:
  iq_to_pdu_impl(float sample_rate, int bitrate, bool check_crc,
                 float min_contrast, bool include_len_byte, bool use_agc,
                 float agc_tau_bits);
  ~iq_to_pdu_impl() override = default;

  void forecast(int noutput_items,
                gr_vector_int &ninput_items_required) override;

  int general_work(int noutput_items, gr_vector_int &ninput_items,
                   gr_vector_const_void_star &input_items,
                   gr_vector_void_star &output_items) override;

private:
  struct frame_t {
    std::vector<uint8_t> info;
    std::vector<uint8_t> payload;
    uint8_t sync0;
    uint8_t sync1;
    uint8_t len;
    uint8_t crc0;
    uint8_t crc1;
    uint16_t crc_calc;
    bool crc_ok;
    bool inverted;
    double start_sample;
    double span_samples;
  };

  float d_sample_rate;
  int d_bitrate;
  bool d_check_crc;
  float d_min_contrast;
  bool d_include_len_byte;
  bool d_use_agc;
  float d_agc_tau_bits;

  double d_spb;
  double d_hb;
  double d_agc_alpha;
  double d_agc_state;
  std::size_t d_phase_steps;
  std::size_t d_min_frame_samples;
  std::size_t d_max_frame_samples;

  std::vector<float> d_env;
  std::size_t d_search_pos;
  std::uint64_t d_env_start_index;
  double d_last_published_end;

  void process_buffer();

  bool try_decode_frame(std::size_t start_index,
                        const std::vector<double> &prefix, frame_t &frame,
                        double &frame_samples) const;

  bool decode_bit(double start, std::size_t bit_idx,
                  const std::vector<double> &prefix, double margin, bool invert,
                  int &bit) const;

  bool decode_byte(double start, std::size_t bit_idx,
                   const std::vector<double> &prefix, double margin,
                   bool invert, uint8_t &out) const;

  double halfbit_mean(double start, std::size_t half_idx,
                      const std::vector<double> &prefix) const;

  double cumulative_at(const std::vector<double> &prefix, double x) const;

  static uint16_t crc16_ccitt_msb(const std::vector<uint8_t> &bytes);

  void publish_pdu(const frame_t &frame);
};

} // namespace felica
} // namespace gr

#endif
