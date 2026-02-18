#include "iq_to_pdu_impl.h"

#include <gnuradio/io_signature.h>
#include <gnuradio/sptr_magic.h>
#include <pmt/pmt.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace gr {
namespace felica {

iq_to_pdu::sptr iq_to_pdu::make(float sample_rate, int bitrate, bool check_crc,
                                float min_contrast, bool include_len_byte,
                                bool use_agc, float agc_tau_bits) {
  return gnuradio::make_block_sptr<iq_to_pdu_impl>(
      sample_rate, bitrate, check_crc, min_contrast, include_len_byte, use_agc,
      agc_tau_bits);
}

iq_to_pdu_impl::iq_to_pdu_impl(float sample_rate, int bitrate, bool check_crc,
                               float min_contrast, bool include_len_byte,
                               bool use_agc, float agc_tau_bits)
    : gr::block("iq_to_pdu", gr::io_signature::make(1, 1, sizeof(gr_complex)),
                gr::io_signature::make(0, 0, 0)),
      d_sample_rate(sample_rate), d_bitrate(bitrate), d_check_crc(check_crc),
      d_min_contrast(min_contrast), d_include_len_byte(include_len_byte),
      d_use_agc(use_agc), d_agc_tau_bits(agc_tau_bits), d_spb(0), d_hb(0),
      d_inv_hb(0), d_agc_alpha(0.0), d_agc_state(0.0), d_phase_steps(4),
      d_min_frame_samples(0), d_max_frame_samples(0), d_env(), d_prefix(1, 0.0),
      d_search_pos(0), d_env_start_index(0), d_last_published_end(-1.0) {
  if (d_sample_rate <= 0.0f) {
    throw std::invalid_argument("sample_rate must be > 0");
  }
  if (d_bitrate <= 0) {
    throw std::invalid_argument("bitrate must be > 0");
  }

  const double spb_est =
      static_cast<double>(d_sample_rate) / static_cast<double>(d_bitrate);
  if (spb_est < 2.0) {
    throw std::invalid_argument("sample_rate/bitrate must be >= 2");
  }

  d_spb = spb_est;
  d_hb = d_spb / 2;
  d_inv_hb = 1.0 / d_hb;
  // Low oversampling (e.g. 500 kS/s at 212 kb/s) does not benefit from
  // a dense phase sweep; cut it to reduce per-candidate work.
  if (d_spb <= 3.0) {
    d_phase_steps = 2;
  }

  if (d_agc_tau_bits <= 0.0f) {
    throw std::invalid_argument("agc_tau_bits must be > 0");
  }
  const double tau_samples =
      std::max(8.0, d_spb * static_cast<double>(d_agc_tau_bits));
  d_agc_alpha = 1.0 / tau_samples;

  d_min_frame_samples =
      static_cast<std::size_t>(std::ceil((6 + 2 + 1 + 2) * 8 * d_spb + d_hb));
  d_max_frame_samples =
      static_cast<std::size_t>(std::ceil((6 + 2 + 255 + 2) * 8 * d_spb + d_hb));

  message_port_register_out(pmt::intern("pdu"));
}

void iq_to_pdu_impl::forecast(int, gr_vector_int &ninput_items_required) {
  ninput_items_required[0] = 0;
}

int iq_to_pdu_impl::general_work(int, gr_vector_int &ninput_items,
                                 gr_vector_const_void_star &input_items,
                                 gr_vector_void_star &) {
  const auto *in = static_cast<const gr_complex *>(input_items[0]);
  const int ninput = ninput_items[0];

  if (ninput > 0) {
    for (int i = 0; i < ninput; ++i) {
      const double env = static_cast<double>(std::abs(in[i]));
      double v = env;
      if (d_use_agc) {
        if (d_agc_state <= 0.0) {
          d_agc_state = std::max(env, 1.0e-9);
        }
        d_agc_state += d_agc_alpha * (env - d_agc_state);
        v = env / std::max(d_agc_state, 1.0e-9);
      }
      const auto sample = static_cast<float>(v);
      d_env.push_back(sample);
      d_prefix.push_back(d_prefix.back() + static_cast<double>(sample));
    }

    process_buffer();
  }

  consume_each(ninput);
  return 0;
}

double iq_to_pdu_impl::cumulative_at(double x) const {
  const double n = static_cast<double>(d_prefix.size() - 1);
  if (x <= 0.0) {
    return d_prefix.front();
  }
  if (x >= n) {
    return d_prefix.back();
  }

  const auto i = static_cast<std::size_t>(std::floor(x));
  const double frac = x - static_cast<double>(i);
  const double a = d_prefix[i];
  const double b = d_prefix[i + 1];
  return a + frac * (b - a);
}

double iq_to_pdu_impl::halfbit_mean(double start, std::size_t half_idx) const {
  const double seg_start = start + static_cast<double>(half_idx) * d_hb;
  const double seg_end = seg_start + d_hb;
  const double sum = cumulative_at(seg_end) - cumulative_at(seg_start);
  return sum * d_inv_hb;
}

bool iq_to_pdu_impl::decode_bit(double start, std::size_t bit_idx,
                                double margin, bool invert, int &bit) const {
  const double first = halfbit_mean(start, bit_idx * 2);
  const double second = halfbit_mean(start, bit_idx * 2 + 1);

  const double delta = first - second;
  if (std::abs(delta) < margin) {
    return false;
  }

  int raw_bit = (delta > 0.0) ? 1 : 0;
  if (invert) {
    raw_bit = 1 - raw_bit;
  }
  bit = raw_bit;
  return true;
}

bool iq_to_pdu_impl::decode_byte(double start, std::size_t bit_idx,
                                 double margin, bool invert,
                                 uint8_t &out) const {
  uint8_t value = 0;
  for (std::size_t i = 0; i < 8; ++i) {
    int bit = 0;
    if (!decode_bit(start, bit_idx + i, margin, invert, bit)) {
      return false;
    }
    value = static_cast<uint8_t>((value << 1) | static_cast<uint8_t>(bit));
  }
  out = value;
  return true;
}

uint16_t iq_to_pdu_impl::crc16_ccitt_msb(const std::vector<uint8_t> &bytes) {
  uint16_t crc = 0x0000;
  for (const auto b : bytes) {
    crc ^= static_cast<uint16_t>(b) << 8;
    for (int i = 0; i < 8; ++i) {
      if ((crc & 0x8000) != 0) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

bool iq_to_pdu_impl::try_decode_frame(std::size_t start_index, frame_t &frame,
                                      double &frame_samples) const {
  if ((start_index + d_min_frame_samples) > d_env.size()) {
    return false;
  }

  constexpr std::size_t preamble_bits = 48;
  double preamble_deltas[preamble_bits];

  for (std::size_t phase_idx = 0; phase_idx < d_phase_steps; ++phase_idx) {
    const double phase = (static_cast<double>(phase_idx) * d_hb) /
                         static_cast<double>(d_phase_steps);
    const double start = static_cast<double>(start_index) + phase;
    if ((start + static_cast<double>(d_min_frame_samples)) >
        static_cast<double>(d_env.size())) {
      continue;
    }

    double first_sum = 0.0;
    double second_sum = 0.0;
    double delta_sum = 0.0;

    for (std::size_t bit = 0; bit < preamble_bits; ++bit) {
      const double first = halfbit_mean(start, bit * 2);
      const double second = halfbit_mean(start, bit * 2 + 1);
      const double delta = first - second;
      first_sum += first;
      second_sum += second;
      delta_sum += delta;
      preamble_deltas[bit] = delta;
    }

    const double first_avg = first_sum / static_cast<double>(preamble_bits);
    const double second_avg = second_sum / static_cast<double>(preamble_bits);
    const double hi = std::max(first_avg, second_avg);
    const double lo = std::min(first_avg, second_avg);
    const double span = hi - lo;

    if (hi <= 0.0) {
      continue;
    }

    const double contrast = span / hi;
    if (contrast < static_cast<double>(d_min_contrast)) {
      continue;
    }

    const bool invert = (delta_sum > 0.0);
    // Keep decision threshold tied to modulation span so weak load-modulated
    // responses are still decodable on top of a large carrier baseline.
    const double margin = std::max(span * 0.03, 1.0e-7);

    std::size_t zero_count = 0;
    for (std::size_t bit = 0; bit < preamble_bits; ++bit) {
      const double delta = preamble_deltas[bit];
      if (std::abs(delta) < margin) {
        zero_count = 0;
        break;
      }
      int value = (delta > 0.0) ? 1 : 0;
      if (invert) {
        value = 1 - value;
      }
      if (value == 0) {
        ++zero_count;
      }
      const std::size_t remaining = preamble_bits - (bit + 1);
      if ((zero_count + remaining) < 44) {
        zero_count = 0;
        break;
      }
    }
    if (zero_count < 44) {
      continue;
    }

    uint8_t sync0 = 0;
    uint8_t sync1 = 0;
    if (!decode_byte(start, 48, margin, invert, sync0)) {
      continue;
    }
    if (!decode_byte(start, 56, margin, invert, sync1)) {
      continue;
    }

    const bool sync_ok =
        ((sync0 == 0xB2 && sync1 == 0x4D) || (sync0 == 0x4D && sync1 == 0xB2));
    if (!sync_ok) {
      continue;
    }

    uint8_t len = 0;
    if (!decode_byte(start, 64, margin, invert, len)) {
      continue;
    }
    if (len == 0) {
      continue;
    }

    const std::size_t total_bits =
        (6 + 2 + static_cast<std::size_t>(len) + 2) * 8;
    const double decoded_span = static_cast<double>(total_bits) * d_spb;
    if ((start + decoded_span) > static_cast<double>(d_env.size())) {
      continue;
    }

    std::vector<uint8_t> info(len);
    for (std::size_t i = 0; i < info.size(); ++i) {
      if (!decode_byte(start, 64 + i * 8, margin, invert, info[i])) {
        info.clear();
        break;
      }
    }
    if (info.empty()) {
      continue;
    }

    uint8_t crc0 = 0;
    uint8_t crc1 = 0;
    if (!decode_byte(start, 64 + static_cast<std::size_t>(len) * 8, margin,
                     invert, crc0)) {
      continue;
    }
    if (!decode_byte(start, 72 + static_cast<std::size_t>(len) * 8, margin,
                     invert, crc1)) {
      continue;
    }

    const uint16_t crc_calc = crc16_ccitt_msb(info);
    const uint16_t crc_rx_be =
        static_cast<uint16_t>(crc0) << 8 | static_cast<uint16_t>(crc1);
    const uint16_t crc_rx_le =
        static_cast<uint16_t>(crc1) << 8 | static_cast<uint16_t>(crc0);
    const bool crc_ok = (crc_calc == crc_rx_be) || (crc_calc == crc_rx_le);

    if (d_check_crc && !crc_ok) {
      continue;
    }

    frame.info = std::move(info);
    frame.payload.clear();
    if (d_include_len_byte) {
      frame.payload = frame.info;
    } else if (frame.info.size() > 1) {
      frame.payload.assign(frame.info.begin() + 1, frame.info.end());
    }

    frame.sync0 = sync0;
    frame.sync1 = sync1;
    frame.len = len;
    frame.crc0 = crc0;
    frame.crc1 = crc1;
    frame.crc_calc = crc_calc;
    frame.crc_ok = crc_ok;
    frame.inverted = invert;
    frame.start_sample = start;
    frame.span_samples = decoded_span;
    frame_samples = decoded_span + phase;

    return true;
  }

  return false;
}

void iq_to_pdu_impl::publish_pdu(const frame_t &frame) {
  pmt::pmt_t meta = pmt::make_dict();
  meta = pmt::dict_add(meta, pmt::intern("len"), pmt::from_long(frame.len));
  meta = pmt::dict_add(meta, pmt::intern("sync0"), pmt::from_long(frame.sync0));
  meta = pmt::dict_add(meta, pmt::intern("sync1"), pmt::from_long(frame.sync1));
  meta =
      pmt::dict_add(meta, pmt::intern("crc_ok"), pmt::from_bool(frame.crc_ok));
  meta = pmt::dict_add(meta, pmt::intern("crc_calc"),
                       pmt::from_long(static_cast<long>(frame.crc_calc)));
  meta = pmt::dict_add(meta, pmt::intern("crc_rx"),
                       pmt::from_long(static_cast<long>(
                           (static_cast<uint16_t>(frame.crc0) << 8) |
                           static_cast<uint16_t>(frame.crc1))));
  meta = pmt::dict_add(meta, pmt::intern("inverted"),
                       pmt::from_bool(frame.inverted));
  meta = pmt::dict_add(meta, pmt::intern("start_sample"),
                       pmt::from_double(frame.start_sample));
  meta = pmt::dict_add(meta, pmt::intern("span_samples"),
                       pmt::from_double(frame.span_samples));

  pmt::pmt_t vec = pmt::make_u8vector(frame.payload.size(), 0);
  for (std::size_t i = 0; i < frame.payload.size(); ++i) {
    pmt::u8vector_set(vec, i, frame.payload[i]);
  }

  message_port_pub(pmt::intern("pdu"), pmt::cons(meta, vec));
}

void iq_to_pdu_impl::process_buffer() {
  if (d_env.size() < d_min_frame_samples) {
    return;
  }

  if (d_search_pos > d_env.size()) {
    d_search_pos = 0;
  }

  std::size_t i = d_search_pos;
  while ((i + d_min_frame_samples) <= d_env.size()) {
    frame_t frame;
    double frame_samples = 0.0;

    if (try_decode_frame(i, frame, frame_samples)) {
      frame.start_sample += static_cast<double>(d_env_start_index);
      const double frame_end = frame.start_sample + frame.span_samples;
      const bool overlaps_prev =
          (d_last_published_end >= 0.0) &&
          (frame.start_sample < (d_last_published_end - d_hb));
      if (!overlaps_prev) {
        publish_pdu(frame);
        d_last_published_end = frame_end;
      }
      const auto adv =
          static_cast<std::size_t>(std::max(1.0, std::floor(frame_samples)));
      i += adv;
    } else {
      ++i;
    }
  }

  d_search_pos = i;

  if (d_search_pos > d_max_frame_samples) {
    const std::size_t drop = d_search_pos - d_max_frame_samples;
    d_env.erase(d_env.begin(),
                d_env.begin() + static_cast<std::ptrdiff_t>(drop));
    d_prefix.erase(d_prefix.begin(),
                   d_prefix.begin() + static_cast<std::ptrdiff_t>(drop));
    d_search_pos -= drop;
    d_env_start_index += static_cast<std::uint64_t>(drop);
  }

  if (d_search_pos > d_env.size()) {
    d_search_pos = d_env.size();
  }
}

} // namespace felica
} // namespace gr
