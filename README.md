# gr-felica

GNU Radio OOT module that outputs FeliCa payloads as PDUs from complex I/Q, based on JIS X 6319-4:2016 preamble/sync/information/CRC framing.

## Build

```bash
cmake -S . -B build
cmake --build build -j
```

## Install

```bash
cmake --install build
```

## GRC Block

- Block name: `FeliCa IQ to PDU`
- Input: `complex` stream
- Output: `pdu` message

## Notes

- Expected bitrate is `212000` or `424000`
- `sample_rate` must match the actual sample rate of the input stream
- `sample_rate / bitrate >= 2` is required
- Payload command/response parsing is not performed; output is emitted as raw bytes in a PDU
- Metadata such as `crc_ok`, `start_sample`, and `span_samples` is attached
