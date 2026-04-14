#pragma once
#include <Arduino.h>

// Minimal CBOR writer for constrained environments.
//
// Notes:
// - No dynamic allocation; caller provides the output buffer.
// - Integers/lengths are encoded per RFC 8949 (major types 0..7).
// - Floats are serialized in network byte order (big-endian).

struct CborBuf {
  uint8_t* p;
  size_t cap;
  size_t len;
};

inline void cbor_init(CborBuf& b, uint8_t* out, size_t cap){ b.p = out; b.cap = cap; b.len = 0; }

inline bool cbor_put(CborBuf& b, uint8_t byte){ if (b.len >= b.cap) return false; b.p[b.len++] = byte; return true; }
inline bool cbor_putn(CborBuf& b, const void* src, size_t n){ if (b.len + n > b.cap) return false; memcpy(b.p + b.len, src, n); b.len += n; return true; }

// major 0: unsigned integer
bool cbor_uint(CborBuf& b, uint64_t v);
// major 1: negative integer (unused)
// major 2: byte string
bool cbor_bstr(CborBuf& b, const uint8_t* data, size_t n);
// major 3: text string
bool cbor_tstr(CborBuf& b, const char* s);
bool cbor_tstrn(CborBuf& b, const char* s, size_t n);
// major 4: array
bool cbor_array(CborBuf& b, size_t n);
// major 5: map
bool cbor_map(CborBuf& b, size_t n);
// major 7: simple and floats
bool cbor_false(CborBuf& b);
bool cbor_true(CborBuf& b);
bool cbor_null(CborBuf& b);
bool cbor_float32(CborBuf& b, float f);
bool cbor_float64(CborBuf& b, double d);
