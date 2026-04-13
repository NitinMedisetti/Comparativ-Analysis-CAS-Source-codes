#include "cbor_codec.h"

static inline bool cbor_put_ai(CborBuf& b, uint8_t major, uint64_t ai){
  if (ai < 24){
    return cbor_put(b, (uint8_t)((major << 5) | (uint8_t)ai));
  } else if (ai <= 0xFF){
    uint8_t hdr[2] = { (uint8_t)((major << 5) | 24), (uint8_t)ai };
    return cbor_putn(b, hdr, 2);
  } else if (ai <= 0xFFFF){
    uint8_t hdr[3] = { (uint8_t)((major << 5) | 25), 0, 0 };
    hdr[1] = (uint8_t)((ai >> 8) & 0xFF);
    hdr[2] = (uint8_t)(ai & 0xFF);
    return cbor_putn(b, hdr, 3);
  } else if (ai <= 0xFFFFFFFFULL){
    uint8_t hdr[5] = { (uint8_t)((major << 5) | 26), 0, 0, 0, 0 };
    hdr[1] = (uint8_t)((ai >> 24) & 0xFF);
    hdr[2] = (uint8_t)((ai >> 16) & 0xFF);
    hdr[3] = (uint8_t)((ai >> 8) & 0xFF);
    hdr[4] = (uint8_t)(ai & 0xFF);
    return cbor_putn(b, hdr, 5);
  } else {
    uint8_t hdr[9] = { (uint8_t)((major << 5) | 27), 0,0,0,0,0,0,0,0 };
    hdr[1] = (uint8_t)((ai >> 56) & 0xFF);
    hdr[2] = (uint8_t)((ai >> 48) & 0xFF);
    hdr[3] = (uint8_t)((ai >> 40) & 0xFF);
    hdr[4] = (uint8_t)((ai >> 32) & 0xFF);
    hdr[5] = (uint8_t)((ai >> 24) & 0xFF);
    hdr[6] = (uint8_t)((ai >> 16) & 0xFF);
    hdr[7] = (uint8_t)((ai >> 8) & 0xFF);
    hdr[8] = (uint8_t)(ai & 0xFF);
    return cbor_putn(b, hdr, 9);
  }
}

bool cbor_uint(CborBuf& b, uint64_t v){ return cbor_put_ai(b, 0, v); }

bool cbor_bstr(CborBuf& b, const uint8_t* data, size_t n){
  if (!cbor_put_ai(b, 2, n)) return false;
  return cbor_putn(b, data, n);
}

bool cbor_tstrn(CborBuf& b, const char* s, size_t n){
  if (!cbor_put_ai(b, 3, n)) return false;
  return cbor_putn(b, s, n);
}

bool cbor_tstr(CborBuf& b, const char* s){
  return cbor_tstrn(b, s, strlen(s));
}

bool cbor_array(CborBuf& b, size_t n){ return cbor_put_ai(b, 4, n); }
bool cbor_map(CborBuf& b, size_t n){ return cbor_put_ai(b, 5, n); }

bool cbor_false(CborBuf& b){ return cbor_put(b, 0xF4); }
bool cbor_true(CborBuf& b){ return cbor_put(b, 0xF5); }
bool cbor_null(CborBuf& b){ return cbor_put(b, 0xF6); }

bool cbor_float32(CborBuf& b, float f){
  uint8_t hdr = 0xFA; // float32
  if (!cbor_put(b, hdr)) return false;
  uint8_t be[4];
  static_assert(sizeof(float)==4, "float must be 32-bit");
  uint32_t v; memcpy(&v, &f, 4);
  // network byte order (big-endian)
  be[0] = (uint8_t)((v >> 24) & 0xFF);
  be[1] = (uint8_t)((v >> 16) & 0xFF);
  be[2] = (uint8_t)((v >> 8) & 0xFF);
  be[3] = (uint8_t)(v & 0xFF);
  return cbor_putn(b, be, 4);
}

bool cbor_float64(CborBuf& b, double d){
  uint8_t hdr = 0xFB; // float64
  if (!cbor_put(b, hdr)) return false;
  uint8_t be[8];
  static_assert(sizeof(double)==8, "double must be 64-bit");
  uint64_t v; memcpy(&v, &d, 8);
  be[0] = (uint8_t)((v >> 56) & 0xFF);
  be[1] = (uint8_t)((v >> 48) & 0xFF);
  be[2] = (uint8_t)((v >> 40) & 0xFF);
  be[3] = (uint8_t)((v >> 32) & 0xFF);
  be[4] = (uint8_t)((v >> 24) & 0xFF);
  be[5] = (uint8_t)((v >> 16) & 0xFF);
  be[6] = (uint8_t)((v >> 8) & 0xFF);
  be[7] = (uint8_t)(v & 0xFF);
  return cbor_putn(b, be, 8);
}
