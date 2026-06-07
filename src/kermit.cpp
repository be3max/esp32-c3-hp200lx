#include "kermit.h"

// Kermit character helpers
#define SOH        0x01
#define KEOL       '\r'          // we transmit \r as packet terminator
#define tochar(c)  ((char)((c) + 32))
#define unchar(c)  ((uint8_t)((c) - 32))
#define ctl(c)     ((uint8_t)((c) ^ 64))

#define MAX_DATA   94            // short-packet data cap
#define RAW_CHUNK  30            // raw bytes per outgoing data packet
                                 // (worst-case 3x expansion = 90 <= 91 data cap)
#define PKT_TIMEOUT 5000UL       // per-packet wait (ms)
#define MAX_RETRY   6

// ---------------------------------------------------------------------------
// low-level helpers
// ---------------------------------------------------------------------------
void KermitClient::setErr(const char* msg) {
    strncpy(_err, msg, sizeof(_err) - 1);
    _err[sizeof(_err) - 1] = 0;
}

void KermitClient::beginOp(char qbin) {
    _err[0] = 0;
    _bytes  = 0;
    _qctl   = '#';
    _qbin   = qbin;   // PUT advertises '&'; receive-side ops start disabled
    _rept   = 0;
    drainInput();
}

void KermitClient::drainInput() {
    uint32_t t = millis();
    while (millis() - t < 50) {
        while (_io.available()) { _io.read(); t = millis(); }
    }
}

uint8_t KermitClient::chk1(const uint8_t* p, size_t n) {
    uint16_t s = 0;
    for (size_t i = 0; i < n; i++) s += p[i];
    return (uint8_t)((s + ((s & 0xC0) >> 6)) & 0x3F);
}

// ---------------------------------------------------------------------------
// data field encode / decode (control, 8-bit, repeat quoting)
// ---------------------------------------------------------------------------
size_t KermitClient::encodeData(const uint8_t* in, size_t inLen,
                                uint8_t* out, size_t outMax) {
    size_t o = 0;
    for (size_t i = 0; i < inLen; i++) {
        uint8_t b = in[i];
        // (no repeat compression on send — simpler, still valid)
        if ((b & 0x80) && _qbin) {
            if (o >= outMax) break;
            out[o++] = _qbin;
            b &= 0x7F;
        }
        uint8_t c7 = b & 0x7F;
        if (c7 < 32 || c7 == 127) {
            if (o + 2 > outMax) break;
            out[o++] = _qctl;
            out[o++] = ctl(c7);
        } else if (c7 == (uint8_t)_qctl ||
                   (_qbin && c7 == (uint8_t)_qbin) ||
                   (_rept && c7 == (uint8_t)_rept)) {
            if (o + 2 > outMax) break;
            out[o++] = _qctl;
            out[o++] = c7;
        } else {
            if (o >= outMax) break;
            out[o++] = c7;
        }
    }
    return o;
}

size_t KermitClient::decodeData(const uint8_t* in, size_t inLen,
                                uint8_t* out, size_t outMax) {
    size_t o = 0, i = 0;
    while (i < inLen) {
        uint8_t c = in[i++];
        int rpt = 1;
        if (_rept && c == (uint8_t)_rept && i < inLen) {
            rpt = unchar(in[i++]);
            if (i >= inLen) break;
            c = in[i++];
        }
        uint8_t hib = 0;
        if (_qbin && c == (uint8_t)_qbin && i < inLen) {
            hib = 0x80;
            c = in[i++];
        }
        if (c == (uint8_t)_qctl && i < inLen) {
            c = in[i++];
            uint8_t m = c & 0x7F;
            if (m >= 63 && m <= 95) c = c ^ 64;   // un-controlify
            // else literal escaped prefix char
        }
        c |= hib;
        for (int r = 0; r < rpt && o < outMax; r++) out[o++] = c;
    }
    return o;
}

// ---------------------------------------------------------------------------
// packet send / receive
// ---------------------------------------------------------------------------
void KermitClient::sendPacket(char type, uint8_t seq,
                              const uint8_t* data, size_t len) {
    uint8_t buf[MAX_DATA + 8];
    size_t n = 0;
    uint8_t lenField = (uint8_t)(len + 3);     // SEQ + TYPE + CHECK + data
    buf[n++] = tochar(lenField);
    buf[n++] = tochar(seq & 0x3F);
    buf[n++] = (uint8_t)type;
    for (size_t i = 0; i < len; i++) buf[n++] = data[i];
    uint8_t c = chk1(buf, n);                   // over LEN..last data byte
    buf[n++] = tochar(c);

    _io.write((uint8_t)SOH);
    _io.write(buf, n);
    _io.write((uint8_t)KEOL);
    _io.flush();
}

// Returns data length (>=0) on a good packet, -1 on timeout/garbage/bad cksum.
int KermitClient::recvPacket(char& type, uint8_t& seq, uint8_t* data,
                             size_t maxData, uint32_t timeoutMs) {
    uint32_t start = millis();

    // sync to SOH
    for (;;) {
        if (millis() - start > timeoutMs) return -1;
        if (!_io.available()) { delay(1); continue; }
        if (_io.read() == SOH) break;
    }

    auto getByte = [&](uint8_t& b) -> bool {
        while (!_io.available()) {
            if (millis() - start > timeoutMs) return false;
            delay(1);
        }
        b = (uint8_t)_io.read();
        return true;
    };

    uint8_t lenCh;
    if (!getByte(lenCh)) return -1;
    uint8_t len = unchar(lenCh);               // SEQ+TYPE+DATA+CHECK count
    if (len < 3 || len > MAX_DATA + 3) return -1;

    uint8_t raw[MAX_DATA + 4];
    raw[0] = lenCh;
    for (uint8_t i = 0; i < len; i++) {
        if (!getByte(raw[1 + i])) return -1;
    }
    // raw now: [lenCh][seq][type][data..][check]
    uint8_t got   = raw[len];                   // checksum char (last)
    uint8_t calc  = tochar(chk1(raw, len));     // over lenCh..last data
    if (got != calc) return -1;

    seq  = unchar(raw[1]);
    type = (char)raw[2];
    size_t dlen = len - 3;                       // data byte count
    if (dlen > maxData) dlen = maxData;
    for (size_t i = 0; i < dlen; i++) data[i] = raw[3 + i];
    return (int)dlen;
}

// ---------------------------------------------------------------------------
// parameter negotiation
// ---------------------------------------------------------------------------
String KermitClient::buildInitParams() {
    // MAXL, TIME, NPAD, PADC, EOL, QCTL, QBIN, CHKT, REPT
    String d;
    d += tochar(MAX_DATA);   // max packet length we accept
    d += tochar(5);          // timeout seconds for peer
    d += tochar(0);          // no padding
    d += (char)ctl(0);       // pad char (NUL) -> '@'
    d += tochar(13);         // EOL we want peer to use (CR)
    d += '#';                // QCTL
    d += '&';                // QBIN: request 8-bit quoting
    d += '1';                // CHKT: 1-char checksum
    d += '~';                // REPT: accept repeat prefix
    return d;
}

void KermitClient::parsePeerParams(const uint8_t* d, size_t n) {
    // fields: [0]MAXL [1]TIME [2]NPAD [3]PADC [4]EOL [5]QCTL [6]QBIN [7]CHKT [8]REPT
    _qctl = (n > 5 && d[5] > 32) ? (char)d[5] : '#';

    if (n > 6) {
        uint8_t q = d[6];
        if (q == 'Y')                 _qbin = '&';      // peer agrees to ours
        else if (q > 32 && q != 'N')  _qbin = (char)q;  // peer's quote char
        else                          _qbin = 0;
    } else _qbin = 0;

    if (n > 8) {
        uint8_t r = d[8];
        _rept = (r > 32 && r != 'N') ? (char)r : 0;
    } else _rept = 0;
}

// ---------------------------------------------------------------------------
// ACK wait (for packets we send: S / F / D / Z / B)
// ---------------------------------------------------------------------------
int KermitClient::waitAck(uint8_t seq, bool grabParams) {
    char    type;
    uint8_t rseq;
    uint8_t data[MAX_DATA + 4];
    int dl = recvPacket(type, rseq, data, sizeof(data), PKT_TIMEOUT);
    if (dl < 0) return -1;
    if (type == 'E') { setErr("host error"); return -1; }
    if (type == 'Y' && rseq == (seq & 0x3F)) {
        if (grabParams && dl > 0) parsePeerParams(data, dl);
        return 1;
    }
    if (type == 'N') return 0;       // NAK -> retry
    return 0;                        // unexpected -> treat as retry
}

bool KermitClient::sendReliable(char type, uint8_t seq, const uint8_t* d,
                                size_t n, bool grabParams, const char* errMsg) {
    for (int t = 0; t < MAX_RETRY; t++) {
        sendPacket(type, seq, d, n);
        if (waitAck(seq, grabParams) > 0) return true;   // ACK (NAK/timeout -> retry)
    }
    setErr(errMsg);
    return false;
}

// ---------------------------------------------------------------------------
// receive a server "send" transaction -> sink (GET, or remote-command text)
// ---------------------------------------------------------------------------
bool KermitClient::receiveStream(KermitDataSink sink, void* ctx) {
    char    type;
    uint8_t seq;
    uint8_t data[MAX_DATA + 4];
    uint8_t dec[MAX_DATA + 4];
    int     lastSeq = -1;
    int     retries = 0;

    for (;;) {
        int dl = recvPacket(type, seq, data, sizeof(data), PKT_TIMEOUT);
        if (dl < 0) {
            if (++retries > MAX_RETRY) { setErr("recv timeout"); return false; }
            // NAK the next expected sequence to prod a resend
            sendPacket('N', (uint8_t)((lastSeq + 1) & 0x3F), nullptr, 0);
            continue;
        }
        retries = 0;

        if (type == 'E') { setErr("host error"); return false; }

        bool dup = ((int)seq == lastSeq);

        switch (type) {
        case 'S':                                  // send-init: negotiate
            parsePeerParams(data, dl);
            { String p = buildInitParams();
              sendPacket('Y', seq, (const uint8_t*)p.c_str(), p.length()); }
            break;
        case 'F':                                  // file header
        case 'X':                                  // text/screen header
            sendPacket('Y', seq, nullptr, 0);
            break;
        case 'D':                                  // data
            if (!dup) {
                size_t n = decodeData(data, dl, dec, sizeof(dec));
                _bytes += n;
                if (sink && !sink(dec, n, ctx)) {
                    sendPacket('E', seq, (const uint8_t*)"abort", 5);
                    setErr("aborted");
                    return false;
                }
            }
            sendPacket('Y', seq, nullptr, 0);
            break;
        case 'Z':                                  // EOF
            sendPacket('Y', seq, nullptr, 0);
            break;
        case 'B':                                  // break / end of transaction
            sendPacket('Y', seq, nullptr, 0);
            return true;
        case 'Y':                                  // stray ack of our command
            break;
        default:
            sendPacket('Y', seq, nullptr, 0);
            break;
        }
        lastSeq = seq;
    }
}

// ---------------------------------------------------------------------------
// public: GET (download a file from host)
// ---------------------------------------------------------------------------
bool KermitClient::get(const char* remoteName, KermitDataSink sink, void* ctx) {
    beginOp();
    // GET command = Receive-Init 'R' carrying the filename.
    sendPacket('R', 0, (const uint8_t*)remoteName, strlen(remoteName));
    return receiveStream(sink, ctx);
}

// ---------------------------------------------------------------------------
// public: PUT (upload a file to host)
// ---------------------------------------------------------------------------
bool KermitClient::put(const char* remoteName, KermitDataSource src, void* ctx) {
    beginOp('&');   // our send-side 8-bit quoting

    uint8_t enc[MAX_DATA + 4];
    uint8_t raw[RAW_CHUNK];

    // 1) Send-Init (negotiate from the ACK)
    String p = buildInitParams();
    if (!sendReliable('S', 0, (const uint8_t*)p.c_str(), p.length(), true, "no S ack"))
        return false;

    uint8_t seq = 1;

    // 2) File header
    if (!sendReliable('F', seq, (const uint8_t*)remoteName, strlen(remoteName),
                      false, "no F ack"))
        return false;
    seq = (seq + 1) & 0x3F;

    // 3) Data packets
    for (;;) {
        int got = src(raw, sizeof(raw), ctx);
        if (got < 0) { setErr("read error"); return false; }
        if (got == 0) break;
        size_t n = encodeData(raw, got, enc, sizeof(enc));
        _bytes += got;
        if (!sendReliable('D', seq, enc, n, false, "no D ack")) return false;
        seq = (seq + 1) & 0x3F;
    }

    // 4) EOF, then 5) Break (end of transaction)
    if (!sendReliable('Z', seq, nullptr, 0, false, "no Z ack")) return false;
    seq = (seq + 1) & 0x3F;
    return sendReliable('B', seq, nullptr, 0, false, "no B ack");
}

// ---------------------------------------------------------------------------
// remote commands: collect server's text reply into `out`
// ---------------------------------------------------------------------------
namespace {
    struct StrSink { String* s; };
    bool strSinkFn(const uint8_t* d, size_t n, void* ctx) {
        String* s = ((StrSink*)ctx)->s;
        for (size_t i = 0; i < n; i++) s->concat((char)d[i]);
        return s->length() < 8192;   // cap reply size
    }
}

bool KermitClient::remoteDelete(const char* spec, String& out) {
    beginOp();
    String d = "E"; if (spec && *spec) d += spec;     // generic: Erase
    sendPacket('G', 0, (const uint8_t*)d.c_str(), d.length());
    StrSink ss{ &out };
    return receiveStream(strSinkFn, &ss);
}

bool KermitClient::hostCommand(const char* cmd, String& out) {
    beginOp();
    sendPacket('C', 0, (const uint8_t*)cmd, strlen(cmd));  // host command
    StrSink ss{ &out };
    return receiveStream(strSinkFn, &ss);
}

bool KermitClient::finishServer() {
    beginOp();
    sendPacket('G', 0, (const uint8_t*)"F", 1);       // generic: Finish
    char type; uint8_t seq; uint8_t data[8];
    return recvPacket(type, seq, data, sizeof(data), PKT_TIMEOUT) >= 0;
}
