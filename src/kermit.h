/*
 * Minimal Kermit protocol CLIENT for ESP32, talking to a host (HP 200LX)
 * running MS-Kermit in SERVER mode over a serial Stream (Serial1, 9600 8N1).
 *
 * Supports: GET (download), PUT (upload), and REMOTE commands (DIR, DELETE,
 * CWD, host command). Type-1 (1-char) checksum, control-quoting, 8-bit quoting
 * and repeat-prefix decode. Short packets only (<=94 bytes). Lock-step ACK/NAK.
 *
 * The host side must already be at the Kermit prompt with `SERVER` issued.
 */
#pragma once
#include <Arduino.h>

// Streaming callbacks so large files never sit fully in RAM.
//   sink:   receives decoded file bytes; return false to abort.
//   source: fills buf (up to maxLen); returns bytes read, 0 = EOF, <0 = error.
typedef bool (*KermitDataSink)(const uint8_t* data, size_t len, void* ctx);
typedef int  (*KermitDataSource)(uint8_t* buf, size_t maxLen, void* ctx);

class KermitClient {
public:
    explicit KermitClient(Stream& io) : _io(io) {}

    // High-level operations (host must be in SERVER mode).
    bool get(const char* remoteName, KermitDataSink sink, void* ctx);
    bool put(const char* remoteName, KermitDataSource src, void* ctx);
    bool remoteDelete(const char* spec, String& out);  // generic 'E'
    bool hostCommand(const char* cmd, String& out);    // type 'C' (DIR/MD/RD/REN)
    bool finishServer();                               // generic 'F' (exit server)

    const char* lastError()       const { return _err; }
    uint32_t    bytesTransferred() const { return _bytes; }

private:
    Stream&  _io;
    char     _err[48]  = {0};
    uint32_t _bytes    = 0;

    // Negotiated quoting (defaults until a Send-Init is exchanged).
    char _qctl = '#';
    char _qbin = 0;     // 0 = disabled
    char _rept = 0;     // 0 = disabled

    // ---- low level ----
    void     beginOp(char qbin = 0);   // reset per-operation state + flush
    void     drainInput();
    void     setErr(const char* msg);
    uint8_t  chk1(const uint8_t* p, size_t n);
    void     sendPacket(char type, uint8_t seq, const uint8_t* data, size_t len);
    int      recvPacket(char& type, uint8_t& seq, uint8_t* data, size_t maxData,
                        uint32_t timeoutMs);

    size_t   encodeData(const uint8_t* in, size_t inLen, uint8_t* out, size_t outMax);
    size_t   decodeData(const uint8_t* in, size_t inLen, uint8_t* out, size_t outMax);

    String   buildInitParams();                 // our Send-Init / ACK param block
    void     parsePeerParams(const uint8_t* d, size_t n);  // negotiate from peer

    // Send a packet and wait for its ACK, retrying on NAK/timeout.
    bool     sendReliable(char type, uint8_t seq, const uint8_t* d, size_t n,
                          bool grabParams, const char* errMsg);
    // Drive a server "send" transaction (file data or remote-command text).
    bool     receiveStream(KermitDataSink sink, void* ctx);
    // Wait for an ACK of a packet we sent (seq must match); negotiates if its
    // data carries params. Returns 1=ACK, 0=NAK/retry, -1=error/timeout.
    int      waitAck(uint8_t seq, bool grabParams);
};
