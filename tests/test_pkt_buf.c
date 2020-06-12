#include <stdint.h>
#include <stdio.h>

#include "../pkt_buf.h"

void assert_fail(const char *file, int line, const char *func, const char *test, const char *msg)
{
    printf("%s:%d: Assertion %s failed in %s: %s\n", file, line, test, func, msg);
}

#define ASSERT(x, msg) {if (!(x)){ assert_fail(__FILE__, __LINE__, __func__, #x, msg); return 1;}}

uint16_t chksum(uint8_t *data, int len) {
  uint8_t lsb = 0, msb = 0;
  for (int i = 0; i < len; i++) {
    msb += data[i];
    lsb += msb;
  }
  return ((uint16_t) msb<<8) | (uint16_t)lsb;
}

int test_stress()
{
    const int BUF_SIZE = 16384;
    const int PKT_SIZE = 255;

    uint8_t mem[BUF_SIZE + 2*sizeof(uint64_t)] = {0};
    uint64_t *guard_front = (uint64_t *)mem;
    uint8_t *backing_buf = mem + sizeof(uint64_t);
    uint64_t *guard_back = (uint64_t *)(mem + sizeof(uint64_t) + BUF_SIZE);

    struct pkt_buf buf;

    int err = 0;

    *guard_front = 0xDEADBEEF;
    *guard_back = 0xFEEBDAED;

    err = pkt_buf_init(&buf, BUF_SIZE, backing_buf);
    ASSERT(err == 0, "Failed to init buffer");

    uint8_t msg[PKT_SIZE];

    for (int i = 0; i < PKT_SIZE; i++) {
        msg[i] = (uint8_t)(i & 0xFF);
    }

    const uint16_t correct = chksum(msg, PKT_SIZE);

    while (!pkt_buf_enqueue(&buf, PKT_SIZE, msg));
    
    for (int i = 0; i < 100; i++) {
        uint8_t data[PKT_SIZE] = {0};
        int len = PKT_SIZE;
        err = pkt_buf_dequeue(&buf, &len, data);
        ASSERT(err == 0, "Failed to dequeue");
        
        uint16_t chk = chksum(data, len);
        ASSERT(chk == correct, "Checksum mismatch");

        pkt_buf_enqueue(&buf, PKT_SIZE, msg);

        ASSERT(*guard_front == 0xDEADBEEF, "Front guard corrupted");
        ASSERT(*guard_back == 0xFEEBDAED, "Back guard corrupted");
    }

    uint8_t data[PKT_SIZE] = {0};
    int len = PKT_SIZE;
    while (!pkt_buf_dequeue(&buf, &len, data)) {
        uint16_t chk = chksum(data, len);
        ASSERT(chk == correct, "Checksum mismatch");
        len = PKT_SIZE;
    }

    ASSERT(*guard_front == 0xDEADBEEF, "Front guard corrupted");
    ASSERT(*guard_back == 0xFEEBDAED, "Back guard corrupted");

    return 0;
}


int main(int argc, char const *argv[])
{
    return test_stress();
}
