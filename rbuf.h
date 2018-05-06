#define RBUF_SIZE 128
#define EOVERFLOW 11
#define EUNDERFLOW 12
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct rbuf{
  volatile char buffer[RBUF_SIZE];
  volatile uint16_t head;
  volatile uint16_t tail;
} rbuf;

void rbuf_init(rbuf* buf);
uint16_t rbuf_size(rbuf* buf);
int rbuf_put(rbuf* buf, char c);
int rbuf_get(rbuf* buf, char* c);

#ifdef __cplusplus
}
#endif
