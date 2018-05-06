#include "rbuf.h"

int rbuf_put(rbuf* buf, char c){
  static uint16_t next;
  next=(buf->head+1)%RBUF_SIZE;
  if(next != buf->tail){
    buf->buffer[buf->head] = c;
    buf->head=next;
    return 0;
  } else {
    return -EOVERFLOW;
  }
}

int rbuf_get(rbuf* buf, char* c){
  if(buf->head != buf->tail){
    *c = buf->buffer[buf->tail];
    buf->tail = (buf->tail+1)%RBUF_SIZE;
    return 0;
  } else {
    return -EUNDERFLOW;
  }
}

uint16_t rbuf_size(rbuf* buf){
  return ((uint16_t)(buf->head-buf->tail))%RBUF_SIZE;
}

void rbuf_init(rbuf* buf){
  buf->head=buf->tail=0;
}

