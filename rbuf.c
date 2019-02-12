/* Little Free Radio - An Open Source Radio for CubeSats
 * Copyright (C) 2018 Grant Iraci, Brian Bezanson
 * A project of the University at Buffalo Nanosatellite Laboratory
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "rbuf.h"
#include "error.h"

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
