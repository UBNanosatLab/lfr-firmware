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

#define RBUF_SIZE 512
#include <stdint.h>
#include "error.h"

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
