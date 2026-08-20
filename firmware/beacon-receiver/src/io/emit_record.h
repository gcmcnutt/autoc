/* emit_record.h — T059/T061: the binary record emitter and its transports.
 * The record is a versioned STRUCT (R13); every sink carries the same bytes. Transports are a plug —
 * when the xiao and the receiver merge into one box, this file is deleted, not redesigned. */
#ifndef BEACON_EMIT_RECORD_H
#define BEACON_EMIT_RECORD_H

#include "record.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct BcnEmitter BcnEmitter;

/* spec ::= binary:<path> | json:- | tcp:<host>:<port> | serial:<dev>[:<baud>]
 * tcp LISTENS (the scope dials in) — the daemon must not depend on a client existing; records emitted
 * with no client connected are dropped at the socket, which is fine: the client sees loss by seq gap. */
int  bcn_emitter_open(BcnEmitter **out, const char *spec, char *err, size_t err_len);
int  bcn_emitter_send(BcnEmitter *e, const BcnRecord *rec);
void bcn_emitter_close(BcnEmitter *e);

#ifdef __cplusplus
}
#endif
#endif
