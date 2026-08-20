/* emit_json.h — T060: JSON-lines projection of the binary record. */
#ifndef BEACON_EMIT_JSON_H
#define BEACON_EMIT_JSON_H

#include "record.h"

#ifdef __cplusplus
extern "C" {
#endif

int bcn_emit_json(int fd, const BcnRecord *rec);

#ifdef __cplusplus
}
#endif
#endif
