/* emit_record.c — T059 binary emitter + T061 tcp/serial transports + the json:- passthrough (which
 * calls emit_json.c: the JSON is a PROJECTION of the binary record, never a second source of truth). */
#define _GNU_SOURCE
#include "emit_record.h"
#include "emit_json.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

typedef enum { SK_BINARY, SK_JSON, SK_TCP, SK_SERIAL } SinkKind;

struct BcnEmitter {
    SinkKind kind;
    int fd;                    /* file / serial / tcp LISTEN socket                                    */
    int client;                /* tcp: the connected scope, -1 = none                                  */
};

static int open_tcp_listen(const char *host, uint16_t port, char *err, size_t err_len)
{
    int fd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
    struct sockaddr_in sa;
    int one = 1;
    if (fd < 0) { snprintf(err, err_len, "emit: socket: %s", strerror(errno)); return -1; }
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    memset(&sa, 0, sizeof sa);
    sa.sin_family = AF_INET;
    sa.sin_port = htons(port);
    sa.sin_addr.s_addr = (host && strcmp(host, "0.0.0.0") != 0) ? inet_addr(host) : INADDR_ANY;
    if (bind(fd, (struct sockaddr *)&sa, sizeof sa) != 0 || listen(fd, 1) != 0) {
        snprintf(err, err_len, "emit: cannot listen on %s:%u: %s", host ? host : "*", port, strerror(errno));
        close(fd);
        return -1;
    }
    return fd;
}

int bcn_emitter_open(BcnEmitter **out, const char *spec, char *err, size_t err_len)
{
    BcnEmitter *e = calloc(1, sizeof *e);
    if (!e) { snprintf(err, err_len, "emit: out of memory"); return -1; }
    e->client = -1;

    if (strncmp(spec, "binary:", 7) == 0) {
        e->kind = SK_BINARY;
        e->fd = open(spec + 7, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (e->fd < 0) { snprintf(err, err_len, "emit: %s: %s", spec + 7, strerror(errno)); free(e); return -1; }
    } else if (strcmp(spec, "json:-") == 0) {
        e->kind = SK_JSON;
        e->fd = STDOUT_FILENO;
    } else if (strncmp(spec, "tcp:", 4) == 0) {
        char host[64] = "0.0.0.0";
        unsigned port = 0;
        const char *p = spec + 4, *colon = strrchr(p, ':');
        if (!colon || sscanf(colon + 1, "%u", &port) != 1 || port == 0 || port > 65535) {
            snprintf(err, err_len, "emit: tcp spec \"%s\" is not tcp:<host>:<port>", spec);
            free(e); return -1;
        }
        if (colon > p && (size_t)(colon - p) < sizeof host) {
            memcpy(host, p, (size_t)(colon - p));
            host[colon - p] = '\0';
        }
        e->kind = SK_TCP;
        e->fd = open_tcp_listen(host, (uint16_t)port, err, err_len);
        if (e->fd < 0) { free(e); return -1; }
    } else if (strncmp(spec, "serial:", 7) == 0) {
        char dev[128];
        unsigned baud = 115200;
        const char *p = spec + 7, *colon = strrchr(p, ':');
        struct termios tio;
        if (colon && sscanf(colon + 1, "%u", &baud) == 1 && (size_t)(colon - p) < sizeof dev) {
            memcpy(dev, p, (size_t)(colon - p)); dev[colon - p] = '\0';
        } else {
            snprintf(dev, sizeof dev, "%s", p);
        }
        e->kind = SK_SERIAL;
        e->fd = open(dev, O_WRONLY | O_NOCTTY);
        if (e->fd < 0) { snprintf(err, err_len, "emit: %s: %s", dev, strerror(errno)); free(e); return -1; }
        if (tcgetattr(e->fd, &tio) == 0) {
            cfmakeraw(&tio);
            cfsetospeed(&tio, baud == 921600 ? B921600 : baud == 460800 ? B460800 :
                              baud == 230400 ? B230400 : B115200);
            tcsetattr(e->fd, TCSANOW, &tio);
        }
    } else {
        snprintf(err, err_len, "emit: unknown sink \"%s\" (binary:|json:-|tcp:|serial:)", spec);
        free(e); return -1;
    }
    *out = e;
    return 0;
}

int bcn_emitter_send(BcnEmitter *e, const BcnRecord *rec)
{
    uint8_t wire[BCN_RECORD_WIRE_BYTES];

    if (e->kind == SK_JSON)
        return bcn_emit_json(e->fd, rec);

    if (e->kind == SK_TCP) {
        if (e->client < 0) {
            e->client = accept4(e->fd, NULL, NULL, SOCK_NONBLOCK);
            if (e->client >= 0) {
                int one = 1;
                setsockopt(e->client, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
            }
        }
        if (e->client < 0) return 0;           /* no scope connected: drop; seq gap tells the story   */
        bcn_record_encode(rec, wire);
        if (write(e->client, wire, sizeof wire) != (ssize_t)sizeof wire) {
            close(e->client);                  /* client gone or too slow — never block the daemon    */
            e->client = -1;
        }
        return 0;
    }

    bcn_record_encode(rec, wire);
    return write(e->fd, wire, sizeof wire) == (ssize_t)sizeof wire ? 0 : -1;
}

void bcn_emitter_close(BcnEmitter *e)
{
    if (!e) return;
    if (e->client >= 0) close(e->client);
    if (e->fd >= 0 && e->fd != STDOUT_FILENO) close(e->fd);
    free(e);
}
