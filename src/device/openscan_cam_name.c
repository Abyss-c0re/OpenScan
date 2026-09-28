#include "openscan/openscan_v4l2.h"

#include <string.h>

/* Same characters as the Android safe-serial check. 63 leaves a NUL. */
static int serial_chars(const char *s) {
    size_t n = strlen(s);
    if (n == 0 || n > 63) return 0;
    for (size_t i = 0; i < n; i++) {
        unsigned char c = (unsigned char)s[i];
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
            (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.')
            continue;
        return 0;
    }
    return 1;
}

int openscan_serial_from_camera_name(const char *name, char *out, size_t n) {
    if (!name || !out || n == 0) return -1;
    const char *tag = NULL;
    if (strncmp(name, "KYT Camera A:", 13) == 0) tag = name + 13;
    else if (strncmp(name, "KYT Camera B:", 13) == 0) tag = name + 13;
    else return -1;
    while (*tag == ' ' || *tag == '\t') tag++;
    size_t raw = strlen(tag);
    if (raw == 0 || raw >= n) return -1;
    memcpy(out, tag, raw + 1);
    if (raw >= 2 && out[raw - 2] == '_' && (out[raw - 1] == 'A' || out[raw - 1] == 'B'))
        out[raw - 2] = 0;
    if (!serial_chars(out)) return -1;
    return 0;
}

int openscan_shared_camera_serial(const char *name_a, const char *name_b, char *out, size_t n) {
    char a[68], b[68];
    if (openscan_serial_from_camera_name(name_a, a, sizeof a) != 0) return -1;
    if (openscan_serial_from_camera_name(name_b, b, sizeof b) != 0) return -1;
    if (strcmp(a, b) != 0) return -1;
    if (!out || strlen(a) + 1 > n) return -1;
    memcpy(out, a, strlen(a) + 1);
    return 0;
}
