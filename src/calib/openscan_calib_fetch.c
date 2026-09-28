#ifndef _WIN32
#define _POSIX_C_SOURCE 200809L
#endif

#include "openscan/openscan_calib.h"

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <direct.h>
#else
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif
#endif

/* Optional. Not shipped. OPENSCAN_CALIB_SIGN, or one line in calib/sign.local.
 * Without it, the vendor lookup is skipped and the user supplies calib/<serial>.txt. */
static int load_sign_secret(char *out, size_t n) {
    const char *env = getenv("OPENSCAN_CALIB_SIGN");
    if (env && env[0] && strlen(env) < n) {
        snprintf(out, n, "%s", env);
        return 0;
    }
    const char *paths[] = {"calib/sign.local", "openscan/calib/sign.local"};
    for (size_t i = 0; i < sizeof paths / sizeof paths[0]; i++) {
        FILE *f = fopen(paths[i], "r");
        if (!f) continue;
        if (!fgets(out, (int)n, f)) {
            fclose(f);
            continue;
        }
        fclose(f);
        size_t len = strlen(out);
        while (len > 0 && (out[len - 1] == '\n' || out[len - 1] == '\r' || out[len - 1] == ' '))
            out[--len] = 0;
        if (len > 0) return 0;
    }
    out[0] = 0;
    return -1;
}

static const char *kHosts[] = {
    "https://sw.3dyunzhan.com/v2/swift/calibration/find",
    "https://swcn.3dyunzhan.com/v2/swift/calibration/find",
};

struct Mem {
    char *data;
    size_t n;
    size_t cap;
};

static size_t mem_write(char *ptr, size_t size, size_t nmemb, void *ud) {
    struct Mem *m = ud;
    size_t add = size * nmemb;
    if (m->n + add + 1 > m->cap) {
        size_t cap = m->cap ? m->cap * 2 : 4096;
        while (cap < m->n + add + 1) cap *= 2;
        if (cap > 2 * 1024 * 1024) return 0;
        char *p = realloc(m->data, cap);
        if (!p) return 0;
        m->data = p;
        m->cap = cap;
    }
    memcpy(m->data + m->n, ptr, add);
    m->n += add;
    m->data[m->n] = 0;
    return add;
}

/* MD5, so the optional lookup can sign a serial without libcrypto. */
static uint32_t md5_rol(uint32_t x, int c) { return (x << c) | (x >> (32 - c)); }

static void md5_block(uint32_t s[4], const uint8_t blk[64]) {
    static const uint32_t K[64] = {
        0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee, 0xf57c0faf, 0x4787c62a, 0xa8304613, 0xfd469501,
        0x698098d8, 0x8b44f7af, 0xffff5bb1, 0x895cd7be, 0x6b901122, 0xfd987193, 0xa679438e, 0x49b40821,
        0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa, 0xd62f105d, 0x02441453, 0xd8a1e681, 0xe7d3fbc8,
        0x21e1cde6, 0xc33707d6, 0xf4d50d87, 0x455a14ed, 0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a,
        0xfffa3942, 0x8771f681, 0x6d9d6122, 0xfde5380c, 0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70,
        0x289b7ec6, 0xeaa127fa, 0xd4ef3085, 0x04881d05, 0xd9d4d039, 0xe6db99e5, 0x1fa27cf8, 0xc4ac5665,
        0xf4292244, 0x432aff97, 0xab9423a7, 0xfc93a039, 0x655b59c3, 0x8f0ccc92, 0xffeff47d, 0x85845dd1,
        0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1, 0xf7537e82, 0xbd3af235, 0x2ad7d2bb, 0xeb86d391};
    static const uint8_t Sft[64] = {
        7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22,
        5, 9, 14, 20, 5, 9, 14, 20, 5, 9, 14, 20, 5, 9, 14, 20,
        4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23,
        6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21};
    uint32_t M[16];
    for (int i = 0; i < 16; i++) {
        M[i] = (uint32_t)blk[i * 4] | ((uint32_t)blk[i * 4 + 1] << 8) |
               ((uint32_t)blk[i * 4 + 2] << 16) | ((uint32_t)blk[i * 4 + 3] << 24);
    }
    uint32_t a = s[0], b = s[1], c = s[2], d = s[3];
    for (int i = 0; i < 64; i++) {
        uint32_t f, g;
        if (i < 16) {
            f = (b & c) | (~b & d);
            g = (uint32_t)i;
        } else if (i < 32) {
            f = (d & b) | (~d & c);
            g = (5u * (uint32_t)i + 1u) % 16u;
        } else if (i < 48) {
            f = b ^ c ^ d;
            g = (3u * (uint32_t)i + 5u) % 16u;
        } else {
            f = c ^ (b | ~d);
            g = (7u * (uint32_t)i) % 16u;
        }
        uint32_t t = d;
        d = c;
        c = b;
        b = b + md5_rol(a + f + K[i] + M[g], Sft[i]);
        a = t;
    }
    s[0] += a;
    s[1] += b;
    s[2] += c;
    s[3] += d;
}

static void md5_hex(const char *text, char out[33]) {
    uint32_t s[4] = {0x67452301u, 0xefcdab89u, 0x98badcfeu, 0x10325476u};
    uint8_t blk[64];
    size_t n = strlen(text);
    size_t off = 0;
    while (n - off >= 64) {
        md5_block(s, (const uint8_t *)text + off);
        off += 64;
    }
    size_t rem = n - off;
    memset(blk, 0, sizeof blk);
    memcpy(blk, text + off, rem);
    blk[rem] = 0x80;
    if (rem >= 56) {
        md5_block(s, blk);
        memset(blk, 0, sizeof blk);
    }
    uint64_t bits = (uint64_t)n * 8u;
    for (int i = 0; i < 8; i++) blk[56 + i] = (uint8_t)(bits >> (8 * i));
    md5_block(s, blk);
    for (int i = 0; i < 4; i++) {
        sprintf(out + i * 8, "%02x%02x%02x%02x", s[i] & 255, (s[i] >> 8) & 255,
                (s[i] >> 16) & 255, (s[i] >> 24) & 255);
    }
    out[32] = 0;
}

static int sign_serial(const char *serial, char out[33]) {
    char secret[128];
    char buf[256];
    char once[33];
    if (load_sign_secret(secret, sizeof secret) != 0) return -1;
    snprintf(buf, sizeof buf, "%s%s", serial, secret);
    md5_hex(buf, once);
    md5_hex(once, out);
    return 0;
}

static int mkdir_one(const char *path) {
    if (!path[0]) return 0;
#ifdef _WIN32
    if (path[1] == ':' && path[2] == 0) return 0;
    if (_mkdir(path) != 0 && errno != EEXIST) return -1;
#else
    if (mkdir(path, 0755) != 0 && errno != EEXIST) return -1;
#endif
    return 0;
}

static int mkdir_p(const char *path) {
    char tmp[512];
    snprintf(tmp, sizeof tmp, "%s", path);
    size_t n = strlen(tmp);
    if (n == 0 || n >= sizeof tmp) return -1;
    for (size_t i = 1; i < n; i++) {
        if (tmp[i] != '/' && tmp[i] != '\\') continue;
        char sep = tmp[i];
        tmp[i] = 0;
        if (mkdir_one(tmp) != 0) return -1;
        tmp[i] = sep;
    }
    return mkdir_one(tmp);
}

/* HTTPS stays out of the link. The curl program is used only for this lookup. */
static int url_ok(const char *u) {
    if (!u) return 0;
    if (strncmp(u, "https://", 8) != 0 && strncmp(u, "http://", 7) != 0) return 0;
    size_t n = strlen(u);
    if (n < 12 || n > 1000) return 0;
    for (size_t i = 0; i < n; i++) {
        unsigned char c = (unsigned char)u[i];
        if (c <= 32 || c >= 127) return 0;
    }
    return 1;
}

static int split_status(struct Mem *m, long *status) {
    if (!m->data || m->n < 4) return -1;
    size_t i = m->n;
    while (i > 0 && m->data[i - 1] != '\n') i--;
    if (i == 0) return -1;
    char *end = NULL;
    long code = strtol(m->data + i, &end, 10);
    if (end == m->data + i || code < 0 || code > 999) return -1;
    size_t body = i > 0 ? i - 1 : 0;
    m->n = body;
    m->data[m->n] = 0;
    *status = code;
    return 0;
}

#ifdef _WIN32
static int append_arg(char *cmd, size_t cap, size_t *used, const char *arg) {
    size_t u = *used;
    if (u && u + 1 < cap) cmd[u++] = ' ';
    if (u + 1 >= cap) return -1;
    cmd[u++] = '"';
    for (const char *p = arg; *p; p++) {
        if (*p == '"') {
            if (u + 2 >= cap) return -1;
            cmd[u++] = '\\';
            cmd[u++] = '"';
        } else {
            if (u + 1 >= cap) return -1;
            cmd[u++] = *p;
        }
    }
    if (u + 1 >= cap) return -1;
    cmd[u++] = '"';
    cmd[u] = 0;
    *used = u;
    return 0;
}

static int curl_capture(char *const argv[], struct Mem *m, long *status) {
    char cmd[8192];
    size_t used = 0;
    cmd[0] = 0;
    if (append_arg(cmd, sizeof cmd, &used, "curl.exe") != 0) return -1;
    for (int i = 1; argv[i]; i++) {
        if (append_arg(cmd, sizeof cmd, &used, argv[i]) != 0) return -1;
    }
    SECURITY_ATTRIBUTES sa;
    memset(&sa, 0, sizeof sa);
    sa.nLength = sizeof sa;
    sa.bInheritHandle = TRUE;
    HANDLE rd = NULL, wr = NULL, nul = NULL;
    if (!CreatePipe(&rd, &wr, &sa, 0)) return -1;
    SetHandleInformation(rd, HANDLE_FLAG_INHERIT, 0);
    nul = CreateFileA("NUL", GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE,
                      &sa, OPEN_EXISTING, 0, NULL);
    STARTUPINFOA si;
    memset(&si, 0, sizeof si);
    si.cb = sizeof si;
    si.dwFlags = STARTF_USESTDHANDLES;
    si.hStdInput = nul;
    si.hStdOutput = wr;
    si.hStdError = nul;
    PROCESS_INFORMATION pi;
    memset(&pi, 0, sizeof pi);
    BOOL ok = CreateProcessA(NULL, cmd, NULL, NULL, TRUE, CREATE_NO_WINDOW, NULL, NULL, &si, &pi);
    CloseHandle(wr);
    if (!ok) {
        CloseHandle(rd);
        if (nul) CloseHandle(nul);
        fprintf(stderr, "calibration lookup needs the curl program on PATH\n");
        return -1;
    }
    char buf[4096];
    int overflow = 0;
    DWORD got = 0;
    while (ReadFile(rd, buf, sizeof buf, &got, NULL) && got > 0) {
        if (mem_write(buf, 1, got, m) != got) overflow = 1;
    }
    CloseHandle(rd);
    if (nul) CloseHandle(nul);
    WaitForSingleObject(pi.hProcess, 30000);
    DWORD code = 1;
    GetExitCodeProcess(pi.hProcess, &code);
    CloseHandle(pi.hProcess);
    CloseHandle(pi.hThread);
    if (code == 127 || code == 9009) {
        fprintf(stderr, "calibration lookup needs the curl program on PATH\n");
        return -1;
    }
    if (overflow || split_status(m, status) != 0) return -1;
    return 0;
}
#else
static int curl_capture(char *const argv[], struct Mem *m, long *status) {
    int fd[2];
    if (pipe(fd) != 0) return -1;
    pid_t pid = fork();
    if (pid < 0) {
        close(fd[0]);
        close(fd[1]);
        return -1;
    }
    if (pid == 0) {
        dup2(fd[1], STDOUT_FILENO);
        close(fd[0]);
        close(fd[1]);
        int devnull = open("/dev/null", O_WRONLY);
        if (devnull >= 0) {
            dup2(devnull, STDERR_FILENO);
            close(devnull);
        }
        execvp("curl", argv);
        _exit(127);
    }
    close(fd[1]);
    char buf[4096];
    int overflow = 0;
    ssize_t r;
    while ((r = read(fd[0], buf, sizeof buf)) > 0) {
        if (mem_write(buf, 1, (size_t)r, m) != (size_t)r) overflow = 1;
    }
    close(fd[0]);
    int st = 0;
    if (waitpid(pid, &st, 0) < 0) return -1;
    if (!WIFEXITED(st) || WEXITSTATUS(st) == 127) {
        fprintf(stderr, "calibration lookup needs the curl program on PATH\n");
        return -1;
    }
    if (overflow || split_status(m, status) != 0) return -1;
    return 0;
}
#endif

static int http_call(const char *method, const char *url, const char *body,
                     const char **headers, int nheaders, struct Mem *m, long *status) {
    if (!url_ok(url) || nheaders < 0 || nheaders > 8) return -1;
    char *argv[40];
    int n = 0;
    argv[n++] = "curl";
    argv[n++] = "-sS";
    argv[n++] = "--http1.1";
    argv[n++] = "--max-time";
    argv[n++] = "20";
    argv[n++] = "-L";
    argv[n++] = "--max-redirs";
    argv[n++] = "3";
    argv[n++] = "--proto";
    argv[n++] = "=http,https";
    argv[n++] = "--proto-redir";
    argv[n++] = "=http,https";
    argv[n++] = "-A";
    argv[n++] = "openscan";
    argv[n++] = "-X";
    argv[n++] = (char *)method;
    for (int i = 0; i < nheaders; i++) {
        argv[n++] = "-H";
        argv[n++] = (char *)headers[i];
    }
    if (body) {
        argv[n++] = "--data-binary";
        argv[n++] = (char *)body;
    }
    argv[n++] = "-w";
    argv[n++] = "\n%{http_code}";
    argv[n++] = (char *)url;
    argv[n++] = NULL;
    return curl_capture(argv, m, status);
}

static int http_get(const char *url, struct Mem *m) {
    long status = 0;
    if (http_call("GET", url, NULL, NULL, 0, m, &status) != 0) return -1;
    return (status >= 200 && status < 300) ? 0 : -1;
}

static int lookup_url(const char *serial, char *url_out, size_t url_n) {
    char sign[33];
    if (sign_serial(serial, sign) != 0) return -1;
    char body[128];
    snprintf(body, sizeof body, "{\"sn\":\"%s\"}", serial);
    char sn_hdr[96], sign_hdr[80];
    snprintf(sn_hdr, sizeof sn_hdr, "sn: %s", serial);
    snprintf(sign_hdr, sizeof sign_hdr, "sign: %s", sign);
    const char *headers[] = {
        "Content-Type: application/json;charset=utf-8",
        "language: en",
        sn_hdr,
        sign_hdr,
    };
    int ok = -1;
    for (size_t i = 0; i < sizeof kHosts / sizeof kHosts[0]; i++) {
        struct Mem m = {0};
        long status = 0;
        /* Lookup only. The calibration file is never uploaded. */
        int rc = http_call("POST", kHosts[i], body, headers, 4, &m, &status);
        if (rc == 0 && status == 200 && m.data && strstr(m.data, "\"code\":\"200\"")) {
            const char *key = strstr(m.data, "\"uploadUrl\"");
            const char *q1 = key ? strchr(key + 11, '"') : NULL;
            const char *http = q1 ? strstr(q1, "http") : NULL;
            const char *q2 = http ? strchr(http, '"') : NULL;
            if (http && q2 && (size_t)(q2 - http) < url_n) {
                memcpy(url_out, http, (size_t)(q2 - http));
                url_out[q2 - http] = 0;
                ok = 0;
                free(m.data);
                break;
            }
        }
        free(m.data);
    }
    return ok;
}

static int looks_like_calib(const char *text) {
    return openscan_calib_text_ok(text);
}

/* Same characters as Android CalibName.safeSerial, capped so it fits calib.serial. */
static int serial_safe(const char *serial) {
    if (!serial || !serial[0]) return 0;
    size_t n = strlen(serial);
    if (n > 63) return 0;
    for (size_t i = 0; i < n; i++) {
        unsigned char c = (unsigned char)serial[i];
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
            (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.')
            continue;
        return 0;
    }
    return 1;
}

int openscan_calib_fetch(const char *serial, const char *dest_path) {
    if (!serial_safe(serial) || !dest_path) return -1;
    char url[1024];
    if (lookup_url(serial, url, sizeof url) != 0) {
        fprintf(stderr, "no factory calibration on the server\n");
        return -1;
    }
    struct Mem file = {0};
    if (http_get(url, &file) != 0 || !looks_like_calib(file.data)) {
        fprintf(stderr, "calibration download was not a calib file\n");
        free(file.data);
        return -1;
    }
    char dir[512];
    snprintf(dir, sizeof dir, "%s", dest_path);
    char *slash = strrchr(dir, '/');
    if (slash) {
        *slash = 0;
        if (dir[0] && mkdir_p(dir) != 0) {
            free(file.data);
            return -1;
        }
    }
    char tmp[560];
    snprintf(tmp, sizeof tmp, "%s.part", dest_path);
    FILE *fp = fopen(tmp, "wb");
    if (!fp) {
        free(file.data);
        return -1;
    }
    size_t wr = fwrite(file.data, 1, file.n, fp);
    fclose(fp);
    free(file.data);
    if (wr != file.n) {
        remove(tmp);
        return -1;
    }
    /* Keep the download only when the file is this unit, not another DevID. */
    openscan_calib got;
    if (openscan_calib_load_for(tmp, serial, &got) != 0) {
        remove(tmp);
        fprintf(stderr, "calibration file is not for this serial\n");
        return -1;
    }
    if (rename(tmp, dest_path) != 0) {
        remove(tmp);
        return -1;
    }
    return 0;
}

/* AppImage and a prefix install keep factory files beside the program:
 * usr/bin/openscan and usr/share/openscan/calib/. */
static int exe_calib_path(const char *serial, char *out, size_t n) {
    char exe[512];
#ifdef _WIN32
    DWORD len = GetModuleFileNameA(NULL, exe, sizeof exe);
    if (len == 0 || len >= sizeof exe) return -1;
    char *slash = strrchr(exe, '\\');
    if (!slash) slash = strrchr(exe, '/');
#elif defined(__APPLE__)
    uint32_t len = sizeof exe;
    if (_NSGetExecutablePath(exe, &len) != 0) return -1;
    char *slash = strrchr(exe, '/');
#else
    ssize_t len = readlink("/proc/self/exe", exe, sizeof exe - 1);
    if (len <= 0) return -1;
    exe[len] = 0;
    char *slash = strrchr(exe, '/');
#endif
    if (!slash) return -1;
    *slash = 0;
    return snprintf(out, n, "%s/../share/openscan/calib/%s.txt", exe, serial) > 0 ? 0 : -1;
}

static int xdg_calib_path(const char *serial, char *out, size_t n) {
#ifdef _WIN32
    const char *base = getenv("APPDATA");
    if (!base || !base[0]) base = getenv("USERPROFILE");
    if (!base || !base[0]) return -1;
    return snprintf(out, n, "%s/openscan/calib/%s.txt", base, serial) > 0 ? 0 : -1;
#elif defined(__APPLE__)
    const char *home = getenv("HOME");
    if (!home || !home[0]) return -1;
    return snprintf(out, n, "%s/Library/Application Support/openscan/calib/%s.txt", home, serial) > 0 ? 0 : -1;
#else
    const char *xdg = getenv("XDG_DATA_HOME");
    const char *home = getenv("HOME");
    if (xdg && xdg[0])
        return snprintf(out, n, "%s/openscan/calib/%s.txt", xdg, serial) > 0 ? 0 : -1;
    if (!home || !home[0]) return -1;
    return snprintf(out, n, "%s/.local/share/openscan/calib/%s.txt", home, serial) > 0 ? 0 : -1;
#endif
}

static int try_load(const char *path, openscan_calib *out, char *used, size_t used_n) {
    if (openscan_calib_load(path, out) != 0) return -1;
    if (used && used_n) snprintf(used, used_n, "%s", path);
    return 0;
}

static int try_load_for(const char *path, const char *serial, openscan_calib *out,
                        char *used, size_t used_n) {
    if (openscan_calib_load_for(path, serial, out) != 0) return -1;
    if (used && used_n) snprintf(used, used_n, "%s", path);
    return 0;
}

int openscan_calib_ensure(const char *serial, const char *explicit_path,
                     openscan_calib *out, char *used_path, size_t used_n) {
    if (!out) return -1;
    if (explicit_path && explicit_path[0])
        return try_load(explicit_path, out, used_path, used_n);
    if (!serial_safe(serial)) return -1;

    char path[512];
    const char *rels[] = {"calib/%s.txt", "openscan/calib/%s.txt"};
    for (size_t i = 0; i < sizeof rels / sizeof rels[0]; i++) {
        snprintf(path, sizeof path, rels[i], serial);
        if (try_load_for(path, serial, out, used_path, used_n) == 0) return 0;
    }
#ifdef OPENSCAN_CALIB_DIR
    snprintf(path, sizeof path, OPENSCAN_CALIB_DIR "/%s.txt", serial);
    if (try_load_for(path, serial, out, used_path, used_n) == 0) return 0;
#endif
    if (exe_calib_path(serial, path, sizeof path) == 0 &&
        try_load_for(path, serial, out, used_path, used_n) == 0)
        return 0;
    if (xdg_calib_path(serial, path, sizeof path) == 0 &&
        try_load_for(path, serial, out, used_path, used_n) == 0)
        return 0;

    if (xdg_calib_path(serial, path, sizeof path) != 0) return -1;
    fprintf(stderr, "fetching factory calibration\n");
    if (openscan_calib_fetch(serial, path) != 0) return -1;
    if (try_load_for(path, serial, out, used_path, used_n) != 0) return -1;
    fprintf(stderr, "saved factory calibration\n");
    return 0;
}

#ifdef OPENSCAN_HTTP_SELFTEST
int main(void) {
    struct Mem m = {0};
    long status = 0;
    if (http_call("GET", "https://example.com/", NULL, NULL, 0, &m, &status) != 0) return 1;
    if (status != 200 || !m.data || !strstr(m.data, "Example Domain")) {
        fprintf(stderr, "status %ld bytes %zu\n", status, m.n);
        return 2;
    }
    puts("http ok");
    free(m.data);
    return 0;
}
#endif

#ifdef OPENSCAN_MD5_SELFTEST
int main(int argc, char **argv) {
    char out[33];
    if (argc > 1) {
        md5_hex(argv[1], out);
        puts(out);
        return 0;
    }
    md5_hex("", out);
    if (strcmp(out, "d41d8cd98f00b204e9800998ecf8427e") != 0) return 1;
    md5_hex("abc", out);
    if (strcmp(out, "900150983cd24fb0d6963f7d28e17f72") != 0) return 2;
    puts("md5 ok");
    return 0;
}
#endif

#ifdef OPENSCAN_CALIB_FETCH_TEST
int main(int argc, char **argv) {
    const char *sn = argc > 1 ? argv[1] : "EXAMPLE01";
    const char *dest = argc > 2 ? argv[2] : "/tmp/openscan-calib-fetch.txt";
    if (openscan_calib_fetch(sn, dest) != 0) return 1;
    openscan_calib cal;
    if (openscan_calib_load(dest, &cal) != 0) return 2;
    printf("ok %s %s baseline %.2f\n", cal.serial, cal.date, cal.cam[1].tvec[0]);
    return 0;
}
#endif
