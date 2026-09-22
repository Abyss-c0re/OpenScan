#include "fox/fox_calib.h"

#include <curl/curl.h>
#include <openssl/evp.h>

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

/* The JMStudio client signs calibration/find as MD5(MD5(serial + this)). */
static const char kSignSecret[] = "REDACTED";

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

static void md5_hex(const char *s, char out[33]) {
    unsigned char dig[EVP_MAX_MD_SIZE];
    unsigned int len = 0;
    EVP_Digest(s, strlen(s), dig, &len, EVP_md5(), NULL);
    for (unsigned int i = 0; i < len && i < 16; i++)
        sprintf(out + i * 2, "%02x", dig[i]);
    out[32] = 0;
}

static void sign_serial(const char *serial, char out[33]) {
    char buf[256];
    char once[33];
    snprintf(buf, sizeof buf, "%s%s", serial, kSignSecret);
    md5_hex(buf, once);
    md5_hex(once, out);
}

static int mkdir_p(const char *path) {
    char tmp[512];
    snprintf(tmp, sizeof tmp, "%s", path);
    size_t n = strlen(tmp);
    if (n == 0 || n >= sizeof tmp) return -1;
    for (size_t i = 1; i < n; i++) {
        if (tmp[i] != '/') continue;
        tmp[i] = 0;
        if (mkdir(tmp, 0755) != 0 && errno != EEXIST) return -1;
        tmp[i] = '/';
    }
    if (mkdir(tmp, 0755) != 0 && errno != EEXIST) return -1;
    return 0;
}

static int http_get(const char *url, struct Mem *m) {
    CURL *c = curl_easy_init();
    if (!c) return -1;
    curl_easy_setopt(c, CURLOPT_URL, url);
    curl_easy_setopt(c, CURLOPT_HTTPGET, 1L);
    curl_easy_setopt(c, CURLOPT_UPLOAD, 0L);
    curl_easy_setopt(c, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(c, CURLOPT_TIMEOUT, 20L);
    curl_easy_setopt(c, CURLOPT_WRITEFUNCTION, mem_write);
    curl_easy_setopt(c, CURLOPT_WRITEDATA, m);
    curl_easy_setopt(c, CURLOPT_USERAGENT, "fox3d");
    CURLcode rc = curl_easy_perform(c);
    long status = 0;
    curl_easy_getinfo(c, CURLINFO_RESPONSE_CODE, &status);
    curl_easy_cleanup(c);
    return (rc == CURLE_OK && status >= 200 && status < 300) ? 0 : -1;
}

static int lookup_url(const char *serial, char *url_out, size_t url_n) {
    char sign[33];
    sign_serial(serial, sign);
    char body[128];
    snprintf(body, sizeof body, "{\"sn\":\"%s\"}", serial);
    struct curl_slist *hdr = NULL;
    hdr = curl_slist_append(hdr, "Content-Type: application/json;charset=utf-8");
    hdr = curl_slist_append(hdr, "language: en");
    char sn_hdr[96], sign_hdr[80];
    snprintf(sn_hdr, sizeof sn_hdr, "sn: %s", serial);
    snprintf(sign_hdr, sizeof sign_hdr, "sign: %s", sign);
    hdr = curl_slist_append(hdr, sn_hdr);
    hdr = curl_slist_append(hdr, sign_hdr);

    int ok = -1;
    for (size_t i = 0; i < sizeof kHosts / sizeof kHosts[0]; i++) {
        struct Mem m = {0};
        CURL *c = curl_easy_init();
        if (!c) break;
        /* Lookup only. The calibration file is never uploaded. */
        curl_easy_setopt(c, CURLOPT_URL, kHosts[i]);
        curl_easy_setopt(c, CURLOPT_UPLOAD, 0L);
        curl_easy_setopt(c, CURLOPT_POST, 1L);
        curl_easy_setopt(c, CURLOPT_POSTFIELDS, body);
        curl_easy_setopt(c, CURLOPT_HTTPHEADER, hdr);
        curl_easy_setopt(c, CURLOPT_TIMEOUT, 20L);
        curl_easy_setopt(c, CURLOPT_WRITEFUNCTION, mem_write);
        curl_easy_setopt(c, CURLOPT_WRITEDATA, &m);
        curl_easy_setopt(c, CURLOPT_USERAGENT, "fox3d");
        CURLcode rc = curl_easy_perform(c);
        long status = 0;
        curl_easy_getinfo(c, CURLINFO_RESPONSE_CODE, &status);
        curl_easy_cleanup(c);
        if (rc == CURLE_OK && status == 200 && m.data && strstr(m.data, "\"code\":\"200\"")) {
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
    curl_slist_free_all(hdr);
    return ok;
}

static int looks_like_calib(const char *text) {
    if (!text || !isdigit((unsigned char)text[0])) return 0;
    return strstr(text, "DevID:") != NULL;
}

int fox_calib_fetch(const char *serial, const char *dest_path) {
    if (!serial || !serial[0] || !dest_path) return -1;
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
    if (wr == 0 || rename(tmp, dest_path) != 0) {
        remove(tmp);
        return -1;
    }
    return 0;
}

static int xdg_calib_path(const char *serial, char *out, size_t n) {
    const char *xdg = getenv("XDG_DATA_HOME");
    const char *home = getenv("HOME");
    if (xdg && xdg[0])
        return snprintf(out, n, "%s/fox3d/calib/%s.txt", xdg, serial) > 0 ? 0 : -1;
    if (!home || !home[0]) return -1;
    return snprintf(out, n, "%s/.local/share/fox3d/calib/%s.txt", home, serial) > 0 ? 0 : -1;
}

static int try_load(const char *path, fox_calib *out, char *used, size_t used_n) {
    if (fox_calib_load(path, out) != 0) return -1;
    if (used && used_n) snprintf(used, used_n, "%s", path);
    return 0;
}

int fox_calib_ensure(const char *serial, const char *explicit_path,
                     fox_calib *out, char *used_path, size_t used_n) {
    if (!out) return -1;
    if (explicit_path && explicit_path[0])
        return try_load(explicit_path, out, used_path, used_n);
    if (!serial || !serial[0]) return -1;

    char path[512];
    const char *rels[] = {"calib/%s.txt", "fox3d/calib/%s.txt"};
    for (size_t i = 0; i < sizeof rels / sizeof rels[0]; i++) {
        snprintf(path, sizeof path, rels[i], serial);
        if (try_load(path, out, used_path, used_n) == 0) return 0;
    }
#ifdef FOX3D_CALIB_DIR
    snprintf(path, sizeof path, FOX3D_CALIB_DIR "/%s.txt", serial);
    if (try_load(path, out, used_path, used_n) == 0) return 0;
#endif
    if (xdg_calib_path(serial, path, sizeof path) == 0 &&
        try_load(path, out, used_path, used_n) == 0)
        return 0;

    if (xdg_calib_path(serial, path, sizeof path) != 0) return -1;
    fprintf(stderr, "fetching factory calibration\n");
    if (fox_calib_fetch(serial, path) != 0) return -1;
    if (try_load(path, out, used_path, used_n) != 0) return -1;
    fprintf(stderr, "saved factory calibration\n");
    return 0;
}

#ifdef FOX_CALIB_FETCH_TEST
int main(int argc, char **argv) {
    const char *sn = argc > 1 ? argv[1] : "EXAMPLE01";
    const char *dest = argc > 2 ? argv[2] : "/tmp/fox-calib-fetch.txt";
    if (fox_calib_fetch(sn, dest) != 0) return 1;
    fox_calib cal;
    if (fox_calib_load(dest, &cal) != 0) return 2;
    printf("ok %s %s baseline %.2f\n", cal.serial, cal.date, cal.cam[1].tvec[0]);
    return 0;
}
#endif
