#ifndef OPENSCAN_APP_H
#define OPENSCAN_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Qt window. Starts idle until the user presses Start scan.
 * calib_path NULL loads or downloads the file for the connected serial. */
int openscan_app_main(int argc, char **argv, const char *calib_path,
                 int exp_a, int gain_a, int exp_b, int gain_b,
                 double scale, double min_mm, double max_mm);

#ifdef __cplusplus
}
#endif

#endif
