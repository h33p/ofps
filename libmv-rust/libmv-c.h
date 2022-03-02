
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif
    double fundamental_from_correspondences_8_point_robust(const double* x1,
            const double* x2,
            size_t x_rows,
            double max_error,
            double* F,
            int* inliers,
            size_t* inliers_sz,
            double outliers_probability);

    double fundamental_from_correspondences_7_point_robust(const double* x1,
            const double* x2,
            size_t x_rows,
            double max_error,
            double* F,
            int* inliers,
            size_t* inliers_sz,
            double outliers_probability);

    int motion_from_essential_and_correspondence(
            const double *E,
            const double *K1,
            const double *x1,
            const double *K2,
            const double *x2,
            double *R,
            double *t);
#ifdef __cplusplus
}
#endif
