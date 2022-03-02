
#include "libmv-c.h"
#include <libmv/multiview/robust_fundamental.h>
#include <libmv/multiview/fundamental.h>
#include <stdio.h>

using namespace libmv;

extern "C" double fundamental_from_correspondences_8_point_robust(const double* x1,
    const double* x2,
    size_t x_rows,
    double max_error,
    double* F,
    int* inliers,
    size_t* inliers_sz,
    double outliers_probability)
{
    Map<const Mat> xm1(x1, 2, x_rows), xm2(x2, 2, x_rows);
    Mat3 Fm;

    vector<int> inliers_vec;

    double ret = FundamentalFromCorrespondences8PointRobust(xm1, xm2, max_error, &Fm, &inliers_vec, outliers_probability);

    for (size_t y = 0; y < 3; y++) {
        for (size_t x = 0; x < 3; x++) {
            F[y * 3 + x] = Fm(y, x);
        }
    }

    if (inliers) {
        memcpy(inliers, inliers_vec.data(), inliers_vec.size());
        *inliers_sz = inliers_vec.size();
    }

    return ret;
}

extern "C" double fundamental_from_correspondences_7_point_robust(const double* x1,
    const double* x2,
    size_t x_rows,
    double max_error,
    double* F,
    int* inliers,
    size_t* inliers_sz,
    double outliers_probability)
{
    Map<const Mat> xm1(x1, 2, x_rows), xm2(x2, 2, x_rows);
    Mat3 Fm;

    vector<int> inliers_vec;

    double ret = FundamentalFromCorrespondences7PointRobust(xm1, xm2, max_error, &Fm, &inliers_vec, outliers_probability);

    for (size_t y = 0; y < 3; y++) {
        for (size_t x = 0; x < 3; x++) {
            F[y * 3 + x] = Fm(y, x);
        }
    }

    if (inliers) {
        memcpy(inliers, inliers_vec.data(), inliers_vec.size());
        *inliers_sz = inliers_vec.size();
    }

    return ret;
}

extern "C" int motion_from_essential_and_correspondence(
    const double *E,
    const double *K1,
    const double *x1,
    const double *K2,
    const double *x2,
    double *R,
    double *t)
{

    Mat3 Rm;
    Vec3 tm;

    bool ret = MotionFromEssentialAndCorrespondence(
        Map<const Mat3>(E),
        Map<const Mat3>(K1),
        Map<const Vec2>(x1),
        Map<const Mat3>(K2),
        Map<const Vec2>(x2),
        &Rm,
        &tm
    );

    for (size_t y = 0; y < 3; y++) {
        for (size_t x = 0; x < 3; x++) {
            R[y * 3 + x] = Rm(y, x);
        }
    }

    for (size_t x = 0; x < 3; x++) {
        t[x] = tm(x);
    }

    return ret;
}
