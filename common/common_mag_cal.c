#include "common_mag_cal.h"
#include <math.h>
#include <stddef.h>

#define MAG_CAL_MIN_SAMPLES 100
#define MAG_CAL_MIN_POINTS 24
#define MAG_CAL_MIN_BUCKETS 18
#define MAG_CAL_TERMS 9

/* 双精度法方程和批量残差只在完成校准时使用，避免压入sensor任务栈。 */
static struct
{
    double normal[MAG_CAL_TERMS][MAG_CAL_TERMS + 1];
    float residual[COMMON_MAG_CAL_MAX_POINTS], sorted[COMMON_MAG_CAL_MAX_POINTS];
    uint8_t rejected[COMMON_MAG_CAL_MAX_POINTS];
} s_fit;

static float mag_distance_squared(const float a[3], const float b[3])
{
    float x = a[0] - b[0], y = a[1] - b[1], z = a[2] - b[2];
    return x * x + y * y + z * z;
}

static uint8_t mag_direction_bucket(const float v[3])
{
    float threshold = 0.5f * fmaxf(fabsf(v[0]), fmaxf(fabsf(v[1]), fabsf(v[2])));
    uint8_t axis, bucket = 0;
    for (axis = 0; axis < 3; ++axis)
    {
        bucket *= 3;
        if (v[axis] > threshold)
        {
            bucket += 2;
        }
        else if (v[axis] >= -threshold)
        {
            ++bucket;
        }
    }
    return bucket;
}

static uint8_t mag_count_buckets(uint32_t mask)
{
    uint8_t n = 0;
    while (mask != 0)
    {
        mask &= mask - 1;
        ++n;
    }
    return n;
}

static uint8_t mag_live_coverage(const common_mag_cal_struct *cal)
{
    float center[3] = {0}, v[3];
    uint32_t mask = 0;
    uint8_t i, axis;
    for (i = 0; i < cal->count; ++i)
    {
        for (axis = 0; axis < 3; ++axis)
        {
            center[axis] += cal->points[i].mag_uT[axis];
        }
    }
    for (axis = 0; axis < 3; ++axis)
    {
        center[axis] /= cal->count;
    }
    for (i = 0; i < cal->count; ++i)
    {
        for (axis = 0; axis < 3; ++axis)
        {
            v[axis] = cal->points[i].mag_uT[axis] - center[axis];
        }
        mask |= (uint32_t)1 << mag_direction_bucket(v);
    }
    mask &= ~((uint32_t)1 << 13); /* 三轴都为零的桶。 */
    return (uint8_t)(mag_count_buckets(mask) * 100 / 26);
}

void common_mag_cal_collect(common_mag_cal_struct *cal, const float mag_uT[3], const float gravity[3], uint8_t gap)
{
    float nearest = INFINITY, distance;
    uint8_t i, j, axis, target = cal->count;
    uint8_t jump = cal->have_last && !gap && mag_distance_squared(mag_uT, cal->last_uT) > 35.0f * 35.0f;
    for (axis = 0; axis < 3; ++axis)
    {
        cal->last_uT[axis] = mag_uT[axis];
    }
    cal->have_last = 1;
    if (jump)
    {
        return;
    }
    for (axis = 0; axis < 3; ++axis)
    {
        if (cal->samples == 0 || mag_uT[axis] < cal->min_uT[axis])
        {
            cal->min_uT[axis] = mag_uT[axis];
        }
        if (cal->samples == 0 || mag_uT[axis] > cal->max_uT[axis])
        {
            cal->max_uT[axis] = mag_uT[axis];
        }
    }
    if (cal->samples < UINT16_MAX)
    {
        ++cal->samples;
    }
    for (i = 0; i < cal->count; ++i)
    {
        distance = mag_distance_squared(mag_uT, cal->points[i].mag_uT);
        if (distance < 10.0f * 10.0f)
        {
            return;
        }
        nearest = fminf(nearest, distance);
    }
    if (cal->count < COMMON_MAG_CAL_MAX_POINTS)
    {
        ++cal->count;
    }
    else if (nearest > cal->closest_squared)
    {
        /* 只有新方向能拉开点间距离才替换拥挤点，避免停留点权重和跨方向平均。 */
        target = cal->closest_index;
    }
    else
    {
        return;
    }
    for (axis = 0; axis < 3; ++axis)
    {
        cal->points[target].mag_uT[axis] = mag_uT[axis];
        cal->points[target].gravity[axis] = 0.0f;
        if (gravity != NULL)
        {
            cal->points[target].gravity[axis] = gravity[axis];
        }
    }
    if (cal->count == COMMON_MAG_CAL_MAX_POINTS)
    {
        cal->closest_squared = INFINITY;
        for (i = 1; i < cal->count; ++i)
        {
            for (j = 0; j < i; ++j)
            {
                distance = mag_distance_squared(cal->points[i].mag_uT, cal->points[j].mag_uT);
                if (distance < cal->closest_squared)
                {
                    cal->closest_squared = distance;
                    cal->closest_index = i;
                }
            }
        }
    }
    cal->coverage_pct = mag_live_coverage(cal);
}

void common_mag_cal_apply(const common_mag_cal_params_struct *params, const float raw_uT[3], float corrected_uT[3])
{
    const float *w = params->soft_sym;
    float x = raw_uT[0] - params->offset_uT[0];
    float y = raw_uT[1] - params->offset_uT[1];
    float z = raw_uT[2] - params->offset_uT[2];
    corrected_uT[0] = w[0] * x + w[3] * y + w[4] * z;
    corrected_uT[1] = w[3] * x + w[1] * y + w[5] * z;
    corrected_uT[2] = w[4] * x + w[5] * y + w[2] * z;
}

float common_mag_cal_dip_deg(const float mag_uT[3], const float gravity[3])
{
    float dot = mag_uT[0] * gravity[0] + mag_uT[1] * gravity[1] + mag_uT[2] * gravity[2];
    float norm = sqrtf(mag_uT[0] * mag_uT[0] + mag_uT[1] * mag_uT[1] + mag_uT[2] * mag_uT[2]);
    if (!(norm > 0.0f))
    {
        return NAN;
    }
    return 57.2957795f * asinf(fmaxf(-1.0f, fminf(1.0f, dot / norm)));
}

/* n行、n+1列的增广矩阵，按列选绝对值最大的主元。 */
static uint8_t mag_solve(uint8_t n, double *matrix[], double solution[])
{
    uint8_t i, row, col, pivot;
    double value, factor;
    for (i = 0; i < n; ++i)
    {
        pivot = i;
        for (row = i + 1; row < n; ++row)
        {
            if (fabs(matrix[row][i]) > fabs(matrix[pivot][i]))
            {
                pivot = row;
            }
        }
        if (!(fabs(matrix[pivot][i]) > 1e-9))
        {
            return 1;
        }
        for (col = i; col <= n; ++col)
        {
            value = matrix[i][col];
            matrix[i][col] = matrix[pivot][col];
            matrix[pivot][col] = value;
        }
        for (row = i + 1; row < n; ++row)
        {
            factor = matrix[row][i] / matrix[i][i];
            for (col = i + 1; col <= n; ++col)
            {
                matrix[row][col] -= factor * matrix[i][col];
            }
        }
    }
    while (i != 0)
    {
        --i;
        value = matrix[i][n];
        for (col = i + 1; col < n; ++col)
        {
            value -= matrix[i][col] * solution[col];
        }
        solution[i] = value / matrix[i][i];
    }
    return 0;
}

static void mag_eigen(double q[3][3], double v[3][3])
{
    uint8_t iter, p, r, i;
    double tau, t, c, s, x, y;
    for (iter = 0; iter < 24; ++iter)
    {
        p = 0;
        r = 1;
        if (fabs(q[0][2]) > fabs(q[p][r]))
        {
            r = 2;
        }
        if (fabs(q[1][2]) > fabs(q[p][r]))
        {
            p = 1;
            r = 2;
        }
        if (fabs(q[p][r]) < 1e-10)
        {
            break;
        }
        tau = (q[r][r] - q[p][p]) / (2.0 * q[p][r]);
        t = 1.0 / (fabs(tau) + sqrt(1.0 + tau * tau));
        if (tau < 0.0)
        {
            t = -t;
        }
        c = 1.0 / sqrt(1.0 + t * t);
        s = t * c;
        q[p][p] -= t * q[p][r];
        q[r][r] += t * q[p][r];
        q[p][r] = q[r][p] = 0.0;
        for (i = 0; i < 3; ++i)
        {
            if (i != p && i != r)
            {
                x = q[i][p];
                y = q[i][r];
                q[i][p] = q[p][i] = c * x - s * y;
                q[i][r] = q[r][i] = s * x + c * y;
            }
            x = v[i][p];
            y = v[i][r];
            v[i][p] = c * x - s * y;
            v[i][r] = s * x + c * y;
        }
    }
}

static uint8_t mag_fit_ellipsoid(const common_mag_cal_struct *cal, common_mag_cal_params_struct *params, uint8_t *shape_ok)
{
    double center[3] = {0}, radius = 0.0;
    double a[9], q[3][3], v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, shift[3], augmented[3][4];
    double x, y, z, k, low, high, radius_cubed;
    double *normal[MAG_CAL_TERMS], *augmented_rows[3] = {augmented[0], augmented[1], augmented[2]};
    static const uint8_t row_axis[6] = {0, 1, 2, 0, 0, 1};
    static const uint8_t col_axis[6] = {0, 1, 2, 1, 2, 2};
    uint8_t n = 0, i, row, col;
    for (i = 0; i < cal->count; ++i)
    {
        if (s_fit.rejected[i] == 0)
        {
            for (row = 0; row < 3; ++row)
            {
                center[row] += cal->points[i].mag_uT[row];
            }
            ++n;
        }
    }
    for (row = 0; row < 3; ++row)
    {
        center[row] /= n;
    }
    for (i = 0; i < cal->count; ++i)
    {
        if (s_fit.rejected[i] == 0)
        {
            const float *m = cal->points[i].mag_uT;
            x = m[0] - center[0];
            y = m[1] - center[1];
            z = m[2] - center[2];
            radius += sqrt(x * x + y * y + z * z);
        }
    }
    radius /= n;
    if (!(radius >= 10.0))
    {
        return 1;
    }
    for (row = 0; row < MAG_CAL_TERMS; ++row)
    {
        normal[row] = s_fit.normal[row];
        for (col = 0; col <= MAG_CAL_TERMS; ++col)
        {
            normal[row][col] = 0.0;
        }
    }
    for (i = 0; i < cal->count; ++i)
    {
        if (s_fit.rejected[i] != 0)
        {
            continue;
        }
        /* 在中心化、无量纲坐标拟合 u^T Q u + 2 p^T u = 1。 */
        x = (cal->points[i].mag_uT[0] - center[0]) / radius;
        y = (cal->points[i].mag_uT[1] - center[1]) / radius;
        z = (cal->points[i].mag_uT[2] - center[2]) / radius;
        a[0] = x * x;
        a[1] = y * y;
        a[2] = z * z;
        a[3] = 2.0 * x * y;
        a[4] = 2.0 * x * z;
        a[5] = 2.0 * y * z;
        a[6] = 2.0 * x;
        a[7] = 2.0 * y;
        a[8] = 2.0 * z;
        for (row = 0; row < MAG_CAL_TERMS; ++row)
        {
            for (col = row; col < MAG_CAL_TERMS; ++col)
            {
                normal[row][col] += a[row] * a[col];
            }
            normal[row][MAG_CAL_TERMS] += a[row];
        }
    }
    for (row = 1; row < MAG_CAL_TERMS; ++row)
    {
        for (col = 0; col < row; ++col)
        {
            normal[row][col] = normal[col][row];
        }
    }
    if (mag_solve(MAG_CAL_TERMS, normal, a) != 0)
    {
        return 1;
    }
    for (i = 0; i < 6; ++i)
    {
        q[row_axis[i]][col_axis[i]] = q[col_axis[i]][row_axis[i]] = a[i];
    }
    for (row = 0; row < 3; ++row)
    {
        for (col = 0; col < 3; ++col)
        {
            augmented[row][col] = q[row][col];
        }
        augmented[row][3] = -a[6 + row];
    }
    if (mag_solve(3, augmented_rows, shift) != 0)
    {
        return 1;
    }
    k = 1.0 - (shift[0] * a[6] + shift[1] * a[7] + shift[2] * a[8]);
    if (!(k > 0.0))
    {
        return 1;
    }
    mag_eigen(q, v);
    for (row = 0; row < 3; ++row)
    {
        if (!(q[row][row] > 0.0))
        {
            return 1;
        }
        q[row][row] = sqrt(q[row][row] / k);
        params->offset_uT[row] = (float)(center[row] + radius * shift[row]);
    }
    low = fmin(q[0][0], fmin(q[1][1], q[2][2]));
    high = fmax(q[0][0], fmax(q[1][1], q[2][2]));
    /* 第一轮允许少量离群点暂时拉大轴比，最终门限在重新拟合后检查。 */
    if (!(high / low <= 6.0))
    {
        return 1;
    }
    /* 三条半轴的乘积是等效半径的三次方，直接比较可省去开立方。 */
    radius_cubed = radius * radius * radius / (q[0][0] * q[1][1] * q[2][2]);
    *shape_ok = radius_cubed >= 20.0 * 20.0 * 20.0 && radius_cubed <= 80.0 * 80.0 * 80.0 && high / low <= 2.5;
    for (i = 0; i < 6; ++i)
    {
        row = row_axis[i];
        col = col_axis[i];
        params->soft_sym[i] = (float)((v[row][0] * q[0][0] * v[col][0] + v[row][1] * q[1][1] * v[col][1] +
                                      v[row][2] * q[2][2] * v[col][2]) * COMMON_MAG_CAL_NOMINAL_UT / radius);
    }
    return 0;
}

static float mag_median(float values[], uint8_t count)
{
    uint8_t i, j;
    float value;
    for (i = 1; i < count; ++i)
    {
        value = values[i];
        j = i;
        while (j > 0 && value < values[j - 1])
        {
            values[j] = values[j - 1];
            --j;
        }
        values[j] = value;
    }
    return 0.5f * (values[(count - 1) / 2] + values[count / 2]);
}

uint8_t common_mag_cal_fit(const common_mag_cal_struct *cal, common_mag_cal_params_struct *params)
{
    common_mag_cal_params_struct candidate = {0};
    float m[3], min_uT[3] = {INFINITY, INFINITY, INFINITY}, max_uT[3] = {-INFINITY, -INFINITY, -INFINITY};
    float residual, median, threshold, error = 0.0f;
    uint8_t i, axis, shape_ok, kept = 0, removed = 0, dips = 0;
    uint32_t mask = 0;
    if (cal->samples < MAG_CAL_MIN_SAMPLES || cal->count < MAG_CAL_MIN_POINTS)
    {
        return 1;
    }
    for (i = 0; i < cal->count; ++i)
    {
        s_fit.rejected[i] = 0;
    }
    if (mag_fit_ellipsoid(cal, &candidate, &shape_ok) != 0)
    {
        return 1;
    }
    for (i = 0; i < cal->count; ++i)
    {
        common_mag_cal_apply(&candidate, cal->points[i].mag_uT, m);
        s_fit.sorted[i] = s_fit.residual[i] = sqrtf(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]) - COMMON_MAG_CAL_NOMINAL_UT;
    }
    median = mag_median(s_fit.sorted, cal->count);
    for (i = 0; i < cal->count; ++i)
    {
        s_fit.sorted[i] = fabsf(s_fit.residual[i] - median);
    }
    threshold = fmaxf(1.0f, 3.7f * mag_median(s_fit.sorted, cal->count));
    for (i = 0; i < cal->count; ++i)
    {
        if (fabsf(s_fit.residual[i] - median) > threshold)
        {
            s_fit.rejected[i] = 1;
            ++removed;
        }
    }
    if (cal->count - removed < MAG_CAL_MIN_POINTS || removed > cal->count / 8 ||
        (removed != 0 && mag_fit_ellipsoid(cal, &candidate, &shape_ok) != 0) || shape_ok == 0)
    {
        return 1;
    }
    for (i = 0; i < cal->count; ++i)
    {
        const common_mag_cal_point_struct *point = &cal->points[i];
        if (s_fit.rejected[i] != 0)
        {
            continue;
        }
        common_mag_cal_apply(&candidate, point->mag_uT, m);
        residual = sqrtf(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]) - COMMON_MAG_CAL_NOMINAL_UT;
        error += residual * residual;
        mask |= (uint32_t)1 << mag_direction_bucket(m);
        ++kept;
        for (axis = 0; axis < 3; ++axis)
        {
            min_uT[axis] = fminf(min_uT[axis], point->mag_uT[axis]);
            max_uT[axis] = fmaxf(max_uT[axis], point->mag_uT[axis]);
        }
        if (point->gravity[0] != 0.0f || point->gravity[1] != 0.0f || point->gravity[2] != 0.0f)
        {
            s_fit.sorted[dips++] = common_mag_cal_dip_deg(m, point->gravity);
        }
    }
    mask &= ~((uint32_t)1 << 13);
    if (mag_count_buckets(mask) < MAG_CAL_MIN_BUCKETS || !(error <= kept * 3.0f * 3.0f))
    {
        return 1;
    }
    for (axis = 0; axis < 3; ++axis)
    {
        if (max_uT[axis] - min_uT[axis] < 20.0f)
        {
            return 1;
        }
    }
    if (dips >= 12)
    {
        median = mag_median(s_fit.sorted, dips);
        kept = 0;
        for (i = 0; i < dips; ++i)
        {
            if (fabsf(s_fit.sorted[i] - median) <= 8.0f)
            {
                ++kept;
            }
        }
        if (kept >= 12 && 4 * kept >= 3 * dips)
        {
            candidate.dip_ref_deg = median;
            candidate.dip_valid = 1;
        }
    }
    *params = candidate;
    return 0;
}
