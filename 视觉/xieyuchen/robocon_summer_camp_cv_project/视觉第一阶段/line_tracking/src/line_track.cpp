#include "line_track.hpp"
#include <stdlib.h>   // for abs

static int g_image_width = 0;
static int g_image_height = 0;
static int g_threshold = 128;
static int g_error = 0;
static int g_target_center_x = 0;
static int g_valid_row_count = 0;
static bool g_is_lost = true;
static int g_left_boundary[LINE_TRACK_MAX_HEIGHT];
static int g_right_boundary[LINE_TRACK_MAX_HEIGHT];
static int g_center_line[LINE_TRACK_MAX_HEIGHT];

static int line_track_clamp_int(int value, int min_value, int max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static void line_track_clear_boundaries(void) {
    for (int i = 0; i < LINE_TRACK_MAX_HEIGHT; i++) {
        g_left_boundary[i] = -1;
        g_right_boundary[i] = -1;
        g_center_line[i] = -1;
    }
}

// 改进：计算全局阈值仅作为备选（实际使用行阈值，此处留作参考）
static int line_track_calc_global_threshold(uint8_t *image, int width, int height) {
    int min_gray = 255, max_gray = 0;
    int roi_start_y = height / 3;
    for (int y = roi_start_y; y < height; y += 2) {
        uint8_t *row = image + y * width;
        for (int x = 0; x < width; x += 2) {
            int gray = row[x];
            if (gray < min_gray) min_gray = gray;
            if (gray > max_gray) max_gray = gray;
        }
    }
    int threshold = (min_gray + max_gray) / 2;
    return line_track_clamp_int(threshold, 70, 220);
}

// 改进：为每一行单独计算动态阈值（行均值 + 偏移）
static int line_track_calc_row_threshold(uint8_t *row, int width, int global_threshold) {
    int sum = 0;
    int count = 0;
    // 降采样计算均值（提高速度）
    for (int x = 0; x < width; x += 4) {
        sum += row[x];
        count++;
    }
    int avg = sum / count;
    // 偏移量可根据实际赛道颜色调整（白色赛道比地面亮，偏移取正）
    int threshold = avg + 30;  // 经验值
    // 限制范围，防止极端情况
    threshold = line_track_clamp_int(threshold, 60, 200);
    return threshold;
}

// 寻找本行中满足阈值条件的白色段，并依据参考中心选出最优段
static bool line_track_find_row_segment(uint8_t *row,
                                        int width,
                                        int threshold,
                                        int reference_center,
                                        int *left,
                                        int *right) {
    int x = 0;
    int best_left = -1;
    int best_right = -1;
    int best_score = -32768;
    int min_run_width = width / 12;
    if (min_run_width < 6) min_run_width = 6;

    while (x < width) {
        // 跳过低于阈值的点（暗区）
        while (x < width && row[x] < threshold) x++;
        if (x >= width) break;

        int run_left = x;
        while (x < width && row[x] >= threshold) x++;
        int run_right = x - 1;
        int run_width = run_right - run_left + 1;

        if (run_width >= min_run_width) {
            int run_center = (run_left + run_right) / 2;
            int score = run_width * 4 - abs(run_center - reference_center);
            // 若参考中心落在该段内，给予极大奖励
            if (reference_center >= run_left && reference_center <= run_right) {
                score += width * 4;
            }
            if (score > best_score) {
                best_score = score;
                best_left = run_left;
                best_right = run_right;
            }
        }
    }

    if (best_left < 0 || best_right < 0) return false;
    *left = best_left;
    *right = best_right;
    return true;
}

void LineTrack_Init(void) {
    g_image_width = 0;
    g_image_height = 0;
    g_threshold = 128;
    g_error = 0;
    g_target_center_x = 0;
    g_valid_row_count = 0;
    g_is_lost = true;
    line_track_clear_boundaries();
}

bool LineTrack_ProcessFrame(uint8_t *image, int width, int height) {
    if (image == NULL || width <= 0 || height <= 0 || height > LINE_TRACK_MAX_HEIGHT) {
        g_is_lost = true;
        g_valid_row_count = 0;
        g_error = 0;
        return false;
    }

    g_image_width = width;
    g_image_height = height;

    // 计算全局阈值（保留用于参考，也可传给行阈值作为基础）
    int global_threshold = line_track_calc_global_threshold(image, width, height);
    g_threshold = global_threshold;  // 保存用于调试

    int reference_center = width / 2;
    int max_lost_rows = height / 8;
    int min_valid_rows = height / 8;
    if (max_lost_rows < 4) max_lost_rows = 4;
    if (min_valid_rows < 6) min_valid_rows = 6;

    line_track_clear_boundaries();

    int weighted_center_sum = 0;
    int weight_sum = 0;
    int valid_rows = 0;
    int lost_rows = 0;

    // 从底部向上扫描
    for (int y = height - 1; y >= 0; y--) {
        uint8_t *row = image + y * width;

        // 改进：使用本行动态阈值
        int row_threshold = line_track_calc_row_threshold(row, width, global_threshold);

        int left = -1, right = -1;
        if (line_track_find_row_segment(row, width, row_threshold, reference_center, &left, &right)) {
            int center = (left + right) / 2;
            int weight = height - y + 1;   // 越近权重越大
            g_left_boundary[y] = left;
            g_right_boundary[y] = right;
            g_center_line[y] = center;
            reference_center = center;
            weighted_center_sum += center * weight;
            weight_sum += weight;
            valid_rows++;
            lost_rows = 0;
        } else if (valid_rows > 0) {
            lost_rows++;
            if (lost_rows > max_lost_rows) break;
        }
    }

    g_valid_row_count = valid_rows;

    if (valid_rows < min_valid_rows || weight_sum == 0) {
        g_is_lost = true;
        g_error = 0;
        g_target_center_x = width / 2;
        return false;
    }

    g_is_lost = false;
    g_target_center_x = weighted_center_sum / weight_sum;
    g_error = g_target_center_x - width / 2;
    return true;
}

int LineTrack_GetError(void) { return g_error; }
bool LineTrack_IsLost(void) { return g_is_lost; }
int LineTrack_GetTargetCenterX(void) { return g_target_center_x; }
int LineTrack_GetThreshold(void) { return g_threshold; }
int LineTrack_GetValidRowCount(void) { return g_valid_row_count; }
const int *LineTrack_GetLeftBoundary(void) { return g_left_boundary; }
const int *LineTrack_GetRightBoundary(void) { return g_right_boundary; }
const int *LineTrack_GetCenterLine(void) { return g_center_line; }