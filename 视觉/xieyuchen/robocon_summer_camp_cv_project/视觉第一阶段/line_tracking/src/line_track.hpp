#ifndef __PROJECT_LINE_TRACK_HPP__
#define __PROJECT_LINE_TRACK_HPP__

#include <stdint.h>
#include <stdbool.h>

#define LINE_TRACK_MAX_HEIGHT 240

#ifdef __cplusplus
extern "C" {
#endif

void LineTrack_Init(void);

bool LineTrack_ProcessFrame(uint8_t *image, int width, int height);

int LineTrack_GetError(void);
bool LineTrack_IsLost(void);
int LineTrack_GetTargetCenterX(void);
int LineTrack_GetThreshold(void);
int LineTrack_GetValidRowCount(void);
const int *LineTrack_GetLeftBoundary(void);
const int *LineTrack_GetRightBoundary(void);
const int *LineTrack_GetCenterLine(void);

#ifdef __cplusplus
}
#endif

#endif