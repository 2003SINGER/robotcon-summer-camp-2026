#include <opencv2/opencv.hpp>
#include <iostream>
#include "line_track.hpp"

using namespace cv;
using namespace std;

int main() {
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "无法打开摄像头 /dev/video0" << endl;
        return -1;
    }

    cap.set(CAP_PROP_FRAME_WIDTH, 320);
    cap.set(CAP_PROP_FRAME_HEIGHT, 240);

    LineTrack_Init();

    Ptr<CLAHE> clahe = createCLAHE(3.0, Size(8, 8));
    Mat frame, gray, processed, display;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        cvtColor(frame, gray, COLOR_BGR2GRAY);
        GaussianBlur(gray, processed, Size(5, 5), 0);
        clahe->apply(processed, processed);

        if (processed.rows > LINE_TRACK_MAX_HEIGHT) {
            resize(processed, processed, Size(processed.cols * LINE_TRACK_MAX_HEIGHT / processed.rows,
                                              LINE_TRACK_MAX_HEIGHT));
        }

        bool ok = LineTrack_ProcessFrame(processed.data, processed.cols, processed.rows);

        cvtColor(processed, display, COLOR_GRAY2BGR);

        const int* left   = LineTrack_GetLeftBoundary();
        const int* right  = LineTrack_GetRightBoundary();
        const int* center = LineTrack_GetCenterLine();

        for (int y = 0; y < processed.rows; y++) {
            if (left[y] >= 0 && left[y] < processed.cols)
                display.at<Vec3b>(y, left[y]) = Vec3b(0, 0, 255);
            if (right[y] >= 0 && right[y] < processed.cols)
                display.at<Vec3b>(y, right[y]) = Vec3b(255, 0, 0);
            if (center[y] >= 0 && center[y] < processed.cols)
                display.at<Vec3b>(y, center[y]) = Vec3b(0, 255, 0);
        }

        int error = LineTrack_GetError();
        bool lost = LineTrack_IsLost();
        char text[64];
        sprintf(text, "Err: %d  %s", error, lost ? "LOST" : "OK");
        putText(display, text, Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);

        imshow("Line Tracking", display);
        if (waitKey(1) == 27) break;
        printf("Error: %d  Lost: %d\n", error, lost);
    }

    cap.release();
    destroyAllWindows();
    return 0;
}