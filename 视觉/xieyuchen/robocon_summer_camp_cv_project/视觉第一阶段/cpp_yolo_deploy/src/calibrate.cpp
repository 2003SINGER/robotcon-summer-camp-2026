#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

int main(){
    Size boardSize(8,5);//棋盘的交叉点数
    vector<vector<Point2f>> imagePoints;
    vector<vector<Point3f>> objectPoints;

    vector<Point3f> obj;

    float squareSize = 26.0f; // 单位：毫米

    for(int i = 0; i< boardSize.height; i++){
        for(int j = 0; j < boardSize.width; j++){
            obj.push_back(Point3f(j * squareSize, i * squareSize, 0));
        }
    }

    VideoCapture cap("/dev/video4");

    
    if(!cap.isOpened()){
        cout << "error" << endl;
        return -1;
    }

    Mat frame,gray;
    int count = 0;
    cout << "space to capture" << endl;

    while (true) {
        cap >> frame;
        if(frame.empty()) break;
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        vector<Point2f> corners;
        bool found = findChessboardCorners(gray, boardSize, corners,
                                            CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

        //   备注： CALIB_CB_ADAPTIVE_THRESH：使用自适应阈值，而不是全局固定阈值。这对光照不均匀的摄像头画面非常友好，能大大提高找到角点的成功率。

        //         CALIB_CB_NORMALIZE_IMAGE：对图像进行归一化（调整对比度/亮度），进一步增强了算法对不同光线条件的鲁棒性。
        
        if(found) {
            //细致并优化

            //亚像素精细化（精度提升的关键）
            cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
                         TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.001));
        //         TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.001)：终止条件。
        // COUNT = 30：最多迭代 30 次。
        // EPS = 0.001：当角点移动的精度小于 0.001 像素时，认为收敛，停止迭代。

            drawChessboardCorners(frame, boardSize, corners, found);
            imshow("标定", frame);

            char key = waitKey(10);
            if (key == ' '){
                imagePoints.push_back(corners);
                objectPoints.push_back(obj);
                count++;
                cout << "已标记第 " << count << "张" << "请变换角度继续" << endl;

           
            // 在画面上显示绿色提示，持续 30 帧
            putText(frame, "SAVED! (" + to_string(count) + "/20)", Point(30, 60), 
            FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 255, 0), 2);
}
            imshow("标定", frame);


            if(key == 27) break; 
        }else{
            imshow("标定", frame);
            if (waitKey(10) == 27) break;
        }
    }

    cap.release();
    destroyAllWindows();

    if(imagePoints.size() < 5){
        cout << "数量小于5" << endl;
        return -1;
    }

        Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    Mat distCoeffs;
    vector<Mat> rvecs, tvecs;
    calibrateCamera(objectPoints, imagePoints, Size(frame.cols, frame.rows),
                    cameraMatrix, distCoeffs, rvecs, tvecs);

    cout << "标定成功！内参矩阵：" << endl << cameraMatrix << endl;

    FileStorage fs("calibration_data.yml", FileStorage::WRITE);
    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
    fs.release();

    cout << "已生成 calibration_data.yml，fx = " << cameraMatrix.at<double>(0, 0) << endl;
    return 0;









}