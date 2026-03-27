#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    for (int i = 0; i < 10; ++i) {  // Try indices from 0 to 9
        cv::VideoCapture cap(i);
        if (cap.isOpened()) {
            std::cout << "Camera found at index: " << i << std::endl;
            cv::Mat frame;
            cap >> frame;
            if (!frame.empty()) {
                cv::imshow("Webcam", frame);
                cv::waitKey(1000);  // Display for a short period
                cv::destroyWindow("Webcam");
            }
            cap.release();  // Release camera when done
        } else {
            std::cout << "No camera at index: " << i << std::endl;
        }
    }
    return 0;
}