#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"


#define drawCross( center, color, d )                                 \
    line( img, cv::Point( center.x - d, center.y - d ), cv::Point( center.x + d, center.y + d ), color, 2, CV_AA, 0); \
    line( img, cv::Point( center.x + d, center.y - d ), cv::Point( center.x - d, center.y + d ), color, 2, CV_AA, 0 )
using namespace std;

cv::Point mousePos;

void On_mouse(int event, int x, int y, int flags, void*)
{
    //if (event == cv::EVENT_LBUTTONDOWN || (event == cv::EVENT_MOUSEMOVE && (flags&cv::EVENT_FLAG_LBUTTON)))
        mousePos = cv::Point(x, y);
}

int main( )
{    cv::KalmanFilter KF(4, 2, 0);
    // intialization of KF...
    KF.transitionMatrix = (cv::Mat_<float>(4, 4) << 1,0,1,0,
                                                    0,1,0,1,
                                                    0,0,1,0,
                                                    0,0,0,1);
    cv::Mat_<float> measurement(2,1);
    measurement.setTo(cv::Scalar(0));

    KF.statePre.at<float>(0) = mousePos.x;
    KF.statePre.at<float>(1) = mousePos.y;
    KF.statePre.at<float>(2) = 0;
    KF.statePre.at<float>(3) = 0;

    cv::setIdentity(KF.measurementMatrix);
    cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-4));
    cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(10));
    cv::setIdentity(KF.errorCovPost, cv::Scalar::all(.1));

    // Image to show mouse tracking
    cv::Mat img(800, 1000, CV_8UC3);
    vector<cv::Point> mousev,kalmanv;
    mousev.clear();
    kalmanv.clear();

    cv::setWindowTitle("kalman","haha");
    cv::setMouseCallback("kalman", On_mouse, 0);

    while(1)
    {
        // First predict, to update the internal statePre variable
        cv::Mat prediction = KF.predict();
        cv::Point predictPt(prediction.at<float>(0), prediction.at<float>(1));

        // Get mouse point

        measurement(0) = mousePos.x;
        measurement(1) = mousePos.y;

        // The update phase
        cv::Mat estimated = KF.correct(measurement);

        cv::Point statePt(estimated.at<float>(0), estimated.at<float>(1));
        cv::Point measPt(measurement(0), measurement(1));
        // plot points
        cv::imshow("kalman", img);
        img = cv::Scalar::all(0);

        mousev.push_back(measPt);
        kalmanv.push_back(statePt);
        drawCross( statePt, cv::Scalar(255,255,255), 5 );
        drawCross( measPt, cv::Scalar(0,0,255), 5 );

        for (int i = 0; i < mousev.size()-1; i++)
            line(img, mousev[i], mousev[i+1], cv::Scalar(255,255,0), 1);

        for (int i = 0; i < kalmanv.size()-1; i++)
            line(img, kalmanv[i], kalmanv[i+1], cv::Scalar(0,155,255), 1);


        char code = (char)cv::waitKey(5);
        if( code == 27 || code == 'q' || code == 'Q' )
            break;
    }

    return 0;
}
