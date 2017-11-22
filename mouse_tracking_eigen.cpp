#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Eigen"
#include <iostream>

using namespace std;

#define drawCross( center, color, d )                                 \
    line( img, cv::Point( center.x - d, center.y - d ), cv::Point( center.x + d, center.y + d ), color, 2, CV_AA, 0); \
    line( img, cv::Point( center.x + d, center.y - d ), cv::Point( center.x - d, center.y + d ), color, 2, CV_AA, 0 )

cv::Point mousePos;

void On_mouse(int event, int x, int y, int flags, void*)
{
    //if (event == cv::EVENT_LBUTTONDOWN || (event == cv::EVENT_MOUSEMOVE && (flags&cv::EVENT_FLAG_LBUTTON)))
        mousePos = cv::Point(x, y);
}


int main( )
{
    cv::setWindowTitle("kalman","haha");
    cv::setMouseCallback("kalman", On_mouse, 0);

    Eigen::Matrix4d A;      //状态转移矩阵，也可能随时间变化
    Eigen::Matrix4d Q;      //过程激励噪声

    Eigen::Matrix<double, 2, 4> H;      //测量矩阵，利用 H*X,计算得到观测数据Z'
    Eigen::Vector2d Z;      //实际观测得到的数据，
    Eigen::Matrix<double, 4, 2> K;            //卡尔曼增益

    Eigen::Matrix2d R;
    Eigen::Matrix4d Pk_prior;     //先验误差协方差
    Eigen::Matrix4d Pk;           //后验误差协方差
    Eigen::Matrix4d Pk_last;
    Eigen::Matrix4d I;

    Eigen::Vector4d X_prior;      //状态变量的先验估计
    Eigen::Vector4d X;            //状态变量的后验估计
    Eigen::Vector4d X_last;


    Pk_prior = Eigen::Matrix4d::Random();

    //std::cout<<Pk_prior<<std::endl;

    Q<<1,0,0,0,
       0,1,0,0,
       0,0,1,0,
       0,0,0,1;

    Q = Q*0.01;

    I<<1,0,0,0,
       0,1,0,0,
       0,0,1,0,
       0,0,0,1;

    R<<1,0,
       0,1;

    A<<1,0,1,0,
       0,1,0,1,
       0,0,1,0,
       0,0,0,1;

    H<<1,0,0,0,
       0,1,0,0;

    X_prior<<0,0,0,0;
    X<<0,0,0,0;


//    std::cout<<Q<<"  "<<std::endl;
//    std::cout<<Q.norm()<<std::endl;       //Q.norm()是啥啊，算出来好奇怪

    // Image to show mouse tracking
    cv::Mat img(600, 800, CV_8UC3);
    std::vector<cv::Point> mousev,kalmanv,xxx_prix;
    mousev.clear();
    kalmanv.clear();

    cv::setWindowTitle("kalman","haha");
    cv::setMouseCallback("kalman", On_mouse, 0);

    CvFont font;// = cv::FONT_HERSHEY_SIMPLEX;
    cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX, 1.0, 1.0, 0, 2, 8);

    while(1)
    {
        Z(0) = mousePos.x;
        Z(1) = mousePos.y;

        //时间更新方程
        X_last = X;
        X_prior = A*X_last;

        Pk_last = Pk;
        Pk_prior = A*Pk_last*A.transpose()+Q;

        //状态更新方程
        K = Pk_prior * H.transpose() * (H * Pk_prior * H.transpose() + R).inverse();
        X = X_prior + K*(Z - H*X_prior);
        Pk = (I - K*H)*Pk_prior;


        // plot points
        cv::imshow("kalman", img);
        img = cv::Scalar::all(12);

        cv::Point mouse_point(Z(0), Z(1));
        cv::Point kalman_result(X(0), X(1));
        cv::Point xx_prior(X_prior(0), X_prior(1));

        mousev.push_back(mouse_point);
        kalmanv.push_back(kalman_result);
        xxx_prix.push_back(xx_prior);

        drawCross( mouse_point, cv::Scalar(255,255,255), 5 );
        drawCross( kalman_result, cv::Scalar(0,0,255), 5 );
        drawCross( xx_prior, cv::Scalar(255,0,255), 5 );

        cv::putText(img, "mouse_pos", cv::Point(550, 10), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,0));
        cv::putText(img, "prior_pos", cv::Point(550, 40), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,155,255));
        cv::putText(img, "kalman_pos", cv::Point(550, 70), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,155,255));

        for (int i = 0; i < mousev.size()-1; i++)
            line(img, mousev[i], mousev[i+1], cv::Scalar(255,255,0), 1);

        for (int i = 0; i < kalmanv.size()-1; i++)
            line(img, kalmanv[i], kalmanv[i+1], cv::Scalar(0,155,255), 1);

        for (int i = 0; i < xxx_prix.size()-1; i++)
            line(img, xxx_prix[i], xxx_prix[i+1], cv::Scalar(255,155,255), 1);


        char code = (char)cv::waitKey(5);
        if( code == 27 || code == 'q' || code == 'Q' )
            break;
    }

    return 0;
}
