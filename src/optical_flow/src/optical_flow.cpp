//#include "stdafx.h"
//#include <windows.h>
#include <opencv2/opencv.hpp>
#include <opencv2/superres/optical_flow.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
           
//オプティカルフローを可視化する。
//縦横のベクトルの強さを色に変換する。
//左：赤、右：緑、上：青、下：黄色
void visualizeFarnebackFlow(
    const Mat& flow,    //オプティカルフロー CV_32FC2
    Mat& visual_flow    //可視化された画像 CV_32FC3
)
{
    visual_flow = Mat::zeros(flow.rows, flow.cols, CV_32FC3);
    int flow_ch = flow.channels();
    int vis_ch = visual_flow.channels();//3のはず
    for(int y = 0; y < flow.rows; y++) {
        float* psrc = (float*)(flow.data + flow.step * y);
        float* pdst = (float*)(visual_flow.data + visual_flow.step * y);
        for(int x = 0; x < flow.cols; x++) {
            float dx = psrc[0];
            float dy = psrc[1];
            float r = (dx < 0.0) ? abs(dx) : 0;
            float g = (dx > 0.0) ? dx : 0;
            float b = (dy < 0.0) ? abs(dy) : 0;
            r += (dy > 0.0) ? dy : 0;
            g += (dy > 0.0) ? dy : 0;
 
            pdst[0] = b;
            pdst[1] = g;
            pdst[2] = r;
 
            psrc += flow_ch;
            pdst += vis_ch;
        }
    }
}
 
int _tmain(int argc, _TCHAR* argv[])
{
    VideoCapture cap(0);
    Mat capture, current, previous,flow,visual_flow;
     
    cvInitSystem(argc, argv);
    cvNamedWindow("flow", CV_WINDOW_AUTOSIZE);
     
    while(true) {
         
        //カメラキャプチャ
        cap >> capture;
         
        //グレイスケールへ変換
        cvtColor(capture, current, CV_BGR2GRAY);
         
        //前のフレームがあれば、オプティカルフローを計算し、表示する
        if(!previous.empty()) {
         
            //オプティカルフロー計算
            calcOpticalFlowFarneback(
                previous, current, flow, 
                0.5, 1, 15, 1, 5, 1.1,
                OPTFLOW_FARNEBACK_GAUSSIAN);
             
            //フローを可視化
            visualizeFarnebackFlow(flow, visual_flow);
             
            //表示
            imshow("flow", visual_flow);
        }
         
        //前のフレームを保存
        previous = current.clone();
         
        if (cvWaitKey(1) == '\x1b') {//ESC
            break;
        }
    }
    cvDestroyAllWindows();
    return 0;
}
