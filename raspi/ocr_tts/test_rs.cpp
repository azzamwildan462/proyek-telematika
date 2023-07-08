#include "librealsense2/rsutil.h"
#include "librealsense2/hpp/rs_pipeline.hpp"
#include "net.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>
#include <vector>

using namespace rs2;
using namespace cv;

rs2::pipeline rs2_pipe;
rs2::config rs2_cfg;
rs2::decimation_filter rs2_dec_filter;
rs2::spatial_filter rs2_spat_filter;

static cv::Mat rs2cv(const rs2::frame &f);

int main()
{

    rs2_cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_ANY, 30);
    rs2_cfg.enable_stream(RS2_STREAM_DEPTH);

    auto config = rs2_pipe.start(rs2_cfg);

    while (1)
    {

        auto frames = rs2_pipe.wait_for_frames(150000);
        rs2::align align_to(RS2_STREAM_COLOR);
        auto color_frame = frames.get_color_frame();
        frames = align_to.process(frames);
        cv::Mat rs2_frame = rs2cv(color_frame);
        cv::Mat bgr_rs2_frame;
        cvtColor(rs2_frame, bgr_rs2_frame, COLOR_RGB2BGR);

        //==================
        rs2::depth_frame depth = frames.get_depth_frame();

        float center[2] = {320, 240};
        float zzz = depth.get_distance(center[0], center[1]) * 100.0f;

        printf("zzz: %.2f\n", zzz);

        imshow("haloo", bgr_rs2_frame);

        waitKey(1);
    }

    printf("haloo\n");

    return 0;
}

static cv::Mat rs2cv(const rs2::frame &f)
{
    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    return Mat(Size(w, h), CV_8UC3, (void *)f.get_data(), Mat::AUTO_STEP);
}
