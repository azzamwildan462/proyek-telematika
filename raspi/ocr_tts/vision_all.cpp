

#include "net.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <fstream>

#include "librealsense2/rsutil.h"
#include "librealsense2/hpp/rs_pipeline.hpp"

#include <wiringPi.h>

#define YOLOV4_TINY 1 // 0 or undef for yolov4

using namespace rs2;
using namespace cv;
using namespace std;

ncnn::Net yolov4;
int target_size;
const float mean_vals[3] = {0, 0, 0};
const float norm_vals[3] = {1 / 255.f, 1 / 255.f, 1 / 255.f};

// REALSENSE
pipeline rs2_pipe;
config rs2_cfg;

const char *class_names[] = {
    "latar", "seseorang", "sepeda",
    "mobil", "sepeda motor", "pesawat", "bis", "kereta", "truk",
    "kapal", "lampu lalu lintas", "tabung pemadam kebakaran", "tanda berhenti",
    "jarak parkir", "bench", "burung", "kucing", "anjing", "kuda",
    "domba", "sapi", "gajah", "beruang", "zebra", "jerapah",
    "tas", "payung", "tas", "dasi", "suitcase",
    "frisbee", "skis", "snowboard", "bola", "layangan",
    "pemukul baseball", "sarung tangan baseball", "skateboard", "surfboard",
    "raket tenis", "botol", "botol kaca", "cangkir", "garpu",
    "pisau", "sendok", "mangkok", "pisang", "apel", "roti",
    "jeruk", "brokoli", "wortel", "hot dog", "pizza", "donat",
    "kue", "kursi", "sofa", "tumbuhan", "tempat tidur", "meja",
    "toilet", "tv", "laptop", "mouse", "remote", "keyboard",
    "handphone", "microwave", "oven", "toaster", "sink",
    "kulkas", "buku", "jam", "vase", "gunting",
    "boneka", "oengering rambut", "sikat gigi"};

// PROGRAM SIAPA?
#define PROGRAM_WILDAN 0
#define PROGRAM_RAFLI 1
#define PROGRAM_NAUFAL 2
uint8_t use_program = 0;

struct Object
{
    Rect_<float> rect;
    int label;
    float prob;
    float center[2];
    float distance; // in cm
};

void audio_outPlay(const char *str);
void play_rafli();

string toLowercase(const string &str)
{
    string lowercaseStr = str;
    transform(lowercaseStr.begin(), lowercaseStr.end(), lowercaseStr.begin(), ::tolower);
    return lowercaseStr;
}

static Mat rs2cv(const frame &f)
{
    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    return Mat(Size(w, h), CV_8UC3, (void *)f.get_data(), Mat::AUTO_STEP);
}

static int detect_yolov4(const Mat &bgr, std::vector<Object> &objects, rs2::depth_frame depth)
{
    int img_w = bgr.cols;
    int img_h = bgr.rows;

    ncnn::Mat in = ncnn::Mat::from_pixels_resize(bgr.data, ncnn::Mat::PIXEL_BGR, bgr.cols, bgr.rows, target_size, target_size);

    in.substract_mean_normalize(mean_vals, norm_vals);

    ncnn::Extractor ex = yolov4.create_extractor();
    ex.set_num_threads(4);

    ex.input("data", in);

    ncnn::Mat out;
    ex.extract("output", out);

    //     printf("%d %d %d\n", out.w, out.h, out.c);
    objects.clear();
    for (int i = 0; i < out.h; i++)
    {
        const float *values = out.row(i);

        Object object;
        object.label = values[0];
        object.prob = values[1];
        object.rect.x = values[2] * img_w;
        object.rect.y = values[3] * img_h;
        object.rect.width = values[4] * img_w - object.rect.x;
        object.rect.height = values[5] * img_h - object.rect.y;
        object.center[0] = object.rect.x + 0.5 * object.rect.width;
        object.center[1] = object.rect.y + 0.5 * object.rect.height;
        object.distance = depth.get_distance(object.center[0], object.center[1]) * 100.0f;

        objects.push_back(object);
    }

    return 0;
}

static void filter_objects(Mat &bgr, const std::vector<Object> &objects)
{
    float min_dist = __FLT_MAX__;
    int8_t min_dist_idx = -1;
    for (size_t i = 0; i < objects.size(); i++)
    {
        const Object &obj = objects[i];

        if (obj.distance < min_dist && obj.distance > __FLT_EPSILON__)
        {
            min_dist = obj.distance;
            min_dist_idx = i;
        }

        rectangle(bgr, obj.rect, Scalar(255, 0, 0));

        char text[256];
        sprintf(text, "%s %.1f cm", class_names[obj.label], obj.distance);

        int baseLine = 0;
        Size label_size = getTextSize(text, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

        int x = obj.rect.x;
        int y = obj.rect.y - label_size.height - baseLine;
        if (y < 0)
            y = 0;
        if (x + label_size.width > bgr.cols)
            x = bgr.cols - label_size.width;

        rectangle(bgr, Rect(Point(x, y), Size(label_size.width, label_size.height + baseLine)),
                  Scalar(255, 255, 255), -1);

        putText(bgr, text, Point(x, y + label_size.height),
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0));
    }

    if (min_dist_idx != -1)
    {
        const Object &obj = objects[min_dist_idx];
        rectangle(bgr, obj.rect, Scalar(0, 0, 255), 5);

        int8_t kanan_kiri = (obj.center[0] < 320 ? -1 : 1);
        int8_t layer_depan = 0;

        if (obj.distance > 400)
            layer_depan = 5;
        else if (obj.distance > 300)
            layer_depan = 4;
        else if (obj.distance > 200)
            layer_depan = 3;
        else if (obj.distance > 100)
            layer_depan = 2;
        else if (obj.distance > 0)
            layer_depan = 1;

        char str_layer_depan[25];
        char str_layer_kanan_kiri[25];

        if (kanan_kiri == 1)
            sprintf(str_layer_kanan_kiri, "di sebelah kanan");
        else
            sprintf(str_layer_kanan_kiri, "di sebelah kiri");

        if (layer_depan == 0x05)
            sprintf(str_layer_depan, "di atas empat meter");
        else if (layer_depan == 0x04)
            sprintf(str_layer_depan, "di atas tiga meter");
        else if (layer_depan == 0x03)
            sprintf(str_layer_depan, "di atas dua meter");
        else if (layer_depan == 0x02)
            sprintf(str_layer_depan, "di atas satu meter");
        else if (layer_depan == 0x01)
            sprintf(str_layer_depan, "di bawah satu meter");

        char str_out[64];
        sprintf(str_out, "%s %s %s", class_names[obj.label], str_layer_kanan_kiri, str_layer_depan);

        printf("WILDAN: %s\n", str_out);
        audio_outPlay(str_out);
    }

    // printf("============================\n");
}

int main(int argc, char **argv)
{
    // original pretrained model from https://github.com/AlexeyAB/darknet
    // the ncnn model https://drive.google.com/drive/folders/1YzILvh0SKQPS_lrb33dmGNq7aVTKPWS0?usp=sharing
    // the ncnn model https://github.com/nihui/ncnn-assets/tree/master/models
#if YOLOV4_TINY
    yolov4.load_param("/home/naufaldi/proyek-telematika/ocr_tts/yolov4-tiny-opt.param");
    yolov4.load_model("/home/naufaldi/proyek-telematika/ocr_tts/yolov4-tiny-opt.bin");
    target_size = 416;
#else
    yolov4.load_param("yolov4-opt.param");
    yolov4.load_model("yolov4-opt.bin");
    target_size = 608;
#endif
    yolov4.opt.num_threads = 4;

    // Start RS capture
    rs2_cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_ANY, 30);
    rs2_cfg.enable_stream(RS2_STREAM_DEPTH);

    auto config = rs2_pipe.start(rs2_cfg);

    // HW-setup
    wiringPiSetup(); // Setup the library
    pinMode(25, INPUT);
    pinMode(24, INPUT);

    //=====================================
    string outText;
    Mat im;
    Mat grayImage;
    // namedWindow("Display window");

    tesseract::TessBaseAPI *ocr = new tesseract::TessBaseAPI();
    ocr->Init(NULL, "eng", tesseract::OEM_LSTM_ONLY);
    ocr->SetPageSegMode(tesseract::PSM_AUTO);
    // VideoCapture cap(4, CAP_V4L2);

    vector<string> wordsToFilter;
    ifstream wordFile("/home/naufaldi/proyek-telematika/ocr_tts/idwords.txt");
    if (wordFile.is_open())
    {
        string word;
        while (getline(wordFile, word))
        {
            wordsToFilter.push_back(toLowercase(word));
        }
        wordFile.close();
    }

    uint8_t toggles = 0;
    while (1)
    {
        toggles = 0;
        toggles |= digitalRead(24);
        toggles |= (digitalRead(25) << 0x01);

        if (toggles == 0b01)
        {
            use_program = PROGRAM_NAUFAL;
        }
        else if (toggles == 0b10)
        {
            use_program = PROGRAM_RAFLI;
            play_rafli();
            waitKey(1000);
            continue;
        }
        else
        {
            use_program = PROGRAM_WILDAN;
        }

        // // get rs frame
        auto frames = rs2_pipe.wait_for_frames(150000);
        rs2::align align_to(RS2_STREAM_COLOR);
        auto color_frame = frames.get_color_frame();
        frames = align_to.process(frames);
        cv::Mat rs2_frame = rs2cv(color_frame);
        cv::Mat bgr_rs2_frame;
        cvtColor(rs2_frame, bgr_rs2_frame, COLOR_RGB2BGR);

        if (use_program == PROGRAM_WILDAN)
        {
            // get depth
            rs2::depth_frame depth = frames.get_depth_frame();

            // Detect and calculate RS
            std::vector<Object> objects;
            detect_yolov4(bgr_rs2_frame, objects, depth);
            filter_objects(bgr_rs2_frame, objects);

            // imshow("Result", bgr_rs2_frame);
        }
        else if (use_program == PROGRAM_NAUFAL)
        {
            // imshow("Normal", bgr_rs2_frame);
            cvtColor(bgr_rs2_frame, grayImage, COLOR_BGR2GRAY);

            /*PIX* leptonicaImage = pixCreateHeader(im.cols, im.rows, 8);
            pixSetData(leptonicaImage, grayImage.data);
            pixSetWpl(leptonicaImage, grayImage.step / 4);*/

            // Apply adaptive thresholding
            Mat binaryImage;
            adaptiveThreshold(grayImage, binaryImage, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 11, 7);
            // threshold(grayImage, binaryImage, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

            // Apply morphological operations (optional)
            cv::Mat morphImage;
            cv::morphologyEx(binaryImage, morphImage, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_OPEN, cv::Size(1, 1)));

            // Perform image denoising (optional)
            Mat denoisedImage;
            fastNlMeansDenoising(morphImage, denoisedImage, 15.0, 7, 21);

            // imshow("Final", denoisedImage);

            // ocr->SetImage(im.data, im.cols, im.rows, 3, im.step);

            ocr->SetImage(denoisedImage.data, denoisedImage.cols, denoisedImage.rows, 1, denoisedImage.step);
            ocr->SetSourceResolution(700);
            outText = string(ocr->GetUTF8Text());
            // cout << outText;

            string lowercaseText = toLowercase(outText);

            vector<string> words;
            istringstream iss(outText);
            string word;
            while (iss >> word)
            {
                words.push_back(word);
            }

            // Filter the individual words from the OCR output
            vector<string> filteredWords;
            for (const string &word : words)
            {
                string lowercaseWord = toLowercase(word);
                if (find(wordsToFilter.begin(), wordsToFilter.end(), lowercaseWord) != wordsToFilter.end())
                {
                    // Word found in the word list, add to the filtered words
                    filteredWords.push_back(word);
                }
            }

            // Reconstruct the filtered output sentence
            string filteredText;
            for (const string &word : filteredWords)
            {
                filteredText += word + " ";
            }

            if (!filteredText.empty())
            {
                printf("NAUFAL: %s\n", filteredText.c_str());
                audio_outPlay(filteredText.c_str());
            }
        }

        waitKey(300);
    }

    return 0;
}

void audio_outPlay(const char *str)
{
    char str_buffer[256];
    sprintf(str_buffer, "gtts-cli '%s' -l id -o out.mp3 && ffplay out.mp3 -nodisp -nostats -hide_banner -autoexit 2>/dev/null", str);
    system(str_buffer);
}

void play_rafli()
{
    string gmaps_dir;
    gmaps_dir.clear();
    ifstream fin("/home/naufaldi/proyek-telematika/Backend\ 2/public/instruksi.txt");
    getline(fin, gmaps_dir);
    fin.close();
    printf("RAFLI: %s\n", gmaps_dir.c_str());
    audio_outPlay(gmaps_dir.c_str());
}

/**
 * RUN AUDIO:
 * gtts-cli '%s' -l id -o out.mp3 && ffplay out.mp3 -nodisp -nostats -hide_banner -autoexit 2>/dev/null
 */
