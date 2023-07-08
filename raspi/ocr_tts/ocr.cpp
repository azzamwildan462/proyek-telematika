#include <iostream>
#include <string>
#include <fstream>
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// argument : g++ ocr.cpp -o cv `pkg-config opencv4 --cflags --libs` -o program -llept -ltesseract
// export TESSDATA_PREFIX=/home/naufaldi/tesseract-4.1.3/tessdata/
// g++ -O3 -std=c++11 basic_ocr.cpp `pkg-config --cflags --libs tesseract opencv` -o basic_ocr

string toLowercase(const string &str)
{
    string lowercaseStr = str;
    transform(lowercaseStr.begin(), lowercaseStr.end(), lowercaseStr.begin(), ::tolower);
    return lowercaseStr;
}

int main()
{

    string outText;
    Mat im;
    Mat grayImage;
    namedWindow("Display window");

    tesseract::TessBaseAPI *ocr = new tesseract::TessBaseAPI();
    ocr->Init(NULL, "eng", tesseract::OEM_LSTM_ONLY);
    ocr->SetPageSegMode(tesseract::PSM_AUTO);
    VideoCapture cap(4, CAP_V4L2);

    vector<string> wordsToFilter;
    ifstream wordFile("words.txt");
    if (wordFile.is_open())
    {
        string word;
        while (getline(wordFile, word))
        {
            wordsToFilter.push_back(toLowercase(word));
        }
        wordFile.close();
    }

    while (1)
    {

        cap >> im;
        cvtColor(im, grayImage, COLOR_BGR2GRAY);

        /*PIX* leptonicaImage = pixCreateHeader(im.cols, im.rows, 8);
        pixSetData(leptonicaImage, grayImage.data);
        pixSetWpl(leptonicaImage, grayImage.step / 4);*/

        // Apply adaptive thresholding
        Mat binaryImage;
        adaptiveThreshold(grayImage, binaryImage, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 17, 10);
        // threshold(grayImage, binaryImage, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

        // Apply morphological operations (optional)
        cv::Mat morphImage;
        cv::morphologyEx(binaryImage, morphImage, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

        // Perform image denoising (optional)
        Mat denoisedImage;
        fastNlMeansDenoising(morphImage, denoisedImage, 15.0, 7, 21);

        imshow("Normal", im);
        imshow("Final", denoisedImage);

        // ocr->SetImage(im.data, im.cols, im.rows, 3, im.step);

        ocr->SetImage(denoisedImage.data, denoisedImage.cols, denoisedImage.rows, 1, denoisedImage.step);
        ocr->SetSourceResolution(500);
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

        cout << "Filtered Output: " << filteredText << endl;

        if (waitKey(1000) == 0x71)
        {
            break;
        }
    }
    ocr->End();
    delete ocr;
    return 0;
}