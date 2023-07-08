cmake_minimum_required(VERSION 3.15)
project(ocr)

set(CMAKE_CXX_STANDARD 17)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
set(leptonica_dir "/home/naufaldi/leptonica-1.82.0/cmake")
#find_package(Leptonica REQUIRED PATHS ${leptonica_dir})
#find_package(Leptonica REQUIRED )

include_directories(${Leptonica_INCLUDE_DIRS})
set(tesseract_dir "/home/naufaldi/tesseract-4.1.3/cmake")
#find_package(Tesseract REQUIRED PATHS ${tesseract_dir})
#find_package(Tesseract REQUIRED )

include_directories(${Tesseract_INCLUDE_DIRS})
add_executable(ocr ocr.cpp)
#target_link_libraries(ocr ${OpenCV_LIBS} ${Tesseract_LIBRARIES} ${Leptonica_LIBRARIES})
target_link_libraries(ocr ${OpenCV_LIBS} -llept -ltesseract)


## WILDAN ##
INCLUDE_DIRECTORIES( /usr/local/include/ncnn )
add_executable( vision_all vision_all.cpp)
target_link_libraries( vision_all ${OpenCV_LIBS} /usr/local/lib/ncnn/libncnn.a pthread gomp realsense2 wiringPi -llept -ltesseract)

add_executable( test_rs test_rs.cpp)
target_link_libraries( test_rs realsense2 ${OpenCV_LIBS})
############