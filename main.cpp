//
//  main.cpp
//  CamSegm
//
//  Created by Márton Szemenyei on 2017. 07. 18..
//  Copyright © 2017. Márton Szemenyei. All rights reserved.
//

#include <iostream>
#include <cfloat>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/ximgproc.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>

std::string path = "";

std::vector<cv::Vec3b> colours =
{
    cv::Vec3b(0,0,0),
    cv::Vec3b(0,0,255),
    cv::Vec3b(0,255,0),
    cv::Vec3b(255,0,0),
    cv::Vec3b(255,0,255),
};

std::vector<cv::Point> pts;

cv::Mat image, mask, labels, disp;
cv::Mat maskDisp, imageDisp;
cv::Mat prevImage, prevMask;
cv::Mat green, white, hsv, elem;

int numClass = 5;
int selected = 0;
int thickness = 4;
int overwrite = 1;
int optFlow = 0;
int currSPSize = 20;
bool buttonDown = false;
unsigned int offs;
int method = 1;

float thresh = 50;

void getSuperPixels();

bool isGreen(int x, int y)
{
    return green.at<char>(y,x);
}

bool isWhite(int x, int y)
{
    return white.at<char>(y,x);
}

cv::Point intersection( const std::vector<float> &line1, const std::vector<float> &line2 )
{
    cv::Point ret(-1,-1);
    
    if (line1.size() == 2 && line2.size() == 2) {
        ret.x = -(line1[1] - line2[1])/(line1[0] - line2[0]);
        ret.y = line1[0] * ret.x + line1[1];
    }
    
    return ret;
}

cv::Point2f getCandidate( const cv::Point &p1, const cv::Point& p2)
{
    if (p1.x == p2.x) {
        return cv::Point2f(0,0);
    }
    
    cv::Point2f ret;
    
    ret.x = float(p1.y-p2.y)/float(p1.x-p2.x);
    ret.y = p1.y - p1.x*ret.x;
    
    return ret;
}

float distance( cv::Point pt, cv::Point2f line )
{
    cv::Point2f normvec( line.x, -1 );
    cv::Point2f toYSection( -pt.x, line.y - pt.y );
    
    return (normvec.x*toYSection.x + normvec.y*toYSection.y)/sqrt(line.x*line.x+1);
}

float distance( cv::Point pt, std::vector<float> &line )
{
    if( line.size() != 2)
        return FLT_MIN;
    
    cv::Point2f normvec( line[0], -1 );
    cv::Point2f toYSection( -pt.x, line[1] - pt.y );
    
    return (normvec.x*toYSection.x + normvec.y*toYSection.y)/sqrt(line[0]*line[0]+1);
}

int getInliers( const std::vector<cv::Point> pts, std::vector<size_t> &inliers, cv::Point2f &line )
{
    int inlierCnt = 0;
    inliers.clear();
    
    if( line.x == 0 && line.y == 0 )
        return inlierCnt;
    
    for (int i = 0; i < pts.size(); i++) {
        if( abs(distance(pts[i], line)) < 2 )
        {
            inlierCnt++;
            inliers.push_back(i);
        }
    }
    
    return inlierCnt;
}

std::vector<float> RANSAC( std::vector<cv::Point> &pts, std::vector<size_t> &inliers )
{
    size_t numCand = pts.size() * 10;
    
    std::vector<std::vector<bool> > picked( pts.size(), std::vector<bool>(pts.size(), false ) );
    
    std::vector<cv::Point2f> candidates;
    
    for (int i = 0; i < numCand; i++) {
        int idx1 = rand() % pts.size();
        int idx2 = rand() % pts.size();
        
        if (!picked[idx1][idx2] && !picked[idx2][idx1]) {
            candidates.push_back(getCandidate(pts[idx1], pts[idx2]));
            picked[idx1][idx2] = picked[idx2][idx1] = true;
        }
    }
    
    inliers.clear();
    std::vector<float> bestLine;
    int bestInliers = 0;
    
    std::vector<float> ret(2,0);
    
    for (int i = 0; i < candidates.size(); i++) {
        
        std::vector<size_t> currInliers;
        int curr = getInliers(pts, currInliers, candidates[i]);
        
        if (curr > bestInliers) {
            bestInliers = curr;
            inliers = currInliers;
            ret[0] = candidates[i].x;
            ret[1] = candidates[i].y;
            /*std::cout << curr << std::endl;
            
            for (int i = 0; i < pts.size(); i++) {
                cv::circle(imageDisp, pts[i], 5, cv::Scalar(0,0,255));
            }
            for (int i = 0; i < inliers.size(); i++) {
                cv::circle(imageDisp, pts[inliers[i]], 5, cv::Scalar(0,255,0));
            }
            
            cv::line(imageDisp, cv::Point(0,ret[1]), cv::Point(640,640*ret[0]+ret[1]), cv::Scalar(255,0,0));
            cv::imshow("imageDisp", imageDisp);
            cv::waitKey();*/
        }
        
    }
    
    return ret;
}

cv::Point raySearchSoccerHorizonBottomUp(cv::Mat &pImg, int x)
{
    int yd = -1;
    
    int xx = x;
    int yy = pImg.rows - 1;
    
    cv::Point last_edge = cv::Point(-1, -1);
    
    do
    {
        //Current pixel
        if (isGreen(xx,yy))
        {
            last_edge = cv::Point(xx, yy);
        }
        
        if (!isGreen(xx,yy) && !isWhite(xx,yy))
        {
            return last_edge;
        }
        
        yy += yd;
    } while (yy != 0 && yy != pImg.rows);
    
    return cv::Point(xx, yy);
}

void FindHorizon(cv::Mat &pImg)
{
    
    int _nx = pImg.cols;
    int _ny = pImg.rows;
    std::vector<float> line1 = {-1,-1}, line2 = {-1,-1};
    /*int nx_col_dist = _nx / 30;
    
    std::vector<cv::Point> pts;
    for (int x = 0; x < _nx; x += nx_col_dist)
    {
        cv::Point hEdge = raySearchSoccerHorizonBottomUp(pImg, x);
        if (hEdge.x != -1 && hEdge.y != -1 && hEdge.y < (_ny - 10))
        {
            pts.push_back(hEdge);
            //cv::circle(image, hEdge, 5, cv::Scalar(0,0,255));
        }
    }
    
    //RANSAC the points into two lines
    if (pts.empty()) {
        return;
    }
    
    std::vector<size_t> inliers;
    //get the first line
    std::vector<float> line1 = RANSAC(pts, inliers);
    size_t line1_nInliers = inliers.size();
    
    if( line1_nInliers == 0)
        return;
    
    for (size_t i = inliers.size()-1; ; i--) {
        pts.erase(pts.begin() + inliers[i] );
        if (i==0) {
            break;
        }
    }
    
    inliers.clear();
    //get the potential second line
    std::vector<float> line2 = RANSAC(pts, inliers);
    size_t line2_nInliers = inliers.size();
    
    
    //Sanity Checks
    cv::Point lhp,rhp,chp;
    
    //Minimum points needed for a line
    if (line1_nInliers <= 5) {
        line1.clear();
    }
    if (line2_nInliers <= 5) {
        line2.clear();
    }
    
    float thresh = 0.f;
    
    if (!line1.empty())
    {
        thresh = 60;
        if (!line2.empty())
        {
            cv::Point inter = intersection(line1, line2);
            if (!(inter.x >= 0 && inter.x < _nx &&
                inter.y >= 0 && inter.y < _ny))
            {
                line2.clear();
            }
        }
    }*/
    
    if (pts.size() >= 2) {
        cv::Point2f templine = getCandidate(pts[0], pts[1]);
        line1[0] = templine.x; line1[1] = templine.y;
        if (pts.size() > 2) {
            templine = getCandidate(pts[1], pts[2]);
            line2[0] = templine.x; line2[1] = templine.y;
        }
    }
    
    for (int y = 0; y < _ny; y++) {
        for (int x = 0; x < _nx; x++) {
            if (distance(cv::Point(x,y), line1) < -thresh || distance(cv::Point(x,y), line2) < -thresh) {
                image.at<cv::Vec3b>(y,x) = cv::Vec3b(0,0,0);
                maskDisp.at<cv::Vec3b>(y,x) = cv::Vec3b(0,0,0);
                mask.at<char>(y,x) = 0;
            }
        }
    }
}

namespace fs = ::boost::filesystem;
void get_all(const fs::path& root, const std::string& ext, std::vector<fs::path>& ret)
{
    if(!fs::exists(root) || !fs::is_directory(root)) return;
    
    fs::recursive_directory_iterator it(root);
    fs::recursive_directory_iterator endit;
    
    while(it != endit)
    {
        if(fs::is_regular_file(*it) && it->path().extension() == ext) ret.push_back(it->path());
        ++it;
        
    }
    
}

boost::regex re("(\\d+)");
boost::match_results<std::string::const_iterator> what1,what2;

template <typename T>
T st2num ( const std::string &Text )
{
    std::stringstream ss(Text);
    T result;
    return ss >> result ? result : 0;
}
struct mysort
{
    bool operator ()(const fs::path & _a,const fs::path & _b)
    {
        std::string a(_a.string()), b(_b.string());
        boost::regex_search(a.cbegin(), a.cend(), what1, re,
                            boost::match_default);
        boost::regex_search(b.cbegin(), b.cend(), what2, re,
                            boost::match_default);
        
        return st2num<int>(what1[1]) < st2num<int>(what2[1]);
        
    }
};

/*
 1 2 3
 0 x x
 x x x
 */
bool compareLabels(int row, int col, int direction)
{
    if (direction > 3 || direction < 0) {
        return false;
    }
    
    int rowoffs = direction ? 1 : 0;
    int coloffs = direction == 2 ? 0 : 1;
    
    return mask.at<char>(row-rowoffs,col-coloffs) == mask.at<char>(row+rowoffs,col+coloffs);
    
}

void refine()
{
    for (int i = 1; i < mask.rows-1; i++) {
        for (int j = 1; j < mask.cols-1; j++) {
            
            if (mask.at<char>(i,j) != 0) {
                continue;
            }
            if (compareLabels(i, j, 0) )
            {
                mask.at<char>(i,j) = mask.at<char>(i,j-1);
            } else if (compareLabels(i, j, 1) )
            {
                mask.at<char>(i,j) = mask.at<char>(i-1,j-1);
            }  else if (compareLabels(i, j, 2) )
            {
                mask.at<char>(i,j) = mask.at<char>(i-1,j);
            } else if (compareLabels(i, j, 3) )
            {
                mask.at<char>(i,j) = mask.at<char>(i-1,j+1);
            }
            
        }
    }
}

void fill( int x, int y, unsigned int val )
{
    if( ( overwrite || mask.at<char>(y,x) == 0 ) )
    {
        if (selected || labels.at<unsigned int>(y,x) == val )
        {
            mask.at<char>(y,x) = selected;
            maskDisp.at<cv::Vec3b>(y,x) = colours[selected];
            imageDisp.at<cv::Vec3b>(y,x) = 0.5*image.at<cv::Vec3b>(y,x) + 0.5*colours[selected];
        }
        
        if( labels.at<unsigned int>(y,x) == val )
        {
            labels.at<unsigned int>(y,x) += offs;
            if (x > 0) {
                fill(x-1, y, val);
            }
            if (y > 0) {
                fill(x, y-1, val);
            }
            if (x < mask.cols - 1) {
                fill(x+1, y, val);
            }
            if (y < mask.rows - 1) {
                fill(x, y+1, val);
            }
        }
    }
}

void showImages()
{
    imageDisp = 0.5*image+0.5*maskDisp;
    if (method == 0) {
        getSuperPixels();
    }
    cv::imshow("image", disp);
}

void on_trackbar_class( int , void* )
{
    std::cout << "Class selected: " << selected << std::endl;
}

void on_trackbar_thickness( int , void* )
{
    std::cout << "Line thickness/box size: " << thickness << std::endl;
    currSPSize = (thickness+1)*4;
    std::cout << "Superpixel size: " << currSPSize << std::endl;
    showImages();
}

void on_trackbar_overwrite( int , void* )
{
    std::cout << "Overwrite protection " << (!overwrite ? "enabled" : "disabled") << std::endl;
}

void on_trackbar_optflow( int , void* )
{
    std::cout << "Optical flow update " << (optFlow ? "enabled" : "disabled") << std::endl;
}

void on_trackbar_flood( int , void* )
{
    if( method == 0 )
    {
        std::cout << "Superpixel Segmentation" << std::endl;
    } else if( method == 1)
    {
        pts.clear();
        std::cout << "Line drawing" << std::endl;
    } else if( method == 2)
    {
        std::cout << "Box drawing" << std::endl;
    } else if( method == 3)
    {
        std::cout << "Circle drawing" << std::endl;
    }
    showImages();
}

static void onMouse( int event, int x, int y, int, void* )
{
    
    switch( event )
    {
        case cv::EVENT_LBUTTONDOWN:
            buttonDown = true;
            break;
        case cv::EVENT_LBUTTONUP:
            buttonDown = false;
            break;
    }
    if (x >= image.cols || y >= image.rows) {
        return;
    }
            
    if( !buttonDown )
        return;
    // Flood fill
    if( method == 0 )
    {
        unsigned int val = labels.at<unsigned int>(y,x);
        fill(x, y, val);
    }
    else if( method == 2)
    {
        int _x = x-thickness-1 < 0 ? 0 : x-thickness-1;
        int _y = y-thickness-1 < 0 ? 0 : y-thickness-1;
        int _w = x+thickness+1 < image.cols ? 2*(thickness+1)+1 : image.cols-1-_x;
        int _h = y+thickness+1 < image.rows ? 2*(thickness+1)+1 : image.rows-1-_y;
        cv::Rect rect(_x,_y,_w,_h);
        if (!overwrite) {
            cv::Mat colourTemp(cv::Size(_w,_h),CV_8UC3,colours[selected]);
            cv::Mat labelTemp(cv::Size(_w,_h),CV_8UC1,selected);
            cv::Mat copyMask = mask(rect) == 0;
            colourTemp.copyTo(maskDisp(rect), copyMask );
            labelTemp.copyTo(mask(rect), copyMask);
        } else
        {
            maskDisp(rect) = colours[selected];
            mask(rect) = selected;
        }
        imageDisp(rect) = 0.5*image(rect) + 0.5*maskDisp(rect);
    }
    else if (method == 3)
    {
        
        int _x = x-thickness-1 < 0 ? 0 : x-thickness-1;
        int _y = y-thickness-1 < 0 ? 0 : y-thickness-1;
        int _w = x+thickness+1 < image.cols ? 2*(thickness+1)+1 : image.cols-1-_x;
        int _h = y+thickness+1 < image.rows ? 2*(thickness+1)+1 : image.rows-1-_y;
        cv::Rect rect(_x,_y,_w,_h);
        if (!overwrite) {
            cv::Mat colourTemp(cv::Size(_w,_h),CV_8UC3,colours[0]);
            cv::Mat labelTemp(cv::Size(_w,_h),CV_8UC1,selected-selected);
            cv::Mat copyMask = mask(rect) == 0;
            cv::circle(colourTemp, cv::Point(x-_x,y-_y), thickness+1, colours[selected], -1);
            cv::circle(labelTemp, cv::Point(x-_x,y-_y), thickness+1, selected, -1);
            colourTemp.copyTo(maskDisp(rect), copyMask );
            labelTemp.copyTo(mask(rect), copyMask);
        } else
        {
            cv::circle(maskDisp, cv::Point(x,y), thickness+1, colours[selected], -1);
            cv::circle(mask, cv::Point(x,y), thickness+1, selected, -1);
        }
        imageDisp(rect) = 0.5*image(rect) + 0.5*maskDisp(rect);
    }
    else if( event == cv::EVENT_LBUTTONDOWN )
    {
        cv::Point currPt(x,y);
        if (!pts.empty()) {
            cv::line(imageDisp, pts.back(), currPt, colours[selected], thickness+1);
            /*cv::line(maskDisp, pts.back(), currPt, colours[selected], thickness+1);
            cv::line(mask, pts.back(), currPt, selected, thickness+1);*/
        }
        cv::circle(imageDisp, currPt, 3, colours[selected]);
        pts.push_back(currPt);
    }
    cv::imshow("image", disp);
    
}

void getSuperPixels()
{
    cv::Ptr< cv::ximgproc::SuperpixelLSC > segmenter = cv::ximgproc::createSuperpixelLSC( image, currSPSize );
    segmenter->iterate( 20 );
    offs = segmenter->getNumberOfSuperpixels();
    segmenter->getLabels( labels );
    cv::Mat contours;
    segmenter->getLabelContourMask(contours);
    for (int i = 0; i < 480; i++) {
        for (int j = 0; j < 640; j++) {
            if( contours.at<char>(i,j) != 0)
            {
                imageDisp.at<cv::Vec3b>(i,j) += cv::Vec3b(0,0,100);
            }
        }
    }
    /*cv::imshow("image", imageDisp);
    cv::waitKey();*/
    
}

cv::Mat readai2( const std::string & fname )
{
    std::ifstream ifs(fname, std::ios::binary|std::ios::ate);
    std::ifstream::pos_type pos = ifs.tellg();
    
    unsigned long size = pos;
    
    unsigned char *rawBytes = new unsigned char[size];
    ifs.seekg(0, std::ios::beg);
    ifs.read((char*)rawBytes, size);
    
    unsigned sx = *(unsigned *) &rawBytes[size - sizeof(unsigned) * 2];
    unsigned sy = *(unsigned *) &rawBytes[size - sizeof(unsigned)];
    //unsigned sz = sx * sy;
    
    if (sx > 1920 || sy > 1080) {
        return cv::Mat();
    }
    
    cv::Mat mat( sy,sx,CV_8UC3 );
    
    for (int q = 0; q < sy; q++) {
        for (int w = 0; w < sx; w++)
        {
            int line = q*3*sx;
            
            int y = rawBytes[w+line];
            int u = (rawBytes[sx*2+w+line]-128) * 112 / 128;
            int v = (rawBytes[sx+w+line]-128) * 158 / 128;
            
            int r = y + (int)(1.140 * v);
            int g = y - (int)((0.394 * u) + (0.581 * v));
            int b = y + (int)(2.032 * u);
            
            r = (r > 255) ? 255 : (r < 0) ? 0 : r;
            g = (g > 255) ? 255 : (g < 0) ? 0 : g;
            b = (b > 255) ? 255 : (b < 0) ? 0 : b;
            
            unsigned offs = w*3+q*sx*3;
            mat.data[offs]      = b;
            mat.data[offs+1]    = g;
            mat.data[offs+2]    = r;
        }
    }
    
    return mat;
}

void imageSelection()
{
    std::string path = "/Users/martonszemenyei/Downloads/Projects/NaoImages/Lab2017";
    
    if (path.back() != '/') {
        path.push_back('/');
    }
    
    fs::path root(path);
    if (!fs::exists(root)) {
        std::cout << "Error: the specified folder (" << path << ") does not exist!" << std::endl;
        return;
    }
    fs::path saveDir(path + "images/");
    if (!fs::exists(saveDir)) {
        fs::create_directory(saveDir);
    }
    
    std::vector<fs::path> files;
    fs::path imageDir(path);
    get_all(imageDir, ".ai2", files);
    
    int32_t cntr = 0;
    
    for( auto file:files )
    {
        std::string fName = file.string();
        
        std::vector<std::string> parts;
        boost::split(parts, fName, boost::is_any_of("."));
        
        if (parts.back() == "ai2") {
            image = readai2(fName);
        } else {
            image = cv::imread(fName);
        }
        if( !image.data )
        {
            std::cout << "Can't read image: " << file.string() << std::endl;
            continue;
        }
        
        cv::imshow("image", image);
        char c = cv::waitKey();
        
        if (c == 13) {
            cv::imwrite(saveDir.string() + std::to_string(cntr++) + ".png", image);
        }
        
    }
}

int main(int argc, char *argv[])
{
    //imageSelection();
    
    if( argc < 2 )
    {
        std::cout << "Usage: robocupannotator path-to-dataset" << std::endl << "The dataset path should contain the \"images\" folder." << std::endl;
        return -1;
        
    }
    
    path = argv[1];
    
    //path = "/Users/martonszemenyei/Downloads/Projects/CamSegm/";
    
    if (path.back() != '/') {
        path.push_back('/');
    }
    
    fs::path root(path);
    if (!fs::exists(root)) {
        std::cout << "Error: the specified folder (" << path << ") does not exist!" << std::endl;
        return -2;
    }
    fs::path saveDirMask(path + "labels/");
    if (!fs::exists(saveDirMask)) {
        fs::create_directory(saveDirMask);
    }
    fs::path horizonImgPath(path + "horImages/");
    if (!fs::exists(horizonImgPath)) {
        fs::create_directory(horizonImgPath);
    }
    fs::path horizonLabelPath(path + "horLabels/");
    if (!fs::exists(horizonLabelPath)) {
        fs::create_directory(horizonLabelPath);
    }
    std::vector<fs::path> files;
    fs::path imageDir(path + "images/");
    get_all(imageDir, ".png", files);
    std::sort( files.begin(), files.end(), mysort() );
    
    cv::namedWindow("image",cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("image", onMouse);
    cv::createTrackbar("Class selection", "image", &selected, numClass-1, on_trackbar_class );
    cv::createTrackbar("Allow Overwrite", "image", &overwrite, 1, on_trackbar_overwrite );
    cv::createTrackbar("Optical Flow", "image", &optFlow, 1, on_trackbar_optflow );
    cv::createTrackbar("Methods", "image", &method, 3, on_trackbar_flood );
    cv::createTrackbar("Thickness/Count", "image", &thickness, 9, on_trackbar_thickness );
    
    //std::vector<std::string> names = {"106."};
    
    int cntr = 0;
    
    for( auto file:files )
    {
        std::string fName = file.string();
        
        /*bool found = false;
        for (auto name : names) {
            if (fName.find(name) != std::string::npos) {
                found = true;
            }
        }
        if (!found) {
            continue;
        }*/
        
        std::vector<std::string> parts;
        boost::split(parts, fName, boost::is_any_of("."));
        
        if (parts.back() == "ai2") {
            image = readai2(fName);
        } else {
            image = cv::imread(fName);
        }
        if( !image.data )
        {
            std::cout << "Can't read image: " << file.string() << std::endl;
            continue;
        }
        
        fs::path maskFile(saveDirMask.string() + fs::change_extension(fName, "png").filename().string());
        //fs::path maskFile(saveDirMask.string() + std::to_string(cntr) + ".png" );
        if( fs::exists(maskFile) )
        {
            mask = cv::imread(maskFile.string(), cv::IMREAD_GRAYSCALE);
        } else
        {
            mask = cv::Mat(image.size(), CV_8UC1, cv::Scalar(0) );
            if (optFlow && prevImage.data )
            {
                cv::Mat flowImg;
                
                cv::Mat currTemp;
                cv::cvtColor(image, currTemp, cv::COLOR_BGR2GRAY);
                
                cv::Ptr<cv::FarnebackOpticalFlow> opt = cv::FarnebackOpticalFlow::create(5,0.5,false,35,10,7,1.5);
                opt->calc(prevImage, currTemp, flowImg);
                
                for (int i = 0; i < flowImg.rows; i++)
                {
                    for (int j = 0; j < flowImg.cols; j++)
                    {
                        cv::Vec2f flow = flowImg.at<cv::Vec2f>(i,j);
                        
                        int newX = j + roundf(flow[0]);
                        int newY = i + roundf(flow[1]);
                        
                        if (newX < 0 || newY < 0 || newX >= flowImg.cols || newY >= flowImg.rows)
                            continue;
                        
                        mask.at<char>(newY,newX) = prevMask.at<char>(i,j);
                    }
                }
                refine();
            }
        }
        
        disp = cv::Mat(image.rows+325, image.cols*2, CV_8UC3, cv::Scalar(0) );
        disp(cv::Rect(0,image.rows, image.cols*2, 325)) = cv::Scalar(255,255,255);
        cv::putText(disp, "Use the class selection slider to select the class to mark on the image:", cv::Point(10, image.rows +25), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,0,0));
        cv::putText(disp, "0 - background; 1 - ball; 2 - robot; 3 - goalpost; 4 - field line", cv::Point(50, image.rows +50), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,0,0));
        cv::putText(disp, "Use the methods slider to set the segmentation method:", cv::Point(10, image.rows +75), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,0,0));
        cv::putText(disp, "0 - use superpixels; 1 - line tool; 2 - square brush; 3 - circular brush", cv::Point(50, image.rows +100), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,0,0));
        cv::putText(disp, "The superpixel count slider can make the superpixels more fine or coarse.", cv::Point(10, image.rows +125), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,0,0));
        cv::putText(disp, "The thickness/size slider sets the thickness of the line or the size of the brush.", cv::Point(10, image.rows +150), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,0,0));
        cv::putText(disp, "By disabling overwrite, the tools will only change background pixels.", cv::Point(10, image.rows +175), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,0,0));
        cv::putText(disp, "By enabling optical flow, the program will try to infere the labels for the next image.", cv::Point(10, image.rows +200), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,0,0));
        cv::putText(disp, "Key commands:", cv::Point(10, image.rows +225), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,0,0));
        cv::putText(disp, "n - go to the next image without saving;       q - Quit         Esc - reset segmentation", cv::Point(50, image.rows +250), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,0,0));
        cv::putText(disp, "Enter - save segmentation and go to the next image;            c - clear lines", cv::Point(50, image.rows +275), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,0,0));
        cv::putText(disp, "f - draw polygon determined by the lines;                        d - draw lines;", cv::Point(50, image.rows +300), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,0,0));
        
        maskDisp = disp(cv::Rect(image.cols,0,image.cols,image.rows));
        for( int i = 0; i < mask.rows; i++)
        {
            for (int j = 0; j < mask.cols; j++) {
                maskDisp.at<cv::Vec3b>(i,j) = colours[mask.at<unsigned char>(i,j)];
            }
        }
        imageDisp = disp(cv::Rect(0,0,image.cols,image.rows));
        showImages();
        std::cout << file.string() << std::endl;
        
        while (true) {
            char c = cv::waitKey(1000);
            bool finished = false;
            
            switch (c) {
                case 13:
                    cv::imwrite(saveDirMask.string() + fs::change_extension(fName, "png").filename().string(), mask);
                    //cv::imwrite(saveDirMask.string() + std::to_string(cntr) + ".png", mask);
                    //cv::imwrite(imageDir.string() + std::to_string(cntr) + ".png", image);
                    // Break missing on purpose
                case 'n':
                    if (optFlow) {
                        cv::cvtColor(image, prevImage, cv::COLOR_BGR2GRAY);
                        mask.copyTo(prevMask);
                    }
                    cntr++;
                    finished = true;
                    break;
                case 27:
                    mask = cv::Mat(image.size(), CV_8UC1, cv::Scalar(0) );
                    disp(cv::Rect(0,0,image.cols*2, image.rows)) = cv::Scalar(0);
                    maskDisp = disp(cv::Rect(image.cols,0,image.cols,image.rows));
                    imageDisp = disp(cv::Rect(0,0,image.cols,image.rows));
                    showImages();
                    break;
                case 'h':
                    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
                    cv::inRange(hsv, cv::Scalar(0,10,40), cv::Scalar(100,150,150), green);
                    elem = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
                    cv::dilate(green, green, elem);
                    cv::inRange(hsv, cv::Scalar(0,0,150), cv::Scalar(255,30,255), white);
                    cv::morphologyEx(white, white, cv::MORPH_CLOSE, elem);
                    cv::imshow("white", green);
                    FindHorizon(image);
                    showImages();
                    if( cv::waitKey() != 13 )
                    {
                        image = cv::imread(fName);
                        mask = cv::imread(maskFile.string(), cv::IMREAD_GRAYSCALE);
                    }
                    // break missing on purpose
                case 'g':
                    cv::imwrite(horizonLabelPath.string() + fs::change_extension(fName, "png").filename().string(), mask);
                    cv::imwrite(horizonImgPath.string() + fs::change_extension(fName, "png").filename().string(), image);
                    finished = true;
                    break;
                case 'f':
                    if (!pts.empty()) {
                        std::vector< std::vector<cv::Point> > temp = {pts};
                        if (!overwrite) {
                            cv::Mat colourTemp(maskDisp.size(),CV_8UC3,colours[0]);
                            cv::Mat labelTemp(mask.size(),CV_8UC1,selected-selected);
                            cv::Mat copyMask = (mask == 0);
                            cv::fillPoly(colourTemp, temp, colours[selected]);
                            cv::fillPoly(labelTemp, temp, selected);
                            colourTemp.copyTo(maskDisp, copyMask );
                            labelTemp.copyTo(mask, copyMask);
                        } else
                        {
                            cv::fillPoly(maskDisp, temp, colours[selected]);
                            cv::fillPoly(mask, temp, selected);
                        }
                        pts.clear();
                        showImages();
                    }
                    break;
                case 'd':
                    if (!overwrite) {
                        cv::Mat colourTemp(maskDisp.size(),CV_8UC3,colours[0]);
                        cv::Mat labelTemp(mask.size(),CV_8UC1,selected-selected);
                        cv::Mat copyMask = (mask == 0);
                        for( int i = 0; !pts.empty() && i < pts.size()-1; i++)
                        {
                            cv::line(colourTemp, pts[i], pts[i+1], colours[selected], thickness+1);
                            cv::line(labelTemp, pts[i], pts[i+1], selected, thickness+1);
                        }                        
                        colourTemp.copyTo(maskDisp, copyMask );
                        labelTemp.copyTo(mask, copyMask);
                    } else
                    {
                        for( int i = 0; !pts.empty() && i < pts.size()-1; i++)
                        {
                            cv::line(maskDisp, pts[i], pts[i+1], colours[selected], thickness+1);
                            cv::line(mask, pts[i], pts[i+1], selected, thickness+1);
                        }
                    }
                    // Break missing on purpose
                case 'c':
                    pts.clear();
                    showImages();
                    break;
                case 'q':
                    exit(0);
                    break;
                default:
                    break;
            }
            
            if( finished )
                break;
        }
    }
    
    
    
    return 0;
}
