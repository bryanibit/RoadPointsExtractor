#include "xy2000_lb2000.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <string>
#include <unordered_map>
#include "opencv2/opencv.hpp"
#include <thread>
#include <chrono>

using namespace std;

const double kDisPoints = 2.0;
const int kWIDTH = 1000;
const int kHEIGHT = 1000;
const double kRES = 0.5;

class PosXY{
public:
    // xy: lon,lat * 1000000
    // meterxy: xy in meter
    // offsetxy: xy in image in meter
    double x;
    double y;
    double meterx;
    double metery;
    double offsetx;
    double offsety;
    PosXY() = default;
    PosXY(double x__, double y__, double meterx__, double metery__, double offsetx__, double offsety__):
            x(x__), y(y__), meterx(meterx__), metery(metery__), offsetx(offsetx__), offsety(offsety__){}
};

std::vector<int> split(string &s, const string &delimiter){
    int pos = s.find_first_not_of(delimiter, 0);
    int after = s.find_first_of(delimiter, pos);
    vector<int> res;
    int temp;
    while(pos != string::npos || after != string::npos){
        temp = stoi(s.substr(pos, after - pos));
        res.push_back(temp);
        pos = s.find_first_not_of(delimiter, after);
        after = s.find_first_of(delimiter, pos);
    }
	return res;
}

class ShowMap{
private:
    int map_width_;
    int map_height_;
    double res_;
    cv::Mat map_;
//            (cv::Size(1000, 1000), CV_8UC3, cv::Scalar(0, 0, 0));
    std::unordered_map<int, PosXY > pixel2xy_;
    vector<int> path;
    vector<std::pair<int,PosXY> > choose_points;

public:
    static void MouseClick(int event, int x, int y, int flag, void* userdata){
        auto showmap_obj = reinterpret_cast<ShowMap*>(userdata);
        showmap_obj->OnMouse(event, x,y,flag);
    }
    void OnMouse(int event, int x, int y, int flag){
        if (event == CV_EVENT_LBUTTONDOWN) {
//            auto choose = cv::Point(x, y);
            std::cout << "x,y: " << x << ", " << y << std::endl;
            int dis_sqrt = std::numeric_limits<int>::max();
            int min_i;
            for(int i = 0; i < path.size(); ++i){
                int col = path[i] / max(map_width_, map_height_);
                int row = path[i] % max(map_width_, map_height_);
                int distance = pow(col - x, 2) + pow(row - y, 2);

                if(distance < dis_sqrt) {
                    dis_sqrt = distance;
                    min_i = i;
                }
            }
            choose_points.push_back(make_pair(path[min_i], PosXY(pixel2xy_[path[min_i]].x, pixel2xy_[path[min_i]].y,
                                       pixel2xy_[path[min_i]].meterx,pixel2xy_[path[min_i]].metery,
                                       pixel2xy_[path[min_i]].offsetx,pixel2xy_[path[min_i]].offsety)));
            // save to txt: pixel2xy_[path[min_i]].first * 0.000001
            // save to txt: pixel2xy_[path[min_i]].second * 0.000001
        }
    }
    ShowMap(int width, int height, double res): map_width_(width), map_height_(height), res_(res){
        map_ = cv::Mat(cv::Size(map_width_, map_height_), CV_8UC3, cv::Scalar(255,255,255));
        cv::namedWindow("showmap", CV_WINDOW_AUTOSIZE);
        cv::setMouseCallback("showmap", ShowMap::MouseClick, this);
    }
    void DrawMap(const vector<PosXY> &offset){
        int col;
        int row;
        int count = 0;
        for(int i = 0; i < offset.size(); ++i){
            col = static_cast<int>(offset[i].offsetx / res_ + map_width_ / 2.0);
            row = static_cast<int>(offset[i].offsety / res_ + map_height_ / 2.0);
            if(col < 0 || row < 0)
                count++;
            else {
                cv::circle(map_, cv::Point(col, row), 1, cv::Scalar(125, 125, 0), -1);
                auto key = col * max(map_width_, map_height_) + row;
                pixel2xy_[key] = PosXY(offset[i].x, offset[i].y,
                        offset[i].meterx, offset[i].metery, offset[i].offsetx, offset[i].offsety);
                path.push_back(key);
            }
        }
        //cv::flip(map_, map_, 0);
        cv::Mat showmap = map_.clone();
        while(1) {
            cv::imshow("showmap", showmap);
            char key = cv::waitKey(10);
            if(key == 27)
                break;
            if(key == 'w'){
                std::ofstream onfile;
                onfile.open("RNDF.txt", std::ios::out);
                if(onfile.is_open()){
                    for(int i = 0; i < choose_points.size(); ++i)
                        onfile<<to_string(i) << " " << std::fixed<<std::setprecision(6)<<
                              choose_points[i].second.x * 0.000001 << " " << std::fixed<<std::setprecision(6)
                              << choose_points[i].second.y * 0.000001 << " 0 2" <<endl;
                }
                onfile.close();
            }
            if(key == 'r'){
                if(choose_points.empty())
                    std::cerr << "choose vector is empty\n";
                else {
                    choose_points.pop_back();
                    showmap = map_.clone();
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            }
            for(int i = 0; i < choose_points.size(); ++i) {
                cv::circle(showmap, cv::Point(choose_points[i].first / max(map_width_, map_height_),
                                           choose_points[i].first % max(map_width_, map_height_)),
                           4, cv::Scalar(0, 0, 255), -1);
                cv::putText(showmap, to_string(i + 1), cv::Point(choose_points[i].first / max(map_width_, map_height_),
                                                          choose_points[i].first % max(map_width_, map_height_)),
                            cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(180, 185, 0), 2);
            }
        }
    }
};

void LonLat2Offset(vector<PosXY> &offset, const PosXY &origin, const vector<vector<int> > &lines){
    if(lines.empty())
        return;
    auto pre_line = lines[0];
    int pre_line_x;
    int pre_line_y;
    Mercator::LB2XY(lines[0][0] * 0.000001 * M_PI / 180.0,
                    lines[0][1] * 0.000001 * M_PI / 180.0, pre_line_x, pre_line_y);
    double pre_x = pre_line_x * 0.1 - origin.meterx;
    double pre_y = pre_line_y * 0.1 - origin.metery;
    offset.emplace_back(lines[0][0], lines[0][1], pre_line_x * 0.1, pre_line_y * 0.1, pre_x, pre_y);

    for(int i = 0; i < lines.size(); ++i){
        int x; int y;
        if(lines[i].size() == 4){
            Mercator::LB2XY(lines[i][0] * 0.000001 * M_PI / 180.0,
                            lines[i][1] * 0.000001 * M_PI / 180.0, x, y);
        }
        double offset_x = x * 0.1 - origin.meterx;
        double offset_y = y * 0.1 - origin.metery;
        // filter some points
        if(sqrt(pow(offset_x - pre_x, 2) + pow(offset_y - pre_y, 2)) >= kDisPoints) {
            offset.emplace_back(lines[i][0], lines[i][1], x * 0.1, y * 0.1, offset_x, offset_y);
            pre_x = offset_x;
            pre_y = offset_y;
        }
    }
}

void ShowOffsetDebug(const vector<PosXY> &offset){
    for(int i = 0; i < offset.size(); ++i){
        std::cout << offset.at(i).offsetx << ", " << offset.at(i).offsety << std::endl;
    }
}

int main(){
    std::vector<std::vector<int> > lines;
    std::ifstream infile;
    infile.open("../res.txt");
    string s;
    while(infile >> s)
        lines.push_back(split(s, ","));
    infile.close();
    // lines: 10234566 3994534 0 2
    if(lines.size() < 10)
        return -1;
    PosXY origin;
    origin.x = lines[0].at(0);
    origin.y = lines[0].at(1);
    for(int i = 1; i < 10; ++i){
        origin.x = (lines[i][0] + origin.x) / 2.0;
        origin.y = (lines[i][1] + origin.y) / 2.0;
    }
    // origin meterx and y
    int meterx;
    int metery;
    Mercator::LB2XY(origin.x * 0.000001 * M_PI / 180.0, 
            origin.y * 0.000001 * M_PI / 180.0, meterx, metery);
    origin.meterx = meterx * 0.1;
    origin.metery = metery * 0.1;
    origin.offsetx = 0;
    origin.offsety = 0;
    vector<PosXY> offset;
    LonLat2Offset(offset, origin, lines);
    std::cout << offset.size() << std::endl;

//    std::ofstream onfile;
//    onfile.open("RNDFFull.txt", std::ios::out);
//    if(onfile.is_open()){
//        for(int i = 0; i < offset.size(); ++i)
//            onfile<<std::fixed<<std::setprecision(6)<< offset[i].x * 0.000001 << " "
//                  << std::fixed<<std::setprecision(6)<< offset[i].y * 0.000001<< " 0 2" <<endl;
//    }
//    onfile.close();

//    ShowOffsetDebug(offset);
    ShowMap sm(kWIDTH,kHEIGHT,kRES);
    sm.DrawMap(offset);
    return 0;
}
