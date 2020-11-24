#include "xy2000_lb2000.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "opencv2/opencv.hpp"

using namespace std;

const double kDisPoints = 2.0;

class PosXY{
public:
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
    unordered_map<int, std::pair<double, double> > pixel2xy_;
    vector<int> path;
public:
    ShowMap(int width, int height, double res): map_width_(width), map_height_(height), res_(res){
        map_ = cv::Mat::zeros(cv::Size(1000, 1000), CV_8UC3);
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
                cv::circle(map_, cv::Point(col, row), 1, cv::Scalar(255, 255, 255), -1);
                auto key = col * max(map_width_, map_height_) + row;
                pixel2xy_[key] = make_pair(offset[i].x, offset[i].y);
                path.push_back(key);
            }
        }

        cv::flip(map_, map_, 0);
        cv::namedWindow("showmap", CV_WINDOW_AUTOSIZE );
        cv::imshow("showmap", map_);
        cv::waitKey(0);
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
//    ShowOffsetDebug(offset);
    ShowMap sm(1000,1000,0.5);
    sm.DrawMap(offset);
    return 0;
}
