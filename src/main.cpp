#include <eigen3/Eigen/Dense>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include "kalmanPos.h"

using std::vector;
using std::string;
using std::ifstream;
using std::stringstream;
using Eigen::VectorXd;
using std::ofstream;

vector<vector<double>> readData(string path){
    vector<vector<double>> data;
    double val;
    
    ifstream filestream(path);
    string line;
    if(filestream.is_open()){
        while(std::getline(filestream,line)){
            stringstream linestream(line);
            vector<double> curr_data;
            while(linestream>>val){
                curr_data.push_back(val);
            }
            data.push_back(curr_data);
        }
    }
    return data;
}

void writeFile(const string path,vector<double> &output){
    ofstream myfile;
    myfile.open(path);
    for( double &val : output){
        myfile<<val<<"\n";
    }
    myfile.close();
}

int main(){
    string path = "../imu_data.txt";
    vector<vector<double>> data = readData(path);
    vector<vector<double>> est_data(4);
    KalmanFilterPos kf;
    for(auto &d: data){
        VectorXd x = kf.processMeasurement(d);
        est_data[0].push_back(x(0));
        est_data[1].push_back(x(1));
        est_data[2].push_back(x(2));
        est_data[3].push_back(x(3));
    }
    writeFile("../output/posx.txt",est_data[0]);
    writeFile("../output/posy.txt",est_data[1]);
    writeFile("../output/velx.txt",est_data[2]);
    writeFile("../output/vely.txt",est_data[3]);

    return 0;
}