// DH_Computation.cpp : This file contains the 'main' function. Program execution begins and ends there.//
//C:\Users\jacob\Documents\C++\DH_Computation\DH_Computation\ur10e_dh.cfg

#include<iostream>
#include<cmath>
#include<fstream>
#include<exception>
#include<vector>
#include<fstream>
#include<string>

#include <cmath>
#include <iomanip>
#include<boost\property_tree\ptree.hpp>
#include<boost\property_tree\xml_parser.hpp>
#include<boost\foreach.hpp>

using namespace std;
using boost::property_tree::ptree;

double pos_x, pos_y, pos_z, rx, ry, rz;//final_set_values
vector<double> j_vals = {};//joint_values

double j1_dh[4][4];
double j2_dh[4][4];
double j3_dh[4][4];
double j4_dh[4][4];
double j5_dh[4][4];
double j6_dh[4][4];//DH matrix for each joint

double* dh_tables[6] = {&j1_dh[4][4],&j2_dh[4][4],&j3_dh[4][4],&j4_dh[4][4],&j5_dh[4][4], &j6_dh[4][4]};//DH matrix pointers for each joint
double** ptr1 = new double* [6];
double dh_final[4][4];


int zero = 0;
int one = 1;

void read_xml() {
	
    string path;
    cout << "Enter the DH Parameter file path"<<endl;
    getline(cin,path);
	
    ptree pt;
    read_xml(path, pt);
    BOOST_FOREACH(ptree::value_type &child, pt.get_child("dh_parameters.joint_parameters"))
    {
        cout << "Node Name: " << child.first<<endl;
        cout << "Theta: " << child.second.get<double>("<xmlattr>.theta");
        j_vals.push_back(child.second.get<double>("<xmlattr>.theta")*(3.14/180));
        cout << " Alpha: " << child.second.get<double>("<xmlattr>.alpha");
        j_vals.push_back(child.second.get<double>("<xmlattr>.alpha"));
        cout << " a: " << child.second.get<double>("<xmlattr>.a");
        j_vals.push_back(child.second.get<double>("<xmlattr>.a"));
        cout << " d: " << child.second.get<double>("<xmlattr>.d");
        j_vals.push_back(child.second.get<double>("<xmlattr>.d"));
        cout << "\n";
        j_vals.shrink_to_fit();
    }
}

double create_DH(double theta, double alpha,  double a, double d, double dh_initi[][4]) {

    double theta_init = theta;
    double alpha_init = alpha;
    double d_init = d;
    double a_init = a;
    cout << theta_init << endl;
    cout << alpha_init << endl;
    cout << d_init << endl;
    cout << a_init << endl;

    for (int i = 0; i <= 3; i++) {
        for (int j = 0; j <= 3; j++) {
            if (i == 0 && j == 0) { cout << i << " " << j << endl; double c00 = cos(theta_init); dh_initi[i][j] = c00;}
            else if (i == 0 && j == 1) { cout << i << " " << j << endl; double cs01 = -sin(theta_init) * cos(alpha_init);  dh_initi[i][j] = cs01; }
            else if (i == 0 && j == 2) { cout << i << " " << j << endl; double ss02 = sin(theta_init) * sin(alpha_init); dh_initi[i][j] = ss02; }
            else if (i == 0 && j == 3) { cout << i << " " << j << endl; double ac03 = a_init * cos(theta_init);  dh_initi[i][j] = ac03; }

            if (i == 1 && j == 0) { cout << i << " " << j << endl; double s10 = sin(theta_init); dh_initi[i][j] = s10; }
            else if (i == 1 && j == 1) { cout << i << " " << j << endl; double cc11 = cos(theta_init) * cos(alpha_init);  dh_initi[i][j] = cc11;  }
            else if (i == 1 && j == 2) { cout << i << " " << j << endl; double cs12 = -cos(theta_init) * sin(alpha_init);   dh_initi[i][j] = cs12; }
            else if (i == 1 && j == 3) { cout << i << " " << j << endl; double as13 = a_init * sin(theta_init); dh_initi[i][j] = as13; }

            if (i == 2 && j == 0) { cout << i << " " << j << endl; double z20 = zero; dh_initi[i][j] = z20; }
            else if (i == 2 && j == 1) { cout << i << " " << j << endl; double s21 = sin(alpha_init); dh_initi[i][j] = s21; }
            else if (i == 2 && j == 2) { cout << i << " " << j << endl; double c22 = cos(alpha_init); dh_initi[i][j] = c22;}
            else if (i == 2 && j == 3) { cout << i << " " << j << endl; double d23 = d_init; dh_initi[i][j] = d23; }

            if (i == 3 && j == 0) { cout << i << " " << j << endl; double z30 = zero; dh_initi[i][j] = z30; }
            else if (i == 3 && j == 1) { cout << i << " " << j << endl; double z31 = zero; dh_initi[i][j] = z31; }
            else if (i == 3 && j == 2) { cout << i << " " << j << endl; double z32 = zero; dh_initi[i][j] = z32; }
            else if (i == 3 && j == 3) { cout << i << " " << j << endl; double o33 = one;  dh_initi[i][j] = o33; }

            //if (i == 0 && j == 0) { cout << i << " " << j << endl; double c00 = cos(theta_init); *dh_initi= c00; dh_initi = dh_initi + 1; cout << c00; }
            //else if (i == 0 && j == 1) { cout << i << " " << j << endl; double cs01 = -sin(theta_init) * cos(alpha_init);  *dh_initi= cs01; dh_initi = dh_initi+1; cout << cs01;}
            //else if (i == 0 && j == 2) { cout << i << " " << j << endl; double ss02 = sin(theta_init) * sin(alpha_init); *dh_initi= ss02; dh_initi = dh_initi+1;}
            //else if (i == 0 && j == 3) { cout << i << " " << j << endl; double ac03 = a_init * cos(theta_init);  *dh_initi= ac03; dh_initi = dh_initi+1;}

            //if (i == 1 && j == 0) { cout << i << " " << j << endl; double s10 = sin(theta_init); *dh_initi= s10; dh_initi = dh_initi + 1;}
            //else if (i == 1 && j == 1) { cout << i << " " << j << endl; double cc11 = cos(theta_init) * cos(alpha_init);  *dh_initi= cc11; dh_initi = dh_initi + 1;}
            //else if (i == 1 && j == 2) { cout << i << " " << j << endl; double cs12 = -cos(theta_init) * sin(alpha_init);   *dh_initi= cs12; dh_initi = dh_initi + 1;}
            //else if (i == 1 && j == 3) { cout << i << " " << j << endl; double as13 = a_init * sin(theta_init); *dh_initi= as13; dh_initi = dh_initi + 1;}

            //if (i == 2 && j == 0) { cout << i << " " << j << endl; double z20 = zero; *dh_initi= z20; dh_initi = dh_initi + 1;}
            //else if (i == 2 && j == 1) { cout << i << " " << j << endl; double s21 = sin(alpha_init); *dh_initi= s21; dh_initi = dh_initi + 1;}
            //else if (i == 2 && j == 2) { cout << i << " " << j << endl; double c22 = cos(alpha_init); *dh_initi= c22; dh_initi = dh_initi + 1;}
            //else if (i == 2 && j == 3) { cout << i << " " << j << endl; double d23 = d_init; *dh_initi= d23; dh_initi = dh_initi + 1; }

            //if (i == 3 && j == 0) { cout << i << " " << j << endl; double z30 = zero; *dh_initi= z30; dh_initi = dh_initi + 1;}
            //else if (i == 3 && j == 1) { cout << i << " " << j << endl; double z31 = zero; *dh_initi= z31; dh_initi = dh_initi + 1;}
            //else if (i == 3 && j == 2) { cout << i << " " << j << endl; double z32 = zero; *dh_initi= z32; dh_initi = dh_initi + 1;}
            //else if (i == 3 && j == 3) { cout << i << " " << j << endl; double o33 = one;  *dh_initi= o33; dh_initi = dh_initi + 1;}//------------pointer loop
        }
    }
    return 0;
}

void matrix_mul() {

    int flag = 1;
    double  sum=0.00000;

    while(flag<=5){
        if (flag == 1) {
            cout << "Enter 1" << endl;
            for (int a = 0; a <= 3; a++) {
                for (int b = 0; b <= 3; b++) {
                    for (int c = 0; c <= 3; c++) {
                        sum = sum + j1_dh[a][c] * j2_dh[c][b];
                    }
                    dh_final[a][b] = sum;
                }
            }
        }
        if (flag == 2) {
            cout << "Enter 2" << endl;
            for (int a = 0; a <= 3; a++) {
                for (int b = 0; b <= 3; b++) {
                    for (int c = 0; c <= 3; c++) {
                        sum = sum + dh_final[a][c] * j3_dh[c][b];
                    }
                    dh_final[a][b] = sum;
                }
            }
        }
        if (flag == 3) {
            cout << "Enter 3" << endl;
            for (int a = 0; a <= 3; a++) {
                for (int b = 0; b <= 3; b++) {
                    for (int c = 0; c <= 3; c++) {
                        sum = sum + dh_final[a][c] * j4_dh[c][b];
                    }
                    dh_final[a][b] = sum;
                }
            }
        }
        if (flag == 4) {
            cout << "Enter 4" << endl;
            for (int a = 0; a <= 3; a++) {
                for (int b = 0; b <= 3; b++) {
                    for (int c = 0; c <= 3; c++) {
                        sum = sum + dh_final[a][c] * j5_dh[c][b];
                    }
                    dh_final[a][b] = sum;
                }
            }
        }
        if (flag == 5) {
            cout << "Enter 5" << endl;
            for (int a = 0; a <= 3; a++) {
                for (int b = 0; b <= 3; b++) {
                    for (int c = 0; c <= 3; c++) {
                        sum = sum + dh_final[a][c] * j6_dh[c][b];
                    }
                    dh_final[a][b] = sum;
                }
            }
        }
        flag = flag + 1;
    }

    cout << "\n Ai" << endl;
    for (int e = 0; e <= 3; e++) {
        for (int f = 0; f <= 3; f++) {
            cout << dh_final[e][f] << " ";
        }
        cout << "\n" << endl;
    }
}

double compute_vals() {

    double posx = dh_final[0][4];
    double posy = dh_final[1][4];
    double posz = dh_final[2][4];

    double rx = atan2(dh_final[2][0],sqrt(pow(dh_final[0][0],2+pow(dh_final[1][0],2))));
    double ry = atan2(dh_final[2][0]/cos(rx), dh_final[0][0]/cos(rx));
    double rz = atan2(dh_final[2][1]/cos(rx), dh_final[2][2] / cos(rx));
    cout << rx << " "<<ry<< " "<<rz << " "<<posx << " "<<posy<< " "<<posz;

    return posx, posy, posz, rx, ry, rz;
}

int main(int argc, char** argv) {//driver


    cout << "Getting Started with DH Matrix Compuatation" << endl;
   

    int m = 0;
    read_xml();//-------------1

    for (int r = 0; r <= 23; r++) {
        cout << j_vals.at(r) << " ";
    }
    
    cout <<"\n";

    while (m <= 20) {//----------------2
        if (m==0){ create_DH(j_vals.at(m), j_vals.at(m + 1), j_vals.at(m + 2), j_vals.at(m + 3), j1_dh); }
        if (m == 4) { create_DH(j_vals.at(m), j_vals.at(m + 1), j_vals.at(m + 2), j_vals.at(m + 3), j2_dh); }
        if (m == 8) { create_DH(j_vals.at(m), j_vals.at(m + 1), j_vals.at(m + 2), j_vals.at(m + 3), j3_dh); }
        if (m == 12) { create_DH(j_vals.at(m), j_vals.at(m + 1), j_vals.at(m + 2), j_vals.at(m + 3), j4_dh); }
        if (m == 16) { create_DH(j_vals.at(m), j_vals.at(m + 1), j_vals.at(m + 2), j_vals.at(m + 3), j5_dh); }
        if (m == 20) { create_DH(j_vals.at(m), j_vals.at(m + 1), j_vals.at(m + 2), j_vals.at(m + 3), j6_dh); }
        m = m + 4;
    }

    //while (m <= 20) {//------------2 automated
    //    cout << "While loop" << endl;
    //    create_DH(j_vals.at(m), j_vals.at(m+1), j_vals.at(m+2), j_vals.at(m+3), dh_tables[(m/4)]);
    //    m = m + 4; }

    cout << "\n J1" << endl;
    for (int m = 0; m <= 3; m++) {
        for (int n = 0; n <= 3; n++) {
            cout << j1_dh[m][n] << " ";}
        cout<<"\n" << endl;}

    cout << "\n J2" << endl;
    for (int e = 0; e <= 3; e++) {
        for (int f = 0; f <= 3; f++) {
            cout << j2_dh[e][f] << " ";}
        cout << "\n" << endl;}

    cout << "\n J3" << endl;
    for (int e = 0; e <= 3; e++) {
        for (int f = 0; f <= 3; f++) {
            cout << j3_dh[e][f] << " ";
        }
        cout << "\n" << endl;
    }

    cout << "\n J4" << endl;
    for (int e = 0; e <= 3; e++) {
        for (int f = 0; f <= 3; f++) {
            cout << j4_dh[e][f] << " ";
        }
        cout << "\n" << endl;
    }

    cout << "\n J5" << endl;
    for (int e = 0; e <= 3; e++) {
        for (int f = 0; f <= 3; f++) {
            cout << j5_dh[e][f] << " ";
        }
        cout << "\n" << endl;
    }

    cout << "\n J6" << endl;
    for (int e = 0; e <= 3; e++) {
        for (int f = 0; f <= 3; f++) {
            cout << j6_dh[e][f] << " ";
        }
        cout << "\n" << endl;
    }

    matrix_mul();//-------------3
    cout<<compute_vals()<<endl;

    system("pause>0");
}