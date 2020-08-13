/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <sstream>
#include "unitree_legged_sdk/threshold_filter.h"


using namespace std;
using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom(): control(LeggedType::A1, LOWLEVEL), udp() {
        control.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();
    void set_jointposes(std::vector< std::vector<float> >);

    Control control;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    std::vector< std::vector<float> > result;
    float qInit[12]={0};
    float qCur[12]={0};
    float qDes[12]={0};
    //float sin_mid_q[3] = {0.0, 1.2, -2.0};
    float standup_init[12]={0};
    float Kp = 0;  
    float Kd = 0;
    double time_consume = 0;
    int rate_count = 0;
    int sin_count = 0;
    int motiontime = 0;
    int count = 0;
    float dt = 0.002;     // 0.001~0.01
    
    legged_robot::ThresholdFilter filters[12];

    ofstream outfile;
};

void Custom::UDPRecv()
{  
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

void Custom::set_jointposes(std::vector< std::vector<float> > input_data)
{
    result = input_data;
}

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos*(1-rate) + targetPos*rate;
    return p;
}

void Custom::RobotControl() 
{

    
    motiontime++;
    udp.GetRecv(state);
    // printf("%d  %f\n", motiontime, state.motorState[FR_2].q);

    // gravity compensation
    cmd.motorCmd[FR_0].tau = -0.65f;
    cmd.motorCmd[FL_0].tau = +0.65f;
    cmd.motorCmd[RR_0].tau = -0.65f;
    cmd.motorCmd[RL_0].tau = +0.65f;


    std::copy(result.at(0).begin(), result.at(0).end(), standup_init);  // initialising standup_init with the first row of the CSV record
    
    
    // if( motiontime >= 100){
    if( motiontime >= 0){
        // first, get record initial position
        // if( motiontime >= 100 && motiontime < 500){
        if( motiontime >= 0 && motiontime < 10){
            qInit[0] = state.motorState[FL_0].q;
            qInit[1] = state.motorState[FL_1].q;
            qInit[2] = state.motorState[FL_2].q;
            qInit[3] = state.motorState[FR_0].q;
            qInit[4] = state.motorState[FR_1].q;
            qInit[5] = state.motorState[FR_2].q;
            qInit[6] = state.motorState[RL_0].q;
            qInit[7] = state.motorState[RL_1].q;
            qInit[8] = state.motorState[RL_2].q;
            qInit[9] = state.motorState[RR_0].q;
            qInit[10] = state.motorState[RR_1].q;
            qInit[11] = state.motorState[RR_2].q;
            for (uint i=0; i<12; i++) {
               filters[i].SetXfilter(standup_init[i]);
               double max_vel = 1; // rad/s
               filters[i].SetLimit(max_vel*dt);
            }
        }
        // second, move to the origin point of a sine movement with Kp Kd
        // if( motiontime >= 500 && motiontime < 1500){
        if( motiontime >= 10 && motiontime < 2500)
        {
            rate_count++;
            double rate = rate_count/2000.0;                       // needs count to 200
            Kp = 5.0; 
            Kd = 1.0;
            
            for(int i=0; i<12; i++) 
                qDes[i] = jointLinearInterpolation(qInit[i], standup_init[i], rate);
        }
        
        if (motiontime >= 2000)
        {   
            count+=2;
            std::copy(result.at(count).begin(), result.at(count).end(), qDes);    // writing values from csv to qDes
            for (uint i=0; i<12; i++) qDes[i]=filters[i].Filter(qDes[i]);
        }


        cmd.motorCmd[FL_0].q = qDes[0];
        cmd.motorCmd[FL_0].dq = 0;
        cmd.motorCmd[FL_0].Kp = Kp;
        cmd.motorCmd[FL_0].Kd = Kd;
        cmd.motorCmd[FL_0].tau = +0.65f;

        cmd.motorCmd[FL_1].q = qDes[1];
        cmd.motorCmd[FL_1].dq = 0;
        cmd.motorCmd[FL_1].Kp = Kp;
        cmd.motorCmd[FL_1].Kd = Kd;
        cmd.motorCmd[FL_1].tau = 0.0f;

        cmd.motorCmd[FL_2].q =  qDes[2];
        cmd.motorCmd[FL_2].dq = 0;
        cmd.motorCmd[FL_2].Kp = Kp;
        cmd.motorCmd[FL_2].Kd = Kd;
        cmd.motorCmd[FL_2].tau = 0.0f;

        cmd.motorCmd[FR_0].q =  qDes[3];
        cmd.motorCmd[FR_0].dq = 0;
        cmd.motorCmd[FR_0].Kp = Kp;
        cmd.motorCmd[FR_0].Kd = Kd;
        cmd.motorCmd[FR_0].tau = -0.65f;

        cmd.motorCmd[FR_1].q =  qDes[4];
        cmd.motorCmd[FR_1].dq = 0;
        cmd.motorCmd[FR_1].Kp = Kp;
        cmd.motorCmd[FR_1].Kd = Kd;
        cmd.motorCmd[FR_1].tau = 0.0f;

        cmd.motorCmd[FR_2].q =  qDes[5];
        cmd.motorCmd[FR_2].dq = 0;
        cmd.motorCmd[FR_2].Kp = Kp;
        cmd.motorCmd[FR_2].Kd = Kd;
        cmd.motorCmd[FR_2].tau = 0.0f;

        cmd.motorCmd[RL_0].q =  qDes[6];
        cmd.motorCmd[RL_0].dq = 0;
        cmd.motorCmd[RL_0].Kp = Kp;
        cmd.motorCmd[RL_0].Kd = Kd;
        cmd.motorCmd[RL_0].tau = +0.65f;

        cmd.motorCmd[RL_1].q =  qDes[7];
        cmd.motorCmd[RL_1].dq = 0;
        cmd.motorCmd[RL_1].Kp = Kp;
        cmd.motorCmd[RL_1].Kd = Kd;
        cmd.motorCmd[RL_1].tau = 0.0f;

        cmd.motorCmd[RL_2].q =  qDes[8];
        cmd.motorCmd[RL_2].dq = 0;
        cmd.motorCmd[RL_2].Kp = Kp;
        cmd.motorCmd[RL_2].Kd = Kd;
        cmd.motorCmd[RL_2].tau = 0.0f;

        cmd.motorCmd[RR_0].q =  qDes[9];
        cmd.motorCmd[RR_0].dq = 0;
        cmd.motorCmd[RR_0].Kp = Kp;
        cmd.motorCmd[RR_0].Kd = Kd;
        cmd.motorCmd[RR_0].tau = -0.65f;

        cmd.motorCmd[RR_1].q =  qDes[10];
        cmd.motorCmd[RR_1].dq = 0;
        cmd.motorCmd[RR_1].Kp = Kp;
        cmd.motorCmd[RR_1].Kd = Kd;
        cmd.motorCmd[RR_1].tau = 0.0f;

        cmd.motorCmd[RR_2].q =  qDes[11];
        cmd.motorCmd[RR_2].dq = 0;
        cmd.motorCmd[RR_2].Kp = Kp;
        cmd.motorCmd[RR_2].Kd = Kd;
        cmd.motorCmd[RR_2].tau = 0.0f;


        qCur[0] = state.motorState[FL_0].q;
        qCur[1] = state.motorState[FL_1].q;
        qCur[2] = state.motorState[FL_2].q;
        qCur[3] = state.motorState[FR_0].q;
        qCur[4] = state.motorState[FR_1].q;
        qCur[5] = state.motorState[FR_2].q;
        qCur[6] = state.motorState[RL_0].q;
        qCur[7] = state.motorState[RL_1].q;
        qCur[8] = state.motorState[RL_2].q;
        qCur[9] = state.motorState[RR_0].q;
        qCur[10] = state.motorState[RR_1].q;
        qCur[11] = state.motorState[RR_2].q;


        for(int i=0; i<12; i++){
            if(i==0)
                outfile << qCur[i];
            else if(i > 0 && i < 11)
                outfile << "," << qCur[i];
            else
                outfile << "," << qCur[i] << endl;
        }

    }

    if(motiontime > 10){
        control.PositionLimit(cmd);
        control.PowerProtect(cmd, state, 1);
        control.PositionProtect(cmd, state, 0.087);
    }

    udp.SetSend(cmd);

}


std::vector< std::vector<float> > CSV_reader()
{
    std::ifstream fin;
    fin.open("standup_back.csv");

    if (!fin)
    {
        std::cout << "Error opening file. \n";
        exit(0);
    }

    std::string temp, line;
    std::vector<std::vector<float> > result;
    std::vector<float> joint_pos;
    float val; 
    joint_pos.clear();
    while (std::getline(fin, line))
    {
        joint_pos.clear();
        std::stringstream ss(line);

        while(ss >> val)
        {
            joint_pos.push_back(val);
            if(ss.peek() == ',') ss.ignore();
        }
        result.push_back(joint_pos);           
    }

    return result;

}

int main(void)
{
    std::cout << "Control level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    
    Custom custom;
    custom.set_jointposes(CSV_reader());
    std::cout << "File read successfully \n";

    custom.outfile.open("result.csv");

    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while(1){
        sleep(10);
    };

    return 0; 
}
