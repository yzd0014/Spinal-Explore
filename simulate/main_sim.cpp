#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <fstream>
#include <functional>
#include <algorithm>

#define _USE_MATH_DEFINES
#include <cmath>

#include <mujoco/mujoco.h>
#include <mujoco/mjxmacro.h>
#include "simulate.h"
#include "array_safety.h"

#include "WriteToFile.h"
#include "Neuron.h"
#include <Eigen/Eigen>
using namespace Eigen;

char error[1000];
int mode = 0;

mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

bool start_sim = true;
bool next_step = false;
unsigned int key_s_counter = 0;

bool startLog = false;
bool startPrinting = false;
int visualization = 0;
bool mlock = true;
int mCounter = 0;

//event controller parameters
mjtNum dTheta = 0;
mjtNum c_a[2];
mjtNum c_b;
mjtNum c_Kp = 5;
mjtNum c_theta_bar = 0;
mjtNum l_bar[2];

mjtNum Kv = 0.5, Kl = 0.7;
ActuationNeuron actuationNeurons[2];

//compute length
mjtNum S = 0.1;
mjtNum L = 0;
mjtNum angleOffset = 0;

mjtNum t_last = 0;
namespace spine
{
    mjtNum err0 = 0;
    mjtNum err1 = 0;
}
namespace PD
{
    mjtNum err0 = 0;
    mjtNum err1 = 0;
}
// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        mj_resetData(m, d);
        mj_forward(m, d);
        start_sim = false;
    }
    if (act == GLFW_PRESS && key == GLFW_KEY_SPACE)
    {
        start_sim = !start_sim;
    }
    if (key == GLFW_KEY_S)
    {
        key_s_counter++;
        //std::cout << key_s_counter << std::endl;
        if (act == GLFW_PRESS)
            next_step = true;
        if (key_s_counter >= 4)
        {
            key_s_counter = 4;
            next_step = true;
        }     
        if (act == GLFW_RELEASE)
        { 
            key_s_counter = 0;
        }
    }

    if (act == GLFW_PRESS && key == GLFW_KEY_L) startLog = !startLog;
    if (act == GLFW_PRESS && key == GLFW_KEY_P) startPrinting = !startPrinting;

    if (act == GLFW_PRESS && key == GLFW_KEY_D)
    {
        if (mode == 0 || mode == 1)
        {
            dTheta += 0.05;
        }
        else
        {
            //c_theta_bar += 0.05;
            d->qvel[0] = 1;
        }

    }
    if (act == GLFW_PRESS && key == GLFW_KEY_A)
    {
        if (mode == 0 || mode == 1)
        {
            dTheta -= 0.05;
        }
        else
        {
            //c_theta_bar -= 0.05;
            l_bar[1] -= 0.01;
        }  
    }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right)
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
        glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

void SpikingController(const mjModel* m, mjData* d)
{
    dTheta = c_Kp * (c_theta_bar - d->qpos[0]);
    for (int i = 0; i < 2; i++)
    {
        actuationNeurons[i].thresholdLength = c_a[i] * dTheta + c_b;
    }

    d->ctrl[1] = actuationNeurons[0].Update();
    d->ctrl[2] = actuationNeurons[1].Update();
    //std::cout << actuationNeurons[0].output_f << std::endl;
    //fs << d->time << ", " << d->qpos[0] << ", " << d->qvel[0] << ", " << d->act[0] << ", " << actuationNeurons[0].input2 << ", " << d->act[1] << ", " << actuationNeurons[1].input2 << "\n";
}

void GetLength(mjtNum i_angle, mjtNum & o_length0, mjtNum & o_length1)
{
    mjtNum lSquare0 = S * S + L * L - 2 * S * L * cos(M_PI * 0.5 - i_angle - angleOffset);
    mjtNum lSquare1 = S * S + L * L - 2 * S * L * cos(M_PI * 0.5 + i_angle - angleOffset);
    o_length0 = sqrt(lSquare0);
    o_length1 = sqrt(lSquare1);
}

void BaseLineController(const mjModel* m, mjData* d)
{
    mjtNum Kp = 50;
    mjtNum Kd = 8;
    mjtNum Kii = 8;
    //mjtNum freq = 0.5;
    //dTheta = 0.8 * sin(2 * 3.14 * d->time * freq);
   /* if (d->time - t_last > 3)
    {
        t_last = d->time;
        dTheta *= -1;
    }*/
    mjtNum gl0 = 0;
    mjtNum gl1 = 0;
    GetLength(dTheta, gl0, gl1);
    /*gl0 *= 0.5;
    gl1 *= 0.5;*/
    spine::err0 += (d->actuator_length[1] - gl0) * m->opt.timestep;
    spine::err1 += (d->actuator_length[2] - gl1) * m->opt.timestep;
    mjtNum mb = 0.1;//larger mb results smaller co-contraction
    mjtNum l0 = std::max(mb - (Kp * (d->actuator_length[1] - gl0) + Kd * d->actuator_velocity[1] + Kii * spine::err0), 0.0);
    mjtNum l1 = std::max(mb - (Kp * (d->actuator_length[2] - gl1) + Kd * d->actuator_velocity[2] + Kii * spine::err1), 0.0);

    /*mjtNum l0 = 0;
    mjtNum l1 = 0;
    GetLength(dTheta, l0, l1);
    l0 *= 0.5;
    l1 *= 0.5;*/

    mjtNum l_spindle = 0.05 * d->actuator_velocity[2] + d->actuator_length[2];
    mjtNum r_spindle = 0.05 * d->actuator_velocity[1] + d->actuator_length[1];
  
    mjtNum l_diff = std::max(l_spindle - r_spindle + (l0 - l1), 0.0);
    mjtNum r_diff = std::max(r_spindle - l_spindle + (l1 - l0), 0.0);
   /* mjtNum l_diff = 0;
    mjtNum r_diff = 0;*/

    mjtNum ctrlCoeff = 1;
    d->ctrl[1] = std::max(r_spindle - l0 - l_diff, 0.0) * ctrlCoeff;
    d->ctrl[2] = std::max(l_spindle - l1 - r_diff, 0.0) * ctrlCoeff;
      
    mjtNum torque0 = d->actuator_force[1] * d->actuator_moment[1];
    mjtNum torque1 = d->actuator_force[2] * d->actuator_moment[2];
    mjtNum netTorque = torque0 + torque1;

    //std::cout << d->qpos[0] << ", " << ml1 << ", " << ml0 << ", " << d->actuator_length[2] << ", " << d->actuator_length[1] << std::endl;
    //std::cout << d->ctrl[2] << ", " << d->ctrl[1] << ", " << l_diff << ", " << r_diff << ", " << l1 << ", " << l0 << std::endl;
    fs << d->time << ", " << d->qpos[0] << std::endl;
    //std::cout << gl1 << ", " << gl0 << "\n";
    //std::cout << d->actuator_length[2] << ", " << d->actuator_length[1] << "\n";
    //std::cout << l1 << ", " << l0 << "\n\n";
}

void RateContorller(const mjModel* m, mjData* d)
{
    mjtNum Kv = 0.03;
    mjtNum normalizatinCoeff = 0.75;
    mjtNum velo_r = std::max(d->actuator_velocity[1], 0.0);
    mjtNum velo_l = std::max(d->actuator_velocity[2], 0.0);
    NeuronRateModel::rates[2] = (d->actuator_length[1] + Kv * velo_r) / normalizatinCoeff;
    NeuronRateModel::rates[3] = (d->actuator_length[2] + Kv * velo_l) / normalizatinCoeff;
    NeuronRateModel::rates[4] = l_bar[0];
    NeuronRateModel::rates[5] = l_bar[1];

    NeuronRateModel::UpdateRates();
    d->ctrl[1] = NeuronRateModel::rates[0];
    d->ctrl[2] = NeuronRateModel::rates[1];
    if (startPrinting) std::cout << d->qpos[0] << std::endl;
    std::cout << NeuronRateModel::rates[6] << ", " << NeuronRateModel::rates[7] << std::endl;
}

void PDController(const mjModel* m, mjData* d)
{
    mjtNum Kp = 40;
    mjtNum Ki = 2;
    mjtNum Kd = 5;
    mjtNum l0 = 0;
    mjtNum l1 = 0;
    mjtNum freq = 2;
    //dTheta = 0.8 * sin(2 * 3.14 * d->time * freq);
   /* if (d->time - t_last > 3)
    {
        t_last = d->time;
        dTheta *= -1;
    }*/
    GetLength(dTheta, l0, l1); //l0 and l1 decides co-contraction 
   /* l0 *= 0.5;
    l1 *= 0.5;*/
    PD::err0 += (d->actuator_length[1] - l0) * m->opt.timestep;
    PD::err1 += (d->actuator_length[2] - l1) * m->opt.timestep;
    mjtNum r_spindle = 0.05 * d->actuator_velocity[1] + d->actuator_length[1];
    mjtNum l_spindle = 0.05 * d->actuator_velocity[2] + d->actuator_length[2];
    d->ctrl[1] = Kp * (r_spindle - l0) + Kd * d->actuator_velocity[1] + Ki * PD::err0;
    d->ctrl[2] = Kp * (l_spindle - l1) + Kd * d->actuator_velocity[2] + Ki * PD::err1;
    
    //d->ctrl[0] = 0.4 * sin(2 * 3.14 * d->time * freq);
    //fs << d->time << ", " << d->qpos[0] << "\n";
}

mjtNum ComputeController(mjtNum maxTheta, mjtNum length0, mjtNum lengthMin)
{
    mjtNum output;
    output = (lengthMin - length0) / -maxTheta;
    return output;
}

void InitializeController(const mjModel* m, mjData* d)
{
    mode = 1;
    mj_forward(m, d);//make sure whatever is defined in xml is available in the following initialization
    if (mode == 0) {
        mjcb_control = PDController;
        
        mjtNum h = 0.6;
        mjtNum w = 0.06;
        L = sqrt(h * h + w * w);
        angleOffset = atan(w / h);

        //d->qvel[0] = 2;
        dTheta = 0.7;
    }
    else if (mode == 1)
    {
        mjcb_control = BaseLineController;
        
        mjtNum h = 0.6;
        mjtNum w = 0.06;
        L = sqrt(h * h + w * w);
        angleOffset = atan(w / h);
        //l_bar[0] = 0.45;
        //l_bar[1] = 0.55;
        //d->qvel[0] = 2;
        dTheta = 0.7;
    }
    else if (mode == 2)
    {
        mCounter = 0;

        mjcb_control = SpikingController;
        l_bar[0] = 0.4;
        l_bar[1] = 0.4;

        c_theta_bar = 0;
        c_b = d->ten_length[0];
        c_a[1] = ComputeController(3.14, c_b, 0);
        c_a[0] = -c_a[1];

        //d->ctrl[0] = -1.41;
        //d->ctrl[0] = 0;
        //d->qpos[0] = 0.3;
        //d->qvel[0] = 0.5;

        d->userdata[0] = -10;
        d->userdata[1] = 10;
        d->userdata[2] = 0;

        actuationNeurons[0] = ActuationNeuron(m, d, l_bar[0], 1, 2);
        actuationNeurons[1] = ActuationNeuron(m, d, l_bar[1], 2, 1);
    }
    else if (mode == 3)
    {
        l_bar[0] = 0.4;
        l_bar[1] = 0.4;
        d->qpos[0] = 0.5;

        int neuronNum = 8;
        NeuronRateModel::rates.resize(neuronNum);
        NeuronRateModel::sum_old.resize(neuronNum);
        NeuronRateModel::positive_sum_old.resize(neuronNum);
        NeuronRateModel::neighbors.resize(neuronNum);
        NeuronRateModel::inputWeights.resize(neuronNum);
        NeuronRateModel::inputSigns.resize(neuronNum);
        for (int i = 0; i < neuronNum; i++)
        {
            NeuronRateModel::rates[i] = 0;
            NeuronRateModel::sum_old[i] = 0;
            NeuronRateModel::positive_sum_old[i] = 0;
        }
        //initialize right motoneuron 
        NeuronRateModel::neighbors[0].push_back(2);
        NeuronRateModel::inputWeights[0].push_back(1);
        NeuronRateModel::inputSigns[0].push_back(1);
        NeuronRateModel::neighbors[0].push_back(4);
        NeuronRateModel::inputWeights[0].push_back(1);
        NeuronRateModel::inputSigns[0].push_back(-1);
        NeuronRateModel::neighbors[0].push_back(7);
        NeuronRateModel::inputWeights[0].push_back(1);
        NeuronRateModel::inputSigns[0].push_back(-1);

        //initialize left motoneuron
        NeuronRateModel::neighbors[1].push_back(3);
        NeuronRateModel::inputWeights[1].push_back(1);
        NeuronRateModel::inputSigns[1].push_back(1);
        NeuronRateModel::neighbors[1].push_back(5);
        NeuronRateModel::inputWeights[1].push_back(1);
        NeuronRateModel::inputSigns[1].push_back(-1);
        NeuronRateModel::neighbors[1].push_back(6);
        NeuronRateModel::inputWeights[1].push_back(1);
        NeuronRateModel::inputSigns[1].push_back(-1);

        //initialize right inter neuron
        NeuronRateModel::neighbors[6].push_back(7);
        NeuronRateModel::inputWeights[6].push_back(0.8);
        NeuronRateModel::inputSigns[6].push_back(-1);
        NeuronRateModel::neighbors[6].push_back(2);
        NeuronRateModel::inputWeights[6].push_back(1);
        NeuronRateModel::inputSigns[6].push_back(1);

        //initialize left inter neuron
        NeuronRateModel::neighbors[7].push_back(6);
        NeuronRateModel::inputWeights[7].push_back(0.8);
        NeuronRateModel::inputSigns[7].push_back(-1);
        NeuronRateModel::neighbors[7].push_back(3);
        NeuronRateModel::inputWeights[7].push_back(1);
        NeuronRateModel::inputSigns[7].push_back(1);

        mjcb_control = RateContorller;
    }
    mj_forward(m, d);//make sure initialization is updated before the starte of simulation
    //startLog = true;
}

int main(void)
{ 
    std::cout << "Do you want to have visualization (0 for no, 1 for yes): ";
    std::cin >> visualization;
    fs.open("../matlab/plot.csv", std::ios::out | std::ios::app);
    // load model from file and check for errors
    m = mj_loadXML("muscle_control_narrow.xml", NULL, error, 1000);
    //m = mj_loadXML("muscle_default.xml", NULL, error, 1000);
    if (!m)
    {
        printf("%s\n", error);
        return 1;
    }

    if (visualization == 1)
    {
        // make data corresponding to model
        d = mj_makeData(m);

        // init GLFW, create window, make OpenGL context current, request v-sync
        // init GLFW
        if (!glfwInit())
            mju_error("Could not initialize GLFW");
        GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);

        // initialize visualization data structures
        mjv_defaultCamera(&cam);
        //mjv_defaultPerturb(&pert);
        mjv_defaultScene(&scn);
        mjv_defaultOption(&opt);
        mjr_defaultContext(&con);

        // create scene and context
        mjv_makeScene(m, &scn, 1000);
        mjr_makeContext(m, &con, mjFONTSCALE_100);

        // ... install GLFW keyboard and mouse callbacks
         // install GLFW mouse and keyboard callbacks
        glfwSetKeyCallback(window, keyboard);
        glfwSetCursorPosCallback(window, mouse_move);
        glfwSetMouseButtonCallback(window, mouse_button);
        glfwSetScrollCallback(window, scroll);
        
        double arr_view[] = { 90, -5, 3, 0, -0.000000, 1.5 };
        cam.azimuth = arr_view[0];
        cam.elevation = arr_view[1];
        cam.distance = arr_view[2];
        cam.lookat[0] = arr_view[3];
        cam.lookat[1] = arr_view[4];
        cam.lookat[2] = arr_view[5];

        InitializeController(m, d);
        // run main loop, target real-time simulation and 60 fps rendering
        while (!glfwWindowShouldClose(window)) {
            if (start_sim || next_step)
            {
                // advance interactive simulation for 1/60 sec
                //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
                //  this loop will finish on time for the next frame to be rendered at 60 fps.
                //  Otherwise add a cpu timer and exit this loop when it is time to render.
                mjtNum simstart = d->time;
                while (d->time - simstart < 1.0 / 60.0)
                {
                    //if (abs(d->qpos[0]) < 0.01 && abs(d->qvel[0]) < 0.001) break;
                    mj_step(m, d);  
                }
               /* if (abs(d->qpos[0]) < 0.01 && abs(d->qvel[0]) < 0.001)
                {
                    std::cout << d->time;
                    break;
                }*/
                next_step = false;
            }
            // get framebuffer viewport
            mjrRect viewport = { 0, 0, 0, 0 };
            glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

            // update scene and render
            mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
            mjr_render(viewport, &scn, &con);

            // swap OpenGL buffers (blocking call due to v-sync)
            glfwSwapBuffers(window);

            // process pending GUI events, call GLFW callbacks
            glfwPollEvents();
        }

        // close GLFW, free visualization storage
        glfwTerminate();
        mjv_freeScene(&scn);
        mjr_freeContext(&con);

        mj_deleteData(d);
    }
    else if (visualization == 0)
    {
        mjtNum gridStep = 0.02;
        for (Kv = 0; Kv <= 1 + 0.1 * gridStep; Kv += gridStep)
        {
            for (Kl = 0; Kl <= 1 + 0.1 * gridStep; Kl += gridStep)
            {
                d = mj_makeData(m);
                InitializeController(m, d);
                while (d->time <= 10)
                {
                    mj_step(m, d);
                    if (abs(d->qpos[0]) < 0.01 && abs(d->qvel[0]) < 0.001) break;
                }
                //Vector3d currVec(0.2 * d->qpos[0], 0.2 * d->qvel[0], 0.6 * d->time);
                //mjtNum err = currVec.norm();
                fs << d->time;
                if (Kl + gridStep <= 1 + 0.1 * gridStep) fs << ", ";
                
                mj_deleteData(d);
                d = nullptr;
                mjcb_control = 0;
            }
            mCounter++;
            std::cout << mCounter << " row finised..." << std::endl;
            fs << "\n";
        }
    }
    else
    {
        std::cout << "please select correct visualization mode to run!" << std::endl;
    }
    mj_deleteModel(m);
    fs.close();
    return 0;
}