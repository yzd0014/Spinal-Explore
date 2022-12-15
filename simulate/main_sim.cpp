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

#include <mujoco/mujoco.h>
#include <mujoco/mjxmacro.h>
#include "simulate.h"
#include "array_safety.h"

#include "MuscleLengthSensor.h"
//#include <Eigen/Eigen>
//using namespace Eigen;

char error[1000];

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

bool start_sim = false;
bool next_step = false;
unsigned int key_s_counter = 0;

std::fstream fs;
bool startLog = false;
bool startPrinting = false;
int visualization;
bool mlock = true;
int mCounter = 0;

mjtNum dt = 0.0000001;
mjtNum remainingTimeToSimulate = 0;
//event controller parameters
mjtNum dTheta = 0;
mjtNum c_a[2];
mjtNum c_b;
mjtNum c_Kp = 5;
mjtNum c_theta_bar = 0;
mjtNum q_bar = 1.3;
mjtNum refractory_dt[2];//0.005;
mjtNum u = 1;
mjtNum l_bar[2];
mjtNum event_times[2];
mjtNum dls[2];

mjtNum dt_p = 0.01;
int dt_p_ticks = 0;
int one_ticks;
int shift_ticks;
mjtNum sensor0[5];
mjtNum sensor0_filtered[5];
MuscleLengthSensor actuatorSensor[2];

void UpdateMaxMinPos(const mjModel* m, mjData* d)
{
    mjtNum currPos = d->qpos[0];
    //std::cout << currPos << std::endl;
    if (currPos > d->userdata[0])
    {
        d->userdata[0] = currPos;
    }
    if (currPos < d->userdata[1])
    {
        d->userdata[1] = currPos;
    }
    d->userdata[2] = d->userdata[0] - d->userdata[1];
}

void MyCallback(const mjModel* m, mjData* d)
{
    //d->ctrl[0] = 0.34*sin(2000 * 6.28 * d->time);
    //d->ctrl[0] = sin(d->time);
}

mjtNum FilterDyn(const mjModel* m, const mjData* d, int id)
{
    mjtNum output = 0;
    output = (d->ctrl[0] - d->act[0]) / -10;

    return output;
}
mjtNum GetLength(mjtNum input)
{
    mjtNum output = 0;
    if (input < 0.00012)
    {
        output = 833.333 * input;
    }
    else if (input >= 0.00012 && input < 0.00034)
    {
        output = 454.545 * input + 0.0454545;
    }
    else if (input >= 0.00034 && input < 0.0006)
    {
        output = 384.615 * input + 0.0692308;
    }
    else if (input >= 0.0006 && input < 0.001)
    {
        output = 250 * input + 0.15;
    }
    else if (input >= 0.001 && input < 0.0018)
    {
        output = 125 * input + 0.275;
    }
    else if (input >= 0.0018 && input < 0.0033)
    {
        output = 66.6667 * input + 0.38;
    }
    else if (input > 0.0033 && input < 0.0089)
    {
        output = 17.8571 * input + 0.541071;
    }

    return output;
}
/*
void EventLengthController(const mjModel* m, mjData* d)
{
    actuatorSensor[0].Update();
    actuatorSensor[1].Update();

    mjtNum dTheta = c_Kp * (c_theta_bar - d->qpos[0]);
    for (int i = 0; i < 2; i++)
    {
        //l_bar[i] = c_a[i] * dTheta + c_b;
        //dls[i] = d->ten_length[i] - l_bar[i];
        actuatorSensor[i].thresholdLength = c_a[i] * dTheta + c_b;
        if (actuatorSensor[i].thresholdLength < 0)
        {
            actuatorSensor[i].thresholdLength = 0;
        }
        if (actuatorSensor[i].thresholdLength > 0.8)
        {
            actuatorSensor[i].thresholdLength = 0.8;
        }
        dls[i] = GetLength(actuatorSensor[i].output);
        //dls[i] = 0;
        if (dls[i] < 0)
        {
            dls[i] = 0;
        }
        else if (dls[i] > 0.4)
        {
            dls[i] = 0.4;
        }
        //dl = 0.4;
        //refractory_dt[i] = -0.1245 * dl + 0.05;
        refractory_dt[i] = -2.4875 * dls[i] + 1;
        
        d->ctrl[i + 1] = 0;
        if (d->time - event_times[i] >= refractory_dt[i])
        {
            event_times[i] = d->time;
            d->ctrl[i+1] = u;
        }
    }
    //fs << d->time << ", " << d->qpos[0] << "\n";
    //mjtNum torque0 = d->actuator_force[1] * d->actuator_moment[1];
    //mjtNum torque1 = d->actuator_force[2] * d->actuator_moment[2];
    //mjtNum netTorque = torque0 + torque1;
    //std::cout << d->time << ", " << d->qpos[0] << ", " << torque0 << ", " << torque1 << ", " << netTorque << "\n";
}*/
void EventLengthController(const mjModel* m, mjData* d)
{
    dTheta = c_Kp * (c_theta_bar - d->qpos[0]);
    for (int i = 0; i < 2; i++)
    {
        actuatorSensor[i].thresholdLength = c_a[i] * dTheta + c_b;
    }
    
    d->ctrl[1] = 0;
    d->ctrl[2] = 0;
    actuatorSensor[0].thresholdReached = false;
    actuatorSensor[1].thresholdReached = false;

    mjtNum timeSimulated = 0;
    mjtNum currSimTime = d->time;
    if (remainingTimeToSimulate > 0)
    {
        //simulation
        actuatorSensor[0].Update(remainingTimeToSimulate, currSimTime, d->ctrl[1]);
        actuatorSensor[1].Update(remainingTimeToSimulate, currSimTime, d->ctrl[2]);
        timeSimulated += remainingTimeToSimulate;
    }
    
    while (1)
    {
        if (timeSimulated + dt >= m->opt.timestep)
        {
            mjtNum lastDt = m->opt.timestep - timeSimulated;
            remainingTimeToSimulate = timeSimulated + dt - m->opt.timestep;
            //simulation
            actuatorSensor[0].Update(lastDt, currSimTime, d->ctrl[1]);
            actuatorSensor[1].Update(lastDt, currSimTime, d->ctrl[2]);
            break;
        }
        //simualtion
        actuatorSensor[0].Update(dt, currSimTime, d->ctrl[1]);
        actuatorSensor[1].Update(dt, currSimTime, d->ctrl[2]);
        timeSimulated += dt;
        if (startLog) fs << d->time << ", " << actuatorSensor[1].input1Accum << ", " << actuatorSensor[1].input2Accum << "\n";
    }
    if (startPrinting)
    {
        {
            mjtNum output_f = actuatorSensor[0].output_f;
            mjtNum input1_f = actuatorSensor[0].threshold_f;
            mjtNum input2_f = actuatorSensor[0].curr_f;
            std::cout << "actuator1 threshold: " << actuatorSensor[0].thresholdLength << " outputf: " << output_f << " threshold_f: " << input1_f << " currlength_f: " << input2_f << " dtheta: " << dTheta << std::endl;
        }
        {
            mjtNum output_f = actuatorSensor[1].output_f;
            mjtNum input1_f = actuatorSensor[1].threshold_f;
            mjtNum input2_f = actuatorSensor[1].curr_f;
            std::cout << "actuator2 threshold: " << actuatorSensor[1].thresholdLength << " output_f: " << output_f << " threshold_f: " << input1_f << " currlength_f: " << input2_f << " dtheta: " << dTheta << std::endl;
        }
    }
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
        //actuatorSensor[0].thresholdLength -= 0.005;
        //actuatorSensor[1].thresholdLength += 0.005;
        c_theta_bar += 0.05;
    }
    if (act == GLFW_PRESS && key == GLFW_KEY_A)
    {
       //actuatorSensor[0].thresholdLength += 0.005;
       //actuatorSensor[1].thresholdLength -= 0.005;
       c_theta_bar -= 0.05;
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
void AngleController(const mjModel* m, mjData* d)
{
    mjtNum q_bar = 0.78;
    mjtNum Kq = 100;
    mjtNum Kqdot = 5;
    mjtNum dq = q_bar - d->qpos[0];
    mjtNum u = 0;
    if (dq > 0)
    {
        u = Kq * dq - Kqdot * d->qvel[0];
        //std::cout << u << std::endl;
        if (u > 0)
        {
            d->ctrl[1] = u;
        }
        else
        {
            d->ctrl[2] = -u;
        }
    }
    else
    {
        u = -Kq * dq + Kqdot * d->qvel[0];
        if (u > 0)
        {
            d->ctrl[2] = u;
        }
        else
        {
            d->ctrl[1] = -u;
        }
    }

    d->ctrl[0] = sin(10 * d->time);
}

void LengthController(const mjModel* m, mjData* d)
{
    mjtNum l_bar = 0.515;//0.41 to 0.79, center length: 0.62
    mjtNum Kp = 1;
    mjtNum Kd = 0.5;
    mjtNum dl = 0;

    dl = d->ten_length[0] - l_bar;
    //std::cout << d->ten_length[0] << std::endl;
    if (dl > 0)
    {
        d->ctrl[1] = Kp * dl - Kd * d->qvel[0];
    }
    else
    {
        d->ctrl[2] = -(Kp * dl - Kd * d->qvel[0]);
    }

    //d->ctrl[0] = sin(10*d->time);
    //std::cout << d->ctrl[0] << std::endl;
}
mjtNum ComputeController(mjtNum maxTheta, mjtNum length0, mjtNum lengthMin)
{
    mjtNum output;
    output = (lengthMin - length0) / -maxTheta;
    return output;
}
int main(void)
{ 
    visualization = 1;
    fs.open("../matlab/plot.csv", std::ios::out | std::ios::app);
    
    if (visualization == 1)
    {
        // load model from file and check for errors
        m = mj_loadXML("muscle_control_narrow.xml", NULL, error, 1000);
        //m = mj_loadXML("muscle_control.xml", NULL, error, 1000);
        //m = mj_loadXML("testbench.xml", NULL, error, 1000);
        if (!m)
        {
            printf("%s\n", error);
            return 1;
        }

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

        mj_forward(m, d);
        int mode = 2;
        if (mode == 0) {
            mjcb_control = AngleController;
        }
        else if (mode == 1)
        {
            mjcb_control = LengthController;
        }
        else if (mode == 2)
        {
            mCounter = 0;
            
            mjcb_control = EventLengthController;
            event_times[0] = 0;
            event_times[1] = 0;
            u = 1;
            l_bar[0] = 0.7;
            l_bar[1] = 0.4;
            
            c_theta_bar = 0;
            c_b = d->ten_length[0];
            c_a[1] = ComputeController(3.14, c_b, 0);
            c_a[0] = -c_a[1];
            
            //d->ctrl[0] = -1.41;
            //d->ctrl[0] = 0;
            //d->qpos[0] = 0.3;

            d->userdata[0] = -10;
            d->userdata[1] = 10;
            d->userdata[2] = 0;

            //actuatorSensor[0].Initialize(l_bar[0], m, d, 1);//right muscle
            //actuatorSensor[1].Initialize(l_bar[1], m, d, 2);//left muscle
            actuatorSensor[0].Initialize(m, d, l_bar[0], 1);
            actuatorSensor[1].Initialize(m, d, l_bar[1], 2);
        }
        else
        {
            mjcb_control = MyCallback;
            //mjcb_act_dyn = FilterDyn;
            dt_p_ticks = round(dt_p / m->opt.timestep);
            shift_ticks = dt_p_ticks / 2;
            for (int i = 0; i < 5; i++)
            {
                sensor0[i] = 0;
                sensor0_filtered[i] = 0;
            }
       
        }
        mj_forward(m, d);
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
                    mj_step(m, d);

                next_step = false;
                //if (d->time > 10) break;
                //std::cout << "actuator1 torque: " << d->actuator_force[1] * d->actuator_moment[1] << std::endl;
                //std::cout << "actuator2 torque: " << d->actuator_force[2] * d->actuator_moment[2] << std::endl;
                //std::cout << "dTheta: " << dTheta << std::endl;
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
        mj_deleteModel(m);
    }
    else if (visualization == 0)
    {
        mlock = true;
        int totalSimTicks = 80000;
        mjtNum act0_ctrl = -1.41;
        
        while (mlock)
        {
            //global varaible initialization
            event_times[0] = 0;
            event_times[1] = 0;
            mCounter = 0;
            l_bar[0] = 0;
            l_bar[1] = 0;
            u = 1;

            // load model from file and check for errors
            m = mj_loadXML("muscle_control.xml", NULL, error, 1000);
            if (!m)
            {
                printf("%s\n", error);
                return 1;
            }

            // make data corresponding to model
            d = mj_makeData(m);
            {
                mjcb_control = EventLengthController;
                if (act0_ctrl >= 1.0)
                {
                    mlock = false;
                }
                d->ctrl[0] = act0_ctrl;
                act0_ctrl += 0.01;
            }
            mj_forward(m, d);

            while (mCounter <= totalSimTicks)
            {
                mj_step(m, d);
                mjtNum torque0 = d->actuator_force[1] * d->actuator_moment[1];
                mjtNum torque1 = d->actuator_force[2] * d->actuator_moment[2];
                mjtNum netTorque = torque0 + torque1;
                mCounter++;
                if (mCounter == totalSimTicks)
                {
                    std::cout << d->actuator_force[1] << std::endl;
                    fs << d->qpos[0] << ", " << torque0 << ", " << torque1 << ", " << netTorque << "\n";
                }
            }
            mj_deleteData(d);
            mj_deleteModel(m);
            mjcb_control = 0;
        }
    }
    else
    {
        std::cout << "please select correct visualization mode to run!" << std::endl;
    }
    
    fs.close();
    return 0;
}