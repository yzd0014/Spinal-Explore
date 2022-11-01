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

//file
std::fstream fs;
int visualization;
bool mlock;
int mCounter;

mjtNum refractory_dt;
mjtNum event_time;
mjtNum u;
mjtNum l_bar;

mjtNum refractory_dts[2];
mjtNum event_times[2];
mjtNum l_bars[2];
mjtNum dls[2];
mjtNum c_Kp = 3;
mjtNum c_a[2];
mjtNum c_pos_bar = 0.25;
void EventLengthController(const mjModel* m, mjData* d)
{
    mjtNum dl = d->ten_length[0] - l_bar;
    refractory_dt = -0.031125 * dl + 0.05;
  
    d->ctrl[1] = 0;
    if (d->time - event_time >= refractory_dt)
    {
        event_time = d->time;
        d->ctrl[1] = u;
    }

    if (visualization == 0)
    {
        mCounter++;
        if (mCounter >= 10000)
        {
            fs << d->ten_length[0] << ", " << -d->actuator_force[1] << "\n";
            if (d->ten_length[0] >= 2.16) mlock = false;
        }
    }
    //std::cout << d->act[0] << std::endl;
    //std::cout << mCounter << ", " << - d->actuator_force[1] << std::endl;
    //std::cout << d->actuator_force[1] << " " << d->qfrc_actuator[0] << std::endl;
    //std::cout << d->xpos[5] << std::endl;
    //std::cout << d->ctrl[0] <<" "<< d->act[0] << std::endl;
}

void ModeTwoController(const mjModel* m, mjData* d)
{
    mjtNum lengthError = c_Kp * (c_pos_bar - d->xpos[3]);
    for (int i = 0; i < 2; i++)
    {
        //l_bars[i] = c_a[i] * lengthError + 1;
        dls[i] = d->ten_length[i] - l_bars[i];
        if (dls[i] < 0)
        {
            dls[i] = 0;
        }
        else if (dls[i] > 1)
        {
            dls[i] = 1;
        }
        refractory_dts[i] = -0.0498 * dls[i] + 0.05;

        d->ctrl[i + 1] = 0;
        if (d->time - event_times[i] >= refractory_dts[i])
        {
            event_times[i] = d->time;
            d->ctrl[i + 1] = u;
        }
    }
    //std::cout << dls[0] << ", " << dls[1] << "\n";
    //std::cout << d->act[0] << ", " << d->act[1] << "\n";
    //std::cout << d->actuator_force[1] << ", " << d->actuator_force[2] << "\n\n";
    //std::cout << d->xpos[3] << "\n";
    //std::cout << refractory_dts[0] << ", " << d->act[0] << "\n";
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

int main(void)
{
    visualization = 1;
    fs.open("plot_data.csv", std::ios::out | std::ios::app);

    if (visualization == 1)
    {
        //global varaible initialization
        event_time = 0;
        mCounter = 0;
        u = 0.2;
        l_bar = 0.6;

        // load model from file and check for errors
        //m = mj_loadXML("load_test.xml", NULL, error, 1000);
        int mode = 0;
        if (mode == 0)
        {
            m = mj_loadXML("load_damping.xml", NULL, error, 1000);
        }
        else if (mode == 1)
        {
            m = mj_loadXML("translation_muscle.xml", NULL, error, 1000);
        }
       
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
        if(mode == 0)
        {
            mjcb_control = EventLengthController;
        }
        else if (mode == 1)
        {
            u = 1;
            c_a[1] = 1;
            c_a[0] = -c_a[1];
            l_bars[0] = 0;
            l_bars[1] = 0.88;
            event_times[0] = 0;
            event_times[1] = 0;
            d->ctrl[0] = 0;
            mjcb_control = ModeTwoController;
        }
        
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
        mjtNum act0_ctrl = -0.02;
        while (mlock)
        {
            //global varaible initialization
            event_time = 0;
            mCounter = 0;
            u = 0.2;
            l_bar = 0.6;

            // load model from file and check for errors
            //m = mj_loadXML("load_test.xml", NULL, error, 1000);
            m = mj_loadXML("load_damping.xml", NULL, error, 1000);
            if (!m)
            {
                printf("%s\n", error);
                return 1;
            }

            // make data corresponding to model
            d = mj_makeData(m);
            {
                mjcb_control = EventLengthController;
                d->ctrl[0] = act0_ctrl;
                act0_ctrl -= 0.02;
            }
            mj_forward(m, d);

            while (mCounter < 10000)
            {
                mj_step(m, d);
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