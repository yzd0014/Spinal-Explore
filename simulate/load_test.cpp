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

bool start_sim = true;
bool next_step = false;
unsigned int key_s_counter = 0;

//file
std::fstream fs;

mjtNum refractory_dt;
mjtNum event_time;
bool mlock;
int mCounter;
mjtNum u;
mjtNum l_bar;
mjtNum event_times[2];

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
    mCounter++;
    if (mCounter >= 10000)
    {
        fs << d->ten_length[0] << ", " << -d->actuator_force[1] << "\n";
        if (d->ten_length[0] >= 2.16) mlock = false;
    }
    //std::cout << mCounter << ", " << - d->actuator_force[1] << std::endl;
    //std::cout << d->actuator_force[1] << " " << d->qfrc_actuator[0] << std::endl;
    //std::cout << d->xpos[5] << std::endl;
    //std::cout << d->ctrl[0] <<" "<< d->act[0] << std::endl;
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
    //file
    fs.open("plot_data_0.6.csv", std::ios::out | std::ios::app);
    mlock = true;
    mjtNum act0_ctrl = -0.02;

    while (mlock)
    {
        //mlock = false;
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

        {
            mjcb_control = EventLengthController;
            d->ctrl[0] = act0_ctrl;
            act0_ctrl -= 0.02;
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
            }
            if (mCounter >= 10000)
            {
                glfwSetWindowShouldClose(window, 1);
                mCounter = 0;
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
  
    fs.close();
    return 0;
}