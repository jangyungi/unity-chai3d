//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   3.1.1 $Rev: 1869 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "chai3d.h"
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------
#ifndef MACOSX
#include "GL/glut.h"
#else
#include "GLUT/glut.h"
#endif
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled 
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;

// maximum number of devices supported by this application
const int MAX_DEVICES = 16;


//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice[MAX_DEVICES];

// number of haptic devices detected
int numHapticDevices = 0;

// labels to display each haptic device model
cLabel* labelHapticDeviceModel[MAX_DEVICES];

// labels to display the position [m] of each haptic device
cLabel* labelHapticDevicePosition[MAX_DEVICES];

// global variable to store the position [m] of each haptic device
cVector3d hapticDevicePosition[MAX_DEVICES];

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelHapticRate;

// small spheres (cursor) representing position of each haptic device
cShapeSphere* cursor[MAX_DEVICES];

// lines representing the velocity vector of each haptic device
cShapeLine* velocity[MAX_DEVICES];

// flag for using damping (ON/OFF)
bool useDamping = false;

// flag for using force field (ON/OFF)
bool useForceField = true;

// flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

// frequency counter to measure the simulation haptic rate
cFrequencyCounter frequencyCounter;

// information about computer screen and GLUT display window
int screenW;
int screenH;
int windowW;
int windowH;
int windowPosX;
int windowPosY;


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void resizeWindow(int w, int h);

// callback when a key is pressed
void keySelect(unsigned char key, int x, int y);

// callback to render graphic scene
void updateGraphics(void);

// callback of GLUT timer
void graphicsTimer(int data);

// function that closes the application
void close(void);

// main haptics simulation loop
void updateHaptics(void);


//==============================================================================
/*
    DEMO:   02-multi-devices.cpp

    This application illustrates how to program forces, torques and gripper
    forces on multiple haptic device.

    In this example the application opens an OpenGL window and displays a
    3D cursor for each device connected to your computer. If the user presses 
    onto the user button (if available on your haptic device), the color of 
    the cursor changes from blue to green.

    This example is very similar to 01-devices, but extends support for multiple
    haptic devices
*/
//==============================================================================

int main(int argc, char* argv[])
{
    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "Demo: 02-multi-devices" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[1] - Enable/Disable potential field" << endl;
    cout << "[2] - Enable/Disable damping" << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[x] - Exit application" << endl;
    cout << endl << endl;


    //--------------------------------------------------------------------------
    // OPENGL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLUT
    glutInit(&argc, argv);

    // retrieve  resolution of computer display and position window accordingly
    screenW = glutGet(GLUT_SCREEN_WIDTH);
    screenH = glutGet(GLUT_SCREEN_HEIGHT);
    windowW = 0.8 * screenH;
    windowH = 0.5 * screenH;
    windowPosY = (screenH - windowH) / 2;
    windowPosX = windowPosY; 

    // initialize the OpenGL GLUT window
    glutInitWindowPosition(windowPosX, windowPosY);
    glutInitWindowSize(windowW, windowH);

    if (stereoMode == C_STEREO_ACTIVE)
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE | GLUT_STEREO);
    else
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);


    // create display context and initialize GLEW library
    glutCreateWindow(argv[0]);

#ifdef GLEW_VERSION
    // initialize GLEW
    glewInit();
#endif

    // setup GLUT options
    glutDisplayFunc(updateGraphics);
    glutKeyboardFunc(keySelect);
    glutReshapeFunc(resizeWindow);
    glutSetWindowTitle("CHAI3D");

    // set fullscreen mode
    if (fullscreen)
    {
        glutFullScreen();
    }


    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setBlack();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    camera->set( cVector3d(0.5, 0.0, 0.0),    // camera position (eye)
                 cVector3d(0.0, 0.0, 0.0),    // look at position (target)
                 cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.01);
    camera->setStereoFocalLength(0.5);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a directional light source
    light = new cDirectionalLight(world);

    // insert light source inside world
    world->addChild(light);

    // enable light source
    light->setEnabled(true);                   

    // define direction of light beam
    light->setDir(-1.0, 0.0, 0.0); 

    // create a font
    cFont *font = NEW_CFONTCALIBRI20();


    //--------------------------------------------------------------------------
    // HAPTIC DEVICES
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();
    
    // get number of haptic devices
    numHapticDevices = handler->getNumDevices();

    // setup each haptic device
    for (int i=0; i<numHapticDevices; i++)
    {
        // get a handle to the first haptic device
        handler->getDevice(hapticDevice[i], i);

        // open a connection to haptic device
        hapticDevice[i]->open();

        // calibrate device (if necessary)
        hapticDevice[i]->calibrate();

        // retrieve information about the current haptic device
        cHapticDeviceInfo info = hapticDevice[i]->getSpecifications();

        // create a sphere (cursor) to represent the haptic device
        cursor[i] = new cShapeSphere(0.01);

        // insert cursor inside world
        world->addChild(cursor[i]);

        // create small line to illustrate the velocity of the haptic device
        velocity[i] = new cShapeLine(cVector3d(0,0,0), cVector3d(0,0,0));

        // insert line inside world
        world->addChild(velocity[i]);


        // display a reference frame if haptic device supports orientations
        if (info.m_sensedRotation == true)
        {
            // display reference frame
            cursor[i]->setShowFrame(true);

            // set the size of the reference frame
            cursor[i]->setFrameSize(0.05);
        }

        // if the device has a gripper, enable the gripper to simulate a user switch
        hapticDevice[i]->setEnableGripperUserSwitch(true);

        // create a label to display the haptic device model
        labelHapticDeviceModel[i] = new cLabel(font);
        camera->m_frontLayer->addChild(labelHapticDeviceModel[i]);
        labelHapticDeviceModel[i]->setText(info.m_modelName);

        // create a label to display the position of haptic device
        labelHapticDevicePosition[i] = new cLabel(font);
        camera->m_frontLayer->addChild(labelHapticDevicePosition[i]);
    }

    // create a label to display the haptic rate of the simulation
    labelHapticRate = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticRate);


    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    cThread* hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);

    // start the main graphics rendering loop
    glutTimerFunc(50, graphicsTimer, 0);
    glutMainLoop();

    // exit
    return (0);
}

//------------------------------------------------------------------------------

void resizeWindow(int w, int h)
{
    windowW = w;
    windowH = h;
}

//------------------------------------------------------------------------------

void keySelect(unsigned char key, int x, int y)
{
    // option ESC: exit
    if ((key == 27) || (key == 'x'))
    {
        exit(0);
    }

    // option 1: enable/disable force field
    if (key == '1')
    {
        useForceField = !useForceField;
        if (useForceField)
            cout << "> Enable force field     \r";
        else
            cout << "> Disable force field    \r";
    }

    // option 2: enable/disable damping
    if (key == '2')
    {
        useDamping = !useDamping;
        if (useDamping)
            cout << "> Enable damping         \r";
        else
            cout << "> Disable damping        \r";
    }

    // option f: toggle fullscreen
    if (key == 'f')
    {
        if (fullscreen)
        {
            windowPosX = glutGet(GLUT_INIT_WINDOW_X);
            windowPosY = glutGet(GLUT_INIT_WINDOW_Y);
            windowW = glutGet(GLUT_INIT_WINDOW_WIDTH);
            windowH = glutGet(GLUT_INIT_WINDOW_HEIGHT);
            glutPositionWindow(windowPosX, windowPosY);
            glutReshapeWindow(windowW, windowH);
            fullscreen = false;
        }
        else
        {
            glutFullScreen();
            fullscreen = true;
        }
    }

    // option m: toggle vertical mirroring
    if (key == 'm')
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
    }
}

//------------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    for (int i=0; i<numHapticDevices; i++)
    {
        hapticDevice[i]->close();
    }
}

//------------------------------------------------------------------------------

void graphicsTimer(int data)
{
    if (simulationRunning)
    {
        glutPostRedisplay();
    }

    glutTimerFunc(50, graphicsTimer, 0);
}

//------------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    int step = 40;
    for (int i=0; i<numHapticDevices; i++)
    {
        // update position of label
        labelHapticDeviceModel[i]->setLocalPos(20, windowH - step, 0);
        step += 20;

        // display new position data
        labelHapticDevicePosition[i]->setText(hapticDevicePosition[i].str(3));

        // update position of label
        labelHapticDevicePosition[i]->setLocalPos(20, windowH - step, 0);
        step += 25;
    }

    // display haptic rate data
    if (numHapticDevices == 0)
    {
        labelHapticRate->setText("no haptic device detected");
    }
    else
    {
        labelHapticRate->setText(cStr(frequencyCounter.getFrequency(), 0) + " Hz");
    }

    // update position of label
    labelHapticRate->setLocalPos((int)(0.5 * (windowW - labelHapticRate->getWidth())), 15);


    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, mirroredDisplay);

    // render world
    camera->renderView(windowW, windowH);

    // swap buffers
    glutSwapBuffers();

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error:  %s\n" << gluErrorString(err);
}

//------------------------------------------------------------------------------

void updateHaptics(void)
{
    // initialize frequency counter
    frequencyCounter.reset();

    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    // main haptic simulation loop
    while(simulationRunning)
    {
        for (int i=0; i<numHapticDevices; i++)
        {
            /////////////////////////////////////////////////////////////////////
            // READ HAPTIC DEVICE
            /////////////////////////////////////////////////////////////////////

            // read position 
            cVector3d position;
            hapticDevice[i]->getPosition(position);
        
            // read orientation 
            cMatrix3d rotation;
            hapticDevice[i]->getRotation(rotation);

            // read gripper position
            double gripperAngle;
            hapticDevice[i]->getGripperAngleRad(gripperAngle);

            // read linear velocity 
            cVector3d linearVelocity;
            hapticDevice[i]->getLinearVelocity(linearVelocity);

            // read angular velocity
            cVector3d angularVelocity;
            hapticDevice[i]->getAngularVelocity(angularVelocity);

            // read gripper angular velocity
            double gripperAngularVelocity;
            hapticDevice[i]->getGripperAngularVelocity(gripperAngularVelocity);

            // read user-switch status (button 0)
            bool button0, button1, button2, button3;
            button0 = false;
            button1 = false;
            button2 = false;
            button3 = false;

            hapticDevice[i]->getUserSwitch(0, button0);
            hapticDevice[i]->getUserSwitch(1, button1);
            hapticDevice[i]->getUserSwitch(2, button2);
            hapticDevice[i]->getUserSwitch(3, button3);


            /////////////////////////////////////////////////////////////////////
            // UPDATE 3D CURSOR MODEL
            /////////////////////////////////////////////////////////////////////
       
            // update arrow
            velocity[i]->m_pointA = position;
            velocity[i]->m_pointB = cAdd(position, linearVelocity);

            // update position and orientation of cursor
            cursor[i]->setLocalPos(position);
            cursor[i]->setLocalRot(rotation);

            // adjust the  color of the cursor according to the status of
            // the user-switch (ON = TRUE / OFF = FALSE)
            if (button0)
            {
                cursor[i]->m_material->setGreenMediumAquamarine(); 
            }
            else if (button1)
            {
                cursor[i]->m_material->setYellowGold();
            }
            else if (button2)
            {
                cursor[i]->m_material->setOrangeCoral();
            }
            else if (button3)
            {
                cursor[i]->m_material->setPurpleLavender();
            }
            else
            {
                cursor[i]->m_material->setBlueRoyal();
            }

            // update global variable for graphic display update
            hapticDevicePosition[i] = position;


            /////////////////////////////////////////////////////////////////////
            // COMPUTE AND APPLY FORCES
            /////////////////////////////////////////////////////////////////////
            
            // desired position
            cVector3d desiredPosition;
            desiredPosition.set(0.0, 0.0, 0.0);

            // desired orientation
            cMatrix3d desiredRotation;
            desiredRotation.identity();

            // variables for forces    
            cVector3d force (0,0,0);
            cVector3d torque (0,0,0);
            double gripperForce = 0.0;

            // apply force field
            if (useForceField)
            {
                // compute linear force
                double Kp = 25; // [N/m]
                cVector3d forceField = Kp * (desiredPosition - position);
                force.add(forceField);

                // compute angular torque
                double Kr = 0.05; // [N/m.rad]
                cVector3d axis;
                double angle;
                cMatrix3d deltaRotation = cTranspose(rotation) * desiredRotation;
                deltaRotation.toAxisAngle(axis, angle);
                torque = rotation * ((Kr * angle) * axis);
            }
    
            // apply damping term
            if (useDamping)
            {
                cHapticDeviceInfo info = hapticDevice[i]->getSpecifications();

                // compute linear damping force
                double Kv = 1.0 * info.m_maxLinearDamping;
                cVector3d forceDamping = -Kv * linearVelocity;
                force.add(forceDamping);

                // compute angular damping force
                double Kvr = 1.0 * info.m_maxAngularDamping;
                cVector3d torqueDamping = -Kvr * angularVelocity;
                torque.add(torqueDamping);

                // compute gripper angular damping force
                double Kvg = 1.0 * info.m_maxGripperAngularDamping;
                gripperForce = gripperForce - Kvg * gripperAngularVelocity;
            }

            // send computed force, torque, and gripper force to haptic device
            hapticDevice[i]->setForceAndTorqueAndGripperForce(force, torque, gripperForce);
        }

        // update frequency counter
        frequencyCounter.signal(1);
    }
    
    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------
