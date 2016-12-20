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
cGenericHapticDevicePtr hapticDevice;

// a label to display the haptic device model
cLabel* labelHapticDeviceModel;

// a label to display the position [m] of the haptic device
cLabel* labelHapticDevicePosition;

// a global variable to store the position [m] of the haptic device
cVector3d hapticDevicePosition;

// a global variable to store the velocity [m/s] of the haptic device
cVector3d hapticDeviceVelocity;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelHapticRate;

// a small sphere (cursor) representing the haptic device 
cShapeSphere* cursor;

// a line representing the velocity of the haptic device
cShapeLine* velocity;

// a scope to monitor position values of haptic device
cScope* scope;

// a level widget to display velocity
cLevel* levelVelocity;

// three dials to display position information
cDial* dialPosX;
cDial* dialPosY;
cDial* dialPosZ;

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
    DEMO:   03-analytics.cpp

    This application illustrates how to program forces, torques and gripper
    forces to your haptic device.

    In this example the application opens an OpenGL window and displays a
    3D cursor for the device connected to your computer. Several widgets are
    used to demonstrate several possible approaches for to display signal data.
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
    cout << "Demo: 03-analytics" << endl;
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
    // OPEN GL - WINDOW DISPLAY
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
    camera->set(cVector3d(0.5, 0.0, 0.0),    // camera position (eye)
                cVector3d(0.0, 0.0, 0.0),    // look at position (target)
                cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.005);
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

    // create a sphere (cursor) to represent the haptic device
    cursor = new cShapeSphere(0.01);

    // insert cursor inside world
    world->addChild(cursor);

    // create small line to illustrate the velocity of the haptic device
    velocity = new cShapeLine(cVector3d(0,0,0), cVector3d(0,0,0));

    // insert line inside world
    world->addChild(velocity);


    //--------------------------------------------------------------------------
    // HAPTIC DEVICE
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get a handle to the first haptic device
    handler->getDevice(hapticDevice, 0);

    // open a connection with the haptic device
    hapticDevice->open();

    // retrieve information about the current haptic device
    cHapticDeviceInfo info = hapticDevice->getSpecifications();

    // if the haptic device provides orientation sensing (stylus), 
    // a reference frame is displayed
    if (info.m_sensedRotation == true)
    {
        // display a reference frame
        cursor->setShowFrame(true);

        // set the size of the reference frame
        cursor->setFrameSize(0.05);
    }

    // if the device has a gripper, enable the gripper to behave like a user switch
    hapticDevice->setEnableGripperUserSwitch(true);


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFont *font = NEW_CFONTCALIBRI20();

    // create a label to display the haptic device model
    labelHapticDeviceModel = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticDeviceModel);
    labelHapticDeviceModel->setText(info.m_modelName);

    // create a label to display the position of haptic device
    labelHapticDevicePosition = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticDevicePosition);

    // create a label to display the haptic rate of the simulation
    labelHapticRate = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticRate);


    ////////////////////////////////////////////////////////////////////////////
    // In the following lines we set up several widgets to display position
    // and velocity data coming from the haptic device. For each widget we
    // define a range of values to expect from the haptic device. In this
    // example the units are meters (as we are tracking a position signal!) and 
    // have a set a default range between -0.1 to 0.1 meters. If you are using 
    // devices with a small or larger workspace, you may want to adjust these 
    // values accordingly. The other settings will modify the visual appearance
    // of the widgets. Have fun playing with these values!
    ////////////////////////////////////////////////////////////////////////////

    // create a scope to plot haptic device position data
    scope = new cScope();
    camera->m_frontLayer->addChild(scope);
    scope->setLocalPos(100,60);
    scope->setRange(-0.1, 0.1);
    scope->setSignalEnabled(true, true, true, false);
    scope->setTransparencyLevel(0.7);

    // create a level to display velocity data
    levelVelocity = new cLevel();
    camera->m_frontLayer->addChild(levelVelocity);
    levelVelocity->setLocalPos(20, 60);
    levelVelocity->setRange(0.0, 1.0);
    levelVelocity->setSize(40);
    levelVelocity->setNumIncrements(46);
    levelVelocity->setSingleIncrementDisplay(false);
    levelVelocity->setTransparencyLevel(0.5);
    
    // three dials to display position data
    dialPosX = new cDial();
    camera->m_frontLayer->addChild(dialPosX);
    dialPosX->setLocalPos(750, 200);
    dialPosX->setRange(-0.1, 0.1);
    dialPosX->setSize(40);
    dialPosX->setSingleIncrementDisplay(true);

    dialPosY = new cDial();
    camera->m_frontLayer->addChild(dialPosY);
    dialPosY->setLocalPos(750, 140);
    dialPosY->setRange(-0.1, 0.1);
    dialPosY->setSize(40);
    dialPosY->setSingleIncrementDisplay(true);

    dialPosZ = new cDial();
    camera->m_frontLayer->addChild(dialPosZ);
    dialPosZ->setLocalPos(750, 80);
    dialPosZ->setRange(-0.1, 0.1);
    dialPosZ->setSize(40);
    dialPosZ->setSingleIncrementDisplay(true);

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
    // update the size of the viewport
    windowW = w;
    windowH = h;

    // update position of label
    labelHapticDevicePosition->setLocalPos(20, windowH - 60, 0);

    // update position of label
    labelHapticDeviceModel->setLocalPos(20, windowH - 40, 0);

    // update position of scope
    scope->setSize(windowW - 200, 180);

    // update position of dials
    dialPosX->setLocalPos(windowW - 50, 210);
    dialPosY->setLocalPos(windowW - 50, 150);
    dialPosZ->setLocalPos(windowW - 50, 90);
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
}

//------------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    hapticDevice->close();
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

    // display new position data
    labelHapticDevicePosition->setText( hapticDevicePosition.str(3));

    // display haptic rate data
    labelHapticRate->setText(cStr(frequencyCounter.getFrequency(), 0) + " Hz");

    // update position of label
    labelHapticRate->setLocalPos((int)(0.5 * (windowW - labelHapticRate->getWidth())), 15);

    // update information to scope
    scope->setSignalValues(hapticDevicePosition.x(), 
                           hapticDevicePosition.y(), 
                           hapticDevicePosition.z());

    // update information to dials
    dialPosX->setValue(hapticDevicePosition.x());
    dialPosY->setValue(hapticDevicePosition.y());
    dialPosZ->setValue(hapticDevicePosition.z());

    // update velocity information to level
    levelVelocity->setValue(hapticDeviceVelocity.length());


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
        /////////////////////////////////////////////////////////////////////
        // READ HAPTIC DEVICE
        /////////////////////////////////////////////////////////////////////

        // read position 
        cVector3d position;
        hapticDevice->getPosition(position);

        // read orientation 
        cMatrix3d rotation;
        hapticDevice->getRotation(rotation);

        // read gripper position
        double gripperAngle;
        hapticDevice->getGripperAngleRad(gripperAngle);

        // read linear velocity 
        cVector3d linearVelocity;
        hapticDevice->getLinearVelocity(linearVelocity);

        // read angular velocity
        cVector3d angularVelocity;
        hapticDevice->getAngularVelocity(angularVelocity);

        // read gripper angular velocity
        double gripperAngularVelocity;
        hapticDevice->getGripperAngularVelocity(gripperAngularVelocity);

        // read user-switch status (button 0)
        bool button0, button1, button2, button3;
        button0 = false;
        button1 = false;
        button2 = false;
        button3 = false;

        hapticDevice->getUserSwitch(0, button0);
        hapticDevice->getUserSwitch(1, button1);
        hapticDevice->getUserSwitch(2, button2);
        hapticDevice->getUserSwitch(3, button3);


        /////////////////////////////////////////////////////////////////////
        // UPDATE 3D CURSOR MODEL
        /////////////////////////////////////////////////////////////////////

        // update arrow
        velocity->m_pointA = position;
        velocity->m_pointB = cAdd(position, linearVelocity);

        // update position and orientation of cursor
        cursor->setLocalPos(position);
        cursor->setLocalRot(rotation);

        // adjust the  color of the cursor according to the status of
        // the user-switch (ON = TRUE / OFF = FALSE)
        if (button0)
        {
            cursor->m_material->setGreenMediumAquamarine(); 
        }
        else if (button1)
        {
            cursor->m_material->setYellowGold();
        }
        else if (button2)
        {
            cursor->m_material->setOrangeCoral();
        }
        else if (button3)
        {
            cursor->m_material->setPurpleLavender();
        }
        else
        {
            cursor->m_material->setBlueRoyal();
        }

        // update global variable for graphic display update
        hapticDevicePosition = position;
        hapticDeviceVelocity = linearVelocity;


        /////////////////////////////////////////////////////////////////////
        // COMPUTE AND APPLY FORCES
        /////////////////////////////////////////////////////////////////////

        cVector3d force (0,0,0);
        cVector3d torque (0,0,0);
        double gripperForce = 0.0;

        // apply force field
        if (useForceField)
        {
            // compute linear force
            double Kp = 25; // [N/m]
            cVector3d forceField = -Kp * position;
            force.add(forceField);

            // compute angular torque
            double Kr = 0.05; // [N/m.rad]
            cVector3d axis;
            double angle;
            rotation.toAxisAngle(axis, angle);
            torque = (-Kr * angle) * axis;
        }

        // apply damping term
        if (useDamping)
        {
            cHapticDeviceInfo info = hapticDevice->getSpecifications();

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
        hapticDevice->setForceAndTorqueAndGripperForce(force, torque, gripperForce);

        // update frequency counter
        frequencyCounter.signal(1);
    }

    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------
