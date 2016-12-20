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
    \version   3.1.1 $Rev: 1922 $
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
// DECLARED CONSTANTS
//------------------------------------------------------------------------------

// number of spheres in the scene
const int NUM_SPHERES = 16;

// radius of each sphere
const double SPHERE_RADIUS = 0.007;


//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cSpotLight *light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// highest stiffness the current haptic device can render
double hapticDeviceMaxStiffness;

// sphere objects
cShapeSphere* spheres[NUM_SPHERES];

// linear velocity of each sphere
cVector3d sphereVel[NUM_SPHERES];

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelHapticRate;

// a label to explain what is happening
cLabel* labelMessage;

// indicates if the haptic simulation currently running
bool simulationRunning = false;

// indicates if the haptic simulation has terminated
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

// root resource path
string resourceRoot;


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


//------------------------------------------------------------------------------
// DECLARED MACROS
//------------------------------------------------------------------------------
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())


//==============================================================================
/*
    DEMO:   09-magnets.cpp

    This example illustrates how to create a simple dynamic simulation using
    small sphere shape primitives. All dynamics and collisions are computed
    in the haptics thread.
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
    cout << "Demo: 09-magnets" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
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
    world->m_backgroundColor.setWhite();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    camera->set(cVector3d(0.20, 0.00, 0.10),    // camera position (eye)
                cVector3d(0.00, 0.00, 0.05),    // lookat position (target)
                cVector3d(0.00, 0.00, 1.00));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    // anything in front or behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.03);
    camera->setStereoFocalLength(1.8);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a light source
    light = new cSpotLight(world);

    // attach light to camera
    world->addChild(light);    

    // enable light source
    light->setEnabled(true);                   

    // position the light source
    light->setLocalPos(0.0, 0.3, 0.4);             

    // define the direction of the light beam
    light->setDir(0.0,-0.25, -0.4);             

    // enable this light source to generate shadows
    light->setShadowMapEnabled(true);

    // set the resolution of the shadow map
    //light->m_shadowMap->setQualityLow();
    light->m_shadowMap->setQualityMedium();

    // set shadow factor
    world->setShadowIntensity(0.3);

    // set light cone half angle
    light->setCutOffAngleDeg(30);


    //--------------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    // retrieve the highest stiffness this device can render
    hapticDeviceMaxStiffness = hapticDeviceInfo.m_maxLinearStiffness;

    // if the haptic devices carries a gripper, enable it to behave like a user switch
    hapticDevice->setEnableGripperUserSwitch(true);


    //--------------------------------------------------------------------------
    // CREATE PLANE
    //--------------------------------------------------------------------------

    // create mesh
    cMesh* plane = new cMesh();

    // add mesh to world
    world->addChild(plane);

    // create plane primitive
    cCreateMap(plane, 0.2, 1.0, 20, 20);

    // compile object
    plane->setUseDisplayList(true);

    // set color properties
    plane->m_material->setWhite();


    //--------------------------------------------------------------------------
    // CREATE SPHERES
    //--------------------------------------------------------------------------

    // create texture
    cTexture2dPtr texture = cTexture2d::create();

    // load texture file
    bool fileload = texture->loadFromFile(RESOURCE_PATH("../resources/images/spheremap-3.jpg"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = texture->loadFromFile("../../../bin/resources/images/spheremap-3.jpg");
        #endif
    }
    if (!fileload)
    {
        cout << "Error - Texture image failed to load correctly." << endl;
        close();
        return (-1);
    }

    // create spheres
    for (int i=0; i<NUM_SPHERES; i++)
    {
        // create a sphere and define its radius
        cShapeSphere* sphere = new cShapeSphere(SPHERE_RADIUS);

        // store pointer to sphere primitive
        spheres[i] = sphere;

        // add sphere primitive to world
        world->addChild(sphere);

        // set the position of the object at the center of the world
        sphere->setLocalPos(0.8 * SPHERE_RADIUS * (double)(i+2) * cos(1.0 * (double)(i)), 
                            0.8 * SPHERE_RADIUS * (double)(i+2) * sin(1.0 * (double)(i)), 
                            SPHERE_RADIUS);

        // set graphic properties of sphere
        sphere->setTexture(texture);
        sphere->m_texture->setSphericalMappingEnabled(true);
        sphere->setUseTexture(true);
        sphere->m_material->setWhite();
    }


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFont *font = NEW_CFONTCALIBRI20();
    
    // create a label to display the haptic rate of the simulation
    labelHapticRate = new cLabel(font);
    labelHapticRate->m_fontColor.setBlack();
    camera->m_frontLayer->addChild(labelHapticRate);

    // create a label with a small message
    labelMessage = new cLabel(font);
    camera->m_frontLayer->addChild(labelMessage);

    // set font color
    labelMessage->m_fontColor.setBlack();

    // set text message
    labelMessage->setText("interact with magnetic spheres - press user switch to disable magnetic effect");

    // create a background
    cBackground* background = new cBackground();
    camera->m_backLayer->addChild(background);

    // set aspect ration of background image a constant
    background->setFixedAspectRatio(true);

    // load background image
    fileload = background->loadFromFile(RESOURCE_PATH("../resources/images/background.png"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = background->loadFromFile("../../../bin/resources/images/background.png");
        #endif
    }
    if (!fileload)
    {
        cout << "Error - Image failed to load correctly." << endl;
        close();
        return (-1);
    }


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
        // exit application
        exit(0);
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

    // update haptic rate data
    labelHapticRate->setText(cStr(frequencyCounter.getFrequency(), 0) + " Hz");

    // update position of label
    labelHapticRate->setLocalPos((int)(0.5 * (windowW - labelHapticRate->getWidth())), 15);

    // update position of message label
    labelMessage->setLocalPos((int)(0.5 * (windowW - labelMessage->getWidth())), 50);


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
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}

//------------------------------------------------------------------------------

void updateHaptics(void)
{
    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    // flag to indicate if haptic forces are active
    bool flagHapticsEnabled = false;

    // reset clock
    cPrecisionClock clock;
    clock.reset();

    // open a connection to haptic device
    hapticDevice->open();

    // calibrate device (if necessary)
    hapticDevice->calibrate();

    // main haptic simulation loop
    while(simulationRunning)
    {
        /////////////////////////////////////////////////////////////////////
        // SIMULATION TIME
        /////////////////////////////////////////////////////////////////////

        // stop the simulation clock
        clock.stop();

        // read the time increment in seconds
        double timeInterval = cMin(0.001, clock.getCurrentTimeSeconds());

        // restart the simulation clock
        clock.reset();
        clock.start();

        // update frequency counter
        frequencyCounter.signal(1);


        /////////////////////////////////////////////////////////////////////////
        // READ HAPTIC DEVICE
        /////////////////////////////////////////////////////////////////////////

        // read position 
        cVector3d position;
        hapticDevice->getPosition(position);

        // read user-switch status (button 0)
        bool button;
        hapticDevice->getUserSwitch(0, button);


        /////////////////////////////////////////////////////////////////////////
        // UPDATE SIMULATION
        /////////////////////////////////////////////////////////////////////////

        // position of walls and ground
        const double WALL_GROUND        = 0.0 + SPHERE_RADIUS;
        const double WALL_LEFT          =-0.1;
        const double WALL_RIGHT         = 0.2;
        const double WALL_FRONT         = 0.08;
        const double WALL_BACK          =-0.08;
        const double SPHERE_STIFFNESS   = 1000.0;
        const double SPHERE_MASS        = 0.04;
        const double K_DAMPING          = 0.996;
        const double K_MAGNET           = 500.0;
        const double HAPTIC_STIFFNESS   = 500.0;

        // clear forces for all spheres
        cVector3d sphereFce[NUM_SPHERES];

        for (int i=0; i<NUM_SPHERES; i++)
        {
            sphereFce[i].zero();
        }

        // compute forces for all spheres
        for (int i=0; i<NUM_SPHERES; i++)
        {
            cVector3d force;
            cVector3d pos0 = spheres[i]->getLocalPos();

            // check forces with all other spheres
            for (int j=i+1; j<NUM_SPHERES; j++)
            {
                // init force
                force.zero();
                
                // get position of sphere
                cVector3d pos1 = spheres[j]->getLocalPos();

                // compute direction vector from sphere 0 to 1
                cVector3d dir01 = cNormalize(pos1 - pos0);

                // compute distance between both spheres
                double distance = cDistance(pos0, pos1);

                // compute magnetic force
                if (!button)
                {
                    if ((distance < 2.5 * SPHERE_RADIUS) && (distance > 2.0 * SPHERE_RADIUS))
                    {
                        force.add(-K_MAGNET * (distance - 2.5 * SPHERE_RADIUS) * dir01);
                    }
                }

                // compute contact force
                if (distance < 2.0 * SPHERE_RADIUS)
                {
                    force.add(SPHERE_STIFFNESS * (distance - 2.0 * SPHERE_RADIUS) * dir01);
                }

                // add force to each sphere
                sphereFce[i].add(force);
                sphereFce[j].add(-force);
            }

            // check forces with ground and walls
            if (pos0.z() < WALL_GROUND)
            {
                sphereFce[i].add(cVector3d(0.0, 0.0, SPHERE_STIFFNESS * (WALL_GROUND - pos0.z())));
            }
            if (pos0.y() < WALL_LEFT)
            {
                sphereFce[i].add(cVector3d(0.0, SPHERE_STIFFNESS * (WALL_LEFT - pos0.y()), 0.0));
            }
            if (pos0.y() > WALL_RIGHT)
            {
                sphereFce[i].add(cVector3d(0.0, SPHERE_STIFFNESS * (WALL_RIGHT - pos0.y()), 0.0));
            }
            if (pos0.x() < WALL_BACK)
            {
                sphereFce[i].add(cVector3d(SPHERE_STIFFNESS * (WALL_BACK - pos0.x()), 0.0, 0.0));
            }
            if (pos0.x() > WALL_FRONT)
            {
                sphereFce[i].add(cVector3d(SPHERE_STIFFNESS * (WALL_FRONT - pos0.x()), 0.0, 0.0));
            }
        }

        // compute haptic force
        cVector3d dirHapticSphere = spheres[0]->getLocalPos() - position;
        cVector3d forceSphere =-SPHERE_STIFFNESS * dirHapticSphere;
        cVector3d forceHaptic = HAPTIC_STIFFNESS * dirHapticSphere;
        sphereFce[0].add(forceSphere);

        // update velocity and position of all spheres
        for (int i=0; i<NUM_SPHERES; i++)
        {
            // compute acceleration
            cVector3d sphereAcc = (sphereFce[i] / SPHERE_MASS) + cVector3d(0.0, 0.0, -9.81);

            // compute velocity
            sphereVel[i] = K_DAMPING * (sphereVel[i] + timeInterval * sphereAcc);

            // compute position
            cVector3d spherePos = spheres[i]->getLocalPos() + timeInterval * sphereVel[i] + cSqr(timeInterval) * sphereAcc;

            // update value to sphere object
            spheres[i]->setLocalPos(spherePos);
        }


        /////////////////////////////////////////////////////////////////////////
        // APPLY FORCES
        /////////////////////////////////////////////////////////////////////////

        // haptic forces are only enabled if a small value is first sent to the device
        if (!flagHapticsEnabled)
        {
            // check for small force
            if (forceHaptic.length() < 1.0)
            {
                flagHapticsEnabled = true;
            }
            else
            {
                forceHaptic.zero();
            }
        }

        // scale the force according to the max stiffness the device can render
        double stiffnessRatio = 1.0;
        if (hapticDeviceMaxStiffness < HAPTIC_STIFFNESS)
            stiffnessRatio = hapticDeviceMaxStiffness / HAPTIC_STIFFNESS;

        // send computed force to haptic device
        hapticDevice->setForce(stiffnessRatio * forceHaptic);
    }

    // close  connection to haptic device
    hapticDevice->close();

    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------
