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
    \version   3.1.1 $Rev: 1907 $
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

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelHapticRate;

// a label to describe application
cLabel* labelDescription;

// number of bitmap images
const int NUM_IMAGES = 16;

// table of bitmap images
cBitmap* bitmaps[NUM_IMAGES];

// scroll position of text
double bitmapPos = 0.0;

// indicates if the haptic simulation currently running
bool simulationRunning = false;

// indicates if the haptic simulation has terminated
bool simulationFinished = true;

// frequency counter to measure the simulation haptic rate
cFrequencyCounter frequencyCounter;

// mouse position 
int mouseX, mouseY;

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
// DECLARED MACROS
//------------------------------------------------------------------------------

// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())


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

// callback to handle mouse click
void mouseClick(int button, int state, int x, int y);

// callback to handle mouse motion
void mouseMove(int x, int y);


//==============================================================================
/*
    DEMO:   06-images.cpp

    This example illustrates the use of bitmap images.

    In the main haptics loop function  "updateHaptics()" , the position
    of the haptic device is retrieved at each simulation iteration.
    The interaction forces are then computed and sent to the haptic device.

    Use the haptic or mouse to scroll the images.

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
    cout << "Demo: 06-images" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[x] - Exit application" << endl;
    cout << endl << endl;

    // parse first arg to try and locate resources
    resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);


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
    glutMouseFunc(mouseClick);
    glutMotionFunc(mouseMove);
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
    camera->set(cVector3d(3.0, 0.0, 0.0),    // camera position (eye)
                cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
                cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.02);
    camera->setStereoFocalLength(3.0);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // enable multi-pass rendering to handle transparent objects
    camera->setUseMultipassTransparency(true);

    // create a directional light source
    light = new cDirectionalLight(world);

    // insert light source inside world
    world->addChild(light);

    // enable light source
    light->setEnabled(true);                   

    // define direction of light beam
    light->setDir(-1.0, 0.0, 0.0);          


    //--------------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device found
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    // create a tool (cursor) and insert into the world
    tool = new cToolCursor(world);
    world->addChild(tool);

    // connect the haptic device to the virtual tool
    tool->setHapticDevice(hapticDevice);

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(1.0);

    // define a radius for the virtual tool (sphere)
    tool->setRadius(0.03);

    // enabled forces immediately
    tool->setWaitForSmallForce(false);

    // hide tool
    tool->setShowEnabled(false);

    // start the haptic tool
    tool->start();


    //--------------------------------------------------------------------------
    // CREATING WORLD
    //--------------------------------------------------------------------------

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // haptic device properties
    double maxForce     = hapticDeviceInfo.m_maxLinearForce;
    double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;
    double maxDamping   = hapticDeviceInfo.m_maxLinearDamping / workspaceScaleFactor;

    // create some viscous environment
    cEffectViscosity* viscosity = new cEffectViscosity(world);
    world->addEffect(viscosity);
    world->m_material->setViscosity(0.5 * maxDamping);

    // create a an invisible object at the center of the world that will attract the tool
    cGenericObject* blackHole = new cGenericObject();
    world->addChild(blackHole);

    // create a magnetic field that attract the tool towards the center of the object
    cEffectMagnet* potentialField = new cEffectMagnet(blackHole);

    // enable the magnetic field effect to occur inside the object. Since a cGenericObject does
    // not own any geometrical description, our tool is always considered to be inside the object
    potentialField->setEnabledInside(true);
    blackHole->addEffect(potentialField);
    blackHole->m_material->setStiffness(maxStiffness);
    blackHole->m_material->setMagnetMaxForce(0.1 * maxForce);
    blackHole->m_material->setMagnetMaxDistance(10.0);
    blackHole->setLocalPos(0.0, 0.0, 0.0);
   

    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFont *font = NEW_CFONTCALIBRI20();
    
    // create a label to display the haptic rate of the simulation
    labelHapticRate = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticRate);
    labelHapticRate->m_fontColor.setGrayLevel(0.5);

    // create a label that describes how to operate the application
    labelDescription = new cLabel(font);
    camera->m_frontLayer->addChild(labelDescription);
    labelDescription->setText("use haptic device or mouse to scroll images");
    labelDescription->m_fontColor.setBlack();

    // create and load 16 bitmap images
    for(int i=0; i<NUM_IMAGES; i++)
    {
        // create new bitmap object
        cBitmap* bitmap = new cBitmap();

        // add bitmap to front layer
        camera->m_frontLayer->addChild(bitmap);

        // add bitmap to array
        bitmaps[i] = bitmap;

        // create file name
        int index = i+1;
        string filename = "polaroid-" + cStr(index) + ".png";

        // load image file
        bool fileload = bitmap->loadFromFile(RESOURCE_PATH("../resources/images/"+filename));
        if (!fileload)
        {
            #if defined(_MSVC)
            fileload = bitmap->loadFromFile("../../../bin/resources/images/"+filename);
            #endif
        }
        if (!fileload)
        {
            cout << "Error - Image failed to load correctly." << endl;
            close();
            return (-1);
        }
    }

    // select image whch will be rendered at the center of screen
    bitmapPos = (int)(0.5 * NUM_IMAGES);

    // create a CHAI3D logo at the bottom corner of the window
    camera->m_frontLayer->addChild(NEW_CHAI3D_BLACK_LOGO());

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

void mouseClick(int button, int state, int x, int y)
{
    // store mouse values
    mouseX = x;
    mouseY = y;
}

//------------------------------------------------------------------------------

void mouseMove(int x, int y)
{
    // compute mouse offset
    int dx = x - mouseX;

    // use mouse data to move images
    double pos = bitmapPos - 0.002 * (double)dx;
    bitmapPos = cClamp(pos, 0.0, (double)(NUM_IMAGES-1));

    // store mouse values
    mouseX = x;
    mouseY = y;
}

//------------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    tool->stop();
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

    // update haptic rate label
    labelHapticRate->setText(cStr(frequencyCounter.getFrequency(), 0) + " Hz");

    // update position of labels
    labelHapticRate->setLocalPos((int)(0.5 * (windowW - labelHapticRate->getWidth())), 15);
    labelDescription->setLocalPos((int)(0.5 * (windowW - labelDescription->getWidth())), 40);


    /////////////////////////////////////////////////////////////////////
    // UPDATE BITMAP IMAGES
    /////////////////////////////////////////////////////////////////////

    // compute the maximum image size ratio
    double ratioMax = 0.0;
    for (int i=0; i<NUM_IMAGES; i++)
    {
        // get width and height of bitmap image
        double w = bitmaps[i]->m_texture->m_image->getWidth();
        double h = bitmaps[i]->m_texture->m_image->getHeight();

        // compute ratio between width and height
        double ratio = w/h;

        //compute maximum ratio
        ratioMax = cMax(ratioMax, ratio);
    }

    // set base image height
    double base_h = 0.5 * windowH; 
    double base_w = ratioMax * base_h; 

    // set distance between images
    double distance = 1.3 * base_w;

    // set position and size of each image
    for (int i=0; i<NUM_IMAGES; i++)
    {
        // get width and height of bitmap image
        double w = bitmaps[i]->m_texture->m_image->getWidth();
        double h = bitmaps[i]->m_texture->m_image->getHeight();

        // compute ratio between width and height
        double ratio = w/h;

        // compute relative position 
        double pos = (double)(i) - bitmapPos;

        // compute size
        double scale = 1.0;
        if (fabs(pos) <= 1.0)
        {
            scale = 1.0 + 0.1 * (1.0 + cos(C_PI * pos));   
        }
        bitmaps[i]->setSize(scale * ratio * base_h, scale * base_h);

        // compute position
        double posX = pos * distance + 0.5 * windowW - 0.5 * scale * ratio * base_h;
        double posY = 0.5 * (windowH - scale * base_h);

        bitmaps[i]->setLocalPos(posX, posY);
    }
    

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
    if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}

//------------------------------------------------------------------------------

void updateHaptics(void)
{
    // initialize frequency counter
    frequencyCounter.reset();

    // initialize precision clock
    cPrecisionClock clock;
    clock.reset();

    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    // main haptic simulation loop
    while(simulationRunning)
    {
        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // update position and orientation of tool
        tool->updateFromDevice();

        // compute interaction forces
        tool->computeInteractionForces();

        // send forces to haptic device
        tool->applyToDevice();

        // stop clock
        double time = clock.stop();

        // scroll text according to the haptic device position
        const double K = 1;
        double command = tool->m_hapticPoint->getGlobalPosGoal().y();
        double pos = bitmapPos - K * time * command;
        bitmapPos = cClamp(pos, 0.0, (double)(NUM_IMAGES-1));

        // restart clock
        clock.start(true);

        // update frequency counter
        frequencyCounter.signal(1);
    }
    
    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------
