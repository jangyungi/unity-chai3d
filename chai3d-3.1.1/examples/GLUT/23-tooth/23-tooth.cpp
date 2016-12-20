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
    \version   3.1.1 $Rev: 1925 $
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

// virtual drill mesh
cMultiMesh* drill;

// a virtual tooth mesh
cMultiMesh* tooth;

// transparency level
double transparencyLevel = 0.3;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelHapticRate;

// indicates if the haptic simulation currently running
bool simulationRunning = false;

// indicates if the haptic simulation has terminated
bool simulationFinished = true;

// frequency counter to measure the simulation haptic rate
cFrequencyCounter frequencyCounter;

// side framebuffer
cFrameBufferPtr frameBuffer;

// side Panel that displays content of framebuffer
cViewPanel* viewPanel;

// information about computer screen and GLUT display window
int screenW;
int screenH;
int windowW;
int windowH;
int windowPosX;
int windowPosY;


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


//==============================================================================
/*
    DEMO:    tooth.cpp

    This demonstration shows how to attach a 3D mesh file to a virtual
    tool. A second mesh (tooth) is loaded in the scene. By pressing the user
    switch of the haptic device it is possible to translate or orient
    the tooth object accordingly.

    In the main haptics loop function  "updateHaptics()" , the position
    of the haptic device is retrieved at each simulation iteration.
    The interaction forces are then computed and sent to the device.
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
    cout << "Demo: 23-tooth" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[1] - Wireframe (ON/OFF)" << endl;
    cout << "[2] - Increase Opacity" << endl;
    cout << "[3] - Reduce Opacity" << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[x] - Exit application" << endl;
    cout << endl << endl;

    // parse first arg to try and locate resources
    string resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);


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
    camera->set(cVector3d(2.5, 0.0, 0.6),    // camera position (eye)
                cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
                cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    // anything in front or behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.03);
    camera->setStereoFocalLength(3.0);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a light source
    light = new cDirectionalLight(world);

    // attach light to camera
    camera->addChild(light);

    // enable light source
    light->setEnabled(true);

    // position the light source
    light->setLocalPos( 0.0, 0.5, 0.0);

    // define the direction of the light beam
    light->setDir(-3.0,-0.5, 0.0);

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

    // if the haptic device has a gripper, enable it as a user switch
    hapticDevice->setEnableGripperUserSwitch(true);

    // define the radius of the tool (sphere)
    double toolRadius = 0.01;

    // define a radius for the tool
    tool->setRadius(toolRadius);

    // hide the device sphere. only show proxy.
    tool->setShowContactPoints(true, false);

    // create a white cursor
    tool->m_hapticPoint->m_sphereProxy->m_material->setWhite();

    // enable if objects in the scene are going to rotate of translate
    // or possibly collide against the tool. If the environment
    // is entirely static, you can set this parameter to "false"
    tool->enableDynamicObjects(true);

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(1.0);

    // haptic forces are enabled only if small forces are first sent to the device;
    // this mode avoids the force spike that occurs when the application starts when 
    // the tool is located inside an object for instance. 
    tool->setWaitForSmallForce(true);

    // start the haptic tool
    tool->start();


    //--------------------------------------------------------------------------
    // COMPOSE OBJECTS
    //--------------------------------------------------------------------------

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // stiffness property
    double maxStiffness	= hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;


    /////////////////////////////////////////////////////////////////////////
    // OBJECT "TOOTH"
    /////////////////////////////////////////////////////////////////////////

    // create a virtual mesh
    tooth = new cMultiMesh();

    // add object to world
    world->addChild(tooth);

    // set the position and orientation of the object at the center of the world
    tooth->setLocalPos(0.0, 0.0, 0.0);
    tooth->rotateAboutGlobalAxisDeg(cVector3d(0.0, 0.0, 1.0),-10);
    tooth->rotateAboutGlobalAxisDeg(cVector3d(0.0, 1.0, 0.0), 10);

    // load an object file
    bool fileload;
    fileload = tooth->loadFromFile(RESOURCE_PATH("../resources/models/tooth/tooth.obj"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = tooth->loadFromFile("../../../bin/resources/models/tooth/tooth.obj");
        #endif
    }
    if (!fileload)
    {
        printf("Error - 3D Model failed to load correctly.\n");
        close();
        return (-1);
    }

    // make the outside of the tooth rendered in semi-transparent
    tooth->getMesh(1)->setUseTransparency(false);
    tooth->getMesh(1)->setTransparencyLevel(transparencyLevel);

    // compute a boundary box
    tooth->computeBoundaryBox(true);

    // resize tooth to screen
    tooth->scale(0.003);

    // compute collision detection algorithm
    tooth->createAABBCollisionDetector(toolRadius);

    // define a default stiffness for the object
    tooth->setStiffness(0.8 * maxStiffness, true);


    /////////////////////////////////////////////////////////////////////////
    // OBJECT "DRILL"
    /////////////////////////////////////////////////////////////////////////

    // create a new mesh.
    drill = new cMultiMesh();

    // load a drill like mesh and attach it to the tool
    fileload = drill->loadFromFile(RESOURCE_PATH("../resources/models/drill/drill.3ds"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = drill->loadFromFile("../../../bin/resources/models/drill/drill.3ds");
        #endif
    }
    if (!fileload)
    {
        printf("Error - 3D Model failed to load correctly.\n");
        close();
        return (-1);
    }

    // resize tool mesh model
    drill->scale(0.004);

    // remove the collision detector. we do not want to compute any
    // force feedback rendering on the object itself.
    drill->deleteCollisionDetector(true);

    // define a material property for the mesh
    cMaterial mat;
    mat.m_ambient.set(0.5f, 0.5f, 0.5f);
    mat.m_diffuse.set(0.8f, 0.8f, 0.8f);
    mat.m_specular.set(1.0f, 1.0f, 1.0f);
    drill->setMaterial(mat, true);
    drill->computeAllNormals();

    // attach drill to tool
    tool->m_image->addChild(drill);


    //--------------------------------------------------------------------------
    // SIDE FRAMEBUFFER
    //--------------------------------------------------------------------------

    // create secondary camera for side view
    cCamera* cameraTool = new cCamera(world);

    // attach camera to tool
    drill->addChild(cameraTool);
    cameraTool->setLocalPos(0.0, 0.0, 0.1); 

    // create framebuffer for side view
    frameBuffer = cFrameBuffer::create();
    frameBuffer->setup(cameraTool);

    // create panel to display side view
    viewPanel = new cViewPanel(frameBuffer);
    camera->m_frontLayer->addChild(viewPanel);
    viewPanel->setLocalPos(10, 10);


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFont *font = NEW_CFONTCALIBRI20();
    
    // create a label to display the haptic rate of the simulation
    labelHapticRate = new cLabel(font);
    labelHapticRate->m_fontColor.setBlack();
    camera->m_frontLayer->addChild(labelHapticRate);

    // create a background
    cBackground* background = new cBackground();
    camera->m_backLayer->addChild(background);

    // set background properties
    background->setCornerColors(cColorf(1.00f, 1.00f, 1.00f),
                                cColorf(0.95f, 0.95f, 0.95f),
                                cColorf(0.85f, 0.85f, 0.85f),
                                cColorf(0.80f, 0.80f, 0.80f));


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

    // update size of framebuffer and view panel
    int side = 0.3 * cMin(w, h);
    frameBuffer->setSize(side, side);
    
    int radius = 0.25 * side;
    viewPanel->setSize(side,side);
    viewPanel->setCornerRadius(radius, radius, radius, radius);
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

    // option 1: wireframe
    if (key == '1')
    {
        // only enable/disable wireframe of one part of the tooth model
        bool useWireMode = tooth->getMesh(1)->getWireMode();
        tooth->getMesh(1)->setWireMode(!useWireMode);
    }
    
    // option 2: decrease transparency
    if (key == '2')
    {
        // decrease transparency level
        transparencyLevel = transparencyLevel - 0.1;
        if (transparencyLevel < 0.0) { transparencyLevel = 0.0; }

        // apply changes to tooth
        tooth->getMesh(1)->setTransparencyLevel(transparencyLevel);
        tooth->getMesh(1)->setUseTransparency(true);

        // if object is almost transparent, make it invisible
        if (transparencyLevel < 0.1)
        {
            tooth->getMesh(1)->setShowEnabled(false, true);
            tooth->getMesh(1)->setHapticEnabled(false, true);
        }
    }

    // option 3: decrease transparency
    if (key == '3')
    {
        // increase transparency level
        transparencyLevel = transparencyLevel + 0.1;
        if (transparencyLevel > 1.0) { transparencyLevel = 1.0; }

        // apply changes to tooth
        tooth->getMesh(1)->setTransparencyLevel(transparencyLevel);

        // make object visible
        if (transparencyLevel >= 0.1)
        {
            tooth->getMesh(1)->setShowEnabled(true, true);
            tooth->getMesh(1)->setHapticEnabled(true, true);
        }

        // disable transparency is transparency level is set to 1.0
        if (transparencyLevel == 1.0)
        {
            tooth->getMesh(1)->setUseTransparency(false);
        }
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

    // update haptic rate data
    labelHapticRate->setText(cStr(frequencyCounter.getFrequency(), 0) + " Hz");

    // update position of label
    labelHapticRate->setLocalPos((int)(0.5 * (windowW - labelHapticRate->getWidth())), 15);


    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, mirroredDisplay);

    // render side framebuffer
    frameBuffer->renderView();

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
    // temp variable to store positions and orientations
    // of tooth and drill
    cVector3d lastPosObject;
    cMatrix3d lastRotObject;
    cVector3d lastPosDevice;
    cMatrix3d lastRotDevice;
    cVector3d lastDeviceObjectPos;
    cMatrix3d lastDeviceObjectRot;
    bool firstTime = false;

    // retrieve information about the current haptic device
    cHapticDeviceInfo info = hapticDevice->getSpecifications();

    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    // main haptic simulation loop
    while(simulationRunning)
    {
        /////////////////////////////////////////////////////////////////////////
        // HAPTIC RENDERING
        /////////////////////////////////////////////////////////////////////////

        // update frequency counter
        frequencyCounter.signal(1);

        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // update position and orientation of tool
        tool->updateFromDevice();

        // compute interaction forces
        tool->computeInteractionForces();

        // send forces to haptic device
        tool->applyToDevice();

 
        /////////////////////////////////////////////////////////////////////////
        // MANIPULATION
        /////////////////////////////////////////////////////////////////////////

        // if the haptic device does track orientations, we automatically
        // orient the drill to remain perpendicular to the tooth
        cVector3d pos = tool->m_hapticPoint->getGlobalPosProxy();
        cMatrix3d rot = tool->getDeviceGlobalRot();

        if (info.m_sensedRotation == false)
        {
            rot.identity();

            cVector3d vx, vy, vz;
            cVector3d zUp (0,0,1);
            cVector3d yUp (0,1,0);
            vx = pos - tooth->getLocalPos();
            if (vx.length() > 0.001)
            {
                vx.normalize();

                if (cAngle(vx,zUp) > 0.001)
                {
                    vy = cCross(zUp, vx);
                    vy.normalize();
                    vz = cCross(vx, vy);
                    vz.normalize();

                }
                else
                {
                    vy = cCross(yUp, vx);
                    vy.normalize();
                    vz = cCross(vx, vy);
                    vz.normalize();
                }

                rot.setCol(vx, vy, vz);
                drill->setLocalRot(rot);
            }
        }

        int button = tool->getUserSwitch(0);
        if (button == 0)
        {
            lastPosDevice = pos;
            lastRotDevice = rot;
            lastPosObject = tooth->getLocalPos();
            lastRotObject = tooth->getLocalRot();
            lastDeviceObjectPos = cTranspose(lastRotDevice) * ((lastPosObject - lastPosDevice) + 0.01*cNormalize(lastPosObject - lastPosDevice));
            lastDeviceObjectRot = cMul(cTranspose(lastRotDevice), lastRotObject);
            drill->setTransparencyLevel (1.0, true, true);
            firstTime = true;
        }
        else if (firstTime)
        {
            drill->setTransparencyLevel (0.3, true, true);
            cMatrix3d newRot =  cMul(rot, lastDeviceObjectRot);
            cVector3d newPos = cAdd(pos, cMul(rot, lastDeviceObjectPos));
            tooth->setLocalPos(newPos);
            tooth->setLocalRot(newRot);
            world->computeGlobalPositions(true);
        }
    }

    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------
