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
    \version   3.1.1 $Rev: 1916 $
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

// some objects.
cShapeSphere* sphere;

// a small sphere which displays the position of a click hit in the world
cShapeSphere* sphereSelect;

// a small line to display the surface normal at the selection point
cShapeLine* normalSelect;

// a pointer to the selected object
cGenericObject* selectedObject = NULL;

// offset between the position of the mmouse click on the object and the object reference frame location.
cVector3d selectedObjectOffset;

// position of mouse click.
cVector3d selectedPoint;

// flag that says if theselected object is being moved by the mous
bool flagMoveObject = false;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelHapticRate;

// a label to explain what is happening
cLabel* labelMessage;

// a widget panel
cPanel* panel;

// some labels
cLabel* labelRed;
cLabel* labelGreen;
cLabel* labelBlue;
cLabel* labelOrange;
cLabel* labelGray;

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

// callback to handle mouse click
void mouseClick(int button, int state, int x, int y);

// callback to handle mouse motion when button is pressed
void mouseMove(int x, int y);


//==============================================================================
/*
    DEMO:   07-mouse-select.cpp

    This application illustrates how the computer mouse can be used to select
    widgets and objects in the virtual world.
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
    cout << "Demo: 07-mouse-select" << endl;
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
    camera->set(cVector3d(0.8, 0.8, 0.6),    // camera position (eye)
                cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
                cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

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
    light = new cDirectionalLight(world);

    // attach light to camera
    camera->addChild(light);

    // enable light source
    light->setEnabled(true);

    // position the light source
    light->setLocalPos(0.0, 0.5, 0.0);

    // define the direction of the light beam
    light->setDir(-2.5,-0.8, 0.0);


    //--------------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    // if the haptic devices carries a gripper, enable it to behave like a user switch
    hapticDevice->setEnableGripperUserSwitch(true);

    // create a tool (cursor) and insert into the world
    tool = new cToolCursor(world);
    world->addChild(tool);

    // connect the haptic device to the tool
    tool->setHapticDevice(hapticDevice);

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(0.9);

    // define the radius of the tool (sphere)
    double toolRadius = 0.01;

    // define a radius for the tool
    tool->setRadius(toolRadius);

    // hide the device sphere. only show proxy.
    tool->setShowContactPoints(true, false);

    // enable if objects in the scene are going to rotate of translate
    // or possibly collide against the tool. If the environment
    // is entirely static, you can set this parameter to "false"
    tool->enableDynamicObjects(true);

    // start the haptic tool
    tool->start();


    //--------------------------------------------------------------------------
    // CREATE OBJECTS
    //--------------------------------------------------------------------------

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // stiffness properties
    double maxStiffness	= hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;

    // create a small sphere to display a selection hit with the mouse
    sphereSelect = new cShapeSphere(0.005);
    world->addChild(sphereSelect);
    sphereSelect->m_material->setRedCrimson();
    sphereSelect->setShowEnabled(false);
    sphereSelect->setGhostEnabled(true);

    normalSelect = new cShapeLine();
    sphereSelect->addChild(normalSelect);
    normalSelect->m_colorPointA.setRedCrimson();
    normalSelect->m_colorPointB.setRedCrimson();
    normalSelect->setShowEnabled(false);
    normalSelect->setGhostEnabled(true);

    
    ////////////////////////////////////////////////////////////////////////////
    // MESH - PRIMITIVE
    ////////////////////////////////////////////////////////////////////////////

    // create a virtual mesh
    cMesh* mesh = new cMesh();

    // add object to world
    world->addChild(mesh);

    // build mesh using a cylinder primitive
    cCreateCircularArrow(mesh,
        0.02,
        0.02, 
        0.50,
        0.10,
        0.04,
        330,
        true,
        32,
        72,
        cVector3d(0,0,1)
        );

    // set material color
    mesh->m_material->setBlueCornflower();

    // set haptic properties
    mesh->m_material->setStiffness(0.5 * maxStiffness);

    // build collision detection tree
    mesh->createAABBCollisionDetector(toolRadius);


    ////////////////////////////////////////////////////////////////////////////
    // SHAPE - BOX
    ////////////////////////////////////////////////////////////////////////////

    cShapeBox* box = new cShapeBox(0.08, 0.08, 0.20);
    world->addChild(box);
    box->setLocalPos(0.0, 0.0, 0.2);
    box->createEffectSurface();
    box->m_material->setStiffness(0.8 * maxStiffness);
    box->m_material->setGrayLight();


    ////////////////////////////////////////////////////////////////////////////
    // SHAPE - SPHERE
    ////////////////////////////////////////////////////////////////////////////

    sphere = new cShapeSphere(0.06);
    world->addChild(sphere);
    sphere->setLocalPos(0.0, 0.0, 0.35);
    sphere->createEffectSurface();
    sphere->m_material->setStiffness(0.8 * maxStiffness);
    sphere->m_material->setGrayLight();


    ////////////////////////////////////////////////////////////////////////////
    // SHAPE - CYLINDER
    ////////////////////////////////////////////////////////////////////////////

    cShapeCylinder* cylinder = new cShapeCylinder(0.12, 0.08, 0.12);
    world->addChild(cylinder);
    cylinder->setLocalPos(0.0, 0.0, 0.0);
    cylinder->createEffectSurface();
    cylinder->m_material->setStiffness(0.8 * maxStiffness);
    cylinder->m_material->setGrayLight();


    ////////////////////////////////////////////////////////////////////////////
    // SHAPE - TORUS
    ////////////////////////////////////////////////////////////////////////////

    cShapeTorus* torus = new cShapeTorus(0.03, 0.4);
    world->addChild(torus);
    torus->setLocalPos(0.0, 0.0, 0.0);
    torus->createEffectSurface();
    torus->m_material->setStiffness(0.8 * maxStiffness);
    torus->m_material->setGrayLight();


    ////////////////////////////////////////////////////////////////////////////
    // SHAPE - LINE
    ////////////////////////////////////////////////////////////////////////////

    cShapeLine* line1 = new cShapeLine(cVector3d(0.0,-0.4, 0.0), cVector3d(0.0, 0.4, 0.0));
    world->addChild(line1);
    line1->m_colorPointA.setGrayDarkSlate();
    line1->m_colorPointB.setGrayDarkSlate();
    line1->setLineStipple(1, 0xF0F0);
    line1->setLineWidth(2.0);

    cShapeLine* line2 = new cShapeLine(cVector3d(0.4, 0.0, 0.0), cVector3d(-0.4, 0.0, 0.0));
    world->addChild(line2);
    line2->m_colorPointA.setGrayDarkSlate();
    line2->m_colorPointB.setGrayDarkSlate();
    line2->setLineStipple(1, 0xF0F0);
    line2->setLineWidth(2.0);


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFont *font = NEW_CFONTCALIBRI20();

    // create a label to display the haptic rate of the simulation
    labelHapticRate = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticRate);

    // set font color
    labelHapticRate->m_fontColor.setBlack();

    // create a background
    cBackground* background = new cBackground();
    camera->m_backLayer->addChild(background);

    // set background properties
    background->setCornerColors(cColorf(1.0, 1.0, 1.0),
                                cColorf(1.0, 1.0, 1.0),
                                cColorf(0.8, 0.8, 0.8),
                                cColorf(0.8, 0.8, 0.8));

    // a widget panel
    panel = new cPanel();
    camera->m_frontLayer->addChild(panel);
    panel->setSize(100, 115);
    panel->m_material->setGrayDim();
    panel->setTransparencyLevel(0.8);

    // create some labels
    labelRed = new cLabel(font);
    panel->addChild(labelRed);
    labelRed->setText("red");
    labelRed->setLocalPos(15, 10, 0.1);
    labelRed->m_fontColor.setWhite();

    labelGreen = new cLabel(font);
    panel->addChild(labelGreen);
    labelGreen->setText("green");
    labelGreen->setLocalPos(15, 30, 0.1);
    labelGreen->m_fontColor.setWhite();

    labelBlue = new cLabel(font);
    panel->addChild(labelBlue);
    labelBlue->setText("blue");
    labelBlue->setLocalPos(15, 50, 0.1);
    labelBlue->m_fontColor.setWhite();

    labelOrange = new cLabel(font);
    panel->addChild(labelOrange);
    labelOrange->setText("orange");
    labelOrange->setLocalPos(15, 70, 0.1);
    labelOrange->m_fontColor.setWhite();

    labelGray = new cLabel(font);
    panel->addChild(labelGray);
    labelGray->setText("gray");
    labelGray->setLocalPos(15, 90, 0.1);
    labelGray->m_fontColor.setWhite();

    // create a label with a small message
    labelMessage = new cLabel(font);
    camera->m_frontLayer->addChild(labelMessage);

    // set font color
    labelMessage->m_fontColor.setBlack();

    // set text message
    labelMessage->setText("use mouse to select and move objects");


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
}

//------------------------------------------------------------------------------

void keySelect(unsigned char key, int x, int y)
{
    // option ESC: exit
    if ((key == 27) || (key == 'x'))
    {
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

void mouseClick(int button, int state, int x, int y)
{
    // mouse button down
    if (state == GLUT_DOWN)
    {
        cCollisionRecorder recorder;
        cCollisionSettings settings;

        flagMoveObject = false;

        // detect for any collision between mouse and front layer widgets
        bool hit = camera->selectFrontLayer(x, (windowH - y), windowW, windowH, recorder, settings);
        if(hit)
        {
            // reset all label font colors to white
            labelRed->m_fontColor.setWhite();
            labelGreen->m_fontColor.setWhite();
            labelBlue->m_fontColor.setWhite();
            labelOrange->m_fontColor.setWhite();
            labelGray->m_fontColor.setWhite();

            // check mouse selection
            if (recorder.m_nearestCollision.m_object == labelRed)
            {
                labelRed->m_fontColor.setBlack();
                if (selectedObject != NULL)
                    selectedObject->m_material->setRedCrimson();
            }
            else if (recorder.m_nearestCollision.m_object == labelGreen)
            {
                labelGreen->m_fontColor.setBlack();
                if (selectedObject != NULL)
                    selectedObject->m_material->setGreenLightSea();
            }
            else if (recorder.m_nearestCollision.m_object == labelBlue)
            {
                labelBlue->m_fontColor.setBlack();
                if (selectedObject != NULL)
                    selectedObject->m_material->setBlueCornflower();
            }
            else if (recorder.m_nearestCollision.m_object == labelOrange)
            {
                labelOrange->m_fontColor.setBlack();
                if (selectedObject != NULL)
                    selectedObject->m_material->setOrangeRed();
            }
            else if (recorder.m_nearestCollision.m_object == labelGray)
            {
                labelGray->m_fontColor.setBlack();
                if (selectedObject != NULL)
                    selectedObject->m_material->setGrayLight();
            }
        }
        else
        {
            // detect for any collision between mouse and world
            bool hit = camera->selectWorld(x, (windowH - y), windowW, windowH, recorder, settings);
            if (hit)
            {
                sphereSelect->setShowEnabled(true);
                normalSelect->setShowEnabled(true);
                selectedPoint = recorder.m_nearestCollision.m_globalPos;
                sphereSelect->setLocalPos(selectedPoint);
                normalSelect->m_pointA.zero();
                normalSelect->m_pointB = 0.1 * recorder.m_nearestCollision.m_globalNormal;
                selectedObject = recorder.m_nearestCollision.m_object;
                selectedObjectOffset = recorder.m_nearestCollision.m_globalPos - selectedObject->getLocalPos();
                flagMoveObject = true;
            }
        }
    }
}

//------------------------------------------------------------------------------

void mouseMove(int x, int y)
{
    // here we move the selected object according to the position of the mouse.
    if ((selectedObject != NULL) && (flagMoveObject))
    {
        // get the vector that goes from the camera to the selected point (mouse click)
        cVector3d vCameraObject = selectedPoint - camera->getLocalPos();

        // get the vector that point in the direction of the camera. ("where the camera is lookint at")
        cVector3d vCameraLookAt = camera->getLookVector();

        // compute the angle between both vectors
        double angle = cAngle(vCameraObject, vCameraLookAt);

        // compute the distance between the camera and the plane that intersects the object and 
        // which is parallel to the camera plane
        double distanceToObjectPlane = vCameraObject.length() * cos(angle);

        // convert the pixel in mouse space into a relative position in the world
        double factor = (distanceToObjectPlane * tan(0.5 * camera->getFieldViewAngleRad())) / (0.5 * windowH);
        double posRelX = factor * (x - (0.5 * windowW));
        double posRelY = factor * ((windowH - y) - (0.5 * windowH));

        // compute the new position in world coordinates
        cVector3d pos = camera->getLocalPos() +
                        distanceToObjectPlane * camera->getLookVector() +
                        posRelX * camera->getRightVector() +
                        posRelY * camera->getUpVector();

        // compute position of object by taking in account offset
        cVector3d posObject = pos - selectedObjectOffset;

        // apply new position to object
        selectedObject->setLocalPos(posObject);

        // place cursor at the position of the mouse click
        sphereSelect->setLocalPos(pos);
    }
}

//------------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // display haptic rate data
    labelHapticRate->setText(cStr(frequencyCounter.getFrequency(), 0) + " Hz");

    // update position of label
    labelHapticRate->setLocalPos((int)(0.5 * (windowW - labelHapticRate->getWidth())), 15);

    // update panel position
    panel->setLocalPos(10, (windowH - panel->getHeight()) - 10);

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
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error:  %s\n" << gluErrorString(err);
}

//------------------------------------------------------------------------------

void updateHaptics(void)
{
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
    }

    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------
