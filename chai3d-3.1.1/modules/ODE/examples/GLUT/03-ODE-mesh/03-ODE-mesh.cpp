//===========================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D
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
    \version   3.1.1 $Rev: 1928 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "chai3d.h"
//---------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//---------------------------------------------------------------------------
#include "CODE.h"
//---------------------------------------------------------------------------
#ifndef MACOSX
#include "GL/glut.h"
#else
#include "GLUT/glut.h"
#endif
//---------------------------------------------------------------------------

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


//---------------------------------------------------------------------------
// CHAI3D VARIABLES
//---------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cSpotLight *light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
shared_ptr<cGenericHapticDevice> hapticDevice;

// a virtual tool representing the haptic device in the scene
cGenericTool* tool;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelHapticRate;


//---------------------------------------------------------------------------
// ODE MODULE VARIABLES
//---------------------------------------------------------------------------

// ODE world
cODEWorld* ODEWorld;

// ODE object
cODEGenericBody* ODEGear1;
cODEGenericBody* ODEGear2;
cODEGenericBody* ODEAxis1;
cODEGenericBody* ODEAxis2;

cODEGenericBody* ODEGPlane0;
cODEGenericBody* ODEGPlane1;
cODEGenericBody* ODEGPlane2;
cODEGenericBody* ODEGPlane3;
cODEGenericBody* ODEGPlane4;
cODEGenericBody* ODEGPlane5;

// a pointer the ODE object grasped by the tool
cODEGenericBody* graspObject;

// grasp position is respect to object
cVector3d graspPos;

// is grasp currently active?
bool graspActive = false;

// a small line used to display a grasp
cShapeLine* graspLine;


//---------------------------------------------------------------------------
// GENERAL VARIABLES
//---------------------------------------------------------------------------

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

// root resource path
string resourceRoot;


//---------------------------------------------------------------------------
// DECLARED MACROS
//---------------------------------------------------------------------------
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())


//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

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


//===========================================================================
/*
    DEMO:    ODE_mesh.cpp

    This example illustrates the use of the ODE framework for simulating
    haptic interaction with dynamic bodies. In this demo we introduce
    triangle based objects (cMesh) for which collision detection between
    triangles is computed. Collision detection using meshes can be a
    rather tricky business under ODE, so make sure to read more about the
    ODE API if you are experiencing some instabilities.

 */
//===========================================================================

int main(int argc, char* argv[])
{
    //-----------------------------------------------------------------------
    // INITIALIZATION
    //-----------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "Demo: 43-ODE-mesh" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[g] - Enable/Disable gravity" << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[x] - Exit application" << endl;
    cout << endl << endl;

    // parse first arg to try and locate resources
    resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);


    //-----------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //-----------------------------------------------------------------------

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


    //-----------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //-----------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setWhite();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and oriente the camera
    camera->set(cVector3d(1.5, 0.0, 0.5),    // camera position (eye)
                cVector3d(0.0, 0.0,-0.5),    // lookat position (target)
                cVector3d(0.0, 0.0, 1.0));   // direction of the "up" vector

    // set the near and far clipping planes of the camera
    // anything in front/behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.02);
    camera->setStereoFocalLength(2.0);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a light source
    light = new cSpotLight(world);

    // attach light to camera
    world->addChild(light);    

    // enable light source
    light->setEnabled(true);                   

    // position the light source
    light->setLocalPos(0.3, 0.0, 1.0);

    // define the direction of the light beam
    light->setDir(-0.3, 0.0,-1.0);             

    // set uniform concentration level of light 
    light->setSpotExponent(0.0);

    // enable this light source to generate shadows
    light->setShadowMapEnabled(true);

    // set the resolution of the shadow map
    light->m_shadowMap->setQualityLow();
    //light->m_shadowMap->setQualityMedium();

    // set light cone half angle
    light->setCutOffAngleDeg(45);


    //-----------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //-----------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device
    shared_ptr<cGenericHapticDevice> hapticDevice;
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    // emulate button if device has a force gripper
    hapticDevice->setEnableGripperUserSwitch(true);

    // create a 3D tool and add it to the world
    tool = new cToolCursor(world);
    world->addChild(tool);

    // connect the haptic device to the tool
    tool->setHapticDevice(hapticDevice);

    // initialize tool by connecting to haptic device
    tool->start();

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(1.3);

    // define a radius for the tool (graphical display)
    double toolRadius = 0.06;
    tool->setRadius(toolRadius, toolRadius);

    // hide the device sphere. only show proxy.
    tool->setShowContactPoints(true, false);

    // enable if objects in the scene are going to rotate of translate
    // or possibly collide against the tool. If the environment
    // is entirely static, you can set this parameter to "false"
    tool->enableDynamicObjects(true);

    // haptic forces are enabled only if small forces are first sent to the device;
    // this mode avoids the force spike that occurs when the application starts when 
    // the tool is located inside an object for instance. 
    tool->setWaitForSmallForce(true);

    // start the haptic tool
    tool->start();


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFont *font = NEW_CFONTCALIBRI20();

    // create a label to display the haptic rate of the simulation
    labelHapticRate = new cLabel(font);
    labelHapticRate->m_fontColor.setBlack();
    camera->m_frontLayer->addChild(labelHapticRate);


    //-----------------------------------------------------------------------
    // CREATE ODE WORLD AND OBJECTS
    //-----------------------------------------------------------------------

    // create a small white line that will be enabled every time the operator
    // grasps an object. The line indicated the connection between the
    // position of the tool and the grasp position on the object
    graspLine = new cShapeLine(cVector3d(0,0,0), cVector3d(0,0,0));
    world->addChild(graspLine);
    graspLine->m_colorPointB.set(1.0, 1.0, 1.0);
    graspLine->m_colorPointB.set(1.0, 1.0, 1.0);
    graspLine->setShowEnabled(false);


    //////////////////////////////////////////////////////////////////////////
    // ODE WORLD
    //////////////////////////////////////////////////////////////////////////

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // stiffness properties
    double maxStiffness	= hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;

    // create an ODE world to simulate dynamic bodies
    ODEWorld = new cODEWorld(world);

    // add ODE world as a node inside world
    world->addChild(ODEWorld);

    // set some gravity
    ODEWorld->setGravity(cVector3d(0.0, 0.0, -9.81));

    // define damping properties
    ODEWorld->setAngularDamping(0.00002);
    ODEWorld->setLinearDamping(0.00002);


    //////////////////////////////////////////////////////////////////////////
    // ODE MESHES
    //////////////////////////////////////////////////////////////////////////

    // create a new ODE object that is automatically added to the ODE world
    ODEGear1 = new cODEGenericBody(ODEWorld);
    ODEGear2 = new cODEGenericBody(ODEWorld);
    ODEAxis1 = new cODEGenericBody(ODEWorld);
    ODEAxis2 = new cODEGenericBody(ODEWorld);

    // create a virtual mesh  that will be used for the geometry
    // representation of the dynamic body
    cMultiMesh* gear1 = new cMultiMesh();
    cMultiMesh* gear2 = new cMultiMesh();
    cMultiMesh* axis1 = new cMultiMesh();
    cMultiMesh* axis2 = new cMultiMesh();

    // load object 1 - a gear
    bool fileload;
    fileload = gear1->loadFromFile(RESOURCE_PATH("../resources/models/gear/gear.3ds"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = gear1->loadFromFile("../../../bin/resources/models/gear/gear.3ds");
        #endif
    }
    gear1->scale(0.0015);
    gear1->createAABBCollisionDetector(toolRadius);

    // load object 2 - a gear
    fileload = gear2->loadFromFile(RESOURCE_PATH("../resources/models/gear/gear.3ds"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = gear2->loadFromFile("../../../bin/resources/models/gear/gear.3ds");
        #endif
    }
    gear2->scale(0.0015);
    gear2->createAABBCollisionDetector(toolRadius);

    // load static object 1 - a shaft
    fileload = axis1->loadFromFile(RESOURCE_PATH("../resources/models/gear/axis.3ds"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = axis1->loadFromFile("../../../bin/resources/models/gear/axis.3ds");
        #endif
    }
    axis1->scale(0.0015);
    axis1->createAABBCollisionDetector(toolRadius);

    // load static object 2 - a shaft
    fileload = axis2->loadFromFile(RESOURCE_PATH("../resources/models/gear/axis.3ds"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = axis2->loadFromFile("../../../bin/resources/models/gear/axis.3ds");
        #endif
    }
    axis2->scale(0.0015);
    axis2->createAABBCollisionDetector(toolRadius);

    // define material properties for both types of objects
    cMaterial matGear;
    matGear.setRedCrimson();
    matGear.setStiffness(0.2 * maxStiffness);
    matGear.setDynamicFriction(0.2);
    matGear.setStaticFriction(0.2);
    gear1->setMaterial(matGear, true);
    gear2->setMaterial(matGear, true);

    cMaterial matAxis;
    matAxis.setGrayLevel(0.6);
    matAxis.setStiffness(0.2 * maxStiffness);
    matAxis.setDynamicFriction(0.2);
    matAxis.setStaticFriction(0.2);
    axis1->setMaterial(matAxis, true);
    axis2->setMaterial(matAxis, true);

    // define image models
    ODEGear1->setImageModel(gear1);
    ODEGear2->setImageModel(gear2);
    ODEAxis1->setImageModel(axis1);
    ODEAxis2->setImageModel(axis2);

    // create a dynamic model of the ODE object. Here we decide to use a box just like
    // the object mesh we just defined
    ODEGear1->createDynamicMesh();
    ODEGear2->createDynamicMesh();
    ODEAxis1->createDynamicCapsule(0.050, 0.47, true);  // true = make object static
    ODEAxis2->createDynamicCapsule(0.050, 0.47, true);  // true = make object static

    // define some mass properties for each gear
    ODEGear1->setMass(0.05);
    ODEGear2->setMass(0.05);

    // default distance between both gears when demo starts
    double DISTANCE = 0.260;

    // position and orient gear 1
    ODEGear1->setLocalPos(0.0,-DISTANCE,  0.5);
    ODEGear1->rotateAboutGlobalAxisDeg(cVector3d(1,0,0), 90);

    // position and orient gear 1
    ODEGear2->setLocalPos(0.0, DISTANCE,  0.5);
    ODEGear2->rotateAboutGlobalAxisDeg(cVector3d(0,0,1), 14);

    // position shaft 1
    ODEAxis1->setLocalPos(0.0,-DISTANCE, -0.7);

    // position shaft 2
    ODEAxis2->setLocalPos(0.0, DISTANCE, -0.7);

    // we now create 6 static walls to bound the workspace of our simulation
    ODEGPlane0 = new cODEGenericBody(ODEWorld);
    ODEGPlane1 = new cODEGenericBody(ODEWorld);
    ODEGPlane2 = new cODEGenericBody(ODEWorld);
    ODEGPlane3 = new cODEGenericBody(ODEWorld);
    ODEGPlane4 = new cODEGenericBody(ODEWorld);
    ODEGPlane5 = new cODEGenericBody(ODEWorld);

    double size = 1.0;
    ODEGPlane0->createStaticPlane(cVector3d(0.0, 0.0,  2.0 *size), cVector3d(0.0, 0.0 ,-1.0));
    ODEGPlane1->createStaticPlane(cVector3d(0.0, 0.0, -size), cVector3d(0.0, 0.0 , 1.0));
    ODEGPlane2->createStaticPlane(cVector3d(0.0,  size, 0.0), cVector3d(0.0,-1.0, 0.0));
    ODEGPlane3->createStaticPlane(cVector3d(0.0, -size, 0.0), cVector3d(0.0, 1.0, 0.0));
    ODEGPlane4->createStaticPlane(cVector3d( size, 0.0, 0.0), cVector3d(-1.0,0.0, 0.0));
    ODEGPlane5->createStaticPlane(cVector3d(-0.8 * size, 0.0, 0.0), cVector3d( 1.0,0.0, 0.0));


    //////////////////////////////////////////////////////////////////////////
    // GROUND
    //////////////////////////////////////////////////////////////////////////

    // create a mesh that represents the ground
    cMesh* ground = new cMesh();
    world->addChild(ground);

    // create a plane
    double groundSize = 2.0;
    cCreatePlane(ground, groundSize, groundSize);

    // position ground in world where the invisible ODE plane is located (ODEGPlane1)
    ground->setLocalPos(0.0, 0.0, -1.0);

    // define some material properties and apply to mesh
    cMaterial matGround;
    matGround.setStiffness(0.3 * maxStiffness);
    matGround.setDynamicFriction(0.2);
    matGround.setStaticFriction(0.0);
    matGround.setWhite();
    matGround.m_emission.setGrayLevel(0.3);
    ground->setMaterial(matGround);

    // setup collision detector
    ground->createAABBCollisionDetector(toolRadius);


    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------

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

//---------------------------------------------------------------------------

void resizeWindow(int w, int h)
{
    windowW = w;
    windowH = h;
}

//---------------------------------------------------------------------------

void keySelect(unsigned char key, int x, int y)
{
    // option ESC: exit
    if ((key == 27) || (key == 'x'))
    {
        // exit application
        exit(0);
    }

    // option g: enable/disable gravity
    if (key == 'g')
    {
        if (ODEWorld->getGravity().length() > 0.0)
        {
            ODEWorld->setGravity(cVector3d(0.0, 0.0, 0.0));
        }
        else
        {
            ODEWorld->setGravity(cVector3d(0.0, 0.0,-9.81));
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

//---------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    tool->stop();
}

//---------------------------------------------------------------------------

void graphicsTimer(int data)
{
    if (simulationRunning)
    {
        glutPostRedisplay();
    }

    glutTimerFunc(50, graphicsTimer, 0);
}

//---------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update haptic rate label
    labelHapticRate->setText (cStr(frequencyCounter.getFrequency(), 0) + " Hz");

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
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));
}

//---------------------------------------------------------------------------

void updateHaptics(void)
{
    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    // reset clock
    cPrecisionClock clock;
    clock.reset();

    // main haptic simulation loop
    while(simulationRunning)
    {
        /////////////////////////////////////////////////////////////////////
        // SIMULATION TIME    
        /////////////////////////////////////////////////////////////////////

        // stop the simulation clock
        clock.stop();

        // read the time increment in seconds
        double timeInterval = cClamp(clock.getCurrentTimeSeconds(), 0.0001, 0.001);

        // restart the simulation clock
        clock.reset();
        clock.start();

        // update frequency counter
        frequencyCounter.signal(1);


        /////////////////////////////////////////////////////////////////////
        // HAPTIC FORCE COMPUTATION
        /////////////////////////////////////////////////////////////////////

        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // update position and orientation of tool
        tool->updateFromDevice();

        // compute interaction forces
        tool->computeInteractionForces();


        /////////////////////////////////////////////////////////////////////
        // DYNAMIC SIMULATION
        /////////////////////////////////////////////////////////////////////
        if (graspActive)
        {
            // check if button pressed
            if (tool->getUserSwitch(0))
            {
                // position grasp point in world coordinates
                cVector3d posA = graspObject->getLocalTransform() * graspPos;

                // position of tool
                cVector3d posB = tool->getHapticPoint(0)->getGlobalPosGoal();

                // update line
                graspLine->m_pointA = posA;
                graspLine->m_pointB = posB;
                graspLine->setShowEnabled(true);

                // compute force
                cVector3d force = 5.0 * (posB - posA);

                // apply force
                graspObject->addExternalForceAtPoint(force, posA);

                // apply reaction force to haptic device
                tool->addDeviceGlobalForce(-force);
            }
            else
            {
                graspLine->setShowEnabled(false);
                graspActive = false;
                graspObject = NULL;
            }
        }
        else
        {
            // get pointer to next interaction point of tool
            cHapticPoint* interactionPoint = tool->getHapticPoint(0);

            // check primary contact point if available
            if (interactionPoint->getNumCollisionEvents() > 0)
            {
                cCollisionEvent* collisionEvent = interactionPoint->getCollisionEvent(0);

                // given the mesh object we may be touching, we search for its owner which
                // could be the mesh itself or a multi-mesh object. Once the owner found, we
                // look for the parent that will point to the ODE object itself.
                cGenericObject* object = collisionEvent->m_object->getOwner()->getOwner();

                // cast to ODE object
                cODEGenericBody* ODEobject = dynamic_cast<cODEGenericBody*>(object);

                // if ODE object, we apply interaction forces
                if (ODEobject != NULL)
                {
                    ODEobject->addExternalForceAtPoint(-0.3 * interactionPoint->getLastComputedForce(),
                        collisionEvent->m_globalPos);

                    // check if button pressed
                    if (tool->getUserSwitch(0))
                    {
                        graspObject = ODEobject;
                        graspPos = collisionEvent->m_localPos;
                        graspActive = true;
                    }
                }
            }
        }

        // update simulation
        ODEWorld->updateDynamics(timeInterval);

        // send forces to haptic device
        tool->applyToDevice();
    }

    // exit haptics thread
    simulationFinished = true;
}

//---------------------------------------------------------------------------
