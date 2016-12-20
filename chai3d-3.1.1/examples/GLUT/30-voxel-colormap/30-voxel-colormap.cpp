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
    \author    Sonny Chan
    \author    Francois Conti
    \version   3.1.1 $Rev: 1292 $
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

// a sphere to show the projected point on the surface
cShapeSphere* cursor;

// a virtual heart object from a CT scan
cVoxelObject* object;

// colour lookup tables for the volume
cImagePtr boneLUT;
cImagePtr softLUT;

// angular velocity of object
cVector3d angVel(0.0, 0.0, 0.1);

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

// last mouse position
int mouseX;
int mouseY;

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

// callback when a key from the representing is pressed
void keySelect(unsigned char key, int x, int y);

// callback to handle mouse click
void mouseClick(int button, int state, int x, int y);

// callback to handle mouse motion when button is pressed
void mouseMove(int x, int y);

// function called before exiting the application
void close(void);

// callback to render graphic scene
void updateGraphics(void);

// callback of GLUT timer
void graphicsTimer(int data);

// main haptics loop
void updateHaptics(void);


//==============================================================================
/*
    DEMO:    30-voxel-colormap.cpp

    This demonstration illustrates the use of a isosurfaces and colormap on CT 
    imaging data.
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
    cout << "Demo: 30-voxel-colormap" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[1] - View skin" << endl;
    cout << "[2] - View bones" << endl;
    cout << "[4,5] Adjust slicing along X axis" << endl;
    cout << "[6,7] Adjust slicing along Y axis" << endl;
    cout << "[8,9] Adjust slicing along Z axis" << endl;
    cout << "[q,w] Adjust quality of graphic rendering" << endl;
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
    world->m_backgroundColor.setBlack();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // define a basis in spherical coordinates for the camera
    camera->setSphericalReferences(cVector3d(0,0,0),    // origin
                                   cVector3d(0,0,1),    // zenith direction
                                   cVector3d(1,0,0));   // azimuth direction

    camera->setSphericalDeg(2.0,    // spherical coordinate radius
                            30,     // spherical coordinate azimuth angle
                            10);    // spherical coordinate polar angle

    // set the near and far clipping planes of the camera
    // anything in front or behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.1, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.02);
    camera->setStereoFocalLength(2.0);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a light source
    light = new cDirectionalLight(world);

    // attach light to camera
    camera->addChild(light);    

    // enable light source
    light->setEnabled(true);

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

    // define a radius for the virtual tool (sphere)
    tool->setRadius(0.02);

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(0.7);

    // oriente tool with camera
    tool->setLocalRot(camera->getLocalRot());

    // haptic forces are enabled only if small forces are first sent to the device;
    // this mode avoids the force spike that occurs when the application starts when 
    // the tool is located inside an object for instance. 
    tool->setWaitForSmallForce(true);

    // start the haptic tool
    tool->start();

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // stiffness properties
    double maxStiffness	= hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;


    //--------------------------------------------------------------------------
    // CREATE OBJECT
    //--------------------------------------------------------------------------

    // create a volumetric model
    object = new cVoxelObject();

    // add object to world
    world->addChild(object);
    
    // position object
    object->setLocalPos(0.0, 0.0, 0.0);

    // rotate object
    object->rotateExtrinsicEulerAnglesDeg(0, 0, 65, C_EULER_ORDER_XYZ);
    
    // set the dimensions by assigning the position of the min and max corners
    object->m_minCorner.set(-0.5,-0.5,-0.5);
    object->m_maxCorner.set( 0.5, 0.5, 0.5);
    
    // set the texture coordinate at each corner.
    object->m_minTextureCoord.set(0.0, 0.0, 0.0);
    object->m_maxTextureCoord.set(1.0, 1.0, 1.0);

    // set haptic properties
    object->m_material->setStiffness(0.2 * maxStiffness);
    object->m_material->setStaticFriction(0.0);
    object->m_material->setDynamicFriction(0.0);

    // enable materials
    object->setUseMaterial(true);

    // set material
    object->m_material->setWhite();

    // set quality of graphic rendering
    object->setQuality(0.5);

    // set graphic rendering mode 
    //object->setRenderingModeIsosurfaceColorMap();   // medium quality
    object->setRenderingModeDVRColorMap();            // high quality


    //--------------------------------------------------------------------------
    // LOAD VOXEL DATA
    //--------------------------------------------------------------------------

    // create multi image
    cMultiImagePtr image = cMultiImage::create();

    int filesloaded = image->loadFromFiles(RESOURCE_PATH("../resources/volumes/heart/heart0"), "png", 179);
    if (filesloaded == 0) {
        #if defined(_MSVC)
        filesloaded = image->loadFromFiles("../../../bin/resources/volumes/heart/heart0", "png", 179);
        #endif
    }
    if (filesloaded == 0) {
        cout << "Error - Failed to load volume data heartXXXX.png." << endl;
        close();
        return -1;
    }

    // create texture
    cTexture3dPtr texture = cTexture3d::create();

    // assign volumetric image to texture
    texture->setImage(image);

    // assign texture to voxel object
    object->setTexture(texture);

    // create texture
    texture = cTexture3d::create();

    // assign volumetric image to texture
    texture->setImage(image);

    // assign texture to voxel object
    object->setTexture(texture);

    // initially select an isosurface corresponding to the bone/heart level
    object->setIsosurfaceValue(0.30);

    // set optical density factor
    object->setOpticalDensity(1.2);


    //--------------------------------------------------------------------------
    // LOAD COLORMAPS
    //--------------------------------------------------------------------------

    boneLUT = cImage::create();
    bool fileLoaded = boneLUT->loadFromFile(RESOURCE_PATH("../resources/volumes/heart/colormap_bone.png"));
    if (!fileLoaded) {
        #if defined(_MSVC)
        fileLoaded = boneLUT->loadFromFile("../../../bin/resources/volumes/heart/colormap_bone.png");
        #endif
    }
    if (!fileLoaded)
    {
        cout << "Error - Failed to load colormap." << endl;
        close();
        return -1;
    }

    softLUT = cImage::create();
    fileLoaded = softLUT->loadFromFile(RESOURCE_PATH("../resources/volumes/heart/colormap_soft.png"));
    if (!fileLoaded) {
        #if defined(_MSVC)
        fileLoaded = softLUT->loadFromFile("../../../bin/resources/volumes/heart/colormap_soft.png");
        #endif
    }
    if (!fileLoaded)
    {
        cout << "Error - Failed to load colormap." << endl;
        close();
        return -1;
    }

    // tell the voxel object to load the colour look-up table as a texture
    object->m_colorMap->setImage(boneLUT);
    

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

    // create a small message
    labelMessage = new cLabel(font);
    labelMessage->m_fontColor.setBlack();
    labelMessage->setText("press keys [1,2] to toggle colormap and keys [4-9] to adjust slicing.");
    camera->m_frontLayer->addChild(labelMessage);

    // create a background
    cBackground* background = new cBackground();
    camera->m_backLayer->addChild(background);

    // set background properties
    background->setCornerColors(cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(0.7f, 0.7f, 0.7f),
                                cColorf(0.7f, 0.7f, 0.7f));


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

    // option 1: render bone and heart
    if (key == '1')
    {
        object->m_colorMap->setImage(boneLUT);
        object->setIsosurfaceValue(0.30);
        cout << "> Isosurface set to " <<  cStr(object->getIsosurfaceValue(), 3) << "       \r";
    }

    // option 2: render soft tissue
    if (key == '2')
    {
        object->m_colorMap->setImage(softLUT);
        object->setIsosurfaceValue(0.16);
        cout << "> Isosurface set to " <<  cStr(object->getIsosurfaceValue(), 3) << "       \r";
    }

    // option 4: reduce size along X axis
    if (key == '4')
    {
        double value = cClamp((object->m_maxCorner.x() - 0.005), 0.01, 0.5);
        object->m_maxCorner.x(value);
        object->m_minCorner.x(-value);
        object->m_maxTextureCoord.x(0.5+value);
        object->m_minTextureCoord.x(0.5-value);
        cout << "> Reduce size along X axis.            \r";
    }

    // option 5: increase size along X axis
    if (key == '5')
    {
        double value = cClamp((object->m_maxCorner.x() + 0.005), 0.01, 0.5);
        object->m_maxCorner.x(value);
        object->m_minCorner.x(-value);
        object->m_maxTextureCoord.x(0.5+value);
        object->m_minTextureCoord.x(0.5-value);
        cout << "> Increase size along X axis.          \r";
    }

    // option 6: reduce size along Y axis
    if (key == '6')
    {
        double value = cClamp((object->m_maxCorner.y() - 0.005), 0.01, 0.5);
        object->m_maxCorner.y(value);
        object->m_minCorner.y(-value);
        object->m_maxTextureCoord.y(0.5+value);
        object->m_minTextureCoord.y(0.5-value);
        cout << "> Reduce size along Y axis.           \r";
    }

    // option 7: increase size along Y axis
    if (key == '7')
    {
        double value = cClamp((object->m_maxCorner.y() + 0.005), 0.01, 0.5);
        object->m_maxCorner.y(value);
        object->m_minCorner.y(-value);
        object->m_maxTextureCoord.y(0.5+value);
        object->m_minTextureCoord.y(0.5-value);
        cout << "> Increase size along Y axis.          \r";
    }

    // option 8: reduce size along Z axis
    if (key == '8')
    {
        double value = cClamp((object->m_maxCorner.z() - 0.005), 0.01, 0.5);
        object->m_maxCorner.z(value);
        object->m_minCorner.z(-value);
        object->m_maxTextureCoord.z(0.5+value);
        object->m_minTextureCoord.z(0.5-value);
        cout << "> Reduce size along Z axis.           \r";
    }

    // option 9: increase size along Z axis
    if (key == '9')
    {
        double value = cClamp((object->m_maxCorner.z() + 0.005), 0.01, 0.5);
        object->m_maxCorner.z(value);
        object->m_minCorner.z(-value);
        object->m_maxTextureCoord.z(0.5+value);
        object->m_minTextureCoord.z(0.5-value);
        cout << "> Increase size along Z axis.         \r";
    }
    // option q: decrease quality of graphic rendering
    if (key == 'q')
    {
        double value = object->getQuality();
        object->setQuality(value - 0.1);
        cout << "> Quality set to " << cStr(object->getQuality(), 1) << "        \r";
    }

    // option w: increase quality of graphic rendering
    if (key == 'w')
    {
        double value = object->getQuality();
        object->setQuality(value + 0.1);
        cout << "> Quality set to " << cStr(object->getQuality(), 1) << "        \r";
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
    mouseX = x;
    mouseY = y;
}

//------------------------------------------------------------------------------

void mouseMove(int x, int y)
{
    // compute mouse motion
    int dx = x - mouseX;
    int dy = y - mouseY;
    mouseX = x;
    mouseY = y;

    // compute ne camera angles
    double azimuthDeg = camera->getSphericalAzimuthDeg() + (0.5 * dy);
    double polarDeg = camera->getSphericalPolarDeg() + (-0.5 * dx);

    // assign new angles
    camera->setSphericalAzimuthDeg(azimuthDeg);
    camera->setSphericalPolarDeg(polarDeg);

    // line up tool with camera
    tool->setLocalRot(camera->getLocalRot());
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

    // update position of message label
    labelMessage->setLocalPos((int)(0.5 * (windowW - labelMessage->getWidth())), 40);


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

enum cMode
{
    IDLE,
    SELECTION
};


void updateHaptics(void)
{
    cMode state = IDLE;
    cGenericObject* selectedObject = NULL;
    cTransform tool_T_object;

    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    // haptic force activation
    bool flagStart = true;
    int counter = 0;

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

        // check if device remains stuck inside voxel object
        cVector3d force = tool->getDeviceGlobalForce();
        if (flagStart)
        {
            if (force.length() != 0.0)
            {
                tool->initialize();
                counter = 0;
            }
            else
            {
                counter++;
                if (counter > 10)
                    flagStart = false;
            }
        }
        else
        {
            if (force.length() > 10.0)
            {
                flagStart = true;
            }
        }


        /////////////////////////////////////////////////////////////////////////
        // MANIPULATION
        /////////////////////////////////////////////////////////////////////////

        // compute transformation from world to tool (haptic device)
        cTransform world_T_tool = tool->getDeviceGlobalTransform();

        // get status of user switch
        bool button = tool->getUserSwitch(0);

        //
        // STATE 1:
        // Idle mode - user presses the user switch
        //
        if ((state == IDLE) && (button == true))
        {
            // check if at least one contact has occurred
            if (tool->m_hapticPoint->getNumCollisionEvents() > 0)
            {
                // get contact event
                cCollisionEvent* collisionEvent = tool->m_hapticPoint->getCollisionEvent(0);

                // get object from contact event
                selectedObject = collisionEvent->m_object;
            }
            else
            {
                selectedObject = object;
            }

            // get transformation from object
            cTransform world_T_object = selectedObject->getGlobalTransform();

            // compute inverse transformation from contact point to object 
            cTransform tool_T_world = world_T_tool;
            tool_T_world.invert();

            // store current transformation tool
            tool_T_object = tool_T_world * world_T_object;

            // update state
            state = SELECTION;
        }


        //
        // STATE 2:
        // Selection mode - operator maintains user switch enabled and moves object
        //
        else if ((state == SELECTION) && (button == true))
        {
            // compute new transformation of object in global coordinates
            cTransform world_T_object = world_T_tool * tool_T_object;

            // compute new transformation of object in local coordinates
            cTransform parent_T_world = selectedObject->getParent()->getLocalTransform();
            parent_T_world.invert();
            cTransform parent_T_object = parent_T_world * world_T_object;

            // assign new local transformation to object
            selectedObject->setLocalTransform(parent_T_object);

            // set zero forces when manipulating objects
            tool->setDeviceGlobalForce(0.0, 0.0, 0.0);

            tool->initialize();
        }

        //
        // STATE 3:
        // Finalize Selection mode - operator releases user switch.
        //
        else
        {
            state = IDLE;
        }


        /////////////////////////////////////////////////////////////////////////
        // FINALIZE
        /////////////////////////////////////////////////////////////////////////

        // send forces to haptic device
        tool->applyToDevice();  
    }
    
    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------
