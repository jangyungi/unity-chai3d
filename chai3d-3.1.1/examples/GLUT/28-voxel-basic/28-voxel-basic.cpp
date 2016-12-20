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

// a virtual torus like object
cVoxelObject* object;

// 3D image data
cMultiImagePtr image;

// Resolution of voxel model
int voxelModelResolution = 64;

// Mutex to object
cMutex mutexObject;

// Mutex to voxel
cMutex mutexVoxel;

// 3D texture object
cTexture3dPtr texture;

// Region of voxels being updated
cCollisionAABBBox volumeUpdate;

// Flag that indicates if model is slowly rotationg
bool flagModelRotating = true;

// Flag that indicates that voxels have been updated
bool flagMarkVolumeForUpdate = false;

// Use cube or spheroid model
bool flagModelCube = false;

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

// callback to capture special keys for zoom in/out
void specialKey(int key, int x, int y);

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

// build voxel shape
void buildVoxelShape(double a_radiusSphere, double a_radiusCylinder);

// build voxel cube
void buildVoxelCube();


//==============================================================================
/*
    DEMO:    28-voxel-basic.cpp

    This demonstration illustrates the use of a 3D voxels.
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
    cout << "Demo: 28-voxel-basic" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[1] - Standard rendering (no shaders)" << endl;
    cout << "[2] - Basic voxel rendering" << endl;
    cout << "[3] - Isosurface with material" << endl;
    cout << "[4] - Isosurface with colors" << endl << endl;
    cout << "[7] - Resolution 32x32" << endl;
    cout << "[8] - Resolution 64x64" << endl;
    cout << "[9] - Resolution 128x128" << endl << endl;
    cout << "[q,w] Adjust quality of graphic rendering" << endl;
    cout << "[+] - Increase voxel opacity (mode Basic voxel rendering only)" << endl;
    cout << "[-] - Decrease voxel opacity (mode Basic voxel rendering only)" << endl;
    cout << "[c] - Swap between Cube or Spheroid model" << endl;
    cout << "[s] - Save volume image to disk (.png)" << endl;
    cout << "[r] - Toggle object rotation" << endl;
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
    glutSpecialFunc(specialKey);
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

    camera->setSphericalDeg(2.3,    // spherical coordinate radius
                            30,     // spherical coordinate azimuth angle
                            10);    // spherical coordinate polar angle

    // set the near and far clipping planes of the camera
    // anything in front or behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.1, 10.0);

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

    // define the direction of the light beam
    light->setDir(-3.0,-0.5, 0.0);

    // set lighting conditions
    light->m_ambient.set(0.5, 0.5, 0.5);
    light->m_diffuse.set(0.8, 0.8, 0.8);
    light->m_specular.set(1.0, 1.0, 1.0);


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
    tool->setRadius(0.05);

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(1.0);

    // oriente tool with camera
    tool->setLocalRot(camera->getLocalRot());

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

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();
    
    // stiffness properties
    double maxStiffness	= hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;


    //--------------------------------------------------------------------------
    // CREATE VOXEL OBJECT
    //--------------------------------------------------------------------------

    // create a volumetric model
    object = new cVoxelObject();

    // add object to world
    world->addChild(object);
    
    // rotate object
    object->rotateAboutGlobalAxisDeg(0, 0, 1, 40);
    
    // set the dimensions by assigning the position of the min and max corners
    object->m_minCorner.set(-0.5,-0.5,-0.5);
    object->m_maxCorner.set( 0.5, 0.5, 0.5);

    // set the texture coordinate at each corner.
    object->m_minTextureCoord.set(0.0, 0.0, 0.0);
    object->m_maxTextureCoord.set(1.0, 1.0, 1.0);

    // set material color
    object->m_material->setOrangeCoral();

    // set stiffness property
    object->setStiffness(0.2 * maxStiffness);

    // show/hide boundary box
    object->setShowBoundaryBox(false);


    //--------------------------------------------------------------------------
    // CREATE VOXEL DATA
    //--------------------------------------------------------------------------

    // create multi image data structure
    image = cMultiImage::create();

    // allocate 3D image data
    image->allocate(voxelModelResolution, voxelModelResolution, voxelModelResolution, GL_RGBA);

    // create texture
    texture = cTexture3d::create();

    // assign texture to voxel object
    object->setTexture(texture);

    // assign volumetric image to texture
    texture->setImage(image);

    // draw some 3D volumetric object
    if (flagModelCube)
    {
        buildVoxelCube();
    }
    else
    {
        buildVoxelShape(0.5, 0.2);
    }

    // set default rendering mode
    object->setRenderingModeIsosurfaceMaterial();

    // set quality of graphic rendering
    object->setQuality(0.5);


    //--------------------------------------------------------------------------
    // SPECIAL SHADER (OPTIONAL)
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    /*
        The following section has been commented, but illustrates how one
        can load a custom shader to render a volumetric object.
    */
    //--------------------------------------------------------------------------

    /*
    // create vertex shader
    cShaderPtr vertexShader = cShader::create(C_VERTEX_SHADER);
    bool fileload = vertexShader->loadSourceFile("./resources/shaders/isosurface.vert");
    if (!fileload)
    {
#if defined(_MSVC)
        fileload = vertexShader->loadSourceFile("../../../bin/resources/shaders/isosurface.vert");
#endif
    }

    // create fragment shader
    cShaderPtr fragmentShader = cShader::create(C_FRAGMENT_SHADER);
    
    fileload = fragmentShader->loadSourceFile("./resources/shaders/isosurface.frag");
    if (!fileload)
    {
#if defined(_MSVC)
        fileload = fragmentShader->loadSourceFile("../../../bin/resources/shaders/isosurface.frag");
#endif
    }
    
    // create program shader
    cShaderProgramPtr voxelShader = cShaderProgram::create();
    
    // assign vertex shader to program shader
    voxelShader->attachShader(vertexShader);
    
    // assign fragment shader to program shader
    voxelShader->attachShader(fragmentShader);
    
    // assign program shader to object
    object->setShaderProgram(voxelShader);
    
    // link program shader
    voxelShader->linkProgram();

    // set custom rendering mode
    object->setRenderingModeCustom();
    */


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
    labelMessage->setText("press user switch to remove voxels (drilling)");
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

    // option 1: select basic rendering mode
    if (key == '1')
    {
        object->setRenderingModeBasic();
        cout << "> Standard rendering            \r";
    }

    // option 2: select shader based rendering mode
    if (key == '2')
    {
        object->setRenderingModeVoxelColors();
        cout << "> Shader rendering              \r";
    }

    // option 3: select isosurface rendering mode (material)
    if (key == '3')
    {
        object->setRenderingModeIsosurfaceMaterial();
        cout << "> Isosurface with material      \r";
    }

    // option 4: select isosurface rendering mode (voxel colors)
    if (key == '4')
    {
        object->setRenderingModeIsosurfaceColors();
        cout << "> Isosurface with colours       \r";
    }

    // option 7: low resolution model
    if (key == '7')
    {
        voxelModelResolution = 32;
        mutexObject.acquire();
        image->allocate(voxelModelResolution, voxelModelResolution, voxelModelResolution, GL_RGBA);
        mutexObject.release();
        texture->markForDeleteAndUpdate();
        if (flagModelCube)
        {
            buildVoxelCube();
        }
        else
        {
            buildVoxelShape(0.5, 0.2);
        }
        cout << "> Low resolution                \r";
    }

    // option 8: medium resolution model
    if (key == '8')
    {
        voxelModelResolution = 64;
        mutexObject.acquire();
        image->allocate(voxelModelResolution, voxelModelResolution, voxelModelResolution, GL_RGBA);
        mutexObject.release();
        texture->markForDeleteAndUpdate();
        if (flagModelCube)
        {
            buildVoxelCube();
        }
        else
        {
            buildVoxelShape(0.5, 0.2);
        }
        cout << "> Medium resolution             \r";
    }

    // option 9: high resolution model
    if (key == '9')
    {
        voxelModelResolution = 128;
        mutexObject.acquire();
        image->allocate(voxelModelResolution, voxelModelResolution, voxelModelResolution, GL_RGBA);
        mutexObject.release();
        texture->markForDeleteAndUpdate();
        if (flagModelCube)
        {
            buildVoxelCube();
        }
        else
        {
            buildVoxelShape(0.5, 0.2);
        }
        cout << "> High resolution               \r";
    }

    // option +: increase opacity
    if (key == '+')
    {
        float opacity = object->getVoxelOpacity();
        opacity = opacity + 0.01;
        object->setVoxelOpacity(opacity);
        cout << "> Increase opacity: " << cStr(object->getVoxelOpacity(), 2) << "               \r";
    }

    // option -: increase opacity
    if (key == '-')
    {
        float opacity = object->getVoxelOpacity();
        opacity = opacity - 0.01;
        object->setVoxelOpacity(opacity);
        cout << "> Decrease opacity   " << cStr(object->getVoxelOpacity(), 2) << "               \r";
    }

    // option c: toggle model between cube and spheroid
    if (key == 'c')
    {
        flagModelCube = !flagModelCube;

        if (flagModelCube)
        {
            buildVoxelCube();
            cout << "> Render Cube               \r";
        }
        else
        {
            buildVoxelShape(0.5, 0.2);
            cout << "> Render Spheroid           \r";
        }
    }

    // option s: save voxel object disk
    if (key == 's')
    {
        mutexObject.acquire();
        image->saveToFiles("volume", "png");
        mutexObject.release();
        cout << "> Volume image saved to disk    \r";
    }

    // option r: toggle model rotating
    if (key == 'r')
    {
        flagModelRotating = !flagModelRotating;
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

void specialKey(int key, int x, int y)
{
    double r = camera->getSphericalRadius();
    switch (key) {
    case GLUT_KEY_UP:
        r = cClamp(r - 0.1, 0.5, 3.0);
        camera->setSphericalRadius(r);
        break;
    case GLUT_KEY_DOWN:
        r = cClamp(r + 0.1, 0.5, 3.0);
        camera->setSphericalRadius(r);
        break;
    }
}

//------------------------------------------------------------------------------

void mouseClick(int button, int state, int x, int y)
{
    mouseX = x;
    mouseY = y;

    if (state == GLUT_DOWN)
    {
        cCollisionRecorder recorder;
        cCollisionSettings settings;

        bool hit = camera->selectWorld(x, (windowH - y), windowW, windowH, recorder, settings);
        if (hit)
        {
            // check if hit involves voxl object
            if (recorder.m_nearestCollision.m_object == object)
            {
                // get selected voxel
                int voxelX = recorder.m_nearestCollision.m_voxelIndexX;
                int voxelY = recorder.m_nearestCollision.m_voxelIndexY;
                int voxelZ = recorder.m_nearestCollision.m_voxelIndexZ;

                // set color to black
                cColorb color(0x00, 0x00, 0x00, 0x00);

                // set color to voxel
                object->m_texture->m_image->setVoxelColor(voxelX, voxelY, voxelZ, color);

                // update voxel data
                object->m_texture->markForUpdate();
            }
        }
    }
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

    // oriente tool with camera
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
    // VOLUME UPDATE
    /////////////////////////////////////////////////////////////////////

    // update region of voxels to be updated
    if (flagMarkVolumeForUpdate)
    {
        mutexVoxel.acquire();
        cVector3d min = volumeUpdate.m_min;
        cVector3d max = volumeUpdate.m_max;
        volumeUpdate.setEmpty();
        mutexVoxel.release();
        texture->markForPartialUpdate(min, max);
        flagMarkVolumeForUpdate = false;
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
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}

//------------------------------------------------------------------------------

void updateHaptics(void)
{
    // reset clock
    cPrecisionClock clock;
    clock.reset();

    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    // main haptic simulation loop
    while(simulationRunning)
    {
        /////////////////////////////////////////////////////////////////////
        // SIMULATION TIME
        /////////////////////////////////////////////////////////////////////

        // stop the simulation clock
        clock.stop();

        // read the time increment in seconds
        double timeInterval = clock.getCurrentTimeSeconds();

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

        // read user switch
        int userSwitches = tool->getUserSwitches();

        // acquire mutex
        if (mutexObject.tryAcquire())
        {
            // compute interaction forces
            tool->computeInteractionForces();

            // check if tool is in contact with voxel object
            if (tool->isInContact(object) && (userSwitches > 0))
            {
                // retrieve contact event
                cCollisionEvent* contact = tool->m_hapticPoint->getCollisionEvent(0);

                // update voxel color
                cColorb color(0x00, 0x00, 0x00, 0x00);
                object->m_texture->m_image->setVoxelColor(contact->m_voxelIndexX, contact->m_voxelIndexY, contact->m_voxelIndexZ, color);

                // mark voxel for update
                mutexVoxel.acquire();
                volumeUpdate.enclose(cVector3d(contact->m_voxelIndexX, contact->m_voxelIndexY, contact->m_voxelIndexZ));
                mutexVoxel.release();
                flagMarkVolumeForUpdate = true;   
            }

            // release mutex
            mutexObject.release();
        }

        // send forces to haptic device
        tool->applyToDevice();

        // rotate object
        if (flagModelRotating)
        {
            object->rotateAboutGlobalAxisDeg(0,0,1, 5 * timeInterval);
        }
    }
    
    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------

void buildVoxelShape(double a_radiusSphere, double a_radiusCylinder)
{
    mutexVoxel.acquire();

    // setup dimension of shape in voxel resolution
    double center = (double)voxelModelResolution / 2.0;
    double radiusSphere = (double)voxelModelResolution * a_radiusSphere;
    double radiusCylinder = (double)voxelModelResolution * a_radiusCylinder;
    double k = 255.0 / (double)voxelModelResolution;
    
    // draw voxels
    for (int z=0; z<voxelModelResolution; z++)
    {
        for (int y=0; y<voxelModelResolution; y++)
        {
            for (int x=0; x<voxelModelResolution; x++)
            {
                double px = (double)x;
                double py = (double)y;
                double pz = (double)z;

                // set translucent voxel
                cColorb color;
                color.set(0x00, 0x00, 0x00, 0x00);

                double distance = sqrt(cSqr(px-center) + cSqr(py-center) + cSqr(pz-center));
                if (distance < radiusSphere)
                {
                    double distanceAxisZ = sqrt(cSqr(px-center) + cSqr(py-center));
                    double distanceAxisY = sqrt(cSqr(px-center) + cSqr(pz-center));
                    double distanceAxisX = sqrt(cSqr(py-center) + cSqr(pz-center));
                
                    if ((distanceAxisZ > radiusCylinder) && (distanceAxisY > radiusCylinder) && (distanceAxisX > radiusCylinder))
                    {
                        // assign color to voxel
                        //color.set(0xff, 0xff, 0xff, 0xff);
                        color.set((GLubyte)(k * fabs(px-center)), (GLubyte)(k * fabs(py-center)), (GLubyte)(k * fabs(pz-center)), 0xff);
                    }
                }

                image->setVoxelColor(x, y, z, color);
            }
        }
    }

    texture->markForUpdate();

    mutexVoxel.release();
}

//------------------------------------------------------------------------------

void buildVoxelCube()
{
    mutexVoxel.acquire();

    // fill all voxels
    for (int z=0; z<voxelModelResolution; z++)
    {
        for (int y=0; y<voxelModelResolution; y++)
        {
            for (int x=0; x<voxelModelResolution; x++)
            {
                double r =       (double)x / (double)voxelModelResolution;
                double g = 1.0 - (double)y / (double)voxelModelResolution;
                double b =       (double)z / (double)voxelModelResolution;

                cColorb color;
                color.setf(r, g, b, 1.0);
                image->setVoxelColor(x, y, z, color);
            }
        }
    }

    texture->markForUpdate();

    mutexVoxel.release();
}

//------------------------------------------------------------------------------
