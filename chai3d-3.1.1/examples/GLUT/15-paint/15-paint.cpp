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

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

// a mesh object to model a color palette
cMesh* palette;

// a mesh object to model a piece of canvas
cMesh* canvas;

// copy of blank canvas texture
cImagePtr canvasOriginal;

// selected paint color
cColorb paintColor;

// a label to explain what is happening
cLabel* labelMessage;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelHapticRate;

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
    DEMO:    15-paint.cpp

    This example models a virtual paint brush and allows the operator to select
    a color by touching the color palette, and paint the empty canvas.
    The amount of paint  released is function of the contact force magnitude.
    Finally the image can be saved to file.
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
    cout << "Demo: 15-paint" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[c] - Clear canvas" << endl;
    cout << "[s] - Save image to file as 'myPicture.jpg'" << endl;
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
    camera->set(cVector3d(0.8, 0.0, 0.0),    // camera position (eye)
                cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
                cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    // anything in front or behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 10.0);

    // set orthographic camera mode
    if (stereoMode == C_STEREO_DISABLED)
    {
        camera->setOrthographicView(1.3);
    }

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.01);
    camera->setStereoFocalLength(1.0);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // disable multi-pass rendering to handle transparent objects
    camera->setUseMultipassTransparency(true);

    // create a light source
    light = new cDirectionalLight(world);

    // add light to world
    world->addChild(light);

    // enable light source
    light->setEnabled(true);                   

    // define the direction of the light beam
    light->setDir(-1.0, 0.0,-0.4);             


    //--------------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    // create a tool (cursor) and insert into the world
    tool = new cToolCursor(world);
    world->addChild(tool);

    // connect the haptic device to the tool
    tool->setHapticDevice(hapticDevice);

    // define a radius for the tool
    double toolRadius = 0.01;

    // set tool radius
    tool->setRadius(toolRadius);

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(1.0);

    // haptic forces are enabled only if small forces are first sent to the device;
    // this mode avoids the force spike that occurs when the application starts when 
    // the tool is located inside an object for instance. 
    tool->setWaitForSmallForce(true);

    // start the haptic tool
    tool->start();


    //--------------------------------------------------------------------------
    // CREATE OBJECTS
    //--------------------------------------------------------------------------

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // maximum stiffness property
    double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;


    /////////////////////////////////////////////////////////////////////////
    // PALETTE: 
    /////////////////////////////////////////////////////////////////////////

    // create a mesh
    palette = new cMesh();

    // create a plane
    cCreatePlane(palette, 0.5, 0.5);

    // create collision detector
    palette->createBruteForceCollisionDetector();

    // add object to world
    world->addChild(palette);

    // set the position of the object
    palette->setLocalPos(-0.25, -0.3, 0.0);
    palette->rotateAboutGlobalAxisDeg(cVector3d(0,1,0), 90);
    palette->rotateAboutGlobalAxisRad(cVector3d(1,0,0), cDegToRad(90));

    // create texture property
    cTexture2dPtr texture = cTexture2d::create();
    palette->setTexture(texture);

    // load texture from file
    bool fileload = palette->m_texture->loadFromFile(RESOURCE_PATH("../resources/images/palette.jpg"));
    if (!fileload)
    {
            #if defined(_MSVC)
            fileload = palette->m_texture->loadFromFile("../../../bin/resources/images/palette.jpg");
            #endif
    }
    if (!fileload)
    {
            cout << "Error - Texture image 'palette.jpg' failed to load correctly." << endl;
            close();
            return (-1);
    }

    // we disable lighting properties for palette
    palette->setUseMaterial(false);

    // convert palette image from RGB to RGBA
    palette->m_texture->m_image->convert(GL_RGBA);

    // we set the white color (0xff, 0xff, 0xff) of the palette image to transparent (0x00).
    palette->m_texture->m_image->setTransparentColor(0xff, 0xff, 0xff, 0x00);

    // enable mipmaps for adaptive texture size rendering
    palette->m_texture->setUseMipmaps(true);
    
    // enable transparency for this object
    palette->setUseTransparency(true);

    // enable texture mapping
    palette->setUseTexture(true);

    // set haptic properties
    palette->m_material->setStiffness(0.5 * maxStiffness);   
    palette->m_material->setStaticFriction(0.2);
    palette->m_material->setDynamicFriction(0.2);

    // initialize a default color for tool brush
    paintColor.setBlueRoyal();
    tool->m_hapticPoint->m_sphereProxy->m_material->setColor(paintColor);


    /////////////////////////////////////////////////////////////////////////
    // CANVAS:
    ////////////////////////////////////////////////////////////////////////

    // create a mesh
    canvas = new cMesh();

    // create a plane
    cCreatePlane(canvas, 0.5, 0.5);

    // create collision detector
    canvas->createBruteForceCollisionDetector();

    // add object to world
    world->addChild(canvas);

    // set the position of the object
    canvas->setLocalPos(-0.25, 0.3, 0.0);
    canvas->rotateAboutGlobalAxisRad(cVector3d(0,1,0), cDegToRad(90));
    canvas->rotateAboutGlobalAxisRad(cVector3d(1,0,0), cDegToRad(90));

    // set graphic properties
    canvas->m_texture = cTexture2d::create();
    fileload = canvas->m_texture->loadFromFile(RESOURCE_PATH("../resources/images/canvas.jpg"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = canvas->m_texture->loadFromFile("../../../bin/resources/images/canvas.jpg");
        #endif
    }
    if (!fileload)
    {
        cout << "Error - Texture image failed to load correctly." << endl;
        close();
        return (-1);
    }

    // create a copy of canvas so that we can clear page when requested
    canvasOriginal = canvas->m_texture->m_image->copy();

    // we disable lighting properties for canvas
    canvas->setUseMaterial(false);

    // enable texture mapping
    canvas->setUseTexture(true);

    // set haptic properties
    canvas->m_material->setStiffness(0.5 * maxStiffness);   
    canvas->m_material->setStaticFriction(0.20);
    canvas->m_material->setDynamicFriction(0.15);
    canvas->m_material->setHapticTriangleSides(true, false);


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFont *font = NEW_CFONTCALIBRI20();
    
    // create a label to display the haptic rate of the simulation
    labelHapticRate = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticRate);

    // set font color
    labelHapticRate->m_fontColor.setGrayLevel(0.4);

    // create a label with a small message
    labelMessage = new cLabel(font);
    camera->m_frontLayer->addChild(labelMessage);

    // set font color
    labelMessage->m_fontColor.setGrayLevel(0.4);

    // set text message
    labelMessage->setText("select a color from the palette (left), and paint on the canvas (right)");


    // create a background
    cBackground* background = new cBackground();
    camera->m_backLayer->addChild(background);

    // set background properties
    background->setCornerColors(cColorf(1.00f, 1.00f, 1.00f),
                                cColorf(1.00f, 1.00f, 1.00f),
                                cColorf(0.80f, 0.80f, 0.80f),
                                cColorf(0.80f, 0.80f, 0.80f));


    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    simulationFinished = false;
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

    // option c: clear canvas
    if (key == 'c')
    {
        // copy original image of canvas to texture
        canvasOriginal->copyTo(canvas->m_texture->m_image);

        // update texture
        canvas->m_texture->markForUpdate();

        // update console message
        cout << "> Canvas has been erased.            \r";
    }

    // option s: save canvas to file
    if (key == 's')
    {
        // save current texture image to file
        canvas->m_texture->m_image->convert(GL_RGBA);
        canvas->m_texture->m_image->saveToFile("myPicture.bmp");
        canvas->m_texture->m_image->saveToFile("myPicture.jpg");
        canvas->m_texture->m_image->saveToFile("myPicture.png");
        canvas->m_texture->m_image->saveToFile("myPicture.ppm");
        canvas->m_texture->m_image->saveToFile("myPicture.raw");
        canvas->m_texture->m_image->saveToFile("myPicture.gif");

        // update console message
        cout << "> Canvas has been saved to file.     \r";
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

    // update position of haptic rate label
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


        /////////////////////////////////////////////////////////////////////
        // HAPTIC FORCE COMPUTATION
        /////////////////////////////////////////////////////////////////////

        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // update position and orientation of tool
        tool->updateFromDevice();

        // compute interaction forces
        tool->computeInteractionForces();

        // get interaction forces magnitude
        double force = tool->getDeviceGlobalForce().length();

        // send forces to haptic device
        tool->applyToDevice();


        /////////////////////////////////////////////////////////////////////
        // INTERACTION WITH PALETTE
        /////////////////////////////////////////////////////////////////////

        if (tool->isInContact(palette))
        {
            cCollisionEvent* contact = tool->m_hapticPoint->getCollisionEvent(0);
            if (contact != NULL)
            {
                // retrieve contact information
                cVector3d localPos = contact->m_localPos;
                unsigned int triangleIndex = contact->m_index;
                cTriangleArrayPtr triangles = contact->m_triangles;

                // retrieve texture coordinate
                cVector3d texCoord = triangles->getTexCoordAtPosition(triangleIndex, localPos);

                // retrieve pixel information
                int px, py;
                palette->m_texture->m_image->getPixelLocation(texCoord, px, py);

                // retrieve color information at pixel
                palette->m_texture->m_image->getPixelColor(px, py, paintColor);

                // update color of tool
                tool->m_hapticPoint->m_sphereProxy->m_material->setColor(paintColor);
            }
        }


        /////////////////////////////////////////////////////////////////////
        // INTERACTION WITH CANVAS
        /////////////////////////////////////////////////////////////////////

        if (tool->isInContact(canvas))
        {
            cCollisionEvent* contact = tool->m_hapticPoint->getCollisionEvent(0);
            if (contact != NULL)
            {
                // retrieve contact information
                cVector3d localPos = contact->m_localPos;
                unsigned int triangleIndex = contact->m_index;
                cTriangleArrayPtr triangles = contact->m_triangles;

                // retrieve texture coordinate
                cVector3d texCoord = triangles->getTexCoordAtPosition(triangleIndex, localPos);

                // retrieve pixel information
                int px, py;
                canvas->m_texture->m_image->getPixelLocation(texCoord, px, py);

                // paint color at tool position
                const double K_INK = 30;
                const double K_SIZE = 10;
                const int BRUSH_SIZE = 25;

                double size = cClamp((K_SIZE * force), 0.0, (double)(BRUSH_SIZE));
                for (int x=-BRUSH_SIZE; x<BRUSH_SIZE; x++)
                {
                    for (int y=-BRUSH_SIZE; y<BRUSH_SIZE; y++)
                    {                        
                        // compute new color percentage
                        double distance = sqrt((double)(x*x+y*y));
                        if (distance <= size)
                        {
                            // get color at location
                            cColorb color, newColor;
                            canvas->m_texture->m_image->getPixelColor(px+x, py+y, color);
                            
                            // compute color factor based of pixel position and force interaction 
                            double factor = cClamp(K_INK * timeInterval * cClamp(force, 0.0, 10.0) * cClamp(1 - distance/size, 0.0, 1.0), 0.0, 1.0);

                            // compute new color
                            newColor.setR((1.0 - factor) * color.getR() + factor * paintColor.getR());
                            newColor.setG((1.0 - factor) * color.getG() + factor * paintColor.getG());
                            newColor.setB((1.0 - factor) * color.getB() + factor * paintColor.getB());

                            // assign new color to pixel
                            canvas->m_texture->m_image->setPixelColor(px+x, py+y, newColor);
                        }              
                    }
                }

                // update texture
                canvas->m_texture->markForUpdate();
            }
        }

        // update frequency counter
        frequencyCounter.signal(1);
    }
    
    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------
