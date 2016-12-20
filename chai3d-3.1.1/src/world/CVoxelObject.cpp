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
    \version   3.1.1 $Rev: 1242 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "world/CVoxelObject.h"
//------------------------------------------------------------------------------
#include "display/CCamera.h"
#include "materials/CTexture3d.h"
#include "shaders/CShaderProgram.h"
#include "collisions/CCollisionAABB.h"
#include "resources/CShaderBasicVoxel-RGBA8.h"
#include "resources/CShaderBasicVoxel-LUT8.h"
#include "resources/CShaderIsosurfaceColor-L8.h"
#include "resources/CShaderIsosurfaceColor-LUT8.h"
#include "resources/CShaderIsosurfaceColor-RGBA8.h"
#include "resources/CShaderIsosurface-L8.h"
#include "resources/CShaderIsosurface-RGBA8.h"
#include "resources/CShaderDVR-LUT8.h"
//------------------------------------------------------------------------------
const int C_RENDERING_MODE_BASIC                        = 0;
const int C_RENDERING_MODE_VOXEL_LUT8                   = 1;
const int C_RENDERING_MODE_VOXEL_RGBA8                  = 2;
const int C_RENDERING_MODE_ISOSURFACE_MATERIAL_L8       = 3;
const int C_RENDERING_MODE_ISOSURFACE_MATERIAL_RGBA     = 4;
const int C_RENDERING_MODE_ISOSURFACE_COLOR_L8          = 5;
const int C_RENDERING_MODE_ISOSURFACE_LUT8              = 6;
const int C_RENDERING_MODE_ISOSURFACE_COLOR_RGBA8       = 7;
const int C_RENDERING_MODE_DVR_LUT8                     = 8;
const int C_RENDERING_MODE_CUSTOM                       = 9;
//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cVoxelObject.
*/
//==============================================================================
cVoxelObject::cVoxelObject()
{
    // set (basic rendering mode without shaders
    setRenderingModeBasic();

    // shaders have not yet been initialized
    m_flagShadersInitialized = false;

    // set rendering quality
    setQuality(0.5);

    // set position of vertex min and max corners
    m_minCorner.set(-0.5,-0.5,-0.5);
    m_maxCorner.set( 0.5, 0.5, 0.5);

    // set texture coordinates for min and max cordners
    m_minTextureCoord.set(0.0, 0.0, 0.0);
    m_maxTextureCoord.set(1.0, 1.0, 1.0);

    // set default material property (disabled)
    m_material->setWhite();

    // disable material
    setUseMaterial(true);

    // disable vertex colors
    setUseVertexColors(false);

    // enable transparency 
    setTransparencyLevel(1.0f);
    setUseTransparency(true);

    // use linear interpollation
    m_useLinearInterpolation = false;

    // isosurface value
    m_isosurfaceValue = 0.1f;

    // voxel opacity value
    m_voxelOpacity = 1.0f;

    // set opacity threshold
    m_opacityThreshold = 0.96f;

    // set optical density factor
    m_opticalDensity = 1.0f;

    // create color map image
    m_colorMap = cTexture1d::create();
    m_colorMap->setTextureUnit(GL_TEXTURE2);
    m_colorMap->setWrapModeS(GL_CLAMP);

    // disable colormap
    m_useColorMap = false;

    // enable texture rendering
    setUseTexture(true);
    
    // render only front faces
    setUseCulling(true);
}


//==============================================================================
/*!
    Destructor of cVoxelObject.
*/
//==============================================================================
cVoxelObject::~cVoxelObject()
{
}


//==============================================================================
/*!
    This method sets the basic rendering mode without shader.
*/
//==============================================================================
void cVoxelObject::setRenderingModeBasic()
{
    // set internal rendering mode
    m_renderingMode = C_RENDERING_MODE_BASIC;

    // set interpolation mode
    setUseLinearInterpolation(false);
}


//==============================================================================
/*!
    This method sets a basic voxel renderer with shader.
*/
//==============================================================================
void cVoxelObject::setRenderingModeVoxelColors()
{
    // set internal rendering mode
    m_renderingMode = C_RENDERING_MODE_VOXEL_RGBA8;

    // set interpolation mode
    setUseLinearInterpolation(false);
}


//==============================================================================
/*!
    This method sets a basic voxel renderer with shader and colormap.
*/
//==============================================================================
void cVoxelObject::setRenderingModeVoxelColorMap()
{
    // set internal rendering mode
    m_renderingMode = C_RENDERING_MODE_VOXEL_LUT8;

    // set interpolation mode
    setUseLinearInterpolation(false);
}


//==============================================================================
/*!
    This method sets an isosurface renderer using shader and material color.
*/
//==============================================================================
void cVoxelObject::setRenderingModeIsosurfaceMaterial()
{
    if (m_texture != nullptr)
    {
        if (m_texture->m_image != nullptr)
        {
            if (m_texture->m_image->getFormat() == GL_LUMINANCE)
            {
                // set internal rendering mode
                m_renderingMode = C_RENDERING_MODE_ISOSURFACE_MATERIAL_L8;
            }
            else
            {
                // set internal rendering mode
                m_renderingMode = C_RENDERING_MODE_ISOSURFACE_MATERIAL_RGBA;
            }
        }
    }

    // set interpolation mode
    setUseLinearInterpolation(true);
}


//==============================================================================
/*!
    This method sets an isosurface renderer using a shader and individual
    voxel colors.
*/
//==============================================================================
void cVoxelObject::setRenderingModeIsosurfaceColors()
{
    if (m_texture != nullptr)
    {
        if (m_texture->m_image != nullptr)
        {
            if (m_texture->m_image->getFormat() == GL_LUMINANCE)
            {
                // set internal rendering mode
                m_renderingMode = C_RENDERING_MODE_ISOSURFACE_COLOR_L8;
            }
            else
            {
                // set internal rendering mode
                m_renderingMode = C_RENDERING_MODE_ISOSURFACE_COLOR_RGBA8;
            }
        }
    }

    // set interpolation mode
    setUseLinearInterpolation(true);
}


//==============================================================================
/*!
    This method sets an isosurface renderer using a shader and colormap.
*/
//==============================================================================
void cVoxelObject::setRenderingModeIsosurfaceColorMap()
{
    // enable colormap
    m_useColorMap = true;

    // set internal rendering mode
    m_renderingMode = C_RENDERING_MODE_ISOSURFACE_LUT8;

    // set interpolation mode
    setUseLinearInterpolation(true);
}


//==============================================================================
/*!
    This method sets a direct volume renderer with color map.
*/
//==============================================================================
void cVoxelObject::setRenderingModeDVRColorMap()
{
    // enable colormap
    m_useColorMap = true;

    // set internal rendering mode
    m_renderingMode = C_RENDERING_MODE_DVR_LUT8;

    // set interpolation mode
    setUseLinearInterpolation(true);
}


//==============================================================================
/*!
    This method sets a custom shader renderer.
*/
//==============================================================================
void cVoxelObject::setRenderingModeCustom()
{
    // set internal rendering mode
    m_renderingMode = C_RENDERING_MODE_CUSTOM;

    // set interpolation mode
    setUseLinearInterpolation(false);
}


//==============================================================================
/*!
    This method renders the object using OpenGL

    \param  a_options  Render options.
*/
//==============================================================================
void cVoxelObject::render(cRenderOptions& a_options)
{
#ifdef C_USE_OPENGL

    /////////////////////////////////////////////////////////////////////////
    // Update model
    /////////////////////////////////////////////////////////////////////////
    if (SECTION_RENDER_OPAQUE_PARTS_ONLY(a_options) && (!a_options.m_creating_shadow_map))
    {
        // update model
        update(a_options);
    }

    /////////////////////////////////////////////////////////////////////////
    // Render parts that use material properties
    /////////////////////////////////////////////////////////////////////////
    if (SECTION_RENDER_PARTS_WITH_MATERIALS(a_options, m_useTransparency))
    {
        if ((m_useColorMap) && (m_colorMap != nullptr))
        {
            m_colorMap->renderInitialize(a_options);
        }

        if (m_useLinearInterpolation)
        {
            if (m_texture != nullptr)
            {
                m_texture->setMinFunction(GL_LINEAR);
                m_texture->setMagFunction(GL_LINEAR);
            }
        }
        else
        {
            if (m_texture != nullptr)
            {
                m_texture->setMinFunction(GL_NEAREST);
                m_texture->setMagFunction(GL_NEAREST);
            }
        }

        // call front faces if volume rendering with a shader
        GLint cullMode;
        if (m_renderingMode > 0)
        {
            glGetIntegerv(GL_CULL_FACE_MODE, &cullMode);
            glCullFace(GL_FRONT);
        }

        // render the mesh (proxy geometry or slices, depending on mode)
        cMesh::render(a_options);

        // restore previous face culling mode
        if (m_renderingMode > 0)
            glCullFace(cullMode);

        if ((m_useColorMap) && (m_colorMap != nullptr))
        {
            m_colorMap->renderFinalize(a_options);
        }
    }

#endif
}


//==============================================================================
/*!
    This method updates the mesh model.
*/
//==============================================================================
void cVoxelObject::update(cRenderOptions& a_options)
{
    ////////////////////////////////////////////////////////////////////////////
    // SETUP COORDINATES OF VOXEL OBJECT (CORNERS)
    ////////////////////////////////////////////////////////////////////////////

    // clear all triangles and vertices
    clear();

    // sanity check
    if (m_texture == nullptr)
    {
        return;
    }
    
    if (m_texture->m_image == nullptr)
    {
        return;
    }

    // retrieve texture size
    int texSizeX = m_texture->m_image->getWidth();
    int texSizeY = m_texture->m_image->getHeight();
    int texSizeZ = m_texture->m_image->getImageCount();
    int texSizeMin = cMin(cMin(texSizeX, texSizeY), texSizeZ);

    // sanity check
    if (texSizeMin == 0)
    {
        return;
    }

    // compute center of model
    cVector3d centerLocal = 0.5 * (m_maxCorner + m_minCorner);
    cVector3d centerGlobal = getGlobalPos() + getGlobalRot() * centerLocal;

    // compute size of model
    cVector3d size = m_maxCorner - m_minCorner;
    double sizeX = fabs(m_maxCorner(0) - m_minCorner(0));
    double sizeY = fabs(m_maxCorner(1) - m_minCorner(1));
    double sizeZ = fabs(m_maxCorner(2) - m_minCorner(2));

    // compute resolution of model based on quality setting
    cVector3d tsize = m_maxTextureCoord - m_minTextureCoord;
    double tSizeX = tsize.x() * texSizeX;
    double tSizeY = tsize.y() * texSizeY;
    double tSizeZ = tsize.z() * texSizeZ;

    float resolution = (float)(m_quality * 4.0 * cMax(cMax(tSizeX, tSizeY), tSizeZ));

    // setup 8 vertices that characterize the volume
    cVector3d v000(m_minCorner(0), m_minCorner(1), m_minCorner(2));
    cVector3d v001(m_minCorner(0), m_minCorner(1), m_maxCorner(2));
    cVector3d v010(m_minCorner(0), m_maxCorner(1), m_minCorner(2));
    cVector3d v011(m_minCorner(0), m_maxCorner(1), m_maxCorner(2));
    cVector3d v100(m_maxCorner(0), m_minCorner(1), m_minCorner(2));
    cVector3d v101(m_maxCorner(0), m_minCorner(1), m_maxCorner(2));
    cVector3d v110(m_maxCorner(0), m_maxCorner(1), m_minCorner(2));
    cVector3d v111(m_maxCorner(0), m_maxCorner(1), m_maxCorner(2));

    // update boundary box of object
    m_boundaryBoxMin = v000;
    m_boundaryBoxMax = v111;

    // setup corresponding texture coordinates at each corner
    cVector3d t000(m_minTextureCoord(0), m_minTextureCoord(1), m_minTextureCoord(2));
    cVector3d t001(m_minTextureCoord(0), m_minTextureCoord(1), m_maxTextureCoord(2));
    cVector3d t010(m_minTextureCoord(0), m_maxTextureCoord(1), m_minTextureCoord(2));
    cVector3d t011(m_minTextureCoord(0), m_maxTextureCoord(1), m_maxTextureCoord(2));
    cVector3d t100(m_maxTextureCoord(0), m_minTextureCoord(1), m_minTextureCoord(2));
    cVector3d t101(m_maxTextureCoord(0), m_minTextureCoord(1), m_maxTextureCoord(2));
    cVector3d t110(m_maxTextureCoord(0), m_maxTextureCoord(1), m_minTextureCoord(2));
    cVector3d t111(m_maxTextureCoord(0), m_maxTextureCoord(1), m_maxTextureCoord(2));


    ////////////////////////////////////////////////////////////////////////////
    // SETUP RENDERING WITH SHADERS
    ////////////////////////////////////////////////////////////////////////////

    // if shader is present, use volume rendering instead of polygons
    if (m_renderingMode > 0)
    {
        // load shaders if not yet initialized
        if (!m_flagShadersInitialized)
        {
            loadRenderingShaders();
        }

        // assign program shader to object if rendering mode is among the selectable modes.
        if (m_renderingMode != C_NUM_VOXEL_RENDERING_MODES)
        {
            setShaderProgram(m_programShaders[m_renderingMode]);
        }

        // setup table of vertices and texture coordinates
        cVector3d v[8] = { v111, v110, v101, v100, v011, v010, v001, v000 };
        cVector3d t[8] = { t111, t110, t101, t100, t011, t010, t001, t000 };

        // setup table of normals
        const cVector3d norm[6] = {
            cVector3d( 1.0, 0.0, 0.0), cVector3d(-1.0, 0.0, 0.0),
            cVector3d( 0.0, 1.0, 0.0), cVector3d( 0.0,-1.0, 0.0),
            cVector3d( 0.0, 0.0, 1.0), cVector3d( 0.0, 0.0,-1.0)
        };

        // setup list of polygons
        const int poly[6][4] = {
            { 1, 0, 2, 3 }, { 4, 5, 7, 6 },
            { 0, 1, 5, 4 }, { 3, 2, 6, 7 },
            { 2, 0, 4, 6 }, { 1, 3, 7, 5 }
        };
        
        // construct the 6 faces (12 triangles) of the volume block
        for (int i = 0; i < 6; ++i)
        {
            const int *p = poly[i];
            const cVector3d &n = norm[i];
            newTriangle(v[p[0]], v[p[1]], v[p[2]], n, n, n,
                        t[p[0]], t[p[1]], t[p[2]]);
            newTriangle(v[p[2]], v[p[3]], v[p[0]], n, n, n,
                        t[p[2]], t[p[3]], t[p[0]]);
        }

        cVector3d tscale;
        for (int i = 0; i < 3; ++i)
        {
            tscale(i) = tsize(i) / size(i);
        }

        // compute idea gradient estimation delta
        cVector3d tdelta(1.0 / (double)(texSizeX),
                         1.0 / (double)(texSizeY),
                         1.0 / (double)(texSizeZ));

        // compute optical density factor
        float opticalDensityFactor = (float)(0.001 * m_opticalDensity * (resolution / size.length()));

        // update the shader uniforms with the corner values
        m_shaderProgram->setUniform("uMinCorner", m_minCorner);
        m_shaderProgram->setUniform("uMaxCorner", m_maxCorner);
        m_shaderProgram->setUniform("uTextureScale", tscale);
        m_shaderProgram->setUniformi("uVolume", 0);
        m_shaderProgram->setUniformi("uColorLUT", 2);
        m_shaderProgram->setUniform("uGradientDelta", tdelta);
        m_shaderProgram->setUniformf("uIsosurface", m_isosurfaceValue);
        m_shaderProgram->setUniformf("uOpacity", m_voxelOpacity);
        m_shaderProgram->setUniformf("uOpacityThreshold", m_opacityThreshold);
        m_shaderProgram->setUniformf("uOpticalDensityFactor", opticalDensityFactor);
        m_shaderProgram->setUniformf("uResolution", resolution);

        return;
    }


    ////////////////////////////////////////////////////////////////////////////
    // SETUP RENDERING WITHOUT SHADERS
    ////////////////////////////////////////////////////////////////////////////

    // disable shader program
    setShaderProgram(nullptr);

    // create 12 edges that characterize cube
    m_edges[0][0] = v000; m_edges[0][1] = v100;
    m_edges[1][0] = v001; m_edges[1][1] = v101;
    m_edges[2][0] = v010; m_edges[2][1] = v110;
    m_edges[3][0] = v011; m_edges[3][1] = v111;
    m_edges[4][0] = v000; m_edges[4][1] = v010;
    m_edges[5][0] = v001; m_edges[5][1] = v011;
    m_edges[6][0] = v100; m_edges[6][1] = v110;
    m_edges[7][0] = v101; m_edges[7][1] = v111;
    m_edges[8][0] = v000; m_edges[8][1] = v001;
    m_edges[9][0] = v010; m_edges[9][1] = v011;
    m_edges[10][0] = v100; m_edges[10][1] = v101;
    m_edges[11][0] = v110; m_edges[11][1] = v111;

    // compute distance between min and max corners
    double distanceMinMax = cDistance(v000, v111);

    // compute a starting point (center of object), a directional vector (object to camera) and a step
    cVector3d vObjectToCamera(1.0, 0.0, 0.0);
    cCamera* camera = a_options.m_camera;

    // define vector from voxel object center to camera
    vObjectToCamera = camera->getGlobalPos() - centerGlobal;

    // convert vector in local coordinates
    cTranspose(m_globalRot).mul(vObjectToCamera);

    // normalize vector
    vObjectToCamera.normalize();

    // surface normals
    cVector3d normal = vObjectToCamera;

    // compute start position
    cVector3d pStart = centerLocal - 0.5 * distanceMinMax * vObjectToCamera;
    cVector3d vDir = vObjectToCamera;
    double step = (1.0 / (double)resolution) * cDistance(v111, v000);

    // compute all triangles
    bool finished = false;
    cVector3d pos = pStart;
    double distance = 0;
    int layers = 0;

    // compute large intersection plane
    double s = 20 * distanceMinMax;
    cVector3d t0(0.0, 0.0, s);
    cVector3d t1(0.0, -0.86*s, -0.5*s);
    cVector3d t2(0.0, 0.86*s, -0.5*s);

    // rotate plane to point toward camera
    cMatrix3d rot;
    rot.identity();
    double angle = cAngle(rot.getCol0(), vDir);
    cVector3d axis = cCross(rot.getCol0(), vDir);
    if (axis.length() > 0.00001)
    {
        rot.rotateAboutGlobalAxisRad(cNormalize(axis), angle);
    }

    rot.mul(t0);
    rot.mul(t1);
    rot.mul(t2);

    while (!finished)
    {
        // compute a large triangle at that position
        cVector3d tri0 = t0+pos;
        cVector3d tri1 = t1+pos;
        cVector3d tri2 = t2+pos;

        // compute possible collisions between triangle and all edges
        cVector3d collisions[10];
        double collisionAngles[10];
        int numCollisions = 0;
        for (int i=0; i<12; i++)
        {
            double v01, v02;
            cVector3d collisionNormal;
            cVector3d collisionPoint;
            collisionPoint.zero();

            // check for collision
            bool collision = cIntersectionSegmentTriangle(m_edges[i][0], 
                m_edges[i][1],
                tri0, 
                tri1, 
                tri2,
                true,
                true,
                collisionPoint,
                collisionNormal,
                v01,
                v02);


            if (collision)
            {
                collisions[numCollisions] = collisionPoint;

                cVector3d up = tri0 - pos;
                cVector3d point = collisionPoint-pos;
                double ang = cRadToDeg(cAngle(up, point));

                cVector3d t = cCross(point, up);
                if (t.length() > 0.000001)
                {
                    double k = cRadToDeg(cAngle(t,vObjectToCamera));
                    if (k > 90)
                    {
                        ang = -ang;
                    }
                }

                collisionAngles[numCollisions] = ang;
                numCollisions++;
            }
        }

        // organize vertices in counter clock wise order
        int i,j;
        i = 0;
        while (i < numCollisions)
        {
            j = i+1;
            while (j < numCollisions)
            {
                if (collisionAngles[i] < collisionAngles[j])
                {
                    cSwap(collisionAngles[i], collisionAngles[j]);
                    cSwap(collisions[i], collisions[j]);
                }
                j++;
            }
            i++;
        }

        // create triangles
        if (numCollisions >= 3)
        {
            int i=2;
            while (i < numCollisions)
            {
                int v0 = newVertex(collisions[0]);
                int v1 = newVertex(collisions[i-1]);
                int v2 = newVertex(collisions[i]);

                cVector3d texCoord, pos, temp;

                // texcoord v0
                pos = m_vertices->getLocalPos(v0);

                texCoord(0) = (pos(0) - m_minCorner(0)) / (m_maxCorner(0) - m_minCorner(0)) * (m_maxTextureCoord(0) - m_minTextureCoord(0)) +  m_minTextureCoord(0);
                texCoord(1) = (pos(1) - m_minCorner(1)) / (m_maxCorner(1) - m_minCorner(1)) * (m_maxTextureCoord(1) - m_minTextureCoord(1)) +  m_minTextureCoord(1);
                texCoord(2) = (pos(2) - m_minCorner(2)) / (m_maxCorner(2) - m_minCorner(2)) * (m_maxTextureCoord(2) - m_minTextureCoord(2)) +  m_minTextureCoord(2);

                m_vertices->setTexCoord(v0, texCoord);
                m_vertices->setNormal(v0, normal);
                //m_vertices->setColor(v0, texCoord(0), texCoord(1), texCoord(2), 1.0);

                // texcoord v1
                pos = m_vertices->getLocalPos(v1);

                texCoord(0) = (pos(0) - m_minCorner(0)) / (m_maxCorner(0) - m_minCorner(0)) * (m_maxTextureCoord(0) - m_minTextureCoord(0)) +  m_minTextureCoord(0);
                texCoord(1) = (pos(1) - m_minCorner(1)) / (m_maxCorner(1) - m_minCorner(1)) * (m_maxTextureCoord(1) - m_minTextureCoord(1)) +  m_minTextureCoord(1);
                texCoord(2) = (pos(2) - m_minCorner(2)) / (m_maxCorner(2) - m_minCorner(2)) * (m_maxTextureCoord(2) - m_minTextureCoord(2)) +  m_minTextureCoord(2);
                m_vertices->setTexCoord(v1, texCoord);
                m_vertices->setNormal(v1, normal);
                //m_vertices->setColor(v1, texCoord(0), texCoord(1), texCoord(2), 1.0);

                // texcoord v1
                pos = m_vertices->getLocalPos(v2);
                texCoord(0) = (pos(0) - m_minCorner(0)) / (m_maxCorner(0) - m_minCorner(0)) * (m_maxTextureCoord(0) - m_minTextureCoord(0)) +  m_minTextureCoord(0);
                texCoord(1) = (pos(1) - m_minCorner(1)) / (m_maxCorner(1) - m_minCorner(1)) * (m_maxTextureCoord(1) - m_minTextureCoord(1)) +  m_minTextureCoord(1);
                texCoord(2) = (pos(2) - m_minCorner(2)) / (m_maxCorner(2) - m_minCorner(2)) * (m_maxTextureCoord(2) - m_minTextureCoord(2)) +  m_minTextureCoord(2);
                m_vertices->setTexCoord(v2, texCoord);
                m_vertices->setNormal(v2, normal);
                //m_vertices->setColor(v2, texCoord(0), texCoord(1), texCoord(2), 1.0);

                newTriangle(v0, v1, v2);
                i++;
            }
        }

        // increment position of given step
        layers++;
        distance = distance + step;
        pos = pStart + distance * vDir;

        // checked if finished
        if (distance > distanceMinMax) { finished = true; }
    }
}


//==============================================================================
/*!
    This method initializes all shader programs that are used to render the
    voxel object. Each mode has its own shader, except for mode 0, which uses 
    classic OpenGL rendering.
*/
//==============================================================================
void cVoxelObject::loadRenderingShaders()
{
    int mode;

    ////////////////////////////////////////////////////////////////////////////
    // MODE: NO SHADER
    ////////////////////////////////////////////////////////////////////////////

    // select mode
    mode = C_RENDERING_MODE_BASIC;

    // no shader
    m_vertexShaders[mode]   = nullptr;
    m_fragmentShaders[mode] = nullptr;
    m_programShaders[mode]  = nullptr;


    ////////////////////////////////////////////////////////////////////////////
    // MODE: SHADER BASIC VOXEL - RGBA8
    ////////////////////////////////////////////////////////////////////////////

    // select mode
    mode = C_RENDERING_MODE_VOXEL_RGBA8;

    // setup vertex shader
    m_vertexShaders[mode] = cShader::create(C_VERTEX_SHADER);
    m_vertexShaders[mode]->loadSourceCode(C_SHADER_BASIC_VOXEL_RGBA8_VERT);

    // setup fragment shader
    m_fragmentShaders[mode] = cShader::create(C_FRAGMENT_SHADER);
    m_fragmentShaders[mode]->loadSourceCode(C_SHADER_BASIC_VOXEL_RGBA8_FRAG);
    
    // setup program shader
    m_programShaders[mode] = cShaderProgram::create();
    m_programShaders[mode]->attachShader(m_vertexShaders[mode]);
    m_programShaders[mode]->attachShader(m_fragmentShaders[mode]);
    
    // link program shader
    m_programShaders[mode]->linkProgram();


    ////////////////////////////////////////////////////////////////////////////
    // MODE: SHADER BASIC VOXEL - LUT8
    ////////////////////////////////////////////////////////////////////////////

    // select mode
    mode = C_RENDERING_MODE_VOXEL_LUT8;

    // setup vertex shader
    m_vertexShaders[mode] = cShader::create(C_VERTEX_SHADER);
    m_vertexShaders[mode]->loadSourceCode(C_SHADER_BASIC_VOXEL_LUT8_VERT);

    // setup fragment shader
    m_fragmentShaders[mode] = cShader::create(C_FRAGMENT_SHADER);
    m_fragmentShaders[mode]->loadSourceCode(C_SHADER_BASIC_VOXEL_LUT8_FRAG);

    // setup program shader
    m_programShaders[mode] = cShaderProgram::create();
    m_programShaders[mode]->attachShader(m_vertexShaders[mode]);
    m_programShaders[mode]->attachShader(m_fragmentShaders[mode]);

    // link program shader
    m_programShaders[mode]->linkProgram();


    ////////////////////////////////////////////////////////////////////////////
    // MODE: SHADER ISOSURFACE MATERIAL - L8
    ////////////////////////////////////////////////////////////////////////////

    // select mode
    mode = C_RENDERING_MODE_ISOSURFACE_MATERIAL_L8;

    // setup vertex shader
    m_vertexShaders[mode] = cShader::create(C_VERTEX_SHADER);
    m_vertexShaders[mode]->loadSourceCode(C_SHADER_ISOSURFACE_L8_VERT);

    // setup fragment shader
    m_fragmentShaders[mode] = cShader::create(C_FRAGMENT_SHADER);
    m_fragmentShaders[mode]->loadSourceCode(C_SHADER_ISOSURFACE_L8_FRAG);

    // setup program shader
    m_programShaders[mode] = cShaderProgram::create();
    m_programShaders[mode]->attachShader(m_vertexShaders[mode]);
    m_programShaders[mode]->attachShader(m_fragmentShaders[mode]);

    // link program shader
    m_programShaders[mode]->linkProgram();


    ////////////////////////////////////////////////////////////////////////////
    // MODE: SHADER ISOSURFACE MATERIAL - RGBA8
    ////////////////////////////////////////////////////////////////////////////

    // select mode
    mode = C_RENDERING_MODE_ISOSURFACE_MATERIAL_RGBA;

    // setup vertex shader
    m_vertexShaders[mode] = cShader::create(C_VERTEX_SHADER);
    m_vertexShaders[mode]->loadSourceCode(C_SHADER_ISOSURFACE_RGBA8_VERT);

    // setup fragment shader
    m_fragmentShaders[mode] = cShader::create(C_FRAGMENT_SHADER);
    m_fragmentShaders[mode]->loadSourceCode(C_SHADER_ISOSURFACE_RGBA8_FRAG);

    // setup program shader
    m_programShaders[mode] = cShaderProgram::create();
    m_programShaders[mode]->attachShader(m_vertexShaders[mode]);
    m_programShaders[mode]->attachShader(m_fragmentShaders[mode]);

    // link program shader
    m_programShaders[mode]->linkProgram();


    ////////////////////////////////////////////////////////////////////////////
    // MODE: SHADER ISOSURFACE COLOR - L8
    ////////////////////////////////////////////////////////////////////////////

    // select mode
    mode = C_RENDERING_MODE_ISOSURFACE_COLOR_L8;

    // setup vertex shader
    m_vertexShaders[mode] = cShader::create(C_VERTEX_SHADER);
    m_vertexShaders[mode]->loadSourceCode(C_SHADER_ISOSURFACE_COLOR_L8_VERT);

    // setup fragment shader
    m_fragmentShaders[mode] = cShader::create(C_FRAGMENT_SHADER);
    m_fragmentShaders[mode]->loadSourceCode(C_SHADER_ISOSURFACE_COLOR_L8_FRAG);

    // setup program shader
    m_programShaders[mode] = cShaderProgram::create();
    m_programShaders[mode]->attachShader(m_vertexShaders[mode]);
    m_programShaders[mode]->attachShader(m_fragmentShaders[mode]);

    // link program shader
    m_programShaders[mode]->linkProgram();


    ////////////////////////////////////////////////////////////////////////////
    // MODE: SHADER ISOSURFACE COLOR - RGBA8
    ////////////////////////////////////////////////////////////////////////////

    // select mode
    mode = C_RENDERING_MODE_ISOSURFACE_COLOR_RGBA8;

    // setup vertex shader
    m_vertexShaders[mode] = cShader::create(C_VERTEX_SHADER);
    m_vertexShaders[mode]->loadSourceCode(C_SHADER_ISOSURFACE_COLOR_RGBA8_VERT);

    // setup fragment shader
    m_fragmentShaders[mode] = cShader::create(C_FRAGMENT_SHADER);
    m_fragmentShaders[mode]->loadSourceCode(C_SHADER_ISOSURFACE_COLOR_RGBA8_FRAG);

    // setup program shader
    m_programShaders[mode] = cShaderProgram::create();
    m_programShaders[mode]->attachShader(m_vertexShaders[mode]);
    m_programShaders[mode]->attachShader(m_fragmentShaders[mode]);

    // link program shader
    m_programShaders[mode]->linkProgram();


    ////////////////////////////////////////////////////////////////////////////
    // MODE: SHADER ISOSURFACE COLORMAP - LUT8
    ////////////////////////////////////////////////////////////////////////////

    // select mode
    mode = C_RENDERING_MODE_ISOSURFACE_LUT8;

    // setup vertex shader
    m_vertexShaders[mode] = cShader::create(C_VERTEX_SHADER);
    m_vertexShaders[mode]->loadSourceCode(C_SHADER_ISOSURFACE_COLOR_LUT8_VERT);

    // setup fragment shader
    m_fragmentShaders[mode] = cShader::create(C_FRAGMENT_SHADER);
    m_fragmentShaders[mode]->loadSourceCode(C_SHADER_ISOSURFACE_COLOR_LUT8_FRAG);

    // setup program shader
    m_programShaders[mode] = cShaderProgram::create();
    m_programShaders[mode]->attachShader(m_vertexShaders[mode]);
    m_programShaders[mode]->attachShader(m_fragmentShaders[mode]);

    // link program shader
    m_programShaders[mode]->linkProgram();


    ////////////////////////////////////////////////////////////////////////////
    // MODE: SHADER DIRECT VOLUME RENDERING (DVR) COLORMAP - LUT8
    ////////////////////////////////////////////////////////////////////////////

    // select mode
    mode = C_RENDERING_MODE_DVR_LUT8;

    // setup vertex shader
    m_vertexShaders[mode] = cShader::create(C_VERTEX_SHADER);
    m_vertexShaders[mode]->loadSourceCode(C_SHADER_DVR_LUT8_VERT);

    // setup fragment shader
    m_fragmentShaders[mode] = cShader::create(C_FRAGMENT_SHADER);
    m_fragmentShaders[mode]->loadSourceCode(C_SHADER_DVR_LUT8_FRAG);

    // setup program shader
    m_programShaders[mode] = cShaderProgram::create();
    m_programShaders[mode]->attachShader(m_vertexShaders[mode]);
    m_programShaders[mode]->attachShader(m_fragmentShaders[mode]);

    // link program shader
    m_programShaders[mode]->linkProgram();


    ////////////////////////////////////////////////////////////////////////////
    // FINALIZATION
    ////////////////////////////////////////////////////////////////////////////

    m_flagShadersInitialized = true;
}


//==============================================================================
/*!
    This method determines whether a given segment intersects this object or any
    of its descendants.\n
    The segment is described by a start point \p a_segmentPointA and end 
    point \p a_segmentPointB. \n
    All detected collisions are reported in the collision recorder passed 
    by argument \p a_recorder. \n
    Specifications about the type of collisions reported are specified by 
    argument \p a_settings.

    \param  a_segmentPointA  Start point of segment.
    \param  a_segmentPointB  End point of segment.
    \param  a_recorder       Recorder which stores all collision events.
    \param  a_settings       Collision settings information.

    \return __true__ if one or more collisions have occurred, __false__ otherwise.
*/
//==============================================================================
bool cVoxelObject::computeOtherCollisionDetection(cVector3d& a_segmentPointA,
                                                  cVector3d& a_segmentPointB,
                                                  cCollisionRecorder& a_recorder,
                                                  cCollisionSettings& a_settings)
{
    ////////////////////////////////////////////////////////////////////////////
    // COMPUTE INFORMATION ABOUT VOXEL OBJECT
    ////////////////////////////////////////////////////////////////////////////

    // get size of 3d texture
    double texSize[3];
    texSize[0] = (double)(m_texture->m_image->getWidth());
    texSize[1] = (double)(m_texture->m_image->getHeight());
    texSize[2] = (double)(m_texture->m_image->getImageCount());

    // sanity check
    if ((texSize[0] == 0) || (texSize[1] == 0) || (texSize[2] == 0))
    {
        return (false);
    }

    // compute size of texels along each axis
    double st[3];
    for (int i=0; i<3; i++)
    {
        if (m_maxTextureCoord(i) == m_minTextureCoord(i))
        {
            st[i] = fabs(m_maxCorner(i) - m_minCorner(i));
        }
        else
        {
            double s = fabs(m_maxCorner(i) - m_minCorner(i));
            st[i] = cMin(s, (s / ((m_maxTextureCoord(i) - m_minTextureCoord(i)) * texSize[i])));
        }
    }

    // compute smallest voxel size
    double voxelSize = cMin(st[0], cMin(st[1], st[2]));

    // sanity check
    if (voxelSize < C_SMALL) 
    { 
        return (false); 
    }


    ////////////////////////////////////////////////////////////////////////////
    // CHECK INTERSECTION WITH OBJECT BOUNDARY BOX
    ////////////////////////////////////////////////////////////////////////////

    // get tool radius
    double radius = a_settings.m_collisionRadius + 0.001 * voxelSize;

    // create bounding box for the collision line segment
    cCollisionAABBBox lineBox;
    lineBox.setEmpty();
    lineBox.enclose(a_segmentPointA);
    lineBox.enclose(a_segmentPointB);

    // create bounding box for entire volumetric object
    cCollisionAABBBox objectBox;
    objectBox.setEmpty();
    objectBox.enclose(m_minCorner - cVector3d(radius, radius, radius));
    objectBox.enclose(m_maxCorner + cVector3d(radius, radius, radius));

    // check for intersection between both boxes
    if (!objectBox.intersect(lineBox))
    {
        // the segment is located outside of the object
        return (false);
    }

    // compute half volume covered by collision sphere in texture space
    int texRadius[3];
    texRadius[0] = (int)ceil(radius / st[0]);
    texRadius[1] = (int)ceil(radius / st[1]);
    texRadius[2] = (int)ceil(radius / st[2]);

    // compute normalized vector from A to B
    cVector3d dir = a_segmentPointB - a_segmentPointA;
    if (dir.length() == 0.0)
    {
        return (false);
    }
    dir.normalize();


    ////////////////////////////////////////////////////////////////////////////
    // COMPUTE COLLISIONS
    ////////////////////////////////////////////////////////////////////////////
    int counter = 0;

    // compute collision radius; this also handle the case when the tool has radius 0.
    double r = cMax(radius + 0.5 * voxelSize, voxelSize);
    double r2 = cSqr(r);

    // compute range of object
    cVector3d objectRange = m_maxCorner - m_minCorner;

    // compute range of texture
    cVector3d texRange = m_maxTextureCoord - m_minTextureCoord;

    // compute min and max texels
    int minTexel[3];
    minTexel[0] = (int)(m_minTextureCoord(0) *  texSize[0]);
    minTexel[1] = (int)(m_minTextureCoord(1) *  texSize[1]);
    minTexel[2] = (int)(m_minTextureCoord(2) *  texSize[2]);

    int maxTexel[3];
    maxTexel[0] = (int)(m_maxTextureCoord(0) *  texSize[0]);
    maxTexel[1] = (int)(m_maxTextureCoord(1) *  texSize[1]);
    maxTexel[2] = (int)(m_maxTextureCoord(2) *  texSize[2]);

    // no collision has occurred yet
    bool hit = false;

    // temp variable to store collision data
    cVector3d collisionPoint;
    cVector3d collisionNormal;
    double collisionDistanceSq = C_LARGE;
    double collisionPointV01 = 0.0;
    double collisionPointV02 = 0.0;
    int voxelIndexX, voxelIndexY, voxelIndexZ;
    
    // compute distance between both point composing segment
    double distanceAB = cDistance(a_segmentPointB, a_segmentPointA);

    // distance counter
    double distance = 0.0;

    // search for collision
    while ((!hit) && (distance < distanceAB))
    {
        // increment step
        distance = cMin((distance + 2.0 * voxelSize), distanceAB);

        // compute next point
        cVector3d pointB = a_segmentPointA + distance * dir;

        // compute point in texels
        int tv1[3];
        tv1[0] =  (int)((m_minTextureCoord(0) + ((pointB(0) - m_minCorner(0)) / (objectRange(0))) * (texRange(0))) * texSize[0]);
        tv1[1] =  (int)((m_minTextureCoord(1) + ((pointB(1) - m_minCorner(1)) / (objectRange(1))) * (texRange(1))) * texSize[1]);
        tv1[2] =  (int)((m_minTextureCoord(2) + ((pointB(2) - m_minCorner(2)) / (objectRange(2))) * (texRange(2))) * texSize[2]);

        // check the area covered by the radius
        int tmin[3];
        tmin[0] = tv1[0] - texRadius[0] - 1;
        tmin[1] = tv1[1] - texRadius[1] - 1;
        tmin[2] = tv1[2] - texRadius[2] - 1;

        int tmax[3];
        tmax[0] = tv1[0] + texRadius[0] + 1;
        tmax[1] = tv1[1] + texRadius[1] + 1;
        tmax[2] = tv1[2] + texRadius[2] + 1;

        // check all voxels 
        for (int t2=tmin[2]; t2<tmax[2]; t2++)
        {
            if ((t2 >= minTexel[2]) && (t2 <= maxTexel[2]))
            for (int t1=tmin[1]; t1<tmax[1]; t1++)
            {
                if ((t1 >= minTexel[1]) && (t1 <= maxTexel[1]))
                for (int t0=tmin[0]; t0<tmax[0]; t0++)
                {
                    if ((t0 >= minTexel[0]) && (t0 <= maxTexel[0]))
                    {
                        // get  voxel color
                        cColorb color;
                        bool result = m_texture->m_image->getVoxelColor(t0, t1, t2, color);

                        if (result)
                        {
                            const float CONVERSION_FACTOR = (1.0f / 255.0f);
                            float level = CONVERSION_FACTOR * (float)(color.getA());

                            if (level >= m_isosurfaceValue)
                            {
                                // compute position of texel in local space
                                double tpos[3];
                                tpos[0] = m_minCorner(0) + (((t0 / texSize[0]) - m_minTextureCoord(0)) / (texRange(0))) * (objectRange(0)) + 0.5 * st[0];
                                tpos[1] = m_minCorner(1) + (((t1 / texSize[1]) - m_minTextureCoord(1)) / (texRange(1))) * (objectRange(1)) + 0.5 * st[1];
                                tpos[2] = m_minCorner(2) + (((t2 / texSize[2]) - m_minTextureCoord(2)) / (texRange(2))) * (objectRange(2)) + 0.5 * st[2];

                                // check if point is located outside of object
                                double distanceSq = cDistanceSq(cVector3d(tpos[0], tpos[1], tpos[2]), pointB);
                                if (distanceSq < r2)
                                {
                                    // check intersection with segment and voxel (approximated by sphere)
                                    cVector3d t_p, t_n;
                                    cVector3d t_collisionPoint, t_collisionNormal;
                                    double t_collisionDistanceSq;

                                    if (cIntersectionSegmentSphere(a_segmentPointA,
                                        pointB,
                                        cVector3d(tpos[0], tpos[1], tpos[2]),
                                        r,
                                        t_collisionPoint,
                                        t_collisionNormal,
                                        t_p,
                                        t_n) > 0)
                                    {
                                        // intersection occurred
                                        hit = true;

                                        counter++;

                                        // compute distance from collision point
                                        t_collisionDistanceSq = cDistanceSq(a_segmentPointA, t_collisionPoint);

                                        // if nearest, then select and store data.
                                        if (t_collisionDistanceSq <= collisionDistanceSq)
                                        {
                                            collisionPoint = t_collisionPoint;
                                            collisionNormal = t_collisionNormal;
                                            collisionDistanceSq = t_collisionDistanceSq;
                                            collisionPointV01 = 0.0;
                                            collisionPointV02 = 0.0;
                                            voxelIndexX = t0;
                                            voxelIndexY = t1;
                                            voxelIndexZ = t2;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // here we finally report the new collision to the collision event handler.
    if (hit)
    {
        // we verify if anew collision needs to be created or if we simply
        // need to update the nearest collision.
        if (a_settings.m_checkForNearestCollisionOnly)
        {
            // no new collision event is create. We just check if we need
            // to update the nearest collision
            if(collisionDistanceSq <= a_recorder.m_nearestCollision.m_squareDistance)
            {
                // report basic collision data
                a_recorder.m_nearestCollision.m_type = C_COL_VOXEL;
                a_recorder.m_nearestCollision.m_object = this;
                a_recorder.m_nearestCollision.m_voxelIndexX = voxelIndexX;
                a_recorder.m_nearestCollision.m_voxelIndexY = voxelIndexY;
                a_recorder.m_nearestCollision.m_voxelIndexZ = voxelIndexZ;
                a_recorder.m_nearestCollision.m_localPos = collisionPoint;
                a_recorder.m_nearestCollision.m_localNormal = collisionNormal;
                a_recorder.m_nearestCollision.m_squareDistance = collisionDistanceSq;
                a_recorder.m_nearestCollision.m_adjustedSegmentAPoint = a_segmentPointA;
                a_recorder.m_nearestCollision.m_posV01 = collisionPointV01;
                a_recorder.m_nearestCollision.m_posV02 = collisionPointV02;

                // report advanced collision data
                if (!a_settings.m_returnMinimalCollisionData)
                {
                    a_recorder.m_nearestCollision.m_globalPos = cAdd(getGlobalPos(),
                        cMul(getGlobalRot(),
                        a_recorder.m_nearestCollision.m_localPos));
                    a_recorder.m_nearestCollision.m_globalNormal = cMul(getGlobalRot(),
                        a_recorder.m_nearestCollision.m_localNormal);
                }
            }
        }
        else
        {
            cCollisionEvent newCollisionEvent;

            // report basic collision data
            newCollisionEvent.m_type = C_COL_VOXEL;
            newCollisionEvent.m_object = this;
            newCollisionEvent.m_triangles = nullptr;
            newCollisionEvent.m_voxelIndexX = voxelIndexX;
            newCollisionEvent.m_voxelIndexY = voxelIndexY;
            newCollisionEvent.m_voxelIndexZ = voxelIndexZ;
            newCollisionEvent.m_localPos = collisionPoint;
            newCollisionEvent.m_localNormal = collisionNormal;
            newCollisionEvent.m_squareDistance = collisionDistanceSq;
            newCollisionEvent.m_adjustedSegmentAPoint = a_segmentPointA;
            newCollisionEvent.m_posV01 = collisionPointV01;
            newCollisionEvent.m_posV02 = collisionPointV02;

            // report advanced collision data
            if (!a_settings.m_returnMinimalCollisionData)
            {
                newCollisionEvent.m_globalPos = cAdd(getGlobalPos(),
                    cMul(getGlobalRot(),
                    newCollisionEvent.m_localPos));
                newCollisionEvent.m_globalNormal = cMul(getGlobalRot(),
                    newCollisionEvent.m_localNormal);
            }

            // add new collision even to collision list
            a_recorder.m_collisions.push_back(newCollisionEvent);

            // check if this new collision is a candidate for "nearest one"
            if(collisionDistanceSq <= a_recorder.m_nearestCollision.m_squareDistance)
            {
                a_recorder.m_nearestCollision = newCollisionEvent;
            }
        }
    }

    // return result
    return (hit);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
