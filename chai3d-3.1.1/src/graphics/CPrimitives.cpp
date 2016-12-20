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
#include "graphics/CPrimitives.h"
//------------------------------------------------------------------------------
#include "graphics/CTriangleArray.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    This function creates a plane by defining its size along the X and Y axes. 
    Texture coordinates are defined so that the bitmap image maps the entire
    plane.

    \param  a_mesh     Mesh object in which primitive is created.
    \param  a_lengthX  Size along X axis.
    \param  a_lengthY  Size along Y axis.
    \param  a_pos      Position where to build the new primitive.
    \param  a_rot      Orientation of the new primitive.
    \param  a_color    Color of vertices.
*/
//==============================================================================
void cCreatePlane(cMesh* a_mesh, 
    const double& a_lengthX, 
    const double& a_lengthY, 
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_color)
{
    cCreatePlane2(a_mesh,
                  a_lengthX,
                  a_lengthY,
                  a_pos,
                  a_rot,
                  a_color,
                  a_color,
                  a_color,
                  a_color);
}


//==============================================================================
/*!
    This function creates a plane by defining its size along the X and Y axes.
    Texture coordinates are defined so that the bitmap image maps the entire 
    plane.

    \param  a_mesh              Mesh object in which primitive is created.
    \param  a_lengthX           Size along X axis.
    \param  a_lengthY           Size along Y axis.
    \param  a_pos               Position where to build the new primitive.
    \param  a_rot               Orientation of the new primitive.
    \param  a_colorTopLeft      Color of top left vertex.
    \param  a_colorTopRight     Color of top right vertex.
    \param  a_colorBottomLeft   Color of bottom left vertex.
    \param  a_colorBottomRight  Color of bottom right vertex.
*/
//==============================================================================
void cCreatePlane2(cMesh* a_mesh, 
    const double& a_lengthX, 
    const double& a_lengthY, 
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_colorTopLeft,
    const cColorf& a_colorTopRight,
    const cColorf& a_colorBottomLeft,
    const cColorf& a_colorBottomRight)
{
    // sanity check
    if (a_lengthX <= 0) { return; }
    if (a_lengthY <= 0) { return; }

    // compute half edges
    double half_length_X = 0.5 * a_lengthX;
    double half_length_Y = 0.5 * a_lengthY;;

    // compute position of vertices
    cVector3d v0 = cAdd(a_pos, cMul(a_rot, cVector3d(-half_length_X, -half_length_Y, 0.0)));
    cVector3d v1 = cAdd(a_pos, cMul(a_rot, cVector3d( half_length_X, -half_length_Y, 0.0)));
    cVector3d v2 = cAdd(a_pos, cMul(a_rot, cVector3d( half_length_X,  half_length_Y, 0.0)));
    cVector3d v3 = cAdd(a_pos, cMul(a_rot, cVector3d(-half_length_X,  half_length_Y, 0.0)));

    // compute surface normal
    cVector3d n = cMul(a_rot, cVector3d(0.0, 0.0, 1.0));

    // create new vertices
    int vertexIndex0 = a_mesh->newVertex(v0);
    int vertexIndex1 = a_mesh->newVertex(v1);
    int vertexIndex2 = a_mesh->newVertex(v2);
    int vertexIndex3 = a_mesh->newVertex(v3);

    // vertex (bottom left)
    a_mesh->m_vertices->setNormal(vertexIndex0, n);
    a_mesh->m_vertices->setTexCoord(vertexIndex0, 0.0, 0.0);
    a_mesh->m_vertices->setColor(vertexIndex0, a_colorBottomLeft);

    // vertex (bottom right)
    a_mesh->m_vertices->setNormal(vertexIndex1, n);
    a_mesh->m_vertices->setTexCoord(vertexIndex1, 1.0, 0.0);
    a_mesh->m_vertices->setColor(vertexIndex1, a_colorBottomRight);

    // vertex (top right)
    a_mesh->m_vertices->setNormal(vertexIndex2, n);
    a_mesh->m_vertices->setTexCoord(vertexIndex2, 1.0, 1.0);
    a_mesh->m_vertices->setColor(vertexIndex2, a_colorTopRight);

    // vertex (top left)
    a_mesh->m_vertices->setNormal(vertexIndex3, n);
    a_mesh->m_vertices->setTexCoord(vertexIndex3, 0.0, 1.0);
    a_mesh->m_vertices->setColor(vertexIndex3, a_colorTopLeft);

    // create triangles
    a_mesh->newTriangle(vertexIndex0, vertexIndex1, vertexIndex2);
    a_mesh->newTriangle(vertexIndex0, vertexIndex2, vertexIndex3);
}


//==============================================================================
/*!
    This function creates a 2D map by defining the size along the X and Y axes, 
    and the number of sides along each axis. For instance, a map containing 3 
    sides along the X axis and 4 sides along the Y axis will contains: 12 squares 
    composed each of two triangles. The total number of vertices will be equal 
    to 20, respectively (3+1)x(4+1). By modifying the Z component of each vertex 
    you can easily create height maps for instance.Texture coordinates are defined 
    so that the bitmap image covers the entire map.

    \param  a_mesh       Mesh object in which primitive is created.
    \param  a_lengthX    Size along X axis.
    \param  a_lengthY    Size along Y axis.
    \param  a_numSidesX  Number of elements along X axis.
    \param  a_numSidesY  Number of elements along Y axis.
    \param  a_pos        Position where to build the new primitive.
    \param  a_rot        Orientation of the new primitive.
    \param  a_color      Color of vertices.
*/
//==============================================================================
void cCreateMap(cMesh* a_mesh, 
    const double& a_lengthX, 
    const double& a_lengthY, 
    const unsigned int a_numSidesX,
    const unsigned int a_numSidesY,
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_color)
{
    // sanity check
    if ((a_numSidesX < 1) || (a_numSidesY < 1)) { return; }
    if (a_lengthX <= 0) { return; }
    if (a_lengthY <= 0) { return; }

    // compute half sides
    double half_length_X = a_lengthX / 2.0;
    double half_length_Y = a_lengthY / 2.0;

    // compute size of single element
    double size_X = a_lengthX / (double)a_numSidesX;
    double size_Y = a_lengthY / (double)a_numSidesY;

    // compute map normal
    cVector3d normal = cMul(a_rot, cVector3d(0.0, 0.0, 1.0));

    // create vertices
    int numVerticesX = a_numSidesX + 1;
    int numVerticesY = a_numSidesY + 1;
    for (int ix=0; ix<numVerticesX; ix++)
    {   
        double posX = -half_length_X + (double)(ix) * size_X;
        double textureCoordX = (double)(ix)/(double)(a_numSidesX);
        for (int iy=0; iy<numVerticesY; iy++)
        {   
            double posY = -half_length_Y + (double)(iy) * size_Y;
            cVector3d pos = cAdd(a_pos, cMul(a_rot, cVector3d(posX, posY, 0.0)));
            double textureCoordY = (double)(iy)/(double)(a_numSidesY);
            cVector3d textCoord(textureCoordX, textureCoordY, 0.0);
            a_mesh->newVertex(pos, normal, textCoord, a_color);
        }
    }

    // create triangles
    for (unsigned int ix=0; ix<a_numSidesX; ix++)
    {   
        for (unsigned int iy=0; iy<a_numSidesY; iy++)
        {   
            unsigned int index00 = (ix * numVerticesY) + iy;
            unsigned int index01 = (ix * numVerticesY) + (iy+1);
            unsigned int index10 = ((ix+1) * numVerticesY) + iy;
            unsigned int index11 = ((ix+1) * numVerticesY) + (iy+1);
            
            a_mesh->newTriangle(index00, index10, index11);
            a_mesh->newTriangle(index00, index11, index01);
        }
    }    
}



//==============================================================================
/*!
    This function creates a disk by defining its radius properties along axis X 
    and axis Y.

    \param  a_mesh       Mesh object in which primitive is created.
    \param  a_radiusX    Radius of sphere along axis X.
    \param  a_radiusY    Radius of sphere along axis Y.
    \param  a_numSlices  Specifies the number of slices composing the disc.
    \param  a_pos        Position where to build the new primitive.
    \param  a_rot        Orientation of the new primitive.
    \param  a_color      Color of vertices.
*/
//==============================================================================
void cCreateDisk(cMesh* a_mesh, 
    const double& a_radiusX, 
    const double& a_radiusY, 
    const unsigned int a_numSlices,
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_color)
{
    // sanity check
    if (a_numSlices < 3) { return; }
    if (a_radiusX <= 0) { return; }
    if (a_radiusY <= 0) { return; }

    // compute offset
    double delta_a = C_TWO_PI / (double)a_numSlices;

    // compute normal
    cVector3d normal = cMul(a_rot, cVector3d(0.0, 0.0, 1.0));

    // create first vertex
    cVector3d p = cAdd(a_pos, cMul(a_rot, cVector3d(0.0, 0.0, 0.0)));
    unsigned int index = a_mesh->newVertex(p);

    // set texture coordinate
    a_mesh->m_vertices->setTexCoord(index, cVector3d(0.5, 0.5, 0.0));

    // set normal
    a_mesh->m_vertices->setNormal(index, normal);

    // create first round of points
    double angle = 0.0;
    for (unsigned int i=0; i<a_numSlices; i++)
    {   
        // set position
        p = cAdd(a_pos, cMul(a_rot, cVector3d(a_radiusX * cos(angle), a_radiusY * sin(angle), 0.0)));
        int vertexID = a_mesh->newVertex(p);

        // set normal
        a_mesh->m_vertices->setNormal(vertexID, normal);

        // set texture coordinate
        a_mesh->m_vertices->setTexCoord(vertexID, cVector3d(0.5 + 0.5 * cos(angle), 0.5 + 0.5 * sin(angle), 0.0));

        angle = angle + delta_a;
    }

    // create triangles
    for (unsigned int i=1; i<a_numSlices; i++)
    {   
        unsigned int index0 = index;
        unsigned int index1 = index + i;
        unsigned int index2 = index + i + 1;
        a_mesh->newTriangle(index0, index1, index2);
    }
    a_mesh->newTriangle(index, index + a_numSlices, index + 1);
}


//==============================================================================
/*!
    This function creates a panel with optional rounded corners.

    \param  a_mesh                  Mesh object in which primitive is created.
    \param  a_lengthX               Size along X axis.
    \param  a_lengthY               Size along Y axis.
    \param  a_radiusCorners         Radius of corners.
    \param  a_numSegmentsPerCorner  Number of segments per rounded corner.
    \param  a_pos                   Position where to build the new primitive.
    \param  a_rot                   Orientation of the new primitive.
    \param  a_color                 Color of vertices.
*/
//==============================================================================
void cCreatePanel(cMesh* a_mesh, 
    const double& a_lengthX, 
    const double& a_lengthY, 
    const double& a_radiusCorners,
    const int& a_numSegmentsPerCorner,
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_color)
{
    cCreatePanel2(a_mesh, 
                  a_lengthX, 
                  a_lengthY, 
                  a_radiusCorners,
                  a_radiusCorners,
                  a_radiusCorners,
                  a_radiusCorners,
                  a_numSegmentsPerCorner,
                  a_pos,
                  a_rot,
                  a_color,
                  a_color,
                  a_color,
                  a_color);
}


//==============================================================================
/*!
    This function creates a panel with optional rounded corners.

    \param  a_mesh                     Mesh object in which primitive is created.
    \param  a_lengthX                  Size along X axis.
    \param  a_lengthY                  Size along Y axis.
    \param  a_cornerTopLeftRadius      Radius of top left corner.
    \param  a_cornerTopRightRadius     Radius of top right corner.
    \param  a_cornerBottomLeftRadius   Radius of bottom left corner.
    \param  a_cornerTopLeftRadius      Radius of bottom right corner.
    \param  a_cornerBottomRightRadius  Number of segments per rounded corner.
    \param  a_numSegmentsPerCorner     Number of segments composing the circular corners.
    \param  a_pos                      Position where to build the new primitive.
    \param  a_rot                      Orientation of the new primitive.
    \param  a_colorTopLeft             Color of top left vertex.
    \param  a_colorTopRight            Color of top right vertex.
    \param  a_colorBottomLeft          Color of bottom left vertex.
    \param  a_colorBottomRight         Color of bottom right vertex.
*/
//==============================================================================
void cCreatePanel2(cMesh* a_mesh, 
    const double& a_lengthX, 
    const double& a_lengthY, 
    const double& a_cornerTopLeftRadius,
    const double& a_cornerTopRightRadius,
    const double& a_cornerBottomLeftRadius,
    const double& a_cornerBottomRightRadius,
    const int& a_numSegmentsPerCorner,
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_colorTopLeft,
    const cColorf& a_colorTopRight,
    const cColorf& a_colorBottomLeft,
    const cColorf& a_colorBottomRight)
{
    // sanity check
    if (a_cornerTopLeftRadius < 0) { return; }
    if (a_cornerTopRightRadius < 0) { return; }
    if (a_cornerBottomLeftRadius < 0) { return; }
    if (a_cornerBottomRightRadius < 0) { return; }
    if (a_lengthX <= 0) { return; }
    if (a_lengthY <= 0) { return; }

    // sharp corners
    if ( (a_cornerTopLeftRadius == 0) &&
         (a_cornerTopRightRadius == 0) &&
         (a_cornerBottomLeftRadius == 0) &&
         (a_cornerBottomRightRadius == 0) )
    {
        cCreatePlane2(a_mesh,
                      a_lengthX,
                      a_lengthY,
                      a_pos,
                      a_rot,
                      a_colorTopLeft,
                      a_colorTopRight,
                      a_colorBottomLeft,
                      a_colorBottomRight);

        return;
    }

    // create a panel with corners
    double maxRadius = cMax(cMax(a_cornerTopLeftRadius, a_cornerTopRightRadius), 
                            cMax(a_cornerBottomLeftRadius, a_cornerBottomRightRadius));
    double minLength = cMin(a_lengthX, a_lengthY) / 2.0;
    maxRadius = cMin(maxRadius, minLength);

    // compute inner rectangle dimensions
    double hx0 = (a_lengthX / 2.0) - a_cornerTopRightRadius;
    double hy0 = (a_lengthY / 2.0) - a_cornerTopRightRadius;

    double hx1 = (a_lengthX / 2.0) - a_cornerTopLeftRadius;
    double hy1 = (a_lengthY / 2.0) - a_cornerTopLeftRadius;

    double hx2 = (a_lengthX / 2.0) - a_cornerBottomLeftRadius;
    double hy2 = (a_lengthY / 2.0) - a_cornerBottomLeftRadius;

    double hx3 = (a_lengthX / 2.0) - a_cornerBottomRightRadius;
    double hy3 = (a_lengthY / 2.0) - a_cornerBottomRightRadius;

    double sx = 0.5 / (a_lengthX / 2.0);
    double sy = 0.5 / (a_lengthY / 2.0);

    // compute surface normal
    cVector3d normal = cMul(a_rot, cVector3d(0.0, 0.0, 1.0));

    // setup variables
    int numVerticesPerCorner = a_numSegmentsPerCorner + 1;
    double deltaAng = C_PI_DIV_2 / (double(a_numSegmentsPerCorner));

    // create center vertex
    cVector3d pos = cAdd(a_pos, cMul(a_rot, cVector3d(0.0, 0.0, 0.0)));
    int vertexIndex0 = a_mesh->newVertex(pos);
    a_mesh->m_vertices->setNormal(vertexIndex0, normal);
    a_mesh->m_vertices->setTexCoord(vertexIndex0, 0.5, 0.5);

    // compute at center of panel
    cColorf color;
    color.setR(0.25f * (a_colorTopLeft.getR() + a_colorTopRight.getR() + a_colorBottomLeft.getR() + a_colorBottomRight.getR()));
    color.setG(0.25f * (a_colorTopLeft.getG() + a_colorTopRight.getG() + a_colorBottomLeft.getG() + a_colorBottomRight.getG()));
    color.setB(0.25f * (a_colorTopLeft.getB() + a_colorTopRight.getB() + a_colorBottomLeft.getB() + a_colorBottomRight.getB()));
    color.setA(0.25f * (a_colorTopLeft.getA() + a_colorTopRight.getA() + a_colorBottomLeft.getA() + a_colorBottomRight.getA()));
    a_mesh->m_vertices->setColor(vertexIndex0, color);

    // create vertices (top right corner)
    for (int i=0; i<numVerticesPerCorner; i++)
    {
        // compute position of vertex
        double angle = (double)(i) * deltaAng;
        double x = hx0 + a_cornerTopRightRadius * cos(angle);
        double y = hy0 + a_cornerTopRightRadius * sin(angle);
        cVector3d p = cAdd(a_pos, cMul(a_rot, cVector3d(x, y, 0.0)));

        // compute texture coordinate
        cVector3d texCoord(0.5 + sx*x, 0.5 + sy*y, 0.0);

        // create new vertex
        a_mesh->newVertex(p, normal, texCoord, a_colorTopRight);
    }

    // create vertices (top left corner)
    for (int i=0; i<numVerticesPerCorner; i++)
    {
        // compute position of vertex
        double angle = (1 * C_PI_DIV_2) + (double)(i) * deltaAng;
        double x =-hx1 + a_cornerTopLeftRadius * cos(angle);
        double y = hy1 + a_cornerTopLeftRadius * sin(angle);
        cVector3d p = cAdd(a_pos, cMul(a_rot, cVector3d(x, y, 0.0)));

        // compute texture coordinate
        cVector3d texCoord(0.5 + sx*x, 0.5 + sy*y, 0.0);

        // create new vertex
        a_mesh->newVertex(p, normal, texCoord, a_colorTopLeft);
    }

    // create vertices (bottom left corner)
    for (int i=0; i<numVerticesPerCorner; i++)
    {
        // compute position of vertex
        double angle = (2 * C_PI_DIV_2) + (double)(i) * deltaAng;
        double x =-hx2 + a_cornerBottomLeftRadius * cos(angle);
        double y =-hy2 + a_cornerBottomLeftRadius * sin(angle);
        cVector3d p = cAdd(a_pos, cMul(a_rot, cVector3d(x, y, 0.0)));

        // compute texture coordinate
        cVector3d texCoord(0.5 + sx*x, 0.5 + sy*y, 0.0);

        // create new vertex
        a_mesh->newVertex(p, normal, texCoord, a_colorBottomLeft);
    }

    // create vertices (bottom right corner)
    for (int i=0; i<numVerticesPerCorner; i++)
    {
        // compute position of vertex
        double angle = (3 * C_PI_DIV_2) + (double)(i) * deltaAng;
        double x = hx3 + a_cornerBottomRightRadius * cos(angle);
        double y =-hy3 + a_cornerBottomRightRadius * sin(angle);
        cVector3d p = cAdd(a_pos, cMul(a_rot, cVector3d(x, y, 0.0)));

        // compute texture coordinate
        cVector3d texCoord(0.5 + sx*x, 0.5 + sy*y, 0.0);

        // create new vertex
        a_mesh->newVertex(p, normal, texCoord, a_colorBottomRight);
    }

    // create triangles
    int numTriangles = 4 * numVerticesPerCorner;
    int v = vertexIndex0 + 1;
    for (int i=0; i<numTriangles; i++)
    {
        int vertexIndex1 = v + i;
        int vertexIndex2 = v + ((i+1) % (numTriangles));

        a_mesh->newTriangle(vertexIndex0, vertexIndex1, vertexIndex2);
    }
}


//==============================================================================
/*!
    This function creates a box by defining its size along the x, y and z axis.
    Texture coordinates are defined so that the bitmap image is displayed
    on each face of the box.

    \param  a_mesh     Mesh object in which primitive is created.
    \param  a_lengthX  Size along X axis.
    \param  a_lengthY  Size along Y axis.
    \param  a_lengthZ  Size along Z axis.
    \param  a_pos      Position where to build the new primitive.
    \param  a_rot      Orientation of the new primitive.
    \param  a_color    Color of vertices.
*/
//==============================================================================
void cCreateBox(cMesh* a_mesh, 
    const double& a_lengthX, 
    const double& a_lengthY, 
    const double& a_lengthZ,
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_color)
{
    // sanity check
    if (a_lengthX < 0) { return; }
    if (a_lengthY < 0) { return; }
    if (a_lengthZ < 0) { return; }

    // compute half edges
    double half_length_X = a_lengthX / 2.0;
    double half_length_Y = a_lengthY / 2.0;
    double half_length_Z = a_lengthZ / 2.0;

    // create texture coordinates
    cVector3d t00(0.0, 0.0, 0.0);
    cVector3d t10(1.0, 0.0, 0.0);
    cVector3d t01(0.0, 1.0, 0.0);
    cVector3d t11(1.0, 1.0, 0.0);

    // compute position of vertices
    cVector3d v000 = cAdd(a_pos, cMul(a_rot, cVector3d(-half_length_X, -half_length_Y, -half_length_Z)));
    cVector3d v100 = cAdd(a_pos, cMul(a_rot, cVector3d( half_length_X, -half_length_Y, -half_length_Z)));
    cVector3d v110 = cAdd(a_pos, cMul(a_rot, cVector3d( half_length_X,  half_length_Y, -half_length_Z)));
    cVector3d v010 = cAdd(a_pos, cMul(a_rot, cVector3d(-half_length_X,  half_length_Y, -half_length_Z)));
    cVector3d v001 = cAdd(a_pos, cMul(a_rot, cVector3d(-half_length_X, -half_length_Y,  half_length_Z)));
    cVector3d v101 = cAdd(a_pos, cMul(a_rot, cVector3d( half_length_X, -half_length_Y,  half_length_Z)));
    cVector3d v111 = cAdd(a_pos, cMul(a_rot, cVector3d( half_length_X,  half_length_Y,  half_length_Z)));
    cVector3d v011 = cAdd(a_pos, cMul(a_rot, cVector3d(-half_length_X,  half_length_Y,  half_length_Z)));

    // compute normals
    cVector3d nx0 = cMul(a_rot, cVector3d(-1.0, 0.0, 0.0));
    cVector3d nx1 = cMul(a_rot, cVector3d( 1.0, 0.0, 0.0));
    cVector3d ny0 = cMul(a_rot, cVector3d( 0.0,-1.0, 0.0));
    cVector3d ny1 = cMul(a_rot, cVector3d( 0.0, 1.0, 0.0));
    cVector3d nz0 = cMul(a_rot, cVector3d( 0.0, 0.0,-1.0));
    cVector3d nz1 = cMul(a_rot, cVector3d( 0.0, 0.0, 1.0));

    // create triangles
    a_mesh->newTriangle(v011, v010, v000, nx0, nx0, nx0, t11, t10, t00, a_color, a_color, a_color);
    a_mesh->newTriangle(v011, v000, v001, nx0, nx0, nx0, t11, t00, t01, a_color, a_color, a_color);
    a_mesh->newTriangle(v101, v100, v110, nx1, nx1, nx1, t01, t00, t10, a_color, a_color, a_color);
    a_mesh->newTriangle(v101, v110, v111, nx1, nx1, nx1, t01, t10, t11, a_color, a_color, a_color);

    a_mesh->newTriangle(v101, v001, v000, ny0, ny0, ny0, t11, t01, t00, a_color, a_color, a_color);
    a_mesh->newTriangle(v101, v000, v100, ny0, ny0, ny0, t11, t00, t10, a_color, a_color, a_color);
    a_mesh->newTriangle(v111, v110, v010, ny1, ny1, ny1, t11, t10, t00, a_color, a_color, a_color);
    a_mesh->newTriangle(v111, v010, v011, ny1, ny1, ny1, t11, t00, t01, a_color, a_color, a_color);

    a_mesh->newTriangle(v000, v010, v110, nz0, nz0, nz0, t00, t01, t11, a_color, a_color, a_color);
    a_mesh->newTriangle(v000, v110, v100, nz0, nz0, nz0, t00, t11, t10, a_color, a_color, a_color);
    a_mesh->newTriangle(v001, v101, v111, nz1, nz1, nz1, t00, t10, t11, a_color, a_color, a_color);
    a_mesh->newTriangle(v001, v111, v011, nz1, nz1, nz1, t00, t11, t01, a_color, a_color, a_color);
}


//==============================================================================
/*!
    This function creates a cylinder by defining its radius and height. 
    The user may  also decide if the top and bottom discs should be included. 
    Texture coordinates are defined so that the bitmap image wraps around 
    the cylinder. The texture coordinates for the top part of the cylinder 
    are set to (0.0, 0.0, 0.0). The texture coordinates for the bottom part of 
    the cylinder are set to (1.0, 1.0, 0.0). When texture is enabled, 
    the colors defining the top and bottoms sections of the cylinder 
    are defined by both texels.

    \param  a_mesh               Mesh object in which primitive is created.
    \param  a_height             Height of the cylinder.
    \param  a_radius             Radius of the cylinder.
    \param  a_numSides           Number of sides composing the cylinder.
    \param  a_numHeightSegments  Number of segments along the cylinder axis.
    \param  a_numRings           Number of rings of the top and bottom parts.
    \param  a_includeTop         If __true__, then the top disc is included.
    \param  a_includeBottom      If __true__, then the bottom disc is included.
    \param  a_pos                Position where to build the new primitive.
    \param  a_rot                Orientation of the new primitive.
    \param  a_color              Color of vertices.
*/
//==============================================================================
void cCreateCylinder(cMesh* a_mesh, 
    const double& a_height,  
    const double& a_radius,
    const unsigned int a_numSides,
    const unsigned int a_numHeightSegments,
    const unsigned int a_numRings,
    const bool a_includeTop,
    const bool a_includeBottom,
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_color)
{
    cCreateCone(a_mesh, 
                a_height,  
                a_radius,
                a_radius,
                a_numSides,
                a_numHeightSegments,
                a_numRings,
                a_includeBottom,
                a_includeTop,
                a_pos,
                a_rot,
                a_color);
}


//==============================================================================
/*!
    This function creates a cone by defining its height and bottom radius. By defining 
    a top radius larger than zero, it is possible to create a truncated
    cone. Top and bottom parts can also be included or not.

    \param  a_mesh               Mesh object in which primitive is created.
    \param  a_height             Height of cone.
    \param  a_radiusBottom       Bottom radius of cone.
    \param  a_radiusTop          Top radius of cone. Apply 0 value for non truncated cone.
    \param  a_numSides           Number of sides composing the cone.
    \param  a_numHeightSegments  Number of segments along the cone axis.
    \param  a_includeTop         If __true__, then the top disc is included. (truncated cone)
    \param  a_includeBottom      If __true__, then the bottom disc is included.
    \param  a_pos                Position where to build the new primitive.
    \param  a_rot                Orientation of the new primitive.
    \param  a_color              Color of vertices.
*/
//==============================================================================
void cCreateCone(cMesh* a_mesh,
    const double& a_height,
    const double& a_radiusBottom,
    const double& a_radiusTop,
    const unsigned int a_numSides,
    const unsigned int a_numHeightSegments,
    const unsigned int a_numRings,
    const bool a_includeBottom,
    const bool a_includeTop,
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_color)
{
    // sanity check
    unsigned int numS = cMax((unsigned int)3, a_numSides);
    unsigned int numH = cMax((unsigned int)1, a_numHeightSegments);

    // compute shared values
    double deltaAng = C_TWO_PI / (double)numS;
    double deltaLen = a_height    / (double)numH;

    // get vertex base id
    int vertexBaseID = a_mesh->getNumVertices();

    if (a_height > 0.0)
    {
        // create vertices
        double nz = (a_radiusBottom - a_radiusTop) / a_height;
        double deltaRadius = (a_radiusTop - a_radiusBottom) / ((double)(numH));
        for (unsigned int i=0; i<=numS; i++)
        {
            cVector3d p;
            double ang = (double)(i) * deltaAng;
            double cosAng = cos(ang);
            double sinAng = sin(ang);
            cVector3d np = cVector3d(cosAng, sinAng, 0.0);
            cVector3d n_ = cMul(a_rot, cVector3d(cosAng, sinAng, nz));
            n_.normalize();

            for (unsigned int j=0; j<=numH; j++)
            {
                cVector3d offset(0.0, 0.0, (double)j * deltaLen);
                double radius = a_radiusBottom + (double)(j) * deltaRadius; 
                np.mulr(radius, p);
                cVector3d p_ = cAdd(a_pos, cMul(a_rot, p + offset));
                cVector3d t_((double)(i)/(double)(numS), (double)(j)/(double)(numH), 0.0);
                a_mesh->newVertex(p_, n_, t_, a_color);
            }
        }

        // create triangles
        for (unsigned int i=0; i<numS; i++)
        {
            for (unsigned int j=0; j<numH; j++)
            {

                int index00 = vertexBaseID + ((i  ) * (numH+1)) + j;
                int index01 = vertexBaseID + ((i+1) * (numH+1)) + j;
                int index10 = vertexBaseID + ((i  ) * (numH+1)) + j+1;
                int index11 = vertexBaseID + ((i+1) * (numH+1)) + j+1;
                a_mesh->newTriangle(index00, index01, index11);
                a_mesh->newTriangle(index00, index11, index10);
            }
        }
    }

     // build cylinder bottom - create vertices and triangles
    if ((a_includeBottom) && (a_radiusBottom > 0.0))
    {
        double radius = a_radiusBottom / (double)(cMax((unsigned int)1, a_numRings));
        cVector3d t(0.0, 0.0, 0.0);
        cVector3d n = cMul(a_rot, cVector3d(0,0,-1));
        cVector3d p = a_pos; 
        unsigned int vertex0 = a_mesh->newVertex(p, n, t, a_color);

        for (unsigned int i=0; i<numS; i++)
        {
            double ang = -(double)(i) * deltaAng;
            p = cAdd(a_pos, cMul(a_rot, cVector3d(radius * cos(ang), radius * sin(ang), 0.0)));
            a_mesh->newVertex(p, n, t, a_color);
        }

        vertexBaseID = vertex0 + 1;
        for (unsigned int i=0; i<numS; i++)
        {
            unsigned int vertex1 = vertexBaseID + i;
            unsigned int vertex2 = vertexBaseID + (i + 1)%numS; 
            a_mesh->newTriangle(vertex0, vertex1, vertex2);
        }

        if (a_numRings > 1)
        {
            int rings = a_numRings - 1;
            for (int i=0; i<rings; i++)
            {
                // create vertices
                for (unsigned int j=0; j<numS; j++)
                {
                    double ang = -(double)(j) * deltaAng;
                    p = cAdd(a_pos, cMul(a_rot, cVector3d((i+2) * radius * cos(ang), (i+2) * radius * sin(ang), 0.0)));
                    a_mesh->newVertex(p, n, t, a_color);
                }

                // create triangles
                for (unsigned int j=0; j<numS; j++)
                {
                    unsigned int vertex00 = vertexBaseID + j;
                    unsigned int vertex01 = vertexBaseID + (j + 1)%numS;
                    unsigned int vertex10 = vertexBaseID + numS + j;
                    unsigned int vertex11 = vertexBaseID + numS + (j + 1)%numS; 
                    a_mesh->newTriangle(vertex00, vertex11, vertex01);
                    a_mesh->newTriangle(vertex00, vertex10, vertex11);
                }

                vertexBaseID = vertexBaseID + numS;
            }
        }
    }

    // build cylinder top - create vertices and triangles
    if ((a_includeTop) && (a_radiusTop > 0.0))
    {
        double radius = a_radiusTop / (double)(cMax((unsigned int)1, a_numRings));
        cVector3d t(1.0, 1.0, 0.0);
        cVector3d n = cMul(a_rot, cVector3d(0,0,1));
        cVector3d p = cAdd(a_pos, cMul(a_rot, cVector3d(0,0,a_height))); 
        unsigned int vertex0 = a_mesh->newVertex(p, n, t, a_color);

        for (unsigned int i=0; i<numS; i++)
        {
            double ang = (double)(i) * deltaAng;
            p = cAdd(a_pos, cMul(a_rot, cVector3d(radius * cos(ang), radius * sin(ang), a_height)));
            a_mesh->newVertex(p, n, t, a_color);
        }

        vertexBaseID = vertex0 + 1;
        for (unsigned int i=0; i<numS; i++)
        {
            unsigned int vertex1 = vertexBaseID + i;
            unsigned int vertex2 = vertexBaseID + (i + 1)%numS; 
            a_mesh->newTriangle(vertex0, vertex1, vertex2);
        }

        if (a_numRings > 1)
        {
            int rings = a_numRings - 1;
            for (int i=0; i<rings; i++)
            {
                // create vertices
                for (unsigned int j=0; j<numS; j++)
                {
                    double ang = (double)(j) * deltaAng;
                    p = cAdd(a_pos, cMul(a_rot, cVector3d((i+2) * radius * cos(ang), (i+2) * radius * sin(ang), a_height)));
                    a_mesh->newVertex(p, n, t, a_color);
                }

                // create triangles
                for (unsigned int j=0; j<numS; j++)
                {
                    unsigned int vertex00 = vertexBaseID + j;
                    unsigned int vertex01 = vertexBaseID + (j + 1)%numS;
                    unsigned int vertex10 = vertexBaseID + numS + j;
                    unsigned int vertex11 = vertexBaseID + numS + (j + 1)%numS; 
                    a_mesh->newTriangle(vertex00, vertex11, vertex01);
                    a_mesh->newTriangle(vertex00, vertex10, vertex11);
                }

                vertexBaseID = vertexBaseID + numS;
            }
        }
    }
}


//==============================================================================
/*!
    This function creates a pipe by defining a height, inner radius and outer 
    radius.

    \param  a_mesh               Mesh object in which primitive is created.
    \param  a_height             Height of the pipe.
    \param  a_innerRadius        Inner radius of the pipe.
    \param  a_outerRadius        Outer radius of the pipe.
    \param  a_numSides           Number of sides composing the pipe.
    \param  a_numHeightSegments  Number of segments along the cylinder axis.
    \param  a_pos                Position where to build the new primitive.
    \param  a_rot                Orientation of the new primitive.
    \param  a_color              Color of vertices.
*/
//==============================================================================
void cCreatePipe(cMesh* a_mesh, 
    const double& a_height,  
    const double& a_innerRadius,
    const double& a_outerRadius,
    const unsigned int a_numSides,
    const unsigned int a_numHeightSegments,
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_color)
{
    // sanity check 
    unsigned int numS = cMax((unsigned int)3, a_numSides);
    unsigned int numH = cMax((unsigned int)1, a_numHeightSegments);
    double innerRadius = cMin(a_innerRadius, a_outerRadius);
    double outerRadius = cMax(a_innerRadius, a_outerRadius);

    // compute shared values
    double deltaAng = C_TWO_PI / (double)numS;
    double deltaLen = a_height    / (double)numH;

    // get vertex base id
    int vertexBaseID;

    if (a_height > 0.0)
    {
        vertexBaseID = a_mesh->getNumVertices();

        // create cylinder vertices
        for (unsigned int i=0; i<=numS; i++)
        {
            cVector3d p;
            double ang = (double)(i) * deltaAng;
            double cosAng = cos(ang);
            double sinAng = sin(ang);
            cVector3d n = cVector3d(cosAng, sinAng, 0.0);
            cVector3d n_ = cMul(a_rot, cVector3d(cosAng, sinAng, 0.0));
            n_.normalize();

            for (unsigned int j=0; j<=numH; j++)
            {
                cVector3d offset(0.0, 0.0, (double)j * deltaLen);
                n.mulr(outerRadius, p);
                cVector3d p_ = cAdd(a_pos, cMul(a_rot, p + offset));
                cVector3d t_((double)(i)/(double)(numS), (double)(j)/(double)(numH), 0.0);
                a_mesh->newVertex(p_, n_, t_, a_color);
            }
        }

        // create cylinder triangles
        for (unsigned int i=0; i<numS; i++)
        {
            for (unsigned int j=0; j<numH; j++)
            {

                int index00 = vertexBaseID + ((i  ) * (numH+1)) + j;
                int index01 = vertexBaseID + ((i+1) * (numH+1)) + j;
                int index10 = vertexBaseID + ((i  ) * (numH+1)) + j+1;
                int index11 = vertexBaseID + ((i+1) * (numH+1)) + j+1;
                a_mesh->newTriangle(index00, index01, index11);
                a_mesh->newTriangle(index00, index11, index10);
            }
        }

        vertexBaseID = a_mesh->getNumVertices();

        // create cylinder vertices
        for (unsigned int i=0; i<=numS; i++)
        {
            cVector3d p;
            double ang = (double)(i) * deltaAng;
            double cosAng = cos(ang);
            double sinAng = sin(ang);
            cVector3d n = cVector3d(cosAng, sinAng, 0.0);
            cVector3d n_ = cMul(a_rot, cVector3d(cosAng, sinAng, 0.0));
            n_.normalize();

            for (unsigned int j=0; j<=numH; j++)
            {
                cVector3d offset(0.0, 0.0, (double)j * deltaLen);
                n.mulr(innerRadius, p);
                cVector3d p_ = cAdd(a_pos, cMul(a_rot, p + offset));
                cVector3d t_((double)(i)/(double)(numS), (double)(j)/(double)(numH), 0.0);
                a_mesh->newVertex(p_, n_, t_, a_color);
            }
        }

        // create cylinder triangles
        for (unsigned int i=0; i<numS; i++)
        {
            for (unsigned int j=0; j<numH; j++)
            {

                int index00 = vertexBaseID + ((i  ) * (numH+1)) + j;
                int index01 = vertexBaseID + ((i+1) * (numH+1)) + j;
                int index10 = vertexBaseID + ((i  ) * (numH+1)) + j+1;
                int index11 = vertexBaseID + ((i+1) * (numH+1)) + j+1;
                a_mesh->newTriangle(index00, index11, index01);
                a_mesh->newTriangle(index00, index10, index11);
            }
        }
    }

    // create extremities
    double deltaRadius = outerRadius - innerRadius;
    if (deltaRadius > 0.0)
    {
        // create top
        vertexBaseID = a_mesh->getNumVertices();

        cVector3d t(0.0, 0.0, 0.0);
        cVector3d n = cMul(a_rot, cVector3d(0,0,-1));

        for (unsigned int i=0; i<=numS; i++)
        {
            double ang = (double)(i) * deltaAng;
            cVector3d p0 = cAdd(a_pos, cMul(a_rot, cVector3d(outerRadius * cos(ang), outerRadius * sin(ang), 0.0))); 
            cVector3d p1 = cAdd(a_pos, cMul(a_rot, cVector3d(innerRadius * cos(ang), innerRadius * sin(ang), 0.0))); 
            a_mesh->newVertex(p0, n, t, a_color);
            a_mesh->newVertex(p1, n, t, a_color);
        }

        for (unsigned int i=0; i<numS; i++)
        {
            unsigned int vertex0 = vertexBaseID + (2*i);
            unsigned int vertex1 = vertexBaseID + (2*i) + 1; 
            unsigned int vertex2 = vertexBaseID + (2*i) + 2;
            unsigned int vertex3 = vertexBaseID + (2*i) + 3; 
            a_mesh->newTriangle(vertex0, vertex1, vertex2);
            a_mesh->newTriangle(vertex1, vertex3, vertex2);
        }

        // create bottom
        vertexBaseID = a_mesh->getNumVertices();

        t.set(1.0, 1.0, 0.0);
        n = cMul(a_rot, cVector3d(0,0,1));

        for (unsigned int i=0; i<=numS; i++)
        {
            double ang = (double)(i) * deltaAng;
            cVector3d p0 = cAdd(a_pos, cMul(a_rot, cVector3d(outerRadius * cos(ang), outerRadius * sin(ang), a_height))); 
            cVector3d p1 = cAdd(a_pos, cMul(a_rot, cVector3d(innerRadius * cos(ang), innerRadius * sin(ang), a_height))); 
            a_mesh->newVertex(p0, n, t, a_color);
            a_mesh->newVertex(p1, n, t, a_color);
        }

        for (unsigned int i=0; i<numS; i++)
        {
            unsigned int vertex0 = vertexBaseID + (2*i);
            unsigned int vertex1 = vertexBaseID + (2*i) + 1; 
            unsigned int vertex2 = vertexBaseID + (2*i) + 2;
            unsigned int vertex3 = vertexBaseID + (2*i) + 3; 
            a_mesh->newTriangle(vertex0, vertex2, vertex1);
            a_mesh->newTriangle(vertex1, vertex2, vertex3);
        }
    }
}


//==============================================================================
/*!
    This function creates sphere by defining its radius.

    \param  a_mesh       Mesh object in which primitive is created.
    \param  a_radius     Radius of sphere.
    \param  a_numSlices  Specifies the number of subdivisions around the z axis (similar to lines of longitude).
    \param  a_numStacks  Specifies the number of subdivisions along the z axis (similar to lines of latitude).
    \param  a_pos        Position where to build the new primitive.
    \param  a_rot        Orientation of the new primitive.
    \param  a_color      Color of vertices.
*/
//==============================================================================
void cCreateSphere(cMesh* a_mesh, 
    const double& a_radius,  
    const unsigned int a_numSlices,   
    const unsigned int a_numStacks,
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_color)
{
    cCreateEllipsoid(a_mesh, 
                     a_radius, 
                     a_radius, 
                     a_radius, 
                     a_numSlices, 
                     a_numStacks, 
                     a_pos, 
                     a_rot,
                     a_color);
}


//==============================================================================
/*!
    This function creates an ellipsoid by defining the radius properties along 
    each axis X, Y and Z.

    \param  a_mesh       Mesh object in which primitive is created.
    \param  a_radiusX    Radius along X axis.
    \param  a_radiusY    Radius along Y axis.
    \param  a_radiusZ    Radius along Z axis.
    \param  a_numSlices  Specifies the number of subdivisions around the z axis (similar to lines of longitude).
    \param  a_numStacks  Specifies the number of subdivisions along the z axis (similar to lines of latitude).
    \param  a_pos        Position where to build the new primitive.
    \param  a_rot        Orientation of the new primitive.
    \param  a_color      Color of vertices.
*/
//==============================================================================
void cCreateEllipsoid(cMesh* a_mesh, 
    const double& a_radiusX,
    const double& a_radiusY,
    const double& a_radiusZ,
    const unsigned int a_numSlices,
    const unsigned int a_numStacks,
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_color)
{
    // sanity check 
    unsigned int numS = cMax((unsigned int)3, a_numSlices);
    unsigned int numH = cMax((unsigned int)2, a_numStacks);

    if (a_radiusX <= 0.0) { return; }
    if (a_radiusY <= 0.0) { return; }
    if (a_radiusZ <= 0.0) { return; }

    // compute shared values
    double deltaAngS = C_TWO_PI / (double)numS;
    double deltaAngH = C_PI / (double)(numH);
    double cX = 1.0 / a_radiusX;
    double cY = 1.0 / a_radiusY;
    double cZ = 1.0 / a_radiusZ;

    // get vertex base id
    int vertexBaseID = a_mesh->getNumVertices();

    if (a_radiusZ > 0.0)
    {
        // create vertices
        for (unsigned int i=0; i<=numS; i++)
        {
            cVector3d p, n;
            double ang = (double)(i) * deltaAngS;
            double cosAng = cos(ang);
            double sinAng = sin(ang);

            for (unsigned int j=0; j<=numH; j++)
            {
                double angH = -C_PI_DIV_2 + (double)(j) * deltaAngH;
                double sinAngH = sin(angH);
                double cosAngH = cos(angH);
                p.set(a_radiusX * cosAng * cosAngH, a_radiusY * sinAng * cosAngH, a_radiusZ * sinAngH);
                n.set(cX * cosAng * cosAngH, cY * sinAng * cosAngH, cZ * sinAngH);
                n.normalize();
                cVector3d n_ = cMul(a_rot, n);
                n_.normalize();
                cVector3d p_ = cAdd(a_pos, cMul(a_rot, p));
                cVector3d t_((double)(i)/(double)(numS), (double)(j)/(double)(numH), 0.0);
                a_mesh->newVertex(p_, n_, t_, a_color);
            }
        }

        // create triangles
        for (unsigned int i=0; i<numS; i++)
        {
            for (unsigned int j=0; j<numH; j++)
            {

                int index00 = vertexBaseID + ((i  ) * (numH+1)) + j;
                int index01 = vertexBaseID + ((i+1) * (numH+1)) + j;
                int index10 = vertexBaseID + ((i  ) * (numH+1)) + j+1;
                int index11 = vertexBaseID + ((i+1) * (numH+1)) + j+1;
                a_mesh->newTriangle(index00, index01, index11);
                a_mesh->newTriangle(index00, index11, index10);
            }
        }
    }
}


//==============================================================================
/*!
    This function creates a torus by defining the inner and outer radius values.

    \param  a_mesh         Mesh object in which primitive is created.
    \param  a_innerRadius  Inner radius of the torus.
    \param  a_outerRadius  Outer radius of the torus.
    \param  a_numSides     Number of sides for each radial section.
    \param  a_numRings     Number of radial divisions for the torus.
    \param  a_pos          Position where to build the new primitive.
    \param  a_rot          Orientation of the new primitive.
    \param  a_color        Color of vertices.
*/
//==============================================================================
void cCreateRing(cMesh* a_mesh,
    const double& a_innerRadius,
    const double& a_outerRadius,
    const unsigned int a_numSides,
    const unsigned int a_numRings,
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_color)
{
    // sanity check 
    unsigned int numS = cMax((unsigned int)3, a_numSides);
    unsigned int numR = cMax((unsigned int)3, a_numRings);
    if (a_innerRadius < 0.0) { return; }
    if (a_outerRadius < 0.0) { return; }

    // compute step values
    double deltaAngS = C_TWO_PI / (double)numS;
    double deltaAngR = C_TWO_PI / (double)numR;

    // get vertex base id
    int vertexBaseID = a_mesh->getNumVertices();

    // create vertices
    for (unsigned int i=0; i<=numR; i++)
    {
        double angR = (double)(i) * deltaAngR;
        double cosAngR = cos(angR);
        double sinAngR = sin(angR);
        cVector3d pos(-a_outerRadius * sinAngR, a_outerRadius * cosAngR, 0.0);
        cMatrix3d rot;
        rot.identity();
        rot.rotateAboutGlobalAxisRad(cVector3d(0.0, 0.0, 1.0), angR);

        for (unsigned int j=0; j<=numS; j++)
        {
            double angS = (double)(j) * deltaAngS;
            double cosAngS = cos(angS);
            double sinAngS = sin(angS);
            cVector3d p_ = cAdd(a_pos, cMul(a_rot, cAdd(pos, cMul(rot, cVector3d(0.0, a_innerRadius * cosAngS, a_innerRadius * sinAngS)))));
            cVector3d n_ = cMul(a_rot, cMul(rot, cVector3d(0.0, cosAngS, sinAngS)));
            cVector3d t_((double)(j)/(double)(numS), (double)(i)/(double)(numR), 0.0);
            a_mesh->newVertex(p_, n_, t_, a_color);
        }
    }

    // create triangles
    for (unsigned int i=0; i<numR; i++)
    {
        for (unsigned int j=0; j<numS; j++)
        {
            int index00 = vertexBaseID + ((i  ) * (numS+1)) + j;
            int index01 = vertexBaseID + ((i+1) * (numS+1)) + j;
            int index10 = vertexBaseID + ((i  ) * (numS+1)) + j+1;
            int index11 = vertexBaseID + ((i+1) * (numS+1)) + j+1;
            a_mesh->newTriangle(index00, index01, index11);
            a_mesh->newTriangle(index00, index11, index10);
        }
    }

}


//==============================================================================
/*!
    This function creates a torus by defining the inner and outer radius values.

    \param  a_mesh                   Mesh object in which primitive is created.
    \param  a_innerRadius0           Inner radius of the ring at the beginning extremity.
    \param  a_innerRadius1           Inner radius of the ring at the end extremity.
    \param  a_outerRadius            Outer radius of the torus.
    \param  a_coverageAngleDEG       Coverage angle in degrees (from 0 to 360).
    \param  a_includeExtremityFaces  Include flat surfaces at extremities of ring section.
    \param  a_numSides               Number of sides for each radial section.
    \param  a_numRings               Number of radial divisions for the torus.
    \param  a_pos                    Position where to build the new primitive.
    \param  a_rot                    Orientation of the new primitive.
    \param  a_color                  Color of vertices.
*/
//==============================================================================
void cCreateRingSection(cMesh* a_mesh, 
    const double& a_innerRadius0,
    const double& a_innerRadius1,
    const double& a_outerRadius,    
    const double& a_coverageAngleDEG,
    const bool a_includeExtremityFaces,
    const unsigned int a_numSides,   
    const unsigned int a_numRings,  
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_color)
{
    // sanity check
    unsigned int numS = cMax((unsigned int)3, a_numSides);
    unsigned int numR = cMax((unsigned int)3, a_numRings);

    double coverageAngle = cClamp(a_coverageAngleDEG, 0.0, 360.0);
    if (coverageAngle == 0.0) { return; }
    
    bool includeExtremityFaces = a_includeExtremityFaces;
    if (coverageAngle == 360)
    {
        includeExtremityFaces = false;
    }
    
    if (a_innerRadius0 < 0.0) { return; }
    if (a_innerRadius1 < 0.0) { return; }
    if (a_outerRadius < 0.0) { return; }

    // compute step values
    double deltaAngS = C_TWO_PI / (double)numS;
    double deltaAngR = (coverageAngle / 360) * (C_TWO_PI / (double)numR);
    double deltaInnerRadius = (a_innerRadius1 - a_innerRadius0) / a_numRings;

    // get vertex base id
    int vertexBaseID = a_mesh->getNumVertices();

    // create vertices
    for (unsigned int i=0; i<=numR; i++)
    {
        double angR = (double)(i) * deltaAngR;
        double cosAngR = cos(angR);
        double sinAngR = sin(angR);
        cVector3d pos(-a_outerRadius * sinAngR, a_outerRadius * cosAngR, 0.0);
        cMatrix3d rot;
        rot.identity();
        rot.rotateAboutGlobalAxisRad(cVector3d(0.0, 0.0, 1.0), angR);

        double innerRadius = a_innerRadius0 + deltaInnerRadius * i;
        for (unsigned int j=0; j<=numS; j++)
        {
            double angS = (double)(j) * deltaAngS;
            double cosAngS = cos(angS);
            double sinAngS = sin(angS);
            cVector3d p_ = cAdd(a_pos, cMul(a_rot, cAdd(pos, cMul(rot, cVector3d(0.0, innerRadius * cosAngS, innerRadius * sinAngS)))));
            cVector3d n_ = cMul(a_rot, cMul(rot, cVector3d(0.0, cosAngS, sinAngS)));
            cVector3d t_((double)(j)/(double)(numS), (double)(i)/(double)(numR), 0.0);
            a_mesh->newVertex(p_, n_, t_, a_color);
        }
    }

    // create ring triangles 
    for (unsigned int i=0; i<numR; i++)
    {
        for (unsigned int j=0; j<numS; j++)
        {
            int index00 = vertexBaseID + ((i  ) * (numS+1)) + j;
            int index01 = vertexBaseID + ((i+1) * (numS+1)) + j;
            int index10 = vertexBaseID + ((i  ) * (numS+1)) + j+1;
            int index11 = vertexBaseID + ((i+1) * (numS+1)) + j+1;
            a_mesh->newTriangle(index00, index01, index11);
            a_mesh->newTriangle(index00, index11, index10);
        }
    }

    // create extremity triangles
    if (includeExtremityFaces)
    {
        // extremity 0:
        if (a_innerRadius0 > 0.0)
        {
            for (unsigned int j=0; j<numS; j++)
            {
                int index0 = vertexBaseID;
                int index1 = vertexBaseID + j;
                int index2 = vertexBaseID + j+1;
                cVector3d pos0 = a_mesh->m_vertices->getLocalPos(index0);
                cVector3d pos1 = a_mesh->m_vertices->getLocalPos(index1);
                cVector3d pos2 = a_mesh->m_vertices->getLocalPos(index2);
                int triangleIndex = a_mesh->newTriangle(pos0, pos1, pos2);
                a_mesh->m_triangles->computeNormal(triangleIndex, true);
            }
        }

        // extremity 1:
        if (a_innerRadius1 > 0.0)
        {
            for (unsigned int j=0; j<numS; j++)
            {
                int index0 = vertexBaseID + (numR * numS);
                int index1 = vertexBaseID + (numR * numS) + j;
                int index2 = vertexBaseID + (numR * numS) + j+1;
                cVector3d pos0 = a_mesh->m_vertices->getLocalPos(index0);
                cVector3d pos1 = a_mesh->m_vertices->getLocalPos(index1);
                cVector3d pos2 = a_mesh->m_vertices->getLocalPos(index2);
                int triangleIndex = a_mesh->newTriangle(pos0, pos1, pos2);
                a_mesh->m_triangles->computeNormal(triangleIndex, true);
            }
        }
    }
}


//==============================================================================
/*!
    This function creates a square pyramid.

    \param  a_mesh           Mesh object in which primitive is created.
    \param  a_height         Height of square pyramid.
    \param  a_baseSize       Size of a base of the pyramid.
    \param  a_includeBottom  If __true__, then the bottom (square) of the pyramid is included.
    \param  a_pos            Position where to build the new primitive.
    \param  a_rot            Orientation of the new primitive.
    \param  a_color          Color of vertices.
*/
//==============================================================================
void cCreateSquarePyramid(cMesh* a_mesh, 
    const double& a_height,  
    const double& a_baseSize,
    const bool a_includeBottom,
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_color)
{
    // sanity check
    if (a_baseSize == 0.0) { return; }

    // temp variables
    cVector3d v0, v1, v2, v3;
    cVector3d t0, t1, t2, t3;
    cVector3d n;
    double s = a_baseSize / 2.0;

    // create texture coordinates
    t0.set(0.0, 0.0, 0.0);
    t1.set(1.0, 0.0, 0.0);
    t2.set(0.5, 1.0, 0.0);

    // create face 1
    v0 = cAdd(a_pos, cMul(a_rot, cVector3d( s, -s, 0.0)));
    v1 = cAdd(a_pos, cMul(a_rot, cVector3d( s,  s, 0.0)));
    v2 = cAdd(a_pos, cMul(a_rot, cVector3d(0.0, 0.0, a_height)));
    n = cMul(a_rot, cNormalize(cCross(cSub(v1, v0), cSub(v2, v1))));
    a_mesh->newTriangle(v0, v1, v2, n, n, n, t0, t1, t2, a_color, a_color, a_color);

    // create face 2
    v0 = cAdd(a_pos, cMul(a_rot, cVector3d( s,  s, 0.0)));
    v1 = cAdd(a_pos, cMul(a_rot, cVector3d(-s,  s, 0.0)));
    n = cMul(a_rot, cNormalize(cCross(cSub(v1, v0), cSub(v2, v1))));
    a_mesh->newTriangle(v0, v1, v2, n, n, n, t0, t1, t2, a_color, a_color, a_color);

    // create face 3
    v0 = cAdd(a_pos, cMul(a_rot, cVector3d(-s,  s, 0.0)));
    v1 = cAdd(a_pos, cMul(a_rot, cVector3d(-s, -s, 0.0)));
    n = cMul(a_rot, cNormalize(cCross(cSub(v1, v0), cSub(v2, v1))));
    a_mesh->newTriangle(v0, v1, v2, n, n, n, t0, t1, t2, a_color, a_color, a_color);

    // create face 4
    v0 = cAdd(a_pos, cMul(a_rot, cVector3d(-s, -s, 0.0)));
    v1 = cAdd(a_pos, cMul(a_rot, cVector3d( s, -s, 0.0)));
    n = cMul(a_rot, cNormalize(cCross(cSub(v1, v0), cSub(v2, v1))));
    a_mesh->newTriangle(v0, v1, v2, n, n, n, t0, t1, t2, a_color, a_color, a_color);

    // create bottom
    if (a_includeBottom)
    {
        v0 = cAdd(a_pos, cMul(a_rot, cVector3d( s, -s, 0.0)));
        v1 = cAdd(a_pos, cMul(a_rot, cVector3d( s,  s, 0.0)));
        v2 = cAdd(a_pos, cMul(a_rot, cVector3d(-s,  s, 0.0)));
        v3 = cAdd(a_pos, cMul(a_rot, cVector3d(-s, -s, 0.0)));
        n = cMul(a_rot, cVector3d(0.0, 0.0,-1.0));

        t0.set(0.0, 1.0, 0.0);
        t1.set(1.0, 1.0, 0.0);
        t2.set(1.0, 0.0, 0.0);
        t3.set(0.0, 0.0, 0.0);

        a_mesh->newTriangle(v0, v3, v2, n, n, n, t0, t3, t2, a_color, a_color, a_color);
        a_mesh->newTriangle(v0, v2, v1, n, n, n, t0, t2, t1, a_color, a_color, a_color);
    }
}


//==============================================================================
/*!
    This function creates a copy of the famous OpenGL tea pot.

    \param  a_mesh   Mesh object in which primitive is created.
    \param  a_size   Size of the tea pot.
    \param  a_pos    Position where to build the new primitive.
    \param  a_rot    Orientation of the new primitive.
    \param  a_color  Color of vertices.
*/
//==============================================================================
void cCreateTeaPot(cMesh* a_mesh, 
    const double& a_size,
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_color)
{
    static short face_indices[1024][6] = 
    {
        {6,5,0 ,0,1,2 }, {0,1,6 ,2,3,0 }, {7,6,1 ,4,5,6 }, {1,2,7 ,6,7,4 },
        {8,7,2 ,8,9,10 }, {2,3,8 ,10,11,8 }, {9,8,3 ,12,8,11 }, {3,4,9 ,11,13,12 },
        {11,10,5 ,14,15,1 }, {5,6,11 ,1,0,14 }, {12,11,6 ,16,17,5 },
        {6,7,12 ,5,4,16 }, {13,12,7 ,18,19,9 }, {7,8,13 ,9,8,18 }, {14,13,8 ,20,18,8 },
        {8,9,14 ,8,12,20 }, {16,15,10 ,21,22,15 }, {10,11,16 ,15,14,21 },
        {17,16,11 ,23,24,17 }, {11,12,17 ,17,16,23 }, {18,17,12 ,25,26,19 },
        {12,13,18 ,19,18,25 }, {19,18,13 ,27,25,18 }, {13,14,19 ,18,20,27 },
        {21,20,15 ,28,29,22 }, {15,16,21 ,22,21,28 }, {22,21,16 ,30,31,24 },
        {16,17,22 ,24,23,30 }, {23,22,17 ,32,33,26 }, {17,18,23 ,26,25,32 },
        {24,23,18 ,34,32,25 }, {18,19,24 ,25,27,34 }, {26,25,20 ,35,36,29 },
        {20,21,26 ,29,28,35 }, {27,26,21 ,37,38,31 }, {21,22,27 ,31,30,37 },
        {28,27,22 ,39,40,33 }, {22,23,28 ,33,32,39 }, {29,28,23 ,41,39,32 },
        {23,24,29 ,32,34,41 }, {31,30,25 ,42,43,36 }, {25,26,31 ,36,35,42 },
        {32,31,26 ,44,45,38 }, {26,27,32 ,38,37,44 }, {33,32,27 ,46,47,40 },
        {27,28,33 ,40,39,46 }, {34,33,28 ,48,46,39 }, {28,29,34 ,39,41,48 },
        {36,35,30 ,49,50,43 }, {30,31,36 ,43,42,49 }, {37,36,31 ,51,52,45 },
        {31,32,37 ,45,44,51 }, {38,37,32 ,53,54,47 }, {32,33,38 ,47,46,53 },
        {39,38,33 ,55,53,46 }, {33,34,39 ,46,48,55 }, {41,40,35 ,56,57,50 },
        {35,36,41 ,50,49,56 }, {42,41,36 ,58,59,52 }, {36,37,42 ,52,51,58 },
        {43,42,37 ,60,61,54 }, {37,38,43 ,54,53,60 }, {44,43,38 ,62,60,53 },
        {38,39,44 ,53,55,62 }, {46,45,40 ,63,64,57 }, {40,41,46 ,57,56,63 },
        {47,46,41 ,65,66,59 }, {41,42,47 ,59,58,65 }, {48,47,42 ,67,68,61 },
        {42,43,48 ,61,60,67 }, {49,48,43 ,69,67,60 }, {43,44,49 ,60,62,69 },
        {51,50,45 ,70,71,64 }, {45,46,51 ,64,63,70 }, {52,51,46 ,72,73,66 },
        {46,47,52 ,66,65,72 }, {53,52,47 ,74,75,68 }, {47,48,53 ,68,67,74 },
        {54,53,48 ,76,74,67 }, {48,49,54 ,67,69,76 }, {56,55,50 ,77,78,71 },
        {50,51,56 ,71,70,77 }, {57,56,51 ,79,80,73 }, {51,52,57 ,73,72,79 },
        {58,57,52 ,81,82,75 }, {52,53,58 ,75,74,81 }, {59,58,53 ,83,81,74 },
        {53,54,59 ,74,76,83 }, {61,60,55 ,84,85,78 }, {55,56,61 ,78,77,84 },
        {62,61,56 ,86,87,80 }, {56,57,62 ,80,79,86 }, {63,62,57 ,88,89,82 },
        {57,58,63 ,82,81,88 }, {64,63,58 ,90,88,81 }, {58,59,64 ,81,83,90 },
        {66,65,60 ,91,92,85 }, {60,61,66 ,85,84,91 }, {67,66,61 ,93,94,87 },
        {61,62,67 ,87,86,93 }, {68,67,62 ,95,96,89 }, {62,63,68 ,89,88,95 },
        {69,68,63 ,97,95,88 }, {63,64,69 ,88,90,97 }, {71,70,65 ,98,99,92 },
        {65,66,71 ,92,91,98 }, {72,71,66 ,100,101,94 }, {66,67,72 ,94,93,100 },
        {73,72,67 ,102,103,96 }, {67,68,73 ,96,95,102 }, {74,73,68 ,104,102,95 },
        {68,69,74 ,95,97,104 }, {76,75,70 ,105,106,99 }, {70,71,76 ,99,98,105 },
        {77,76,71 ,107,108,101 }, {71,72,77 ,101,100,107 }, {78,77,72 ,109,110,103 },
        {72,73,78 ,103,102,109 }, {79,78,73 ,111,109,102 }, {73,74,79 ,102,104,111 },
        {1,0,75 ,3,2,106 }, {75,76,1 ,106,105,3 }, {2,1,76 ,7,6,108 },
        {76,77,2 ,108,107,7 }, {3,2,77 ,11,10,110 }, {77,78,3 ,110,109,11 },
        {4,3,78 ,13,11,109 }, {78,79,4 ,109,111,13 }, {84,9,4 ,112,12,13 },
        {4,80,84 ,13,113,112 }, {85,84,80 ,114,112,113 }, {80,81,85 ,113,115,114 },
        {86,85,81 ,116,114,115 }, {81,82,86 ,115,117,116 }, {87,86,82 ,118,116,117 },
        {82,83,87 ,117,119,118 }, {88,14,9 ,120,20,12 }, {9,84,88 ,12,112,120 },
        {89,88,84 ,121,120,112 }, {84,85,89 ,112,114,121 }, {90,89,85 ,122,121,114 },
        {85,86,90 ,114,116,122 }, {91,90,86 ,123,122,116 }, {86,87,91 ,116,118,123 },
        {92,19,14 ,124,27,20 }, {14,88,92 ,20,120,124 }, {93,92,88 ,125,124,120 },
        {88,89,93 ,120,121,125 }, {94,93,89 ,126,125,121 }, {89,90,94 ,121,122,126 },
        {95,94,90 ,127,126,122 }, {90,91,95 ,122,123,127 }, {96,24,19 ,128,34,27 },
        {19,92,96 ,27,124,128 }, {97,96,92 ,129,128,124 }, {92,93,97 ,124,125,129 },
        {98,97,93 ,130,129,125 }, {93,94,98 ,125,126,130 }, {99,98,94 ,131,130,126 },
        {94,95,99 ,126,127,131 }, {100,29,24 ,132,41,34 }, {24,96,100 ,34,128,132 },
        {101,100,96 ,133,132,128 }, {96,97,101 ,128,129,133 }, {102,101,97 ,134,133,129 },
        {97,98,102 ,129,130,134 }, {103,102,98 ,135,134,130 }, {98,99,103 ,130,131,135 },
        {104,34,29 ,136,48,41 }, {29,100,104 ,41,132,136 }, {105,104,100 ,137,136,132 },
        {100,101,105 ,132,133,137 }, {106,105,101 ,138,137,133 }, {101,102,106 ,133,134,138 },
        {107,106,102 ,139,138,134 }, {102,103,107 ,134,135,139 }, {108,39,34 ,140,55,48 },
        {34,104,108 ,48,136,140 }, {109,108,104 ,141,140,136 }, {104,105,109 ,136,137,141 },
        {110,109,105 ,142,141,137 }, {105,106,110 ,137,138,142 }, {111,110,106 ,143,142,138 },
        {106,107,111 ,138,139,143 }, {112,44,39 ,144,62,55 }, {39,108,112 ,55,140,144 },
        {113,112,108 ,145,144,140 }, {108,109,113 ,140,141,145 }, {114,113,109 ,146,145,141 },
        {109,110,114 ,141,142,146 }, {115,114,110 ,147,146,142 }, {110,111,115 ,142,143,147 },
        {116,49,44 ,148,69,62 }, {44,112,116 ,62,144,148 }, {117,116,112 ,149,148,144 },
        {112,113,117 ,144,145,149 }, {118,117,113 ,150,149,145 }, {113,114,118 ,145,146,150 },
        {119,118,114 ,151,150,146 }, {114,115,119 ,146,147,151 }, {120,54,49 ,152,76,69 },
        {49,116,120 ,69,148,152 }, {121,120,116 ,153,152,148 }, {116,117,121 ,148,149,153 },
        {122,121,117 ,154,153,149 }, {117,118,122 ,149,150,154 }, {123,122,118 ,155,154,150 },
        {118,119,123 ,150,151,155 }, {124,59,54 ,156,83,76 }, {54,120,124 ,76,152,156 },
        {125,124,120 ,157,156,152 }, {120,121,125 ,152,153,157 }, {126,125,121 ,158,157,153 },
        {121,122,126 ,153,154,158 }, {127,126,122 ,159,158,154 }, {122,123,127 ,154,155,159 },
        {128,64,59 ,160,90,83 }, {59,124,128 ,83,156,160 }, {129,128,124 ,161,160,156 },
        {124,125,129 ,156,157,161 }, {130,129,125 ,162,161,157 }, {125,126,130 ,157,158,162 },
        {131,130,126 ,163,162,158 }, {126,127,131 ,158,159,163 }, {132,69,64 ,164,97,90 },
        {64,128,132 ,90,160,164 }, {133,132,128 ,165,164,160 }, {128,129,133 ,160,161,165 },
        {134,133,129 ,166,165,161 }, {129,130,134 ,161,162,166 }, {135,134,130 ,167,166,162 },
        {130,131,135 ,162,163,167 }, {136,74,69 ,168,104,97 }, {69,132,136 ,97,164,168 },
        {137,136,132 ,169,168,164 }, {132,133,137 ,164,165,169 }, {138,137,133 ,170,169,165 },
        {133,134,138 ,165,166,170 }, {139,138,134 ,171,170,166 }, {134,135,139 ,166,167,171 },
        {140,79,74 ,172,111,104 }, {74,136,140 ,104,168,172 }, {141,140,136 ,173,172,168 },
        {136,137,141 ,168,169,173 }, {142,141,137 ,174,173,169 }, {137,138,142 ,169,170,174 },
        {143,142,138 ,175,174,170 }, {138,139,143 ,170,171,175 }, {80,4,79 ,113,13,111 },
        {79,140,80 ,111,172,113 }, {81,80,140 ,115,113,172 }, {140,141,81 ,172,173,115 },
        {82,81,141 ,117,115,173 }, {141,142,82 ,173,174,117 }, {83,82,142 ,119,117,174 },
        {142,143,83 ,174,175,119 }, {148,87,83 ,176,118,119 }, {83,144,148 ,119,177,176 },
        {149,148,144 ,178,176,177 }, {144,145,149 ,177,179,178 }, {150,149,145 ,180,178,179 },
        {145,146,150 ,179,181,180 }, {151,150,146 ,182,180,181 }, {146,147,151 ,181,183,182 },
        {152,91,87 ,184,123,118 }, {87,148,152 ,118,176,184 }, {153,152,148 ,185,184,176 },
        {148,149,153 ,176,178,185 }, {154,153,149 ,186,185,178 }, {149,150,154 ,178,180,186 },
        {155,154,150 ,187,186,180 }, {150,151,155 ,180,182,187 }, {156,95,91 ,188,127,123 },
        {91,152,156 ,123,184,188 }, {157,156,152 ,189,188,184 }, {152,153,157 ,184,185,189 },
        {158,157,153 ,190,189,185 }, {153,154,158 ,185,186,190 }, {159,158,154 ,191,190,186 },
        {154,155,159 ,186,187,191 }, {160,99,95 ,192,131,127 }, {95,156,160 ,127,188,192 },
        {161,160,156 ,193,192,188 }, {156,157,161 ,188,189,193 }, {162,161,157 ,194,193,189 },
        {157,158,162 ,189,190,194 }, {163,162,158 ,195,194,190 }, {158,159,163 ,190,191,195 },
        {164,103,99 ,196,135,131 }, {99,160,164 ,131,192,196 }, {165,164,160 ,197,196,192 },
        {160,161,165 ,192,193,197 }, {166,165,161 ,198,197,193 }, {161,162,166 ,193,194,198 },
        {167,166,162 ,199,198,194 }, {162,163,167 ,194,195,199 }, {168,107,103 ,200,139,135 },
        {103,164,168 ,135,196,200 }, {169,168,164 ,201,200,196 }, {164,165,169 ,196,197,201 },
        {170,169,165 ,202,201,197 }, {165,166,170 ,197,198,202 }, {171,170,166 ,203,202,198 },
        {166,167,171 ,198,199,203 }, {172,111,107 ,204,143,139 }, {107,168,172 ,139,200,204 },
        {173,172,168 ,205,204,200 }, {168,169,173 ,200,201,205 }, {174,173,169 ,206,205,201 },
        {169,170,174 ,201,202,206 }, {175,174,170 ,207,206,202 }, {170,171,175 ,202,203,207 },
        {176,115,111 ,208,147,143 }, {111,172,176 ,143,204,208 }, {177,176,172 ,209,208,204 },
        {172,173,177 ,204,205,209 }, {178,177,173 ,210,209,205 }, {173,174,178 ,205,206,210 },
        {179,178,174 ,211,210,206 }, {174,175,179 ,206,207,211 }, {180,119,115 ,212,151,147 },
        {115,176,180 ,147,208,212 }, {181,180,176 ,213,212,208 }, {176,177,181 ,208,209,213 },
        {182,181,177 ,214,213,209 }, {177,178,182 ,209,210,214 }, {183,182,178 ,215,214,210 },
        {178,179,183 ,210,211,215 }, {184,123,119 ,216,155,151 }, {119,180,184 ,151,212,216 },
        {185,184,180 ,217,216,212 }, {180,181,185 ,212,213,217 }, {186,185,181 ,218,217,213 },
        {181,182,186 ,213,214,218 }, {187,186,182 ,219,218,214 }, {182,183,187 ,214,215,219 },
        {188,127,123 ,220,159,155 }, {123,184,188 ,155,216,220 }, {189,188,184 ,221,220,216 },
        {184,185,189 ,216,217,221 }, {190,189,185 ,222,221,217 }, {185,186,190 ,217,218,222 },
        {191,190,186 ,223,222,218 }, {186,187,191 ,218,219,223 }, {192,131,127 ,224,163,159 },
        {127,188,192 ,159,220,224 }, {193,192,188 ,225,224,220 }, {188,189,193 ,220,221,225 },
        {194,193,189 ,226,225,221 }, {189,190,194 ,221,222,226 }, {195,194,190 ,227,226,222 },
        {190,191,195 ,222,223,227 }, {196,135,131 ,228,167,163 }, {131,192,196 ,163,224,228 },
        {197,196,192 ,229,228,224 }, {192,193,197 ,224,225,229 }, {198,197,193 ,230,229,225 },
        {193,194,198 ,225,226,230 }, {199,198,194 ,231,230,226 }, {194,195,199 ,226,227,231 },
        {200,139,135 ,232,171,167 }, {135,196,200 ,167,228,232 }, {201,200,196 ,233,232,228 },
        {196,197,201 ,228,229,233 }, {202,201,197 ,234,233,229 }, {197,198,202 ,229,230,234 },
        {203,202,198 ,235,234,230 }, {198,199,203 ,230,231,235 }, {204,143,139 ,236,175,171 },
        {139,200,204 ,171,232,236 }, {205,204,200 ,237,236,232 }, {200,201,205 ,232,233,237 },
        {206,205,201 ,238,237,233 }, {201,202,206 ,233,234,238 }, {207,206,202 ,239,238,234 },
        {202,203,207 ,234,235,239 }, {144,83,143 ,177,119,175 }, {143,204,144 ,175,236,177 },
        {145,144,204 ,179,177,236 }, {204,205,145 ,236,237,179 }, {146,145,205 ,181,179,237 },
        {205,206,146 ,237,238,181 }, {147,146,206 ,183,181,238 }, {206,207,147 ,238,239,183 },
        {212,151,147 ,240,182,183 }, {147,208,212 ,183,241,240 }, {213,212,208 ,242,243,244 },
        {208,209,213 ,244,245,242 }, {214,213,209 ,246,242,245 }, {209,210,214 ,245,247,246 },
        {211,214,210 ,248,246,247 }, {210,211,211 ,249,249,249 }, {215,155,151 ,250,187,182 },
        {151,212,215 ,182,240,250 }, {216,215,212 ,251,252,243 }, {212,213,216 ,243,242,251 },
        {217,216,213 ,253,251,242 }, {213,214,217 ,242,246,253 }, {211,217,214 ,248,253,246 },
        {214,211,211 ,249,249,249 }, {218,159,155 ,254,191,187 }, {155,215,218 ,187,250,254 },
        {219,218,215 ,255,256,252 }, {215,216,219 ,252,251,255 }, {220,219,216 ,257,255,251 },
        {216,217,220 ,251,253,257 }, {211,220,217 ,248,257,253 }, {217,211,211 ,249,249,249 },
        {221,163,159 ,258,195,191 }, {159,218,221 ,191,254,258 }, {222,221,218 ,259,260,256 },
        {218,219,222 ,256,255,259 }, {223,222,219 ,261,259,255 }, {219,220,223 ,255,257,261 },
        {211,223,220 ,248,261,257 }, {220,211,211 ,249,249,249 }, {224,167,163 ,262,199,195 },
        {163,221,224 ,195,258,262 }, {225,224,221 ,263,264,260 }, {221,222,225 ,260,259,263 },
        {226,225,222 ,265,263,259 }, {222,223,226 ,259,261,265 }, {211,226,223 ,248,265,261 },
        {223,211,211 ,249,249,249 }, {227,171,167 ,266,203,199 }, {167,224,227 ,199,262,266 },
        {228,227,224 ,267,268,264 }, {224,225,228 ,264,263,267 }, {229,228,225 ,269,267,263 },
        {225,226,229 ,263,265,269 }, {211,229,226 ,248,269,265 }, {226,211,211 ,249,249,249 },
        {230,175,171 ,270,207,203 }, {171,227,230 ,203,266,270 }, {231,230,227 ,271,272,268 },
        {227,228,231 ,268,267,271 }, {232,231,228 ,273,271,267 }, {228,229,232 ,267,269,273 },
        {211,232,229 ,248,273,269 }, {229,211,211 ,249,249,249 }, {233,179,175 ,274,211,207 },
        {175,230,233 ,207,270,274 }, {234,233,230 ,275,276,272 }, {230,231,234 ,272,271,275 },
        {235,234,231 ,277,275,271 }, {231,232,235 ,271,273,277 }, {211,235,232 ,248,277,273 },
        {232,211,211 ,249,249,249 }, {236,183,179 ,278,215,211 }, {179,233,236 ,211,274,278 },
        {237,236,233 ,279,280,276 }, {233,234,237 ,276,275,279 }, {238,237,234 ,281,279,275 },
        {234,235,238 ,275,277,281 }, {211,238,235 ,248,281,277 }, {235,211,211 ,249,249,249 },
        {239,187,183 ,282,219,215 }, {183,236,239 ,215,278,282 }, {240,239,236 ,283,284,280 },
        {236,237,240 ,280,279,283 }, {241,240,237 ,285,283,279 }, {237,238,241 ,279,281,285 },
        {211,241,238 ,248,285,281 }, {238,211,211 ,249,249,249 }, {242,191,187 ,286,223,219 },
        {187,239,242 ,219,282,286 }, {243,242,239 ,287,288,284 }, {239,240,243 ,284,283,287 },
        {244,243,240 ,289,287,283 }, {240,241,244 ,283,285,289 }, {211,244,241 ,248,289,285 },
        {241,211,211 ,249,249,249 }, {245,195,191 ,290,227,223 }, {191,242,245 ,223,286,290 },
        {246,245,242 ,291,292,288 }, {242,243,246 ,288,287,291 }, {247,246,243 ,293,291,287 },
        {243,244,247 ,287,289,293 }, {211,247,244 ,248,293,289 }, {244,211,211 ,249,249,249 },
        {248,199,195 ,294,231,227 }, {195,245,248 ,227,290,294 }, {249,248,245 ,295,296,292 },
        {245,246,249 ,292,291,295 }, {250,249,246 ,297,295,291 }, {246,247,250 ,291,293,297 },
        {211,250,247 ,248,297,293 }, {247,211,211 ,249,249,249 }, {251,203,199 ,298,235,231 },
        {199,248,251 ,231,294,298 }, {252,251,248 ,299,300,296 }, {248,249,252 ,296,295,299 },
        {253,252,249 ,301,299,295 }, {249,250,253 ,295,297,301 }, {211,253,250 ,248,301,297 },
        {250,211,211 ,249,249,249 }, {254,207,203 ,302,239,235 }, {203,251,254 ,235,298,302 },
        {255,254,251 ,303,304,300 }, {251,252,255 ,300,299,303 }, {256,255,252 ,305,303,299 },
        {252,253,256 ,299,301,305 }, {211,256,253 ,248,305,301 }, {253,211,211 ,249,249,249 },
        {208,147,207 ,241,183,239 }, {207,254,208 ,239,302,241 }, {209,208,254 ,245,244,304 },
        {254,255,209 ,304,303,245 }, {210,209,255 ,247,245,303 }, {255,256,210 ,303,305,247 },
        {211,210,256 ,248,247,305 }, {256,211,211 ,249,249,249 }, {263,262,257 ,306,307,308 },
        {257,258,263 ,308,309,306 }, {264,263,258 ,310,306,309 }, {258,259,264 ,309,311,310 },
        {265,264,259 ,312,310,311 }, {259,260,265 ,311,313,312 }, {266,265,260 ,314,315,316 },
        {260,261,266 ,316,317,314 }, {268,267,262 ,318,319,320 }, {262,263,268 ,320,321,318 },
        {269,268,263 ,322,318,321 }, {263,264,269 ,321,323,322 }, {270,269,264 ,324,322,323 },
        {264,265,270 ,323,325,324 }, {271,270,265 ,326,324,325 }, {265,266,271 ,325,327,326 },
        {273,272,267 ,328,329,330 }, {267,268,273 ,330,331,328 }, {274,273,268 ,332,328,331 },
        {268,269,274 ,331,333,332 }, {275,274,269 ,334,332,333 }, {269,270,275 ,333,335,334 },
        {276,275,270 ,336,334,335 }, {270,271,276 ,335,337,336 }, {278,277,272 ,338,339,340 },
        {272,273,278 ,340,341,338 }, {279,278,273 ,342,338,341 }, {273,274,279 ,341,343,342 },
        {280,279,274 ,344,342,343 }, {274,275,280 ,343,345,344 }, {281,280,275 ,346,347,348 },
        {275,276,281 ,348,349,346 }, {283,282,277 ,350,351,339 }, {277,278,283 ,339,338,350 },
        {284,283,278 ,352,350,338 }, {278,279,284 ,338,342,352 }, {285,284,279 ,353,352,342 },
        {279,280,285 ,342,344,353 }, {286,285,280 ,354,355,347 }, {280,281,286 ,347,346,354 },
        {288,287,282 ,356,357,358 }, {282,283,288 ,358,359,356 }, {289,288,283 ,360,356,359 },
        {283,284,289 ,359,361,360 }, {290,289,284 ,362,360,361 }, {284,285,290 ,361,363,362 },
        {291,290,285 ,364,362,363 }, {285,286,291 ,363,365,364 }, {293,292,287 ,366,367,368 },
        {287,288,293 ,368,369,366 }, {294,293,288 ,370,366,369 }, {288,289,294 ,369,371,370 },
        {295,294,289 ,372,370,371 }, {289,290,295 ,371,373,372 }, {296,295,290 ,374,372,373 },
        {290,291,296 ,373,375,374 }, {258,257,292 ,309,308,376 }, {292,293,258 ,376,377,309 },
        {259,258,293 ,311,309,377 }, {293,294,259 ,377,378,311 }, {260,259,294 ,313,311,378 },
        {294,295,260 ,378,379,313 }, {261,260,295 ,317,316,380 }, {295,296,261 ,380,381,317 },
        {300,266,261 ,382,383,384 }, {261,297,300 ,384,385,382 }, {301,300,297 ,386,382,385 },
        {297,298,301 ,385,387,386 }, {302,301,298 ,388,386,387 }, {298,299,302 ,387,389,388 },
        {303,302,299 ,390,388,389 }, {299,115,303 ,389,391,390 }, {304,271,266 ,392,326,327 },
        {266,300,304 ,327,393,392 }, {305,304,300 ,394,392,393 }, {300,301,305 ,393,395,394 },
        {306,305,301 ,396,394,395 }, {301,302,306 ,395,397,396 }, {307,306,302 ,398,396,397 },
        {302,303,307 ,397,399,398 }, {308,276,271 ,400,336,337 }, {271,304,308 ,337,401,400 },
        {309,308,304 ,402,400,401 }, {304,305,309 ,401,403,402 }, {310,309,305 ,404,402,403 },
        {305,306,310 ,403,405,404 }, {311,310,306 ,406,404,405 }, {306,307,311 ,405,407,406 },
        {312,281,276 ,408,346,349 }, {276,308,312 ,349,409,408 }, {313,312,308 ,410,408,409 },
        {308,309,313 ,409,411,410 }, {314,313,309 ,412,410,411 }, {309,310,314 ,411,413,412 },
        {315,314,310 ,414,412,413 }, {310,311,315 ,413,415,414 }, {316,286,281 ,416,354,346 },
        {281,312,316 ,346,408,416 }, {317,316,312 ,417,416,408 }, {312,313,317 ,408,410,417 },
        {318,317,313 ,418,417,410 }, {313,314,318 ,410,412,418 }, {319,318,314 ,419,418,412 },
        {314,315,319 ,412,414,419 }, {320,291,286 ,420,364,365 }, {286,316,320 ,365,421,420 },
        {321,320,316 ,422,420,421 }, {316,317,321 ,421,423,422 }, {322,321,317 ,424,422,423 },
        {317,318,322 ,423,425,424 }, {323,322,318 ,426,424,425 }, {318,319,323 ,425,427,426 },
        {324,296,291 ,428,374,375 }, {291,320,324 ,375,429,428 }, {325,324,320 ,430,428,429 },
        {320,321,325 ,429,431,430 }, {326,325,321 ,432,430,431 }, {321,322,326 ,431,433,432 },
        {327,326,322 ,434,432,433 }, {322,323,327 ,433,435,434 }, {297,261,296 ,385,384,436 },
        {296,324,297 ,436,437,385 }, {298,297,324 ,387,385,437 }, {324,325,298 ,437,438,387 },
        {299,298,325 ,389,387,438 }, {325,326,299 ,438,439,389 }, {115,299,326 ,391,389,439 },
        {326,327,115 ,439,440,391 }, {334,333,328 ,441,442,441 }, {328,329,334 ,441,443,441 },
        {335,334,329 ,444,445,446 }, {329,330,335 ,446,447,444 }, {336,335,330 ,448,444,447 },
        {330,331,336 ,447,449,448 }, {337,336,331 ,450,448,449 }, {331,332,337 ,449,451,450 },
        {339,338,333 ,452,453,454 }, {333,334,339 ,454,455,452 }, {340,339,334 ,456,452,455 },
        {334,335,340 ,455,457,456 }, {341,340,335 ,458,456,457 }, {335,336,341 ,457,459,458 },
        {342,341,336 ,460,458,459 }, {336,337,342 ,459,461,460 }, {344,343,338 ,462,463,464 },
        {338,339,344 ,464,465,462 }, {345,344,339 ,466,462,465 }, {339,340,345 ,465,467,466 },
        {346,345,340 ,468,466,467 }, {340,341,346 ,467,469,468 }, {347,346,341 ,470,468,469 },
        {341,342,347 ,469,460,470 }, {349,348,343 ,471,472,473 }, {343,344,349 ,473,474,471 },
        {350,349,344 ,475,476,477 }, {344,345,350 ,477,466,475 }, {351,350,345 ,478,475,466 },
        {345,346,351 ,466,479,478 }, {352,351,346 ,480,478,479 }, {346,347,352 ,479,481,480 },
        {354,353,348 ,482,483,472 }, {348,349,354 ,472,471,482 }, {355,354,349 ,484,485,476 },
        {349,350,355 ,476,475,484 }, {356,355,350 ,486,484,475 }, {350,351,356 ,475,478,486 },
        {357,356,351 ,487,486,478 }, {351,352,357 ,478,480,487 }, {359,358,353 ,488,489,490 },
        {353,354,359 ,490,491,488 }, {360,359,354 ,492,488,491 }, {354,355,360 ,491,493,492 },
        {361,360,355 ,494,492,493 }, {355,356,361 ,493,495,494 }, {362,361,356 ,496,494,495 },
        {356,357,362 ,495,497,496 }, {364,363,358 ,498,499,500 }, {358,359,364 ,500,501,498 },
        {365,364,359 ,502,498,501 }, {359,360,365 ,501,503,502 }, {366,365,360 ,504,502,503 },
        {360,361,366 ,503,505,504 }, {367,366,361 ,506,504,505 }, {361,362,367 ,505,507,506 },
        {329,328,363 ,443,508,509 }, {363,364,329 ,509,510,443 }, {330,329,364 ,447,446,511 },
        {364,365,330 ,511,512,447 }, {331,330,365 ,449,447,512 }, {365,366,331 ,512,513,449 },
        {332,331,366 ,514,449,513 }, {366,367,332 ,513,515,514 }, {372,337,332 ,516,450,451 },
        {332,368,372 ,451,517,516 }, {373,372,368 ,518,516,517 }, {368,369,373 ,517,519,518 },
        {374,373,369 ,520,521,522 }, {369,370,374 ,522,523,520 }, {375,374,370 ,524,525,524 },
        {370,371,375 ,524,526,524 }, {376,342,337 ,527,460,461 }, {337,372,376 ,461,516,527 },
        {377,376,372 ,528,527,516 }, {372,373,377 ,516,518,528 }, {378,377,373 ,529,530,521 },
        {373,374,378 ,521,520,529 }, {379,378,374 ,531,532,531 }, {374,375,379 ,531,533,531 },
        {380,347,342 ,534,470,460 }, {342,376,380 ,460,535,534 }, {381,380,376 ,536,537,527 },
        {376,377,381 ,527,528,536 }, {382,381,377 ,538,539,530 }, {377,378,382 ,530,529,538 },
        {383,382,378 ,540,538,529 }, {378,379,383 ,529,541,540 }, {384,352,347 ,542,480,481 },
        {347,380,384 ,481,543,542 }, {385,384,380 ,544,542,543 }, {380,381,385 ,543,545,544 },
        {386,385,381 ,546,547,539 }, {381,382,386 ,539,538,546 }, {387,386,382 ,548,546,538 },
        {382,383,387 ,538,549,548 }, {388,357,352 ,550,487,480 }, {352,384,388 ,480,542,550 },
        {389,388,384 ,551,552,542 }, {384,385,389 ,542,553,551 }, {390,389,385 ,554,555,547 },
        {385,386,390 ,547,546,554 }, {391,390,386 ,556,554,546 }, {386,387,391 ,546,548,556 },
        {392,362,357 ,557,496,497 }, {357,388,392 ,497,558,557 }, {393,392,388 ,559,560,561 },
        {388,389,393 ,561,562,559 }, {394,393,389 ,563,564,555 }, {389,390,394 ,555,554,563 },
        {395,394,390 ,565,563,554 }, {390,391,395 ,554,566,565 }, {396,367,362 ,567,506,507 },
        {362,392,396 ,507,560,567 }, {397,396,392 ,568,567,560 }, {392,393,397 ,560,559,568 },
        {398,397,393 ,569,570,564 }, {393,394,398 ,564,563,569 }, {399,398,394 ,571,572,571 },
        {394,395,399 ,571,573,571 }, {368,332,367 ,517,514,515 }, {367,396,368 ,515,567,517 },
        {369,368,396 ,519,517,567 }, {396,397,369 ,567,568,519 }, {370,369,397 ,574,575,570 },
        {397,398,370 ,570,569,574 }, {371,370,398 ,576,577,576 }, {398,399,371 ,576,578,576 },
        {405,400,400 ,249,249,249 }, {400,401,405 ,579,580,581 }, {406,405,401 ,582,583,584 },
        {401,402,406 ,584,585,582 }, {407,406,402 ,586,582,585 }, {402,403,407 ,585,587,586 },
        {408,407,403 ,588,589,590 }, {403,404,408 ,590,591,588 }, {409,400,400 ,249,249,249 },
        {400,405,409 ,579,581,592 }, {410,409,405 ,593,594,583 }, {405,406,410 ,583,582,593 },
        {411,410,406 ,595,593,582 }, {406,407,411 ,582,586,595 }, {412,411,407 ,596,597,589 },
        {407,408,412 ,589,588,596 }, {413,400,400 ,249,249,249 }, {400,409,413 ,579,592,598 },
        {414,413,409 ,599,600,594 }, {409,410,414 ,594,593,599 }, {415,414,410 ,601,599,593 },
        {410,411,415 ,593,595,601 }, {416,415,411 ,602,603,597 }, {411,412,416 ,597,596,602 },
        {417,400,400 ,249,249,249 }, {400,413,417 ,579,598,604 }, {418,417,413 ,605,606,600 },
        {413,414,418 ,600,599,605 }, {419,418,414 ,607,605,599 }, {414,415,419 ,599,601,607 },
        {420,419,415 ,608,609,603 }, {415,416,420 ,603,602,608 }, {421,400,400 ,249,249,249 },
        {400,417,421 ,579,604,610 }, {422,421,417 ,611,612,606 }, {417,418,422 ,606,605,611 },
        {423,422,418 ,613,611,605 }, {418,419,423 ,605,607,613 }, {424,423,419 ,614,615,609 },
        {419,420,424 ,609,608,614 }, {425,400,400 ,249,249,249 }, {400,421,425 ,579,610,616 },
        {426,425,421 ,617,618,612 }, {421,422,426 ,612,611,617 }, {427,426,422 ,619,617,611 },
        {422,423,427 ,611,613,619 }, {428,427,423 ,620,621,615 }, {423,424,428 ,615,614,620 },
        {429,400,400 ,249,249,249 }, {400,425,429 ,579,616,622 }, {430,429,425 ,623,624,618 },
        {425,426,430 ,618,617,623 }, {431,430,426 ,625,623,617 }, {426,427,431 ,617,619,625 },
        {432,431,427 ,626,627,621 }, {427,428,432 ,621,620,626 }, {433,400,400 ,249,249,249 },
        {400,429,433 ,579,622,628 }, {434,433,429 ,629,630,624 }, {429,430,434 ,624,623,629 },
        {435,434,430 ,631,629,623 }, {430,431,435 ,623,625,631 }, {436,435,431 ,632,633,627 },
        {431,432,436 ,627,626,632 }, {437,400,400 ,249,249,249 }, {400,433,437 ,579,628,634 },
        {438,437,433 ,635,636,630 }, {433,434,438 ,630,629,635 }, {439,438,434 ,637,635,629 },
        {434,435,439 ,629,631,637 }, {440,439,435 ,638,639,633 }, {435,436,440 ,633,632,638 },
        {441,400,400 ,640,249,249 }, {400,437,441 ,579,634,641 }, {442,441,437 ,642,640,636 },
        {437,438,442 ,636,635,642 }, {443,442,438 ,643,642,635 }, {438,439,443 ,635,637,643 },
        {444,443,439 ,644,645,639 }, {439,440,444 ,639,638,644 }, {445,400,400 ,646,249,249 },
        {400,441,445 ,579,641,647 }, {446,445,441 ,648,646,640 }, {441,442,446 ,640,642,648 },
        {447,446,442 ,649,648,642 }, {442,443,447 ,642,643,649 }, {448,447,443 ,650,651,645 },
        {443,444,448 ,645,644,650 }, {449,400,400 ,652,249,249 }, {400,445,449 ,579,647,653 },
        {450,449,445 ,654,652,646 }, {445,446,450 ,646,648,654 }, {451,450,446 ,655,654,648 },
        {446,447,451 ,648,649,655 }, {452,451,447 ,656,657,651 }, {447,448,452 ,651,650,656 },
        {453,400,400 ,658,249,249 }, {400,449,453 ,579,653,659 }, {454,453,449 ,660,658,652 },
        {449,450,454 ,652,654,660 }, {455,454,450 ,661,660,654 }, {450,451,455 ,654,655,661 },
        {456,455,451 ,662,663,657 }, {451,452,456 ,657,656,662 }, {457,400,400 ,664,249,249 },
        {400,453,457 ,579,659,665 }, {458,457,453 ,666,664,658 }, {453,454,458 ,658,660,666 },
        {459,458,454 ,667,666,660 }, {454,455,459 ,660,661,667 }, {460,459,455 ,668,669,663 },
        {455,456,460 ,663,662,668 }, {461,400,400 ,249,249,249 }, {400,457,461 ,579,665,670 },
        {462,461,457 ,671,672,664 }, {457,458,462 ,664,666,671 }, {463,462,458 ,673,671,666 },
        {458,459,463 ,666,667,673 }, {464,463,459 ,674,675,669 }, {459,460,464 ,669,668,674 },
        {401,400,400 ,249,249,249 }, {400,461,401 ,579,670,580 }, {402,401,461 ,585,584,672 },
        {461,462,402 ,672,671,585 }, {403,402,462 ,587,585,671 }, {462,463,403 ,671,673,587 },
        {404,403,463 ,591,590,675 }, {463,464,404 ,675,674,591 }, {469,408,404 ,676,677,678 },
        {404,465,469 ,678,679,676 }, {470,469,465 ,680,676,679 }, {465,466,470 ,679,681,680 },
        {471,470,466 ,682,680,681 }, {466,467,471 ,681,683,682 }, {472,471,467 ,684,682,683 },
        {467,468,472 ,683,685,684 }, {473,412,408 ,686,687,677 }, {408,469,473 ,677,676,686 },
        {474,473,469 ,688,686,676 }, {469,470,474 ,676,680,688 }, {475,474,470 ,689,688,680 },
        {470,471,475 ,680,682,689 }, {476,475,471 ,690,689,682 }, {471,472,476 ,682,684,690 },
        {477,416,412 ,691,692,687 }, {412,473,477 ,687,686,691 }, {478,477,473 ,693,691,686 },
        {473,474,478 ,686,688,693 }, {479,478,474 ,694,693,688 }, {474,475,479 ,688,689,694 },
        {480,479,475 ,695,694,689 }, {475,476,480 ,689,690,695 }, {481,420,416 ,696,697,692 },
        {416,477,481 ,692,691,696 }, {482,481,477 ,698,696,691 }, {477,478,482 ,691,693,698 },
        {483,482,478 ,699,698,693 }, {478,479,483 ,693,694,699 }, {484,483,479 ,700,699,694 },
        {479,480,484 ,694,695,700 }, {485,424,420 ,701,702,697 }, {420,481,485 ,697,696,701 },
        {486,485,481 ,703,701,696 }, {481,482,486 ,696,698,703 }, {487,486,482 ,704,703,698 },
        {482,483,487 ,698,699,704 }, {488,487,483 ,705,704,699 }, {483,484,488 ,699,700,705 },
        {489,428,424 ,706,707,702 }, {424,485,489 ,702,701,706 }, {490,489,485 ,708,706,701 },
        {485,486,490 ,701,703,708 }, {491,490,486 ,709,708,703 }, {486,487,491 ,703,704,709 },
        {492,491,487 ,710,709,704 }, {487,488,492 ,704,705,710 }, {493,432,428 ,711,712,707 },
        {428,489,493 ,707,706,711 }, {494,493,489 ,713,711,706 }, {489,490,494 ,706,708,713 },
        {495,494,490 ,714,713,708 }, {490,491,495 ,708,709,714 }, {496,495,491 ,715,714,709 },
        {491,492,496 ,709,710,715 }, {497,436,432 ,716,717,712 }, {432,493,497 ,712,711,716 },
        {498,497,493 ,718,716,711 }, {493,494,498 ,711,713,718 }, {499,498,494 ,719,718,713 },
        {494,495,499 ,713,714,719 }, {500,499,495 ,720,719,714 }, {495,496,500 ,714,715,720 },
        {501,440,436 ,721,722,717 }, {436,497,501 ,717,716,721 }, {502,501,497 ,723,721,716 },
        {497,498,502 ,716,718,723 }, {503,502,498 ,724,723,718 }, {498,499,503 ,718,719,724 },
        {504,503,499 ,725,724,719 }, {499,500,504 ,719,720,725 }, {505,444,440 ,726,727,722 },
        {440,501,505 ,722,721,726 }, {506,505,501 ,728,726,721 }, {501,502,506 ,721,723,728 },
        {507,506,502 ,729,728,723 }, {502,503,507 ,723,724,729 }, {508,507,503 ,730,729,724 },
        {503,504,508 ,724,725,730 }, {509,448,444 ,731,732,727 }, {444,505,509 ,727,726,731 },
        {510,509,505 ,733,731,726 }, {505,506,510 ,726,728,733 }, {511,510,506 ,734,733,728 },
        {506,507,511 ,728,729,734 }, {512,511,507 ,735,734,729 }, {507,508,512 ,729,730,735 },
        {513,452,448 ,736,737,732 }, {448,509,513 ,732,731,736 }, {514,513,509 ,738,736,731 },
        {509,510,514 ,731,733,738 }, {515,514,510 ,739,738,733 }, {510,511,515 ,733,734,739 },
        {516,515,511 ,740,739,734 }, {511,512,516 ,734,735,740 }, {517,456,452 ,741,742,737 },
        {452,513,517 ,737,736,741 }, {518,517,513 ,743,741,736 }, {513,514,518 ,736,738,743 },
        {519,518,514 ,744,743,738 }, {514,515,519 ,738,739,744 }, {520,519,515 ,745,744,739 },
        {515,516,520 ,739,740,745 }, {521,460,456 ,746,747,742 }, {456,517,521 ,742,741,746 },
        {522,521,517 ,748,746,741 }, {517,518,522 ,741,743,748 }, {523,522,518 ,749,748,743 },
        {518,519,523 ,743,744,749 }, {524,523,519 ,750,749,744 }, {519,520,524 ,744,745,750 },
        {525,464,460 ,751,752,747 }, {460,521,525 ,747,746,751 }, {526,525,521 ,753,751,746 },
        {521,522,526 ,746,748,753 }, {527,526,522 ,754,753,748 }, {522,523,527 ,748,749,754 },
        {528,527,523 ,755,754,749 }, {523,524,528 ,749,750,755 }, {465,404,464 ,679,678,752 },
        {464,525,465 ,752,751,679 }, {466,465,525 ,681,679,751 }, {525,526,466 ,751,753,681 },
        {467,466,526 ,683,681,753 }, {526,527,467 ,753,754,683 }, {468,467,527 ,685,683,754 },
        {527,528,468 ,754,755,685 }
    };
    static GLfloat vertices [529][3] = {
    {0.184492f,0.128343f,4.11017e-008f},{0.181454f,0.139828f,4.11017e-008f},{0.184979f,0.143656f,4.11017e-008f},
    {0.192149f,0.139828f,4.11017e-008f},{0.200049f,0.128343f,4.11017e-008f},{0.167613f,0.128343f,0.0854836f},
    {0.16481f,0.139828f,0.0842911f},{0.168062f,0.143656f,0.0856746f},{0.174677f,0.139828f,0.0884892f},
    {0.181964f,0.128343f,0.0915896f},{0.121332f,0.128343f,0.154633f},{0.119175f,0.139828f,0.152476f},
    {0.121678f,0.143656f,0.154978f},{0.126768f,0.139828f,0.160069f},{0.132377f,0.128343f,0.165678f},
    {0.052183f,0.128343f,0.200914f},{0.0509905f,0.139828f,0.198111f},{0.052374f,0.143656f,0.201362f},
    {0.0551881f,0.139828f,0.207977f},{0.058289f,0.128343f,0.215265f},{-0.0333007f,0.128343f,0.217793f},
    {-0.0333007f,0.139828f,0.214755f},{-0.0333007f,0.143656f,0.218279f},{-0.0333007f,0.139828f,0.22545f},
    {-0.0333007f,0.128343f,0.23335f},{-0.124691f,0.128343f,0.200914f},{-0.120084f,0.139828f,0.198111f},
    {-0.119713f,0.143656f,0.201362f},{-0.121882f,0.139828f,0.207977f},{-0.12489f,0.128343f,0.215265f},
    {-0.193184f,0.128343f,0.154633f},{-0.187991f,0.139828f,0.152476f},{-0.188935f,0.143656f,0.154978f},
    {-0.193452f,0.139828f,0.160069f},{-0.198979f,0.128343f,0.165678f},{-0.236184f,0.128343f,0.0854836f},
    {-0.232242f,0.139828f,0.0842911f},{-0.234909f,0.143656f,0.0856746f},{-0.241309f,0.139828f,0.0884892f},
    {-0.248566f,0.128343f,0.0915896f},{-0.251094f,0.128343f,4.11017e-008f},{-0.248055f,0.139828f,4.11017e-008f},
    {-0.25158f,0.143656f,4.11017e-008f},{-0.25875f,0.139828f,4.11017e-008f},{-0.26665f,0.128343f,4.11017e-008f},
    {-0.234215f,0.128343f,-0.0854836f},{-0.231412f,0.139828f,-0.0842911f},{-0.234663f,0.143656f,-0.0856746f},
    {-0.241278f,0.139828f,-0.0884887f},{-0.248566f,0.128343f,-0.0915896f},{-0.187933f,0.128343f,-0.154633f},
    {-0.185777f,0.139828f,-0.152475f},{-0.188279f,0.143656f,-0.154978f},{-0.19337f,0.139828f,-0.160069f},
    {-0.198979f,0.128343f,-0.165678f},{-0.118784f,0.128343f,-0.200914f},{-0.117592f,0.139828f,-0.198111f},
    {-0.118975f,0.143656f,-0.201362f},{-0.12179f,0.139828f,-0.207977f},{-0.12489f,0.128343f,-0.215265f},
    {-0.0333007f,0.128343f,-0.217793f},{-0.0333007f,0.139828f,-0.214754f},{-0.0333007f,0.143656f,-0.218279f},
    {-0.0333007f,0.139828f,-0.22545f},{-0.0333007f,0.128343f,-0.23335f},{0.052183f,0.128343f,-0.200914f},
    {0.0509905f,0.139828f,-0.198111f},{0.052374f,0.143656f,-0.201362f},{0.0551881f,0.139828f,-0.207977f},
    {0.058289f,0.128343f,-0.215265f},{0.121332f,0.128343f,-0.154633f},{0.119175f,0.139828f,-0.152475f},
    {0.121678f,0.143656f,-0.154978f},{0.126768f,0.139828f,-0.160069f},{0.132377f,0.128343f,-0.165678f},
    {0.167613f,0.128343f,-0.0854836f},{0.16481f,0.139828f,-0.0842911f},{0.168062f,0.143656f,-0.0856746f},
    {0.174677f,0.139828f,-0.0884887f},{0.181964f,0.128343f,-0.0915896f},{0.22861f,0.0672707f,4.11017e-008f},
    {0.253525f,0.00729244f,4.11017e-008f},{0.271148f,-0.050498f,4.11017e-008f},{0.277832f,-0.105007f,4.11017e-008f},
    {0.208312f,0.0672707f,0.1028f},{0.231296f,0.00729244f,0.112579f},{0.247553f,-0.050498f,0.119496f},
    {0.253719f,-0.105007f,0.12212f},{0.152656f,0.0672707f,0.185956f},{0.170345f,0.00729244f,0.203646f},
    {0.182857f,-0.050498f,0.216158f},{0.187603f,-0.105007f,0.220904f},{0.0694989f,0.0672707f,0.241612f},
    {0.0792783f,0.00729244f,0.264596f},{0.0861953f,-0.050498f,0.280853f},{0.088819f,-0.105007f,0.28702f},
    {-0.0333007f,0.0672707f,0.26191f},{-0.0333007f,0.00729244f,0.286825f},{-0.0333007f,-0.050498f,0.304448f},
    {-0.0333007f,-0.105007f,0.311133f},{-0.136101f,0.0672707f,0.241612f},{-0.14588f,0.00729244f,0.264596f},
    {-0.152796f,-0.050498f,0.280853f},{-0.15542f,-0.105007f,0.28702f},{-0.219257f,0.0672707f,0.185956f},
    {-0.236947f,0.00729244f,0.203646f},{-0.249459f,-0.050498f,0.216158f},{-0.254205f,-0.105007f,0.220904f},
    {-0.274913f,0.0672707f,0.1028f},{-0.297897f,0.00729244f,0.112579f},{-0.314154f,-0.050498f,0.119496f},
    {-0.320321f,-0.105007f,0.12212f},{-0.295211f,0.0672707f,4.11017e-008f},{-0.320126f,0.00729244f,4.11017e-008f},
    {-0.337749f,-0.050498f,4.11017e-008f},{-0.344433f,-0.105007f,4.11017e-008f},{-0.274913f,0.0672707f,-0.1028f},
    {-0.297897f,0.00729244f,-0.112579f},{-0.314154f,-0.050498f,-0.119496f},{-0.320321f,-0.105007f,-0.12212f},
    {-0.219257f,0.0672707f,-0.185956f},{-0.236947f,0.00729244f,-0.203646f},{-0.249459f,-0.050498f,-0.216158f},
    {-0.254205f,-0.105007f,-0.220904f},{-0.136101f,0.0672707f,-0.241612f},{-0.14588f,0.00729244f,-0.264596f},
    {-0.152796f,-0.050498f,-0.280853f},{-0.15542f,-0.105007f,-0.28702f},{-0.0333007f,0.0672707f,-0.26191f},
    {-0.0333007f,0.00729244f,-0.286825f},{-0.0333007f,-0.050498f,-0.304448f},{-0.0333007f,-0.105007f,-0.311133f},
    {0.0694989f,0.0672707f,-0.241612f},{0.0792783f,0.00729244f,-0.264596f},{0.0861953f,-0.050498f,-0.280853f},
    {0.088819f,-0.105007f,-0.28702f},{0.152656f,0.0672707f,-0.185956f},{0.170345f,0.00729244f,-0.203646f},
    {0.182857f,-0.050498f,-0.216158f},{0.187603f,-0.105007f,-0.220904f},{0.208312f,0.0672707f,-0.1028f},
    {0.231296f,0.00729244f,-0.112579f},{0.247553f,-0.050498f,-0.119496f},{0.253719f,-0.105007f,-0.12212f},
    {0.265678f,-0.15113f,4.11017e-008f},{0.23894f,-0.185221f,4.11017e-008f},{0.212202f,-0.208373f,4.11017e-008f},
    {0.200049f,-0.221682f,4.11017e-008f},{0.242507f,-0.15113f,0.117349f},{0.217842f,-0.185221f,0.106855f},
    {0.193176f,-0.208373f,0.0963602f},{0.181964f,-0.221682f,0.0915896f},{0.178974f,-0.15113f,0.212275f},
    {0.15999f,-0.185221f,0.193292f},{0.141006f,-0.208373f,0.174307f},{0.132377f,-0.221682f,0.165678f},
    {0.0840483f,-0.15113f,0.275808f},{0.0735537f,-0.185221f,0.251143f},{0.0630591f,-0.208373f,0.226477f},
    {0.058289f,-0.221682f,0.215265f},{-0.0333007f,-0.15113f,0.298979f},{-0.0333007f,-0.185221f,0.272241f},
    {-0.0333007f,-0.208373f,0.245503f},{-0.0333007f,-0.221682f,0.23335f},{-0.15065f,-0.15113f,0.275808f},
    {-0.140155f,-0.185221f,0.251143f},{-0.129661f,-0.208373f,0.226477f},{-0.12489f,-0.221682f,0.215265f},
    {-0.245576f,-0.15113f,0.212275f},{-0.226592f,-0.185221f,0.193292f},{-0.207608f,-0.208373f,0.174307f},
    {-0.198979f,-0.221682f,0.165678f},{-0.309109f,-0.15113f,0.117349f},{-0.284443f,-0.185221f,0.106855f},
    {-0.259777f,-0.208373f,0.0963602f},{-0.248566f,-0.221682f,0.0915896f},{-0.33228f,-0.15113f,4.11017e-008f},
    {-0.305542f,-0.185221f,4.11017e-008f},{-0.278804f,-0.208373f,4.11017e-008f},{-0.26665f,-0.221682f,4.11017e-008f},
    {-0.309109f,-0.15113f,-0.117349f},{-0.284443f,-0.185221f,-0.106854f},{-0.259777f,-0.208373f,-0.0963597f},
    {-0.248566f,-0.221682f,-0.0915896f},{-0.245576f,-0.15113f,-0.212275f},{-0.226592f,-0.185221f,-0.193291f},
    {-0.207608f,-0.208373f,-0.174307f},{-0.198979f,-0.221682f,-0.165678f},{-0.15065f,-0.15113f,-0.275808f},
    {-0.140155f,-0.185221f,-0.251142f},{-0.129661f,-0.208373f,-0.226476f},{-0.12489f,-0.221682f,-0.215265f},
    {-0.0333007f,-0.15113f,-0.298979f},{-0.0333007f,-0.185221f,-0.272241f},{-0.0333007f,-0.208373f,-0.245503f},
    {-0.0333007f,-0.221682f,-0.23335f},{0.0840483f,-0.15113f,-0.275808f},{0.0735537f,-0.185221f,-0.251142f},
    {0.0630591f,-0.208373f,-0.226476f},{0.058289f,-0.221682f,-0.215265f},{0.178974f,-0.15113f,-0.212275f},
    {0.15999f,-0.185221f,-0.193291f},{0.141006f,-0.208373f,-0.174307f},{0.132377f,-0.221682f,-0.165678f},
    {0.242507f,-0.15113f,-0.117349f},{0.217842f,-0.185221f,-0.106854f},{0.193176f,-0.208373f,-0.0963597f},
    {0.181964f,-0.221682f,-0.0915896f},{0.194762f,-0.23025f,4.11017e-008f},{0.166505f,-0.237725f,4.11017e-008f},
    {0.0966823f,-0.243011f,4.11017e-008f},{-0.0333007f,-0.245017f,4.11017e-008f},{0.177087f,-0.23025f,0.0895148f},
    {0.15102f,-0.237725f,0.0784237f},{0.0866085f,-0.243011f,0.0510184f},{0.128624f,-0.23025f,0.161925f},
    {0.108561f,-0.237725f,0.141862f},{0.0589873f,-0.243011f,0.0922879f},{0.0562137f,-0.23025f,0.210388f},
    {0.0451231f,-0.237725f,0.18432f},{0.0177176f,-0.243011f,0.11991f},{-0.0333007f,-0.23025f,0.228063f},
    {-0.0333007f,-0.237725f,0.199806f},{-0.0333007f,-0.243011f,0.129983f},{-0.122815f,-0.23025f,0.210388f},
    {-0.111724f,-0.237725f,0.18432f},{-0.084319f,-0.243011f,0.11991f},{-0.195225f,-0.23025f,0.161925f},
    {-0.175163f,-0.237725f,0.141862f},{-0.125588f,-0.243011f,0.0922879f},{-0.243689f,-0.23025f,0.0895148f},
    {-0.217621f,-0.237725f,0.0784237f},{-0.15321f,-0.243011f,0.0510184f},{-0.261363f,-0.23025f,4.11017e-008f},
    {-0.233106f,-0.237725f,4.11017e-008f},{-0.163284f,-0.243011f,4.11017e-008f},{-0.243689f,-0.23025f,-0.0895143f},
    {-0.217621f,-0.237725f,-0.0784237f},{-0.15321f,-0.243011f,-0.0510182f},{-0.195225f,-0.23025f,-0.161924f},
    {-0.175163f,-0.237725f,-0.141862f},{-0.125588f,-0.243011f,-0.0922879f},{-0.122815f,-0.23025f,-0.210388f},
    {-0.111724f,-0.237725f,-0.184321f},{-0.084319f,-0.243011f,-0.119909f},{-0.0333007f,-0.23025f,-0.228063f},
    {-0.0333007f,-0.237725f,-0.199805f},{-0.0333007f,-0.243011f,-0.129983f},{0.0562137f,-0.23025f,-0.210388f},
    {0.0451231f,-0.237725f,-0.184321f},{0.0177176f,-0.243011f,-0.119909f},{0.128624f,-0.23025f,-0.161924f},
    {0.108561f,-0.237725f,-0.141862f},{0.0589873f,-0.243011f,-0.0922879f},{0.177087f,-0.23025f,-0.0895143f},
    {0.15102f,-0.237725f,-0.0784237f},{0.0866085f,-0.243011f,-0.0510182f},{-0.282207f,0.0700049f,4.11017e-008f},
    {-0.354886f,0.0694582f,4.11017e-008f},{-0.408604f,0.06563f,4.11017e-008f},{-0.441905f,0.0552382f,4.11017e-008f},
    {-0.45333f,0.0350026f,4.11017e-008f},{-0.279776f,0.0754744f,0.0262519f},{-0.35705f,0.0748418f,0.0262519f},
    {-0.41377f,0.0704154f,0.0262519f},{-0.448704f,0.0584004f,0.0262519f},{-0.460622f,0.0350026f,0.0262519f},
    {-0.274429f,0.0875063f,0.0350025f},{-0.361813f,0.086686f,0.0350025f},{-0.425133f,0.0809434f,0.0350025f},
    {-0.463661f,0.0653563f,0.0350025f},{-0.476665f,0.0350026f,0.0350025f},{-0.269081f,0.0995383f,0.0262519f},
    {-0.366576f,0.0985302f,0.0262519f},{-0.436497f,0.0914714f,0.0262519f},{-0.478617f,0.0723122f,0.0262519f},
    {-0.492708f,0.0350026f,0.0262519f},{-0.26665f,0.105008f,4.11017e-008f},{-0.368741f,0.103914f,4.11017e-008f},
    {-0.441662f,0.0962568f,4.11017e-008f},{-0.485416f,0.0754744f,4.11017e-008f},{-0.5f,0.0350026f,4.11017e-008f},
    {-0.269081f,0.0995383f,-0.0262518f},{-0.366576f,0.0985302f,-0.0262518f},{-0.436497f,0.0914714f,-0.0262518f},
    {-0.478617f,0.0723122f,-0.0262518f},{-0.492708f,0.0350026f,-0.0262518f},{-0.274429f,0.0875063f,-0.0350024f},
    {-0.361813f,0.086686f,-0.0350024f},{-0.425133f,0.0809434f,-0.0350024f},{-0.463661f,0.0653563f,-0.0350024f},
    {-0.476665f,0.0350026f,-0.0350024f},{-0.279776f,0.0754744f,-0.0262518f},{-0.35705f,0.0748418f,-0.0262518f},
    {-0.41377f,0.0704154f,-0.0262518f},{-0.448704f,0.0584004f,-0.0262518f},{-0.460622f,0.0350026f,-0.0262518f},
    {-0.447253f,0.00328176f,4.11017e-008f},{-0.42805f,-0.0350022f,4.11017e-008f},{-0.394263f,-0.0732861f,4.11017e-008f},
    {-0.453881f,0.000219572f,0.0262519f},{-0.43276f,-0.0396737f,0.0262519f},{-0.395915f,-0.0790543f,0.0262519f},
    {-0.342003f,-0.112299f,0.0262519f},{-0.468461f,-0.0065171f,0.0350025f},{-0.443121f,-0.0499511f,0.0350025f},
    {-0.39955f,-0.0917445f,0.0350025f},{-0.336655f,-0.128342f,0.0350025f},{-0.483041f,-0.0132538f,0.0262519f},
    {-0.453482f,-0.0602285f,0.0262519f},{-0.403185f,-0.104434f,0.0262519f},{-0.331307f,-0.144385f,0.0262519f},
    {-0.489669f,-0.016316f,4.11017e-008f},{-0.458191f,-0.0649001f,4.11017e-008f},{-0.404837f,-0.110203f,4.11017e-008f},
    {-0.328877f,-0.151677f,4.11017e-008f},{-0.483041f,-0.0132538f,-0.0262518f},{-0.453482f,-0.0602285f,-0.0262518f},
    {-0.403185f,-0.104434f,-0.0262518f},{-0.331307f,-0.144385f,-0.0262518f},{-0.468461f,-0.0065171f,-0.0350024f},
    {-0.443121f,-0.0499511f,-0.0350024f},{-0.39955f,-0.0917445f,-0.0350024f},{-0.336655f,-0.128342f,-0.0350024f},
    {-0.453881f,0.000219572f,-0.0262518f},{-0.43276f,-0.0396737f,-0.0262518f},{-0.395915f,-0.0790543f,-0.0262518f},
    {-0.342003f,-0.112299f,-0.0262518f},{0.231162f,-0.0233347f,4.11017e-008f},{0.305786f,-0.00619809f,4.11017e-008f},
    {0.338114f,0.0350026f,4.11017e-008f},{0.355858f,0.0849538f,4.11017e-008f},{0.386729f,0.128343f,4.11017e-008f},
    {0.231162f,-0.0433882f,0.0577541f},{0.311482f,-0.0210673f,0.0521482f},{0.345406f,0.0263432f,0.0398153f},
    {0.364745f,0.0818207f,0.0274825f},{0.401313f,0.128343f,0.0218766f},{0.231162f,-0.0875058f,0.0770057f},
    {0.324016f,-0.0537795f,0.0695309f},{0.361449f,0.00729244f,0.0530871f},{0.384298f,0.0749271f,0.0366433f},
    {0.433398f,0.128343f,0.0291687f},{0.231162f,-0.131624f,0.0577541f},{0.336549f,-0.0864917f,0.0521482f},
    {0.377492f,-0.0117583f,0.0398153f},{0.40385f,0.0680342f,0.0274825f},{0.465484f,0.128343f,0.0218766f},
    {0.231162f,-0.151677f,4.11017e-008f},{0.342246f,-0.101361f,4.11017e-008f},{0.384784f,-0.0204178f,4.11017e-008f},
    {0.412737f,0.0649004f,4.11017e-008f},{0.480068f,0.128343f,4.11017e-008f},{0.231162f,-0.131624f,-0.0577541f},
    {0.336549f,-0.0864917f,-0.0521483f},{0.377492f,-0.0117583f,-0.0398152f},{0.40385f,0.0680342f,-0.0274823f},
    {0.465484f,0.128343f,-0.0218764f},{0.231162f,-0.0875058f,-0.0770052f},{0.324016f,-0.0537795f,-0.069531f},
    {0.361449f,0.00729244f,-0.0530869f},{0.384298f,0.0749271f,-0.0366431f},{0.433398f,0.128343f,-0.0291686f},
    {0.231162f,-0.0433882f,-0.0577541f},{0.311482f,-0.0210673f,-0.0521483f},{0.345406f,0.0263432f,-0.0398152f},
    {0.364745f,0.0818207f,-0.0274823f},{0.401313f,0.128343f,-0.0218764f},{0.39791f,0.134906f,4.11017e-008f},
    {0.406174f,0.137093f,4.11017e-008f},{0.408605f,0.134906f,4.11017e-008f},{0.402285f,0.128343f,4.11017e-008f},
    {0.413529f,0.135226f,0.0205093f},{0.420834f,0.137606f,0.0175013f},{0.421053f,0.135354f,0.0144932f},
    {0.412008f,0.128343f,0.013126f},{0.447891f,0.135931f,0.0273457f},{0.453087f,0.138734f,0.023335f},
    {0.448439f,0.136341f,0.0193243f},{0.433398f,0.128343f,0.0175013f},{0.482254f,0.136636f,0.0205093f},
    {0.48534f,0.139862f,0.0175013f},{0.475824f,0.137328f,0.0144932f},{0.454789f,0.128343f,0.013126f},
    {0.497873f,0.136956f,4.11017e-008f},{0.5f,0.140375f,4.11017e-008f},{0.488272f,0.137777f,4.11017e-008f},
    {0.464512f,0.128343f,4.11017e-008f},{0.482254f,0.136636f,-0.0205092f},{0.48534f,0.139862f,-0.0175012f},
    {0.475824f,0.137328f,-0.0144931f},{0.454789f,0.128343f,-0.0131258f},{0.447891f,0.135931f,-0.0273456f},
    {0.453087f,0.138734f,-0.0233349f},{0.448439f,0.136341f,-0.0193242f},{0.433398f,0.128343f,-0.0175012f},
    {0.413529f,0.135226f,-0.0205092f},{0.420834f,0.137606f,-0.0175012f},{0.421053f,0.135354f,-0.0144931f},
    {0.412008f,0.128343f,-0.0131258f},{-0.0333007f,0.245017f,4.11017e-008f},{0.0196889f,0.237361f,4.11017e-008f},
    {0.0172584f,0.218766f,4.11017e-008f},{-0.00267358f,0.195795f,4.11017e-008f},{-0.00218743f,0.175012f,4.11017e-008f},
    {0.0156008f,0.237361f,0.0208539f},{0.0133564f,0.218766f,0.0198937f},{-0.005041f,0.195795f,0.0120397f},
    {-0.00459869f,0.175012f,0.012212f},{0.00437127f,0.237361f,0.037672f},{0.00264001f,0.218766f,0.0359408f},
    {-0.0115391f,0.195795f,0.0217617f},{-0.0112103f,0.175012f,0.0220905f},{-0.0124468f,0.237361f,0.0489016f},
    {-0.013407f,0.218766f,0.0466572f},{-0.0212611f,0.195795f,0.0282598f},{-0.0210887f,0.175012f,0.0287021f},
    {-0.0333007f,0.237361f,0.0529899f},{-0.0333007f,0.218766f,0.0505591f},{-0.0333007f,0.195795f,0.0306272f},
    {-0.0333007f,0.175012f,0.0311133f},{-0.0541546f,0.237361f,0.0489016f},{-0.0531944f,0.218766f,0.0466572f},
    {-0.0453403f,0.195795f,0.0282598f},{-0.0455127f,0.175012f,0.0287021f},{-0.0709727f,0.237361f,0.037672f},
    {-0.0692414f,0.218766f,0.0359408f},{-0.0550623f,0.195795f,0.0217617f},{-0.0553911f,0.175012f,0.0220905f},
    {-0.0822022f,0.237361f,0.0208539f},{-0.0799578f,0.218766f,0.0198937f},{-0.0615604f,0.195795f,0.0120397f},
    {-0.0620027f,0.175012f,0.012212f},{-0.0862905f,0.237361f,4.11017e-008f},{-0.0838598f,0.218766f,4.11017e-008f},
    {-0.0639278f,0.195795f,4.11017e-008f},{-0.064414f,0.175012f,4.11017e-008f},{-0.0822022f,0.237361f,-0.0208538f},
    {-0.0799578f,0.218766f,-0.0198936f},{-0.0615604f,0.195795f,-0.0120395f},{-0.0620027f,0.175012f,-0.0122119f},
    {-0.0709727f,0.237361f,-0.0376719f},{-0.0692414f,0.218766f,-0.0359406f},{-0.0550623f,0.195795f,-0.0217616f},
    {-0.0553911f,0.175012f,-0.0220904f},{-0.0541546f,0.237361f,-0.0489013f},{-0.0531944f,0.218766f,-0.0466571f},
    {-0.0453403f,0.195795f,-0.0282596f},{-0.0455127f,0.175012f,-0.0287019f},{-0.0333007f,0.237361f,-0.0529895f},
    {-0.0333007f,0.218766f,-0.050559f},{-0.0333007f,0.195795f,-0.0306271f},{-0.0333007f,0.175012f,-0.0311132f},
    {-0.0124468f,0.237361f,-0.0489013f},{-0.013407f,0.218766f,-0.0466571f},{-0.0212611f,0.195795f,-0.0282596f},
    {-0.0210887f,0.175012f,-0.0287019f},{0.00437127f,0.237361f,-0.0376719f},{0.00264001f,0.218766f,-0.0359406f},
    {-0.0115391f,0.195795f,-0.0217616f},{-0.0112103f,0.175012f,-0.0220904f},{0.0156008f,0.237361f,-0.0208538f},
    {0.0133564f,0.218766f,-0.0198936f},{-0.005041f,0.195795f,-0.0120395f},{-0.00459869f,0.175012f,-0.0122119f},
    {0.0376763f,0.161157f,4.11017e-008f},{0.0950415f,0.151678f,4.11017e-008f},{0.146573f,0.142198f,4.11017e-008f},
    {0.168936f,0.128343f,4.11017e-008f},{0.0321756f,0.161157f,0.0278586f},{0.0850953f,0.151678f,0.0503744f},
    {0.132632f,0.142198f,0.0706003f},{0.153262f,0.128343f,0.0793776f},{0.0170931f,0.161157f,0.0503939f},
    {0.0578222f,0.151678f,0.0911233f},{0.0944094f,0.142198f,0.127711f},{0.110287f,0.128343f,0.143588f},
    {-0.00544215f,0.161157f,0.0654767f},{0.0170734f,0.151678f,0.118396f},{0.0372998f,0.142198f,0.165934f},
    {0.046077f,0.128343f,0.186563f},{-0.0333007f,0.161157f,0.0709774f},{-0.0333007f,0.151678f,0.128342f},
    {-0.0333007f,0.142198f,0.179874f},{-0.0333007f,0.128343f,0.202236f},{-0.0611592f,0.161157f,0.0654767f},
    {-0.083675f,0.151678f,0.118396f},{-0.103901f,0.142198f,0.165934f},{-0.112678f,0.128343f,0.186563f},
    {-0.0836945f,0.161157f,0.0503939f},{-0.124424f,0.151678f,0.0911233f},{-0.161011f,0.142198f,0.127711f},
    {-0.176888f,0.128343f,0.143588f},{-0.0987773f,0.161157f,0.0278586f},{-0.151696f,0.151678f,0.0503744f},
    {-0.199234f,0.142198f,0.0706003f},{-0.219864f,0.128343f,0.0793776f},{-0.104278f,0.161157f,4.11017e-008f},
    {-0.161643f,0.151678f,4.11017e-008f},{-0.213174f,0.142198f,4.11017e-008f},{-0.235537f,0.128343f,4.11017e-008f},
    {-0.0987773f,0.161157f,-0.0278585f},{-0.151696f,0.151678f,-0.050374f},{-0.199234f,0.142198f,-0.0706004f},
    {-0.219864f,0.128343f,-0.0793776f},{-0.0836945f,0.161157f,-0.0503937f},{-0.124424f,0.151678f,-0.0911228f},
    {-0.161011f,0.142198f,-0.12771f},{-0.176888f,0.128343f,-0.143588f},{-0.0611592f,0.161157f,-0.0654762f},
    {-0.083675f,0.151678f,-0.118396f},{-0.103901f,0.142198f,-0.165933f},{-0.112678f,0.128343f,-0.186563f},
    {-0.0333007f,0.161157f,-0.0709769f},{-0.0333007f,0.151678f,-0.128342f},{-0.0333007f,0.142198f,-0.179873f},
    {-0.0333007f,0.128343f,-0.202236f},{-0.00544215f,0.161157f,-0.0654762f},{0.0170734f,0.151678f,-0.118396f},
    {0.0372998f,0.142198f,-0.165933f},{0.046077f,0.128343f,-0.186563f},{0.0170931f,0.161157f,-0.0503937f},
    {0.0578222f,0.151678f,-0.0911228f},{0.0944094f,0.142198f,-0.12771f},{0.110287f,0.128343f,-0.143588f},
    {0.0321756f,0.161157f,-0.0278585f},{0.0850953f,0.151678f,-0.050374f},{0.132632f,0.142198f,-0.0706004f},
    {0.153262f,0.128343f,-0.0793776f}
    };
    static GLfloat normals [756][3] = {
    {0.915916f,0.255667f,0.309406f},{0.866653f,0.255977f,0.428239f},{0.964789f,0.255245f,0.0634993f},
    {0.964789f,0.255245f,-0.0634985f},{0.697135f,-0.677193f,0.235397f},{0.659316f,-0.677671f,0.325675f},
    {0.734792f,-0.676566f,0.0483649f},{0.734791f,-0.676567f,-0.0483634f},{-0.609662f,-0.747029f,-0.265066f},
    {-0.421971f,-0.882337f,-0.20838f},{-0.470739f,-0.881728f,-0.0309823f},{-0.665532f,-0.746276f,-0.0117622f},
    {-0.800361f,-0.497846f,-0.334023f},{-0.867771f,-0.496957f,-0.00262826f},{0.727059f,0.256199f,0.636983f},
    {0.636981f,0.256199f,0.727061f},{0.552868f,-0.678027f,0.484372f},{0.48437f,-0.678027f,0.55287f},
    {-0.461293f,-0.747477f,-0.478003f},{-0.309895f,-0.882524f,-0.353719f},{-0.611123f,-0.498459f,-0.614872f},
    {0.428241f,0.255972f,0.866653f},{0.309411f,0.255667f,0.915915f},{0.325679f,-0.677664f,0.659321f},
    {0.235402f,-0.677173f,0.697153f},{-0.243044f,-0.747009f,-0.618795f},{-0.15067f,-0.882068f,-0.446379f},
    {-0.329079f,-0.497894f,-0.802377f},{0.0652336f,0.255217f,0.964681f},{-0.0569109f,0.27122f,0.960833f},
    {0.0487659f,-0.676515f,0.734812f},{-0.0487111f,-0.663956f,0.746184f},{0.0117389f,-0.746154f,-0.66567f},
    {0.0311277f,-0.880511f,-0.473003f},{0.00262837f,-0.496957f,-0.867771f},{-0.29489f,0.319838f,0.900413f},
    {-0.408656f,0.36266f,0.837543f},{-0.255021f,-0.624505f,0.738213f},{-0.362402f,-0.587522f,0.723521f},
    {0.267119f,-0.742968f,-0.613715f},{0.215491f,-0.874545f,-0.434437f},{0.334163f,-0.497243f,-0.800677f},
    {-0.604902f,0.407631f,0.684055f},{-0.704195f,0.386383f,0.595666f},{-0.558138f,-0.540808f,0.629292f},
    {-0.624408f,-0.564779f,0.539573f},{0.48488f,-0.738582f,-0.468388f},{0.367327f,-0.873071f,-0.320654f},
    {0.615439f,-0.497018f,-0.611725f},{-0.840898f,0.371275f,0.393757f},{-0.90746f,0.309695f,0.28391f},
    {-0.733123f,-0.577664f,0.358935f},{-0.732582f,-0.636187f,0.242054f},{0.625189f,-0.740685f,-0.246019f},
    {0.453086f,-0.8784f,-0.152071f},{0.802951f,-0.49681f,-0.329317f},{-0.957425f,0.284034f,0.0516016f},
    {-0.964334f,0.25511f,-0.0705648f},{-0.754704f,-0.654232f,0.049019f},{-0.734743f,-0.676448f,-0.0507067f},
    {0.667136f,-0.744844f,0.0116832f},{0.470734f,-0.881716f,0.0314264f},{0.867923f,-0.496692f,0.00264245f},
    {-0.915922f,0.255641f,-0.309411f},{-0.866659f,0.255947f,-0.428245f},{-0.697154f,-0.677172f,-0.235401f},
    {-0.65932f,-0.677665f,-0.32568f},{0.60966f,-0.747031f,0.265064f},{0.421975f,-0.882335f,0.208381f},
    {0.800362f,-0.497848f,0.33402f},{-0.727061f,0.256188f,-0.636985f},{-0.636982f,0.256199f,-0.72706f},
    {-0.552868f,-0.678027f,-0.484372f},{-0.484371f,-0.678028f,-0.552868f},{0.461293f,-0.747478f,0.478002f},
    {0.309892f,-0.882526f,0.353715f},{0.611118f,-0.498469f,0.614868f},{-0.42824f,0.255977f,-0.866653f},
    {-0.309407f,0.255667f,-0.915916f},{-0.325676f,-0.677671f,-0.659316f},{-0.235397f,-0.677193f,-0.697135f},
    {0.243046f,-0.747008f,0.618795f},{0.150666f,-0.882072f,0.446373f},{0.329078f,-0.497895f,0.802377f},
    {-0.063499f,0.255245f,-0.964789f},{0.0634989f,0.255245f,-0.964789f},{-0.0483637f,-0.676567f,-0.734791f},
    {0.0483645f,-0.676566f,-0.734792f},{-0.0117625f,-0.746278f,0.665531f},{-0.0309827f,-0.881727f,0.470741f},
    {-0.00262813f,-0.496956f,0.867772f},{0.309407f,0.255667f,-0.915916f},{0.428239f,0.255985f,-0.866651f},
    {0.235397f,-0.677193f,-0.697135f},{0.32567f,-0.677682f,-0.659307f},{-0.265065f,-0.747031f,0.60966f},
    {-0.208386f,-0.88233f,0.421982f},{-0.334022f,-0.497849f,0.80036f},{0.636979f,0.25622f,-0.727055f},
    {0.727055f,0.25622f,-0.636979f},{0.484354f,-0.678054f,-0.55285f},{0.552851f,-0.678054f,-0.484354f},
    {-0.478004f,-0.747475f,0.461296f},{-0.353728f,-0.882517f,0.309905f},{-0.614867f,-0.498469f,0.611119f},
    {0.86665f,0.255985f,-0.428239f},{0.915916f,0.255667f,-0.309407f},{0.659307f,-0.677682f,-0.325671f},
    {0.697135f,-0.677193f,-0.235398f},{-0.618797f,-0.747006f,0.243047f},{-0.446379f,-0.882068f,0.15067f},
    {-0.802374f,-0.497899f,0.329078f},{-0.844816f,-0.404552f,-0.350177f},{-0.914881f,-0.403724f,-0.000562311f},
    {-0.868963f,-0.338795f,-0.360724f},{-0.941117f,-0.338081f,-0.00105213f},{-0.903357f,-0.207975f,-0.37509f},
    {-0.978226f,-0.207541f,-0.001149f},{-0.922181f,0.0681314f,-0.380711f},{-0.997695f,0.0678541f,0.000826348f},
    {-0.646084f,-0.405106f,-0.646889f},{-0.664408f,-0.339299f,-0.665911f},{-0.690769f,-0.208343f,-0.69241f},
    {-0.706055f,0.068113f,-0.704873f},{-0.349116f,-0.40457f,-0.845246f},{-0.358741f,-0.338844f,-0.869764f},
    {-0.372923f,-0.20808f,-0.90423f},{-0.38227f,0.0678893f,-0.921554f},{0.000561957f,-0.403724f,-0.914881f},
    {0.00105191f,-0.338081f,-0.941117f},{0.0011484f,-0.207542f,-0.978225f},{-0.000825592f,0.0678506f,-0.997695f},
    {0.350177f,-0.404552f,-0.844816f},{0.360723f,-0.338796f,-0.868963f},{0.375088f,-0.207977f,-0.903357f},
    {0.38071f,0.0681321f,-0.922181f},{0.646889f,-0.405104f,-0.646085f},{0.665912f,-0.3393f,-0.664407f},
    {0.69241f,-0.208344f,-0.690768f},{0.704873f,0.0681091f,-0.706056f},{0.845246f,-0.404569f,-0.349116f},
    {0.869765f,-0.338841f,-0.358743f},{0.90423f,-0.208081f,-0.372922f},{0.921555f,0.0678845f,-0.382268f},
    {0.91488f,-0.403725f,0.000563648f},{0.941117f,-0.338081f,0.001052f},{0.978226f,-0.207539f,0.00114733f},
    {0.997695f,0.0678488f,-0.000826532f},{0.844818f,-0.404549f,0.350175f},{0.868962f,-0.338796f,0.360725f},
    {0.903356f,-0.207977f,0.37509f},{0.922181f,0.0681272f,0.380711f},{0.646086f,-0.405103f,0.646888f},
    {0.66441f,-0.339297f,0.665911f},{0.690769f,-0.208341f,0.69241f},{0.706055f,0.0681064f,0.704875f},
    {0.349115f,-0.404571f,0.845246f},{0.358742f,-0.338842f,0.869764f},{0.372925f,-0.208076f,0.90423f},
    {0.38227f,0.0678876f,0.921553f},{-0.00056228f,-0.403724f,0.914881f},{-0.00105215f,-0.338081f,0.941117f},
    {-0.00114887f,-0.207541f,0.978226f},{0.000825791f,0.0678507f,0.997695f},{-0.350176f,-0.404552f,0.844816f},
    {-0.360724f,-0.338795f,0.868963f},{-0.375091f,-0.207975f,0.903357f},{-0.380712f,0.0681318f,0.92218f},
    {-0.646888f,-0.405104f,0.646086f},{-0.665911f,-0.339298f,0.664409f},{-0.692411f,-0.208339f,0.690769f},
    {-0.704874f,0.0681079f,0.706055f},{-0.845246f,-0.40457f,0.349117f},{-0.869764f,-0.338843f,0.358742f},
    {-0.90423f,-0.208076f,0.372925f},{-0.921553f,0.0678894f,0.382271f},{-0.829014f,0.446626f,-0.336542f},
    {-0.895149f,0.445728f,0.00587759f},{-0.669848f,0.69074f,-0.272364f},{-0.723934f,0.689856f,0.00428515f},
    {-0.642819f,0.717289f,-0.26885f},{-0.697655f,0.716429f,-0.00271169f},{-0.735465f,0.603391f,-0.308237f},
    {-0.798123f,0.602483f,-0.0036328f},{-0.636683f,0.447071f,-0.628301f},{-0.513968f,0.691304f,-0.507874f},
    {-0.490357f,0.717847f,-0.494211f},{-0.56095f,0.604018f,-0.566125f},{-0.347602f,0.446484f,-0.824515f},
    {-0.280399f,0.690733f,-0.666532f},{-0.263769f,0.71729f,-0.64492f},{-0.301411f,0.603431f,-0.738257f},
    {-0.00587676f,0.445726f,-0.89515f},{-0.00428658f,0.689857f,-0.723933f},{0.00271131f,0.716433f,-0.69765f},
    {0.00363493f,0.602475f,-0.79813f},{0.336542f,0.446624f,-0.829015f},{0.272366f,0.690737f,-0.66985f},
    {0.268846f,0.717296f,-0.642813f},{0.308234f,0.603396f,-0.735463f},{0.628299f,0.447074f,-0.636684f},
    {0.507874f,0.691303f,-0.513969f},{0.494212f,0.717847f,-0.490358f},{0.566122f,0.604023f,-0.560948f},
    {0.824516f,0.446482f,-0.347601f},{0.666531f,0.690734f,-0.280399f},{0.64492f,0.71729f,-0.263769f},
    {0.738256f,0.603433f,-0.301409f},{0.895151f,0.445725f,-0.00587695f},{0.723935f,0.689855f,-0.00428471f},
    {0.69765f,0.716433f,0.00271151f},{0.798122f,0.602485f,0.0036329f},{0.829015f,0.446626f,0.336541f},
    {0.669847f,0.690741f,0.272364f},{0.642822f,0.717286f,0.268852f},{0.735451f,0.603412f,0.308229f},
    {0.636683f,0.447072f,0.6283f},{0.513966f,0.691306f,0.507872f},{0.49036f,0.717843f,0.494215f},
    {0.56095f,0.604018f,0.566125f},{0.347602f,0.446482f,0.824516f},{0.280398f,0.690735f,0.66653f},
    {0.263771f,0.717287f,0.644922f},{0.301413f,0.60342f,0.738265f},{0.00587753f,0.445728f,0.895149f},
    {0.00428544f,0.689858f,0.723932f},{-0.00271214f,0.716428f,0.697656f},{-0.00363358f,0.602473f,0.798131f},
    {-0.336542f,0.446626f,0.829014f},{-0.272365f,0.690741f,0.669847f},{-0.268852f,0.717286f,0.642821f},
    {-0.308235f,0.603398f,0.735461f},{-0.628299f,0.447074f,0.636684f},{-0.507873f,0.691306f,0.513967f},
    {-0.494216f,0.717842f,0.490361f},{-0.566122f,0.604021f,0.560949f},{-0.824516f,0.446483f,0.347601f},
    {-0.666529f,0.690735f,0.280398f},{-0.644924f,0.717285f,0.263772f},{-0.738259f,0.603428f,0.301412f},
    {-0.8064f,0.524923f,-0.27235f},{-0.849708f,0.52428f,0.0559279f},{-0.155622f,0.986123f,-0.0578273f},
    {-0.229084f,0.966813f,-0.11311f},{-0.255675f,0.966616f,-0.0168275f},{-0.166246f,0.986066f,0.00594871f},
    {-0.0486499f,0.998674f,-0.016849f},{-0.0514925f,0.998669f,0.00298265f},{-8.32472e-009f,1.0f,7.02944e-009f},
    {0.0f,0.0f,1.0f},{-0.639799f,0.525793f,-0.560534f},{-0.1213f,0.986175f,-0.112891f},
    {-0.168207f,0.966874f,-0.191994f},{-0.0383706f,0.99868f,-0.0341553f},{-0.376867f,0.525423f,-0.762825f},
    {-0.0688977f,0.986142f,-0.150921f},{-0.0817984f,0.966726f,-0.242383f},{-0.022397f,0.998677f,-0.0462899f},
    {-0.0559277f,0.524255f,-0.849723f},{-0.00594904f,0.986066f,-0.166246f},{0.0168281f,0.966616f,-0.255675f},
    {-0.00298278f,0.998669f,-0.0514924f},{0.272352f,0.524911f,-0.806408f},{0.0578278f,0.986123f,-0.155622f},
    {0.11311f,0.966813f,-0.229084f},{0.0168492f,0.998674f,-0.0486499f},{0.560531f,0.525794f,-0.639801f},
    {0.11289f,0.986175f,-0.1213f},{0.191994f,0.966874f,-0.168206f},{0.0341553f,0.99868f,-0.0383706f},
    {0.762815f,0.525441f,-0.376862f},{0.15092f,0.986142f,-0.0688974f},{0.242382f,0.966726f,-0.0817978f},
    {0.0462898f,0.998677f,-0.0223971f},{0.849717f,0.524265f,-0.0559245f},{0.166246f,0.986066f,-0.00594889f},
    {0.255676f,0.966616f,0.0168282f},{0.0514926f,0.998669f,-0.00298276f},{0.806389f,0.524944f,0.272342f},
    {0.155623f,0.986122f,0.057828f},{0.229086f,0.966812f,0.113111f},{0.04865f,0.998674f,0.0168493f},
    {0.639784f,0.525831f,0.560517f},{0.121301f,0.986175f,0.112891f},{0.168208f,0.966873f,0.191995f},
    {0.0383704f,0.99868f,0.0341554f},{0.376866f,0.525424f,0.762825f},{0.0688976f,0.986142f,0.150921f},
    {0.0817974f,0.966727f,0.242381f},{0.0223969f,0.998677f,0.0462897f},{0.0559279f,0.524252f,0.849725f},
    {0.00594896f,0.986067f,0.166245f},{-0.0168276f,0.966617f,0.255673f},{0.00298265f,0.998669f,0.0514925f},
    {-0.27235f,0.52492f,0.806402f},{-0.0578275f,0.986123f,0.155622f},{-0.113112f,0.966812f,0.229086f},
    {-0.016849f,0.998674f,0.0486499f},{-0.560522f,0.525819f,0.639789f},{-0.112892f,0.986175f,0.121301f},
    {-0.191996f,0.966873f,0.168209f},{-0.0341554f,0.99868f,0.0383705f},{-0.76282f,0.525434f,0.376863f},
    {-0.150921f,0.986142f,0.068898f},{-0.242383f,0.966726f,0.0817978f},{-0.0462897f,0.998677f,0.022397f},
    {-0.0305204f,0.978539f,-0.20379f},{-0.00801589f,0.979094f,-0.203249f},{-0.00772565f,0.997628f,-0.0683996f},
    {-0.0405564f,0.999176f,0.00183418f},{-0.155235f,0.965079f,-0.211007f},{-0.191282f,0.981532f,0.00256176f},
    {-0.304622f,0.929164f,-0.209425f},{-0.306094f,0.948731f,0.0788392f},{-0.851976f,0.45734f,-0.254907f},
    {-0.85684f,0.436446f,-0.274481f},{-0.874815f,0.477609f,-0.0811686f},{-0.87373f,0.476915f,0.0956451f},
    {-0.0209923f,0.587616f,-0.808868f},{-0.00553686f,0.589776f,-0.807548f},{-0.00519571f,0.591366f,-0.806387f},
    {-0.0349612f,0.593632f,-0.803977f},{-0.0999156f,0.561613f,-0.821345f},{-0.148459f,0.559938f,-0.815126f},
    {-0.281589f,0.425541f,-0.860013f},{-0.373134f,0.339809f,-0.863308f},{-0.462668f,0.110332f,-0.879639f},
    {-0.480014f,0.0167619f,-0.877101f},{0.0233441f,-0.587074f,-0.809197f},{0.00609993f,-0.589938f,-0.807426f},
    {0.00583244f,-0.591353f,-0.806392f},{0.0395373f,-0.591348f,-0.805446f},{0.110262f,-0.554584f,-0.82479f},
    {0.164493f,-0.54338f,-0.823214f},{0.297812f,-0.40319f,-0.865301f},{0.384489f,-0.310453f,-0.86936f},
    {0.465877f,-0.0912907f,-0.880128f},{0.479863f,-0.00360097f,-0.877336f},{0.0568949f,-0.99838f,-0.000915956f},
    {0.0105652f,-0.997594f,0.0685118f},{0.0103096f,-0.979258f,-0.202355f},{0.0703127f,-0.976766f,-0.202448f},
    {0.266733f,-0.96377f,0.0010396f},{0.310215f,-0.92496f,-0.219583f},{0.422642f,-0.902633f,-0.0814097f},
    {0.404644f,-0.889573f,-0.21195f},{0.99708f,-0.0763432f,-0.00199503f},{0.935469f,-0.342472f,0.0872354f},
    {0.905237f,-0.334049f,-0.262597f},{0.963597f,0.0131061f,-0.267036f},{0.0396583f,-0.978098f,0.204332f},
    {0.010124f,-0.979114f,0.203058f},{0.203134f,-0.954647f,0.21768f},{0.410532f,-0.88301f,0.227499f},
    {0.948218f,-0.164563f,0.271666f},{0.899718f,-0.339798f,0.273945f},{0.0214167f,-0.5875f,0.808941f},
    {0.00553687f,-0.589776f,0.807548f},{0.00583227f,-0.591198f,0.806506f},{0.039724f,-0.590501f,0.806058f},
    {0.104228f,-0.55836f,0.823025f},{0.166066f,-0.539153f,0.825673f},{0.293942f,-0.405907f,0.865354f},
    {0.38696f,-0.300225f,0.871853f},{0.463661f,-0.0902488f,0.881404f},{0.479718f,0.00410942f,0.877413f},
    {-0.0186005f,0.588139f,0.808546f},{-0.00482685f,0.589572f,0.807702f},{-0.0051959f,0.59117f,0.80653f},
    {-0.0352178f,0.592569f,0.80475f},{-0.0921073f,0.566458f,0.818927f},{-0.150659f,0.554364f,0.818524f},
    {-0.275955f,0.42995f,0.859647f},{-0.376239f,0.325636f,0.867413f},{-0.459348f,0.108248f,0.881636f},
    {-0.479874f,0.00463354f,0.877325f},{-0.00769202f,0.97927f,0.202415f},{-0.0513301f,0.978379f,0.20035f},
    {-0.229944f,0.951142f,0.206045f},{-0.319029f,0.927602f,0.194359f},{-0.851547f,0.456875f,0.257162f},
    {-0.86496f,0.440582f,0.240276f},{-0.924014f,-0.274507f,-0.266165f},{-0.947142f,-0.183567f,-0.263105f},
    {-0.978055f,-0.188091f,-0.0896123f},{-0.94564f,-0.325174f,0.00524297f},{-0.816165f,-0.520611f,-0.250677f},
    {-0.823673f,-0.566896f,0.0138323f},{-0.657982f,-0.72073f,-0.218193f},{-0.644146f,-0.764819f,0.0112687f},
    {-0.520438f,-0.830564f,-0.19826f},{-0.531526f,-0.84384f,0.0735863f},{-0.458605f,-0.143497f,-0.876978f},
    {-0.456181f,-0.195095f,-0.868238f},{-0.414544f,-0.288035f,-0.863243f},{-0.425715f,-0.347247f,-0.835575f},
    {-0.363509f,-0.427206f,-0.827863f},{-0.354618f,-0.481787f,-0.801329f},{-0.306959f,-0.511551f,-0.802553f},
    {-0.32953f,-0.534392f,-0.778354f},{0.456208f,0.149417f,-0.877239f},{0.44896f,0.206263f,-0.869419f},
    {0.401984f,0.3038f,-0.863779f},{0.404962f,0.363147f,-0.839124f},{0.34386f,0.438452f,-0.830374f},
    {0.333793f,0.487181f,-0.806993f},{0.291862f,0.512601f,-0.8075f},{0.310046f,0.53283f,-0.787378f},
    {0.92697f,0.375116f,-0.0038936f},{0.871577f,0.416255f,-0.259008f},{0.754307f,0.656438f,-0.0105182f},
    {0.705291f,0.670292f,-0.230809f},{0.569473f,0.82197f,-0.00812794f},{0.532984f,0.820735f,-0.205726f},
    {0.480131f,0.874123f,-0.0733768f},{0.476584f,0.857445f,-0.194048f},{0.915211f,0.301917f,0.266899f},
    {0.763263f,0.594819f,0.252231f},{0.585366f,0.779527f,0.222901f},{0.472121f,0.857016f,0.20646f},
    {0.458184f,0.144451f,0.877041f},{0.447237f,0.206304f,0.870298f},{0.405491f,0.297967f,0.864172f},
    {0.397789f,0.365916f,0.84135f},{0.344281f,0.435477f,0.831764f},{0.323851f,0.488784f,0.810068f},
    {0.289768f,0.509891f,0.809967f},{0.295416f,0.531498f,0.793876f},{-0.460977f,-0.137411f,0.876709f},
    {-0.453965f,-0.19533f,0.869346f},{-0.418756f,-0.280865f,0.863573f},{-0.416538f,-0.350996f,0.838629f},
    {-0.363894f,-0.423572f,0.829559f},{-0.341759f,-0.483918f,0.805621f},{-0.303999f,-0.508013f,0.805921f},
    {-0.310041f,-0.532822f,0.787385f},{-0.947648f,-0.182599f,0.261957f},{-0.896985f,-0.359891f,0.256705f},
    {-0.772905f,-0.59307f,0.225579f},{-0.589942f,-0.782673f,0.19847f},{-0.515854f,-0.836549f,0.184608f},
    {0.223676f,-0.925959f,-0.30425f},{0.233034f,-0.918665f,-0.318981f},{0.230486f,-0.966651f,0.11163f},
    {0.809823f,-0.526916f,-0.257967f},{0.753737f,-0.605672f,-0.255035f},{0.780951f,-0.617674f,-0.0927053f},
    {0.870758f,-0.491531f,-0.0133312f},{0.849841f,-0.436761f,-0.294976f},{0.87584f,-0.480806f,-0.0415903f},
    {0.644369f,-0.67043f,-0.367849f},{0.556874f,-0.737388f,-0.382296f},{0.123432f,-0.412413f,-0.902596f},
    {0.0713064f,-0.398932f,-0.914204f},{0.0670385f,-0.424373f,-0.903003f},{0.18216f,-0.41474f,-0.89152f},
    {0.273926f,-0.373487f,-0.886268f},{0.299728f,-0.348187f,-0.888217f},{0.31031f,-0.339395f,-0.887986f},
    {0.268579f,-0.323524f,-0.907302f},{0.0084573f,-0.13038f,-0.991428f},{0.194165f,-0.442992f,-0.875248f},
    {-0.346553f,0.332405f,-0.877159f},{-0.214891f,0.390605f,-0.895126f},{-0.21479f,0.39175f,-0.894649f},
    {-0.460877f,0.238342f,-0.85486f},{-0.783685f,0.241144f,-0.57244f},{-0.52905f,0.0385361f,-0.847715f},
    {-0.46141f,0.0874265f,-0.882869f},{-0.337303f,0.0867027f,-0.937395f},{-0.257736f,0.201463f,-0.944979f},
    {-0.40859f,0.906698f,-0.104655f},{-0.408906f,0.906911f,0.101525f},{-0.391516f,0.871055f,-0.296611f},
    {-0.389623f,0.873576f,-0.291649f},{-0.92414f,0.381676f,0.0169956f},{-0.884305f,0.458926f,0.0859759f},
    {-0.864307f,0.443033f,-0.238109f},{-0.852331f,0.521906f,0.0338666f},{-0.761301f,0.539364f,-0.359871f},
    {-0.575857f,0.817548f,0.00178272f},{-0.509833f,0.786034f,-0.3496f},{-0.391747f,0.870857f,0.296887f},
    {-0.388802f,0.870349f,0.302203f},{-0.89033f,0.389273f,0.236177f},{-0.870283f,0.440371f,0.22064f},
    {-0.858622f,0.422187f,0.290733f},{-0.588833f,0.733522f,0.339443f},{-0.358068f,0.326564f,0.874725f},
    {-0.21397f,0.390676f,0.895315f},{-0.214808f,0.391586f,0.894716f},{-0.461665f,0.242362f,0.853303f},
    {-0.589324f,0.113305f,0.799912f},{-0.526283f,0.0561237f,0.848455f},{-0.473813f,0.0745215f,0.877467f},
    {-0.336081f,0.112558f,0.935083f},{-0.265489f,0.174056f,0.948272f},{-0.210746f,0.185579f,0.959764f},
    {0.119032f,-0.4115f,0.903603f},{0.0471226f,-0.399496f,0.915523f},{0.0677062f,-0.420376f,0.90482f},
    {0.189791f,-0.410249f,0.892006f},{0.277374f,-0.368916f,0.887111f},{0.305808f,-0.345592f,0.887157f},
    {0.318381f,-0.327024f,0.889769f},{0.271549f,-0.328585f,0.904595f},{0.233987f,-0.391839f,0.889782f},
    {0.193198f,-0.456555f,0.868465f},{0.212006f,-0.9232f,0.320555f},{0.224731f,-0.925303f,0.305467f},
    {0.237367f,-0.927042f,0.290259f},{0.754979f,-0.601497f,0.261168f},{0.853307f,-0.43989f,0.279936f},
    {0.779815f,-0.50813f,0.365639f},{0.653678f,-0.661819f,0.367016f},{0.53826f,-0.757972f,0.368449f},
    {0.272794f,-0.850498f,-0.449707f},{0.372441f,-0.927835f,0.0202464f},{0.203269f,-0.942302f,-0.265986f},
    {0.251385f,-0.966236f,0.0565083f},{-0.410741f,-0.731485f,0.544263f},{-0.238701f,-0.790631f,0.563848f},
    {-0.559985f,-0.66426f,0.495152f},{-0.574255f,-0.638058f,0.512946f},{-0.60903f,0.634918f,0.475353f},
    {-0.581789f,0.657063f,0.479363f},{-0.635511f,0.611981f,0.47075f},{0.0497197f,-0.674283f,-0.736797f},
    {0.0441771f,-0.851062f,-0.523203f},{0.0606892f,-0.735079f,0.67526f},{0.0463619f,-0.879734f,0.473201f},
    {-0.188275f,0.0881443f,0.978153f},{-0.1765f,0.108922f,0.978255f},{-0.199943f,0.0673161f,0.977492f},
    {-0.204983f,0.209184f,-0.956151f},{-0.195958f,0.141135f,-0.970403f},{-0.118302f,-0.68152f,-0.722174f},
    {-0.14469f,-0.597241f,-0.788903f},{0.227417f,-0.882216f,0.412282f},{0.178733f,-0.95353f,0.242558f},
    {0.169256f,-0.516235f,0.839556f},{0.17183f,-0.514575f,0.840052f},{-0.688887f,0.721959f,0.0648909f},
    {-0.681053f,0.504505f,-0.530699f},{-0.748468f,0.29529f,-0.593801f},{-0.761295f,0.137436f,-0.633673f},
    {0.293061f,-0.955865f,-0.0209379f},{0.214856f,-0.975323f,-0.0508133f},{0.370055f,-0.923898f,0.0973186f},
    {0.36346f,-0.891859f,0.26923f},{-0.428963f,0.85241f,0.298978f},{-0.745338f,0.317156f,0.586416f},
    {-0.781666f,0.19556f,0.592245f},{-0.69698f,0.433634f,0.571122f},{0.219803f,-0.849486f,-0.479647f},
    {0.142722f,-0.93264f,-0.331381f},{0.3586f,-0.892616f,-0.273209f},{-0.2061f,0.210881f,0.955537f},
    {-0.194275f,0.235869f,0.952168f},{-0.0416883f,-0.758081f,0.650826f},{0.0719689f,-0.782164f,0.618903f},
    {-0.119054f,-0.677088f,0.726208f},{-0.119747f,-0.6134f,0.780641f},{0.0800582f,-0.809076f,-0.582225f},
    {-0.0147285f,-0.847432f,-0.5307f},{0.168937f,-0.516675f,-0.839349f},{0.170453f,-0.525851f,-0.833322f},
    {0.249609f,-0.832161f,0.49518f},{0.155584f,-0.917926f,0.364974f},{-0.234776f,-0.792124f,-0.5634f},
    {-0.409519f,-0.733986f,-0.541811f},{-0.186784f,0.0928505f,-0.978003f},{-0.174803f,0.0336099f,-0.98403f},
    {-0.198076f,0.151748f,-0.968369f},{-0.55697f,-0.66883f,-0.492393f},{-0.578139f,-0.642373f,-0.503103f},
    {-0.605798f,0.637442f,-0.476106f},{-0.618425f,0.595528f,-0.512735f},{-0.591196f,0.67728f,-0.437926f},
    {-7.24813e-008f,-1.0f,2.25328e-007f},{-0.143004f,-0.989722f,3.72675e-008f},{-0.131817f,-0.989786f,-0.0542894f},
    {-0.845312f,0.412163f,-0.339954f},{-0.889416f,0.130006f,-0.438222f},{-0.989484f,0.12937f,-0.0646914f},
    {-0.911615f,0.410972f,0.00775737f},{-0.715722f,0.655509f,-0.240936f},{-0.754321f,0.654651f,0.0493132f},
    {-0.94723f,-0.0229677f,-0.31973f},{-0.89649f,-0.0226276f,-0.442486f},{-0.997573f,-0.0231758f,-0.0656504f},
    {-0.99759f,-0.0231784f,0.0653949f},{-0.100586f,-0.989831f,-0.100586f},{-0.649755f,0.412738f,-0.638331f},
    {-0.6531f,0.130235f,-0.745989f},{-0.567382f,0.656764f,-0.496728f},{-0.752113f,-0.0224387f,-0.658652f},
    {-0.658806f,-0.0224409f,-0.751979f},{-0.05429f,-0.989786f,-0.131817f},{-0.354854f,0.411885f,-0.839303f},
    {-0.316189f,0.129725f,-0.939785f},{-0.333613f,0.656224f,-0.67681f},{-0.442717f,-0.022629f,-0.896376f},
    {-0.319537f,-0.0229696f,-0.947295f},{0.0f,-0.989722f,-0.143003f},{-0.00775884f,0.410976f,-0.911613f},
    {0.0646909f,0.129379f,-0.989483f},{-0.0493123f,0.654651f,-0.754322f},{-0.0653945f,-0.0231784f,-0.99759f},
    {0.0656509f,-0.0231757f,-0.997573f},{0.05429f,-0.989786f,-0.131817f},{0.339955f,0.412166f,-0.845311f},
    {0.438225f,0.130006f,-0.889414f},{0.240935f,0.655509f,-0.715722f},{0.319731f,-0.0229676f,-0.94723f},
    {0.442486f,-0.0226276f,-0.89649f},{0.100586f,-0.989831f,-0.100586f},{0.638331f,0.412738f,-0.649755f},
    {0.745989f,0.130235f,-0.6531f},{0.496728f,0.656764f,-0.567382f},{0.658652f,-0.0224387f,-0.752113f},
    {0.751979f,-0.0224409f,-0.658805f},{0.131817f,-0.989786f,-0.05429f},{0.839303f,0.411885f,-0.354854f},
    {0.939785f,0.129725f,-0.316189f},{0.67681f,0.656225f,-0.333613f},{0.896376f,-0.022629f,-0.442716f},
    {0.947295f,-0.0229697f,-0.319537f},{0.143004f,-0.989722f,3.63359e-008f},{0.911613f,0.410976f,-0.00775858f},
    {0.989483f,0.129379f,0.0646911f},{0.754322f,0.654651f,-0.049312f},{0.99759f,-0.0231784f,-0.0653941f},
    {0.997573f,-0.0231757f,0.0656513f},{0.131817f,-0.989786f,0.0542901f},{0.845311f,0.412166f,0.339955f},
    {0.889414f,0.130006f,0.438225f},{0.715722f,0.655509f,0.240936f},{0.94723f,-0.0229676f,0.319731f},
    {0.89649f,-0.0226276f,0.442486f},{0.512997f,0.102296f,0.852273f},{0.100586f,-0.989831f,0.100587f},
    {0.649755f,0.412738f,0.638331f},{0.567382f,0.656764f,0.496728f},{0.752113f,-0.0224387f,0.658652f},
    {0.658805f,-0.022441f,0.751979f},{0.238855f,0.0979899f,0.966099f},{0.0542894f,-0.989786f,0.131818f},
    {0.354854f,0.411883f,0.839304f},{0.333613f,0.656224f,0.67681f},{0.442716f,-0.0226291f,0.896376f},
    {0.319537f,-0.0229697f,0.947295f},{-0.0484141f,0.096817f,0.994124f},{0.0f,-0.989722f,0.143004f},
    {0.00775801f,0.41097f,0.911616f},{0.0493123f,0.654651f,0.754321f},{0.0653945f,-0.0231784f,0.99759f},
    {-0.0656508f,-0.0231757f,0.997573f},{-0.334318f,0.0991773f,0.937228f},{-0.0542894f,-0.989786f,0.131818f},
    {-0.339955f,0.412161f,0.845313f},{-0.240935f,0.655509f,0.715722f},{-0.319731f,-0.0229677f,0.94723f},
    {-0.442486f,-0.0226276f,0.89649f},{-0.597689f,0.104345f,0.794909f},{-0.100586f,-0.989831f,0.100587f},
    {-0.63833f,0.412736f,0.649756f},{-0.496728f,0.656764f,0.567382f},{-0.658652f,-0.0224387f,0.752113f},
    {-0.751979f,-0.022441f,0.658806f},{-0.131818f,-0.989786f,0.0542894f},{-0.839302f,0.411886f,0.354855f},
    {-0.939786f,0.12972f,0.316187f},{-0.67681f,0.656224f,0.333613f},{-0.896376f,-0.0226291f,0.442717f},
    {-0.947295f,-0.0229697f,0.319537f},{-0.229425f,-0.969259f,-0.0888904f},{-0.294119f,-0.944671f,-0.145226f},
    {-0.328217f,-0.944355f,-0.0216021f},{-0.246498f,-0.969128f,0.00544522f},{-0.158335f,-0.985171f,-0.066097f},
    {-0.171994f,-0.985098f,-0.000587967f},{-0.3271f,-0.933351f,-0.147858f},{-0.35959f,-0.933039f,-0.0115217f},
    {-0.499154f,-0.849974f,-0.168494f},{-0.526347f,-0.849564f,0.034643f},{-0.177493f,-0.969364f,-0.169792f},
    {-0.215969f,-0.94477f,-0.246509f},{-0.120721f,-0.985216f,-0.121552f},{-0.245276f,-0.933496f,-0.261582f},
    {-0.395596f,-0.85052f,-0.346582f},{-0.099028f,-0.969284f,-0.225127f},{-0.105017f,-0.944533f,-0.311172f},
    {-0.0650029f,-0.985169f,-0.158799f},{-0.126385f,-0.933285f,-0.336162f},{-0.233049f,-0.850291f,-0.471904f},
    {-0.00544504f,-0.969128f,-0.246497f},{0.0216021f,-0.944356f,-0.328214f},{0.000587892f,-0.985098f,-0.171994f},
    {0.0115212f,-0.933038f,-0.359593f},{-0.034643f,-0.84956f,-0.526354f},{0.0888908f,-0.969259f,-0.229424f},
    {0.145227f,-0.944672f,-0.294117f},{0.0660966f,-0.985171f,-0.158335f},{0.147859f,-0.933351f,-0.3271f},
    {0.168495f,-0.849972f,-0.499156f},{0.169792f,-0.969364f,-0.177492f},{0.246509f,-0.94477f,-0.215969f},
    {0.121553f,-0.985216f,-0.120721f},{0.261584f,-0.933495f,-0.245278f},{0.346586f,-0.850517f,-0.395599f},
    {0.225127f,-0.969284f,-0.0990282f},{0.311173f,-0.944533f,-0.105017f},{0.158799f,-0.985169f,-0.0650034f},
    {0.336163f,-0.933285f,-0.126385f},{0.471906f,-0.85029f,-0.233049f},{0.246497f,-0.969128f,-0.00544467f},
    {0.328215f,-0.944356f,0.0216022f},{0.171994f,-0.985098f,0.000587972f},{0.359593f,-0.933038f,0.0115207f},
    {0.526354f,-0.84956f,-0.0346433f},{0.229424f,-0.969259f,0.0888909f},{0.294117f,-0.944671f,0.145227f},
    {0.158334f,-0.985171f,0.0660973f},{0.327099f,-0.933351f,0.147859f},{0.499156f,-0.849972f,0.168494f},
    {0.177492f,-0.969364f,0.169792f},{0.215968f,-0.94477f,0.246511f},{0.120722f,-0.985216f,0.121553f},
    {0.245275f,-0.933496f,0.261583f},{0.395594f,-0.85052f,0.346584f},{0.0990274f,-0.969284f,0.225128f},
    {0.105018f,-0.944532f,0.311175f},{0.0650032f,-0.985169f,0.158799f},{0.126386f,-0.933286f,0.336161f},
    {0.233048f,-0.850291f,0.471904f},{0.00544516f,-0.969128f,0.246498f},{-0.0216023f,-0.944355f,0.328217f},
    {-0.000588046f,-0.985098f,0.171994f},{-0.0115215f,-0.933039f,0.35959f},{0.0346433f,-0.849564f,0.526347f},
    {-0.0888905f,-0.969259f,0.229425f},{-0.145226f,-0.944671f,0.294119f},{-0.0660972f,-0.985171f,0.158334f},
    {-0.147858f,-0.933351f,0.327099f},{-0.168494f,-0.849974f,0.499154f},{-0.169792f,-0.969364f,0.177493f},
    {-0.24651f,-0.94477f,0.215968f},{-0.121553f,-0.985216f,0.120722f},{-0.261581f,-0.933497f,0.245274f},
    {-0.34658f,-0.850523f,0.395592f},{-0.225128f,-0.969284f,0.0990274f},{-0.311175f,-0.944533f,0.105018f},
    {-0.158799f,-0.985169f,0.0650033f},{-0.336161f,-0.933286f,0.126386f},{-0.471902f,-0.850293f,0.233047f}
    };

    // build triangles
    cVector3d zero(0,0,0);
    for(int i=0;i<1024;i++)
    {
        // get indices for vertices and normals
        int vi0=face_indices[i][0];
        int ni0=face_indices[i][0+3];
        int vi1=face_indices[i][1];
        int ni1=face_indices[i][1+3];
        int vi2=face_indices[i][2];
        int ni2=face_indices[i][2+3];

        // generate normals and vectors in local coordinates
        cVector3d normal0(-normals[ni0][2],-normals[ni0][0],-normals[ni0][1]);
        cVector3d vertex0(vertices[vi0][2], vertices[vi0][0], vertices[vi0][1]);
        cVector3d normal1(-normals[ni1][2],-normals[ni1][0],-normals[ni1][1]);
        cVector3d vertex1(vertices[vi1][2], vertices[vi1][0], vertices[vi1][1]);
        cVector3d normal2(-normals[ni2][2],-normals[ni2][0],-normals[ni2][1]);
        cVector3d vertex2(vertices[vi2][2], vertices[vi2][0], vertices[vi2][1]);

        // apply transformation
        double scale = 1.0;
        cVector3d v0 = cAdd(a_pos, cMul(a_rot, cMul(scale * a_size, vertex0)));
        cVector3d v1 = cAdd(a_pos, cMul(a_rot, cMul(scale * a_size, vertex1)));
        cVector3d v2 = cAdd(a_pos, cMul(a_rot, cMul(scale * a_size, vertex2)));
        cVector3d n0 = cMul(a_rot, normal0);
        cVector3d n1 = cMul(a_rot, normal1);
        cVector3d n2 = cMul(a_rot, normal2);
    
        // create new triangle
        a_mesh->newTriangle(v2, v1, v0, n2, n1, n0, zero, zero, zero, a_color, a_color, a_color);
    }
}


//==============================================================================
/*!
    This function creates a linear arrow.

    \param  a_mesh                          Mesh object in which primitive is created.
    \param  a_length                        Length of arrow.
    \param  a_radiusShaft                   Radius of arrow shaft.
    \param  a_lengthTip                     Length or arrow tip.
    \param  a_radiusTip                     Radius of arrow tip.
    \param  a_includeTipsAtBothExtremities  Include tip at both extremities of arrow.
    \param  a_numSides                      Number of sides for each radial section.
    \param  a_direction                     Direction of arrow.
    \param  a_pos                           Position where to build the new primitive.
    \param  a_color                         Color of vertices.
*/
//==============================================================================
void cCreateArrow(cMesh* a_mesh, 
    const double& a_length,
    const double& a_radiusShaft,
    const double& a_lengthTip,
    const double& a_radiusTip,
    const bool a_includeTipsAtBothExtremities,
    const unsigned int a_numSides,
    const cVector3d& a_direction,
    const cVector3d& a_pos,
    const cColorf& a_color)
{
    // sanity check
    if ((a_direction.length() == 0.0)   ||
        (a_length < 0)                  ||
        (a_radiusShaft < 0)             ||
        (a_lengthTip < 0)               ||
        (a_radiusTip > 360)) 
    { return; }
    
    // create rotation frame from direction vector
    cVector3d vx,vy,vz,t0,t1, t;;
    t0.set(1,0,0);
    t1.set(0,1,0);
    vz = cNormalize(a_direction);
    double ang0 = cAngle(vz, t0);
    double ang1 = cAngle(vz, t1);
    if (ang0>ang1)
    {
        t = cCross(vz, t0);  
    }
    else
    {
        t = cCross(vz, t1);
    }
    vy = cNormalize(t);
    vx = cNormalize(cCross(vy, vz));

    cMatrix3d rot;
    rot.setCol(vx, vy, vz);

    cVector3d offset0, offset1, offset2;
    double length;
    if (a_includeTipsAtBothExtremities)
    {
        length = a_length - 2.0 * a_lengthTip;
        offset0.set(0, 0, length + a_lengthTip);
        offset1.set(0, 0, a_lengthTip);
    }
    else
    {
        length = a_length - 1.0 * a_lengthTip;
        offset0.set(0, 0, length);
        offset1.set(0, 0, 0);
    }
    
    // create first tip
    cVector3d pos0 = cAdd(a_pos, cMul(rot, offset0));
    cMatrix3d rot0 = rot;

    cCreateCone(a_mesh, 
                a_lengthTip,  
                a_radiusTip,
                0.0,
                a_numSides,
                1,
                1,
                true,
                false,
                pos0,
                rot0,
                a_color);

    // create arrow shaft
    cVector3d pos1 = cAdd(a_pos, cMul(rot, offset1));
    cMatrix3d rot1 = rot;

    if (length > 0)
    {
        cCreateCylinder(a_mesh, 
                        length,  
                        a_radiusShaft,
                        a_numSides,
                        1,
                        1,
                        true,
                        true,
                        pos1,
                        rot1,
                        a_color);
    }

    // create possibly second tip
    if (a_includeTipsAtBothExtremities)
    {
        offset2.set(0, 0, a_lengthTip);
        cVector3d pos2 = cAdd(a_pos, cMul(rot, offset2));  
        cMatrix3d rot2p;
        rot2p.identity();
        rot2p.rotateAboutGlobalAxisDeg(cVector3d(1,0,0), 180);
        cMatrix3d rot2 = cMul(rot,rot2p);

        cCreateCone(a_mesh, 
                    a_lengthTip,  
                    a_radiusTip,
                    0.0,
                    a_numSides,
                    1,
                    1,
                    true,
                    false,
                    pos2,
                    rot2,
                    a_color);
    }
}


//==============================================================================
/*!
    This function creates a circular arrow.

    \param  a_mesh                          Mesh object in which primitive is created.
    \param  a_innerRadius0                  Length of arrow.
    \param  a_innerRadius1                  Radius of arrow shaft.
    \param  a_outerRadius                   Radius of arrow shaft.
    \param  a_lengthTip                     Length or arrow tip.
    \param  a_radiusTip                     Radius of arrow tip.
    \param  a_coverageAngleDeg              Coverage angle of the arrow (0-360 degrees)
    \param  a_includeTipsAtBothExtremities  Include tip at both extremities of arrow.
    \param  a_numSides                      Number of sides for each radial section.
    \param  a_numRings                      Number of radial divisions for the circular shaft.
    \param  a_direction                     Direction of circular arrow plane.
    \param  a_pos                           Position where to build the new primitive.
    \param  a_color                         Color of vertices.
*/
//==============================================================================
void cCreateCircularArrow(cMesh* a_mesh,
    const double& a_innerRadius0,
    const double& a_innerRadius1,
    const double& a_outerRadius,
    const double& a_lengthTip,
    const double& a_radiusTip,
    const double& a_coverageAngleDeg,
    const bool a_includeTipsAtBothExtremities,
    const unsigned int a_numSides,
    const unsigned int a_numRings, 
    const cVector3d& a_direction,
    const cVector3d& a_pos,
    const cColorf& a_color)
{
    // sanity check
    if ((a_direction.length() == 0.0)   ||
        (a_innerRadius0 < 0)            ||
        (a_innerRadius1 < 0)            ||
        (a_coverageAngleDeg < 0)        ||
        (a_coverageAngleDeg > 360)      ||
        (a_outerRadius < 0)             ||
        (a_outerRadius < 0))  
    { return; }
    
    // create rotation frame from direction vector
    cVector3d vx,vy,vz,t0,t1, t;;
    t0.set(1,0,0);
    t1.set(0,1,0);
    vz = cNormalize(a_direction);
    double ang0 = cAngle(vz, t0);
    double ang1 = cAngle(vz, t1);
    if (ang0>ang1)
    {
        t = cCross(vz, t0);
    }
    else
    {
        t = cCross(vz, t1);
    }
    vy = cNormalize(t);
    vx = cNormalize(cCross(vy, vz));

    cMatrix3d rot;
    rot.setCol(vx, vy, vz);

    // create ring
    cCreateRingSection(a_mesh,
                       a_innerRadius0,
                       a_innerRadius1,
                       a_outerRadius,
                       a_coverageAngleDeg, 
                       true,
                       a_numSides,
                       a_numRings,
                       a_pos,
                       rot,
                       a_color);

    cVector3d offset0, offset1, pos0, pos1;
    cMatrix3d rot0, rot0p, rot1, rot1p;

    offset0.set(a_outerRadius * cCosDeg(a_coverageAngleDeg + 90),
                a_outerRadius * cSinDeg(a_coverageAngleDeg + 90),
                0.0);
    pos0 = cAdd(a_pos, cMul(rot, offset0));
    rot0p.identity();
    rot0p.rotateAboutGlobalAxisDeg(cVector3d(0,1,0), -90);
    rot0p.rotateAboutGlobalAxisDeg(cVector3d(0,0,1), a_coverageAngleDeg);
    rot0 = cMul(rot,rot0p);

    // create first arrow tip
    cCreateCone(a_mesh, 
                a_lengthTip,  
                a_radiusTip,
                0.0,
                a_numSides,
                1,
                1,
                true,
                false,
                pos0,
                rot0,
                a_color);

    // create second arrow tip
    if (a_includeTipsAtBothExtremities)
    {
        offset1.set(0, a_outerRadius, 0);
        pos1 = cAdd(a_pos, cMul(rot, offset1));
        rot1p.identity();
        rot1p.rotateAboutGlobalAxisDeg(cVector3d(0,1,0), 90);
        rot1 = cMul(rot, rot1p);

        // create first arrow tip
        cCreateCone(a_mesh,
                    a_lengthTip,
                    a_radiusTip,
                    0.0,
                    a_numSides,
                    1,
                    1,
                    true,
                    false,
                    pos1,
                    rot1,
                    a_color);
    }
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
