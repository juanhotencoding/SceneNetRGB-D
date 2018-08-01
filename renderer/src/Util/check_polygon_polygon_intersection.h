/*
 * This file is part of SceneNet RGB-D.
 *
 * Copyright (C) 2017 Imperial College London
 * 
 * The use of the code within this file and all code within files that 
 * make up the software that is SemanticFusion is permitted for 
 * non-commercial purposes only.  The full terms and conditions that 
 * apply to the code within this file are detailed within the LICENSE.txt 
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/semantic-fusion/scenenet-rgbd-license/> 
 * unless explicitly stated.  By downloading this file you agree to 
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then 
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#include <iostream>


/// Separating axis theorem
/// Ref: http://gamemath.com/2011/09/detecting-whether-two-convex-polygons-overlap/

// Very simple vector class
struct _vec2D { float x,y; };


// 2D axially-aligned bounding box.
struct Box2D
{
    _vec2D min, max;
};


// _vecdot product operator
float _vecdot(/*const */_vec2D &a, /*const */_vec2D &b)
{
    return a.x*b.x + a.y*b.y;
}


// Gather up one-dimensional extents of the projection of the polygon
// onto this axis.
void gatherPolygonProjectionExtents(
    int vertCount, /*const */_vec2D *vertList, // input polygon verts
    /*const */_vec2D &v,                       // axis to project onto
    float &outMin, float &outMax          // 1D extents are output here
)
{

    // Initialize extents to a single point, the first vertex
    outMin = outMax = _vecdot(v, vertList[0]);

    // Now scan all the rest, growing extents to include them
    for (int i = 1 ; i < vertCount ; ++i)
    {
        float d = _vecdot(v, vertList[i]);
        if      (d < outMin) outMin = d;
        else if (d > outMax) outMax = d;
    }
}

// Helper routine: test if two convex polygons overlap, using only the edges of
// the first polygon (polygon "a") to build the list of candidate separating axes.
bool findSeparatingAxis(
    int aVertCount, /*const */_vec2D *aVertList,
    int bVertCount, /*const */_vec2D *bVertList)

{
    // Iterate over all the edges
    int prev = aVertCount-1;
    for (int cur = 0 ; cur < aVertCount ; ++cur)
    {

        // Get edge vector.  (Assume operator- is overloaded)
        _vec2D edge;
        edge.x = aVertList[cur].x - aVertList[prev].x;
        edge.y = aVertList[cur].y - aVertList[prev].y;

        // Rotate vector 90 degrees (doesn't matter which way) to get
        // candidate separating axis.
        _vec2D v;
        v.x =  edge.y;
        v.y = -edge.x;

        /// normalise it!
        float norm_ = sqrt(v.x*v.x + v.y*v.y);

        v.x = v.x / norm_;
        v.y = v.y / norm_;

        // Gather extents of both polygons projected onto this axis
        float aMin, aMax, bMin, bMax;
        gatherPolygonProjectionExtents(aVertCount, aVertList, v, aMin, aMax);
        gatherPolygonProjectionExtents(bVertCount, bVertList, v, bMin, bMax);

        // Is this a separating axis?
        if (aMax < bMin) return true;
        if (bMax < aMin) return true;

        // Next edge, please
        prev = cur;
    }

    // Failed to find a separating axis
    return false;
}



// Here is our high level entry point.  It tests whether two polygons intersect.  The
// polygons must be convex, and they must not be degenerate.
bool convexPolygonOverlap(
    int aVertCount, /*const */_vec2D *aVertList,
    int bVertCount, /*const */_vec2D *bVertList
) {

    // First, use all of A's edges to get candidate separating axes
    if (findSeparatingAxis(aVertCount, aVertList, bVertCount, bVertList))
        return false;

    // Now swap roles, and use B's edges
    if (findSeparatingAxis(bVertCount, bVertList, aVertCount, aVertList))
        return false;

    // No separating axis found.  They must overlap
    return true;
}
