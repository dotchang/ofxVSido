#include "ClosestPtSegmentSegment.h"
#include <float.h>

// Clamp n to lie within the range [min, max]
float Clamp(float n, float min, float max) {
    if (n < min) return min;
    if (n > max) return max;
    return n;
}

//const float EPSILON = 0.000001f;

// Closest point between two segments from Christer Ericson's book

// Computes closest points C1 and C2 of S1(s)=P1+s*(Q1-P1) and
// S2(t)=P2+t*(Q2-P2), returning s and t. Function result is squared
// distance between between S1(s) and S2(t)
float ClosestPtSegmentSegment(const ofVec3f p1, const ofVec3f q1, const ofVec3f p2, const ofVec3f q2,
                              float &s, float &t, ofVec3f &c1, ofVec3f &c2)
{
	typedef ofVec3f Vector;
	
    Vector d1 = q1 - p1; // Direction vector of segment S1
    Vector d2 = q2 - p2; // Direction vector of segment S2
    Vector r = p1 - p2;
    float a = d1.dot(d1); //Dot(d1, d1); // Squared length of segment S1, always nonnegative
    float e = d2.dot(d2); //Dot(d2, d2); // Squared length of segment S2, always nonnegative
    float f = d2.dot(r);  //Dot(d2, r);

    // Check if either or both segments degenerate into points
    if (a <= FLT_EPSILON && e <= FLT_EPSILON) {
        // Both segments degenerate into points
        s = t = 0.0f;
        c1 = p1;
        c2 = p2;
        return (c1-c2).dot(c1-c2); //Dot(c1 - c2, c1 - c2);
    }
    if (a <= FLT_EPSILON) {
        // First segment degenerates into a point
        s = 0.0f;
        t = f / e; // s = 0 => t = (b*s + f) / e = f / e
        t = Clamp(t, 0.0f, 1.0f);
    } else {
        float c = d1.dot(r); //Dot(d1, r);
        if (e <= FLT_EPSILON) {
            // Second segment degenerates into a point
            t = 0.0f;
            s = Clamp(-c / a, 0.0f, 1.0f); // t = 0 => s = (b*t - c) / a = -c / a
        } else {
            // The general nondegenerate case starts here
            float b = d1.dot(d2); //Dot(d1, d2);
            float denom = a*e-b*b; // Always nonnegative

            // If segments not parallel, compute closest point on L1 to L2, and
            // clamp to segment S1. Else pick arbitrary s (here 0)
            if (denom != 0.0f) {
                s = Clamp((b*f - c*e) / denom, 0.0f, 1.0f);
            } else s = 0.0f;

            // Compute point on L2 closest to S1(s) using
            // t = Dot((P1+D1*s)-P2,D2) / Dot(D2,D2) = (b*s + f) / e
            t = (b*s + f) / e;

            // If t in [0,1] done. Else clamp t, recompute s for the new value
            // of t using s = Dot((P2+D2*t)-P1,D1) / Dot(D1,D1)= (t*b - c) / a
            // and clamp s to [0, 1]
            if (t < 0.0f) {
                t = 0.0f;
                s = Clamp(-c / a, 0.0f, 1.0f);
            } else if (t > 1.0f) {
                t = 1.0f;
                s = Clamp((b - c) / a, 0.0f, 1.0f);
            }
        }
    }

    c1 = p1 + d1 * s;
    c2 = p2 + d2 * t;
    return (c1-c2).dot(c1-c2); //Dot(c1 - c2, c1 - c2);
}
