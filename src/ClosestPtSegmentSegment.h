#ifndef __CLOSEST_PT_SEGMENT_SEGMENT_H__
#define __CLOSEST_PT_SEGMENT_SEGMENT_H__

#include "ofVec3f.h"

float Clamp(float n, float min, float max);
float ClosestPtSegmentSegment(const ofVec3f p1, const ofVec3f q1, const ofVec3f p2, const ofVec3f q2,
                              float &s, float &t, ofVec3f &c1, ofVec3f &c2);

#endif // __CLOSEST_PT_SEGMENT_SEGMENT_H__
