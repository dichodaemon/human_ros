#ifndef DATACOLLECTION_H
#define DATACOLLECTION_H

#ifdef _WIN32
#include <windows.h>
#define isnan _isnan
#endif

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <plib/js.h>

#include <tgfclient.h>
#include <portability.h>

#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>

#include "facade.h"


bool InitTrackData(tTrack* track);
float computeCurvature( tTrackSeg * segment );
void nextCurve( tCarElt * car, Status & status );
bool SendMessages(int index, tCarElt* car, tSituation *s);


