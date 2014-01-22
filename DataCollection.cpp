#include "DataCollection.h"

Facade facade( 6000 );
tTrack* my_track;

#define CURVATURE_TOLERANCE 0.5
#define CURVATURE_MAX_DISTANCE 200

bool InitTrackData(tTrack* track)
{
	my_track = track;
}

float computeCurvature( tTrackSeg * segment ) 
{
  if ( segment->type == TR_STR ) {
    return 0;
  } else if( segment->type == TR_LFT ) {
    return 1.0 / segment->radius;
  } else {
    return -1.0 / segment->radius;
  }
}

void nextCurve( tCarElt * car, Status & status )
{
  float curvature = computeCurvature( car->_trkPos.seg );;
  float cummulated = 0;
  if ( car->_trkPos.seg->type == TR_STR ) {
    cummulated = car->_trkPos.seg->length - car->_trkPos.toStart;
  } else {
    cummulated = ( car->_trkPos.seg->arc - car->_trkPos.toStart ) * car->_trkPos.seg->radius;
  }


  tTrackSeg * current = car->_trkPos.seg->next;
  
  while ( cummulated < CURVATURE_MAX_DISTANCE ) {
    float currentCurvature = computeCurvature( current );
    float ratio = 1.0;
    if ( currentCurvature != 0.0 ) {
      ratio = abs( curvature / currentCurvature );
    } else if ( curvature != currentCurvature ) {
      ratio = 1.0 + 10 * CURVATURE_TOLERANCE;
    }

    if ( ratio > 1.0 + CURVATURE_TOLERANCE || ratio < 1.0 - CURVATURE_TOLERANCE ) {
      status.nextCurvature = currentCurvature;
      status.nextDistance = cummulated;
      return;
    }
    cummulated += current->length;
    current = current->next;
  }
  status.nextCurvature = curvature;
  status.nextDistance = cummulated;
}


int id = -1;
float length = 0;

bool SendMessages(int index, tCarElt* car, tSituation *s)
{
	printf("Send For Test\n");
	if ( (id < 0 && car->_trkPos.seg->id == 0) || (id>=0 && car->_trkPos.seg->id!=id))
	{
		id = car->_trkPos.seg->id;
		length += car->_trkPos.seg->length;
	}
	//printf("dist to start: %f\t%f\n",RtGetDistFromStart(car),RtGetDistFromStart2(&(car->_trkPos)));
	//printf("ID: %d,\t l1: %f,\t l2: %f\n", car->_trkPos.seg->id, length, RtGetDistFromStart(car));
	//printf("SendMessage\n");	

	// command
	Command command;
	command.steering = car->ctrl.steer;
	command.acceleration = car->ctrl.accelCmd;
	command.brake = car->ctrl.brakeCmd;
	command.gear = car->ctrl.gear;

	// status
	/*
	Status status;
	status.rpm = car->priv.enginerpm;
	status.gear = car->priv.gear;
	status.gearRatio = car->_gearRatio[car->_gear + car->_gearOffset];
	status.lowerGearRatio = car->_gearRatio[car->_gear + car->_gearOffset - 1];
	status.maxRPM = car->_enginerpmMax;
	status.wheelRadius = car->_wheelRadius( REAR_RGT );
	
	float yaw = car->_yaw - RtTrackSideTgAngleL( &car->_trkPos );
	NORM_PI_PI( yaw );
	status.trackYaw = yaw;
	status.trackDistance = car->_trkPos.toMiddle;
	status.trackCurvature = computeCurvature( car->_trkPos.seg );
	status.trackWidth = car->_trkPos.seg->width;
	nextCurve( car, status );
	status.speed = car->_speed_x;
	status.yaw = car->_yaw;
	status.x = car->_pos_X;
	status.y = car->_pos_Y;
	*/
	// compute modified status (track to straight road)
	
	Status status;
	status.rpm = car->priv.enginerpm;
	status.gear = car->priv.gear;
	status.gearRatio = car->_gearRatio[car->_gear + car->_gearOffset];
	status.lowerGearRatio = car->_gearRatio[car->_gear + car->_gearOffset - 1];
	status.maxRPM = car->_enginerpmMax;
	status.wheelRadius = car->_wheelRadius( REAR_RGT );
	

	float yaw = car->_yaw - RtTrackSideTgAngleL( &car->_trkPos );
	NORM_PI_PI( yaw );
	status.trackYaw = yaw;
	status.trackDistance = car->_trkPos.toMiddle;
	status.trackCurvature = 0;
	status.trackWidth = car->_trkPos.seg->width;
	nextCurve( car, status );
	status.nextCurvature = 0;
	status.speed = sqrt(car->_speed_x*car->_speed_x + car->_speed_y);
	status.yaw = yaw;
	status.x = RtGetDistFromStart(car);
	status.y = car->_trkPos.toMiddle;
	
	//printf("yaw: %f\n", yaw);

	// obstacles
	/*
	Obstacles obstacles( s->_ncars - 1 );
	int count = 0;
	for ( int i = 0; i <= obstacles.size(); ++i )
	{
		if ( s->cars[i]->index != car->index )
		{
			float x = s->cars[i]->_pos_X - status.x;
			float y = s->cars[i]->_pos_Y - status.y;
			float yaw = s->cars[i]->_yaw - status.yaw;
			NORM_PI_PI( yaw );
			Obstacle & o = obstacles[count];
			o.id = s->cars[i]->index;
			o.x = x * cos( -status.yaw ) - y * sin( -status.yaw );
			o.y = y * cos( -status.yaw ) + x * sin( -status.yaw );
			o.theta = yaw;
			o.vX = s->cars[i]->_speed_x * cos( yaw ) + s->cars[i]->_speed_y * cos( yaw + 3.14159265 );
			o.vY = s->cars[i]->_speed_x * sin( yaw ) + s->cars[i]->_speed_y * sin( yaw + 3.14159265 );
			o.width = s->cars[i]->_dimension_y;
			o.height = s->cars[i]->_dimension_x;
			count++;
		}
	}
	*/
	// compute modified obstacles (track to straight road)
	
	Obstacles obstacles( s->_ncars - 1 );
	int count = 0;
	for ( int i = 0; i <= obstacles.size(); ++i )
	{
		if ( s->cars[i]->index != car->index )
		{
			float x = RtGetDistFromStart(s->cars[i]) - status.x;
			if ( x > my_track->length/2.0)
				x -= my_track->length;
			if ( x<= -my_track->length/2.0)
				x += my_track->length;
			float y = s->cars[i]->_trkPos.toMiddle - status.y;
			float yaw = s->cars[i]->_yaw - RtTrackSideTgAngleL( &(s->cars[i]->_trkPos) ) - status.yaw;
			float speed = sqrt(s->cars[i]->_speed_x * s->cars[i]->_speed_x
							 + s->cars[i]->_speed_y * s->cars[i]->_speed_y);
			NORM_PI_PI( yaw );
			Obstacle & o = obstacles[count];
			o.id = s->cars[i]->index;
			o.x = x * cos( -status.yaw ) - y * sin( -status.yaw );
			o.y = y * cos( -status.yaw ) + x * sin( -status.yaw );
			o.theta = yaw;
			o.vX = speed * cos( yaw );
			o.vY = speed * sin( yaw );
			o.width = s->cars[i]->_dimension_y;
			o.height = s->cars[i]->_dimension_x;
			count++;
		}
	}
	
	//printf("\tobstacles over\n");

	//facade.setCommand( command );
	facade.setStatus( status );
	facade.setObstacles( obstacles );

	return true;
}


