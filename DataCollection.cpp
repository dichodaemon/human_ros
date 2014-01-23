#include "DataCollection.h"

Facade facade( 6000 );
tTrack* my_track;
tCarElt* my_car;

TrackParam trackParam;
CarParam carParam;

bool SaveCarData(const CarParam& car, char* FileName)
{
	FILE* fp = fopen(FileName,"w");
	if (fp==NULL)
	{
		printf("Can't create Car Param File with name \"%s\"\n", FileName);
		return false;
	}

	fprintf(fp,"maxRPM: %f\n", car.maxRPM);
	fprintf(fp,"wheelRadius: %f\n", car.wheelRadius);
	fprintf(fp,"nGear: %d\n", car.nGear);

	fprintf(fp,"gearRatio: ");
	for (int i=0; i<car.nGear; i++)	
		fprintf(fp,"%f\t", car.gearRatio[i]);
	fprintf(fp,"\n");
	
	fprintf(fp,"gearOffset: %d\n", car.gearOffset);
	fprintf(fp,"width: %f\n", car.width);
	fprintf(fp,"length: %f\n", car.length);
	
	fclose(fp);	

	return true;
}

bool InitCarData(tCarElt* car)
{
	my_car = car;

	carParam.maxRPM = car->_enginerpmMax;
	carParam.wheelRadius = car->_wheelRadius( REAR_RGT );
	carParam.nGear = car->_gearNb;
	carParam.gearRatio.clear();
	for (int i=0; i<carParam.nGear; i++)
		carParam.gearRatio.push_back(car->_gearRatio[i]);
	carParam.gearOffset = car->_gearOffset;
	carParam.width = car->_dimension_y;
	carParam.length = car->_dimension_x;

	SaveCarData(carParam, "tmpyyf_car.txt");
	return true;
}

bool SaveTrackData(const TrackParam& track, char* FileName)
{
	FILE* fp = fopen(FileName,"w");
	if (fp==NULL)
	{
		printf("Can't create Track Param File with name \"%s\"\n", FileName);
		return false;
	}
	
	fprintf(fp, "length: %f\n", track.length);
	fprintf(fp, "width: %f\n", track.width);
	fprintf(fp, "nSeg: %d\n", track.nSeg);

	fprintf(fp, "Segs:\n");

	TrackSeg seg;
	for (int i=0; i<track.nSeg; i++)
	{
		seg = track.segs[i];
		fprintf(fp, "Sid: %d\n", seg.id);
		fprintf(fp, "Slength: %f\n", seg.length);
		fprintf(fp, "Swidth: %f\n", seg.width);
		fprintf(fp, "Scurvature: %f\n", seg.curvature);
		fprintf(fp, "Sangle: %f\n", seg.angle);
		fprintf(fp, "SdistFromStart: %f\n", seg.distFromStart);
	}

	fclose(fp);
	return true;
}

bool InitTrackData(tTrack* track)
{
	my_track = track;
	
	trackParam.length = track->length;
	trackParam.width = track->width;
	trackParam.segs.clear();
	
	tTrackSeg *seg;
	TrackSeg trackSeg;
	
	// find the first segment
	seg = track->seg;
	while (seg->id != 0)
		seg = seg->next;

	float totalLength = 0;
	// save the segments
	do
	{
		trackSeg.id = seg->id;
		trackSeg.length = seg->length;
		trackSeg.width = seg->width;
		trackSeg.curvature = computeCurvature (seg);
		trackSeg.angle = seg->arc;
		trackSeg.distFromStart = seg->lgfromstart;
		trackParam.segs.push_back(trackSeg);
		totalLength += seg->length;
		printf("totalLengh: %f\n", totalLength);
		seg = seg->next;
	}while (seg->id != 0);

	trackParam.nSeg = trackParam.segs.size();
	
	SaveTrackData(trackParam, "tmpyyf_track.txt");

	return true;
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


bool SendMessages(int index, tCarElt* car, tSituation *s)
{
	return true;
	// command
	Command command;
	command.steering = car->ctrl.steer;
	command.acceleration = car->ctrl.accelCmd;
	command.brake = car->ctrl.brakeCmd;
	command.gear = car->ctrl.gear;
	
	// status
	Status status;

	status.gear = car->priv.gear;
	status.rpm = car->priv.enginerpm;
	status.speed = sqrt(car->_speed_x*car->_speed_x + car->_speed_y*car->_speed_y);
	status.yaw = car->_yaw - RtTrackSideTgAngleL(&car->_trkPos);
	NORM_PI_PI(status.yaw);
	status.x = RtGetDistFromStart(car);
	status.y = car->_trkPos.toMiddle;
	
	// obstacles
	Obstacles obstacles( s->_ncars - 1 );
	int count = 0;
	for ( int i = 0; i <= obstacles.size(); ++i )
	{
		if ( s->cars[i]->index != car->index )
		{
			float x = RtGetDistFromStart(s->cars[i]) - status.x;
			if ( x > trackParam.length/2.0)
				x -= trackParam.length;
			if ( x<= -trackParam.length/2.0)
				x += trackParam.length;

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
			o.length = s->cars[i]->_dimension_x;
			count++;
		}
	}
	
	Buffer buffer;
	buffer.command = command;
	buffer.status = status;
	buffer.nObstacles = obstacles.size();
	buffer.obstacles = obstacles;

	//printf("\tobstacles over\n");

	//facade.setCommand( command );
	//facade.setStatus( status );
	//facade.setObstacles( obstacles );

	return true;
}


