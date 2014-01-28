#include "DataCollection.h"

#define CarDataFileName "tmpyyf_car.txt"
#define TrackDataFileName "tmpyyf_track.txt"

Facade facade( 6000 );
tTrack* my_track;
tCarElt* my_car;

TrackParam trackParam;
CarParam carParam;

const int MaxStringLength = 100;
const int nCarStr = 7;
const char carStr[nCarStr][MaxStringLength] = {"maxRPM", "wheelRadius", "nGear", "gearRatio", "gearOffset", "width", "length"};
const int nTrackStr = 10;
const char trackStr[nTrackStr][MaxStringLength] = {"length", "width", "nSeg", "segs", "Sid", "Slength", "Swidth", "Scurvature", "Sangle", "SdistFromStart"};

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

	fprintf(fp,"gearRatio: [");
	for (int i=0; i<car.nGear-1; i++)	
		fprintf(fp,"%f\t", car.gearRatio[i]);
	fprintf(fp,"%f]\n", car.gearRatio[car.nGear-1]);
	
	fprintf(fp,"gearOffset: %d\n", car.gearOffset);
	fprintf(fp,"width: %f\n", car.width);
	fprintf(fp,"length: %f\n", car.length);
	
	fclose(fp);

	return true;
}

bool LoadCarData(CarParam& car, char* FileName)
{
	FILE* fp = fopen(FileName, "r");
	if (fp==NULL)
	{
		printf("Can't load Car Param File with name \"%s\"\n", FileName);
		return false;
	}
	
	char tmpStr[255];
	int i;
	while (fscanf(fp, "%s", tmpStr)!=EOF)
	{
		for (i=0; i<nCarStr; i++)
		{
			if (strncmp(tmpStr, carStr[i], strlen(carStr[i])) == 0)
				break;
		}
		switch (i)
		{
			case 0: fscanf(fp,"%f\n", &car.maxRPM); break;
			case 1: fscanf(fp,"%f\n", &car.wheelRadius); break;
			case 2: fscanf(fp,"%d\n", &car.nGear); break;
			case 3: 
				car.gearRatio.resize(car.nGear);
				fscanf(fp,"[");
				for (i=0; i<car.nGear; i++)
					fscanf(fp,"%f", &car.gearRatio[i]);
				fscanf(fp,"\n");
				break;
			case 4: fscanf(fp,"%d\n", &car.gearOffset); break;
			case 5: fscanf(fp,"%f\n", &car.width); break;
			case 6: fscanf(fp, "%f\n", &car.length); break;
			default: break;
		}
	}
	fclose(fp);
//	SaveCarData(car, "tmpyyf_car_load.txt");		// for test
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

	SaveCarData(carParam, CarDataFileName);
//	LoadCarData(carParam, CarDataFileName);		// for test
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
	

	fprintf(fp, "# Track parameters: length(float), width(float), nSeg(int)\n");
	fprintf(fp, "%f\t%f\t%f\n", track.length, track.width, track.nSeg);

	fprintf(fp, "# Segs: One segment per line\n");
	fprintf(fp, "# Segment parameters: id(int), length, width, curvature, angle, distFromStart\n");


	TrackSeg seg;
	for (int i=0; i<track.nSeg; i++)
	{
		seg = track.segs[i];
		fprintf(fp, "%d\t%f\t%f\t%f\t%f\t%f\n", seg.id, seg.length, seg.width, seg.curvature, seg.angle, seg.distFromStart);
	}

	fclose(fp);
	return true;
}

bool LoadTrackData(TrackParam& track, char* FileName)
{
	FILE* fp = fopen(FileName, "r");
	if (fp==NULL)
	{
		printf("Can't load Track Param File with name \"%s\"\n", FileName);
		return false;
	}
	
	char tmpStr[255];
	int i;
	TrackSeg seg;
	int flag = 0;
	const int iOffset = -4;
	const int enoughFlag = 0x3F;

	track.segs.clear();
	int step = 0;
	while (fgets(tmpStr, strlen(tmpStr), fp)!=NULL)
	{
		if (tmpStr[0]=='#')
			continue;

		if (step==0)	// track parameters
		{
			fscanf(fp,"%f%f%d\n", &track.length, &track.width, &track.nSeg);
			step = 1;
		}else
		{
			fscanf(fp,"%d%f%f%f%f%f\n", &seg.id, &seg.length, &seg.width, &seg.curvature, &seg.angle, &seg.distFromStart);
			if (step==track.nSeg)
				break;
			step++;
		}
	}
	fclose(fp);
//	SaveTrackData(track, "tmpyyf_track_load.txt");		//for test
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
		//printf("totalLengh: %f\n", totalLength);
		seg = seg->next;
	}while (seg->id != 0);

	trackParam.nSeg = trackParam.segs.size();
	
	SaveTrackData(trackParam, TrackDataFileName);
//	LoadTrackData(trackParam, TrackDataFileName);	// for test
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

	for (int i=0; i<buffer.nObstacles; i++)
		buffer.obstacles[i] = obstacles[i];

	//printf("\tobstacles over\n");

	//facade.setCommand( command );
	//facade.setStatus( status );
	//facade.setObstacles( obstacles );
	facade.setBuffer (buffer);

	return true;
}


