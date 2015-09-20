#pragma once
#include "Boid.h"
#include <cstring>


class Flocking {
public:
	int update();
	void addBoid();
	void addBoid(int x, int y);
    void removeBoid(int x, int y, int radius);
    void setBounds(int xbound, int ybound);
    int flockSize();
    void setDestination(int x, int y,float area);
    void setDestination(Vec2f dest,float area);

    void useCollisionSDF(bool val);
    vector<Boid>* getBoidsHandle();
    Vec2f** calculatePartialDerivaties();
    void draw();

    void setSimulationParameters(
    		int 	mboundaryPadding 	,
            float 	mmaxSpeed 			,
            float 	mmaxForce 			,
          	float 	mflockSepWeight 	,
          	float 	mflockAliWeight 	,
          	float 	mflockCohWeight 	,
          	float 	mcollisionWeight 	,
          	float 	mflockSepRadius 	,
          	float 	mflockAliRadius 	,
          	float 	mflockCohRadius 	,
          	float 	mdestWeight 		);



	private:
    int x_bound,y_bound;
    Vec2f destination;
    float destinationArea;
    Vec2f destinationSeek;

    bool useCollisionFromSDF;
    float** collisionSDF;
    Vec2f** partialDerivaties;
    vector<Boid> boids;

	//Simulation args
    int 	boundaryPadding ;
    float 	maxSpeed 		;
    float 	maxForce 		;
	float 	flockSepWeight ;
	float 	flockAliWeight ;
	float 	flockCohWeight ;
	float 	collisionWeight;
	float 	flockSepRadius ;
	float 	flockAliRadius ;
	float 	flockCohRadius ;
   	float 	destWeight 	;

};
