#include "ClearPath.h"
#include "Navigator.h"

using namespace Directive;

Navigator::Navigator(Directive::UnitType navRadius, Directive::UnitType navMaxSpeed, const Directive::Vector& currentLocation, const Directive::Vector& newTargetLocation)
	: target(newTargetLocation)
	, radius(navRadius)
	, maxSpeed(navMaxSpeed)
	, position(currentLocation)
{
}

void Navigator::Update(UnitType deltaTime)
{
	velocity = target - position;
	velocity = maxSpeed * velocity.GetSafeNormal();
	
	position += velocity * deltaTime;
}
