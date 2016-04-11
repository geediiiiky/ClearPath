#include "ClearPath.h"
#include "Navigator.h"

using namespace Directive;

Navigator::Navigator(const Directive::Vector & newTargetLocation, Directive::UnitType navRadius, Directive::UnitType navMaxSpeed)
	: target(newTargetLocation)
	, radius(navRadius)
	, maxSpeed(navMaxSpeed)
{
}

void Navigator::Update(UnitType deltaTime)
{
	velocity = target - position;
	velocity = maxSpeed * velocity.GetSafeNormal();
	
	position += velocity * deltaTime;
}
