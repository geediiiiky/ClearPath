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
	if (!arrived)
	{
		auto toDestination = target - position;
		if (toDestination.SizeSquared2D() <= maxSpeed * maxSpeed * deltaTime * deltaTime)
		{
			velocity = toDestination / deltaTime;
			position = target;

			arrived = true;
		}
		else
		{
			velocity = maxSpeed * toDestination.GetSafeNormal();
			position += velocity * deltaTime;
		}
	}

	if (showDebug)
	{
		if (DrawDebugLine)
		{
			DrawDebugLine(position, target, FColor::Red, false, deltaTime);
		}
	}
}
