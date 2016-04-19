#include "ClearPath.h"
#include "Navigator.h"

#include <vector>

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
	if (arrived)
	{
		return;
	}

	auto querier = NavigatorQuerier::Instance();
	for (auto otherWeakPtr : querier->navs)
	{
		auto other = otherWeakPtr.lock();
		if (!other)
		{
			continue;
		}

		auto otherRadius = other->radius;
		auto ohterPosition = other->position;
		auto otherVelocity = other->velocity;

		auto relativePosition = ohterPosition - this->position;
		auto relativeVelocity = this->velocity - otherVelocity;
		auto combinedRadius = otherRadius + this->radius;
		auto combinedRadiusSquared = combinedRadius * combinedRadius;
		auto distanceSquared = relativePosition.SizeSquared2D();

		const auto leg = FMath::Sqrt(distanceSquared - combinedRadiusSquared);

		auto leftDir = Vector(relativePosition.X * leg - relativePosition.Y * combinedRadius, relativePosition.X * combinedRadius + relativePosition.Y * leg, 0) / distanceSquared;
		leftDir = leftDir.GetSafeNormal();

		auto rightDir = Vector(relativePosition.X * leg + relativePosition.Y * combinedRadius, -relativePosition.X * combinedRadius + relativePosition.Y * leg, 0) / distanceSquared;
		rightDir = rightDir.GetSafeNormal();

		DrawLine(position, position + leftDir * leg * 2, FColor::Red, false, deltaTime);
		DrawLine(position, position + rightDir * leg * 2, FColor::Red, false, deltaTime);
	}


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
	
	DrawLine(position, target, FColor::Red, false, deltaTime);
}

void Navigator::DrawLine(const FVector& start, const FVector& end, const FColor& color, bool persistent, float lifetime) const
{
	if (showDebug)
	{
		if (DrawDebugLine)
		{
			DrawDebugLine(start, end, color, persistent, lifetime);
		}
	}
}
