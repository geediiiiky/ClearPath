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
	debugColor = FColor(FMath::Rand() % 255, FMath::Rand() % 255, FMath::Rand() % 255);
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
		auto otherPosition = other->position;
		auto otherVelocity = other->velocity;

		auto relativePosition = otherPosition - this->position;
		auto relativePositionDir = relativePosition.GetSafeNormal();
		auto relativeVelocity = this->velocity - otherVelocity;
		auto combinedRadius = otherRadius + this->radius;
		auto combinedRadiusSquared = combinedRadius * combinedRadius;
		auto distanceSquared = relativePosition.SizeSquared2D();
		auto distance = FMath::Sqrt(relativePosition.SizeSquared2D());

		const auto leg = FMath::Sqrt(distanceSquared - combinedRadiusSquared);

		auto leftDir = Vector(relativePosition.X * leg - relativePosition.Y * combinedRadius, relativePosition.X * combinedRadius + relativePosition.Y * leg, 0) / distanceSquared;
		leftDir = leftDir.GetSafeNormal();

		auto rightDir = Vector(relativePosition.X * leg + relativePosition.Y * combinedRadius, -relativePosition.X * combinedRadius + relativePosition.Y * leg, 0) / distanceSquared;
		rightDir = rightDir.GetSafeNormal();

		DrawLine(position, position + leftDir * leg * 2, debugColor, false, 0.15f);
		DrawLine(position, position + rightDir * leg * 2, debugColor, false, 0.15f);

		// cut off
		Vector M = relativePosition - combinedRadius * relativePositionDir + position;
		auto halfEdge = FMath::Tan(FMath::Asin(combinedRadius / distance)) * (distance - combinedRadius);
		Vector edgeLeftEnd = M + halfEdge * FVector(-relativePositionDir.Y, relativePositionDir.X, 0);
		Vector edgeRightEnd = M + halfEdge * FVector(relativePositionDir.Y, -relativePositionDir.X, 0);

		DrawLine(edgeRightEnd, edgeLeftEnd, debugColor, false, 1.f);
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
	
	//DrawLine(position, target, FColor::White, false, deltaTime);
}

void Navigator::DrawLine(const FVector& start, const FVector& end, const FColor& color, bool persistent, float lifetime) const
{
	if (showDebug)
	{
		if (DrawDebugLine)
		{
			DrawDebugLine(start + Vector(0,0,100), end + Vector(0, 0, 100), color, persistent, lifetime);
		}
	}
}
