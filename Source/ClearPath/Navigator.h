#pragma once

#include <functional>

namespace Directive
{
	using UnitType = float;
	using Vector = FVector;
}

class Navigator
{
public:
	Navigator(Directive::UnitType navRadius, Directive::UnitType navMaxSpeed, const Directive::Vector& currentLocation, const Directive::Vector& newTargetLocation);

	void Update(Directive::UnitType deltaTime);

	Directive::Vector GetVelocity() const { return velocity; }
	Directive::Vector GetPosition() const { return position; }
	
	std::function<void(const FVector&, const FVector&, const FColor&, bool, float)> DrawDebugLine;

private:
	// intrinsic
	bool showDebug = true;
	Directive::UnitType radius = 0;
	Directive::UnitType maxSpeed = 0;

	// extrinsic
	Directive::Vector target{ 0 };
	Directive::Vector velocity{ 0 };
	Directive::Vector position{ 0 };
};