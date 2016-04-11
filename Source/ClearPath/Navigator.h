#pragma once

namespace Directive
{
	using UnitType = float;
	using Vector = FVector;
}

class Navigator
{
public:
	Navigator(const Directive::Vector & newTargetLocation, Directive::UnitType navRadius, Directive::UnitType navMaxSpeed);
	void Update(Directive::UnitType deltaTime);

private:
	// intrinsic
	Directive::UnitType radius = 0;
	Directive::UnitType maxSpeed = 0;

	// extrinsic
	Directive::Vector target;
	Directive::Vector velocity;
	Directive::Vector position;
};