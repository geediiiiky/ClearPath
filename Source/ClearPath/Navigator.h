#pragma once

#include <functional>
#include <memory>
#include <vector>
namespace Directive
{
	using UnitType = float;
	using Vector = FVector;
}

class Navigator;
struct NavigatorQuerier
{
	std::vector<std::weak_ptr<Navigator>> navs;

	void Register(std::weak_ptr<Navigator> nav)
	{
		navs.push_back(nav);
	}

	static NavigatorQuerier* Instance()
	{
		static NavigatorQuerier instance;
		return &instance;
	}
};

struct Segment
{
	Directive::Vector point;
	Directive::Vector dir;
	Directive::UnitType length;

	Directive::Vector Normal() const { return Directive::Vector(-dir.Y, dir.X, dir.Z); }
};

class Navigator
{
private:
	Navigator() = default;
	Navigator(const Navigator & o) = default;
	Navigator(Directive::UnitType navRadius, Directive::UnitType navMaxSpeed, const Directive::Vector& currentLocation, const Directive::Vector& newTargetLocation);

public:
	void Update(Directive::UnitType deltaTime);

	Directive::Vector GetVelocity() const { return velocity; }
	Directive::Vector GetPosition() const { return position; }
	
	std::function<void(const FVector&, const FVector&, const FColor&, bool, float)> DrawDebugLine;
	std::function<void(const FVector&, const FVector&, const FColor&, bool, float)> DrawDebugBox;

	template<class ... T>
	static std::shared_ptr<Navigator> Create(T&& ... all) {
		//auto ptr = std::make_shared<Navigator>(std::forward<T>(all)...);
		auto ptr = std::shared_ptr<Navigator>(new Navigator(std::forward<T>(all)...));
		NavigatorQuerier::Instance()->Register(ptr);
		return ptr;
	}

private:
	void DrawLine(const FVector& start, const FVector& end, const FColor& color, bool persistent, float lifetime) const;

private:
	// intrinsic
	bool showDebug = true;
	Directive::UnitType radius = 0;
	Directive::UnitType maxSpeed = 0;

	// extrinsic
	Directive::Vector target{ 0 };
	Directive::Vector velocity{ 0 };
	Directive::Vector position{ 0 };

	bool arrived = false;
};