#pragma once

#include <functional>
#include <memory>
#include <vector>
#include <set>

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
    
    void Update(Directive::UnitType deltaTime);
    void Update2(Directive::UnitType deltaTime);
};

class Segment
{
public:
    enum class NormalDir
    {
        Left,
        Right
    };
    
private:
	Directive::Vector point;
	Directive::Vector dir;
	Directive::UnitType length = 0;
    NormalDir normalDir = NormalDir::Left;
    bool isRay = false;
    
    Segment() = default;
    Segment(const Directive::Vector& _point, const Directive::Vector& _dir, Directive::UnitType _length, NormalDir _normalDir, bool _isRay)
    : point(_point.X, _point.Y, 0), dir(_dir.GetSafeNormal2D()), length(_length), normalDir(_normalDir), isRay(_isRay)
    {
    }

public:
    static Segment CreateRay(const Directive::Vector& point, const Directive::Vector& dir, NormalDir normalDir)
    {
        return Segment(point, dir, 0, normalDir, true);
    }
    
    static Segment CreateSegment(const Directive::Vector& point, const Directive::Vector& dir, Directive::UnitType length, NormalDir normalDir)
    {
        return Segment(point, dir, length, normalDir, false);
    }
    
    static Segment CreateSegment(const Directive::Vector& point1, const Directive::Vector& point2, NormalDir normalDir)
    {
        const auto offset = point2 - point1;
        Directive::Vector dir(offset.GetSafeNormal2D());
        Directive::UnitType length(offset.Size2D());
        return Segment(point1, dir, length, normalDir, false);
    }
    
    Directive::Vector GetNormal() const { return normalDir == NormalDir::Left ? Directive::Vector(-dir.Y, dir.X, dir.Z) : Directive::Vector(dir.Y, -dir.X, dir.Z); }
    Directive::Vector GetDir() const { return dir; }
    Directive::Vector GetPoint1() const { return point; }
    Directive::Vector GetPoint2() const { return point + dir * length; }
    Directive::UnitType GetLength() const { return length; }
    bool IsRay() const { return isRay; }
    
    void Draw(std::function<void(const FVector&, const FVector&)> draw) const
    {
        if (draw)
        {
            draw(point, point + dir * (isRay ? 10000 : length));
        }
    }
};

class Navigator
{
private:
	Navigator() = default;
	Navigator(const Navigator & o) = default;
	Navigator(Directive::UnitType navRadius, Directive::UnitType navMaxSpeed, const Directive::Vector& currentLocation, const Directive::Vector& newTargetLocation);

public:
    void Update(Directive::UnitType deltaTime);
    void Update2(Directive::UnitType deltaTime);

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
    Directive::Vector CalcDesiredVelocity(Directive::UnitType deltaTime) const;
    std::vector<Segment> CalcBoundaryEdgesAgainst(const Navigator& other) const;
    using SegmentIntersectionPoints = std::set<Directive::Vector, std::function<bool(const Directive::Vector&, const Directive::Vector&)>>;
    std::vector<std::vector<SegmentIntersectionPoints>> CalcBEIntersections() const;    // calculate Boundary Edges intersections against each other
    bool CalcIntersection(const Segment& seg1, const Segment& seg2, Directive::Vector& intersection) const;
    struct ClassifiedBEIntersection
    {
        Directive::Vector point;
        bool inside;
    };
    struct ClassifiedBEIntersectionsOnSegement
    {
        Directive::Vector dir;
        std::vector<ClassifiedBEIntersection> intersections;
    };
    std::vector<ClassifiedBEIntersectionsOnSegement> ClassifyIntersecions(const std::vector<std::vector<SegmentIntersectionPoints>>& intersections) const;// Classify BE intersections as inside or outside
    std::vector<Segment> ClassifyInsideSegments(const std::vector<ClassifiedBEIntersectionsOnSegement>& intersections) const;// Classify BE segments and find all inside sgements
    bool IsWithinPCR(const Directive::Vector& testVelocity, const std::vector<Segment>& PCR) const;   // potentially colliding region
    bool TestWillCollide(const Directive::Vector& testVelocity) const;
    std::vector<Directive::Vector> CalcValidVelocitiesOnSegments(const std::vector<Segment>& segments) const;
    Directive::Vector CalcBestVelocity(const std::vector<Directive::Vector>& validVelocities) const;
    bool SatifiesConsistentVelocityOrientation(const Directive::Vector& newVelocity) const;
    
	void DrawLine(const FVector& start, const FVector& end, const FColor& color, bool persistent, float lifetime) const;
    void DrawBox(const FVector& center, const FVector& extent, const FColor& color, bool persistent, float lifetime) const;

private:
	// intrinsic
	bool showDebug = true;
	Directive::UnitType radius = 0;
	Directive::UnitType maxSpeed = 0;

	// extrinsic
	Directive::Vector target{ 0 };
	Directive::Vector velocity{ 0 };
	Directive::Vector position{ 0 };
    
    Directive::Vector nextVelocity{ 0 };
    Directive::Vector desiredVel{ 0 };
    
    std::vector<std::vector<Segment>> boundaryEdges;

	FColor debugColor;

	bool arrived = false;
};