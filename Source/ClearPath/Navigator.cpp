#include "ClearPath.h"
#include "Navigator.h"

#include <vector>
#include <algorithm>

using namespace Directive;

void NavigatorQuerier::Update(Directive::UnitType deltaTime)
{
    navs.erase(std::remove_if(navs.begin(), navs.end(), [](auto navPtr){ return navPtr.lock().get() == nullptr; }), navs.end());
    
    for (auto nav : navs)
    {
        nav.lock()->Update(deltaTime);
    }
}

void NavigatorQuerier::Update2(Directive::UnitType deltaTime)
{
    for (auto nav : navs)
    {
        nav.lock()->Update2(deltaTime);
    }
}

Navigator::Navigator(Directive::UnitType navRadius, Directive::UnitType navMaxSpeed, const Directive::Vector& currentLocation, const Directive::Vector& newTargetLocation)
	: radius(navRadius)
	, maxSpeed(navMaxSpeed)
    , target(newTargetLocation)
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
    
    // calculate the desired velocity
    desiredVel = CalcDesiredVelocity(deltaTime);

    // get all BEs
    boundaryEdges.clear();
	auto querier = NavigatorQuerier::Instance();
	for (auto otherWeakPtr : querier->navs)
	{
		auto other = otherWeakPtr.lock();
		if (!other || other.get() == this)
		{
			continue;
		}

        auto segments = CalcBoundaryEdgesAgainst(*other);
        for (const auto& seg : segments)
        {
            seg.Draw([this, deltaTime](const auto& p1, const auto& p2)
            {
                DrawLine(p1 + position, p2 + position, debugColor, false, deltaTime);
            });
        }
        
        boundaryEdges.emplace_back(std::move(segments));
	}
    
    auto intersections = CalcIntersections();
    for (const auto& pcr : intersections)
    {
        for (const auto& seg : pcr)
        {
            for (const auto& point : seg)
            {
                DrawBox(position + point, Vector(10, 10, 10), debugColor, false, 0.1);
            }
        }
    }
    
    bool willCollide = TestWillCollide(desiredVel);
    
    DrawLine(position, position + desiredVel, willCollide ? FColor::Red : FColor::White, false, deltaTime);
    
    if (!willCollide)
    {
        nextVelocity = desiredVel;
    }
    else
    {
        std::vector<Directive::Vector> validVelocities = CalcValidVelocitiesOnBE();

//		for (auto v : validVelocities)
//		{
//			DrawLine(position + Vector(0, 0, 20), position + v + Vector(0, 0, 20), FColor::Green, false, deltaTime);
//		}

        nextVelocity = CalcBestVelocity(validVelocities);
    }
    
    DrawLine(position + Vector(0,0,10), position + nextVelocity + Vector(0,0,10), FColor::White, false, deltaTime);
}

Directive::Vector Navigator::CalcDesiredVelocity(UnitType deltaTime) const
{
    Vector desiredV(0);
    auto toDestination = target - position;
    if (toDestination.SizeSquared2D() <= maxSpeed * maxSpeed * deltaTime * deltaTime)
    {
        desiredV = toDestination / deltaTime;
    }
    else
    {
        desiredV = maxSpeed * toDestination.GetSafeNormal();
    }
    
    return desiredV;
}

bool Navigator::TestWillCollide(const Vector& testVelocity) const
{
    bool willCollide = false;
    for (const auto& pcr : boundaryEdges)
    {
        if (IsWithinPCR(testVelocity, pcr))
        {
            willCollide = true;
            break;
        }
    }
    
    return willCollide;
}

bool Navigator::IsWithinPCR(const Directive::Vector& testVelocity, const std::vector<Segment>& PCR) const
{
    bool inside = true;
    for (const auto& boundaryEdge : PCR)
    {
        if ((boundaryEdge.GetNormal() | (testVelocity - boundaryEdge.GetPoint1())) <= 0)
        {
            inside = false;
            break;
        }
    }

    return inside;
}

std::vector<std::vector<Navigator::SegmentIntersectionPoints>> Navigator::CalcIntersections() const
{
    std::vector<std::vector<SegmentIntersectionPoints>> result;
    for (const auto& pcr : boundaryEdges)
    {
        std::vector<SegmentIntersectionPoints> pcrIntersections;
        for (const auto& seg : pcr)
        {
            auto endPoint = seg.GetPoint1();
            auto comp = [endPoint](const Vector& lhs, const Vector& rhs) -> bool { return (lhs - endPoint).SizeSquared2D() < (rhs - endPoint).SizeSquared2D(); };
            pcrIntersections.emplace_back(SegmentIntersectionPoints(comp));
        }
        
        result.emplace_back(std::move(pcrIntersections));
    }
    
    for (auto pcrIndex = 0; pcrIndex < boundaryEdges.size(); ++pcrIndex)
    {
        const auto& pcr = boundaryEdges[pcrIndex];
        for (auto segIndex = 0; segIndex < pcr.size(); ++segIndex)
        {
            const auto& seg = pcr[segIndex];
            
            result[pcrIndex][segIndex].insert(seg.GetPoint1());
            if (seg.IsRay() == false)
            {
                result[pcrIndex][segIndex].insert(seg.GetPoint2());
            }
            
            for (auto otherPcrIndex = pcrIndex + 1; otherPcrIndex < boundaryEdges.size(); ++otherPcrIndex)
            {
                const auto& otherPcr = boundaryEdges[otherPcrIndex];
                for (auto otherSegIndex = 0; otherSegIndex < otherPcr.size(); ++otherSegIndex)
                {
                    const auto& otherSeg = otherPcr[otherSegIndex];
                    Vector intersection(0);
                    if (CalcIntersection(seg, otherSeg, intersection))
                    {
                        result[pcrIndex][segIndex].insert(intersection);
                        result[otherPcrIndex][otherSegIndex].insert(intersection);
                    }
                }
            }
        }
    }
    
    return result;
}

bool Navigator::CalcIntersection(const Segment& seg1, const Segment& seg2, Vector& intersection) const
{
    const FVector2D dir1(seg1.GetDir());
    const FVector2D dir2(seg2.GetDir());
    const auto crossProduct = dir1 ^ dir2;
    
    if (crossProduct == 0)
    {
        // parallel, or even collinear
        return false;
    }
    
    const FVector2D offset(seg2.GetPoint1() - seg1.GetPoint1());
    Directive::UnitType t1 = (offset ^ dir2) / crossProduct;
    Directive::UnitType t2 = (offset ^ dir1) / crossProduct;
    
    if ((t1 >= 0 && (seg1.IsRay() || t1 <= seg1.GetLength()))
        && (t2 >= 0 && (seg2.IsRay() || t2 <= seg2.GetLength())))
    {
        intersection = seg1.GetPoint1() + t1 * seg1.GetDir();
        return true;
    }
    
    return false;
}

std::vector<Vector> Navigator::CalcValidVelocitiesOnBE() const
{
    std::vector<Vector> validVelocities;
    // find the intersection point on the boundary
    // calculate the potential new velocities
    for (const auto& beCollection : boundaryEdges)
    {
        bool in = true;
        for (const auto& boundary : beCollection)
        {
            const auto dotProduct = (boundary.GetPoint1() | boundary.GetDir());
            const auto discriminant = FMath::Square(dotProduct) + FMath::Square(maxSpeed) - (boundary.GetPoint1().SizeSquared2D());
            
            if (discriminant < 0)
            {
                // no intersection against this segment
                continue;
            }
            
            const auto sqrtDiscriminant = FMath::Sqrt(discriminant);
            float tLeft = -dotProduct - sqrtDiscriminant;
            float tRight = -dotProduct + sqrtDiscriminant;
            
            if (boundary.GetPoint1().IsNearlyZero() == false && boundary.GetPoint1().SizeSquared2D() < maxSpeed * maxSpeed)
            {
                validVelocities.emplace_back(boundary.GetPoint1());
            }
            
            if (!boundary.IsRay() && boundary.GetPoint2().IsNearlyZero() == false && boundary.GetPoint2().SizeSquared2D() < maxSpeed * maxSpeed)
            {
                validVelocities.emplace_back(boundary.GetPoint2());
            }
            
            if (tLeft >= 0 && (boundary.IsRay() || tLeft <= boundary.GetLength()))
            {
                validVelocities.emplace_back(boundary.GetPoint1() + tLeft * boundary.GetDir());
            }
            
            if (tRight >= 0 && (boundary.IsRay() || tRight <= boundary.GetLength()))
            {
                validVelocities.emplace_back(boundary.GetPoint1() + tRight * boundary.GetDir());
            }
        }
    }
    
    return validVelocities;
}

Directive::Vector Navigator::CalcBestVelocity(const std::vector<Directive::Vector>& validVelocities) const
{
    auto maxDotProduct(-1);
    Directive::Vector newVelocity(0);

    for (const auto& vel : validVelocities)
    {
        if (SatifiesConsistentVelocityOrientation(vel))
        {
            auto dotProduct = (desiredVel | vel);
            if (dotProduct > maxDotProduct)
            {
                maxDotProduct = dotProduct;
                newVelocity = vel;
            }
        }
    }

    return newVelocity;
}

bool Navigator::SatifiesConsistentVelocityOrientation(const Directive::Vector& newVelocity) const
{
    auto querier = NavigatorQuerier::Instance();
    for (auto otherWeakPtr : querier->navs)
    {
        auto other = otherWeakPtr.lock();
        if (!other || other.get() == this)
        {
            continue;
        }
        
        const auto relativeVelocity = this->velocity - other->velocity;
        const auto relativePosition = other->position - this->position;
        const auto apex = (this->velocity + other->velocity) / 2;
        const Vector relativePositionVertical(relativePosition.Y, -relativePosition.X, 0);

		const auto oldSign = (relativePositionVertical | relativeVelocity) >= 0;
		const auto newSign = ((newVelocity - apex) | relativePositionVertical) >= 0;
        if (oldSign != newSign)
        {
            return false;
        }
    }

    return true;
}

void Navigator::Update2(UnitType deltaTime)
{
//	auto toDestination = target - position;
//	if (toDestination.SizeSquared2D() <= maxSpeed * maxSpeed * deltaTime * deltaTime)
//	{
//		velocity = toDestination / deltaTime;
//		position = target;
//
//		arrived = true;
//	}
//	else
//	{
//		velocity = maxSpeed * toDestination.GetSafeNormal();
//		position += velocity * deltaTime;
//	}
	
    if (arrived)
    {
        return;
    }
    
    velocity = nextVelocity;
    position += velocity * deltaTime;
    
    auto velocitySquare = velocity.SizeSquared2D();
    auto distanceSquare = (position - target).SizeSquared2D();
    if (velocitySquare < 0.81 * maxSpeed * maxSpeed && distanceSquare <= 9 * radius * radius)
    {
        arrived = true;
    }
}

std::vector<Segment> Navigator::CalcBoundaryEdgesAgainst(const Navigator& other) const
{
    std::vector<Segment> segments;
    
    const auto myRadius = this->radius;
    const auto myPosition = this->position;
    const auto myDesiredVelocity = this->velocity;
    
    const auto otherRadius = other.radius;
    const auto otherPosition = other.position;
    const auto otherVelocity = other.velocity;
    
    const auto relativePosition = otherPosition - myPosition;
    const auto relativePositionDir = relativePosition.GetSafeNormal();
    const auto relativeVelocity = myDesiredVelocity - otherVelocity;
    const auto combinedRadius = otherRadius + myRadius;
    const auto combinedRadiusSquared = combinedRadius * combinedRadius;
    const auto distanceSquared = relativePosition.SizeSquared2D();
    const auto distance = FMath::Sqrt(relativePosition.SizeSquared2D());
    
    const auto velocityOffset = (myDesiredVelocity + otherVelocity) / 2;
    const Directive::UnitType invLookForwardTime = 0.25;  // 1/n sec
    
    if (distance > combinedRadius)
    {
        // calc cut off edge first
        Vector M = (relativePosition - combinedRadius * relativePositionDir) * invLookForwardTime + velocityOffset;
        auto halfEdge = FMath::Tan(FMath::Asin(combinedRadius / distance)) * (distance - combinedRadius)  * invLookForwardTime;
        Vector edgeLeftEnd = M + halfEdge * FVector(-relativePositionDir.Y, relativePositionDir.X, 0);
        Vector edgeRightEnd = M + halfEdge * FVector(relativePositionDir.Y, -relativePositionDir.X, 0);
        
        segments.emplace_back(Segment::CreateSegment(edgeLeftEnd, edgeRightEnd, Segment::NormalDir::Left));

        // then two legs. the rays starts from the cut off edge ends
        const auto leg = FMath::Sqrt(distanceSquared - combinedRadiusSquared);
        
        auto leftDir = Vector(relativePosition.X * leg - relativePosition.Y * combinedRadius, relativePosition.X * combinedRadius + relativePosition.Y * leg, 0) / distanceSquared;
        leftDir = leftDir.GetSafeNormal();
        
        auto rightDir = Vector(relativePosition.X * leg + relativePosition.Y * combinedRadius, -relativePosition.X * combinedRadius + relativePosition.Y * leg, 0) / distanceSquared;
        rightDir = rightDir.GetSafeNormal();
        
        segments.emplace_back(Segment::CreateRay(edgeLeftEnd, leftDir, Segment::NormalDir::Right));
        segments.emplace_back(Segment::CreateRay(edgeRightEnd, rightDir, Segment::NormalDir::Left));
        
    }
    else
    {
        FVector leftDir(-relativePositionDir.Y, relativePositionDir.X, 0);
        FVector rightDir(relativePositionDir.Y, -relativePositionDir.X, 0);
        
        segments.emplace_back(Segment::CreateRay(velocityOffset, leftDir, Segment::NormalDir::Right));
        segments.emplace_back(Segment::CreateRay(velocityOffset, rightDir, Segment::NormalDir::Left));
    }
    
    return segments;
}

void Navigator::DrawLine(const FVector& start, const FVector& end, const FColor& color, bool persistent, float lifetime) const
{
	if (showDebug)
	{
		if (DrawDebugLine)
		{
			DrawDebugLine(start + Vector(0,0,100), end + Vector(0, 0, 100), color, persistent, 0.1);
		}
	}
}

void Navigator::DrawBox(const FVector& center, const FVector& extent, const FColor& color, bool persistent, float lifetime) const
{
    if (showDebug)
    {
        if (DrawDebugBox)
        {
            DrawDebugBox(center + Vector(0,0,100), extent, color, persistent, 0.1);
        }
    }
}
