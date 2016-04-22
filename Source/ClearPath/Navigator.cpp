#include "ClearPath.h"
#include "Navigator.h"

#include <vector>

using namespace Directive;

void NavigatorQuerier::Unregister(Navigator* nav)
{
    navs.erase(std::remove_if(navs.begin(), navs.end(), [nav](auto navPtr){ return navPtr.lock().get() == nav; }), navs.end());
}

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

Navigator::~Navigator()
{
    NavigatorQuerier::Instance()->Unregister(this);
}

void Navigator::Update(UnitType deltaTime)
{
	if (arrived)
	{
		return;
	}
    
    // calculate the desired velocity
    desiredVel = velocity;
    auto toDestination = target - position;
    if (toDestination.SizeSquared2D() <= maxSpeed * maxSpeed * deltaTime * deltaTime)
    {
        desiredVel = toDestination / deltaTime;
    }
    else
    {
        desiredVel = maxSpeed * toDestination.GetSafeNormal();
    }

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
    
    // test if there will be collision at all
    bool willCollide = false;
    for (const auto& beCollection : boundaryEdges)
    {
        bool in = true;
        for (const auto& be : beCollection)
        {
            if ((be.GetNormal() | (desiredVel - be.GetPoint1())) <= 0)
            {
                in = false;
                break;
            }
        }
        
        // in BE of this navigator
        if (in)
        {
            willCollide = true;
            break;
        }
    }
    
    DrawLine(position, position + desiredVel, willCollide ? FColor::Red : FColor::White, false, deltaTime);
    
    if (!willCollide)
    {
        nextVelocity = desiredVel;
    }
    
    std::vector<Directive::Vector> validVelocities;
    if (willCollide)
    {
        // find the intersection point on the boundary
        // calculate the potential new velocities
        for (const auto& beCollection : boundaryEdges)
        {
            bool in = true;
            for (const auto& be : beCollection)
            {
                const auto dotProduct = (be.GetPoint1() | be.GetDir());
                const auto discriminant = FMath::Square(dotProduct) + FMath::Square(maxSpeed) - (be.GetPoint1().SizeSquared2D());
                
                if (discriminant < 0)
                {
                    // no intersection against this segment
                    continue;
                }
                
                const auto sqrtDiscriminant = FMath::Sqrt(discriminant);
                float tLeft = -dotProduct - sqrtDiscriminant;
                float tRight = -dotProduct + sqrtDiscriminant;
                
                if (be.GetPoint1().IsNearlyZero() == false && be.GetPoint1().Size2D() < maxSpeed * maxSpeed)
                {
                    validVelocities.emplace_back(be.GetPoint1());
                }
                
                if (!be.IsRay() && be.GetPoint2().IsNearlyZero() == false && be.GetPoint2().Size2D() < maxSpeed * maxSpeed)
                {
                    validVelocities.emplace_back(be.GetPoint2());
                }
                
                if (tLeft >= 0 && (be.IsRay() || tLeft <= be.GetLength()))
                {
                    validVelocities.emplace_back(be.GetPoint1() + tLeft * be.GetDir());
                }
                
                if (tRight >= 0 && (be.IsRay() || tRight <= be.GetLength()))
                {
                    validVelocities.emplace_back(be.GetPoint1() + tRight * be.GetDir());
                }
            }
        }
        
        // pick the proper new velocity
        if (validVelocities.size() == 0)
        {
            // no valid adjust, relax the constraints or slow down the speed and try again.
        }
        else
        {
            // among all the valid velocities find one closest to the current velocity
            auto maxDotProduct(0);
            nextVelocity = Directive::Vector(0);
            bool found = false;
            const Directive::Vector relativeVerticalVel(-velocity.Y, velocity.X, 0);
            for (const auto& vel : validVelocities)
            {
                // TODO: make sure that (relativeVelocity | relativePosition.Vertical) has the same signal as (newVelocity | relativePosition.Vertical)
                if ((relativeVerticalVel | vel) * (velocity | relativeVerticalVel) >= 0)
                {
                    auto dotProduct = (velocity | vel);
                    if (dotProduct > maxDotProduct)
                    {
                        maxDotProduct = dotProduct;
                        nextVelocity = vel;
                        found = true;
                        break;
                    }
                }
            }
            
            if (found)
            {
                DrawLine(position + FVector(0,0,10), position + FVector(0,0,10) + nextVelocity, FColor::White, false, deltaTime);
            }
            else
            {
                DrawLine(position + FVector(0,0,10), position + FVector(0,0,10) + nextVelocity, FColor::Green, false, deltaTime);
            }
        }
    }
    
    
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
    
    if (velocity.SizeSquared2D() < 0.81 * maxSpeed * maxSpeed && (position - target).SizeSquared2D() <= maxSpeed * maxSpeed * 0.25)
    {
        arrived = true;
    }
}

std::vector<Segment> Navigator::CalcBoundaryEdgesAgainst(const Navigator& other) const
{
    std::vector<Segment> segments;
    
    const auto myRadius = this->radius;
    const auto myPosition = this->position;
    const auto myDesiredVelocity = this->desiredVel;
    
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
    const Directive::UnitType invLookForwardTime = 0.25;  // 2 sec
    
    if (distance > combinedRadius)
    {
        const auto leg = FMath::Sqrt(distanceSquared - combinedRadiusSquared);
        
        auto leftDir = Vector(relativePosition.X * leg - relativePosition.Y * combinedRadius, relativePosition.X * combinedRadius + relativePosition.Y * leg, 0) / distanceSquared;
        leftDir = leftDir.GetSafeNormal();
        
        auto rightDir = Vector(relativePosition.X * leg + relativePosition.Y * combinedRadius, -relativePosition.X * combinedRadius + relativePosition.Y * leg, 0) / distanceSquared;
        rightDir = rightDir.GetSafeNormal();
        
        segments.emplace_back(Segment::CreateRay(velocityOffset, leftDir, Segment::NormalDir::Right));
        segments.emplace_back(Segment::CreateRay(velocityOffset, rightDir, Segment::NormalDir::Left));
        
        // cut off
        Vector M = (relativePosition - combinedRadius * relativePositionDir) * invLookForwardTime + velocityOffset;
        auto halfEdge = FMath::Tan(FMath::Asin(combinedRadius / distance)) * (distance - combinedRadius)  * invLookForwardTime;
        Vector edgeLeftEnd = M + halfEdge * FVector(-relativePositionDir.Y, relativePositionDir.X, 0);
        Vector edgeRightEnd = M + halfEdge * FVector(relativePositionDir.Y, -relativePositionDir.X, 0);
        
        segments.emplace_back(Segment::CreateSegment(edgeLeftEnd, edgeRightEnd, Segment::NormalDir::Left));
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
			DrawDebugLine(start + Vector(0,0,100), end + Vector(0, 0, 100), color, persistent, 0.05);
		}
	}
}
