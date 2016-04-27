#include "ClearPath.h"
#include "Navigator.h"

#include <vector>
#include <algorithm>

DECLARE_CYCLE_STAT(TEXT("Update"),STATGROUP_AVOIDANCE_Update,STATGROUP_AVOIDANCE);
DECLARE_CYCLE_STAT(TEXT("CalcBoundaryEdges"),STAT_CalcBoundaryEdges,STATGROUP_AVOIDANCE);
DECLARE_CYCLE_STAT(TEXT("CalcBEIntersections"),STAT_CalcBEIntersections,STATGROUP_AVOIDANCE);
DECLARE_CYCLE_STAT(TEXT("CalcIntersection"),STAT_CalcIntersection,STATGROUP_AVOIDANCE);
DECLARE_CYCLE_STAT(TEXT("ClassifyIntersecions"),STAT_ClassifyIntersecions,STATGROUP_AVOIDANCE);
DECLARE_CYCLE_STAT(TEXT("ClassifyInsideSegments"),STAT_ClassifyInsideSegments,STATGROUP_AVOIDANCE);
DECLARE_CYCLE_STAT(TEXT("CalcValidVelocitiesOnSegments"),STAT_CalcValidVelocitiesOnSegments,STATGROUP_AVOIDANCE);
DECLARE_CYCLE_STAT(TEXT("CalcBestVelocity"),STAT_CalcBestVelocity,STATGROUP_AVOIDANCE);
DECLARE_CYCLE_STAT(TEXT("InsertingIntersection"),STAT_InsertingIntersection,STATGROUP_AVOIDANCE);
DECLARE_CYCLE_STAT(TEXT("IsWithinPCR"),STAT_IsWithinPCR,STATGROUP_AVOIDANCE);
DECLARE_CYCLE_STAT(TEXT("IsWithinPCRTest"),STAT_IsWithinPCRTest,STATGROUP_AVOIDANCE);

using namespace Directive;

inline Vector GetApex(const Vector& myVelocity, const Vector& otherVelocity)
{
    auto mySize = myVelocity.Size2D();
    auto otherSize = otherVelocity.Size2D();
    if (mySize + otherSize == 0)
    {
        return Vector::ZeroVector;
    }
    auto alpha = mySize / (otherSize + mySize);
//    return (myVelocity * alpha + otherVelocity * (1 - alpha));
    
    return (myVelocity + otherVelocity) / 2;
}




void NavigatorQuerier::Update(Directive::UnitType deltaTime)
{
    navs.erase(std::remove_if(navs.begin(), navs.end(), [](auto navPtr){ return navPtr.lock().get() == nullptr; }), navs.end());
    
    for (auto nav : navs)
    {
        nav.lock()->Update(deltaTime);
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
	if (arrived || maxSpeed <= 0)
	{
        velocity = Vector(0);
		return;
	}
    
    auto p = this->position;
    auto comparator = [p](std::shared_ptr<Navigator> nav1, std::shared_ptr<Navigator> nav2){ return (nav1->position - p).SizeSquared2D() < (nav2->position - p).SizeSquared2D(); };
    std::set<std::shared_ptr<Navigator>, decltype(comparator)> nearests(comparator);
    for (auto otherWeakPtr : NavigatorQuerier::Instance()->navs)
    {
        auto other = otherWeakPtr.lock();
        if (other && other.get() != this)
        {
            nearests.insert(other);
            if (nearests.size() > 3)
            {
                nearests.erase(std::prev(nearests.end()));
            }
        }
    }
    
    for (auto other:nearests)
    {
        DrawLine(position, other->position, FColor::Cyan, false, 0.02);
    }
    
    SCOPE_CYCLE_COUNTER(STATGROUP_AVOIDANCE_Update);
    
    // calculate the desired velocity
    desiredVel = CalcDesiredVelocity(deltaTime);

    // get all BEs
    boundaryEdges.clear();
	auto querier = NavigatorQuerier::Instance();
	for (auto other : nearests)
	{
		//auto other = otherWeakPtr.lock();
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
    
    bool willCollide = TestWillCollide(desiredVel);
    
    DrawLine(position+Vector(0,0,300), position + desiredVel+Vector(0,0,300), willCollide ? FColor::Red : FColor::White, false, deltaTime);
    
    if (!willCollide)
    {
        nextVelocity = desiredVel;
    }
    else
    {
        auto intersections = CalcBEIntersections();
        auto classifiedIntersections = ClassifyIntersecions(intersections);
        auto classifiedSegments = ClassifyInsideSegments(classifiedIntersections);
        for (const auto& seg : classifiedSegments)
        {
            seg.Draw([this](const auto& p1, const auto& p2)
            {
                DrawLine(p1 + position + FVector(0, 0, 100), p2 + position + FVector(0, 0, 100), FColor::Blue, false, 0.02);
            });
        }
        
        std::vector<Directive::Vector> validVelocities = CalcValidVelocitiesOnSegments(classifiedSegments);

		for (auto v : validVelocities)
		{
			DrawLine(position + Vector(0, 0, 400), position + v + Vector(0, 0, 20), FColor::Green, false, 0.02);
            DrawBox(position + v + Vector(0, 0, 20), Vector(10), FColor::Green, false, 0.02);
		}

        nextVelocity = CalcBestVelocity(validVelocities);
        
        DrawBox(position + nextVelocity + Vector(0, 0, 20), Vector(12), FColor::Yellow, false, 0.02);
    }
    
    DrawLine(position + Vector(0,0,10), position + nextVelocity + Vector(0,0,10), FColor::White, false, deltaTime);
    
	velocity = nextVelocity;
	position += velocity * deltaTime;

    auto velocitySquare = velocity.SizeSquared2D();
    auto distanceSquare = (position - target).SizeSquared2D();

	bool targetOccupied = false;
	for (auto other : nearests)
	{
        if ((other->position - target).SizeSquared2D() <= FMath::Square(other->radius + this->radius))
		{
			targetOccupied = true;
			break;
		}
	}

	if (distanceSquare <= 0.01 * radius * radius)
	{
		arrived = true;
	}

    auto dotDesiredVelocity = (desiredVel | velocity);
	auto cosineAngleSquare = dotDesiredVelocity * dotDesiredVelocity / desiredVel.SizeSquared2D() / velocity.SizeSquared2D();
    if (targetOccupied && cosineAngleSquare <= 0.5 && distanceSquare <= 9 * radius * radius)
    {
        arrived = true;
    }
}

Directive::Vector Navigator::CalcDesiredVelocity(UnitType deltaTime) const
{
    Vector desiredV(0);
    auto toDestination = target - position;
	toDestination.Z = 0;
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
    SCOPE_CYCLE_COUNTER(STAT_IsWithinPCR);
    
    
    bool inside = true;
    for (const auto& boundaryEdge : PCR)
    {
        auto dotProduct = (boundaryEdge.GetNormal() | (testVelocity - boundaryEdge.GetPoint1()));
        if (dotProduct <= KINDA_SMALL_NUMBER)
        {
            inside = false;
            break;
        }
    }
    
    return inside;
}

std::vector<std::vector<Navigator::SegmentIntersectionPoints>> Navigator::CalcBEIntersections() const
{
    SCOPE_CYCLE_COUNTER(STAT_CalcBEIntersections);
    
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
            
            {
                SCOPE_CYCLE_COUNTER(STAT_InsertingIntersection);
                result[pcrIndex][segIndex].insert(seg.GetPoint1());
            }
            
            if (seg.IsRay() == false)
            {
                SCOPE_CYCLE_COUNTER(STAT_InsertingIntersection);
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
                        SCOPE_CYCLE_COUNTER(STAT_InsertingIntersection);
                        
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
    SCOPE_CYCLE_COUNTER(STAT_CalcIntersection);
    
    
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

std::vector<Navigator::ClassifiedBEIntersectionsOnSegement> Navigator::ClassifyIntersecions(const std::vector<std::vector<SegmentIntersectionPoints>>& intersections) const
{
    
    SCOPE_CYCLE_COUNTER(STAT_ClassifyIntersecions);
    

    std::vector<ClassifiedBEIntersectionsOnSegement> results;
    for (auto pcrIndex = 0; pcrIndex < intersections.size(); ++pcrIndex)
    {
        for (auto segIndex = 0; segIndex < intersections.at(pcrIndex).size(); ++segIndex)
        {
            const auto& intersectionsOnSeg = intersections.at(pcrIndex).at(segIndex);
            
            ClassifiedBEIntersectionsOnSegement points;
            points.dir = boundaryEdges.at(pcrIndex).at(segIndex).GetDir();
            for (const auto& point : intersectionsOnSeg)
            {
                bool inside = TestWillCollide(point);
                
                points.intersections.emplace_back(ClassifiedBEIntersection{point, inside});
            }
            
            for (const auto& point : points.intersections)
            {
                DrawBox(point.point + position, Vector(5,5,5), point.inside ? FColor::Red : FColor::Blue, false, 0.2);
            }
            
            results.emplace_back(std::move(points));
        }
    }
    
    return results;
}

std::vector<Segment> Navigator::ClassifyInsideSegments(const std::vector<Navigator::ClassifiedBEIntersectionsOnSegement>& intersections) const
{
    SCOPE_CYCLE_COUNTER(STAT_ClassifyInsideSegments);

    std::vector<Segment> result;
    for (const auto& intersectionsOnSeg : intersections)
    {
        if (intersectionsOnSeg.intersections.size() == 0)
        {
            continue;   // no intersections on this segment, which should never happen
        }
        else
        {
            bool previousWasOutside = false;
            for (auto i = 1; i < intersectionsOnSeg.intersections.size(); ++i)
            {
                if (!previousWasOutside && intersectionsOnSeg.intersections.at(i - 1).inside == false && intersectionsOnSeg.intersections.at(i).inside == false)
                {
                    previousWasOutside = true;
                    result.emplace_back(Segment::CreateSegment(intersectionsOnSeg.intersections.at(i-1).point, intersectionsOnSeg.intersections.at(i).point, Segment::NormalDir::Left));
                }
                else
                {
                    previousWasOutside = false;
                }
            }
            
            const auto& lastIntersection = intersectionsOnSeg.intersections.back();
            if (!previousWasOutside && lastIntersection.inside == false)
            {
                auto anyPointOnRay = lastIntersection.point + 100 * intersectionsOnSeg.dir;
                if(!TestWillCollide(anyPointOnRay))
                {
                    result.emplace_back(Segment::CreateRay(lastIntersection.point, intersectionsOnSeg.dir, Segment::NormalDir::Left));
                }
            }
        }
    }
    
    return result;
}

std::vector<Vector> Navigator::CalcValidVelocitiesOnSegments(const std::vector<Segment>& segments) const
{
    SCOPE_CYCLE_COUNTER(STAT_CalcValidVelocitiesOnSegments);

    
    std::vector<Vector> validVelocities;
    // find the intersection point on the boundary
    // calculate the potential new velocities
    for (const auto& seg : segments)
    {
        const auto dotProduct = (seg.GetPoint1() | seg.GetDir());
        const auto discriminant = FMath::Square(dotProduct) + FMath::Square(maxSpeed) - (seg.GetPoint1().SizeSquared2D());
        
        if (discriminant < 0)
        {
            // no intersection against this segment
            continue;
        }
        
        const auto sqrtDiscriminant = FMath::Sqrt(discriminant);
        float tLeft = -dotProduct - sqrtDiscriminant;
        float tRight = -dotProduct + sqrtDiscriminant;
        
        if (seg.GetPoint1().IsNearlyZero() == false && seg.GetPoint1().SizeSquared2D() < maxSpeed * maxSpeed)
        {
            auto newVelocity = seg.GetPoint1();
            if (newVelocity.SizeSquared2D() > maxSpeed * maxSpeed / 64)
            {
                validVelocities.emplace_back(seg.GetPoint1());
            }
        }
        
        if (!seg.IsRay() && seg.GetPoint2().IsNearlyZero() == false && seg.GetPoint2().SizeSquared2D() < maxSpeed * maxSpeed)
        {
            auto newVelocity = seg.GetPoint2();
            if (newVelocity.SizeSquared2D() > maxSpeed * maxSpeed / 64)
            {
                validVelocities.emplace_back(seg.GetPoint2());
            }
        }
        
        if (tLeft >= 0 && (seg.IsRay() || tLeft <= seg.GetLength()))
        {
            auto newVelocity = seg.GetPoint1() + tLeft * seg.GetDir();
            if (newVelocity.SizeSquared2D() > maxSpeed * maxSpeed / 64)
            {
                validVelocities.emplace_back(newVelocity);
            }
        }
        
        if (tRight >= 0 && (seg.IsRay() || tRight <= seg.GetLength()))
        {
            auto newVelocity = seg.GetPoint1() + tRight * seg.GetDir();
            if (newVelocity.SizeSquared2D() > maxSpeed * maxSpeed / 64)
            {
                validVelocities.emplace_back(newVelocity);
            }
        }
    }
    
    return validVelocities;
}

Directive::Vector Navigator::CalcBestVelocity(const std::vector<Directive::Vector>& validVelocities) const
{
    SCOPE_CYCLE_COUNTER(STAT_CalcBestVelocity);

    
    auto maxDotProduct(std::numeric_limits<UnitType>::lowest());
    Directive::Vector bestVelocity(0);
    
    auto smallestSpeed(std::numeric_limits<UnitType>::max());
    Directive::Vector smallestVelocity(0);

    auto found = false;
    for (const auto& vel : validVelocities)
    {
        if (SatifiesConsistentVelocityOrientation(vel))
        {
            // make sure that the new velocity doesn't turn around from previous velocity
            const auto dotVelocity = (velocity | vel);
            const auto dotDesiredVel = (desiredVel | vel);
            
            const auto alpha = 0.5; // 0 full current velocity, 1 full desired velocity
            const auto dotProduct = dotDesiredVel * alpha + (1 - alpha) * dotVelocity;
            
            if (dotProduct > maxDotProduct)
            {
                maxDotProduct = dotProduct;
                bestVelocity = vel;
                found = true;
            }
            
            auto speed = vel.SizeSquared2D();
            if (speed < smallestSpeed)
            {
                smallestSpeed = speed;
                smallestVelocity = vel;
            }
        }
    }

    DrawDebugLine(position + Vector(0,0,100), position + bestVelocity + Vector(0, 0, 100), FColor::Red, false, 5);
    return bestVelocity;
    //return found ? bestVelocity : smallestVelocity;
}

bool Navigator::SatifiesConsistentVelocityOrientation(const Directive::Vector& newVelocity) const
{
    return true;
    
    auto querier = NavigatorQuerier::Instance();
    for (auto otherWeakPtr : querier->navs)
    {
        auto other = otherWeakPtr.lock();
        if (!other || other.get() == this)
        {
            continue;
        }
        
        const auto relativeVelocity = this->velocity - other->velocity;
        if (relativeVelocity.IsNearlyZero())
        {
            continue;
        }
        
        const auto apex = GetApex(this->velocity, other->velocity);
        if (apex.IsNearlyZero())
        {
            continue;
        }
        
        const auto relativePosition = other->position - this->position;
        const Vector relativePositionVertical(relativePosition.Y, -relativePosition.X, 0);

		const auto part1 = (relativePositionVertical | relativeVelocity);
		const auto part2 = ((newVelocity - apex) | relativePositionVertical);
        if (FMath::Abs(part1) <= KINDA_SMALL_NUMBER || FMath::Abs(part2) <= KINDA_SMALL_NUMBER)
        {
            continue;
        }
        
        if (part1 * part2 / relativePositionVertical.SizeSquared2D() < -KINDA_SMALL_NUMBER)
        {
            return false;
        }
    }

    return true;
}

void Navigator::SetNewTargetLocation(const Directive::Vector newTarget)
{
    target = newTarget;
    arrived = false;
}

std::vector<Segment> Navigator::CalcBoundaryEdgesAgainst(const Navigator& other) const
{
    SCOPE_CYCLE_COUNTER(STAT_CalcBoundaryEdges);

    
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
    
    const auto velocityOffset = GetApex(myDesiredVelocity, otherVelocity);
    const Directive::UnitType invLookForwardTime = 0.25;  // 1/n sec
    
    if (distance > combinedRadius)
    {
        Vector edgeLeftEnd = velocityOffset;
        Vector edgeRightEnd = velocityOffset;
        
        // calc cut off edge first
        auto lookTime = invLookForwardTime;
        for (int tries = 0; tries < 4; ++tries)
        {
            const auto freeSpace = (distance - combinedRadius) * lookTime;
            if (freeSpace < maxSpeed / 16)
            {
                // I don't want my speed to be too small
                lookTime *= 2;
                continue;
            }
            Vector M = (relativePosition - combinedRadius * relativePositionDir) * lookTime + velocityOffset;
            auto halfEdge = FMath::Tan(FMath::Asin(combinedRadius / distance)) * (distance - combinedRadius)  * lookTime;
            edgeLeftEnd = M + halfEdge * FVector(-relativePositionDir.Y, relativePositionDir.X, 0);
            edgeRightEnd = M + halfEdge * FVector(relativePositionDir.Y, -relativePositionDir.X, 0);
            
            segments.emplace_back(Segment::CreateSegment(edgeLeftEnd, edgeRightEnd, Segment::NormalDir::Left));
            break;
        }

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
        Vector leftDir(-relativePositionDir.Y, relativePositionDir.X, 0);
        Vector rightDir(relativePositionDir.Y, -relativePositionDir.X, 0);
        if (relativePositionDir.IsNearlyZero())
        {
            auto sumVelocityDir = (this->desiredVel + otherVelocity).GetSafeNormal2D();
            leftDir = Vector(-sumVelocityDir.Y, sumVelocityDir.X, 0);
            rightDir = Vector(sumVelocityDir.Y, -sumVelocityDir.X, 0);
        }
        
        if (!leftDir.IsNearlyZero())
        {
            segments.emplace_back(Segment::CreateRay(velocityOffset, leftDir, Segment::NormalDir::Right));
        }
        
        if (!rightDir.IsNearlyZero())
        {
            segments.emplace_back(Segment::CreateRay(velocityOffset, rightDir, Segment::NormalDir::Left));
        }
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
            DrawDebugBox(center + Vector(0,0,100), extent, color, persistent, 0);
        }
    }
}
