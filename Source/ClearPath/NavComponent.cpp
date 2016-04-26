// Fill out your copyright notice in the Description page of Project Settings.

#include "ClearPath.h"
#include "NavComponent.h"


// Sets default values for this component's properties
UNavComponent::UNavComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	bWantsBeginPlay = true;
	PrimaryComponentTick.bCanEverTick = true;

	// ...
}


// Called when the game starts
void UNavComponent::BeginPlay()
{
	Super::BeginPlay();
}


// Called every frame
void UNavComponent::TickComponent( float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction )
{
	Super::TickComponent( DeltaTime, TickType, ThisTickFunction );

	// get location from nav
//	if (nav)
//	{
//		nav->Update(DeltaTime);
//	}
}

static TAutoConsoleVariable<int> CVarDrawDebugs(TEXT("DrawDebugLines"), 1, TEXT("Whether or not draw debug lines"));

void UNavComponent::CreateNavigator_Implementation(const FVector& newTargetLocation, float navRadius, float navMaxSpeed)
{
	nav = Navigator::Create(navRadius, navMaxSpeed, GetOwner()->GetActorLocation(), newTargetLocation);

	nav->DrawDebugLine = [this](const FVector& start, const FVector& end, const FColor& color, bool persistent, float lifetime)
	{
        if (CVarDrawDebugs.GetValueOnGameThread() != 0 && drawDebug)
        {
            DrawDebugLine(GetOwner()->GetWorld(), start, end, color, persistent, lifetime, 0, 1.f);
        }
	};

	nav->DrawDebugBox = [this](const FVector& center, const FVector& extent, const FColor& color, bool persistent, float lifetime)
	{
        if (CVarDrawDebugs.GetValueOnGameThread() != 0 && drawDebug)
        {
            DrawDebugBox(GetOwner()->GetWorld(), center, extent, color, persistent, lifetime);
        }
	};
}

void UNavComponent::SetNavigatorLocation(const FVector& newTargetLocation)
{
    nav->SetNewTargetLocation(newTargetLocation);
}

FVector UNavComponent::GetNavLocation() const
{
	return nav ? nav->GetPosition() : FVector(0);
}

FVector UNavComponent::GetNavVelocity() const
{
	return nav ? nav->GetVelocity() : FVector(0);
}