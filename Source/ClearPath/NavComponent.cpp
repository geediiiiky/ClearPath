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
	if (nav)
	{
		nav->Update(DeltaTime);
	}
}

void UNavComponent::CreateNavigator_Implementation(const FVector& newTargetLocation, float navRadius, float navMaxSpeed)
{
	nav = std::make_unique<Navigator>(navRadius, navMaxSpeed, GetOwner()->GetActorLocation(), newTargetLocation);
}

FVector UNavComponent::GetNavLocation() const
{
	return nav->GetPosition();
}

FVector UNavComponent::GetNavVelocity() const
{
	return nav->GetVelocity();
}