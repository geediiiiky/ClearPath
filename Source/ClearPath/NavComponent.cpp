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

	location = GetOwner()->GetActorLocation();
	
}


// Called every frame
void UNavComponent::TickComponent( float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction )
{
	Super::TickComponent( DeltaTime, TickType, ThisTickFunction );

	FVector toTarget = targetLocation - location;
	FVector velocity = toTarget.GetSafeNormal() * maxSpeed;

	location = location + velocity * DeltaTime;
}

