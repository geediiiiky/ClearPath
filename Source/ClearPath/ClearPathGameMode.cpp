// Fill out your copyright notice in the Description page of Project Settings.

#include "ClearPath.h"
#include "ClearPathGameMode.h"

#include "Navigator.h"


void AClearPathGameMode::Tick(float DeltaSeconds)
{
	time += DeltaSeconds;
	const auto fixedDT = 0.01f;
	while (time > fixedDT)
	{
		time -= fixedDT;
		NavigatorQuerier::Instance()->Update(fixedDT);
	}
    
   
    Super::Tick(DeltaSeconds);
}
