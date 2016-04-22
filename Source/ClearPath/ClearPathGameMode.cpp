// Fill out your copyright notice in the Description page of Project Settings.

#include "ClearPath.h"
#include "ClearPathGameMode.h"

#include "Navigator.h"


void AClearPathGameMode::Tick(float DeltaSeconds)
{
    NavigatorQuerier::Instance()->Update(DeltaSeconds);
    NavigatorQuerier::Instance()->Update2(DeltaSeconds);
    
   
    Super::Tick(DeltaSeconds);
}
