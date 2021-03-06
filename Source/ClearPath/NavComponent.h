// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Components/ActorComponent.h"
#include "Navigator.h"

#include <memory>

#include "NavComponent.generated.h"

UCLASS( Blueprintable, ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class CLEARPATH_API UNavComponent : public UActorComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UNavComponent();

	// Called when the game starts
	virtual void BeginPlay() override;
	
	// Called every frame
	virtual void TickComponent( float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction ) override;

	UFUNCTION(BlueprintNativeEvent, BlueprintCallable, Category = "NavComponent")
	void CreateNavigator(const FVector& newTargetLocation, float navRadius, float navMaxSpeed);
    
    UFUNCTION(BlueprintCallable, BlueprintCallable, Category = "NavComponent")
    void SetNavigatorLocation(const FVector& newTargetLocation);

	UFUNCTION(BlueprintCallable, Category = "NavComponent")
	FVector GetNavLocation() const;

	UFUNCTION(BlueprintCallable, Category = "NavComponent")
	FVector GetNavVelocity() const;
    
    UPROPERTY(EditAnywhere, BlueprintReadwrite)
    bool drawDebug = false;

private:
	std::shared_ptr<Navigator> nav;
};
