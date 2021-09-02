// Copyright Epic Games, Inc. All Rights Reserved.

#include "roboNavPlayerController.h"
#include "Blueprint/AIBlueprintHelperLibrary.h"
#include "Runtime/Engine/Classes/Components/DecalComponent.h"
#include "HeadMountedDisplayFunctionLibrary.h"
#include "roboNavCharacter.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "Engine/World.h"


AroboNavPlayerController::AroboNavPlayerController()
{
	bShowMouseCursor = true;
	DefaultMouseCursor = EMouseCursor::Crosshairs;
}

//void AroboNavPlayerController::PlayerTick(float DeltaTime)
//{
//	Super::PlayerTick(DeltaTime);
//
//	// keep updating the destination every tick while desired
//	if (bMoveToMouseCursor)
//	{
//		MoveToMouseCursor();
//	}
//}

void AroboNavPlayerController::SetupInputComponent()
{
	// set up gameplay key bindings
	Super::SetupInputComponent();

	//InputComponent->BindAction("SetDestination", IE_Pressed, this, &AroboNavPlayerController::OnSetDestinationPressed);
	//InputComponent->BindAction("SetDestination", IE_Released, this, &AroboNavPlayerController::OnSetDestinationReleased);

	//InputComponent->BindAxis("MoveForward", this, &AroboNavPlayerController::MoveForward);
	//InputComponent->BindAxis("MoveRight", this, &AroboNavPlayerController::MoveRight);

	// support touch devices 
	//InputComponent->BindTouch(EInputEvent::IE_Pressed, this, &AroboNavPlayerController::MoveToTouchLocation);
	//InputComponent->BindTouch(EInputEvent::IE_Repeat, this, &AroboNavPlayerController::MoveToTouchLocation);

	//InputComponent->BindAction("ResetVR", IE_Pressed, this, &AroboNavPlayerController::OnResetVR);
}

void AroboNavPlayerController::OnResetVR()
{
	UHeadMountedDisplayFunctionLibrary::ResetOrientationAndPosition();
}

void AroboNavPlayerController::MoveToMouseCursor()
{
	if (UHeadMountedDisplayFunctionLibrary::IsHeadMountedDisplayEnabled())
	{
		if (AroboNavCharacter* MyPawn = Cast<AroboNavCharacter>(GetPawn()))
		{
			if (MyPawn->GetCursorToWorld())
			{
				UAIBlueprintHelperLibrary::SimpleMoveToLocation(this, MyPawn->GetCursorToWorld()->GetComponentLocation());
			}
		}
	}
	else
	{
		// Trace to see what is under the mouse cursor
		FHitResult Hit;
		GetHitResultUnderCursor(ECC_Visibility, false, Hit);

		if (Hit.bBlockingHit)
		{
			// We hit something, move there
			SetNewMoveDestination(Hit.ImpactPoint);
		}
	}
}

void AroboNavPlayerController::MoveToTouchLocation(const ETouchIndex::Type FingerIndex, const FVector Location)
{
	FVector2D ScreenSpaceLocation(Location);

	// Trace to see what is under the touch location
	FHitResult HitResult;
	GetHitResultAtScreenPosition(ScreenSpaceLocation, CurrentClickTraceChannel, true, HitResult);
	if (HitResult.bBlockingHit)
	{
		// We hit something, move there
		SetNewMoveDestination(HitResult.ImpactPoint);
	}
}



void AroboNavPlayerController::SetNewMoveDestination(const FVector DestLocation)
{
	APawn* const MyPawn = GetPawn();
	if (MyPawn)
	{
		float const Distance = FVector::Dist(DestLocation, MyPawn->GetActorLocation());

		// We need to issue move command only if far enough in order for walk animation to play correctly
		if ((Distance > 120.0f))
		{
			UAIBlueprintHelperLibrary::SimpleMoveToLocation(this, DestLocation);
		}
	}
}

void AroboNavPlayerController::OnSetDestinationPressed()
{
	// set flag to keep updating destination until released
	bMoveToMouseCursor = true;
}

void AroboNavPlayerController::OnSetDestinationReleased()
{
	// clear flag to indicate we should stop updating the destination
	bMoveToMouseCursor = false;
}


//void AroboNavPlayerController::MoveForward(float Axis) {
//	APawn* const MyPawn = GetPawn();
//	FRotator Rotation = MyPawn->GetControlRotation();
//	FRotator YawRotation(0.0f, Rotation.Yaw, 0.0f);
//	FVector Direction = FRotationMatrix(YawRotation).GetUnitAxis(EAxis::X);
//	MyPawn->AddMovementInput(Direction, Axis);
//
//}
//void AroboNavPlayerController::MoveRight(float Axis) {
//	APawn* const MyPawn = GetPawn();
//	FRotator Rotation = MyPawn->GetControlRotation();
//	FRotator YawRotation(0.0f, Rotation.Yaw, 0.0f);
//	FVector Direction = FRotationMatrix(YawRotation).GetUnitAxis(EAxis::Y);
//	MyPawn->AddMovementInput(Direction, Axis);
//}

