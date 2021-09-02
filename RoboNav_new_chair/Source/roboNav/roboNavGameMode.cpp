// Copyright Epic Games, Inc. All Rights Reserved.

#include "roboNavGameMode.h"
#include "roboNavPlayerController.h"
#include "roboNavCharacter.h"
#include "UObject/ConstructorHelpers.h"

AroboNavGameMode::AroboNavGameMode()
{
	// use our custom PlayerController class
	PlayerControllerClass = AroboNavPlayerController::StaticClass();

	// set default pawn class to our Blueprinted character
	static ConstructorHelpers::FClassFinder<APawn> PlayerPawnBPClass(TEXT("/Game/TopDownCPP/Blueprints/TopDownCharacter"));
	if (PlayerPawnBPClass.Class != nullptr)
	{
		DefaultPawnClass = PlayerPawnBPClass.Class;
	}
}