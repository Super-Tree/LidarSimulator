// Fill out your copyright notice in the Description page of Project Settings.

#include "Carla.h"
#include "Lidar.h"
#include "stdlib.h"
#include <cmath>
#include "DrawDebugHelpers.h"
#include "Engine/CollisionProfile.h"
#include "Runtime/Engine/Classes/Kismet/KismetMathLibrary.h"
#include "StaticMeshResources.h"
#include<iostream>
using namespace std;
const float pitch_array_radin[32]={ -0.436332, -0.255481, -0.179437, -0.138056, -0.11212, -0.09437,
                                    -0.0814545, -0.0756251, -0.0698132, -0.0640012, -0.0581718, -0.0523599,
                                    -0.0465479, -0.0407185, -0.0349066, -0.0290946, -0.0232652, -0.0174533, -0.0116413,
                                    -0.00581195, 0.0, 0.00581195, 0.0116413, 0.0174533, 0.0232652, 0.0290946, 0.0407185,
                                     0.0581718, 0.0814545, 0.122173, 0.180345, 0.261799};
const float pitch_array_theta[32]={-25.0, -14.638, -10.281, -7.91, -6.424, -5.407, 
                                    -4.667, -4.333, -4.0, -3.667, -3.333, -3.0, 
                                    -2.667, -2.333, -2.0, -1.667, -1.333, -1.0, 
                                    -0.667, -0.333, 0.0, 0.333, 0.667, 1.0, 
                                    1.333, 1.667, 2.333, 3.333, 4.667, 7.0, 10.333, 15.0};

ALidar::ALidar(const FObjectInitializer& ObjectInitializer)
  : Super(ObjectInitializer)
{
  PrimaryActorTick.bCanEverTick = true;

  auto MeshComp = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("CamMesh0"));
  MeshComp->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);
  MeshComp->bHiddenInGame = true;
  MeshComp->CastShadow = false;
  MeshComp->PostPhysicsComponentTick.bCanEverTick = false;
  RootComponent = MeshComp;
}

void ALidar::Set(const ULidarDescription &LidarDescription)
{
  Super::Set(LidarDescription);
  Description = &LidarDescription;
  LidarMeasurement = FLidarMeasurement(GetId(), Description->Channels);
  CreateLasers();
}

void ALidar::CreateLasers()
{
  check(Description != nullptr);
  const auto NumberOfLasers = Description->Channels;
  check(NumberOfLasers > 0u);
  const float DeltaAngle =
    (Description->UpperFovLimit - Description->LowerFovLimit) /
    static_cast<float>(NumberOfLasers - 1);
  LaserAngles.Empty(NumberOfLasers);
  for(auto i = 0u; i < NumberOfLasers; ++i)
  {
    const float VerticalAngle = Description->UpperFovLimit - static_cast<float>(i) * DeltaAngle;
    // LaserAngles.Emplace(VerticalAngle);
    if (Description->LidarType == 1 )
      LaserAngles.Emplace(pitch_array_theta[i]);//claude
    else
      LaserAngles.Emplace(VerticalAngle);
  }
}

void ALidar::Tick(const float DeltaTime)
{
  Super::Tick(DeltaTime);
  float myDeltaTime = 1.0 / Description->RotationFrequency;//claude
  ReadPoints(myDeltaTime);
  WriteSensorData(LidarMeasurement.GetView());
}

double ALidar::GaussianKernel(double mu,double sigma)
{
  // using Box-Muller transform to generate two independent standard normally disbributed normal variables
  // see wikipedia
  double U = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
  double V = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
  double X = sqrt(-2.0 * ::log(U)) * cos( 2.0*M_PI * V);
  //double Y = sqrt(-2.0 * ::log(U)) * sin( 2.0*M_PI * V); // the other indep. normal variable
  // we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}

void ALidar::ReadPoints(const float DeltaTime)
{
  check(Description != nullptr);
  check(Description->Channels == LaserAngles.Num());
  
  float YawStart = 0.0;
  if(Description->LidarType ==2)//claude
  {
    YawStart = -Description->HorizonRange/2.0;
  }
  else
  {
    YawStart = 0.0;
  }

  const uint32 ChannelCount = Description->Channels;
  const uint32 PointsToScanWithOneLaser = FMath::RoundHalfFromZero(Description->PointsPerSecond * DeltaTime / float(ChannelCount));
  
  const float CurrentHorizontalAngle = LidarMeasurement.GetHorizontalAngle();
  const float AngleDistanceOfTick = Description->RotationFrequency * Description->HorizonRange * DeltaTime;
  const float AngleDistanceOfLaserMeasure = AngleDistanceOfTick / PointsToScanWithOneLaser;

  LidarMeasurement.Reset(ChannelCount * PointsToScanWithOneLaser);
  for(auto Channel = 0u; Channel < ChannelCount; ++Channel)
  {
    for(auto i = 0u; i < PointsToScanWithOneLaser; ++i)
    {
      FVector Point;
      const float Angle = YawStart + CurrentHorizontalAngle + AngleDistanceOfLaserMeasure * i;
      if(Description->DebugFlag == 1)
      {      
        cout<<"PointsToScanWithOneLaser"<<PointsToScanWithOneLaser<<"AngleDistanceOfLaserMeasure"<<AngleDistanceOfLaserMeasure<<"AngleDistanceOfTick"<<AngleDistanceOfTick<<endl;
        cout<<"DeltaTime"<<DeltaTime<<"CurrentHorizontalAngle"<<CurrentHorizontalAngle<<"Channel:"<<Channel<<"horizonidx"<<i<<"Angle:"<<Angle<<endl;
      }

      if (ShootLaser(Channel, Angle, Point))
      {
        float scale_nosie = GaussianKernel(0.0,Description->GaussianNoise);//claude 
        float yaw_theta = std::atan2((Point.Y),(Point.X+0.00001)); 
        float pitch_theta = std::atan2((Point.Z),std::sqrt(Point.X * Point.X + Point.Y*Point.Y)); 
        float x_noise = std::cos(yaw_theta)*std::cos(pitch_theta);
        float y_noise = std::sin(yaw_theta)*std::cos(pitch_theta);
        float z_noise = std::sin(pitch_theta);
        Point.X += x_noise*scale_nosie;
        Point.Y += y_noise*scale_nosie;
        Point.Z += z_noise*scale_nosie;
        float obj_distance2 = (Point.X * Point.X) + (Point.Y * Point.Y);
        float det_range2 = (Description->Range * Description->Range);
        float prob_keep = std::pow((det_range2 / (det_range2 + obj_distance2)),Description->DropOutPattern);
        if( (rand()%100)*0.01 < prob_keep )
          LidarMeasurement.WritePoint(Channel, Point);
      }
    }
  }
  //claude
  // const float HorizontalAngle = std::fmod(CurrentHorizontalAngle + AngleDistanceOfTick, 360.0f);
  LidarMeasurement.SetFrameNumber(GFrameCounter);
  // LidarMeasurement.SetHorizontalAngle(HorizontalAngle);
}

bool ALidar::ShootLaser(const uint32 Channel, const float HorizontalAngle, FVector &XYZ) const
{
  const float VerticalAngle = LaserAngles[Channel];

  FCollisionQueryParams TraceParams = FCollisionQueryParams(FName(TEXT("Laser_Trace")), true, this);
  TraceParams.bTraceComplex = true;
  TraceParams.bReturnPhysicalMaterial = false;

  FHitResult HitInfo(ForceInit);

  FVector LidarBodyLoc = GetActorLocation();
  FRotator LidarBodyRot = GetActorRotation();
  FRotator LaserRot (VerticalAngle, HorizontalAngle, 0);  // float InPitch, float InYaw, float InRoll
  FRotator ResultRot = UKismetMathLibrary::ComposeRotators(LaserRot,LidarBodyRot);
  const auto Range = Description->Range;
  FVector EndTrace = Range * UKismetMathLibrary::GetForwardVector(ResultRot) + LidarBodyLoc;

  GetWorld()->LineTraceSingleByChannel(HitInfo,LidarBodyLoc,EndTrace,ECC_MAX,TraceParams,FCollisionResponseParams::DefaultResponseParam);

  if (HitInfo.bBlockingHit)
  {
    if (Description->ShowDebugPoints)
    {
      DrawDebugPoint(GetWorld(),HitInfo.ImpactPoint,
        10,  //size
        FColor(255,0,255),
        false,  //persistent (never goes away)
        0.1  //point leaves a trail on moving object
      );
    }

    XYZ = LidarBodyLoc - HitInfo.ImpactPoint;
    XYZ = UKismetMathLibrary::RotateAngleAxis(XYZ,- LidarBodyRot.Yaw + 90,FVector(0, 0, 1));
    return true;
  } 
  else 
  {
    return false;
  }
}
