using UnityEngine;
using FuwaFuwa.Math;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Burst;
using Codice.CM.WorkspaceServer;

namespace FuwaFuwa
{
	[BurstCompile]
	public static class PhysicsLibrary
	{
		public static void ResetSimulationPose(ref DynamicBoneSolver solver)
		{
			NativeArray<SVector3>.Copy(solver.AnimPositions, solver.SimPositions);
		}

		public static void ResetVelocity(ref DynamicBoneSolver solver)
		{
			NativeArray<SVector3>.Copy(solver.SimPositions, solver.PrevSimPositions);
		}

		public static void AddForces(ref DynamicBoneSolver solver, in SimulationContext simulationContext)
		{
			float4 deltaTimeFactor = simulationContext.DeltaTime * simulationContext.BaseFrameRate;

			// èdóÕ
			SVector3 gravity = SVector3.UpVector * new float4(9.18f * simulationContext.DeltaTime * simulationContext.DeltaTime);
			for (int packedPositionIndex = 0; packedPositionIndex < solver.SimPositions.Length; ++packedPositionIndex)
			{
				solver.SimPositions[packedPositionIndex] -= gravity * solver.FixedMasks[packedPositionIndex] * solver.DummyBoneMasks[packedPositionIndex];
			}

			// épê®ïœâªÇ…î∫Ç§äOóÕ
			SVector3 worldVelocity = STransform.InverseTransformPosition(simulationContext.OwnerTransform, simulationContext.PrevOwnerTransform.Translation);
			SQuaternion worldAngularVelocity = STransform.InverseTransformRotation(simulationContext.OwnerTransform, simulationContext.PrevOwnerTransform.Rotation);
			float4 velocityDamping = new float4(0.98f);
			float4 angularVelocityDamping = new float4(0.4f);
			for (int packedPositionIndex = 0; packedPositionIndex < solver.SimPositions.Length; ++packedPositionIndex)
			{
				SVector3 prevSimPosition = solver.PrevSimPositions[packedPositionIndex];
				SVector3 linearizedWorldAngularVelocity = SQuaternion.RotateVector(worldAngularVelocity, prevSimPosition) - prevSimPosition;

				solver.SimPositions[packedPositionIndex] += (worldVelocity * velocityDamping + linearizedWorldAngularVelocity * angularVelocityDamping) * solver.FixedMasks[packedPositionIndex];
			}
		}

		public static void VerletIntegrate(ref DynamicBoneSolver solver, in SimulationContext simulationContext)
		{
			float4 deltaTimeFactor = simulationContext.DeltaTime * simulationContext.BaseFrameRate;
			for (int packedIndex = 0; packedIndex < solver.SimPositions.Length; ++packedIndex)
			{
				SVector3 copiedSimPosition = solver.SimPositions[packedIndex];
				SVector3 velocity = (copiedSimPosition - solver.PrevSimPositions[packedIndex]);
				SVector3 nextSimPosition = copiedSimPosition + (velocity * solver.FixedMasks[packedIndex] * solver.DummyBoneMasks[packedIndex]);

				solver.SimPositions[packedIndex] = nextSimPosition;
				solver.PrevSimPositions[packedIndex] = copiedSimPosition;
			}
		}
	};
}
