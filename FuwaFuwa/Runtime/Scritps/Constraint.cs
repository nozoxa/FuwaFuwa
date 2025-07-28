using UnityEngine;
using Unity.Mathematics;
using System.Collections.Generic;
using Unity.Collections;
using FuwaFuwa.Math;
using System;
using UnityEngine.Animations;


namespace FuwaFuwa
{
	public struct VerticalStructure
	{
		public int PackedBoneCount;
		public int DummyBoneCount;
	}

	public struct VerticalConstraint
	{
		public int PackedFirstBoneIndex;
		public int PackedSecondBoneIndex;
	}

	public static class ConstraintLibrary
	{
		public static void InsertLeafBone(List<Bone> chain)
		{
			Bone leafBone = new Bone();
			Bone firstBone = chain[^2];
			Bone secondBone = chain[^1];
			leafBone.transform = null;
			leafBone.position = firstBone.position + (secondBone.position - firstBone.position);
			leafBone.isDummy = true;
			chain.Add(leafBone);
		}

		public static void FillSIMDRegisterWithDummyBone(List<Bone> chain, int dummyBoneCount)
		{
			for (int i = 0; i < dummyBoneCount; ++i)
			{
				Bone dummyBone = new Bone();
				Bone firstBone = chain[^2];
				Bone secondBone = chain[^1];
				dummyBone.transform = null;
				dummyBone.position = firstBone.position + (secondBone.position - firstBone.position);
				dummyBone.isDummy = true;
				chain.Add(dummyBone);
			}
		}

		public static void MakeVerticalStructure(ref DynamicBoneSolver solver, List<List<Bone>> chains)
		{
			solver.VerticalStructures = new NativeArray<VerticalStructure>(chains.Count, Allocator.Persistent);

			for (int chainIndex = 0; chainIndex < chains.Count; ++chainIndex)
			{
				List<Bone> bones = chains[chainIndex];
				if (bones.Count <= 1)
				{
					Debug.LogWarning("Chain to be simulated must contain at least two bones");
					return;
				}

				solver.ActualBoneCount += bones.Count;

				VerticalStructure verticalStructure = new VerticalStructure();

				// チェーンの末尾にボーンを挿入して、末端の挙動を改善
				InsertLeafBone(bones);
				verticalStructure.DummyBoneCount = 1;

				// SIMDによる計算を簡単にするためにダミーボーンを追加
				int addjustChainLength = SMathLibrary.CeilMultiple(bones.Count + 4, 4);
				int dummyBoneCount = addjustChainLength - bones.Count;
				FillSIMDRegisterWithDummyBone(bones, dummyBoneCount);
				verticalStructure.DummyBoneCount += dummyBoneCount;

				solver.AllBoneCount += bones.Count;
				verticalStructure.PackedBoneCount = SMathLibrary.CeilMultiple(bones.Count, 4) / 4;
				solver.VerticalStructures[chainIndex] = verticalStructure;
			}
		}

		public static void ConstraintVerticalStructure(ref DynamicBoneSolver solver, ref SimulationContext context, ref PhysicsSettings physicsSettings)
		{
			float4 sqrDeltaTime = context.DeltaTime * context.DeltaTime;
			float4 inverseMass = SMathConstans.TwoReal;
			float4 compliance = FuwaFuwaComponent.Compliance[(int)physicsSettings.VerticalStructureConstraintStiffness];

			int chainBeginIndex = 0;
			foreach (VerticalStructure verticalStructure in solver.VerticalStructures)
			{
				int endIndex = verticalStructure.PackedBoneCount - 1;
				for (int boneIndexOffset = 0; boneIndexOffset < endIndex; ++boneIndexOffset)
				{
					int boneIndex = chainBeginIndex + boneIndexOffset;
					int nextBoneIndex = boneIndex + 1;

					// アニメーションポーズのボーンを計算
					SVector3 animPosePosition = solver.AnimPositions[boneIndex];
					SVector3 NextAnimPosePosition = solver.AnimPositions[nextBoneIndex];
					SVector3 NeighbouringAnimPosition = SMathLibrary.Shuffle(animPosePosition, NextAnimPosePosition, RegisterComponent.LeftY, RegisterComponent.LeftZ, RegisterComponent.LeftW, RegisterComponent.RightX);
					SVector3 animBone = NeighbouringAnimPosition - animPosePosition;

					// シミュレーション中のボーンを計算
					SVector3 simPosition = solver.SimPositions[boneIndex];
					SVector3 nextSimPosition = solver.SimPositions[nextBoneIndex];
					SVector3 NeighbouringSimPosition = SMathLibrary.Shuffle(simPosition, nextSimPosition, RegisterComponent.LeftY, RegisterComponent.LeftZ, RegisterComponent.LeftW, RegisterComponent.RightX);
					SVector3 simBone = NeighbouringSimPosition - simPosition;

					// アニメーションポーズのボーンを基準にどれだけ伸びているのかを求める
					float4 animBoneLength = SVector3.Magnitude(animBone);
					float4 simBoneLength = SVector3.Magnitude(simBone);
					float4 constraint = math.max(simBoneLength - (animBoneLength), SMathConstans.ZeroReal);
					constraint = SMathLibrary.PartialSum(constraint);

					float4 lambda = solver.VerticalStructureLambdas[boneIndex];
					float4 deltaLambda = ComputeDeltaLambda(simBoneLength, animBoneLength, inverseMass, compliance, lambda, sqrDeltaTime);
					solver.VerticalStructureLambdas[boneIndex] += deltaLambda;

					// 伸びた分を戻す
					float4 fixedMask = new float4(solver.FixedMasks[boneIndex].yzw, solver.FixedMasks[nextBoneIndex].x);
					//SVector3 ConstraintedSimPos = NeighbouringSimPosition - (constraint * SVector3.Normalize(simBone));
					SVector3 ConstraintedSimPos = NeighbouringSimPosition - (deltaLambda * SVector3.Normalize(simBone));

					solver.SimPositions[boneIndex] = SMathLibrary.Shuffle(simPosition, ConstraintedSimPos, RegisterComponent.LeftX, RegisterComponent.RightX, RegisterComponent.RightY, RegisterComponent.RightZ) * fixedMask;
					solver.SimPositions[nextBoneIndex] = SMathLibrary.Shuffle(ConstraintedSimPos, nextSimPosition, RegisterComponent.LeftW, RegisterComponent.RightY, RegisterComponent.RightZ, RegisterComponent.RightW) * fixedMask;
				}

				chainBeginIndex += verticalStructure.PackedBoneCount;
			}
		}

		public static void ResetLambdas(ref DynamicBoneSolver solver)
		{
			for (int i = 0; i < solver.VerticalStructureLambdas.Length; ++i)
			{
				solver.VerticalStructureLambdas[i] = SMathConstans.ZeroReal;
			}
		}

		private static float4 ComputeDeltaLambda(float4 lenght, float4 desiredLenght, float4 inverseMass, float4 compliance, float4 lambda, float4 sqrDeltaTime)
		{
			float4 constraint = SMathLibrary.Max(lenght - desiredLenght, SMathConstans.ZeroReal);
			float4 complianceTilda = compliance / sqrDeltaTime;
			return (constraint - complianceTilda * lambda) / (inverseMass + complianceTilda);
		}
	}
}
