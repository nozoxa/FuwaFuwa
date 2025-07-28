using UnityEngine;
using Unity.Mathematics;
using Unity.Burst;
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

	[BurstCompile]
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

				// �`�F�[���̖����Ƀ{�[����}�����āA���[�̋��������P
				InsertLeafBone(bones);
				verticalStructure.DummyBoneCount = 1;

				// SIMD�ɂ��v�Z���ȒP�ɂ��邽�߂Ƀ_�~�[�{�[����ǉ�
				int addjustChainLength = SMathLibrary.CeilMultiple(bones.Count + 4, 4);
				int dummyBoneCount = addjustChainLength - bones.Count;
				FillSIMDRegisterWithDummyBone(bones, dummyBoneCount);
				verticalStructure.DummyBoneCount += dummyBoneCount;

				solver.AllBoneCount += bones.Count;
				verticalStructure.PackedBoneCount = SMathLibrary.CeilMultiple(bones.Count, 4) / 4;
				solver.VerticalStructures[chainIndex] = verticalStructure;
			}
		}

		public static void ConstrainVerticalStructure(ref DynamicBoneSolver solver, ref SimulationContext context, ref PhysicsSettings physicsSettings)
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

					// �A�j���[�V�����|�[�Y�̃{�[�����v�Z
					SVector3 animPosePosition = solver.AnimPositions[boneIndex];
					SVector3 nextAnimPosePosition = solver.AnimPositions[nextBoneIndex];
					SVector3 neighbouringAnimPosition = SMathLibrary.Shuffle(animPosePosition, nextAnimPosePosition, RegisterComponent.LeftY, RegisterComponent.LeftZ, RegisterComponent.LeftW, RegisterComponent.RightX);
					SVector3 animBone = neighbouringAnimPosition - animPosePosition;

					// �V�~�����[�V�������̃{�[�����v�Z
					SVector3 simPosition = solver.SimPositions[boneIndex];
					SVector3 nextSimPosition = solver.SimPositions[nextBoneIndex];
					SVector3 neighbouringSimPosition = SMathLibrary.Shuffle(simPosition, nextSimPosition, RegisterComponent.LeftY, RegisterComponent.LeftZ, RegisterComponent.LeftW, RegisterComponent.RightX);
					SVector3 simBone = neighbouringSimPosition - simPosition;

					// �A�j���[�V�����|�[�Y�̃{�[������ɂǂꂾ���L�тĂ���̂������߂�
					float4 animBoneLength = SVector3.Magnitude(animBone);
					float4 simBoneLength = SVector3.Magnitude(simBone);

					float4 lambda = solver.VerticalStructureLambdas[boneIndex];
					float4 deltaLambda = ComputeDeltaLambda(simBoneLength, animBoneLength, inverseMass, compliance, lambda, sqrDeltaTime);
					solver.VerticalStructureLambdas[boneIndex] += deltaLambda;
					SVector3 constraint = deltaLambda * SVector3.Normalize(simBone);

					// �L�т�����߂�
					float4 fixedMask = new float4(solver.FixedMasks[boneIndex].yzw, solver.FixedMasks[nextBoneIndex].x);
					SVector3 ConstraintedSimPos = neighbouringSimPosition - constraint;
					solver.SimPositions[boneIndex] = SMathLibrary.Shuffle(simPosition, ConstraintedSimPos, RegisterComponent.LeftX, RegisterComponent.RightX, RegisterComponent.RightY, RegisterComponent.RightZ) * fixedMask;
					solver.SimPositions[nextBoneIndex] = SMathLibrary.Shuffle(ConstraintedSimPos, nextSimPosition, RegisterComponent.LeftW, RegisterComponent.RightY, RegisterComponent.RightZ, RegisterComponent.RightW) * fixedMask;

					// �O�t���[���̍��W�����������ړ����A�x�����ϕ��ŕs�v�ȑ��x���v�Z���Ȃ��悤�ɂ���
					// ������s��Ȃ��ꍇ�A����ɂ��ʒu�ω��̕����ɉ������ĕ����̖\��ɂȂ���\��������
					SVector3 prevSimPosition = solver.PrevSimPositions[boneIndex];
					SVector3 prevNextSimPosition = solver.PrevSimPositions[nextBoneIndex];
					SVector3 prevNeighbouringSimPosition = SMathLibrary.Shuffle(prevSimPosition, prevNextSimPosition, RegisterComponent.LeftY, RegisterComponent.LeftZ, RegisterComponent.LeftW, RegisterComponent.RightX);
					SVector3 prevConstraintedSimPos = neighbouringSimPosition - constraint;
					solver.PrevSimPositions[boneIndex] = SMathLibrary.Shuffle(prevSimPosition, prevConstraintedSimPos, RegisterComponent.LeftX, RegisterComponent.RightX, RegisterComponent.RightY, RegisterComponent.RightZ) * fixedMask;
					solver.PrevSimPositions[nextBoneIndex] = SMathLibrary.Shuffle(prevConstraintedSimPos, prevNextSimPosition, RegisterComponent.LeftW, RegisterComponent.RightY, RegisterComponent.RightZ, RegisterComponent.RightW) * fixedMask;
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
