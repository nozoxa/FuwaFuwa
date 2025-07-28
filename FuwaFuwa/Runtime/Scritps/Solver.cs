using UnityEngine;
using UnityEngine.Animations;
using Unity.Mathematics;
using Unity.Collections;
using System;
using System.Collections.Generic;
using FuwaFuwa.Math;
using Codice.Client.Common.WebApi.Requests;
using UnityEditor.AnimatedValues;


namespace FuwaFuwa
{
	public struct Bone
	{
		public Bone(Transform transform, Vector3 position, bool isFirstBone = false, bool isDummy = false)
		{
			this.transform = transform;
			this.position = position;
			this.isFirstBone = isFirstBone;
			this.isDummy = isDummy;
		}

		public Transform transform;
		public Vector3 position;
		public bool isFirstBone;
		public bool isDummy;
	}

	public struct DynamicBoneSolver
	{
		public int ActualBoneCount;
		public int AllBoneCount;
		public TransformSceneHandle OwnerTransformSceneHandle;
		public NativeArray<TransformStreamHandle> BoneTransformStreamHandles;
		public NativeArray<SVector3> SimPositions;
		public NativeArray<SVector3> PrevSimPositions;
		public NativeArray<SVector3> AnimPositions;
		public NativeArray<float4> DummyBoneMasks;
		public NativeArray<float4> FixedMasks;
		public NativeArray<VerticalStructure> VerticalStructures;
		public NativeArray<float4> VerticalStructureLambdas;
	}

	public static class SolverLibrary
	{
		public static void InitializeSolver(ref DynamicBoneSolver solver, ChainSetting[] chainSettings, PhysicsSettings physicsSettings, Animator animator)
		{
			// �Q�[���I�u�W�F�N�g�̎p���ւ̎Q�Ƃ��擾
			// �Q�[���I�u�W�F�N�g���̂̎p���̕ω����V�~�����[�V�����ɉ�������
			solver.OwnerTransformSceneHandle = animator.BindSceneTransform(animator.gameObject.transform);

			// �V�~�����[�V�����Ώۂ̃{�[�������W
			solver.ActualBoneCount = 0;
			solver.AllBoneCount = 0;
			List<List<Bone>> chains = new List<List<Bone>>(chainSettings.Length);
			foreach (ChainSetting chainSetting in chainSettings)
			{
				foreach (Transform bone in chainSetting.RootBone)
				{
					List<Bone> chain = new List<Bone>();
					chain.Add(new Bone(chainSetting.RootBone, chainSetting.RootBone.position, true, false));

					GatherChains(bone, chain, chains);
				}
			}

			// �{�[���̐��Ȃǃ`�F�[���Ɋւ�����𐶐�
			ConstraintLibrary.MakeVerticalStructure(ref solver, chains);

			MakeSoverParameters(ref solver, chains);

			BindBoneTransform(ref solver, chains, animator);

			// �{�[�����̃f�o�b�O�\��
			foreach (List<Bone> chain in chains)
			{
				foreach (Bone bone in chain)
				{
					if (bone.transform == null)
					{
						continue;
					}

					Debug.Log(bone.transform.name);
				}
			}
		}

		public static void UninitializeSolver(ref DynamicBoneSolver solver)
		{
			solver.SimPositions.Dispose();
			solver.PrevSimPositions.Dispose();
			solver.AnimPositions.Dispose();
			solver.DummyBoneMasks.Dispose();
			solver.FixedMasks.Dispose();
			solver.VerticalStructures.Dispose();
			solver.VerticalStructureLambdas.Dispose();
			solver.BoneTransformStreamHandles.Dispose();
		}

		public static void UpdateSimulationContext(in DynamicBoneSolver solver, ref SimulationContext simulationContext, AnimationStream stream)
		{
			if (simulationContext.IsFirstUpdate)
			{
				// ���������̓t���[�����[�g�����肵�Ă��Ȃ��\�����������ߌŒ�l���g�p
				simulationContext.DeltaTime = 1.0f / simulationContext.BaseFrameRate;
				simulationContext.DeltaTimeFactor = simulationContext.DeltaTime * simulationContext.BaseFrameRate;

				Vector3 position;
				Quaternion rotation;
				solver.OwnerTransformSceneHandle.GetGlobalTR(stream, out position, out rotation);
				simulationContext.OwnerTransform = simulationContext.PrevOwnerTransform = new STransform(position, rotation);
			}
			else
			{
				// �e�L�g�E�ȃq�b�`�΍�
				simulationContext.DeltaTime = stream.deltaTime >= 0.066f ? 1.0f / simulationContext.BaseFrameRate : stream.deltaTime;
				simulationContext.DeltaTimeFactor = simulationContext.DeltaTime * simulationContext.BaseFrameRate;

				simulationContext.PrevOwnerTransform = simulationContext.OwnerTransform;
				Vector3 position;
				Quaternion rotation;
				solver.OwnerTransformSceneHandle.GetGlobalTR(stream, out position, out rotation);
				simulationContext.OwnerTransform = new STransform(position, rotation);
			}
		}

		public static void UpdateAnimationPose(ref DynamicBoneSolver solver, AnimationStream stream)
		{
			int transformHandleIndex = 0;
			for (int positionIndex = 0; positionIndex < solver.AnimPositions.Length; ++positionIndex)
			{
				float4 dummyBoneMask = solver.DummyBoneMasks[positionIndex];

				for (int registerIndex = 0; registerIndex < 4; ++registerIndex)
				{
					if (dummyBoneMask[registerIndex] <= 0.0f)
					{
						continue;
					}

					SVector3 animPosition = solver.AnimPositions[positionIndex];
					TransformStreamHandle boneHandle = solver.BoneTransformStreamHandles[transformHandleIndex];
					animPosition[registerIndex] = boneHandle.GetPosition(stream);
					solver.AnimPositions[positionIndex] = animPosition;
					++transformHandleIndex;
				}
			}
		}

		public static void UpdateFixedPositions(ref DynamicBoneSolver solver, AnimationStream stream)
		{
			int transformHandleIndex = 0;
			for (int positionIndex = 0; positionIndex < solver.AnimPositions.Length; ++positionIndex)
			{
				float4 dummyBoneMask = solver.DummyBoneMasks[positionIndex];
				float4 fixedMask = solver.FixedMasks[positionIndex];

				for (int registerIndex = 0; registerIndex < 4; ++registerIndex)
				{
					if (dummyBoneMask[registerIndex] <= 0.0f)
					{
						continue;
					}

					if (fixedMask[registerIndex] <= 0.0f)
					{
						SVector3 simPosition = solver.SimPositions[positionIndex];
						TransformStreamHandle boneHandle = solver.BoneTransformStreamHandles[transformHandleIndex];
						simPosition[registerIndex] = boneHandle.GetPosition(stream);
						solver.SimPositions[positionIndex] = simPosition;
					}

					++transformHandleIndex;
				}
			}
		}

		public static void ApplySimulationResult(in DynamicBoneSolver solver, AnimationStream stream)
		{
			// �V�~�����[�V�������ʂ̈ʒu�Ɖ�]���A�j���[�V�����|�[�Y�Ƃ��ďo�͂���
			int transformHandleIndex = 0;
			int chainBeginIndex = 0;
			Span<Vector3> simPositions = stackalloc Vector3[4];
			Span<Vector3> simBoneDirections = stackalloc Vector3[4];
			foreach (VerticalStructure verticalStructure in solver.VerticalStructures)
			{
				// ���[��1�O�@previous end index
				int endIndexPrev = verticalStructure.PackedBoneCount - 1;
				for (int boneIndexOffset = 0; boneIndexOffset < verticalStructure.PackedBoneCount; ++boneIndexOffset)
				{
					int boneIndex = chainBeginIndex + boneIndexOffset;
					float4 dummyBoneMask = solver.DummyBoneMasks[boneIndex];

					// �ʒu���v�Z
					solver.SimPositions[boneIndex].Break(simPositions);

					// ��]���v�Z
					if (boneIndexOffset < endIndexPrev)
					{
						int nextBoneIndex = boneIndex + 1;

						// �V�~�����[�V�������̃{�[�����v�Z
						SVector3 simPosition = solver.SimPositions[boneIndex];
						SVector3 nextSimPosition = solver.SimPositions[nextBoneIndex];
						SVector3 NeighbouringSimPosition = SMathLibrary.Shuffle(simPosition, nextSimPosition, RegisterComponent.LeftY, RegisterComponent.LeftZ, RegisterComponent.LeftW, RegisterComponent.RightX);
						SVector3 simBone = NeighbouringSimPosition - simPosition;
						simBone.Break(simBoneDirections);
					}

					for (int registerIndex = 0; registerIndex < 4; ++registerIndex)
					{
						if (dummyBoneMask[registerIndex] <= 0.0f)
						{
							continue;
						}

						// �ʒu�̔��f
						TransformStreamHandle boneHandle = solver.BoneTransformStreamHandles[transformHandleIndex];
						boneHandle.SetPosition(stream, simPositions[registerIndex]);

						// ��]�̔��f
						if (transformHandleIndex < solver.BoneTransformStreamHandles.Length - 1)
						{
							TransformStreamHandle childBoneHandle = solver.BoneTransformStreamHandles[transformHandleIndex + 1];
							Vector3 originalBoneDir = childBoneHandle.GetPosition(stream) - boneHandle.GetPosition(stream);
							boneHandle.SetRotation(stream, Quaternion.FromToRotation(originalBoneDir, simBoneDirections[registerIndex]) * boneHandle.GetRotation(stream));
						}

						++transformHandleIndex;
					}
				}

				chainBeginIndex += verticalStructure.PackedBoneCount;
			}
		}

		private static void MakeSoverParameters(ref DynamicBoneSolver solver, List<List<Bone>> chains)
		{
			int packedPositionCount = solver.AllBoneCount / 4;
			solver.SimPositions = new NativeArray<SVector3>(packedPositionCount, Allocator.Persistent);
			solver.DummyBoneMasks = new NativeArray<float4>(packedPositionCount, Allocator.Persistent);
			solver.FixedMasks = new NativeArray<float4>(packedPositionCount, Allocator.Persistent);
			int positionIndexBase = 0;
			foreach (List<Bone> chain in chains)
			{
				for (int boneIndex = 0; boneIndex < chain.Count; boneIndex += 4)
				{
					solver.SimPositions[positionIndexBase] = new SVector3(
						chain[boneIndex + 0].position,
						chain[boneIndex + 1].position,
						chain[boneIndex + 2].position,
						chain[boneIndex + 3].position
					);

					// true�̏ꍇ��0.0f�ł��邱�Ƃɒ���
					// ����ɂ�艽�炩�̃p�����[�^�̌v�Z����Z�݂̂�0.0f�ɂ��Ė������ł���
					solver.DummyBoneMasks[positionIndexBase] = new float4(
						chain[boneIndex + 0].isDummy ? 0.0f : 1.0f,
						chain[boneIndex + 1].isDummy ? 0.0f : 1.0f,
						chain[boneIndex + 2].isDummy ? 0.0f : 1.0f,
						chain[boneIndex + 3].isDummy ? 0.0f : 1.0f
					);

					solver.FixedMasks[positionIndexBase] = new float4(
						chain[boneIndex + 0].isFirstBone ? 0.0f : 1.0f,
						chain[boneIndex + 1].isFirstBone ? 0.0f : 1.0f,
						chain[boneIndex + 2].isFirstBone ? 0.0f : 1.0f,
						chain[boneIndex + 3].isFirstBone ? 0.0f : 1.0f
					);

					++positionIndexBase;
				}
			}

			solver.PrevSimPositions = new NativeArray<SVector3>(solver.SimPositions, Allocator.Persistent);
			solver.AnimPositions = new NativeArray<SVector3>(solver.SimPositions, Allocator.Persistent);

			solver.VerticalStructureLambdas = new NativeArray<float4>(packedPositionCount, Allocator.Persistent);
		}

		private static void BindBoneTransform(ref DynamicBoneSolver solver, List<List<Bone>> chains, Animator animator)
		{
			solver.BoneTransformStreamHandles = new NativeArray<TransformStreamHandle>(solver.ActualBoneCount, Allocator.Persistent);

			int boneCount = 0;
			foreach (List<Bone> bones in chains)
			{
				for (int boneIndex = 0; boneIndex < bones.Count; ++boneIndex)
				{
					if (bones[boneIndex].transform == null)
					{
						continue;
					}

					solver.BoneTransformStreamHandles[boneCount] = animator.BindStreamTransform(bones[boneIndex].transform);
					++boneCount;
				}
			}
		}

		// �e����q�Ɍ������čċA�I�Ƀ{�[�������
		private static void GatherChains(Transform boneTransform, List<Bone> chain, List<List<Bone>> chains)
		{
			chain.Add(new Bone(boneTransform, boneTransform.position));
			if (boneTransform.childCount <= 0)
			{
				chains.Add(chain);
				return;
			}

			foreach (Transform child in boneTransform)
			{
				List<Bone> newCain = chain;

				// �����V�~�����[�V�����͕��򂪊܂܂�Ă��Ȃ��A1�q����̃`�F�[����Ώۂɂ��Ă���
				// ���̂��߃`�F�[�������򂵂Ă���ꍇ�͕ʂ̃`�F�[���Ƃ��Đ؂蕪���Ă���
				if (boneTransform.childCount >= 2)
				{
					newCain = new List<Bone>(chain);
				}

				GatherChains(child, newCain, chains);
			}
		}
	}
}

