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
			// ゲームオブジェクトの姿勢への参照を取得
			// ゲームオブジェクト自体の姿勢の変化をシミュレーションに加味する
			solver.OwnerTransformSceneHandle = animator.BindSceneTransform(animator.gameObject.transform);

			// シミュレーション対象のボーンを収集
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

			// ボーンの数などチェーンに関する情報を生成
			ConstraintLibrary.MakeVerticalStructure(ref solver, chains);

			MakeSoverParameters(ref solver, chains);

			BindBoneTransform(ref solver, chains, animator);

			// ボーン名のデバッグ表示
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
				// 初期化時はフレームレートが安定していない可能性が高いため固定値を使用
				simulationContext.DeltaTime = 1.0f / simulationContext.BaseFrameRate;
				simulationContext.DeltaTimeFactor = simulationContext.DeltaTime * simulationContext.BaseFrameRate;

				Vector3 position;
				Quaternion rotation;
				solver.OwnerTransformSceneHandle.GetGlobalTR(stream, out position, out rotation);
				simulationContext.OwnerTransform = simulationContext.PrevOwnerTransform = new STransform(position, rotation);
			}
			else
			{
				// テキトウなヒッチ対策
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
			// シミュレーション結果の位置と回転をアニメーションポーズとして出力する
			int transformHandleIndex = 0;
			int chainBeginIndex = 0;
			Span<Vector3> simPositions = stackalloc Vector3[4];
			Span<Vector3> simBoneDirections = stackalloc Vector3[4];
			foreach (VerticalStructure verticalStructure in solver.VerticalStructures)
			{
				// 末端の1つ前　previous end index
				int endIndexPrev = verticalStructure.PackedBoneCount - 1;
				for (int boneIndexOffset = 0; boneIndexOffset < verticalStructure.PackedBoneCount; ++boneIndexOffset)
				{
					int boneIndex = chainBeginIndex + boneIndexOffset;
					float4 dummyBoneMask = solver.DummyBoneMasks[boneIndex];

					// 位置を計算
					solver.SimPositions[boneIndex].Break(simPositions);

					// 回転を計算
					if (boneIndexOffset < endIndexPrev)
					{
						int nextBoneIndex = boneIndex + 1;

						// シミュレーション中のボーンを計算
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

						// 位置の反映
						TransformStreamHandle boneHandle = solver.BoneTransformStreamHandles[transformHandleIndex];
						boneHandle.SetPosition(stream, simPositions[registerIndex]);

						// 回転の反映
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

					// trueの場合が0.0fであることに注意
					// これにより何らかのパラメータの計算を乗算のみで0.0fにして無効化できる
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

		// 親から子に向かって再帰的にボーンを回収
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

				// 物理シミュレーションは分岐が含まれていない、1繋がりのチェーンを対象にしている
				// そのためチェーンが分岐している場合は別のチェーンとして切り分けている
				if (boneTransform.childCount >= 2)
				{
					newCain = new List<Bone>(chain);
				}

				GatherChains(child, newCain, chains);
			}
		}
	}
}

