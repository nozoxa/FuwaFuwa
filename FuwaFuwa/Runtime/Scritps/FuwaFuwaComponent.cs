using UnityEngine;
using UnityEngine.Assertions.Must;
using UnityEngine.Animations;
using UnityEngine.Playables;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using System;
using System.Collections.Generic;
using FuwaFuwa;
using FuwaFuwa.Math;


namespace FuwaFuwa
{
	[Serializable]
	public struct ChainSetting
	{
		public Transform RootBone;
	}

	public enum ComplianceType : byte
	{
		Concrete,
		Wood,
		Leather,
		Tendon,
		Rubber,
		Muscle,
		Fat,
		Max,
	};

	[Serializable]
	public struct PhysicsSettings
	{
		public ComplianceType VerticalStructureConstraintStiffness;
		public int SolverIterations;
	}

	public struct SimulationContext
	{
		public bool IsFirstUpdate;
		public float BaseFrameRate;
		public float DeltaTime;
		public float DeltaTimeFactor;
		public STransform PrevOwnerTransform;
		public STransform OwnerTransform;
	}

	[RequireComponent(typeof(Animator))]
	public sealed class FuwaFuwaComponent : MonoBehaviour
	{
		[SerializeField]
		private ChainSetting[] _chainSettings;

		[SerializeField]
		private PhysicsSettings _physicsSettings = new PhysicsSettings { VerticalStructureConstraintStiffness = ComplianceType.Leather, SolverIterations = 8 };

		[SerializeField]
		private AnimationClip _animClip;

		private NativeArray<VerticalStructure> _verticalStructures;
		private NativeArray<Vector3> _bonePositions;

		private DynamicBoneSolver _solver;
		private PlayableGraph _graph;

		// https://blog.mmacklin.com/2016/10/12/xpbd-slides-and-stiffness/
		public static readonly float[] Compliance = new float[(int)ComplianceType.Max]
		{
			0.00000000004f,
			0.00000000016f,
			0.000000001f,
			0.000000002f,
			0.0000001f,
			0.00002f,
			0.0001f,
		};

		private float _time = 0.0f;
		private Vector3 _initialPos;

		private void Initialize()
		{
			_initialPos = gameObject.transform.localPosition;

			Application.targetFrameRate = 30;

			// �\���o�[�̐���
			Animator animator = GetComponent<Animator>();
			SolverLibrary.InitializeSolver(ref _solver, _chainSettings, _physicsSettings, animator);

			// GameObject��ʂ��ă{�[���𓮂����Ȃ����ߍ폜
			AnimatorUtility.OptimizeTransformHierarchy(gameObject, null);

			// �V�~�����[�V������PlayableAPI�Ŏ��s
			_graph = PlayableGraph.Create("FuwaFuwa Job");
			_graph.SetTimeUpdateMode(DirectorUpdateMode.GameTime);
			FuwaFuwaJob job = new FuwaFuwaJob
			{
				Solver = _solver,
				SimulationContext = new SimulationContext { IsFirstUpdate = true, BaseFrameRate = 60 },
				PhysicsSettings = _physicsSettings,
			};

			AnimationScriptPlayable scriptPlayable = AnimationScriptPlayable.Create(_graph, job, 1);

			// �A�j���[�V�����N���b�v�̃m�[�h��ڑ�
			if (_animClip != null)
			{
				AnimationClipPlayable animClipPlayable = AnimationClipPlayable.Create(_graph, _animClip);
				_graph.Connect(animClipPlayable, 0, scriptPlayable, 0);
			}

			AnimationPlayableOutput playableOutput = AnimationPlayableOutput.Create(_graph, "Animation Output", animator);
			playableOutput.SetSourcePlayable(scriptPlayable);
			_graph.Play();
		}

		private void Uninitialize()
		{
			_graph.Destroy();
			SolverLibrary.UninitializeSolver(ref _solver);
		}

		void Start()
		{
			Initialize();
		}

		void Update()
		{
			float sin = Mathf.Sin(_time) * 0.5f;
			gameObject.transform.localPosition = _initialPos + new Vector3(1.0f, 0.0f, 0.0f) * sin;
			_time += Time.deltaTime;
		}

		private void OnDestroy()
		{
			Uninitialize();
		}
	}

	[BurstCompile]
	public struct FuwaFuwaJob : IAnimationJob
	{
		public float DeltaTime;
		public DynamicBoneSolver Solver;
		public SimulationContext SimulationContext;
		public PhysicsSettings PhysicsSettings;

		public void ProcessAnimation(AnimationStream stream)
		{
			SolverLibrary.UpdateSimulationContext(in Solver, ref SimulationContext, stream);

			SolverLibrary.UpdateAnimationPose(ref Solver, stream);

			SolverLibrary.UpdateFixedPositions(ref Solver, stream);

			if (SimulationContext.IsFirstUpdate)
			{
				PhysicsLibrary.ResetSimulationPose(ref Solver);
				PhysicsLibrary.ResetVelocity(ref Solver);
				SimulationContext.IsFirstUpdate = false;
			}

			PhysicsLibrary.AddForces(ref Solver, in SimulationContext);

			PhysicsLibrary.VerletIntegrate(ref Solver, in SimulationContext);

			ConstraintLibrary.ResetLambdas(ref Solver);

			for (int i = 0; i < PhysicsSettings.SolverIterations; ++i)
			{
				ConstraintLibrary.ConstraintVerticalStructure(ref Solver, ref SimulationContext, ref PhysicsSettings);
			}

			SolverLibrary.ApplySimulationResult(in Solver, stream);
		}

		public void ProcessRootMotion(AnimationStream stream)
		{
		}
	};
}
