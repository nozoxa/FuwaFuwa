using UnityEngine;
using Unity.Burst;
using Unity.Mathematics;
using System.Runtime.CompilerServices;


namespace FuwaFuwa.Math
{
	[BurstCompile]
	public struct STransform
	{
		public SVector3 Translation;
		public SQuaternion Rotation;
		public SVector3 Scale3D;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public STransform(Vector3 translation, Quaternion rotation, Vector3 scale3D)
		{
			this.Translation = new SVector3(translation);
			this.Rotation = new SQuaternion(rotation);
			this.Scale3D = new SVector3(scale3D);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public STransform(Vector3 translation, Quaternion rotation)
			: this(translation, rotation, Vector3.one)
		{
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static SVector3 InverseTransformPosition(STransform transform, SVector3 v)
		{
			return SQuaternion.UnrotateVector(transform.Rotation, (v - transform.Translation)) * SVector3.GetSafeScaleReciprocalt(transform.Scale3D);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static SQuaternion InverseTransformRotation(STransform transform, SQuaternion q)
		{
			return SQuaternion.Inverse(transform.Rotation) * q;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static SVector3 InverseTransformVector(STransform transform, SVector3 v)
		{
			return SQuaternion.UnrotateVector(transform.Rotation, v) * SVector3.GetSafeScaleReciprocalt(transform.Scale3D);
		}
	}
}
