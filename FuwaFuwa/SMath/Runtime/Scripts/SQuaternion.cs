using UnityEngine;
using Unity.Burst;
using Unity.Mathematics;
using System;
using System.Runtime.CompilerServices;


namespace FuwaFuwa.Math
{
	[BurstCompile]
	public struct SQuaternion
	{
		public float4 x;
		public float4 y;
		public float4 z;
		public float4 w;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public SQuaternion(Quaternion x, Quaternion y, Quaternion z, Quaternion w)
		{
			this.x = new float4(x.x, y.x, z.x, w.x);
			this.y = new float4(x.y, y.y, z.y, w.y);
			this.z = new float4(x.z, y.z, z.z, w.z);
			this.w = new float4(x.w, y.w, z.w, w.w);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public SQuaternion(Quaternion q)
			: this(q, q, q, q)
		{
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public SQuaternion(float4 x, float4 y, float4 z, float4 w)
		{
			this.x = x;
			this.y = y;
			this.z = z;
			this.w = w;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static SQuaternion operator *(SQuaternion lhs, SQuaternion rhs)
		{
			return new SQuaternion(
				lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
				lhs.w * rhs.y + lhs.y * rhs.w + lhs.z * rhs.x - lhs.x * rhs.z,
				lhs.w * rhs.z + lhs.z * rhs.w + lhs.x * rhs.y - lhs.y * rhs.x,
				lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z
			);
		}

		public readonly void Break(Span<Quaternion> quaternions)
		{
			quaternions[0] = new Quaternion(x.x, y.x, z.x, w.x);
			quaternions[1] = new Quaternion(x.y, y.y, z.y, w.y);
			quaternions[2] = new Quaternion(x.z, y.z, z.z, w.z);
			quaternions[3] = new Quaternion(x.w, y.w, z.w, w.w);
		}

		public readonly void Break(out Quaternion q0, out Quaternion q1, out Quaternion q2, out Quaternion q3)
		{
			q0 = new Quaternion(x.x, y.x, z.x, w.x);
			q1 = new Quaternion(x.y, y.y, z.y, w.y);
			q2 = new Quaternion(x.z, y.z, z.z, w.z);
			q3 = new Quaternion(x.w, y.w, z.w, w.w);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static SQuaternion Inverse(SQuaternion q)
		{
			return new SQuaternion(-q.x, -q.y, -q.z, q.w);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static SVector3 RotateVector(SQuaternion q, SVector3 v)
		{
			SVector3 vecQ = new SVector3(-q.x, -q.y, -q.z);
			SVector3 tt = SMathConstans.TwoReal * SVector3.Cross(vecQ, v);
			return v + (q.w * tt) + SVector3.Cross(vecQ, tt);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static SVector3 UnrotateVector(SQuaternion q, SVector3 v)
		{
			SVector3 invQ = new SVector3(-q.x, -q.y, -q.z);
			SVector3 tt = SMathConstans.TwoReal * SVector3.Cross(invQ, v);
			return v + (q.w * tt) + SVector3.Cross(invQ, tt);
		}

		public static SQuaternion FromToRotation(SVector3 a, SVector3 b)
		{
			float4 normAB = SMathLibrary.Sqrt(SVector3.SqrMagnitude(a) * SVector3.SqrMagnitude(b));
			float4 w = normAB + SVector3.Dot(a, b);

			bool4 resultAMask = w >= SMathConstans.SmallReal * normAB;
			SQuaternion resultA = new SQuaternion(
				a.y * b.z - a.z * b.y,
				a.z * b.x - a.x * b.z,
				a.x * b.y - a.y * b.x,
				w
			);

			// 2‚Â‚ÌƒxƒNƒgƒ‹‚ª”½‘Î•ûŒü‚ðŒü‚¢‚Ä‚¢‚éê‡
			float4 absX = SMathLibrary.Abs(a.x);
			float4 absY = SMathLibrary.Abs(a.y);
			float4 absZ = SMathLibrary.Abs(a.z);

			bool4 orthogonalBasisMask = (absX > absY) & (absX > absZ);
			SVector3 orthogonalBasis = SMathLibrary.Select(orthogonalBasisMask, SVector3.YAxisVector, -SVector3.XAxisVector);
			SQuaternion resultB = new SQuaternion(
				a.y * b.z - a.z * b.y,
				a.z * b.x - a.x * b.z,
				a.x * b.y - a.y * b.x,
				SMathConstans.ZeroReal
			);

			return SMathLibrary.Select(resultAMask, resultA, resultB);
		}
	}
}