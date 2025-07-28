using UnityEngine;
using Unity.Burst;
using Unity.Mathematics;
using System.Runtime.CompilerServices;
using System;


namespace FuwaFuwa.Math
{
	[BurstCompile]
	public struct SVector3
	{
		public float4 x;
		public float4 y;
		public float4 z;

		private static readonly SVector3 _zeroVector = new SVector3(Vector3.zero);
		public static SVector3 ZeroVector => _zeroVector;

		private static readonly SVector3 _oneVector = new SVector3(Vector3.one);
		public static SVector3 OneVector => _oneVector;

		private static readonly SVector3 _upVector = new SVector3(SMathConstans.ZeroReal, SMathConstans.OneReal, SMathConstans.ZeroReal);
		public static SVector3 UpVector => _upVector;

		private static readonly SVector3 _rightVector = new SVector3(SMathConstans.OneReal, SMathConstans.ZeroReal, SMathConstans.ZeroReal);
		public static SVector3 RightVector => _rightVector;

		private static readonly SVector3 _forwardVector = new SVector3(SMathConstans.ZeroReal, SMathConstans.ZeroReal, SMathConstans.OneReal);
		public static SVector3 ForwardVector => _forwardVector;

		private static readonly SVector3 _yAxisVector = new SVector3(SMathConstans.ZeroReal, SMathConstans.OneReal, SMathConstans.ZeroReal);
		public static SVector3 YAxisVector => _yAxisVector;

		private static readonly SVector3 _xAxisVector = new SVector3(SMathConstans.OneReal, SMathConstans.ZeroReal, SMathConstans.ZeroReal);
		public static SVector3 XAxisVector => _xAxisVector;

		private static readonly SVector3 _zAxisVector = new SVector3(SMathConstans.ZeroReal, SMathConstans.ZeroReal, SMathConstans.OneReal);
		public static SVector3 ZAxisVector => _zAxisVector;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public SVector3(Vector3 x, Vector3 y, Vector3 z, Vector3 w)
		{
			this.x = new float4(x.x, y.x, z.x, w.x);
			this.y = new float4(x.y, y.y, z.y, w.y);
			this.z = new float4(x.z, y.z, z.z, w.z);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public SVector3(Vector3 v)
			: this(v, v, v, v)
		{
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public SVector3(float4 x, float4 y, float4 z)
		{
			this.x = x;
			this.y = y;
			this.z = z;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static SVector3 operator +(SVector3 lhs, SVector3 rhs)
		{
			return new SVector3(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static SVector3 operator -(SVector3 lhs, SVector3 rhs)
		{
			return new SVector3(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static SVector3 operator -(SVector3 a)
		{
			return new SVector3(-a.x, -a.y, -a.z);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static SVector3 operator *(float4 lhs, SVector3 rhs)
		{
			return new SVector3(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static SVector3 operator *(SVector3 lhs, float4 rhs)
		{
			return new SVector3(rhs * lhs.x, rhs * lhs.y, rhs * lhs.z);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static SVector3 operator *(SVector3 lhs, SVector3 rhs)
		{
			return new SVector3(lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static SVector3 operator /(SVector3 lhs, float4 rhs)
		{
			return new SVector3(lhs.x / rhs, lhs.y / rhs, lhs.z / rhs);
		}

		public Vector3 this[int index]
		{
			get
			{
				return new Vector3(x[index], y[index], z[index]);
			}
			set
			{
				x[index] = value.x;
				y[index] = value.y;
				z[index] = value.z;
			}
		}

		public readonly void Break(Span<Vector3> Vectors)
		{
			Vectors[0] = new Vector3(x.x, y.x, z.x);
			Vectors[1] = new Vector3(x.y, y.y, z.y);
			Vectors[2] = new Vector3(x.z, y.z, z.z);
			Vectors[3] = new Vector3(x.w, y.w, z.w);
		}

		public readonly void Break(out Vector3 v0, out Vector3 v1, out Vector3 v2, out Vector3 v3)
		{
			v0 = new Vector3(x.x, y.x, z.x);
			v1 = new Vector3(x.y, y.y, z.y);
			v2 = new Vector3(x.z, y.z, z.z);
			v3 = new Vector3(x.w, y.w, z.w);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static float4 Dot(SVector3 lhs, SVector3 rhs)
		{
			return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static SVector3 Cross(SVector3 lhs, SVector3 rhs)
		{
			return new SVector3(lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z, lhs.x * rhs.y - lhs.y * rhs.x);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static float4 Distance(SVector3 lhs, SVector3 rhs)
		{
			float4 x = lhs.x - rhs.x;
			float4 y = lhs.y - rhs.y;
			float4 z = lhs.z - rhs.z;

			return math.sqrt(x * x + y * y + z * z);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static float4 Magnitude(SVector3 v)
		{
			return math.sqrt((v.x * v.x) + (v.y * v.y) + (v.z * v.z));
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static float4 SqrMagnitude(SVector3 v)
		{
			return (v.x * v.x) + (v.y * v.y) + (v.z * v.z);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static SVector3 Normalize(SVector3 v)
		{
			float4 lenght = Magnitude(v);
			bool4 safeMask = lenght > SMathConstans.SmallReal;
			float4 safeLength = SMathLibrary.Select(safeMask, lenght, SMathConstans.OneReal);
			return SMathLibrary.Select(lenght > SMathConstans.SmallReal, v / safeLength, SVector3.ZeroVector);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static SVector3 GetSafeScaleReciprocalt(SVector3 scale)
		{
			SVector3 SafeReciprocalScale = SVector3.ZeroVector;
			SafeReciprocalScale.x = SMathLibrary.Select(math.abs(scale.x) > SMathConstans.SmallReal, SMathConstans.OneReal / scale.x, SMathConstans.ZeroReal);
			SafeReciprocalScale.y = SMathLibrary.Select(math.abs(scale.y) > SMathConstans.SmallReal, SMathConstans.OneReal / scale.y, SMathConstans.ZeroReal);
			SafeReciprocalScale.z = SMathLibrary.Select(math.abs(scale.z) > SMathConstans.SmallReal, SMathConstans.OneReal / scale.z, SMathConstans.ZeroReal);

			return SafeReciprocalScale;
		}
	};
}
