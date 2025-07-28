using UnityEngine;
using Unity.Mathematics;
using Unity.Burst;
using System.Runtime.CompilerServices;


namespace FuwaFuwa.Math
{
	public static class SMathConstans
	{
		private static readonly float4 _smallReal = new float4(1E-05f);
		public static float4 SmallReal => _smallReal;

		public static float4 ZeroReal => float4.zero;

		private static readonly float4 _oneReal = new float4(1.0f);
		public static float4 OneReal => _oneReal;

		private static readonly float4 _twoReal = new float4(2.0f);
		public static float4 TwoReal => _twoReal;
	}

	public enum RegisterComponent : byte
	{
		LeftX,
		LeftY,
		LeftZ,
		LeftW,
		RightX,
		RightY,
		RightZ,
		RightW
	};

	[BurstCompile]
	public static class SMathLibrary
	{
		// num Ç mutiple(î{êî) Ç…êÿÇËè„Ç∞ÇÈ
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static int CeilMultiple(int num, int mutiple)
		{
			return ((num + (mutiple - 1)) / mutiple) * mutiple;
		}

		// êîóÒÇÃïîï™òaÇãÅÇﬂÇÈ
		public static float4 PartialSum(float4 v)
		{
			float x = v.x;
			float xy = x + v.y;
			float xyz = xy + v.z;
			float xyzw = math.csum(v);

			return new float4(x, xy, xyz, xyzw);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static float4 Sqrt(float4 v)
		{
			return math.sqrt(v);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static float4 Abs(float4 v)
		{
			return math.abs(v);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static float4 Max(float4 a, float4 b)
		{
			return math.max(a, b);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static float4 Min(float4 a, float4 b)
		{
			return math.min(a, b);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static float4 Select(bool4 mask, float4 trueValue, float4 falseValue)
		{
			return math.select(falseValue, trueValue, mask);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static SVector3 Select(bool4 mask, SVector3 trueValue, SVector3 falseValue)
		{
			return new SVector3(
				SMathLibrary.Select(mask, trueValue.x, falseValue.x),
				SMathLibrary.Select(mask, trueValue.y, falseValue.y),
				SMathLibrary.Select(mask, trueValue.z, falseValue.z)
			);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static SQuaternion Select(bool4 mask, SQuaternion trueValue, SQuaternion falseValue)
		{
			return new SQuaternion(
				SMathLibrary.Select(mask, trueValue.x, falseValue.x),
				SMathLibrary.Select(mask, trueValue.y, falseValue.y),
				SMathLibrary.Select(mask, trueValue.z, falseValue.z),
				SMathLibrary.Select(mask, trueValue.w, falseValue.w)
			);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static float4 Shuffle(float4 left, float4 right, RegisterComponent x, RegisterComponent y, RegisterComponent z, RegisterComponent w)
		{
			return math.shuffle(left, right, (math.ShuffleComponent)x, (math.ShuffleComponent)y, (math.ShuffleComponent)z, (math.ShuffleComponent)w);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static SVector3 Shuffle(SVector3 left, SVector3 right, RegisterComponent x, RegisterComponent y, RegisterComponent z, RegisterComponent w)
		{
			return new SVector3(
				SMathLibrary.Shuffle(left.x, right.x, x, y, z, w),
				SMathLibrary.Shuffle(left.y, right.y, x, y, z, w),
				SMathLibrary.Shuffle(left.z, right.z, x, y, z, w)
			);
		}
	}
}
