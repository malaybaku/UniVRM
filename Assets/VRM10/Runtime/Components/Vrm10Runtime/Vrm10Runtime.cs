using System;
using System.Collections.Generic;
using System.Linq;
using UniGLTF;
using UniGLTF.Utils;
using UnityEngine;
using UniVRM10.FastSpringBones.Blittables;
using UniVRM10.FastSpringBones.System;

namespace UniVRM10
{
    /// <summary>
    /// VRM モデルインスタンスを、状態をもって、元の状態から操作・変更するためのクラス。
    /// また、仕様に従ってその操作を行う。
    ///
    /// 操作対象としては以下が挙げられる。
    /// - ControlRig
    /// - Constraint
    /// - LookAt
    /// - Expression
    /// </summary>
    public class Vrm10Runtime : IDisposable
    {
        private readonly Vrm10Instance m_target;
        private readonly Transform m_head;
        private readonly FastSpringBoneService m_fastSpringBoneService;
        private readonly IReadOnlyDictionary<Transform, TransformState> m_defaultTransformStates;

        private FastSpringBoneBuffer m_fastSpringBoneBuffer;

        /// <summary>
        /// Control Rig may be null.
        /// Control Rig is generated at loading runtime only.
        /// </summary>
        public Vrm10RuntimeControlRig ControlRig { get; }

        public IVrm10Constraint[] Constraints { get; }
        public Vrm10RuntimeExpression Expression { get; }
        public Vrm10RuntimeLookAt LookAt { get; }

        public Vrm10Runtime(Vrm10Instance target, ControlRigGenerationOption controlRigGenerationOption)
        {
            m_target = target;

            if (!target.TryGetBoneTransform(HumanBodyBones.Head, out m_head))
            {
                throw new Exception();
            }

            if (controlRigGenerationOption != ControlRigGenerationOption.None)
            {
                ControlRig = new Vrm10RuntimeControlRig(target.Humanoid, m_target.transform, controlRigGenerationOption);
            }
            Constraints = target.GetComponentsInChildren<IVrm10Constraint>();
            LookAt = new Vrm10RuntimeLookAt(target.Vrm.LookAt, target.Humanoid, m_head, target.LookAtTargetType, target.Gaze);
            Expression = new Vrm10RuntimeExpression(target, LookAt, LookAt.EyeDirectionApplicable);

            var instance = target.GetComponent<RuntimeGltfInstance>();
            if (instance != null)
            {
                // ランタイムインポートならここに到達してゼロコストになる
                m_defaultTransformStates = instance.InitialTransformStates;
            }
            else
            {
                // エディタでプレハブ配置してる奴ならこっちに到達して収集する
                m_defaultTransformStates = target.GetComponentsInChildren<Transform>()
                    .ToDictionary(tf => tf, tf => new TransformState(tf));
            }

            // NOTE: FastSpringBoneService は UnitTest などでは動作しない
            if (Application.isPlaying)
            {
                m_fastSpringBoneService = FastSpringBoneService.Instance;
                m_fastSpringBoneBuffer = CreateFastSpringBoneBuffer(m_target.SpringBone);
                m_fastSpringBoneService.BufferCombiner.Register(m_fastSpringBoneBuffer);
            }
        }

        public void Dispose()
        {
            ControlRig?.Dispose();
            m_fastSpringBoneService.BufferCombiner.Unregister(m_fastSpringBoneBuffer);
            m_fastSpringBoneBuffer.Dispose();
        }

        /// <summary>
        /// このVRMに紐づくSpringBone関連のバッファを再構築する
        /// ランタイム実行時にSpringBoneに対して変更を行いたいときは、このメソッドを明示的に呼ぶ必要がある
        /// </summary>
        public void ReconstructSpringBone()
        {
            m_fastSpringBoneService.BufferCombiner.Unregister(m_fastSpringBoneBuffer);

            m_fastSpringBoneBuffer.Dispose();
            m_fastSpringBoneBuffer = CreateFastSpringBoneBuffer(m_target.SpringBone);

            m_fastSpringBoneService.BufferCombiner.Register(m_fastSpringBoneBuffer);
        }

        /// <summary>
        /// SpringBoneの個数を変更せずにパラメータのみを変更したとき、その変更をAllocを行わずに適用する
        /// </summary>
        public void ApplySpringBoneJointParameterChange()
        {
            //Joint数まで検証すると正常系の計算コストとして高いため、Springsの個数だけ見るに留める
            if (m_target.SpringBone.Springs.Count != m_fastSpringBoneBuffer.Springs.Length)
            {
                throw new InvalidOperationException("Spring count is inconsistent");
            }

            m_fastSpringBoneService.BufferCombiner.Unregister(m_fastSpringBoneBuffer);
            var index = 0;
            foreach (var spring in m_target.SpringBone.Springs)
            {
                var len = spring.Joints.Count - 1;
                for (var i = 0; i < len; i++)
                {
                    var source = spring.Joints[i];
                    var dest = m_fastSpringBoneBuffer.Joints[index];
                    dest.radius = source.m_jointRadius;
                    dest.dragForce = source.m_dragForce;
                    dest.gravityDir = source.m_gravityDir;
                    dest.gravityPower = source.m_gravityPower;
                    dest.stiffnessForce = source.m_stiffnessForce;
                    var arr = m_fastSpringBoneBuffer.Joints;
                    arr[index] = dest;
                    m_fastSpringBoneBuffer.Joints = arr;
                    index++;
                }
            }
            m_fastSpringBoneService.BufferCombiner.Register(m_fastSpringBoneBuffer);
        }
        
        private FastSpringBoneBuffer CreateFastSpringBoneBuffer(Vrm10InstanceSpringBone springBone)
        {
            return new FastSpringBoneBuffer(
                springBone.Springs.Select(spring => new FastSpringBoneSpring
                {
                    center = spring.Center,
                    colliders = spring.ColliderGroups
                    .SelectMany(group => group.Colliders)
                    .Select(collider => new FastSpringBoneCollider
                    {
                        Transform = collider.transform,
                        Collider = new BlittableCollider
                        {
                            offset = collider.Offset,
                            radius = collider.Radius,
                            tail = collider.Tail,
                            colliderType = TranslateColliderType(collider.ColliderType)
                        }
                    }).ToArray(),
                    joints = spring.Joints
                    .Select(joint => new FastSpringBoneJoint
                    {
                        Transform = joint.transform,
                        Joint = new BlittableJoint
                        {
                            radius = joint.m_jointRadius,
                            dragForce = joint.m_dragForce,
                            gravityDir = joint.m_gravityDir,
                            gravityPower = joint.m_gravityPower,
                            stiffnessForce = joint.m_stiffnessForce
                        },
                        DefaultLocalRotation = GetOrAddDefaultTransformState(joint.transform).LocalRotation,
                    }).ToArray(),
                }).ToArray());
        }

        private TransformState GetOrAddDefaultTransformState(Transform tf)
        {
            if (m_defaultTransformStates.TryGetValue(tf, out var defaultTransformState))
            {
                return defaultTransformState;
            }

            Debug.LogWarning($"{tf.name} does not exist on load.");
            return new TransformState(null);
        }

        private static BlittableColliderType TranslateColliderType(VRM10SpringBoneColliderTypes colliderType)
        {
            switch (colliderType)
            {
                case VRM10SpringBoneColliderTypes.Sphere:
                    return BlittableColliderType.Sphere;
                case VRM10SpringBoneColliderTypes.Capsule:
                    return BlittableColliderType.Capsule;
                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        /// <summary>
        /// 毎フレーム関連コンポーネントを解決する
        ///
        /// * Contraint
        /// * Spring
        /// * LookAt
        /// * Expression
        ///
        /// </summary>
        public void Process()
        {
            // 1. Control Rig
            ControlRig?.Process();

            // 2. Constraints
            foreach (var constraint in Constraints)
            {
                constraint.Process();
            }

            // 3. Gaze control
            LookAt.Process(m_target.LookAtTargetType, m_target.Gaze);

            // 4. Expression
            Expression.Process();
        }
    }
}