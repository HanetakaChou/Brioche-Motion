# Brioche Motion  

- [x] Motion Capture
    - [x] Video Capture  
    - [x] Motion Detector  
- [ ] Animation  
    - [ ] Animation Retargeting      
    - [ ] Ragdoll Physics  
    - [ ] IK (Inverse Kinematics)  
        - [ ] Reaching IK  
            - [ ] Two Joints IK  
            - [ ] Three Joints IK  
            - [ ] CCD (Cyclic Coordinate Descent) IK  
            - [ ] FABRIK (Forward And Backward Reaching Inverse Kinematics)  
        - [ ] Foot IK  
        - [ ] Look At IK  


## Animation Retargeting  
  
### Morph Target Names  

| [PMX](https://images-wixmp-ed30a86b8c4ca887773594c2.wixmp.com/i/0b7b5e4b-c62e-41f7-8ced-1f3e58c4f5bf/d5nbmvp-5779f5ac-d476-426c-8ee6-2111eff8e76c.png) | [VRM 0.0](https://github.com/vrm-c/vrm-specification/blob/master/specification/0.0/README.md#predefined-expression-name) | [VRM 1.0](https://github.com/vrm-c/vrm-specification/blob/master/specification/VRMC_vrm-1.0/expressions.md#preset-expressions) | [ARKit](https://developer.apple.com/documentation/arkit/arfaceanchor/blendshapelocation) / [MediaPipe](https://ai.google.dev/edge/mediapipe/solutions/vision/face_landmarker) |  
| :-: | :-: | :-: | :-: |   
| N/A | Neutral | neutral | $$\displaystyle \begin{vmatrix} \text{\_neutral} \end{vmatrix} \begin{vmatrix} 1.0 \end{vmatrix}$$ |    
| あ | A | aa | $$\displaystyle \begin{vmatrix} \text{jawOpen} \\ \text{mouthFunnel} \end{vmatrix} \begin{vmatrix} 0.8 \\ 0.2 \end{vmatrix}$$ |   
| い | I | ih | $$\displaystyle \begin{vmatrix} \text{mouthStretchLeft} \\ \text{mouthStretchRight} \end{vmatrix} \begin{vmatrix} 0.5 \\ 0.5 \end{vmatrix}$$ |  
| う | U | ou | $$\displaystyle \begin{vmatrix} \text{mouthPucker} \end{vmatrix} \begin{vmatrix} 1.0 \end{vmatrix}$$ |  
| え | E | ee | $$\displaystyle \begin{vmatrix} \text{mouthRollLower} \\ \text{mouthRollUpper} \end{vmatrix} \begin{vmatrix} 0.5 \\ 0.5 \end{vmatrix}$$ |  
| お | O | oh | $$\displaystyle \begin{vmatrix} \text{mouthFunnel} \\ \text{mouthPucker} \end{vmatrix} \begin{vmatrix} 0.8 \\ 0.2 \end{vmatrix}$$ |  
| まばたき | Blink | blink | $$\displaystyle \begin{vmatrix} \text{eyeBlinkLeft} \\ \text{eyeBlinkRight} \end{vmatrix} \begin{vmatrix} 0.5 \\ 0.5 \end{vmatrix}$$ |  
| ウィンク | Blink_L | blinkLeft | $$\displaystyle \begin{vmatrix} \text{eyeBlinkLeft} \end{vmatrix} \begin{vmatrix} 1.0 \end{vmatrix}$$ |   
| ウィンク右 | Blink_R | blinkRight | $$\displaystyle \begin{vmatrix} \text{eyeBlinkRight} \end{vmatrix} \begin{vmatrix} 1.0 \end{vmatrix}$$ |  
| 笑い + にっこり + にこり | Fun | relaxed | $$\displaystyle \begin{vmatrix} \text{cheekSquintLeft} \\ \text{cheekSquintRight} \\ \text{mouthDimpleLeft} \\ \text{mouthDimpleRight} \\ \text{mouthSmileLeft} \\ \text{mouthSmileRight} \end{vmatrix} \begin{vmatrix} 0.2 \\ 0.2 \\ 0.3 \\ 0.3 \\ 0.6 \\ 0.6 \end{vmatrix}$$ |    
| 怒り | Angry | anger | $$\displaystyle \begin{vmatrix} \text{browDownLeft} \\ \text{browDownRight} \\ \text{noseSneerLeft} \\ \text{noseSneerRight} \\ \text{mouthPressLeft} \\ \text{mouthPressRight} \end{vmatrix} \begin{vmatrix} 0.7 \\ 0.7 \\ 0.3 \\ 0.3 \\ 0.5 \\ 0.5 \end{vmatrix}$$ |    
| 悲しむ + 困る | Sorrow | sad | $$\displaystyle \begin{vmatrix} \text{browInnerUp} \\ \text{cheekSquintLeft} \\ \text{cheekSquintRight} \\ \text{mouthFrownLeft} \\ \text{mouthFrownRight} \end{vmatrix} \begin{vmatrix} 0.8 \\ 0.3 \\ 0.3 \\ 0.8 \\ 0.8 \end{vmatrix}$$ |  
| 喜び | Joy | happy | $$\displaystyle \begin{vmatrix} \text{cheekPuff} \\ \text{cheekSquintLeft} \\ \text{eyeSquintRight} \\ \text{mouthSmileLeft} \\ \text{mouthSmileRight} \\ \text{mouthStretchLeft} \\ \text{mouthStretchRight} \end{vmatrix} \begin{vmatrix} 0.4 \\ 0.4 \\ 0.4 \\ 0.7 \\ 0.7 \\ 0.5 \\ 0.5 \end{vmatrix}$$ |  
| 真面目 | N/A | surprised | $$\displaystyle \begin{vmatrix} \text{eyeWideLeft} \\ \text{eyeWideRight} \\ \text{browOuterUpLeft} \\ \text{browOuterUpRight} \\ \text{jawOpen} \end{vmatrix} \begin{vmatrix} 0.6 \\ 0.6 \\ 0.5 \\ 0.5 \\ 0.8 \end{vmatrix}$$ |  
| 上 | LookUp | lookUp | $$\displaystyle \begin{vmatrix} \text{eyeLookUpLeft} \\ \text{eyeLookUpRight} \end{vmatrix} \begin{vmatrix} 0.75 \\ 0.75 \end{vmatrix}$$ |  
| 下 | LookDown | lookDown | $$\displaystyle \begin{vmatrix} \text{eyeLookDownLeft} \\ \text{eyeLookDownRight} \end{vmatrix} \begin{vmatrix} 0.75 \\ 0.75 \end{vmatrix}$$ |  
| N/A | LookLeft | lookLeft | $$\displaystyle \begin{vmatrix} \text{eyeLookOutLeft} \\ \text{eyeSquintLeft} \end{vmatrix} \begin{vmatrix} 1.0 \\ 0.35 \end{vmatrix}$$ |  
| N/A | LookRight | lookRight | $$\displaystyle \begin{vmatrix} \text{eyeLookOutRight} \\ \text{eyeSquintRight} \end{vmatrix} \begin{vmatrix} 1.0 \\ 0.35 \end{vmatrix}$$ |   

### Joint Names  

[Blender MMD Tools: Internal Dictionary](https://github.com/UuuNyaa/blender_mmd_tools/blob/main/mmd_tools/translations.py)  

[VRM 0.0: Defined Bones](https://github.com/vrm-c/vrm-specification/blob/master/specification/0.0/README.md#defined-bones)  

[VRM 1.0: List of Humanoid Bones](https://github.com/vrm-c/vrm-specification/blob/master/specification/VRMC_vrm-1.0/humanoid.md#list-of-humanoid-bones)  

[VRM Addon for Blender: Human Bone Mapper MMD Mapping](https://github.com/saturday06/VRM-Addon-for-Blender/blob/main/src/io_scene_vrm/common/human_bone_mapper/mmd_mapping.py)  

[VRM 1.0: About Pose Data Compatibility](https://github.com/vrm-c/vrm-specification/blob/master/specification/VRMC_vrm_animation-1.0/how_to_transform_human_pose.md)  

[VRM 1.0: BVH to VRMA](https://github.com/vrm-c/bvh2vrma/blob/main/src/lib/bvh-converter/convertBVHToVRMAnimation.ts)  

[Unity3D: Human Body Bones](https://docs.unity3d.com/ScriptReference/HumanBodyBones.html)  

[ARKit: Validating a Model for Motion Capture](https://developer.apple.com/documentation/arkit/validating-a-model-for-motion-capture)  

[ARKit: ARSkeleton JointName](https://developer.apple.com/documentation/arkit/arskeleton/jointname)  

[MediaPipe: Face LandMarker facial transformation matrix](https://ai.google.dev/edge/mediapipe/solutions/vision/face_landmarker)  

[MediaPipe: Pose LandMarker pose world landmark](https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker)  

[JoltPhycis: SkeletonMapper](https://github.com/jrouwe/JoltPhysics/blob/master/Samples/Tests/Rig/SkeletonMapperTest.cpp)  

| PMX | VRM | VRM Parent | ARKit | MediaPipe |   
| :-: | :-: | :-: | :-: | :-: |   
| センター | hips | N/A | root | $$\displaystyle \begin{vmatrix} \text{leftHip} \\ \text{rightHip} \end{vmatrix} \begin{vmatrix} 0.5 \\ 0.5 \end{vmatrix}$$ |  
| 上半身 | spine | hips | N/A | N/A |  
| 上半身2 | chest | spine | N/A | N/A |  
| N/A | upperChest | chest | N/A | N/A |  
| 首 | neck | upperChest | N/A | Neck to Head Transform by Face Landmarker |  
| 頭 | head | neck | head | Neck to Head Transform by Face Landmarker |  
| 左目 | leftEye | head | N/A | $$\displaystyle \begin{vmatrix} \text{leftEye} \end{vmatrix} \begin{vmatrix} 1.0 \end{vmatrix}$$ |  
| 右目 | rightEye | head | N/A | $$\displaystyle \begin{vmatrix} \text{rightEye} \end{vmatrix} \begin{vmatrix} 1.0 \end{vmatrix}$$ |   
| N/A | jaw | head | N/A | $$\displaystyle \begin{vmatrix} \text{leftMouth} \\ \text{rightMouth} \end{vmatrix} \begin{vmatrix} 0.5 \\ 0.5 \end{vmatrix}$$ |  
| 左肩 | leftShoulder | upperChest | leftShoulder | $$\displaystyle \begin{vmatrix} \text{leftShoulder} \end{vmatrix} \begin{vmatrix} 1.0 \end{vmatrix}$$ |     
| 左腕 | leftUpperArm | leftShoulder | N/A | N/A |  
| 左ひじ | leftLowerArm | leftUpperArm | N/A | $$\displaystyle \begin{vmatrix} \text{leftElbow} \end{vmatrix} \begin{vmatrix} 1.0 \end{vmatrix}$$ |  
| 左手首 | leftHand | leftLowerArm | leftHand | $$\displaystyle \begin{vmatrix} \text{leftWrist} \end{vmatrix} \begin{vmatrix} 1.0 \end{vmatrix}$$ |  
| 右肩 | rightShoulder | upperChest | rightShoulder | $$\displaystyle \begin{vmatrix} \text{rightShoulder} \end{vmatrix} \begin{vmatrix} 1.0 \end{vmatrix}$$ |     
| 右腕 | rightUpperArm | rightShoulder | N/A | N/A |  
| 右ひじ | rightLowerArm | rightUpperArm | N/A | $$\displaystyle \begin{vmatrix} \text{rightElbow} \end{vmatrix} \begin{vmatrix} 1.0 \end{vmatrix}$$ |  
| 右手首 | rightHand | rightLowerArm | rightHand | $$\displaystyle \begin{vmatrix} \text{rightWrist} \end{vmatrix} \begin{vmatrix} 1.0 \end{vmatrix}$$ |  