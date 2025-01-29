# Brioche Motion  

- [x] Motion Capture  
    - [x] Video Capture (Backend [OpenCV](https://github.com/HanetakaChou/OpenCV))  
    - [x] Motion Detector (Backend [MeidaPipe](https://github.com/HanetakaChou/mediapipe))  
- [ ] Animation  
    - [ ] IK (Inverse Kinematics)  
        - [ ] Reaching IK (Target Position)  
            - [x] Two Joints IK  
            - [ ] ~~Three Joints IK~~  
            - [x] CCD (Cyclic Coordinate Descent) IK  
            - [ ] ~~FABRIK (Forward And Backward Reaching Inverse Kinematics)~~  
            - [ ] ~~Biped Foot IK~~  
            - [ ] ~~Quadruped Foot IK~~  
        - [ ] Aim IK (Target Rotation)  
            - [ ] ~~Look At IK~~  
    - [ ] Skeleton Mapper  
        - [x] Ragdoll Mapping   
        - [ ] ~~Animation Retargeting Mapping~~  
    - [ ] Ragdoll  
        - [ ] Ragdoll Kinematics Controller  
        - [ ] ~~Ragdoll Motors Controller~~  

## Morph Target Names  

| [ARKit](https://developer.apple.com/documentation/arkit/arfaceanchor/blendshapelocation) / [MediaPipe](https://ai.google.dev/edge/mediapipe/solutions/vision/face_landmarker) | [FACS AU](https://www.cs.cmu.edu/~face/facs.htm) | [MMD](https://www.deviantart.com/xoriu/art/MMD-Facial-Expressions-Chart-341504917) | [VRM 0.0](https://github.com/vrm-c/vrm-specification/blob/master/specification/0.0/README.md#predefined-expression-name) | [VRM 1.0](https://github.com/vrm-c/vrm-specification/blob/master/specification/VRMC_vrm-1.0/expressions.md#preset-expressions) |  
| :-: | :-: | :-: | :-: | :-: |   
| BrowDownLeft | AU 4 | Brow Angry (怒り) + Eye Angry (ｷﾘｯ) + Mouth Angry (∧) | Angry | anger |  
| BrowDownRight | AU 4 | Brow Angry (怒り) + Eye Angry (ｷﾘｯ) + Mouth Angry (∧) | Angry | anger |  
| BrowInnerUp | AU 1 | N/A | N/A | N/A |   
| BrowOuterUpLeft | AU 2 | N/A | N/A | N/A | 
| BrowOuterUpRight | AU 2 | N/A | N/A | N/A |  
| CheekPuff | AU 13 | N/A | N/A | N/A |  
| CheekSquintLeft | AU 6 | Brow Happy (にこり) + Eye Happy (笑い) + Mouth Happy (にっこり) | Joy | happy |  
| CheekSquintRight | AU 6 | Brow Happy (にこり) + Eye Happy (笑い) + Mouth Happy (にっこり) | Joy | happy |  
| EyeBlinkLeft | AU 45 | Eye Blink L (ウィンク２) | Blink_L | blinkLeft |    
| EyeBlinkRight | AU 45 | Eye Blink R (ｳｨﾝｸ２右) | Blink_R | blinkRight |  
| EyeLookDownLeft | AU 64 | N/A | LookDown | lookDown |  
| EyeLookDownRight | AU 64 | N/A | LookDown | lookDown |  
| EyeLookInLeft | AU 62 | N/A | LookRight | lookRight |  
| EyeLookInRight | AU 61 | N/A | LookLeft | lookLeft |  
| EyeLookOutLeft | AU 61 | N/A | LookLeft | lookLeft |  
| EyeLookOutRight | AU 62 | N/A | LookRight | lookRight |  
| EyeLookUpLeft | AU 63 | N/A | LookUp | lookUp |  
| EyeLookUpRight | AU 63 | N/A | LookUp | lookUp |  
| EyeSquintLeft | AU 6 | Brow Happy (にこり) + Eye Happy (笑い) + Mouth Happy (にっこり) | Joy | happy | happy |  
| EyeSquintRight | AU 6 | Brow Happy (にこり) + Eye Happy (笑い) + Mouth Happy (にっこり) | Joy | happy | happy |  
| EyeWideLeft | AU 5 | Brow Surprised (上) + Eye Surprised (びっくり) + Mouth Surprised (▲) | N/A | surprised |  
| EyeWideRight | AU 5 | Brow Surprised (上) + Eye Surprised (びっくり) + Mouth Surprised (▲) | N/A | surprised |  
| JawForward | AU 29 | N/A | N/A | N/A |  
| JawLeft | N/A | N/A | N/A | N/A |  
| JawOpen | AU 26 | Mouth A (あ) | A | aa |   
| JawRight | N/A | N/A | N/A | N/A  |  
| MouthClose | AU 23 | Mouth N (ん) | N/A | N/A | 
| MouthDimpleLeft | AU 14 | N/A | N/A  | N/A  |  
| MouthDimpleRight | AU 14 | N/A | N/A  | N/A  |  
| MouthFrownLeft | AU 15 | Brow Sad (困る) + Eye Sad (じと目) + Mouth Sad (口角下げ) | Sorrow | sad |  
| MouthFrownRight | AU 15 | Brow Sad (困る) + Eye Sad (じと目) + Mouth Sad (口角下げ) | Sorrow | sad |  
| MouthFunnel | AU 22 | Mouth O (お) | O | oh |  
| MouthLeft | N/A | N/A | N/A | N/A |  
| MouthLowerDownLeft | AU 16 | N/A | N/A | N/A |  
| MouthLowerDownRight | AU 16 | N/A | N/A | N/A |  
| MouthPressLeft | AU 24 | N/A | N/A| N/A |
| MouthPressRight | AU 24 | N/A | N/A | N/A |
| MouthPucker | AU 18 | Mouth U (う) | U | ou |  
| MouthRight | N/A | N/A | N/A | N/A |  
| MouthRollLower | AU 28 | N/A | N/A | N/A |  
| MouthRollUpper | AU 28 | N/A | N/A | N/A |
| MouthShrugLower | AU 17 | N/A | N/A | N/A |  
| MouthShrugUpper | AU 17 | N/A | N/A | N/A |  
| MouthSmileLeft | AU 12 | Brow Happy (にこり) + Eye Happy (笑い) + Mouth Happy (にっこり) | Joy | happy |  
| MouthSmileRight | AU 12 | Brow Happy (にこり) + Eye Happy (笑い) + Mouth Happy (にっこり) | Joy | happy |  
| MouthStretchLeft | AU 20 | Mouth I (い) | I | ih |  
| MouthStretchRight | AU 20 | Mouth I (い) | I | ih  |  
| MouthUpperUpLeft | AU 10 | Mouth E (え) | E | ee | 
| MouthUpperUpRight | AU 10 | Mouth E (え) | E | ee |  
| NoseSneerLeft | AU 9 | Brow Angry (怒り) + Eye Angry (ｷﾘｯ) + Mouth Angry (∧) | Angry | Angry | anger |  
| NoseSneerRight | AU 9 | Brow Angry (怒り) + Eye Angry (ｷﾘｯ) + Mouth Angry (∧) | Angry | Angry | anger |  

## Skeleton Joint Names  

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