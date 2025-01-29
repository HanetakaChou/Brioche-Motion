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
| browDownLeft | AU 4 | Brow Angry (怒り) + Eye Angry (ｷﾘｯ) + Mouth Angry (∧) | Angry | anger |  
| browDownRight | AU 4 | Brow Angry (怒り) + Eye Angry (ｷﾘｯ) + Mouth Angry (∧) | Angry | anger |  
| browInnerUp | AU 1 | N/A | N/A | N/A |   
| browOuterUpLeft | AU 2 | N/A | N/A | N/A | 
| browOuterUpRight | AU 2 | N/A | N/A | N/A |  
| cheekPuff | AU 13 | N/A | N/A | N/A |  
| cheekSquintLeft | AU 6 | Brow Happy (にこり) + Eye Happy (笑い) + Mouth Happy (にっこり) | Joy | happy |  
| cheekSquintRight | AU 6 | Brow Happy (にこり) + Eye Happy (笑い) + Mouth Happy (にっこり) | Joy | happy |  
| eyeBlinkLeft | AU 45 | Eye Blink L (ウィンク２) | Blink_L | blinkLeft |    
| eyeBlinkRight | AU 45 | Eye Blink R (ｳｨﾝｸ２右) | Blink_R | blinkRight |  
| eyeSquintLeft | AU 6 | Brow Happy (にこり) + Eye Happy (笑い) + Mouth Happy (にっこり) | Joy | happy | happy |  
| eyeSquintRight | AU 6 | Brow Happy (にこり) + Eye Happy (笑い) + Mouth Happy (にっこり) | Joy | happy | happy |  
| eyeWideLeft | AU 5 | Brow Surprised (上) + Eye Surprised (びっくり) + Mouth Surprised (▲) | N/A | surprised |  
| eyeWideRight | AU 5 | Brow Surprised (上) + Eye Surprised (びっくり) + Mouth Surprised (▲) | N/A | surprised |  
| jawForward | AU 29 | N/A | N/A | N/A |  
| jawLeft | N/A | N/A | N/A | N/A |  
| jawOpen | AU 26 | Mouth A (あ) | A | aa |   
| jawRight | N/A | N/A | N/A | N/A  |  
| mouthClose | AU 23 | Mouth N (ん) | N/A | N/A | 
| mouthDimpleLeft | AU 14 | N/A | N/A  | N/A  |  
| mouthDimpleRight | AU 14 | N/A | N/A  | N/A  |  
| mouthFrownLeft | AU 15 | Brow Sad (困る) + Eye Sad (じと目) + Mouth Sad (口角下げ) | Sorrow | sad |  
| mouthFrownRight | AU 15 | Brow Sad (困る) + Eye Sad (じと目) + Mouth Sad (口角下げ) | Sorrow | sad |  
| mouthFunnel | AU 22 | Mouth O (お) | O | oh |  
| mouthLeft | N/A | N/A | N/A | N/A |  
| mouthLowerDownLeft | AU 16 | N/A | N/A | N/A |  
| mouthLowerDownRight | AU 16 | N/A | N/A | N/A |  
| mouthPressLeft | AU 24 | N/A | N/A| N/A |
| mouthPressRight | AU 24 | N/A | N/A | N/A |
| mouthPucker | AU 18 | Mouth U (う) | U | ou |  
| mouthRight | N/A | N/A | N/A | N/A |  
| mouthRollLower | AU 28 | N/A | N/A | N/A |  
| mouthRollUpper | AU 28 | N/A | N/A | N/A |
| mouthShrugLower | AU 17 | N/A | N/A | N/A |  
| mouthShrugUpper | AU 17 | N/A | N/A | N/A |  
| mouthSmileLeft | AU 12 | Brow Happy (にこり) + Eye Happy (笑い) + Mouth Happy (にっこり) | Joy | happy |  
| mouthSmileRight | AU 12 | Brow Happy (にこり) + Eye Happy (笑い) + Mouth Happy (にっこり) | Joy | happy |  
| mouthStretchLeft | AU 20 | Mouth I (い) | I | ih |  
| mouthStretchRight | AU 20 | Mouth I (い) | I | ih  |  
| mouthUpperUpLeft | AU 10 | Mouth E (え) | E | ee | 
| mouthUpperUpRight | AU 10 | Mouth E (え) | E | ee |  
| noseSneerLeft | AU 9 | Brow Angry (怒り) + Eye Angry (ｷﾘｯ) + Mouth Angry (∧) | Angry | Angry | anger |  
| noseSneerRight | AU 9 | Brow Angry (怒り) + Eye Angry (ｷﾘｯ) + Mouth Angry (∧) | Angry | Angry | anger |  

## Face Skeleton Joint Names  

| [ARKit](https://developer.apple.com/documentation/arkit/arskeleton/jointname) | [MediaPipe](https://ai.google.dev/edge/mediapipe/solutions/vision/face_landmarker) | [MMD](https://github.com/saturday06/VRM-Addon-for-Blender/blob/main/src/io_scene_vrm/common/human_bone_mapper/mmd_mapping.py) | [VRM 0.0](https://github.com/vrm-c/vrm-specification/blob/master/specification/0.0/README.md#defined-bones) | [VRM 1.0](https://github.com/vrm-c/vrm-specification/blob/master/specification/VRMC_vrm-1.0/humanoid.md#list-of-humanoid-bones) |  
| :-: | :-: | :-: | :-: | :-: |   
| head | output_facial_transformation_matrixes | Head (頭) | head | head |  

| [ARKit](https://developer.apple.com/documentation/arkit/arfaceanchor/blendshapelocation) / [MediaPipe](https://ai.google.dev/edge/mediapipe/solutions/vision/face_landmarker) | [FACS AU](https://www.cs.cmu.edu/~face/facs.htm) | [MMD](https://github.com/saturday06/VRM-Addon-for-Blender/blob/main/src/io_scene_vrm/common/human_bone_mapper/mmd_mapping.py) | [VRM 0.0](https://github.com/vrm-c/vrm-specification/blob/master/specification/0.0/README.md#defined-bones) | [VRM 1.0](https://github.com/vrm-c/vrm-specification/blob/master/specification/VRMC_vrm-1.0/humanoid.md#list-of-humanoid-bones) |  
| :-: | :-: | :-: | :-: | :-: |   
| eyeLookDownLeft | AU 64 (Look Down) | Right Eye (右目) + Left Eye (左目) | rightEye + leftEye | rightEye + leftEye |  
| eyeLookDownRight | AU 64 (Look Down) | Right Eye (右目) + Left Eye (左目) | rightEye + leftEye | rightEye + leftEye |  
| eyeLookInLeft | AU 62 (Look Right) | Left Eye (左目) | leftEye | leftEye |  
| eyeLookInRight | AU 61 (Look Left) | Right Eye (右目) | rightEye | rightEye |  
| eyeLookOutLeft | AU 61 (Look Left) | Left Eye (左目) | leftEye | leftEye |  
| eyeLookOutRight | AU 62 (Look Right) | Right Eye (右目) | rightEye | rightEye |  
| eyeLookUpLeft | AU 63 (Look Up) | Right Eye (右目) + Left Eye (左目) | rightEye + leftEye | rightEye + leftEye |  
| eyeLookUpRight | AU 63 (Look Up) | Right Eye (右目) + Left Eye (左目) | rightEye + leftEye | rightEye + leftEye |  

## Pose Skeleton Joint Names  

| [ARKit](https://developer.apple.com/documentation/arkit/arskeleton/jointname) | [MediaPipe](https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker) | [MMD](https://github.com/saturday06/VRM-Addon-for-Blender/blob/main/src/io_scene_vrm/common/human_bone_mapper/mmd_mapping.py) | [VRM 0.0](https://github.com/vrm-c/vrm-specification/blob/master/specification/0.0/README.md#defined-bones) | [VRM 1.0](https://github.com/vrm-c/vrm-specification/blob/master/specification/VRMC_vrm-1.0/humanoid.md#list-of-humanoid-bones) |  
| :-: | :-: | :-: | :-: | :-: |   
| rightShoulder | right shoulder | Right Arm (右腕) | rightUpperArm | rightUpperArm |  
| N/A | right elbow | Right Elbow (右ひじ) | rightLowerArm | rightLowerArm |  
| rightHand | right wrist | Right Wrist (右手首) | rightHand | rightHand |  
| N/A | right index | Right Index Finger 1 (右人指１) | rightIndexProximal | rightIndexProximal |  
| N/A | right hip | Right Leg (右足) | rightUpperLeg | rightUpperLeg |    
| N/A | right knee | Right Knee (右ひざ) | rightLowerLeg | rightLowerLeg |   
| rightFoot | right ankle | Right Ankle (右足首) | rightFoot | rightFoot |   
| N/A | right foot index | Right Toe Tip (右つま先) | rightToes | rightToes |  
| leftShoulder | left shoulder | Left Arm (左腕) | leftUpperArm | leftUpperArm |  
| N/A | left elbow | Left Elbow (左ひじ) | leftLowerArm | leftLowerArm |  
| leftHand | left wrist | Left Wrist (左手首) | leftHand | leftHand |  
| N/A | left index | Left Index Finger 1 (左人指１) | leftIndexProximal | leftIndexProximal |  
| N/A | left hip | Left Leg (左足) | leftUpperLeg | leftUpperLeg |    
| N/A | left knee | Left Knee (左ひざ) | leftLowerLeg | leftLowerLeg |   
| leftFoot | left ankle | Left Ankle (左足首) | leftFoot | leftFoot |   
| N/A | left foot index | Left Toe Tip (左つま先) | leftToes | leftToes |  
