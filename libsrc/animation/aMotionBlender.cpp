#include "aMotionBlender.h"
#include <algorithm>
using namespace std;

MotionBlender::MotionBlender()
{

}

MotionBlender::~MotionBlender()
{

}

void MotionBlender::loadMotion1(const std::string& m1)
{
    mMotion1.load(m1);
    mBlend.load(m1);
}

void MotionBlender::loadMotion2(const std::string& m2)
{
    mMotion2.load(m2);
}

void MotionBlender::append(BVHController& input, int startKeyId, int endKeyId, BVHController& output)
{
    // Appends frames from input into output
    // startKeyId is the starting frame from motion 1 to put in output
    // endKeyId is the ending frame from motion 1 to put in output
    // Append the keys to output based on the motion curves in input
    for(int i = startKeyId; i < endKeyId; i++) {
        Actor tmp = input.getKey(i);
        output.appendKey(i, tmp);
    }
}

void MotionBlender::align(BVHController& motion1, BVHController& motion2, 
    int startKeyId, int endKeyId, int numBlendFrames)
{   
    // Align motion 1 and motion 2
    // startKeyId is the starting frame from motion 1
    // endKeyId is the target frame from motion 2
    // numBlendFrames is the number of frames over which to produce the blend
    // Edit the root transformation curves in motion 2 to start at the same position as motion 1 for blending

    Quaternion m1 = motion1.getJointCurve(0).getKey(startKeyId);
    Quaternion m2 = motion2.getJointCurve(0).getKey(endKeyId);

    // Rotations
    mat3 start_rot, end_rot;
    start_rot = m1.ToRotation();
    end_rot = m2.ToRotation();

    vec3 angle1, angle2;

    start_rot.ToEulerAnglesYXZ(angle1);
    end_rot.ToEulerAnglesYXZ(angle2);
    float theta = angle1[1] - angle2[1];
    
    Quaternion new_rot;
    new_rot.FromAxisAngle(vec3(0,1,0), theta);
    mat3 new_rot_copy = new_rot.ToRotation();
    // Translation
    const vec3& diff = motion1.getRootCurve().getKey(startKeyId) - new_rot_copy*motion2.getRootCurve().
    getKey(endKeyId);


    for (int i = endKeyId; i <= endKeyId + numBlendFrames; i++) {
        // Align Translation

        const vec3& existing = motion2.getKey(i).getRoot()->getLocalTranslation();
        motion2.getRootCurve().editKey(i, new_rot_copy*existing + diff);

        // Align Rotation

        Quaternion q = new_rot * motion2.getJointCurve(0).getKey(i);
        q.Normalize();
        cout << "-------------" << endl;
        cout << "ORIGINAL: "<< motion2.getJointCurve(0).getKey(i) << endl;
        motion2.getJointCurve(0).editKey(i, q);
        cout << "NEW (SHOULD ONLY MOVE in y): " << q << endl;
        cout << "HEADER: " << new_rot << endl;
        cout << "-------------" << endl;
    }
}

void MotionBlender::crossfade(BVHController& motion1, BVHController& motion2,
    int startKeyId, int endKeyId, int numBlendFrames, BVHController& blend)
{
    // Crossfade from motion 1 to motion 2
    // startKeyId is the starting frame from motion 1
    // endKeyId is the target frame from motion 2
    // numBlendFrames is the number of frames over which to produce the blend
    // Append the blended frames to "blend"
    for (int i = 0; i < numBlendFrames; i++) {
        double alpha = (i*1.0)/(numBlendFrames-1);

        // LERP THE ROOT
        vec3 new_translation = (1-alpha) * motion1.getRootCurve().getKey(startKeyId+i) + 
            alpha * motion2.getRootCurve().getKey(endKeyId+i);
        blend.getRootCurve().appendKey(new_translation);

        // SLERP EVERYTHING ELSE
        for(int j = 0; j < motion1.getNumJoints(); j++) {
            Quaternion new_joint_quat = Quaternion::Slerp(motion1.getJointCurve(j).getKey(startKeyId + i), 
                motion2.getJointCurve(j).getKey(endKeyId + i), alpha);
            blend.getJointCurve(j).appendKey(new_joint_quat);
        }
    }
}

void MotionBlender::blend(int startKeyId, int endKeyId, int numBlendFrames)
{
    if (startKeyId >= mMotion1.getNumKeys())
    {
        std::cout << "Invalid start key for motion 1" << startKeyId << std::endl;
        return;
    }

    if (endKeyId >= mMotion2.getNumKeys())
    {
        std::cout << "Invalid end key for motion 2" << startKeyId << std::endl;
        return;
    }

    if (startKeyId + numBlendFrames > mMotion1.getNumKeys())
    {
        std::cout << "Not enough frames for blending\n";
        numBlendFrames = mMotion1.getNumKeys() - startKeyId;
    }

    if (endKeyId + numBlendFrames > mMotion2.getNumKeys())
    {
        std::cout << "Not enough frames for blending\n";
        numBlendFrames = mMotion2.getNumKeys() - endKeyId;
    }

    if (numBlendFrames <= 0) return;
    mBlend.clear();

    // Add the beginning of the motion1 into mBlend
    append(mMotion1, 0, startKeyId, mBlend);

    // Align motion2 with motion1 and output the crossfade over numBlendFrames
    // Append the result into mBlend
    align(mMotion1, mMotion2, startKeyId, endKeyId, getNumKeys2()-(endKeyId+1));

    crossfade(mMotion1, mMotion2, startKeyId, endKeyId, numBlendFrames, mBlend);

    // Align the remainder of motion 2 with the end of the crossfade
    //align(mBlend, mMotion2, startKeyId + numBlendFrames - 1, endKeyId + numBlendFrames, getNumKeys2()-(endKeyId + numBlendFrames));

    // Append the remaining frames of motion 2 into mBlend
    append(mMotion2, endKeyId+numBlendFrames, getNumKeys2(), mBlend);
}

void MotionBlender::update(double dt)
{
    mBlend.update(dt);
}
