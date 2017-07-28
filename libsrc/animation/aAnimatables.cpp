#include "aAnimatables.h"
#include <cstring>

AnimatableTransform::AnimatableTransform() : 
    mId(-1), 
    mName(""), 
    mChannelCount(0), 
    mRotOrder("xyz"), 
    mDirty(false),
    mParent(0),
    mChildren(),
    mLocal2Parent(),
    mLocal2Global()
{

}

AnimatableTransform::AnimatableTransform(const std::string& name) :
    mId(-1),
    mName(name),
    mChannelCount(0),
    mRotOrder("xyz"),
    mDirty(false),
    mParent(0),
    mChildren(),
    mLocal2Parent(),
    mLocal2Global()
{

}

AnimatableTransform::AnimatableTransform(const AnimatableTransform& joint)
{
    *this = joint;
}

AnimatableTransform& AnimatableTransform::operator=(const AnimatableTransform& orig)
{
    if (&orig == this)
    {
        return *this;
    }

    // copy everything except parents/children
    mParent = 0;
    mChildren.clear();
    mDirty = true;

    mId = orig.mId;
    mName = orig.mName;
    mChannelCount = orig.mChannelCount;
    mRotOrder = orig.mRotOrder;
    mLocal2Parent = orig.mLocal2Parent;
    mLocal2Global = orig.mLocal2Global;

    return *this;
}

AnimatableTransform::~AnimatableTransform()
{

}

AnimatableTransform* AnimatableTransform::getParent()
{
    return mParent;
}
void AnimatableTransform::setParent(AnimatableTransform* parent)
{
    mParent = parent;
}

int AnimatableTransform::getNumChildren() const
{
    return mChildren.size();
}
AnimatableTransform* AnimatableTransform::getChildAt(int index)
{
    assert(index >= 0 && index < (int) mChildren.size());
    return mChildren[index];
}
void AnimatableTransform::appendChild(AnimatableTransform* child)
{
    mChildren.push_back(child);
}

void AnimatableTransform::setName(const std::string& name)
{
    mName = name;
}
void AnimatableTransform::setID(int id)
{
    mId = id;
    if (strncmp("Site", mName.c_str(), 4) == 0)
    {
        char dummy[32];
        sprintf(dummy, "Site%d", mId);
        mName = dummy;
    }
}
void AnimatableTransform::setNumChannels(int count)
{
    mChannelCount = count;
}
void AnimatableTransform::setRotationOrder(const std::string& _order)
{
    std::string order = _order;

    if (order.find("Zrotation Xrotation Yrotation") != std::string::npos) mRotOrder = "zxy";
    else if (order.find("Zrotation Yrotation Xrotation") != std::string::npos) mRotOrder = "zyx";
    else if (order.find("Xrotation Yrotation Zrotation") != std::string::npos) mRotOrder = "xyz";
    else if (order.find("Xrotation Zrotation Yrotation") != std::string::npos) mRotOrder = "xzy";
    else if (order.find("Yrotation Xrotation Zrotation") != std::string::npos) mRotOrder = "yxz";
    else if (order.find("Yrotation Zrotation Xrotation") != std::string::npos) mRotOrder = "yzx";
    else mRotOrder = order;

}

void AnimatableTransform::setLocal2Parent(const Transform& transform)
{
    mLocal2Parent = transform;
}
void AnimatableTransform::setLocalTranslation(const vec3& translation)
{
    mLocal2Parent.m_translation = translation;
}
void AnimatableTransform::setLocalRotation(const mat3& rotation)
{
    mLocal2Parent.m_rotation = rotation;
}

int AnimatableTransform::getID() const
{
    return mId;
}
const std::string& AnimatableTransform::getName() const
{
    return mName;
}
int AnimatableTransform::getNumChannels() const
{
    return mChannelCount;
}
const std::string& AnimatableTransform::getRotationOrder() const
{
    return mRotOrder;
}

const Transform& AnimatableTransform::getLocal2Parent() const
{
    return mLocal2Parent;
}
const vec3& AnimatableTransform::getLocalTranslation() const
{
    return mLocal2Parent.m_translation;
}
const mat3& AnimatableTransform::getLocalRotation() const
{
    return mLocal2Parent.m_rotation;
}

const Transform& AnimatableTransform::getLocal2Global() const
{
    return mLocal2Global;
}
const vec3& AnimatableTransform::getGlobalTranslation() const
{
    return mLocal2Global.m_translation;
}
const mat3& AnimatableTransform::getGlobalRotation() const
{
    return mLocal2Global.m_rotation;
}

void AnimatableTransform::updateTransformation()
{
    if (mParent) {
        Transform parent = mParent->mLocal2Global;
        mLocal2Global = parent*mLocal2Parent;
    } else {
        mLocal2Global = mLocal2Parent;
    }
    for (int i = 0; i < mChildren.size(); ++i) {
        mChildren[i]->updateTransformation();
    }
}

void AnimatableTransform::Attach(AnimatableTransform* pParent, AnimatableTransform* pChild)
{
    if (pChild)
    {
        Joint* pOldParent = pChild->mParent;
        if (pOldParent)
        {
            // erase the child from old parent's children list
            std::vector<Joint*>::iterator iter;
            for (iter = pOldParent->mChildren.begin(); iter != pOldParent->mChildren.end(); iter++)
            {
                if ((*iter) == pChild)
                {
                    iter = pOldParent->mChildren.erase(iter);
                }
            }
        }
        // Set the new parent
        pChild->mParent = pParent;
        // Add child to new parent's children list
        if (pParent)
        {
            pParent->mChildren.push_back(pChild);
        }
    }
}

void AnimatableTransform::Detach(AnimatableTransform* pParent, AnimatableTransform* pChild)
{
    if (pChild && pChild->mParent == pParent)
    {
        if (pParent)
        {
            // erase the child from parent's children list
            for (int i = 0; i < (int) pParent->mChildren.size(); i++)
            {
                if (pParent->mChildren[i] == pChild)
                {
                    for (int j = i; j < (int) pParent->mChildren.size() - 1; j++)
                    {
                        pParent->mChildren[j] = pParent->mChildren[j + 1];
                    }
                    pParent->mChildren.resize(pParent->mChildren.size() - 1);
                    break;
                }
            }
        }
        pChild->mParent = NULL;
    }
}


/****************************************************************
*
*    	    Hierarchy
*
****************************************************************/

AnimatableHierarchy::AnimatableHierarchy() : mRoot(0)
{
}

AnimatableHierarchy::AnimatableHierarchy(const AnimatableHierarchy& skeleton)
{
    *this = skeleton;
}

AnimatableHierarchy& AnimatableHierarchy::operator = (const AnimatableHierarchy& orig)
{
    // Performs a deep copy
    if (&orig == this)
    {
        return *this;
    }

    mTransforms.clear();
    mRoot = 0;

    // Copy joints
    for (unsigned int i = 0; i < orig.mTransforms.size(); i++)
    {
        AnimatableTransform* joint = new AnimatableTransform(*(orig.mTransforms[i]));
        mTransforms.push_back(joint);
        //std::cout << "Copy " << joint->GetName() << std::endl;
    }

    // Copy parent/children relationships
    for (unsigned int i = 0; i < orig.mTransforms.size(); i++)
    {
        Joint* joint = orig.mTransforms[i];
        if (joint->getParent())
        {
            Joint* parent = mTransforms[joint->getParent()->getID()];
            mTransforms[i]->setParent(parent);
        }
        else
        {
            mRoot = mTransforms[i];
            mRoot->setParent(0);
        }

        for (int j = 0; j < joint->getNumChildren(); j++)
        {
            AnimatableTransform* child = mTransforms[joint->getChildAt(j)->getID()];
            mTransforms[i]->appendChild(child);
        }
    }

    return *this;
}

AnimatableHierarchy::~AnimatableHierarchy()
{
    clear();
}

void AnimatableHierarchy::clear()
{
    mRoot = NULL;
    mTransforms.clear();
}

void AnimatableHierarchy::updateTransforms()
{
    if (!mRoot) return; // Nothing loaded
    mRoot->updateTransformation();
}

AnimatableTransform* AnimatableHierarchy::getByName(const std::string& name) const
{
    std::vector<AnimatableTransform*>::const_iterator iter;
    for (iter = mTransforms.begin(); iter != mTransforms.end(); iter++)
    {
        if (name == ((*iter)->getName()))
            return (*iter);
    }
    return NULL;
}
AnimatableTransform* AnimatableHierarchy::getByID(int id) const
{
    assert(id >= 0 && id < (int) mTransforms.size());
    return mTransforms[id];
}
AnimatableTransform* AnimatableHierarchy::getRoot() const
{
    return mRoot;
}

void AnimatableHierarchy::addTransform(AnimatableTransform* joint, bool isRoot)
{
    joint->setID(mTransforms.size());
    mTransforms.push_back(joint);
    if (isRoot) mRoot = joint;
}
void AnimatableHierarchy::deleteTransform(const std::string& name)
{
    AnimatableTransform* joint = getByName(name);
    if (!joint) return; // no work to do

    for (int i = 0; i < joint->getNumChildren(); i++)
    {
        AnimatableTransform* child = joint->getChildAt(i);
        AnimatableTransform::Detach(joint, child);
        deleteTransform(child->getName());
    }

    // Re-assign ids and delete orphans
    AnimatableTransform* parent = joint->getParent();
    if (parent)
    {
        AnimatableTransform::Detach(parent, joint);
        if (parent->getNumChildren() == 0) parent->setNumChannels(0);
    }
    for (int i = joint->getID(); i < (int) mTransforms.size() - 1; i++)
    {
        mTransforms[i] = mTransforms[i + 1];
        mTransforms[i]->setID(i);
    }
    mTransforms.resize(mTransforms.size() - 1);
    delete joint;
}
