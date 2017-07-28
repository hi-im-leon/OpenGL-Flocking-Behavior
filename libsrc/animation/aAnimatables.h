#ifndef Animatables_H_
#define Animatables_H_

#include "aTransform.h"
#include <vector>

// Classes for organizing animatble transforms into hierachies

class AnimatableTransform
{
public:
    AnimatableTransform();
    AnimatableTransform(const std::string& name);
    AnimatableTransform(const AnimatableTransform& joint);
    virtual AnimatableTransform& operator=(const AnimatableTransform& joint);

    virtual ~AnimatableTransform();

    AnimatableTransform* getParent();
    void setParent(AnimatableTransform* parent);

    int getNumChildren() const;
    AnimatableTransform* getChildAt(int index);
    void appendChild(AnimatableTransform* child);

    void updateTransformation();

    void setName(const std::string& name);
    void setID(int id);
    void setNumChannels(int count);
    void setRotationOrder(const std::string& order);

    void setLocal2Parent(const Transform& transform);
    void setLocalTranslation(const vec3& translation);
    void setLocalRotation(const mat3& rotation);

    int getID() const;
    const std::string& getName() const;
    int getNumChannels() const;
    const std::string& getRotationOrder() const;

    const Transform& getLocal2Parent() const;
    const vec3& getLocalTranslation() const;
    const mat3& getLocalRotation() const;

    const Transform& getLocal2Global() const;
    const vec3& getGlobalTranslation() const;
    const mat3& getGlobalRotation() const;

    static void Attach(AnimatableTransform* pParent, AnimatableTransform* pChild);
    static void Detach(AnimatableTransform* pParent, AnimatableTransform* pChild);

protected:

    int mId;
    std::string mName;
    int mChannelCount;
    std::string mRotOrder;
    bool mDirty;

    AnimatableTransform* mParent;
    std::vector<AnimatableTransform*> mChildren;

    Transform mLocal2Parent;
    Transform mLocal2Global;
};

class AnimatableHierarchy
{
public:
    AnimatableHierarchy();
    AnimatableHierarchy(const AnimatableHierarchy& skeleton); // Deep copy
    virtual AnimatableHierarchy& operator=(const AnimatableHierarchy& orig); // Deep copy

    virtual ~AnimatableHierarchy();
    virtual void updateTransforms();
    virtual void clear();

    AnimatableTransform* getByName(const std::string& name) const;
    AnimatableTransform* getByID(int id) const;
    AnimatableTransform* getRoot() const;

    void addTransform(AnimatableTransform* joint, bool isRoot = false);
    void deleteTransform(const std::string& name);

    int getNumTransforms() const { return (int) mTransforms.size(); }

protected:
    std::vector<AnimatableTransform*> mTransforms;
    AnimatableTransform* mRoot;
};


typedef AnimatableHierarchy Actor;
typedef AnimatableTransform Joint;

#endif
