include_directories(/usr/include/eigen3)
set(SRC
    aVector.cpp
    aRotation.cpp
    )

set(SRC_SPLINE
    aSplineVec3.cpp
    )

set(SRC_SSPLINE
    aSplineQuaternion.cpp
    )

set(SRC_FK
    aBVHController.cpp
    aTransform.cpp
    aAnimatables.cpp
    aCharacterDrawer.cpp
    )

set(SRC_IK
    aIKController.cpp
    )

set(SRC_BLEND
    aMotionBlender.cpp
    )

set(SRC_PSYS
    aFireworks.cpp
    aJitter.cpp
    aParticleSystem.cpp
    )

set(SRC_BEHAVE
    aBehaviors.cpp
    aBehaviorController.cpp
    )

add_library(animationbasic SHARED ${SRC})
add_library(animationcurve SHARED ${SRC_SPLINE})
add_library(animationscurve SHARED ${SRC_SSPLINE})
add_library(animationfk SHARED ${SRC_FK})
add_library(animationik SHARED ${SRC_IK})
add_library(animationblend SHARED ${SRC_BLEND})
add_library(animationpsys SHARED ${SRC_PSYS})
add_library(animationbehave SHARED ${SRC_BEHAVE})
