#include <string>
#include "BVHViewer.h"
#include "GL/glew.h"
#include "GL/glut.h"
#include <algorithm>
#include <AntTweakBar.h>

using namespace std;
double MAX_VEL = 150;
double L_MAX_VEL = 200;

BVHViewer::BVHViewer() : mBVHController(), mDrawer()
{
    this->NUM_CHARS = 20;
    this->leader_colors = vec3(rand()*1.0/RAND_MAX, rand()*1.0/RAND_MAX, rand()*1.0/RAND_MAX);
    this->follower_colors = vec3(rand()*1.0/RAND_MAX, rand()*1.0/RAND_MAX, rand()*1.0/RAND_MAX);
    vec3 pos, vel;

    for(int i = 0; i < NUM_CHARS; i++) {
        if (i == 0) {
            pos = vec3(-700, 0, 0);
            vel = vec3(0,0,0);
        } else {
            pos = vec3((rand()*1.0/RAND_MAX)*300, 0, (rand()*1.0/RAND_MAX)*300);
            vel = vec3((rand()*1.0/RAND_MAX)*50, 0, (rand()*1.0/RAND_MAX)*50);
        }
        
        CharacterParticle * p = new CharacterParticle(pos, vel);
        this->characters.push_back(p);
    } 
}

BVHViewer::~BVHViewer()
{
    for(int i = 0; i < NUM_CHARS; i++) {
        delete this->characters[i];
    }
}

void BVHViewer::load(const std::string& filename)
{
    loadMotion(filename);
}

void BVHViewer::loadMotion(const std::string& filename)
{   
    mBVHController.load(filename);
    mFilename = pruneName(filename);
}

void BVHViewer::initializeGui()
{
    ABasicViewer::initializeGui();
}

void BVHViewer::onStepBack()
{
    double dt = 1.0/mBVHController.getFramerate();
    mCurrentFrame = mCurrentFrame - 1;
    if (mCurrentFrame < 0) mCurrentFrame = mBVHController.getNumKeys() - 1;
    mCurrentTime = mCurrentFrame * dt;
}

void BVHViewer::onStepForward()
{
    double dt = 1.0/mBVHController.getFramerate();
    mCurrentFrame = (mCurrentFrame + 1) % mBVHController.getNumKeys();
    mCurrentTime = mCurrentFrame * dt;
}

void BVHViewer::onTimer(int value)
{   

    double dt = 1.0/mBVHController.getFramerate();

    CharacterParticle * c, * c2;
    // BOIDZ

    // Calculate the velocity for individual
    for (int i = 0; i < NUM_CHARS; i++) {
        c  = this->characters[i];

        // Special code for leader!
        if (i == 0) {
            vec3 leader_v = vec3(200 * sin(mCurrentTime/2), 0, 0);
            c->vel = 200 * leader_v;

            // cout << "LEADER VELOCITY IS:" << c->vel << endl;    
        } else {
            vec3 vel_sep, vel_coh, vel_aln, v_aln;
            double avg_speed = 0.0;

            // Look at every other individual and calculate from there
            for (int j = 0; j < NUM_CHARS; j++) {
                if (i == j) { continue; }

                c2 = this->characters[j];
                vec3 dist = c->pos - c2->pos;

                // Separation
                vel_sep += (dist/(dist.Length()) * (1.0/(dist.Length()))); 

                // Cohesion
                vel_coh += c2->pos/(NUM_CHARS * 1.0);

                // Alignment
                v_aln += c2->vel;
                avg_speed += c2->vel.Length();
            }

            vel_coh -= c->pos; // I think this goes heres
            avg_speed /= NUM_CHARS;
            double length = v_aln.Length();
            // cout << "avg speed: " << avg_speed << " v_aln: " << (v_aln/v_aln.Length()) << endl;

            if (length > 4.0) {
                vel_aln = (v_aln/v_aln.Length()) * avg_speed;
                // cout << vel_aln << endl;
            }

            // } else {
            //     cout << "ERROR: Average speed was 0" << endl<<endl;
            // }
            // c->vel += (10000 * vel_sep + vel_coh) * dt;
            c->vel += (10000 * vel_sep + vel_coh + vel_aln) * dt;
        }


        if (c->vel.Length() > MAX_VEL) {
            c->vel = c->vel / c->vel.Length() * L_MAX_VEL;
        }

        if (i != 0) {
            vec3 v_to_leader = this->characters[0]->pos - c->pos;
            v_to_leader = (v_to_leader/v_to_leader.Length()) * MAX_VEL;
            c->vel = 0.9 * c->vel + 0.1 * v_to_leader;
        }

        // Euler Integrate
        c->pos += c->vel * dt;
    }

    ABasicViewer::onTimer(value);
    mBVHController.update(mCurrentTime);
    mCurrentFrame = mBVHController.getFrame(mCurrentTime);

    // Reset root to 0, y, 0
    const vec3& old_root = mBVHController.getKey(mCurrentFrame).getRoot()->getLocalTranslation();
    const vec3& new_root = vec3(0, old_root[1], 0);
    mBVHController.getRootCurve().editKey(mCurrentFrame, new_root);
}

void BVHViewer::draw3DView()
{
    const double Rad2Deg = (180.0f / M_PI); 
    glViewport(0, 0, (GLsizei)mWindowWidth, (GLsizei)mWindowHeight);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE); // Draw the front face only, except for the texts and lights.
    glEnable(GL_LIGHTING);

    // GLfloat fogColor[4] = {1.0, 0, 0, 1};
    // glFogi(GL_FOG_MODE, GL_LINEAR);
    // glFogfv(GL_FOG_COLOR, fogColor);
    // glClearColor(fogColor[0], fogColor[1], fogColor[2], 1);

    // glHint(GL_FOG_HINT, GL_DONT_CARE);
    // glFogf(GL_FOG_DENSITY, 0.8);
    // glFogf(GL_FOG_START, 0);             // Fog Start Depth
    // glFogf(GL_FOG_END, 500.0);               // Fog End Depth
    // glEnable(GL_FOG);                   // Enables GL_FOG

    // Set the view to the current camera settings.
    mCamera.draw();

    GLfloat pos[4];
    pos[0] = mCamera.getPosition()[0];
    pos[1] = mCamera.getPosition()[1];
    pos[2] = mCamera.getPosition()[2];
    pos[3] = 1.0;
    glLightfv(GL_LIGHT0, GL_POSITION, pos);

    // TODO: Change this to loop through characters, drawing each 

    // motion1.getRootCurve().getKey(startKeyId)
    for (int i = 0; i < NUM_CHARS; i++) {
        glPushMatrix();
        vec3 pos = characters[i]->pos;
        glScalef(0.5, 0.5, 0.5);
        glTranslatef(pos[0], pos[1], pos[2]);
        vec3 velocity = characters[i]->vel;
        glRotatef(atan2(velocity[0], velocity[2])*Rad2Deg, 0, 1, 0);
        if (i) { mDrawer.chosenColor = follower_colors; }
        else { mDrawer.chosenColor = leader_colors; }
        mDrawer.draw(mBVHController.getSkeleton());
        glPopMatrix();
    }
    
    glDisable(GL_LIGHTING);
    displayGrid();
}

