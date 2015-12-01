/**************************************************************************
  CSC C85 - UTSC RoboSoccer AI core

  This file is where the actual planning is done and commands are sent
  to the robot.

  Please read all comments in this file, and add code where needed to
  implement your game playing logic.

  Things to consider:

  - Plan - don't just react
  - Use the heading vectors!
  - Mind the noise (it's everywhere)
  - Try to predict what your oponent will do
  - Use feedback from the camera

  What your code should not do:

  - Attack the opponent, or otherwise behave aggressively toward the
    oponent
  - Hog the ball (you can kick it, push it, or leave it alone)
  - Sit at the goal-line or inside the goal
  - Run completely out of bounds

  AI scaffold: Parker-Lee-Estrada, Summer 2013

  Version: 0.2 - Updated Oct 2, 2014 - F. Estrada
***************************************************************************/

#include "imagecapture/imageCapture.h"
#include "roboAI.h"			// <--- Look at this header file!
#include <nxtlibc/nxtlibc.h>
#include <stdio.h>
#include <stdlib.h>

#define PI 3.14159265358979323846

void clear_motion_flags(struct RoboAI *ai)
{
// Reset all motion flags. See roboAI.h for what each flag represents
// *You may or may not want to use these*
    ai->st.mv_fwd=0;
    ai->st.mv_back=0;
    ai->st.mv_bl=0;
    ai->st.mv_br=0;
    ai->st.mv_fl=0;
    ai->st.mv_fr=0;
}

struct blob *id_coloured_blob(struct RoboAI *ai, struct blob *blobs, int col)
{
/////////////////////////////////////////////////////////////////////////////
// ** DO NOT CHANGE THIS FUNCTION **
// This function looks for and identifies a blob with the specified colour.
// It uses colour contrast betwen the R, G, and B channels to pick the
// blob that is closest in colour to the specified target. If multiple
// blobs with similar colour exist, then it picks the most saturated one.
//
// Inputs: The robot's AI data structure, a list of blobs, and a colour target:
// Colour parameter: 0 -> R
//                   1 -> G
//                   2 -> B
// Returns: Pointer to the blob with the desired colour, or NULL if no such
// 	     blob can be found.
/////////////////////////////////////////////////////////////////////////////

    struct blob *p, *fnd;
    double BCRT=1.05;			// Ball colour ratio threshold
    double c1,c2,c3,m,mi,ma;
    double oc1,oc2,oc3;
    int i;

    oc1=1000;
    oc2=1;
    oc3=1;

    p=blobs;
    fnd=NULL;
    while (p!=NULL)
    {
        if (col==0) {
            c1=p->R;    // detect red
            c2=p->G;
            c3=p->B;
        }
        else if (col==1) {
            c1=p->G;    // detect green
            c2=p->R;
            c3=p->B;
        }
        else if (col==2) {
            c1=p->B;    // detect blue
            c2=p->G;
            c3=p->R;
        }

        // Normalization and range extension
        mi=p->R;
        if (p->G<mi) mi=p->G;
        if (p->B<mi) mi=p->B;
        ma=p->R;
        if (p->G>ma) ma=p->G;
        if (p->B>ma) ma=p->B;

        c1=(c1-mi)/(ma-mi);
        c2=(c2-mi)/(ma-mi);
        c3=(c3-mi)/(ma-mi);
        c1+=.001;
        c2+=.001;
        c3+=.001;

        if (c1/c2>BCRT&&c1/c3>BCRT)			// Blob has sufficient colour contrast
        {
            m=(c1/c2)+(c1/c3);				// Total color contrast ch1 vs ch2 and ch3
            if (fnd==NULL||m>(oc1/oc2)+(oc1/oc3)) 	// Found the first blob with this color, or a more colorful one
            {
                fnd=p;
                oc1=c1;
                oc2=c2;
                oc3=c3;
            }
        }
        p=p->next;
    }

    return(fnd);
}

void track_agents(struct RoboAI *ai, struct blob *blobs)
{
////////////////////////////////////////////////////////////////////////
// ** DO NOT CHANGE THIS FUNCTION **
// This function does the tracking of each agent in the field. It looks
// for blobs that represent the bot, the ball, and our opponent (which
// colour is assigned to each bot is determined by a command line
// parameter).
// It keeps track within the robot's AI data structure of multiple
// parameters related to each agent:
// - Position
// - Velocity vector. Not valid while rotating, but possibly valid
//   while turning.
// - Heading (a unit vector in the direction of motion). Not valid
//   while rotating - possibly valid while turning
// - Pointers to the blob data structure for each agent
//
// This function will update the blob data structure with the velocity
// and heading information from tracking.
//
// In addition to this, if calibration data is available then this
// function adjusts the Y location of the bot and the opponent to
// adjust for perspective projection error. See the handout on how
// to perform the calibration process.
//
// Note that the blob data
// structure itself contains another useful vector with the blob
// orientation (obtained directly from the blob shape, valid at all
// times even under rotation, but can be pointing backward!)
//
// This function receives a pointer to the robot's AI data structure,
// and a list of blobs.
/////////////////////////////////////////////////////////////////////////

    struct blob *p;
    double mg,vx,vy,pink,doff,dmin,dmax,adj;
    double NOISE_VAR=5;

// Reset ID flags
    ai->st.ballID=0;
    ai->st.selfID=0;
    ai->st.oppID=0;
    ai->st.ball=NULL;			// Be sure you check these are not NULL before
    ai->st.self=NULL;			// trying to access data for the ball/self/opponent!
    ai->st.opp=NULL;

// Find the ball
    p=id_coloured_blob(ai,blobs,2);
    if (p)
    {
        ai->st.ball=p;			// New pointer to ball
        ai->st.ballID=1;			// Set ID flag for ball (we found it!)
        ai->st.bvx=p->cx-ai->st.old_bcx;	// Update ball velocity in ai structure and blob structure
        ai->st.bvy=p->cy-ai->st.old_bcy;
        ai->st.ball->vx=ai->st.bvx;
        ai->st.ball->vy=ai->st.bvy;

        ai->st.old_bcx=p->cx; 		// Update old position for next frame's computation
        ai->st.old_bcy=p->cy;
        ai->st.ball->idtype=3;

        vx=ai->st.bvx;			// Compute heading direction (normalized motion vector)
        vy=ai->st.bvy;
        mg=sqrt((vx*vx)+(vy*vy));
        if (mg>NOISE_VAR)			// Update heading vector if meaningful motion detected
        {
            vx/=mg;
            vy/=mg;
            ai->st.bmx=vx;
            ai->st.bmy=vy;
        }
        ai->st.ball->mx=ai->st.bmx;
        ai->st.ball->my=ai->st.bmy;
    }
    else {
        ai->st.ball=NULL;
    }

// ID our bot
    if (ai->st.botCol==0) p=id_coloured_blob(ai,blobs,1);
    else p=id_coloured_blob(ai,blobs,0);
    if (p)
    {
        ai->st.self=p;			// Update pointer to self-blob

        // Adjust Y position if we have calibration data
        if (fabs(p->adj_Y[0][0])>.1)
        {
            dmax=384.0-p->adj_Y[0][0];
            dmin=767.0-p->adj_Y[1][0];
            pink=(dmax-dmin)/(768.0-384.0);
            adj=dmin+((p->adj_Y[1][0]-p->cy)*pink);
            p->cy=p->cy+adj;
            if (p->cy>767) p->cy=767;
            if (p->cy<1) p->cy=1;
        }

        ai->st.selfID=1;
        ai->st.svx=p->cx-ai->st.old_scx;
        ai->st.svy=p->cy-ai->st.old_scy;
        ai->st.self->vx=ai->st.svx;
        ai->st.self->vy=ai->st.svy;

        ai->st.old_scx=p->cx;
        ai->st.old_scy=p->cy;
        ai->st.self->idtype=1;

        vx=ai->st.svx;
        vy=ai->st.svy;
        mg=sqrt((vx*vx)+(vy*vy));
        if (mg>NOISE_VAR)
        {
            vx/=mg;
            vy/=mg;
            ai->st.smx=vx;
            ai->st.smy=vy;
        }

        ai->st.self->mx=ai->st.smx;
        ai->st.self->my=ai->st.smy;
    }
    else ai->st.self=NULL;

// ID our opponent
    if (ai->st.botCol==0) p=id_coloured_blob(ai,blobs,0);
    else p=id_coloured_blob(ai,blobs,1);
    if (p)
    {
        ai->st.opp=p;

        if (fabs(p->adj_Y[0][1])>.1)
        {
            dmax=384.0-p->adj_Y[0][1];
            dmin=767.0-p->adj_Y[1][1];
            pink=(dmax-dmin)/(768.0-384.0);
            adj=dmin+((p->adj_Y[1][1]-p->cy)*pink);
            p->cy=p->cy+adj;
            if (p->cy>767) p->cy=767;
            if (p->cy<1) p->cy=1;
        }

        ai->st.oppID=1;
        ai->st.ovx=p->cx-ai->st.old_ocx;
        ai->st.ovy=p->cy-ai->st.old_ocy;
        ai->st.opp->vx=ai->st.ovx;
        ai->st.opp->vy=ai->st.ovy;

        ai->st.old_ocx=p->cx;
        ai->st.old_ocy=p->cy;
        ai->st.opp->idtype=2;

        vx=ai->st.ovx;
        vy=ai->st.ovy;
        mg=sqrt((vx*vx)+(vy*vy));
        if (mg>NOISE_VAR)
        {
            vx/=mg;
            vy/=mg;
            ai->st.omx=vx;
            ai->st.omy=vy;
        }
        ai->st.opp->mx=ai->st.omx;
        ai->st.opp->my=ai->st.omy;
    }
    else ai->st.opp=NULL;

}

void id_bot(struct RoboAI *ai, struct blob *blobs)
{
///////////////////////////////////////////////////////////////////////////////
// ** DO NOT CHANGE THIS FUNCTION **
// This routine calls track_agents() to identify the blobs corresponding to the
// robots and the ball. It commands the bot to move forward slowly so heading
// can be established from blob-tracking.
//
// NOTE 1: All heading estimates, velocity vectors, position, and orientation
//         are noisy. Remember what you have learned about noise management.
//
// NOTE 2: Heading and velocity estimates are not valid while the robot is
//         rotating in place (and the final heading vector is not valid either).
//         To re-establish heading, forward/backward motion is needed.
//
// NOTE 3: However, you do have a reliable orientation vector within the blob
//         data structures derived from blob shape. It points along the long
//         side of the rectangular 'uniform' of your bot. It is valid at all
//         times (even when rotating), but may be pointing backward and the
//         pointing direction can change over time.
//
// You should *NOT* call this function during the game. This is only for the
// initialization step. Calling this function during the game will result in
// unpredictable behaviour since it will update the AI state.
///////////////////////////////////////////////////////////////////////////////

    struct blob *p;
    static double stepID=0;
    double frame_inc=1.0/5.0;

    drive_speed(30);			// Start forward motion to establish heading
    // Will move for a few frames.

    track_agents(ai,blobs);		// Call the tracking function to find each agent

    if (ai->st.selfID==1&&ai->st.self!=NULL)
        fprintf(stderr,"Successfully identified self blob at (%f,%f)\n",ai->st.self->cx,ai->st.self->cy);
    if (ai->st.oppID==1&&ai->st.opp!=NULL)
        fprintf(stderr,"Successfully identified opponent blob at (%f,%f)\n",ai->st.opp->cx,ai->st.opp->cy);
    if (ai->st.ballID==1&&ai->st.ball!=NULL)
        fprintf(stderr,"Successfully identified ball blob at (%f,%f)\n",ai->st.ball->cx,ai->st.ball->cy);

    stepID+=frame_inc;
    if (stepID>=1&&ai->st.selfID==1)	// Stop after a suitable number of frames.
    {
        ai->st.state+=1;
        stepID=0;
        all_stop();
    }
    else if (stepID>=1) stepID=0;

// At each point, each agent currently in the field should have been identified.
    return;
}

int setupAI(int mode, int own_col, struct RoboAI *ai)
{
/////////////////////////////////////////////////////////////////////////////
// ** DO NOT CHANGE THIS FUNCTION **
// This sets up the initial AI for the robot. There are three different modes:
//
// SOCCER -> Complete AI, tries to win a soccer game against an opponent
// PENALTY -> Score a goal (no goalie!)
// CHASE -> Kick the ball and chase it around the field
//
// Each mode sets a different initial state (0, 100, 200). Hence,
// AI states for SOCCER will be 0 through 99
// AI states for PENALTY will be 100 through 199
// AI states for CHASE will be 200 through 299
//
// You will of course have to add code to the AI_main() routine to handle
// each mode's states and do the right thing.
//
// Your bot should not become confused about what mode it started in!
//////////////////////////////////////////////////////////////////////////////

    switch (mode) {
    case AI_SOCCER:
        fprintf(stderr,"Standard Robo-Soccer mode requested\n");
        ai->st.state=0;		// <-- Set AI initial state to 0
        break;
    case AI_PENALTY:
        fprintf(stderr,"Penalty mode! let's kick it!\n");
        ai->st.state=100;	// <-- Set AI initial state to 100
        break;
    case AI_CHASE:
        fprintf(stderr,"Chasing the ball...\n");
        ai->st.state=200;	// <-- Set AI initial state to 200
        break;
    default:
        fprintf(stderr, "AI mode %d is not implemented, setting mode to SOCCER\n", mode);
        ai->st.state=0;
    }

    all_stop();			// Stop bot,
    ai->runAI = AI_main;		// and initialize all remaining AI data
    ai->calibrate = AI_calibrate;
    ai->st.ball=NULL;
    ai->st.self=NULL;
    ai->st.opp=NULL;
    ai->st.side=0;
    ai->st.botCol=own_col;
    ai->st.old_bcx=0;
    ai->st.old_bcy=0;
    ai->st.old_scx=0;
    ai->st.old_scy=0;
    ai->st.old_ocx=0;
    ai->st.old_ocy=0;
    ai->st.bvx=0;
    ai->st.bvy=0;
    ai->st.svx=0;
    ai->st.svy=0;
    ai->st.ovx=0;
    ai->st.ovy=0;
    ai->st.selfID=0;
    ai->st.oppID=0;
    ai->st.ballID=0;
    clear_motion_flags(ai);
    fprintf(stderr,"Initialized!\n");

    return(1);
}

void AI_calibrate(struct RoboAI *ai, struct blob *blobs)
{
// Basic colour blob tracking loop for calibration of vertical offset
// See the handout for the sequence of steps needed to achieve calibration.
    track_agents(ai,blobs);
}

void AI_main(struct RoboAI *ai, struct blob *blobs, void *state)
{
    /*************************************************************************
     This is the main AI loop.

     It is called by the imageCapture code *once* per frame. And it *must not*
     enter a loop or wait for visual events, since no visual refresh will happen
     until this call returns!

     Therefore. Everything you do in here must be based on the states in your
     AI and the actions the robot will perform must be started or stopped
     depending on *state transitions*.

     E.g. If your robot is currently standing still, with state = 03, and
      your AI determines it should start moving forward and transition to
      state 4. Then what you must do is
      - send a command to start forward motion at the desired speed
      - update the robot's state
      - return

     I can not emphasize this enough. Unless this call returns, no image
     processing will occur, no new information will be processed, and your
     bot will be stuck on its last action/state.

     You will be working with a state-based AI. You are free to determine
     how many states there will be, what each state will represent, and
     what actions the robot will perform based on the state as well as the
     state transitions.

     You must *FULLY* document your state representation in the report

     The first two states for each more are already defined:
     State 0,100,200 - Before robot ID has taken place (this state is the initial
               	    state, or is the result of pressing 'r' to reset the AI)
     State 1,101,201 - State after robot ID has taken place. At this point the AI
               	    knows where the robot is, as well as where the opponent and
               	    ball are (if visible on the playfield)

     Relevant UI keyboard commands:
     'r' - reset the AI. Will set AI state to zero and re-initialize the AI
    data structure.
     't' - Toggle the AI routine (i.e. start/stop calls to AI_main() ).
     'o' - Robot immediate all-stop! - do not allow your NXT to get damaged!

     ** Do not change the behaviour of the robot ID routine **
    **************************************************************************/
    //fprintf(stderr,"State %d\n", ai->st.state);
    if (ai->st.state==0||ai->st.state==100||ai->st.state==200)  	// Initial set up - find own, ball, and opponent blobs
    {
        // Carry out self id process.
        fprintf(stderr,"Initial state, self-id in progress...\n");
        id_bot(ai,blobs);
        if ((ai->st.state%100)!=0)	// The id_bot() routine will change the AI state to initial state + 1
        {   // if robot identification is successful.
            if (ai->st.self->cx>=512) ai->st.side=1;
            else ai->st.side=0;
            all_stop();
            clear_motion_flags(ai);
            fprintf(stderr,"Self-ID complete. Current position: (%f,%f), current heading: [%f, %f], AI state=%d\n",ai->st.self->cx,ai->st.self->cy,ai->st.self->mx,ai->st.self->my,ai->st.state);
        }
    }
    else
    {
        /****************************************************************************
         TO DO:
         You will need to replace this 'catch-all' code with actual program logic to
         implement your bot's state-based AI.

         After id_bot() has successfully completed its work, the state should be
         1 - if the bot is in SOCCER mode
         101 - if the bot is in PENALTY mode
         201 - if the bot is in CHASE mode

         Your AI code needs to handle these states and their associated state
         transitions which will determine the robot's behaviour for each mode.

         Please note that in this function you should add appropriate functions below
         to handle each state's processing, and the code here should mostly deal with
         state transitions and with calling the appropriate function based on what
         the bot is supposed to be doing.
        *****************************************************************************/
        //fprintf(stderr,"Just trackin'! State %d\n", ai->st.state);	// bot, opponent, and ball.
        track_agents(ai,blobs);		// Currently, does nothing but endlessly track

        switch(ai->st.state)
        {
        //Soccer
        case 0:
            soccer_start(ai, blobs, state);
            break;
        case 1:
            soccer_start(ai, blobs, state);
            break;
        //Penalty
        case 101:
            penalty_start(ai, blobs, state);
            ai->st.frame_count = 0;
            ai->st.motor_power = 0;
            ai->st.accumulated_proximity=0;
            break;
        case 102:
            penalty_align(ai, blobs, state);
            ai->st.frame_count++;
            break;
        case 103:
            penalty_approach(ai, blobs, state);
            break;
        case 105:
            break; // Final penalty state, nothing to do

        case 201:
            chase_start(ai, blobs, state);
            break;
        case 202:
            chase_lostBall(ai, blobs, state);
            break;
        case 203:
            chase_chaseBall(ai, blobs, state);
            break;
        case 204:
            chase_kickBall(ai, blobs, state);
            break;
        }
    }

}

/**********************************************************************************

                                 Helper functions

**********************************************************************************/
double avoid_border(double *point)
{
    const double border_dist = 100; // Go no closer than this to the outer borders    
    double self_x = point[0];
    double self_y = point[1];

    if (self_x < border_dist) // Too close to left side
    {
        printf("Too close to left side\n");
        if (self_y < border_dist) // Too close to top and left
        {
           return PI/4; // Head down/right
        }
        else if (self_y > 768 - border_dist) // Too close to bottom
        {
           return -PI/4; // Head up/right
        }
        return 0; // Head right
    }
    else if (self_x > 1024 - border_dist) // Too close to right side
    {
        printf("Too close to right side\n");
        if (self_y < border_dist) // Too close to top and right
        {
           return PI * 0.75; // Head down/left
        }
        else if (self_y > 768 - border_dist) // Too close to bottom and right
        {
           return -PI * 0.75; // Head up/left
        }
        return PI; // Head left
    }
    else // X value ok
    {
       if (self_y < border_dist) // Too close to top
       {
           printf("Too close to top side\n");
           return PI/2; // Head down
       }
       else if (self_y > 768 - border_dist) // Too close to bottom
       {
           printf("Too close to bottom side\n");
           return -PI/2; // Head up
       }
    }
}
double arc_heading_ppa(double *point_1, double *point_2, double final_angle)//double *point_3)
{
/////////////////////////////////////////////////////////////////////////////
// Return target angle in radians, from two points and a final angle.
//
// Find the heading that follows an arced path that passes through point_1
// and point_2, such that the angle when reaching point_2 is final_angle.
/////////////////////////////////////////////////////////////////////////////
 
    printf("arc_heading\n\t1:%f, %f\n\t2:%f, %f\n\tfinal_angle:%f\n",
           point_1[0], point_1[1],
           point_2[0], point_2[1],
           final_angle * PI / 180);
    
    const double border_dist = 100; // Go no closer than this to the outer borders    
    double self_x = point_1[0];
    double self_y = point_1[1];
    
    if (self_x < border_dist || self_x > 1024 - border_dist ||
        self_y < border_dist || self_y > 768 - border_dist)
        return avoid_border(point_1);
    
    //double final_angle = atan2(point_3[1] - point_2[1],
    //                           point_3[0] - point_2[0]);
    
    printf("final_angle: %f\n", final_angle);
    if (find_distance(point_1, point_2) < 100)
    {
       return final_angle;
    }
    
    double target_angle = 2 * atan2(point_1[1] - point_2[1],
                                    point_1[0] - point_2[0])
                          - final_angle;
    while (target_angle >= PI)
        target_angle -= 2 * PI;
    while (target_angle < -PI)
        target_angle += 2 * PI;
    return target_angle;
}
double find_distance(double *point_1, double *point_2)
{
    return sqrt(pow(point_2[0] - point_1[0], 2) +
                pow(point_2[1] - point_1[1], 2));
}

double distance_to_line(double *point_1, double *point_2, double *point_3)
{
    // Adapted from https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
    double x_1 = point_1[0];
    double y_1 = point_1[1];
    double x_2 = point_2[0];
    double y_2 = point_2[1];
    double x_3 = point_3[0];
    double y_3 = point_3[1];

    return (abs(x_1 * (y_3 - y_2)
                - y_1 * (x_3 - x_2)
                + x_3 * y_2 - y_3 * x_2)
            / find_distance(point_2, point_3));
}


double angle_to(double *point_1, double *point_2)
{
    return atan2(point_1[1] - point_2[1],
                 point_1[0] - point_2[0]);
}

double add_angles(double angle1, double angle2)
{
    double theta = angle1 + angle2;
    while (theta < -PI)
        theta += 2 * PI;
    while (theta >= PI)
        theta -= 2 * PI;
    return theta;
}

double attack_heading(double *self_loc, double *ball_loc, double *goal_loc)
{
    double BORDER_WIDTH = 150;
    double theta;
    double x = self_loc[0];
    double y = self_loc[1];
    if (distance_to_line(self_loc, ball_loc, goal_loc) < 20 && find_distance(self_loc, ball_loc) < 100)
    {
        theta = angle_to(goal_loc, ball_loc);
    }
    else if (distance_to_line(self_loc, ball_loc, goal_loc) < 100
          && find_distance(self_loc, ball_loc) < 200
          && find_distance(self_loc, goal_loc) < 20 + find_distance(ball_loc, goal_loc))
    {
        if (add_angles(angle_to(ball_loc, goal_loc), -angle_to(self_loc, goal_loc)) > 0)
        {
            theta = angle_to(goal_loc, ball_loc) + PI/2;
        }
        else
        {
            theta = angle_to(goal_loc, ball_loc) - PI/2;
        }
    }    
    else if (abs(add_angles(angle_to(self_loc, ball_loc), -angle_to(goal_loc, ball_loc))) < PI/2)
    {
        theta = angle_to(ball_loc, goal_loc);
    }
    else
    {
        theta = -angle_to(goal_loc, ball_loc)+ 2*angle_to(self_loc, ball_loc);
    }
    
    double vector_x = cos(theta);
    double vector_y = sin(theta);
    
    if (x < BORDER_WIDTH)
        vector_x = fmax(vector_x, (BORDER_WIDTH - x) / BORDER_WIDTH);
    if (x > 1024 - BORDER_WIDTH)
        vector_x = fmin(vector_x, -(x - (1024 - BORDER_WIDTH)) / BORDER_WIDTH);
    if (y < BORDER_WIDTH)
        vector_y = fmax(vector_y, (BORDER_WIDTH - y) / BORDER_WIDTH);
    if (y > 768 - BORDER_WIDTH)
        vector_y = fmin(vector_y, -(y - (768 - BORDER_WIDTH)) / BORDER_WIDTH);
        
    theta = atan2(vector_y, vector_x);

    return theta; //x + scale * vector_x, y + scale * vector_y;
}


double arc_heading(double *point_1, double *point_2, double *point_3)
{
/////////////////////////////////////////////////////////////////////////////
// Return target angle in radians.
//
// Find the heading that follows an arced path that passes through point_1
// and point_2, such that the line from point_2 to point_3 is a tangent to
// the arc. Something following this path would be aimed at point_3 when it
// reached point_2.
/////////////////////////////////////////////////////////////////////////////
 
    printf("arc_heading\n\t1:%f, %f\n\t2:%f, %f\n\t3:%f, %f\n",
           point_1[0], point_1[1],
           point_2[0], point_2[1],
           point_3[0], point_3[1]);
    
    const double border_dist = 100; // Go no closer than this to the outer borders    
    double shot_angle = atan2(point_3[1] - point_2[1],
                               point_3[0] - point_2[0]);
    double self_x = point_1[0];
    double self_y = point_1[1];
    
    printf("shot_angle: %f\n", shot_angle);
    if (find_distance(point_1, point_2) < 100)
    {
       return shot_angle;
    }
    
    if (self_x < border_dist) // Too close to left side
    {
        printf("Too close to left side\n");
        if (self_y < border_dist) // Too close to top and left
        {
           return PI/4; // Head down/right
        }
        else if (self_y > 768 - border_dist) // Too close to bottom
        {
           return -PI/4; // Head up/right
        }
        return 0; // Head right
    }
    else if (self_x > 1024 - border_dist) // Too close to right side
    {
        printf("Too close to right side\n");
        if (self_y < border_dist) // Too close to top and right
        {
           return PI * 0.75; // Head down/left
        }
        else if (self_y > 768 - border_dist) // Too close to bottom and right
        {
           return -PI * 0.75; // Head up/left
        }
        return PI; // Head left
    }
    else // X value ok
    {
       if (self_y < border_dist) // Too close to top
       {
           printf("Too close to top side\n");
           return PI/2; // Head down
       }
       else if (self_y > 768 - border_dist) // Too close to bottom
       {
           printf("Too close to bottom side\n");
           return -PI/2; // Head up
       }
    }
    

    double target_angle = 2 * atan2(point_1[1] - point_2[1],
                                    point_1[0] - point_2[0])
                          - atan2(point_3[1] - point_2[1],
                                  point_3[0] - point_2[0]);
    while (target_angle >= PI)
        target_angle -= 2 * PI;
    while (target_angle < -PI)
        target_angle += 2 * PI;
    return target_angle;
}


void get_rally_point(struct RoboAI *ai, double distance_back, double *result)
{
    
    double self_loc[2] = {ai->st.self->cx, ai->st.self->cy};
    double ball_loc[2] = {ai->st.ball->cx, ai->st.ball->cy};
    // Coordinates of opponent's goal
    double goal_loc[2] = {(ai->st.side == 0) ?  0.0: 1024.0, 384};
    double ball_to_goal_dist = find_distance(ball_loc, goal_loc);
    double bot_to_ball_dist = find_distance(ball_loc, self_loc);
    //printf("bot_to_ball_dist=%f\n", bot_to_ball_dist);

    result[0] = ball_loc[0] - distance_back * (goal_loc[0]-ball_loc[0]) / ball_to_goal_dist;
    result[1] = ball_loc[1] - distance_back * (goal_loc[1]-ball_loc[1]) / ball_to_goal_dist;
}


/**********************************************************************************

                            State functions

**********************************************************************************/

void move_to(struct RoboAI *ai, double *target_loc, double target_theta)
/////////////////////////////////////////////////////////////////////////////
//  Move to ai->st.target_x, ai->st.target_y, trying to be pointing in
//  direction ai->st.target_theta when we get there.
//
//////////////////////////////////////////////////////////////////////////////
{
    const double k_P = 5;
    const double k_I = 0;//1e-8;
    const double k_D = 0;//5;//-5;//3e6;
    const double PROXIMITY = 100; // How close should we be to the rally point before we move on
    const double MAX_MOTOR_SPEED = 80;
    
    double self_loc[2] = {ai->st.self->cx, ai->st.self->cy};
    double target_heading =  arc_heading_ppa(self_loc, target_loc,
                                               target_theta);
    double heading_angle = atan2(ai->st.smy, ai->st.smx);
    double direction_angle = atan2(ai->st.self->dy, ai->st.self->dx);
    double angle_diff = fabs(heading_angle - direction_angle);
    //double bot_to_ball_dist = find_distance(ball_loc, kicker_loc);
    clock_t curr_time = clock();
    double time_diff;
    
    if (angle_diff > 0.5 * PI && angle_diff < 1.5 * PI)
    {
        direction_angle -= PI;
    }

    while (direction_angle >= PI) direction_angle -= 2 * PI;
    while (direction_angle < -PI) direction_angle += 2 * PI;
    double angle_error = direction_angle - target_heading;
    printf("Target heading:%f Angle error: %f\n", target_heading * 180 / PI,
                                                  angle_error * 180 / PI);
    //return; // Return before any movement for testing purposes
    while (angle_error >= PI) angle_error -= 2 * PI;
    while (angle_error < -PI) angle_error += 2 * PI;
    if (angle_error > PI / 8)
    {

    }
    else if (angle_error < -PI / 8)
    {
        drive_custom (40, 15);
    }
    else if (ai->st.prev_time) // prev_time is not the dummy initial value 0
    {
        time_diff = difftime(curr_time, ai->st.prev_time);
        double angle_diff = angle_error - ai->st.prev_angle_error;
        int left_speed = MAX_MOTOR_SPEED;  // Reduce later if needed for steering
        int right_speed = MAX_MOTOR_SPEED; // Reduce later if needed for steering
        while (angle_diff >= PI)
            angle_diff -= 2 * PI;
        while (angle_diff < -PI)
            angle_diff += 2 * PI;
        double d_error = 1e6 * angle_diff / time_diff;
        double weighted_sum = (k_P * (angle_error)// < 0 ? -1 : 1) * fmin(fabs(angle_error), MAX_P)
                              + k_I * ai->st.angle_error_sum
                              + k_D * d_error);
        weighted_sum = fmax(-MAX_MOTOR_SPEED, ai->st.motor_power);
        weighted_sum = fmin(MAX_MOTOR_SPEED, ai->st.motor_power);
        if (ai->st.motor_power < 0)      // Need counterclockwise heading adjustment
        {
            right_speed += ai->st.motor_power*.8;//= motor_output;//
        } 
        else // Need clockwise heading adjustment
        {
            left_speed -= ai->st.motor_power*.8;//= motor_output;//
        }
        drive_custom (left_speed, right_speed);
     }

     ai->st.angle_error_sum += angle_error * time_diff;
     ai->st.prev_angle_error = angle_error; 
     ai->st.prev_time = curr_time;
}

void penalty_start(struct RoboAI *ai, struct blob *blobs, void *state)
{
/////////////////////////////////////////////////////////////////////////////
//                          Penalty: initial state 101
//
// Wait until we know where our bot and the ball are, then progress to the
// alignment stage.
//////////////////////////////////////////////////////////////////////////////
    if(ai->st.selfID && ai->st.ballID) // Know where self and ball are
    {
        ai->st.state = 102; // Progress to alignment stage

        // Initialize values for PID
        ai->st.prev_angle_error = 0;
        ai->st.angle_error_sum = 0;
        ai->st.prev_time = 0;
        ai->st.desired_rps = 0;

    }
}

void penalty_align(struct RoboAI *ai, struct blob *blobs, void *state)
{
/////////////////////////////////////////////////////////////////////////////
//                          Penalty: alignment state 102
//
// Travel to a short distance behind the ball, arriving in line with a goal shot.
//////////////////////////////////////////////////////////////////////////////
    //double target_loc[2] = {512.0, 384.0};
    //move_to(ai, target_loc, PI);
    //return;
    
    const double k_P = 5;
    const double k_I = 0;//1e-8;
    const double k_D = 0;//5;//-5;//3e6;

    const double PROXIMITY = 100; // How close should we be to the rally point before we move on

    if (ai->st.ball == NULL || ai->st.self == NULL) // Lost ball or self
    {
        ai->st.state = 101; // Return to initial stage until ball and self found
        return;
    }
    const double MAX_MOTOR_SPEED = 100;
    int left_speed = MAX_MOTOR_SPEED;  // Reduce later if needed for steering
    int right_speed = MAX_MOTOR_SPEED; // Reduce later if needed for steering
    double  d_error;

// Locations
    double self_loc[2] = {ai->st.self->cx, ai->st.self->cy};
    double goal_loc[2] = {(ai->st.side == 0) ? 1024 : 0, 384}; // Coordinates of opponent's goal
    double ball_loc[2] = {ai->st.ball->cx, ai->st.ball->cy};
    double kicker_loc[2] = {ai->st.self->cx + 100.0 * ai->st.self->dx,
                             ai->st.self->cy + 100.0 * ai->st.self->dy};
    printf("Self loc %f, %f\n", self_loc[0], self_loc[1]);
    double rally_point[2];
    get_rally_point(ai, PROXIMITY, rally_point);
    double shot_angle = atan2(goal_loc[1] - ball_loc[1],
                                  goal_loc[0] - ball_loc[0]);
    double heading_angle = atan2(ai->st.smy, ai->st.smx);
    double direction_angle = atan2(ai->st.self->dy, ai->st.self->dx);
    double angle_diff = fabs(heading_angle - direction_angle);
    double bot_to_ball_dist = find_distance(ball_loc, kicker_loc);
    clock_t curr_time = clock();
    double weighted_sum;
    int motor_output;
    double time_diff;


    if (angle_diff > 0.5 * PI && angle_diff < 1.5 * PI)
    {
        direction_angle -= PI;
    }

    while (direction_angle >= PI)
        direction_angle -= 2 * PI;
    while (direction_angle < -PI)
        direction_angle += 2 * PI;
    //double target_heading = arc_heading(self_loc, rally_point, goal_loc);
    double target_heading = attack_heading(self_loc, rally_point, goal_loc);
    double angle_error = direction_angle - target_heading;
    printf("Target heading:%f Angle error: %f\n", target_heading * 180 / PI,
                                                  angle_error * 180 / PI);
    //return; // Return before any movement for testing purposes
    while (angle_error >= PI)
        angle_error -= 2 * PI;
    while (angle_error < -PI)
        angle_error += 2 * PI;
    if (angle_error > PI / 8 && bot_to_ball_dist > PROXIMITY*1.5)
    {
        printf("Pivoting left: angle error %f\n", angle_error * 180 / PI);
        drive_custom (15, 40);
        //pivot_left_speed(15);
        return;
    }
    else if (angle_error < -PI / 8 && bot_to_ball_dist > PROXIMITY*1.5)
    {
        printf("Pivoting right: angle error %f\n", angle_error * 180 / PI);
        //pivot_right_speed(15);
        drive_custom (40, 15);
        return;
    }
    else
        printf("PID in control. Angle error: %f\n", angle_error * 180 / PI);
    if (ai->st.prev_time) // prev_time is not the dummy initial value 0
    {
        time_diff = difftime(curr_time, ai->st.prev_time);
        double angle_diff = angle_error - ai->st.prev_angle_error;
        while (angle_diff >= PI)
            angle_diff -= 2 * PI;
        while (angle_diff < -PI)
            angle_diff += 2 * PI;
        d_error = 1e6 * angle_diff / time_diff;
        ai->st.angle_error_sum += angle_error * time_diff;
    }
    if ((angle_error < 0) != (ai->st.prev_angle_error <0)) // Error has changed sign
    {
         printf("Resetting ai->st.motor_power\n");
         ai->st.motor_power = 0; // Reset motor power "windup"
    }
    weighted_sum = (k_P * (angle_error)// < 0 ? -1 : 1) * fmin(fabs(angle_error), MAX_P)
                    + k_I * ai->st.angle_error_sum
                    + k_D * d_error);
    printf("P:%f I:%f D:%f\n",
            k_P * (angle_error),// < 0 ? -1 : 1) * fmin(fabs(angle_error), MAX_P),
            k_I * ai->st.angle_error_sum,
            k_D * d_error);
    ai->st.motor_power = weighted_sum;

    ai->st.motor_power = fmax(-MAX_MOTOR_SPEED, ai->st.motor_power);
    ai->st.motor_power = fmin(MAX_MOTOR_SPEED, ai->st.motor_power);

    if (ai->st.motor_power < 0)      // Need counterclockwise heading adjustment
    {
        //printf("Left motor correction factor:%f\n", 10/(-weighted_sum));
        right_speed += ai->st.motor_power*.8;//= motor_output;//
    }
    else // Need clockwise heading adjustment
    {
        //printf("Right motor correction factor:%f\n", 10/(weighted_sum));
        left_speed -= ai->st.motor_power*.8;//= motor_output;//
    }
    drive_custom (left_speed, right_speed);

    printf("bot_to_ball_dist=%f\n", bot_to_ball_dist);


    printf("left_speed:%d right_speed:%d ai->st.motor_power:%f\n", left_speed, right_speed,
           ai->st.motor_power);
    printf("Heading angle:%f d_error:%f degrees/sec\n"
            "Direction angle:%f Diff from heading:%f\n"
            "Arc heading:%f\n\n",
             heading_angle * 180 / PI, d_error * 180 / PI,
             direction_angle * 180 / PI,
            (direction_angle - heading_angle) * 180 / PI,
             arc_heading(self_loc, rally_point, goal_loc) * 180 / PI);

    ai->st.prev_angle_error = angle_error;
    ai->st.prev_time = curr_time;
    printf("Ball velocity:%f, %f\n", ai->st.bvx, ai->st.bvy);

    if ((bot_to_ball_dist < PROXIMITY) && (fabs(time_diff) > 1e-6))
    {
    //Is our robot in Proximity of Rally Point
        ai->st.accumulated_proximity += CLOCKS_PER_SEC/time_diff;
    }
    printf("time_diff:%f accumulated_proximity:%f (fabs(time_diff) > 1e-6):%s\n\n", 
           time_diff, ai->st.accumulated_proximity,
           ((fabs(time_diff) > 1e-6)) ? "true" : "false");
    //Is our robot in Proximity of Rally Point
    if(ai->st.accumulated_proximity > 800/MAX_MOTOR_SPEED)
    {
        //kick();
        //all_stop();
        //exit(0);
        ai->st.kick_begin = clock();
        ai->st.state = 103; // Progress to penalty_approach stage
        penalty_approach(ai, blobs, state);
    }
    else
    {
        retract_speed(10);
    }
}

void penalty_approach(struct RoboAI *ai, struct blob *blobs, void *state)
{
/////////////////////////////////////////////////////////////////////////////
//                          Penalty: approach state 103
//
//  Move straight towards the ball until close enough to kick.
//////////////////////////////////////////////////////////////////////////////
    //fprintf(stderr,"Ready to kick state 103");
    //exit(0);
    clock_t curr_time = clock();
    printf("Time since kick start:%f\n", 1.0 * difftime(curr_time, ai->st.kick_begin));
    printf("Ball velocity:%f, %f\n", ai->st.bvx, ai->st.bvy);

    if (difftime(curr_time, ai->st.kick_begin)/1e6 < 0.15)
    {
        printf("Kicking\n");
        kick_speed(90);
    }
    else if (difftime(curr_time, ai->st.kick_begin) / 1e6 < 1.5)
    {
        printf("Retracting\n");
        retract_speed(20);
    }
    else
    {
        kick_speed(0);
        all_stop();
        ai->st.state = 101; // Disable for actual penalty kick
        //exit(0);o
    }
}

void data_collection_mode(struct RoboAI *ai, struct blob *blobs, void *state)
{
    clock_t curr_time = clock();
    int right_speed = 100-(ai->st.frame_count/20);
    double heading_angle = atan2(ai->st.smy, ai->st.smx);
    if (right_speed < 0)
    {
        all_stop();
        exit(0);
    }
    drive_custom (100, right_speed);
    printf("%ld %d %f\n", curr_time, right_speed, heading_angle);
}

void soccer_start(struct RoboAI *ai, struct blob *blobs, void *state)
{
/////////////////////////////////////////////////////////////////////////////
//                          soccer_start: initial state 1
//
// Decide whether to attack or to defend based on locations of Self/Ball/Opp
// G-Self (Our Goal on Left), B- Ball, R-Opp (Goal to right)
//  G   (B)   R        -> Attack or Defend (State 2)
//  R   (B)   G        -> Chase     (State 3)
//
//  R    G   (B)       -> Attack (Straight Path) (State 4)
//  G    R   (B)       -> Attack (Curve Path) (State 5)
//
// (B)   R    G        -> Defend to Net (Curve Path) (State 6)
// (B)   G    R        -> Chase (State 7)
//
//////////////////////////////////////////////////////////////////////////////
    double self_loc[2] = {ai->st.self->cx, ai->st.self->cy};
    double opp_loc[2] = {ai->st.opp->cx, ai->st.opp->cy};
    double ball_loc[2] = {ai->st.ball->cx, ai->st.ball->cy};
    double self_goal[2] = {(ai->st.side == 1) ? 1024 : 0, 384}; 
    double opp_goal[2] = {(ai->st.side == 0) ? 1024 : 0, 384}; 

    //is_collinear(goal_loc[0],goal_loc[1], ball_loc[0],ball_loc[1], opp_loc[0], opp_loc[1] );
    printf("Defend\n");
      printf("%f  % f   %f\n",self_loc[0], ball_loc[0], opp_loc[0] );
    if (self_loc[0] <= ball_loc[0] && ball_loc[0] <= opp_loc[0])
    {
        printf("Running Defend\n");
        //ai->st.state = 2;
        //arc_heading_ppa(self_loc, ball_loc, 0);
        //drive(ai,blob,state);
    }

}


Bool is_collinear(double goalX,double goalY, double ballX, double ballY, double oppX, double oppY){
    double collinear = (goalX * (ballY - oppY) + ballX * (oppY - goalY) + oppX * (goalY - ballY) ) / 2;
    printf("COLLINEAR %f  %f! %f %f !%f %f ! %f", goalX, goalY, ballX,  ballY,  oppX,  oppY, collinear);

    if (abs(collinear) < 500){
        return True;
    }
    else{
        return False;
    }
}


/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
//                          Start CHASE Ball
//////////////////////////////////////////////////////////////////////////////

void chase_start(struct RoboAI *ai, struct blob *blobs, void *state)
{
/////////////////////////////////////////////////////////////////////////////
//                          201
//  Start of chase AI
/////////////////////////////////////////////////////////////////////////////
    if (ai->st.ballID)
    {
        ai->st.prev_angle_error = 0;
        ai->st.state = 203; // Go after ball
    }
    else
    {
        //ai->st.state = 202; // no ball or lost track of ball
    }
}

void chase_lostBall(struct RoboAI *ai, struct blob *blobs, void *state)
{
/////////////////////////////////////////////////////////////////////////////
//                          202
//  When ball blob not detected
/////////////////////////////////////////////////////////////////////////////

    // ok maybe don't need this state

}

void chase_chaseBall(struct RoboAI *ai, struct blob *blobs, void *state)
{
/////////////////////////////////////////////////////////////////////////////
//                          203
//  Go after ball
/////////////////////////////////////////////////////////////////////////////

    if (ai->st.selfID && ai->st.ballID)
    {
        const double k_P = 4;
        const double MAX_P = 1.2;
        const double MAX_MOTOR_SPEED = 80;
        int left_speed = MAX_MOTOR_SPEED; 
        int right_speed = MAX_MOTOR_SPEED;
        const double dist_kicker_offset = 100.0; // dist from kicker to bot center
        const double dist_scoop_offset = 200.0; // dist from scoop to bot center
        const double dist_scoop_width = 200.0; // width of scoop opening
        double scoop_pos[2];
        double furture_ball_pos[2];
        double vect_bot_ball[3]; // from bot to ball, x, y, rad.angle
        double angle_bot; // current robot heading angle (not velocity angle)
        double angle_error;

        //calculate center pos of scoop opening
        scoop_pos[0] += (dist_scoop_offset * ai->st.smx);
        scoop_pos[1] += (dist_scoop_offset * ai->st.smy);

        if (scoop_pos[0] < 0)
            scoop_pos[0] = 0;
        if (scoop_pos[0] > 1024)
            scoop_pos[0] = 1024;

        if (scoop_pos[1] < 0)
            scoop_pos[1] = 0;
        if (scoop_pos[1] > 768)
            scoop_pos[1] = 768;
        
        // get vector from bot to future ball
        estimate_ball_pos(ai, blobs, furture_ball_pos);
        //vect_bot_ball[0] = ai->st.ball->cx - scoop_pos[0];
        //vect_bot_ball[1] = ai->st.ball->cy - scoop_pos[1];
        vect_bot_ball[0] = ai->st.ball->cx - (dist_kicker_offset * ai->st.smx);
        vect_bot_ball[1] = ai->st.ball->cy - (dist_kicker_offset * ai->st.smy);
        vect_bot_ball[2] = atan2(vect_bot_ball[1], vect_bot_ball[0]);
        while (vect_bot_ball[2] >= PI)
            vect_bot_ball[2] -= (2 * PI);

        while (vect_bot_ball[2] < -PI)
            vect_bot_ball[2] += (2 * PI);

        angle_bot = atan2(ai->st.smy, ai->st.smx);
        while (angle_bot >= PI)
            angle_bot -= (2 * PI);

        while (angle_bot < -PI)
            angle_bot += (2 * PI);

        angle_error = vect_bot_ball[2] - angle_bot;

        while (angle_error >= PI)
            angle_error -= 2 * PI;
        while (angle_error < -PI)
            angle_error += 2 * PI;

        // If angle error is >= 90, just do a 90 degree pivot first
        if (angle_error > PI / 2)
        {
            ai->st.chase_pivot_start_angle = angle_bot;
            ai->st.chase_pivot_target_angle = angle_bot + PI/2;
            while (ai->st.chase_pivot_target_angle >= PI)
                ai->st.chase_pivot_target_angle -= 2 * PI;

            ai->st.chase_pivot_isLeft = 1;
            ai->st.state = 205;
            return; //************* P I V O T *************//
        }
        else if (angle_error < -PI /2)
        {
            ai->st.chase_pivot_start_angle = angle_bot;
            ai->st.chase_pivot_target_angle = angle_bot - PI/2;
            while (ai->st.chase_pivot_target_angle < -PI)
                ai->st.chase_pivot_target_angle += 2 * PI;

            ai->st.chase_pivot_isLeft = 0;
            ai->st.state = 205;
            return; //************* P I V O T *************//
        }

        // <90 degree angle change, just turns
        if (angle_error > PI / 8)
        {
            //printf("Pivoting left: angle error %f\n", angle_error * 180 / PI);
            drive_custom (20, 80);
            return;
        }
        else if (angle_error < -PI / 8)
        {
            //printf("Pivoting right: angle error %f\n", angle_error * 180 / PI);
            drive_custom (80, 20);
            return;
        }

        // decide how much to turn       
        ai->st.motor_power = k_P * angle_error;

        ai->st.motor_power = fmax(-MAX_MOTOR_SPEED, ai->st.motor_power);
        ai->st.motor_power = fmin(MAX_MOTOR_SPEED, ai->st.motor_power);

        if (ai->st.motor_power < 0)      // Need counterclockwise heading adjustment
        {
            right_speed += ai->st.motor_power * 0.8;
        }
        else // Need clockwise heading adjustment
        {
            left_speed -= ai->st.motor_power * 0.8;
        }
        
        drive_custom (left_speed, right_speed);

    }
    else // no ball or lost track of it
    {
        ai->st.state = 201; // no ball or lost track of ball
    }

}

void chase_kickBall(struct RoboAI *ai, struct blob *blobs, void *state)
{
/////////////////////////////////////////////////////////////////////////////
//                          204
//  Kick the ball
/////////////////////////////////////////////////////////////////////////////
    clock_t curr_time = clock();
    printf("Time since kick start:%f\n", 1.0 * difftime(curr_time, ai->st.kick_begin));
    printf("Ball velocity:%f, %f\n", ai->st.bvx, ai->st.bvy);

    if (difftime(curr_time, ai->st.kick_begin)/1e6 < 0.2)
    {
        printf("Kicking\n");
        kick_speed(100);
    }
    else if (difftime(curr_time, ai->st.kick_begin) / 1e6 < 1.5)
    {
        printf("Retracting\n");
        retract_speed(20);
    }
    else
    {
        kick_speed(0);
        all_stop();
        exit(0);
    }
}


void chase_pivot(struct RoboAI *ai, struct blob *blobs, void *state)
{
/////////////////////////////////////////////////////////////////////////////
//                          205
//  Pivot 90 degree left or right quickly, temperarily ignore other stuffs...
/////////////////////////////////////////////////////////////////////////////
    double angle_bot; // current robot heading angle (not velocity angle)

    angle_bot = atan2(ai->st.smy, ai->st.smx);
    while (angle_bot >= PI)
        angle_bot -= (2 * PI);

    while (angle_bot <= -PI)
        angle_bot += (2 * PI);

    if (ai->st.chase_pivot_isLeft) // turning left
    {
        if (angle_bot > ai->st.chase_pivot_target_angle)
        {
            //pivot_left();
            drive_custom(20, 100);
        }
        else
        {
            ai->st.state = 203; //done pivot
        }
    }
    else // chase_pivot_isLeft == 0, turning right
    {
        if (angle_bot < ai->st.chase_pivot_target_angle)
        {
            //pivot_right();
            drive_custom(100, 20);
        }
        else
        {
            ai->st.state = 203; //done pivot
        }
    }
}

// Given all blobs, guess what the ball will be at intersection with bot
// Need to check for wall...
void estimate_ball_pos(struct RoboAI *ai, struct blob * blobs, double *pos)
{
    // estimate future ball pos = 
    //  curr_pos + ball_vel * (dist_bot_ball / vel_bot)
    
    // Estimate x,y coordinates separately
    pos[0] = ai->st.ball->cx + 
            (ai->st.bvx * fabs(ai->st.ball->cx - ai->st.self->cx) / ai->st.svx);
    pos[1] = ai->st.ball->cy + 
            (ai->st.bvy * fabs(ai->st.ball->cy - ai->st.self->cy) / ai->st.svy);
    // Check for wall, reflect back
    if (pos[0] < 0.0)
    {
        pos[0] = -pos[0];
    }
    if (pos[1] < 0.0)
    {
        pos[1] = -pos[1];
    }
    
    if (pos[0] > 1024.0)
    {
        pos[0] = 1024 - (pos[0] - 1024);
    }
    if (pos[1] > 768.0)
    {
        pos[1] = 768 - (pos[1] - 768);
    }
}


/////////////////////////////////////////////////////////////////////////////
//                          END  CHASE Ball functions
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

