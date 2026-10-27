#ifndef DYNAMIC_CAMERA_H
#define DYNAMIC_CAMERA_H

#include <iostream>
#include <limits>

#include "camera.h"

constexpr float INF = std::numeric_limits<float>::infinity();

// AABB in 1D
struct Interval {
    float min;
    float max;

    constexpr bool contains(const Interval& other) const {
        return (min <= other.min) && (max >= other.max);
    }

    constexpr bool contains(float x) const { return (min <= x) && (x <= max); }

    // signedDistance returns a positive smallest distance of x from the interval boundary
    // if it is not contained, else it returns a smallest (in magnitude) negative distance
    // from the interval boundary.
    constexpr float signedDistance(float x) const {
        if (contains(x)) {
            // 0 or negative
            return std::max(x - max, min - x);
        }
        // positive
        return std::max(x - max, min - x);
    }
};

struct DynamicCamera {
    void update(Camera& camera,                           // the camera to update
                const sf::Vector2f& trackedPosWorld,      // world coords of player to track
                const sf::Vector2f& trackedVelocityWorld  // world velocity of player to track
    ) {
        // collect statistics on the player position and velocity in (normalized, if you like)
        // window coordinates. 
        sf::Vector2f playerPosWindowPercent = camera.worldToWindowPercentDist(trackedPosWorld-cameraUnadjustedCenter);
        playerPosWindowPercent.x += 0.5f;
        playerPosWindowPercent.y += 0.5f;

        //Compute lookAhead distance in world coords, adjusting for 
        // if the player is moving left or right.
        sf::Vector2f lookAheadWorld = camera.windowPercentToWorldDist(sf::Vector2f{lookAhead, 0.f});
        if (trackedVelocityWorld.x < 0.f) { // moving left
            lookAheadWorld.x = -lookAheadWorld.x;
        }
        else if (trackedVelocityWorld.x == 0.f) { // not moving
            lookAheadWorld.x = 0.f;
        }
        // otherwise, keep positive


        // Update lookAhead state using an EMA over its previous state and the lookAhead world
        // vector. 
        // = alpha * Xn + (1 - alpha) * Sn-1
        lookaheadState = alpha * lookAheadWorld.x + (1.f - alpha) * lookaheadState;
        
        // Determine if the player is in the dead zone, soft rejection zone, or hard
        // rejection zone. For each, perform EMA on the delta of the camera displacement 
        // (ignoring lookahead) appropriately.
        
        // get direction
        float dir = 1.f;
        if (playerPosWindowPercent.x < 0.5f) {
            dir = -1.f;
        }

        // compute deltaPercent based on which zone the player is in
        float deltaPercent = 0.f;
        if (deadZoneX.contains(playerPosWindowPercent.x)) {
            // dead zone, do nothing
        }
        else if (softZoneX.contains(playerPosWindowPercent.x)) {
            // soft rejection zone
            deltaPercent = alpha * dir * deadZoneX.signedDistance(playerPosWindowPercent.x);
        }
        else {
            // hard rejection zone
            deltaPercent = dir * softZoneX.signedDistance(playerPosWindowPercent.x);
        }
        float deltaWorld = camera.windowPercentToWorldDist(sf::Vector2f{deltaPercent, 0.f}).x;
        
        // Update cameraUnadjustedCenter, but without lookahead.
        cameraUnadjustedCenter.x += deltaWorld;

        // set the camera center by offsetting the cameraUnadjustedCenter, combining lookAhead.
        camera.setCameraCenter({cameraUnadjustedCenter.x + lookaheadState, cameraUnadjustedCenter.y});

    }

    float alpha;  // alpha to use for damping of camera position, *as well as* for smoothing of
                  // lookahead offset.
    Interval deadZoneX;  // percentage interval along x-axis to use for dead zone in [0, 1]
    // percentage interval along x-axis to use for soft rejection zone in [0, 1]
    Interval softZoneX = {-INF, INF};
    // (everything beyond softZone is considered hard zone).
    float lookAhead;  // screen-percentage lookahead
    mutable sf::Vector2f
        cameraUnadjustedCenter;      // the camera world center from last frame, ignoring lookahead
    mutable float lookaheadState{};  // the EMA of the accumulated lookahead offset in world coords
};

#endif // DYNAMIC_CAMERA_H
