#include "Simbody.h"

using namespace SimTK;

// This Force element holds a point on one body (the "follower") onto a plane
// on another via a spring that acts always along the plane normal.
class DirectionalSpringDamper : public Force::Custom::Implementation
{
public:
    DirectionalSpringDamper(const MobilizedBody &plane, const UnitVec3 &normal, Real h,
                            const MobilizedBody &follower, const Vec3 &point,
                            Real k, Real c) // stiffness and damping
        : plane(plane), normal(normal), h(h),
          follower(follower), point(point), k(k), c(c)
    {
        assert(k >= 0 && c >= 0);
    }

    virtual void calcForce(const State &state,
                           Vector_<SpatialVec> &bodyForces,
                           Vector_<Vec3> &particleForces,
                           Vector &mobilityForces) const override
    {
        const Vec3 p = follower.findStationLocationInAnotherBody(state, point, plane);
        const Vec3 v = follower.findStationVelocityInAnotherBody(state, point, plane);
        const Real x = dot(p, normal) - h; // height of point over plane
        const Real s = dot(v, normal);     // speed along normal
        const Vec3 forceOnPlane = plane.expressVectorInGroundFrame(state, k * x * normal + s * c);
        // Apply equal and opposite forces at the same point in space.
        plane.applyForceToBodyPoint(state, p, forceOnPlane, bodyForces);
        follower.applyForceToBodyPoint(state, point, -forceOnPlane, bodyForces);
    }

    virtual Real calcPotentialEnergy(const State &state) const override
    {
        const Vec3 p = follower.findStationLocationInAnotherBody(state, point, plane);
        const Real x = dot(p, normal) - h; // height of point over plane
        return k * x * x / 2;
    }

private:
    MobilizedBody plane;
    UnitVec3 normal;
    Real h;
    MobilizedBody follower;
    Vec3 point;
    Real k, c;
};