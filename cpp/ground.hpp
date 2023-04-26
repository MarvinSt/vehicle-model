#include "Simbody.h"

using namespace SimTK;

// This Force element holds a point on one body (the "follower") onto a plane
// on another via a spring that acts always along the plane normal.
class WheelContact : public Force::Custom::Implementation
{
public:
    WheelContact(const UnitVec3 &normal,
                 const MobilizedBody &follower, const Vec3 &point,
                 Real k, Real c) // stiffness and damping
        : normal(normal), follower(follower), point(point), k(k), c(c)
    {
        assert(k >= 0 && c >= 0);
    }

    Real calcTerrainHeight(Vec3 p_contact) const
    {
        // terrain lookup... fake for now
        const Real h_terrain = 0.0;
        // we should actually recompute the deflection based on the distance of the ground position
        // to the wheel center and deduct the known distance of point again, but the error
        // we make here is likely small
        const Real x = fmax(h_terrain - p_contact[2], 0.0); // + x when below terrain
        return x;
    }

    virtual void calcForce(const State &state,
                           Vector_<SpatialVec> &bodyForces,
                           Vector_<Vec3> &particleForces,
                           Vector &mobilityForces) const override
    {
        Vec3 p_contact, v_contact;
        follower.findStationLocationAndVelocityInGround(state, point, p_contact, v_contact);

        const Real x = calcTerrainHeight(p_contact);

        const Real s = dot(v_contact, normal); // speed along normal
        const Real d = x > 0.0 ? -1.0 : 0.0;   // only apply damping when there is positive force
        const Vec3 forceOnPlane = k * x * normal + s * c * d;

        follower.applyForceToBodyPoint(state, point, forceOnPlane, bodyForces);
    }

    virtual Real calcPotentialEnergy(const State &state) const override
    {
        Vec3 p_ground, v_ground;
        follower.findStationLocationAndVelocityInGround(state, point, p_ground, v_ground);
        const Real x = calcTerrainHeight(p_ground);
        return k * x * x / 2;
    }

private:
    UnitVec3 normal;
    MobilizedBody follower;
    Vec3 point;
    Real k, c;
};