#pragma once
// #include <iostream>
// #include <limits>
// #include <vector>
// #include <cmath>

#include "table.hpp"

#include "Simbody.h"
using namespace SimTK;

// TODO: Store potential energy in the lookup table as well
// Extrapolation is a bit more tricky, we would need to compute
// the derivative and extrapolate using the derivative

using namespace LookupTable;

namespace ForceElement
{
    /**
     * @brief TabularSpringDamper class implemented as a custom force element. This part contains a spring, damper and bump stop, all characteristics are configurable and with optional extrapolation properties.
     *
     */
    class TabularSpringDamper : public Force::Custom::Implementation
    {
    public:
        TabularSpringDamper(const MobilizedBody &body1, const Vec3 &station1,
                            const MobilizedBody &body2, const Vec3 &station2, Real x0, Real f_0 = 0.0, Real x_bump = 1.0e9) // stiffness and damping
            : m_matter(body1.getMatterSubsystem()), m_body1(body1.getMobilizedBodyIndex()), m_station1(station1),
              m_body2(body2.getMobilizedBodyIndex()), m_station2(station2), m_x0(x0), m_f0(f_0), m_x_bump(x_bump)
        {
            this->m_spring = LookupTable1D();
            this->m_bumpstop = LookupTable1D(false, false);
            this->m_damper = LookupTable1D();
        }

        void push_spring_table(Real disp, Real force)
        {
            m_spring.push_back(disp, force);
        }

        void clear_spring_table()
        {
            m_spring.clear();
        }

        void push_bumpstop_table(Real disp, Real force)
        {
            m_bumpstop.push_back(disp, force);
        }

        void clear_bumpstop_table()
        {
            m_bumpstop.clear();
        }

        void push_damper_table(Real velocity, Real force)
        {
            m_damper.push_back(velocity, force);
        }

        void clear_damper_table()
        {
            m_damper.clear();
        }

        void set_freelength(Real x0)
        {
            m_x0 = x0;
        }

        void set_bumpstop_gap(Real m_bumpstop)
        {
            // The bumpstop gap is defined relative to the spring freelength
            // i.e. the bumpstop will engage when d > x0 + x_bump
            m_x_bump = m_bumpstop;
        }

        void set_preload_force(Real f0)
        {
            m_f0 = f0;
        }

        int get_spring_table_size()
        {
            return m_spring.get_size();
        }

        Real eval_force(Real d, Real v) const
        {
            Real stretch = d - m_x0;                               // + -> tension, - -> compression
            auto frcSpring = m_spring.eval(-(stretch));            // - -> tension, + -> compression
            auto frcBump = m_bumpstop.eval(-(stretch - m_x_bump)); // - -> tension, + -> compression
            auto frcDamper = m_damper.eval(v);

            return -(frcSpring + frcBump + m_f0); // + -> tension, - -> compression
        }

        Real eval_pot_energy(Real d) const
        {
            Real stretch = d - m_x0;                                     // + -> tension, - -> compression
            auto frcSpring = m_spring.eval_integ(-(stretch));            // - -> tension, + -> compression
            auto frcBump = m_bumpstop.eval_integ(-(stretch - m_x_bump)); // - -> tension, + -> compression
            // auto frcDamper = m_damper.eval(v);

            return -(frcSpring + frcBump + m_f0 * 0.0); // + -> tension, - -> compression
        }

        virtual void calcForce(const State &state,
                               Vector_<SpatialVec> &bodyForces,
                               Vector_<Vec3> &particleForces,
                               Vector &mobilityForces) const override
        {
            const Transform &X_GB1 = m_matter.getMobilizedBody(m_body1).getBodyTransform(state);
            const Transform &X_GB2 = m_matter.getMobilizedBody(m_body2).getBodyTransform(state);

            const Vec3 s1_G = X_GB1.R() * m_station1;
            const Vec3 s2_G = X_GB2.R() * m_station2;

            const Vec3 p1_G = X_GB1.p() + s1_G; // station measured from ground origin
            const Vec3 p2_G = X_GB2.p() + s2_G;

            const Vec3 r_G = p2_G - p1_G; // vector from point1 to point2
            const Real d = r_G.norm();    // distance between the points

            const Vec3 v1_G = m_matter.getMobilizedBody(m_body1).findStationVelocityInGround(state, m_station1);
            const Vec3 v2_G = m_matter.getMobilizedBody(m_body2).findStationVelocityInGround(state, m_station2);
            const Vec3 vRel = v2_G - v1_G; // relative velocity
            const Real v = vRel.elementwiseMultiply(r_G / d).sum();

            const Real frcScalar = eval_force(d, v);

            const Vec3 f1_G = (frcScalar / d) * r_G;
            bodyForces[m_body1] += SpatialVec(s1_G % f1_G, f1_G);
            bodyForces[m_body2] -= SpatialVec(s2_G % f1_G, f1_G);
        }

        virtual Real calcPotentialEnergy(const State &state) const override
        {
            const Transform &X_GB1 = m_matter.getMobilizedBody(m_body1).getBodyTransform(state);
            const Transform &X_GB2 = m_matter.getMobilizedBody(m_body2).getBodyTransform(state);

            const Vec3 s1_G = X_GB1.R() * m_station1;
            const Vec3 s2_G = X_GB2.R() * m_station2;

            const Vec3 p1_G = X_GB1.p() + s1_G; // station measured from ground origin
            const Vec3 p2_G = X_GB2.p() + s2_G;

            const Vec3 r_G = p2_G - p1_G;  // vector from point1 to point2
            const Real d = r_G.norm();     // distance between the points
            const Real stretch = d - m_x0; // + -> tension, - -> compression

            // this is quite coarse, it should be integrated along the curve
            // const Real frcScalar = eval_force(d, 0.0);
            return fabs(eval_pot_energy(d));

            // return fabs(frcScalar * stretch / 2); // 1/2 k (x-x0)^2
            // return k * stretch * stretch / 2;    // 1/2 k (x-x0)^2
        }

    protected:
        const SimbodyMatterSubsystem &m_matter;

        MobilizedBodyIndex m_body1;
        MobilizedBodyIndex m_body2;
        Vec3 m_station1;
        Vec3 m_station2;

        Real m_x0;     // free length
        Real m_x_bump; // bump stop gap

        Real m_f0; // static preload;

        LookupTable1D m_spring;
        LookupTable1D m_bumpstop;
        LookupTable1D m_damper;
    };
}