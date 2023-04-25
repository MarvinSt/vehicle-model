#include <iostream>
#include <limits>
#include <vector>
#include <cmath>

#include "Simbody.h"
using namespace SimTK;

struct TableEntry
{
    Real index;
    Real value;
};

class LookupTable1D
{
private:
    TableEntry m_table[48];
    int size = 0;
    bool m_allow_extrapolation_begin = true;
    bool m_allow_extrapolation_end = true;

public:
    LookupTable1D(bool allow_extrapolation_begin = true, bool allow_extrapolation_end = true)
    {
        m_allow_extrapolation_end = allow_extrapolation_begin;
        m_allow_extrapolation_end = allow_extrapolation_end;
    }

    int push_back(Real index, Real value)
    {
        if (size >= sizeof(m_table))
            return 1;

        m_table[size++] = TableEntry{index, value};
        return 0;
    }

    Real eval(Real index) const
    {
        if (size == 0)
            return 0.0;

        int idx;

        for (idx = 0; idx < size - 1; idx++)
        {
            if (m_table[idx + 1].index >= index)
                break;
        }

        auto ratio = (index - m_table[idx + 0].index) / (m_table[idx + 1].index - m_table[idx + 0].index);

        if (!m_allow_extrapolation_begin)
            if (ratio < 0.0)
                return m_table[idx + 0].value;

        if (!m_allow_extrapolation_end)
            if (ratio > 1.0)
                return m_table[idx + 1].value;

        return m_table[idx + 0].value + ratio * (m_table[idx + 1].value - m_table[idx + 0].value);
    }

    int get_size()
    {
        return size;
    }
};

class LookupTable1DVar
{
private:
    std::vector<TableEntry> table;
    bool m_allow_extrapolation_begin = true;
    bool m_allow_extrapolation_end = true;

public:
    LookupTable1DVar(bool allow_extrapolation_begin = true, bool allow_extrapolation_end = true)
    {
        m_allow_extrapolation_end = allow_extrapolation_begin;
        m_allow_extrapolation_end = allow_extrapolation_end;
    }

    int push_back(Real index, Real value)
    {
        table.push_back(TableEntry{index, value});
        return 0;
    }

    Real eval(Real index) const
    {
        if (table.size() == 0)
            return 0.0;

        int idx;

        for (idx = 0; idx < table.size() - 1; idx++)
        {
            if (table[idx + 1].index >= index)
                break;
        }

        auto ratio = (index - table[idx + 0].index) / (table[idx + 1].index - table[idx + 0].index);

        if (!m_allow_extrapolation_begin)
            if (ratio < 0.0)
                return table[idx + 0].value;

        if (!m_allow_extrapolation_end)
            if (ratio > 1.0)
                return table[idx + 1].value;

        return table[idx + 0].value + ratio * (table[idx + 1].value - table[idx + 0].value);
    }
};

class TabularSpring : public Force::Custom::Implementation
{
public:
    TabularSpring(const MobilizedBody &body1, const Vec3 &station1,
                  const MobilizedBody &body2, const Vec3 &station2, Real x0, Real f_0 = 0.0, Real x_bump = 1.0e9) // stiffness and damping
        : m_matter(body1.getMatterSubsystem()), m_body1(body1.getMobilizedBodyIndex()), m_station1(station1),
          m_body2(body2.getMobilizedBodyIndex()), m_station2(station2), m_x0(x0), f_0(f_0), m_x_bump(x_bump)
    {
        this->m_springforce = LookupTable1D();
        this->m_bumpstop = LookupTable1D(false, false);
    }

    void push_spring_table(Real disp, Real force)
    {
        m_springforce.push_back(disp, force);
    }

    void set_freelength(Real x0)
    {
        m_x0 = x0;
    }

    void push_bumpstop_table(Real disp, Real force)
    {
        m_bumpstop.push_back(disp, force);
    }

    void set_bumpstop_gap(Real m_bumpstop)
    {
        // The bumpstop gap is defined relative to the spring freelength
        // i.e. the bumpstop will engage when d > x0 + x_bump
        m_x_bump = m_bumpstop;
    }

    int get_spring_table_size()
    {
        return m_springforce.get_size();
    }

    Real eval_force(Real d) const
    {
        Real stretch = d - m_x0;                               // + -> tension, - -> compression
        auto frcSpring = m_springforce.eval(-(stretch));       // - -> tension, + -> compression
        auto frcBump = m_bumpstop.eval(-(stretch - m_x_bump)); // - -> tension, + -> compression
        return -(frcSpring + frcBump + f_0);                   // + -> tension, - -> compression
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

        const Real frcScalar = eval_force(d);

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

        const Real frcScalar = eval_force(d);

        return fabs(frcScalar * stretch / 2); // 1/2 k (x-x0)^2
        // return k * stretch * stretch / 2;    // 1/2 k (x-x0)^2
    }

private:
    const SimbodyMatterSubsystem &m_matter;

    MobilizedBodyIndex m_body1;
    MobilizedBodyIndex m_body2;
    Vec3 m_station1;
    Vec3 m_station2;

    Real m_x0;     // free length
    Real m_x_bump; // bump stop gap

    Real f_0; // static preload;

    LookupTable1D m_springforce;
    LookupTable1D m_bumpstop;
};