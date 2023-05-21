#pragma once

#include "Simbody.h"
#include "json.hpp"

#include <string>
#include <fstream>

#include <vector>

using namespace SimTK;
using namespace json;

JSON LoadParameters(std::string filename)
{
    std::ifstream f(filename);
    std::stringstream buffer;
    buffer << f.rdbuf();
    f.close();
    return JSON::Load(buffer.str());
}

Vec3 GetVec3(JSON data, Vec3 scale = Vec3(1.0, 1.0, 1.0))
{
    return Vec3(data[0].ToFloat(), data[1].ToFloat(), data[2].ToFloat()).elementwiseMultiply(scale);
}

MassProperties GetMassInertia(JSON data, Real mass_scale = 1.0, Real inertia_scale = 1.0)
{
    auto inertia = GetVec3(data["inertia"]).scalarMultiply(inertia_scale);
    auto mass = data["mass"].ToFloat() * mass_scale;
    return MassProperties(mass, Vec3(0), Inertia(inertia));
}

Vec3 FlipAxis(Vec3 in, int axis)
{
    in[axis] = -in[axis];
    return in;
}

Rotation RotationFromDirection(Vec3 direction = Vec3(0.0, 0.0, 1.0), CoordinateAxis axis = ZAxis)
{
    return Rotation().setRotationFromOneAxis(UnitVec3(direction), axis);
}

Transform GetGlobalTransform(MobilizedBody body)
{
    auto transform = Transform(Vec3(0));
    while (true)
    {
        transform = (body.getDefaultInboardFrame() * body.getDefaultOutboardFrame().invert()) * transform;
        auto parent_body = body.getParentMobilizedBody();

        if (parent_body.isSameMobilizedBody(body) || parent_body.isGround())
            break;

        body = parent_body;
    }
    return transform;
}

Vec3 PosWorldToBody(MobilizedBody body, Vec3 position)
{
    auto transform = GetGlobalTransform(body);
    return (transform.invert() * Transform(position)).p();
}

Vec3 PosBodyToWorld(MobilizedBody body, Vec3 position)
{
    auto transform = GetGlobalTransform(body);
    return (transform * Transform(position)).p();
}

Transform GetGlobalTransformNew(MobilizedBody body)
{
    auto transform = Transform(Vec3(0));
    // transform = body.getDefaultOutboardFrame().invert();
    while (true)
    {
        transform = (body.getDefaultInboardFrame() * body.getDefaultOutboardFrame()) * transform;
        auto parent_body = body.getParentMobilizedBody();

        if (parent_body.isSameMobilizedBody(body) || parent_body.isGround())
            break;

        body = parent_body;
    }
    return transform;
}

struct FramePair
{
    Transform X_PF;
    Transform X_BM;
    // Transform X_FM;

    // FramePair() : X_FM(Transform())
    // {
    // }
};

std::vector<FramePair>
GetTransformChain(MobilizedBody body)
{
    std::vector<FramePair> transform;
    // auto transform = Transform(Vec3(0));
    // transform = body.getDefaultOutboardFrame().invert();
    while (true)
    {
        transform.push_back({body.getDefaultInboardFrame(), body.getDefaultOutboardFrame()});

        // transform = (body.getDefaultInboardFrame() * body.getDefaultOutboardFrame()) * transform;
        auto parent_body = body.getParentMobilizedBody();

        if (parent_body.isSameMobilizedBody(body) || parent_body.isGround())
            break;

        body = parent_body;
    }
    return transform;
}

Vec3 PosBodyToWorldNew(MobilizedBody body, Vec3 position)
{
    auto transform = GetGlobalTransformNew(body);
    return (transform * Transform(position)).p();
}

Rotation FromDirectionVector(Vec3 direction, CoordinateAxis axis = ZAxis)
{
    return Rotation().setRotationFromOneAxis(UnitVec3(direction), axis);
}

Rotation FromDirectionVector(Vec3 direction_a, Vec3 direction_b)
{
    return Rotation().setRotationFromTwoAxes(UnitVec3(direction_a), ZAxis, UnitVec3(direction_b), ZAxis);
}

Transform TransformWorldToBody(MobilizedBody body, Vec3 position, Vec3 direction = Vec3(0.0, 0.0, 1.0), CoordinateAxis axis = ZAxis) // , CoordinateAxis axis = ZAxis, bool invert = false
{
    auto transform = GetGlobalTransform(body);

    // calculate the original direction vector
    UnitVec3 orig_direction;
    if (axis == XAxis)
        orig_direction = transform.R().x();
    if (axis == YAxis)
        orig_direction = transform.R().y();
    if (axis == ZAxis)
        orig_direction = transform.R().z();

    // std::cout << transform.p() << std::endl;
    auto rotation = Rotation().setRotationFromTwoAxes(UnitVec3(direction), axis, UnitVec3(orig_direction), axis); // Rotation().setRotationFromOneAxis(UnitVec3(direction), axis);
    auto trans = (transform.invert() * Transform(rotation, position));
    return trans;
}

Vec3 ProjectPointOnLine(Vec3 point, Vec3 line_a, Vec3 line_b)
{
    auto ab = line_b - line_a;
    auto ap = point - line_a;

    auto t = ab.elementwiseMultiply(ap).sum() / ab.elementwiseMultiply(ab).sum();
    return line_a + t * ab;
}

Constraint CreateLink(MobilizedBody &chassis_body, Vec3 chassis_link_pos, MobilizedBody &upright_body, Vec3 upright_link_pos, int variant = 1)
{
    Constraint cons;

    auto dist = (chassis_link_pos - upright_link_pos).norm();

    assert(fabs(dist) > 1.0e-3);

    switch (variant)
    {
    case 0:
        cons = Constraint::SphereOnSphereContact(chassis_body, PosWorldToBody(chassis_body, chassis_link_pos), dist / 2.0, upright_body, PosWorldToBody(upright_body, upright_link_pos), dist / 2.0, false);
        break;
    case 1:
        cons = Constraint::Rod(chassis_body, PosWorldToBody(chassis_body, chassis_link_pos), upright_body, PosWorldToBody(upright_body, upright_link_pos), dist);
        break;
    }

    return cons;
}