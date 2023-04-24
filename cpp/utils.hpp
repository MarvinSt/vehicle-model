
#include "Simbody.h"
#include "json.hpp"

#include <string>
#include <fstream>

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

Rotation FromDirectionVector(Vec3 direction, CoordinateAxis axis = ZAxis)
{
    return Rotation().setRotationFromOneAxis(UnitVec3(direction), axis);
}

Rotation FromDirectionVector(Vec3 direction_a, Vec3 direction_b)
{
    return Rotation().setRotationFromTwoAxes(UnitVec3(direction_a), ZAxis, UnitVec3(direction_b), ZAxis);
}

Transform TransformWorldToBody(MobilizedBody body, Vec3 position, Vec3 direction, CoordinateAxis axis = ZAxis, bool invert = false)
{
    auto transform = GetGlobalTransform(body);

    // calculate the original direction vector
    auto orig_direction = position - transform.p();

    std::cout << transform.p() << std::endl;

    auto rotation = Rotation().setRotationFromTwoAxes(UnitVec3(direction), axis, UnitVec3(orig_direction), axis); // Rotation().setRotationFromOneAxis(UnitVec3(direction), axis);

    auto trans = (transform.invert() * Transform(rotation.invert(), position));
    return trans;
}