#include <glm/fwd.hpp>
#include <glm/vec3.hpp>

#include <gtest/gtest.h>

#include "../src/physics/components/Transform.hpp"
#include "../src/physics/components/ShapeSphere.hpp"
#include "../src/physics/components/ShapeBox.hpp"
#include "../src/physics/lib/Gjk.hpp"

using namespace PixiePhysics;


TEST(SphereSupportFunction, SphereSupportPoint)
{
    ShapeSphere sphere{1.0f};
    Transform transform{glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f)};
    glm::vec3 direction(1.0f, 0.0f, 0.0f);

    glm::vec3 result = SupportFunction<ShapeSphere>::get(sphere, transform, direction);
    EXPECT_EQ(result, glm::vec3(1.0f, 0.0f, 0.0f));
    EXPECT_NEAR(result.x, 1.0f, EPSILON);
    EXPECT_NEAR(result.y, 0.0f, EPSILON);
    EXPECT_NEAR(result.z, 0.0f, EPSILON);
}

TEST(BoxSupportFunction, BoxSupportPointAxisAlligned)
{
    ShapeBox box{glm::vec3(1.0f)};
    Transform transform{glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f)};
    glm::vec3 direction(1.0f, 0.0f, 0.0f);

    glm::vec3 result = SupportFunction<ShapeBox>::get(box, transform, direction);
    EXPECT_NEAR(result.x, 1.0f, EPSILON);
    EXPECT_NEAR(result.y, 1.0f, EPSILON);
    EXPECT_NEAR(result.z, 1.0f, EPSILON);
}

TEST(BoxSupportFunction, BoxSupportPointDiagonalDir)
{
    ShapeBox box{glm::vec3(1.0f)};
    Transform transform{glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f)};
    glm::vec3 direction(-1.0f, 0.5, -0.2f);

    glm::vec3 result = SupportFunction<ShapeBox>::get(box, transform, direction);
    EXPECT_NEAR(result.x, -1.0f, EPSILON);
    EXPECT_NEAR(result.y, 1.0f, EPSILON);
    EXPECT_NEAR(result.z, -1.0f, EPSILON);

}

TEST(BoxSupportFunction, BoxSupportPointTransletedBox)
{
    ShapeBox box{glm::vec3(1.0f)};
    Transform transform{glm::vec3(2.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f)};
    glm::vec3 direction(-1.0f, 0.5, -0.2f);

    glm::vec3 result = SupportFunction<ShapeBox>::get(box, transform, direction);
    EXPECT_NEAR(result.x, 1.0f, EPSILON);
    EXPECT_NEAR(result.y, 3.0f, EPSILON);
    EXPECT_NEAR(result.z, 1.0f, EPSILON);
}

TEST(BoxSupportFunction, BoxSupportPointRotatedBox)
{
    ShapeBox box{glm::vec3(1.0f)};
    Transform transform{glm::vec3(0.0f), glm::quat(0.0f, 0.0f, 0.7071068f, 0.7071068f)};
    glm::vec3 direction(1.0f, 0.0f, 0.0f);

    glm::vec3 result = SupportFunction<ShapeBox>::get(box, transform, direction);
    EXPECT_NEAR(result.x, 1.0f, EPSILON);
    EXPECT_NEAR(result.y, 1.0f, EPSILON);
    EXPECT_NEAR(result.z, 1.0f, EPSILON);
}

TEST(ClosestPointTests, ClosestPointOnLine_OnLine)
{
    glm::vec3 a(0.0f, 0.0f, 0.0f);
    glm::vec3 b(2.0f, 0.0f, 0.0f);
    glm::vec3 point(1.0f, 0.0f, 0.0f);

    auto [result, feature] = FindClosestPointOnLine(a, b, point);
    EXPECT_NEAR(result.x, 1.0f, EPSILON);
    EXPECT_NEAR(result.y, 0.0f, EPSILON);
    EXPECT_NEAR(result.z, 0.0f, EPSILON);
    EXPECT_EQ(feature, EnumLineClosestFeature::AB);
}

TEST(ClosestPointTests, ClosestPointOnLine_OffLine)
{
    glm::vec3 a(0.0f, 0.0f, 0.0f);
    glm::vec3 b(2.0f, 0.0f, 0.0f);
    glm::vec3 point(1.0f, 1.0f, 0.0f);

    auto [result, feature] = FindClosestPointOnLine(a, b, point);
    EXPECT_NEAR(result.x, 1.0f, EPSILON);
    EXPECT_NEAR(result.y, 0.0f, EPSILON);
    EXPECT_NEAR(result.z, 0.0f, EPSILON);
    EXPECT_EQ(feature, EnumLineClosestFeature::AB);
}

TEST(ClosestPointTests, ClosestPointOnLine_BeforeStart)
{
    glm::vec3 a(1.0f, 0.0f, 0.0f);
    glm::vec3 b(3.0f, 0.0f, 0.0f);
    glm::vec3 point(0.0f, 0.0f, 0.0f);

    auto [result, feature] = FindClosestPointOnLine(a, b, point);
    EXPECT_NEAR(result.x, 1.0f, EPSILON);
    EXPECT_NEAR(result.y, 0.0f, EPSILON);
    EXPECT_NEAR(result.z, 0.0f, EPSILON);
    EXPECT_EQ(feature, EnumLineClosestFeature::A);
}

TEST(ClosestPointTests, ClosestPointOnLine_AfterEnd)
{
    glm::vec3 a(0.0f, 0.0f, 0.0f);
    glm::vec3 b(2.0f, 0.0f, 0.0f);
    glm::vec3 point(3.0f, 0.0f, 0.0f);

    auto [result, feature] = FindClosestPointOnLine(a, b, point);
    EXPECT_NEAR(result.x, 2.0f, EPSILON);
    EXPECT_NEAR(result.y, 0.0f, EPSILON);
    EXPECT_NEAR(result.z, 0.0f, EPSILON);
    EXPECT_EQ(feature, EnumLineClosestFeature::B);
}

TEST(ClosestPointTests, ClosestPointOnSurface_OnSurface)
{
    glm::vec3 vec1(1.0f, 0.0f, 0.0f);
    glm::vec3 vec2(0.0f, 1.0f, 0.0f);
    glm::vec3 point(0.5f, 0.5f, 0.0f);

    glm::vec3 result = FindClosestPointOnSurface(vec1, vec2, point);
    EXPECT_NEAR(result.x, 0.5f, EPSILON);
    EXPECT_NEAR(result.y, 0.5f, EPSILON);
    EXPECT_NEAR(result.z, 0.0f, EPSILON);
}

TEST(ClosestPointTests, ClosestPointOnSurface_OffSurface)
{
    glm::vec3 vec1(1.0f, 0.0f, 0.0f);
    glm::vec3 vec2(0.0f, 1.0f, 0.0f);
    glm::vec3 point(0.5f, 0.5f, 1.0f);

    glm::vec3 result = FindClosestPointOnSurface(vec1, vec2, point);
    EXPECT_NEAR(result.x, 0.5f, EPSILON);
    EXPECT_NEAR(result.y, 0.5f, EPSILON);
    EXPECT_NEAR(result.z, 0.0f, EPSILON);
}


TEST(ClosestPointTests, ClosestPointOnTriangle_InsideTriangle)
{
    glm::vec3 a(0.0f, 0.0f, 0.0f);
    glm::vec3 b(1.0f, 0.0f, 0.0f);
    glm::vec3 c(0.0f, 1.0f, 0.0f);
    glm::vec3 point(0.33f, 0.33f, 0.0f);

    auto [result, feature] = FindClosestPointOnTriangle(a, b, c, point);
    EXPECT_NEAR(result.x, 0.33f, EPSILON);
    EXPECT_NEAR(result.y, 0.33f, EPSILON);
    EXPECT_NEAR(result.z, 0.0f, EPSILON);
    EXPECT_EQ(feature, EnumTriangleClosestFeature::ABC);
}

TEST(ClosestPointTests, ClosestPointOnTriangle_OnEdge)
{
    glm::vec3 a(0.0f, 0.0f, 0.0f);
    glm::vec3 b(1.0f, 0.0f, 0.0f);
    glm::vec3 c(0.0f, 1.0f, 0.0f);
    glm::vec3 point(0.5f, 0.0f, 0.1f);

    auto [result, feature] = FindClosestPointOnTriangle(a, b, c, point);
    EXPECT_NEAR(result.x, 0.5f, EPSILON);
    EXPECT_NEAR(result.y, 0.0f, EPSILON);
    EXPECT_NEAR(result.z, 0.0f, EPSILON);
    EXPECT_EQ(feature, EnumTriangleClosestFeature::AB);
}

TEST(ClosestPointTests, ClosestPointOnTriangle_OutsideTriangle)
{
    glm::vec3 a(0.0f, 0.0f, 0.0f);
    glm::vec3 b(1.0f, 0.0f, 0.0f);
    glm::vec3 c(0.0f, 1.0f, 0.0f);
    glm::vec3 point(-0.5f, -0.5f, 0.0f);

    auto [result, feature] = FindClosestPointOnTriangle(a, b, c, point);
    EXPECT_NEAR(result.x, 0.0f, EPSILON);
    EXPECT_NEAR(result.y, 0.0f, EPSILON);
    EXPECT_NEAR(result.z, 0.0f, EPSILON);
    EXPECT_EQ(feature, EnumTriangleClosestFeature::A);
}

TEST(ClosestPointTests, ClosestPointOnTriangle_OutsideClosestToCA)
{
    glm::vec3 a(0.0f, 0.0f, 0.0f);
    glm::vec3 b(2.0f, 0.0f, 0.0f);
    glm::vec3 c(0.0f, 2.0f, 0.0f);
    glm::vec3 point(-1.0f, 1.0f, 0.0f);

    auto [result, feature] = FindClosestPointOnTriangle(a, b, c, point);
    EXPECT_NEAR(result.x, 0.0f, EPSILON);
    EXPECT_NEAR(result.y, 1.0f, EPSILON);
    EXPECT_NEAR(result.z, 0.0f, EPSILON);
    EXPECT_EQ(feature, EnumTriangleClosestFeature::AC);
}


TEST(ClosestPointTests, ClosestPointOnTetrahedron_InsideTetrahedron)
{
    glm::vec3 a(0.0f, 0.0f, 0.0f);
    glm::vec3 b(1.0f, 0.0f, 0.0f);
    glm::vec3 c(0.0f, 1.0f, 0.0f);
    glm::vec3 d(0.0f, 0.0f, 1.0f);
    glm::vec3 point(0.25f, 0.25f, 0.25f);

    auto [result, feature] = FindClosestPointOnTetrahedron(a, b, c, d, point);
    EXPECT_NEAR(result.x, 0.25f, EPSILON);
    EXPECT_NEAR(result.y, 0.25f, EPSILON);
    EXPECT_NEAR(result.z, 0.25f, EPSILON);
    EXPECT_EQ(feature, EnumTetrahedronClosestFeature::ABCD);
}

TEST(ClosestPointTests, ClosestPointOnTetrahedron_OnFace)
{
    glm::vec3 a(0.0f, 0.0f, 0.0f);
    glm::vec3 b(1.0f, 0.0f, 0.0f);
    glm::vec3 c(0.0f, 1.0f, 0.0f);
    glm::vec3 d(0.0f, 0.0f, 1.0f);
    glm::vec3 point(0.33f, 0.33f, 0.0f);

    auto [result, feature] = FindClosestPointOnTetrahedron(a, b, c, d, point);
    EXPECT_NEAR(result.x, 0.33f, EPSILON);
    EXPECT_NEAR(result.y, 0.33f, EPSILON);
    EXPECT_NEAR(result.z, 0.0f, EPSILON);
    EXPECT_EQ(feature, EnumTetrahedronClosestFeature::ABC);
}

TEST(ClosestPointTests, ClosestPointOnTetrahedron_OutsideTetrahedron)
{
    glm::vec3 a(0.0f, 0.0f, 0.0f);
    glm::vec3 b(1.0f, 0.0f, 0.0f);
    glm::vec3 c(0.0f, 1.0f, 0.0f);
    glm::vec3 d(0.0f, 0.0f, 1.0f);
    glm::vec3 point(2.0f, 2.0f, 2.0f);

    auto [result, feature] = FindClosestPointOnTetrahedron(a, b, c, d, point);

    EXPECT_NEAR(result.x, 1.0f/3.0f, EPSILON);
    EXPECT_NEAR(result.y, 1.0f/3.0f, EPSILON);
    EXPECT_NEAR(result.z, 1.0f/3.0f, EPSILON);
    EXPECT_EQ(feature, EnumTetrahedronClosestFeature::BDC);
}


TEST(GJK, ClosestPointsSphereBox)
{
    ShapeSphere sphere{1.0f};
    Transform transformSphere{glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f)};
    ShapeBox box{glm::vec3(1.0f)};
    Transform transformBox{glm::vec3(3.0f, 0.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f)};

    auto [spherePoint, boxPoint] = GetClosestPoints<ShapeSphere, ShapeBox>(sphere, transformSphere, box, transformBox);
    EXPECT_NEAR(spherePoint.x, 1.0f, EPSILON);
    EXPECT_NEAR(spherePoint.y, 0.0f, EPSILON);
    EXPECT_NEAR(spherePoint.z, 0.0f, EPSILON);

    EXPECT_NEAR(boxPoint.x, 2.0f, EPSILON);
    EXPECT_NEAR(boxPoint.y, 0.0f, EPSILON);
    EXPECT_NEAR(boxPoint.z, 0.0f, EPSILON);
}
