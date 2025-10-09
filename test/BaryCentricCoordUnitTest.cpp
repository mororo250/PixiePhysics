#include <glm/vec3.hpp>
#include <gtest/gtest.h>
#include <tuple>

#include "../src/physics/lib/BarycentricCoord.hpp"
#include "../src/physics/utils/Constatns.hpp"

using namespace PixiePhysics;

// --- Tests for GetBaryCentricCoord on a Line ---

TEST(BarycentricCoordTests, Line_OriginProjectsOntoP0_Infinite)
{
    glm::vec3 p0(1.0f, 0.0f, 0.0f);
    glm::vec3 p1(3.0f, 0.0f, 0.0f);
    glm::vec3 target(0.0f);

    auto [success, u, v] = GetBaryCentricCoord(p0, p1, target);

    EXPECT_TRUE(success);
    EXPECT_NEAR(u, 1.5f, EPSILON);
    EXPECT_NEAR(v, -0.5f, EPSILON);
}

TEST(BarycentricCoordTests, Line_OriginProjectsOntoP1_Infinite)
{
    glm::vec3 p0(-3.0f, 0.0f, 0.0f);
    glm::vec3 p1(-1.0f, 0.0f, 0.0f);
    glm::vec3 target(0.0f);

    auto [success, u, v] = GetBaryCentricCoord(p0, p1, target);

    EXPECT_TRUE(success);
    EXPECT_NEAR(u, -0.5f, EPSILON);
    EXPECT_NEAR(v, 1.5f, EPSILON);
}

TEST(BarycentricCoordTests, Line_OriginProjectsOntoMidpoint)
{
    glm::vec3 p0(-1.0f, 0.0f, 0.0f);
    glm::vec3 p1(1.0f, 0.0f, 0.0f);
    glm::vec3 target(0.0f);

    auto [success, u, v] = GetBaryCentricCoord(p0, p1, target);

    EXPECT_TRUE(success);
    EXPECT_NEAR(u, 0.5f, EPSILON);
    EXPECT_NEAR(v, 0.5f, EPSILON);
}

TEST(BarycentricCoordTests, Line_OriginProjectsOntoSegment)
{
    glm::vec3 p0(-1.0f, 0.0f, 0.0f);
    glm::vec3 p1(2.0f, 0.0f, 0.0f);
    glm::vec3 target(0.0f);

    auto [success, u, v] = GetBaryCentricCoord(p0, p1, target);

    EXPECT_TRUE(success);
    EXPECT_NEAR(u, 2.0f / 3.0f, EPSILON);
    EXPECT_NEAR(v, 1.0f / 3.0f, EPSILON);
}

TEST(BarycentricCoordTests, Line_OriginIsP0)
{
    glm::vec3 p0(0.0f, 0.0f, 0.0f);
    glm::vec3 p1(2.0f, 0.0f, 0.0f);
    glm::vec3 target(0.0f);

    auto [success, u, v] = GetBaryCentricCoord(p0, p1, target);

    EXPECT_TRUE(success);
    EXPECT_NEAR(u, 1.0f, EPSILON);
    EXPECT_NEAR(v, 0.0f, EPSILON);
}

TEST(BarycentricCoordTests, Line_OriginIsP1)
{
    glm::vec3 p0(-2.0f, 0.0f, 0.0f);
    glm::vec3 p1(0.0f, 0.0f, 0.0f);
    glm::vec3 target(0.0f);

    auto [success, u, v] = GetBaryCentricCoord(p0, p1, target);

    EXPECT_TRUE(success);
    EXPECT_NEAR(u, 0.0f, EPSILON);
    EXPECT_NEAR(v, 1.0f, EPSILON);
}

// --- Tests for GetBaryCentricCoord on a Triangle ---

TEST(BarycentricCoordTests, Triangle_OriginProjectsOntoP0_InfinitePlane)
{
    glm::vec3 p0(1.0f, 1.0f, 0.0f);
    glm::vec3 p1(3.0f, 1.0f, 0.0f);
    glm::vec3 p2(1.0f, 3.0f, 0.0f);
    glm::vec3 target(0.0f);

    auto [success, u, v, w] = GetBaryCentricCoord(p0, p1, p2, target);

    EXPECT_TRUE(success);
    EXPECT_NEAR(u, 2.0f, EPSILON);
    EXPECT_NEAR(v, -0.5f, EPSILON);
    EXPECT_NEAR(w, -0.5f, EPSILON);
}

TEST(BarycentricCoordTests, Triangle_OriginProjectsOntoP1_InfinitePlane)
{
    glm::vec3 p0(3.0f, 0.0f, 0.0f);
    glm::vec3 p1(1.0f, 1.0f, 0.0f);
    glm::vec3 p2(3.0f, 2.0f, 0.0f);
    glm::vec3 target(0.0f);

    auto [success, u, v, w] = GetBaryCentricCoord(p0, p1, p2, target);

    EXPECT_TRUE(success);
    EXPECT_NEAR(u, 0.25f, EPSILON);
    EXPECT_NEAR(v, 1.5f, EPSILON);
    EXPECT_NEAR(w, -0.75f, EPSILON);
}

TEST(BarycentricCoordTests, Triangle_OriginProjectsOntoP2_InfinitePlane)
{
    glm::vec3 p0(3.0f, 1.0f, 0.0f);
    glm::vec3 p1(1.0f, 1.0f, 0.0f);
    glm::vec3 p2(1.0f, -1.0f, 0.0f);
    glm::vec3 target(0.0f);

    auto [success, u, v, w] = GetBaryCentricCoord(p0, p1, p2, target);

    EXPECT_TRUE(success);
    EXPECT_NEAR(u, -0.5f, EPSILON);
    EXPECT_NEAR(v, 1.0f, EPSILON);
    EXPECT_NEAR(w, 0.5f, EPSILON);
}

TEST(BarycentricCoordTests, Triangle_OriginProjectsOntoEdgeP0P1_InfinitePlane)
{
    glm::vec3 p0(-1.0f, -1.0f, 0.0f);
    glm::vec3 p1(1.0f, -1.0f, 0.0f);
    glm::vec3 p2(0.0f, 1.0f, 0.0f);
    glm::vec3 target(0.0f);
    auto [success, u, v, w] = GetBaryCentricCoord(p0, p1, p2, target);

    EXPECT_TRUE(success);
    EXPECT_NEAR(u, 0.25f, EPSILON);
    EXPECT_NEAR(v, 0.25f, EPSILON);
    EXPECT_NEAR(w, 0.5f, EPSILON);
}

TEST(BarycentricCoordTests, Triangle_OriginProjectsOntoEdgeP2P0_InfinitePlane)
{
    glm::vec3 p0(-1.0f, 1.0f, 0.0f);
    glm::vec3 p1(1.0f, -1.0f, 0.0f);
    glm::vec3 p2(-1.0f, -1.0f, 0.0f);
    glm::vec3 target(0.0f);

    auto [success, u, v, w] = GetBaryCentricCoord(p0, p1, p2, target);

    EXPECT_TRUE(success);
    EXPECT_NEAR(u, 0.5f, EPSILON);
    EXPECT_NEAR(v, 0.5f, EPSILON);
    EXPECT_NEAR(w, 0.0f, EPSILON);
}


TEST(BarycentricCoordTests, Triangle_OriginProjectsInside_Centroid)
{
    glm::vec3 p0(1.0f, -1.0f, 0.0f);
    glm::vec3 p1(-1.0f, -1.0f, 0.0f);
    glm::vec3 p2(0.0f, 2.0f, 0.0f);
    glm::vec3 target(0.0f);

    auto [success, u, v, w] = GetBaryCentricCoord(p0, p1, p2, target);

    EXPECT_TRUE(success);
    EXPECT_NEAR(u, 1.0f / 3.0f, EPSILON);
    EXPECT_NEAR(v, 1.0f / 3.0f, EPSILON);
    EXPECT_NEAR(w, 1.0f / 3.0f, EPSILON);
}

TEST(BarycentricCoordTests, Triangle_OriginIsP0)
{
    // Origin is exactly vertex P0
    glm::vec3 p0(0.0f, 0.0f, 0.0f);
    glm::vec3 p1(2.0f, 0.0f, 0.0f);
    glm::vec3 p2(0.0f, 2.0f, 0.0f);
    glm::vec3 target(0.0f);

    auto [success, u, v, w] = GetBaryCentricCoord(p0, p1, p2, target);

    EXPECT_TRUE(success);
    EXPECT_NEAR(u, 1.0f, EPSILON);
    EXPECT_NEAR(v, 0.0f, EPSILON);
    EXPECT_NEAR(w, 0.0f, EPSILON);
}