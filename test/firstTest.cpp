#include <gtest/gtest.h>
#include <glm/glm.hpp>

TEST(HelloTest, BasicAssertions)
{
    EXPECT_STRNE("hello", "world");
    EXPECT_EQ(7 * 6, 42);
}

TEST(MathLibrary, BasicAssertion)
{
    glm::vec2 test = glm::vec2(1.0f, 1.0f);
    test *= 2.0f;

    EXPECT_EQ(test.x, 2);
    EXPECT_EQ(test.y, 2);
}
