/**
 * @file test.cpp
 * @author Jialiang Han (hanjiali@umich.edu)
 * @brief for test
 * @version 0.1
 * @date 2021-12-26
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "gtest/gtest.h"
#include "utility.h"
// #include "lookup_table.h"

TEST(Utility, DegNormalization)
{
    float t1 = 361, t2 = -359, t3 = 721, t4 = -719;
    std::vector<float> one{t1, t2, t3, t4};
    int expect, result;
    for (const auto &number : one)
    {
        expect = 1.0;
        result = Utility::DegNormalization(number);
        EXPECT_FLOAT_EQ(expect, result);
    }
}
TEST(Utility, RadNormalization)
{
    float t1 = 361, t2 = -359, t3 = 721, t4 = -719;
    std::vector<float> one{t1, t2, t3, t4};
    int expect, result;
    for (const auto &number : one)
    {
        expect = Utility::ConvertDegToRad(1.0);
        result = Utility::RadNormalization(Utility::ConvertDegToRad(number));
        EXPECT_FLOAT_EQ(expect, result);
    }
}
TEST(Utility, IsIntersect)
{
    Eigen::Vector2f p1(0, 0), p2(60, 0), p3(60, 30), p4(0, 30), p5(23.686, 10.8453);
    std::vector<Eigen::Vector2f> polygon{p1, p2, p3, p4, p1};
    std::vector<Eigen::Vector2f> intersect{p5};
    // std::vector<Eigen::Vector2f> outside{p5, p7, p8};
    int expect, result;
    for (const auto &point : intersect)
    {
        expect = 1;
        Eigen::Vector2f p9(10000, point.y());
        result = Utility::IsIntersect(point, p9, polygon[1], polygon[2]);
        EXPECT_EQ(expect, result);
    }
}

TEST(Utility, FindIntersectionPoint)
{
    Eigen::Vector2f p1(0, 0), p2(1, 1), p3(0, 1), p4(1, 0);
    Eigen::Vector2f expect(1, 1);
    Eigen::Vector2f result = Utility::FindIntersectionPoint(p1, p2, p3, p2);
    EXPECT_EQ(expect, result);
}

TEST(Utility, OnSegment)
{
    Eigen::Vector2f p1(-1, 0), p2(0, 0), p3(0, 10), p4(-1, 10), p5(0, 5), p6(0.5, 0), p7(-10, 1), p8(10, 1);
    std::vector<Eigen::Vector2f> polygon{p1, p2, p3, p4, p1};
    std::vector<Eigen::Vector2f> outside{p5};
    bool expect = true, result;
    result = Utility::OnSegment(p2, p3, p5);
    EXPECT_EQ(expect, result);
    // for (const auto &point : outside)
    // {
    //     expect = true;
    //     for (uint i = 0; i < polygon.size() - 1; ++i)
    //     {
    //         result = Utility::OnSegment(polygon[i], polygon[i + 1], point);
    //         EXPECT_EQ(expect, result);
    //     }
    // }
}

TEST(Utility, IsAboveSegment)
{
    Eigen::Vector2f p1(0, 0), p2(1, 1), p3(0, 1), p4(1, 0);
    int expect = 1;
    int result = Utility::IsAboveSegment(p1, p2, p3);
    EXPECT_EQ(expect, result);
    expect = 0;
    result = Utility::IsAboveSegment(p1, p2, p4);
    EXPECT_EQ(expect, result);
}

TEST(Utility, IsInsidePolygon)
{
    Eigen::Vector2f p1(0, 0), p2(60, 0), p3(60, 30), p4(0, 30), p5(23.686, 10.8453);
    std::vector<Eigen::Vector2f> polygon{p1, p2, p3, p4, p1};
    std::vector<Eigen::Vector2f> inside{p5};
    // std::vector<Eigen::Vector2f> outside{p5, p7, p8};
    int expect, result;
    for (const auto &point : inside)
    {
        expect = 1;
        result = Utility::IsInsidePolygon(polygon, point);
        EXPECT_EQ(expect, result);
    }
}
TEST(Utility, GetAngleBetweenTwoVector)
{
    Eigen::Vector2f p1(0, 0), p2(1, 0), p3(-1, -1), p4(1, 1), p5(-1, 1),
        p6(1, -1);

    float expect_1 = Utility::ConvertDegToRad(-135);
    float result_1 = Utility::GetAngleBetweenTwoVector(p1, p2, p1, p3);
    EXPECT_FLOAT_EQ(expect_1, result_1);
    float expect_2 = (Utility::ConvertDegToRad(45));
    float result_2 = Utility::GetAngleBetweenTwoVector(p1, p2, p1, p4);
    EXPECT_FLOAT_EQ(expect_2, result_2);
    float expect_3 = (Utility::ConvertDegToRad(135));
    float result_3 = Utility::GetAngleBetweenTwoVector(p1, p2, p1, p5);
    EXPECT_FLOAT_EQ(expect_3, result_3);
    float expect_4 = (Utility::ConvertDegToRad(-45));
    float result_4 = Utility::GetAngleBetweenTwoVector(p1, p2, p1, p6);
    EXPECT_FLOAT_EQ(expect_4, result_4);
}
TEST(Utility, GetAngleRangeFromPointToSegment)
{
    Eigen::Vector2f p1(0, 0), p2(-1, 0), p3(-1, 1);

    float expect_1 = Utility::DegToZeroTo2P(Utility::ConvertDegToRad(180));
    float expect_2 = Utility::RadNormalization(Utility::ConvertDegToRad(-45));
    Utility::AngleRange result =
        Utility::GetAngleRangeFromPointToSegment(p2, p3, p1);
    EXPECT_FLOAT_EQ(expect_1, result.first);
    EXPECT_FLOAT_EQ(expect_2, result.second);
}
TEST(Utility, GetAngleRangeFromPointToPolygon)
{
    Eigen::Vector2f p1(0, 0), origin(-2, 0);
    Utility::Polygon polygon = Utility::CreatePolygon(origin);
    float expect_range = Utility::RadNormalization(Utility::ConvertDegToRad(45));
    float expect_start_angle = Utility::RadToZeroTo2P(Utility::ConvertDegToRad(135));
    Utility::AngleRange result =
        Utility::GetAngleRangeFromPointToPolygon(polygon, p1);
    EXPECT_FLOAT_EQ(expect_range, result.second);
    EXPECT_FLOAT_EQ(expect_start_angle, result.first);
}

TEST(Utility, IsOverlap)
{
    Utility::AngleRange first, second;
    first = std::make_pair(0.059347, 6.69515);
    second = std::make_pair(6.7545, 8.24555);
    bool result = Utility::IsOverlap(first, second);
    // Utility::AngleRange first1 = std::make_pair(, 6.69515);
    // Utility::AngleRange second1= std::make_pair(6.7545, 8.24555);
    // bool result = Utility::IsOverlap(first1, second1);
    EXPECT_TRUE(result);
}

TEST(Utility, FindVisibleVertexFromNode)
{
    Eigen::Vector2f p1(0, 0), p2(1, 0), p3(2, 0), p4(2, 1), p5(1, 1), origin(1, 0);
    Utility::Polygon polygon = Utility::CreatePolygon(origin);

    std::vector<Eigen::Vector2f> result = Utility::FindVisibleVertexFromNode(polygon, p1);
    EXPECT_EQ(result.size(), 2);
    EXPECT_EQ(result[1].x(), p5.x());
}

TEST(Utility, GetDistanceFromPolygonToPoint)
{
    Eigen::Vector2f p1(37, 15), p2(38, 16);
    Utility::Polygon polygon = Utility::CreatePolygon(p2);

    float result = Utility::GetDistanceFromPolygonToPoint(polygon, p1);
    EXPECT_FLOAT_EQ(result, sqrt(2));
}

TEST(Utility, GetDistanceFromSegmentToPoint)
{
    Eigen::Vector2f p1(37, 15), p2(38, 16), p3(38, 17);

    float result = Utility::GetDistanceFromSegmentToPoint(p2, p3, p1);
    EXPECT_FLOAT_EQ(result, sqrt(2));
}
TEST(Utility, IsAngleRangeInclude)
{
    Utility::AngleRange ar1(Utility::ConvertDegToRad(330), Utility::ConvertDegToRad(60)), ar2(Utility::ConvertDegToRad(351), Utility::ConvertDegToRad(7)), ar3(Utility::ConvertDegToRad(2), Utility::ConvertDegToRad(7)), ar4(Utility::ConvertDegToRad(-30), Utility::ConvertDegToRad(40));

    bool result1 = Utility::IsAngleRangeInclude(ar1, ar2);
    bool result2 = Utility::IsAngleRangeInclude(ar1, ar3);
    bool result3 = Utility::IsAngleRangeInclude(ar1, ar4);
    EXPECT_TRUE(result1);
    EXPECT_TRUE(result2);
    EXPECT_TRUE(result3);
}
TEST(Utility, MinusAngleRange)
{
    Utility::AngleRange ar1(Utility::ConvertDegToRad(330), Utility::ConvertDegToRad(60)), ar2(Utility::ConvertDegToRad(340), Utility::ConvertDegToRad(60)), ar3(Utility::ConvertDegToRad(350), Utility::ConvertDegToRad(60)), ar4(Utility::ConvertDegToRad(360), Utility::ConvertDegToRad(60));

    Utility::AngleRange result1 = Utility::MinusAngleRange(ar1, ar2);
    // Utility::AngleRange result2 = Utility::MinusAngleRange(ar1, ar3);
    // Utility::AngleRange result3 = Utility::MinusAngleRange(ar1, ar4);
    EXPECT_FLOAT_EQ(result1.first, Utility::ConvertDegToRad(330));
    EXPECT_FLOAT_EQ(result1.second, Utility::ConvertDegToRad(10));
}
TEST(Utility, GetAngle)
{
    HybridAStar::Node2D start, goal;
    start.setX(3.77815);
    start.setY(22.1007);
    goal.setX(38.1625);
    goal.setY(26.6218);

    float result = Utility::GetAngle(start, goal);

    EXPECT_FLOAT_EQ(result, Utility::ConvertDegToRad(45));
}

TEST(Utility, CreatePolygon)
{
    Eigen::Vector2f center(1, 2);
    float width = 2, height = 4, heading = M_PI / 2;

    Utility::Polygon result = Utility::CreatePolygon(center, width, height, heading);
    for (const auto &point : result)
    {
        DLOG(INFO) << "point is " << point.x() << " " << point.y();
    }
    EXPECT_FLOAT_EQ(2, result[0].x());
}

TEST(Utility, IsPolygonIntersectWithPolygon)
{
    Eigen::Vector2f origin1(0, 0), origin2(2, 0);
    Utility::Polygon polygon1, polygon2;
    polygon1 = Utility::CreatePolygon(origin1);
    polygon2 = Utility::CreatePolygon(origin2);
    bool result = Utility::IsPolygonIntersectWithPolygon(polygon1, polygon2);

    EXPECT_TRUE(!result);
}
TEST(Utility, GetDistanceFromPolygonToSegment)
{
    Eigen::Vector2f origin1(0, 0), start(2, 0), end(3, 0);
    Utility::Polygon polygon1;
    polygon1 = Utility::CreatePolygon(origin1);
    float result = Utility::GetDistanceFromPolygonToSegment(polygon1, start, end);
    float expect = 1;

    EXPECT_FLOAT_EQ(result, expect);
}

TEST(Utility, CombinePolyon)
{
    Eigen::Vector2f origin1(0, 0), origin2(1, 0), origin3(2, 0);

    Utility::Polygon result = Utility::CombinePolyon(Utility::CreatePolygon(origin1), Utility::CreatePolygon(origin3));
    EXPECT_FLOAT_EQ(result.size(), 0);
    // EXPECT_FLOAT_EQ(result[0].x(), 0);
    // EXPECT_FLOAT_EQ(result[1].x(), 2);
    // EXPECT_FLOAT_EQ(result[2].x(), 2);
    // EXPECT_FLOAT_EQ(result[3].x(), 0);
    // EXPECT_FLOAT_EQ(result[4].x(), 0);
    // EXPECT_FLOAT_EQ(result[0].y(), 0);
    // EXPECT_FLOAT_EQ(result[1].y(), 0);
    // EXPECT_FLOAT_EQ(result[2].y(), 1);
    // EXPECT_FLOAT_EQ(result[3].y(), 1);
    // EXPECT_FLOAT_EQ(result[4].y(), 0);
}

TEST(Utility, GetDistanceFromPolygonToPointAtAngle)
{
    Eigen::Vector2f point(0, 0), origin(1, 0);
    float angle = Utility::ConvertDegToRad(45);

    float result = Utility::GetDistanceFromPolygonToPointAtAngle(Utility::CreatePolygon(origin), point, angle);
    EXPECT_FLOAT_EQ(result, 1);
}

TEST(Utility, IsSegmentIntersectWithPolygon)
{
    Eigen::Vector2f start(0, 0), end(10, 0), origin(0, 0);
    Utility::Polygon polygon = Utility::CreatePolygon(origin);

    int expect = 1;
    int result = Utility::IsSegmentIntersectWithPolygon(polygon, start, end);
    EXPECT_EQ(expect, result);
}

TEST(Utility, IsAngleRangeIncludeAngle)
{
    Utility::AngleRange ar1(Utility::ConvertDegToRad(150), Utility::ConvertDegToRad(50));
    float angle1 = Utility::ConvertDegToRad(180);

    Utility::AngleRange ar2(Utility::ConvertDegToRad(330), Utility::ConvertDegToRad(50));
    float angle2 = Utility::ConvertDegToRad(10);

    EXPECT_TRUE(Utility::IsAngleRangeInclude(ar1, angle1));
    EXPECT_TRUE(Utility::IsAngleRangeInclude(ar2, angle2));
}