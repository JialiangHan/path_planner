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
        p6(1, -1), p7(-1, 0);

    // float expect_1 = Utility::ConvertDegToRad(-135);
    // float result_1 = Utility::GetAngleBetweenTwoVector(p1, p2, p1, p3);
    // EXPECT_FLOAT_EQ(expect_1, result_1);
    // float expect_2 = (Utility::ConvertDegToRad(45));
    // float result_2 = Utility::GetAngleBetweenTwoVector(p1, p2, p1, p4);
    // EXPECT_FLOAT_EQ(expect_2, result_2);
    // float expect_3 = (Utility::ConvertDegToRad(135));
    // float result_3 = Utility::GetAngleBetweenTwoVector(p1, p2, p1, p5);
    // EXPECT_FLOAT_EQ(expect_3, result_3);
    // float expect_4 = (Utility::ConvertDegToRad(-45));
    // float result_4 = Utility::GetAngleBetweenTwoVector(p1, p2, p1, p6);
    // EXPECT_FLOAT_EQ(expect_4, result_4);
    float expect_1 = Utility::ConvertDegToRad(180);
    float result_1 = Utility::GetAngleBetweenTwoVector(p1, p2, p1, p7);
    EXPECT_FLOAT_EQ(expect_1, result_1);
}
TEST(Utility, GetAngleRangeFromPointToSegment)
{
    Eigen::Vector2f p1(0, 0), p2(-1, 0), p3(-1, 1);

    float expect_1 = Utility::DegToZeroTo2P(Utility::ConvertDegToRad(135));
    float expect_2 = Utility::RadNormalization(Utility::ConvertDegToRad(45));
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
    EXPECT_FALSE(result);
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
    Utility::AngleRange ar1(Utility::ConvertDegToRad(0), Utility::ConvertDegToRad(60)), ar2(Utility::ConvertDegToRad(0), Utility::ConvertDegToRad(7)), ar3(Utility::ConvertDegToRad(60), Utility::ConvertDegToRad(7)), ar4(Utility::ConvertDegToRad(-30), Utility::ConvertDegToRad(40)), ar5(Utility::ConvertDegToRad(120), Utility::ConvertDegToRad(0));

    bool result1 = Utility::IsAngleRangeInclude(ar1, ar2);
    // bool result2 = Utility::IsAngleRangeInclude(ar1, ar3);
    // bool result3 = Utility::IsAngleRangeInclude(ar1, ar4);
    // bool result4 = Utility::IsAngleRangeInclude(ar1, ar5);
    EXPECT_TRUE(result1);
    // EXPECT_TRUE(!result2);
    // EXPECT_TRUE(!result3);
    // EXPECT_TRUE(result4);
}
TEST(Utility, MinusAngleRangeOverlap)
{
    Utility::AngleRange ar1(Utility::ConvertDegToRad(330), Utility::ConvertDegToRad(60)), ar2(Utility::ConvertDegToRad(340), Utility::ConvertDegToRad(60)), ar3(Utility::ConvertDegToRad(350), Utility::ConvertDegToRad(60)), ar4(Utility::ConvertDegToRad(360), Utility::ConvertDegToRad(60));

    Utility::AngleRange result1 = Utility::MinusAngleRangeOverlap(ar1, ar2);
    // Utility::AngleRange result2 = Utility::MinusAngleRangeOverlap(ar1, ar3);
    // Utility::AngleRange result3 = Utility::MinusAngleRangeOverlap(ar1, ar4);
    EXPECT_FLOAT_EQ(result1.first, Utility::ConvertDegToRad(330));
    EXPECT_TRUE(Utility::IsEqual(result1.second, Utility::ConvertDegToRad(10)));
}
TEST(Utility, GetAngle)
{
    HybridAStar::Node2D start, goal;
    start.setX(0);
    start.setY(0);
    goal.setX(0);
    goal.setY(-1);

    float result = Utility::GetAngle(start, goal);

    EXPECT_FLOAT_EQ(result, Utility::ConvertDegToRad(270));
}

TEST(Utility, CreatePolygon)
{
    Eigen::Vector2f center(1, 2);
    float width = 2, height = 4, heading = M_PI / 2;

    Utility::Polygon result = Utility::CreatePolygon(center, width, height, heading);
    // for (const auto &point : result)
    // {
    //     DLOG(INFO) << "point is " << point.x() << " " << point.y();
    // }
    EXPECT_FLOAT_EQ(3, result[0].x());
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

TEST(Utility, CombinePolygon)
{
    Eigen::Vector2f origin1(0, 0), origin2(1, 0), origin3(2, 0);

    Utility::Polygon result = Utility::CombinePolygon(Utility::CreatePolygon(origin1), Utility::CreatePolygon(origin3));
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
    EXPECT_FLOAT_EQ(result, sqrt(2));
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
    Utility::AngleRange ar1(Utility::ConvertDegToRad(333), Utility::ConvertDegToRad(50));
    float angle1 = Utility::ConvertDegToRad(17);

    // Utility::AngleRange ar2(Utility::ConvertDegToRad(330), Utility::ConvertDegToRad(50));
    // float angle2 = Utility::ConvertDegToRad(10);

    EXPECT_TRUE(Utility::IsAngleRangeInclude(ar1, angle1));
    // EXPECT_TRUE(Utility::IsAngleRangeInclude(ar2, angle2));
}

TEST(Utility, FindAngleRange)
{
    // float a1 = Utility::ConvertDegToRad(150);
    // float a2 = Utility::ConvertDegToRad(10);
    float a3 = Utility::ConvertDegToRad(-135);
    float a4 = Utility::ConvertDegToRad(95);
    // Utility::AngleRange out1 = Utility::FindAngleRange(a1, a2);
    // Utility::AngleRange out2 = Utility::FindAngleRange(a2, a3);
    Utility::AngleRange out3 = Utility::FindAngleRange(a3, a4);
    // Utility::AngleRange out4 = Utility::FindAngleRange(a4, a1);

    // EXPECT_FLOAT_EQ(Utility::ConvertRadToDeg(out1.first), 10);
    // EXPECT_FLOAT_EQ(Utility::ConvertRadToDeg(out1.second), 140);
    // EXPECT_FLOAT_EQ(Utility::ConvertRadToDeg(out2.first), -150);
    // EXPECT_FLOAT_EQ(Utility::ConvertRadToDeg(out2.second), 160);
    EXPECT_FLOAT_EQ(Utility::ConvertRadToDeg(out3.first), 95);
    EXPECT_FLOAT_EQ(Utility::ConvertRadToDeg(out3.second), 130);
    // EXPECT_FLOAT_EQ(Utility::ConvertRadToDeg(out4.first), -10);
    // EXPECT_FLOAT_EQ(Utility::ConvertRadToDeg(out4.second), 160);
}

TEST(Utility, IsPointInsideCircle)
{
    Eigen::Vector2f center(0, 0), p1(0.5, 0), p2(1, 0), p3(2, 0);
    float radius = 1;

    EXPECT_TRUE(Utility::IsPointInsideCircle(center, radius, p1));
    EXPECT_FALSE(Utility::IsPointInsideCircle(center, radius, p2));
    EXPECT_TRUE(!Utility::IsPointInsideCircle(center, radius, p3));
}

TEST(Utility, GetUnitVector)
{
    Eigen::Vector2f p1(0, 0), p2(10, 0);

    EXPECT_FLOAT_EQ(Utility::GetUnitVector(p1, p2).x(), 1);
    EXPECT_FLOAT_EQ(Utility::GetUnitVector(p1, p2).y(), 0);
}

TEST(Utility, IsPointOnLine)
{
    Eigen::Vector2f p1(0, 0), p2(0, 1), p3(1, 0), p4(1, -1);

    // EXPECT_FALSE(Utility::IsPointOnLine(p1, p2, p3));
    EXPECT_TRUE(Utility::IsPointOnLine(p4, p3, p2));
}

TEST(Utility, GetDistanceFromPointToLine)
{
    Eigen::Vector2f point(0, 0), start(0, 1), unit_vector(1, 0), start1(1, 1);

    float distance = Utility::GetDistanceFromPointToLine(point, start, unit_vector);
    float distance1 = Utility::GetDistanceFromPointToLine(point, start1, unit_vector);

    EXPECT_FLOAT_EQ(distance, 1);
    EXPECT_FLOAT_EQ(distance1, 1);
}

TEST(Utility, FindProjectionPoint)
{
    Eigen::Vector2f point(0, 0), start(-10, 1), unit_vector(1, 0), projection;

    projection = Utility::FindProjectionPoint(point, start, unit_vector);

    // EXPECT_FLOAT_EQ(projection.x(), 0);
    // EXPECT_FLOAT_EQ(projection.y(), 1);
}

TEST(Utility, GetIntersectionPointsBetweenCircleAndLine)
{
    Eigen::Vector2f center(0, 0), start(0, 5), unit_vector(0, -1);
    float radius = 1;
    std::vector<Eigen::Vector2f> intersections1 = Utility::GetIntersectionPointsBetweenCircleAndLine(center, radius, start, unit_vector);

    // std::vector<Eigen::Vector2f> intersections2 = Utility::GetIntersectionPointsBetweenCircleAndLine(center, radius, center, unit_vector);

    EXPECT_FLOAT_EQ(intersections1[0].x(), 0);
    EXPECT_FLOAT_EQ(intersections1[0].y(), -1);
    EXPECT_FLOAT_EQ(intersections1[1].x(), 0);
    EXPECT_FLOAT_EQ(intersections1[1].y(), 1);
    // EXPECT_FLOAT_EQ(intersections2[0].x(), 1);
    // EXPECT_FLOAT_EQ(intersections2[0].y(), 0);
    // EXPECT_FLOAT_EQ(intersections2[1].x(), -1);
    // EXPECT_FLOAT_EQ(intersections2[1].y(), 0);
}

TEST(Utility, IsEdgeInsideCircle)
{
    Eigen::Vector2f center(0, 0), s1(0, 0), e1(0.5, 0.5), s2(0.5, 0.5), e2(1, 1), s3(1, 1), e3(2, 2);
    float radius = 1;
    int result1 = Utility::IsEdgeInsideCircle(center, radius, s1, e1);

    int result2 = Utility::IsEdgeInsideCircle(center, radius, s2, e2);

    int result3 = Utility::IsEdgeInsideCircle(center, radius, s3, e3);

    EXPECT_EQ(result1, 1);
    EXPECT_EQ(result2, 0);
    EXPECT_EQ(result3, -1);
}

TEST(Utility, IsPolygonInsideCircle)
{
    Eigen::Vector2f center(0, 0), p1(0, 0), p2(0.5, 0), p3(0.5, 0.5), p4(0, 0.5), p5(2, 0), p6(2, 2), p7(0, 2), p8(2, 1), p9(3, 0);

    Utility::Polygon polygon1{p1, p2, p3, p4, p1};
    Utility::Polygon polygon2{p1, p5, p6, p7, p1};
    Utility::Polygon polygon3{p5, p9, p6, p8, p5};
    float radius = 1;

    int result1 = Utility::IsPolygonInsideCircle(center, radius, polygon1);

    int result2 = Utility::IsPolygonInsideCircle(center, radius, polygon2);

    int result3 = Utility::IsPolygonInsideCircle(center, radius, polygon3);

    EXPECT_EQ(result1, 1);
    EXPECT_EQ(result2, 0);
    EXPECT_EQ(result3, -1);
}

TEST(Utility, GetIntersectionPointsBetweenCircleAndSegment)
{
    Eigen::Vector2f center(0, 0), p1(0, 0.5), p2(0.5, 0), p3(0, 0), p4(2, 0), p5(0, 5), p6(0, -5);

    float radius = 1;

    std::vector<Eigen::Vector2f> result1;
    std::vector<Eigen::Vector2f> result2;
    std::vector<Eigen::Vector2f> result3;

    // result1 = Utility::GetIntersectionPointsBetweenCircleAndSegment(center, radius, p1, p2);

    // result2 = Utility::GetIntersectionPointsBetweenCircleAndSegment(center, radius, p3, p4);

    result3 = Utility::GetIntersectionPointsBetweenCircleAndSegment(center, radius, p5, p6);

    // EXPECT_EQ(result1.size(), 0);
    // EXPECT_EQ(result2.size(), 1);
    EXPECT_EQ(result3.size(), 2);

    // EXPECT_EQ(result2[0].x(), 1);
    // EXPECT_EQ(result2[0].y(), 0);

    EXPECT_EQ(result3[0].x(), 0);
    EXPECT_EQ(result3[0].y(), -1);
    if (result3.size() > 1)
    {
        EXPECT_EQ(result3[1].x(), 0);
        EXPECT_EQ(result3[1].y(), 1);
    }
}

TEST(Utility, GetAngleRangeFromPointToEdgeAtRadius)
{
    Eigen::Vector2f center(0, 0), p1(0, 0.5), p2(0.5, 0), p3(0, 0), p4(2, 0), p5(0, 5), p6(0, -5);

    float radius = 1;

    Utility::AngleRange result1;
    Utility::AngleRange result2;
    Utility::AngleRange result3;

    result1 = Utility::GetAngleRangeFromPointToEdgeAtRadius(center, radius, p1, p2);
    result2 = Utility::GetAngleRangeFromPointToEdgeAtRadius(center, radius, p3, p4);
    result3 = Utility::GetAngleRangeFromPointToEdgeAtRadius(center, radius, p5, p6);
    EXPECT_EQ(result1.first, 0);
    EXPECT_FLOAT_EQ(result1.second, Utility::ConvertDegToRad(90));
    EXPECT_EQ(result2.first, 0);
    EXPECT_EQ(result2.second, 0);
    EXPECT_FLOAT_EQ(result3.first, Utility::ConvertDegToRad(90));
    EXPECT_FLOAT_EQ(result3.second, Utility::ConvertDegToRad(180));
}

TEST(Utility, CombineAngleRange)
{
    Utility::AngleRange ar1(Utility::ConvertDegToRad(0), Utility::ConvertDegToRad(30)), ar2(Utility::ConvertDegToRad(45), Utility::ConvertDegToRad(45)), ar3(Utility::ConvertDegToRad(0), Utility::ConvertDegToRad(45));

    Utility::AngleRange result1;
    Utility::AngleRange result2;
    Utility::AngleRange result3;

    // result1 = Utility::CombineAngleRange(ar1, ar2);
    result2 = Utility::CombineAngleRange(ar1, ar3);
    result3 = Utility::CombineAngleRange(ar2, ar3);
    // EXPECT_EQ(result1.first, -1);
    // EXPECT_FLOAT_EQ(result1.second, -1);
    EXPECT_EQ(result2.first, 0);
    EXPECT_EQ(result2.second, Utility::ConvertDegToRad(45));
    EXPECT_FLOAT_EQ(result3.first, Utility::ConvertDegToRad(0));
    EXPECT_FLOAT_EQ(result3.second, Utility::ConvertDegToRad(90));
}

TEST(Utility, GetAngleRangeFromPointToPolygonAtRadius)
{
    Eigen::Vector2f center(0, 0), p1(0, 0.5);
    float radius = 1;
    Utility::Polygon polygon1 = Utility::CreatePolygon(p1);
    Utility::AngleRange result1;

    result1 = Utility::GetAngleRangeFromPointToPolygonAtRadius(center, radius, polygon1);

    EXPECT_FLOAT_EQ(result1.first, Utility::ConvertDegToRad(30));
    EXPECT_FLOAT_EQ(result1.second, Utility::ConvertDegToRad(60));
}

TEST(Utility, FindCommonAngleRange)
{
    Utility::AngleRange ar1(Utility::ConvertDegToRad(0), Utility::ConvertDegToRad(50)), ar2(Utility::ConvertDegToRad(45), Utility::ConvertDegToRad(45)), ar3(Utility::ConvertDegToRad(60), Utility::ConvertDegToRad(40));

    Utility::AngleRange result1;
    Utility::AngleRange result2;
    Utility::AngleRange result3;

    result1 = Utility::FindCommonAngleRange(ar1, ar3);
    result2 = Utility::FindCommonAngleRange(ar1, ar2);
    result3 = Utility::FindCommonAngleRange(ar2, ar3);
    EXPECT_EQ(result1.first, -1);
    EXPECT_FLOAT_EQ(result1.second, -1);
    EXPECT_FLOAT_EQ(result2.first, Utility::ConvertDegToRad(45));
    EXPECT_TRUE(Utility::IsEqual(result2.second, Utility::ConvertDegToRad(5)));
    EXPECT_FLOAT_EQ(result3.first, Utility::ConvertDegToRad(60));
    EXPECT_FLOAT_EQ(result3.second, Utility::ConvertDegToRad(30));
}

TEST(Utility, MinusAngleRange)
{
    Utility::AngleRange ar1(Utility::ConvertDegToRad(0), Utility::ConvertDegToRad(90)), ar2(Utility::ConvertDegToRad(100), Utility::ConvertDegToRad(60)), ar3(Utility::ConvertDegToRad(60), Utility::ConvertDegToRad(60)), ar4(Utility::ConvertDegToRad(30), Utility::ConvertDegToRad(30));
    // not overlap and not include
    std::vector<Utility::AngleRange> result1 = Utility::MinusAngleRange(ar1, ar2);
    // overlap
    std::vector<Utility::AngleRange> result2 = Utility::MinusAngleRange(ar1, ar3);
    // include
    std::vector<Utility::AngleRange> result3 = Utility::MinusAngleRange(ar1, ar4);
    EXPECT_FLOAT_EQ(result1.size(), 0);
    EXPECT_FLOAT_EQ(result2[0].first, Utility::ConvertDegToRad(0));
    EXPECT_FLOAT_EQ(result2[0].second, Utility::ConvertDegToRad(60));
    EXPECT_FLOAT_EQ(result3[0].first, Utility::ConvertDegToRad(0));
    EXPECT_FLOAT_EQ(result3[0].second, Utility::ConvertDegToRad(30));
    EXPECT_FLOAT_EQ(result3[1].first, Utility::ConvertDegToRad(60));
    EXPECT_FLOAT_EQ(result3[1].second, Utility::ConvertDegToRad(30));
}

TEST(Utility, GetAngleDistance)
{
    float a1 = Utility::ConvertDegToRad(-40), a2 = Utility::ConvertDegToRad(180), a3 = Utility::ConvertDegToRad(360);

    Utility::AngleRange ar1(Utility::ConvertDegToRad(30), Utility::ConvertDegToRad(30));

    float result5 = Utility::GetAngleDistance(a1, ar1);
    float result6 = Utility::GetAngleDistance(a2, ar1);
    float result7 = Utility::GetAngleDistance(a3, ar1);
    EXPECT_FLOAT_EQ(result5, Utility::ConvertDegToRad(70));
    EXPECT_FLOAT_EQ(result6, Utility::ConvertDegToRad(120));
    EXPECT_FLOAT_EQ(result7, Utility::ConvertDegToRad(30));

    // float result1 = Utility::GetAngleDistance(a1, a2);
    // float result2 = Utility::GetAngleDistance(a2, a1);
    // float result3 = Utility::GetAngleDistance(a1, a3);
    // float result4 = Utility::GetAngleDistance(a3, a1);
    // EXPECT_FLOAT_EQ(result1, Utility::ConvertDegToRad(30));
    // EXPECT_FLOAT_EQ(result2, Utility::ConvertDegToRad(-30));
    // EXPECT_TRUE(Utility::IsEqual(result3, Utility::ConvertDegToRad(-30)));
    // EXPECT_TRUE(Utility::IsEqual(result4, Utility::ConvertDegToRad(30)));
}