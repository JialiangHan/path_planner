#include "computational_geometry.h"

namespace ComputationalGeometry
{
    float CrossProduct(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2)
    {
        float out;
        out = p1.x() * p2.y() - p2.x() * p1.y();
        // DLOG(INFO) << "result is " << out;
        return out;
    }
    Segment::Segment(const Eigen::Vector2f &start, const float &radius, const float &rad)
    {
        start_ = start;
        Eigen::Vector2f rotation_matrix(std::cos(rad), std::sin(rad));
        end_ = start_ + radius * rotation_matrix;
        CalculateLength();
    }

    bool Segment::IsIntersect(const Segment &another_segment) const
    {
        if ((start_.x() == end_.x() && end_.x() == another_segment.GetStart().x() && another_segment.GetStart().x() == another_segment.GetEnd().x()) || (start_.y() == end_.y() && end_.y() == another_segment.GetStart().y() && another_segment.GetStart().y() == another_segment.GetEnd().y()))
        {
            // these segments are have same coordinate. maybe they are overlapped.
            return false;
        }
        if (std::max(another_segment.GetStart().x(), another_segment.GetEnd().x()) < std::min(start_.x(), end_.x()) ||
            std::max(another_segment.GetStart().y(), another_segment.GetEnd().y()) < std::min(start_.y(), end_.y()) ||
            std::max(start_.x(), end_.x()) < std::min(another_segment.GetStart().x(), another_segment.GetEnd().x()) ||
            std::max(start_.y(), end_.y()) < std::min(another_segment.GetStart().y(), another_segment.GetEnd().y()))
        {
            // DLOG(INFO) << "these two segments are far away!";
            return false;
        }
        else
        {
            if (CrossProduct(start_ - another_segment.GetStart(), another_segment.GetEnd() - another_segment.GetStart()) * CrossProduct(end_ - another_segment.GetStart(), another_segment.GetEnd() - another_segment.GetStart()) <= 0 &&
                CrossProduct(another_segment.GetStart() - end_, start_ - end_) * CrossProduct(another_segment.GetEnd() - end_, start_ - end_) <= 0)
            {
                // DLOG(INFO) << "Intersection!!";
                return true;
            }
            return false;
        }
    }
    Eigen::Vector2f Segment::FindIntersectionPoint(const Segment &another_segment) const
    {
        Eigen::Vector2f out;
        if (0 == IsIntersect(another_segment))
        {
            out.x() = 10000;
            out.y() = 10000;
        }
        else
        {
            Eigen::Vector2f dir_1 = end_ - start_, dir_2 = another_segment.GetEnd() - another_segment.GetStart();
            float t;
            t = CrossProduct(another_segment.GetStart() - start_, dir_2) / CrossProduct(dir_1, dir_2);
            out = start_ + t * dir_1;
        }
        return out;
    }

    bool Segment::OnSegment(const Eigen::Vector2f &point) const
    {
        if (point.x() <= std::max(start_.x(), end_.x()) &&
            point.x() >= std::min(start_.x(), end_.x()) &&
            point.y() <= std::max(start_.y(), end_.y()) &&
            point.y() >= std::min(start_.y(), end_.y()))
        {
            Eigen::Vector2f start_point = point - start_;
            Eigen::Vector2f start_end_ = end_ - start_;
            if (abs(start_point.dot(start_end_) / start_point.norm() / start_end_.norm() - 1) < 1e-6)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }
}