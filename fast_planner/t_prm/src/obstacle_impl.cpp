/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022,
 *  ETH Zurich - V4RL, Department of Mechanical and Process Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Matthias Busenhart
 *********************************************************************/

#include <tprm/obstacle_impl.h>

namespace tprm {

StaticSphereObstacle::StaticSphereObstacle(const Eigen::Vector3d& center, double radius) : StaticObstacle(center), m_radius(radius) {}

bool StaticSphereObstacle::isColliding(const Vector3d& point) const {
    return (point - m_COM).squaredNorm() <= m_radius * m_radius;
}

bool StaticSphereObstacle::isColliding(const Vector3d& segment_from, const Vector3d& segment_to) const {
    if (isColliding(segment_from))
        return true;
    if (isColliding(segment_to))
        return true;

    const double r2 = m_radius * m_radius;

    // compute the closest point on the segment
    Vector3d p2m1 = segment_to - segment_from;
    Vector3d p1mc = segment_from - m_COM;

    double dot = p1mc.dot(p2m1);
    double squarednorm = p2m1.squaredNorm();

    // clamp down t to the segment
    double t = -1. * (dot / squarednorm);

    if (t < 0.)
        t = 0.;
    if (t > 1.)
        t = 1.;
    Vector3d closest = segment_from + p2m1 * t;
    return ((m_COM - closest).squaredNorm() <= r2);
}

DynamicSphereObstacle::DynamicSphereObstacle(const Vector3d& COM, const Vector3d& velocity, double radius) : DynamicObstacle(COM, velocity), m_radius(radius) {}

bool DynamicSphereObstacle::isColliding(const Vector3d& point, double& hitTimeFrom, double& hitTimeTo) const {
    if (getVelocity().squaredNorm() < std::numeric_limits<double>::epsilon()) {
        if ((m_COM - point).squaredNorm() < m_radius * m_radius) {
            hitTimeFrom = 0.0;
            hitTimeTo = std::numeric_limits<double>::infinity();
            return true;
        } else {
            return false;
        }
    }
    /*
    代码中引入了一个技巧：将时间变量归一化到 [0, 1] 区间。
    定义归一化时间：τ = t / maxTime，其中 t ∈ [0, maxTime]
    直接使用速度 v 时，如果速度很小，v·v 可能接近0，导致数值不稳定
    归一化时间 τ 在 [0, 1] 范围内，便于处理
    */
    const double maxTime = 10000;  // 最大检测时间
    Vector3d d = -getVelocity() * maxTime;  // 障碍物在maxTime内的位移
    Vector3d f = point - m_COM;             // 从障碍物中心到检测点的向量
    double a = d.squaredNorm();
    double b = 2.0 * f.dot(d);
    double c = f.squaredNorm() - m_radius * m_radius;
    double discriminant = b * b - 4.0 * a * c;
    if (discriminant < 0)
        return false;       // 无实数解，无碰撞
    double t1 = (-b - std::sqrt(discriminant)) / (2.0 * a);
    double t2 = (-b + std::sqrt(discriminant)) / (2.0 * a);
    if (t1 < 0 && t2 < 0)
        return false;       // 无实数解，无碰撞
    if (t1 < 0) {
        hitTimeFrom = 0;           // 从现在开始碰撞
        hitTimeTo = t2 * maxTime;  // 到t2*maxTime结束
    }
    else {
        hitTimeFrom = t1 * maxTime;  // 从t1*maxTime开始碰撞
        hitTimeTo = t2 * maxTime;    // 到t2*maxTime结束
    }
    return true;
}

} /* namespace tprm */