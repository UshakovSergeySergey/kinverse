#pragma once

#include "exports.h"

namespace kinverse {
  namespace core {

    class KINVERSE_CORE_API JointConstraints {
     public:
      JointConstraints(double maximumSpeed, double minimumAxisValue, double maximumAxisValue);

      void setMaximumSpeed(double maximumSpeed);
      double getMaximumSpeed() const;

      void setMinimumAxisValue(double minimumAxisValue);
      double getMinimumAxisValue() const;

      void setMaximumAxisValue(double maximumAxisValue);
      double getMaximumAxisValue() const;

      bool violatesRangeConstraint(double axisValue) const;
      double clampAxisValue(double axisValue) const;

     private:
      double m_maximumSpeed{ 0.0 };
      double m_minimumAxisValue{ 0.0 };
      double m_maximumAxisValue{ 0.0 };
    };

  }  // namespace core
}  // namespace kinverse
