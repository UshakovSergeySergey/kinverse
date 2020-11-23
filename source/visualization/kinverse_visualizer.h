#pragma once

#include "exports.h"
#include "i_gizmo.h"

namespace kinverse {
  namespace visualization {

    class KinverseVisualizerImpl;

    class KINVERSE_VISUALIZATION_API KinverseVisualizer {
     public:
      using Ptr = std::shared_ptr<KinverseVisualizer>;
      using ConstPtr = std::shared_ptr<const KinverseVisualizer>;

      // starts event processing loop in parallel thread
      explicit KinverseVisualizer();

      // it is up to user to start event processing loop
      KinverseVisualizer(void* renderWindow, void* renderer);

      ~KinverseVisualizer();
      void addGizmo(IGizmo::Ptr gizmo);

     private:
      void processEvents();

      std::shared_ptr<KinverseVisualizerImpl> m_pImpl{ nullptr };
      std::vector<IGizmo::Ptr> m_gizmos{};
      std::thread m_eventProcessingThread;
    };

  }  // namespace visualization
}  // namespace kinverse
