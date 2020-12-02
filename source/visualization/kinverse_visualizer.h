#pragma once

#include "exports.h"
#include "i_gizmo.h"

namespace kinverse {
  namespace visualization {

    /**
     * @class KinverseVisualizerImpl
     * @brief Forward declaration of internal class. This class encapsulates VTK library, so that when using @p visualization module users don't have to link
     * VTK libraries.
     */
    class KinverseVisualizerImpl;

    /**
     * @class KinverseVisualizer
     * @brief This class is responsible for rendering gizmos. Just create instance of this class, add some gizmos, and that's it - you have interactive window
     * with rendered objects. This class works in two modes.
     *
     * When built with constructor without parameters, it creates everything related to rendering and event processing behind the scene,
     * It also starts parallel thread for event processing.
     *
     * You can also use parameterized constructor in order to easily integrate Qt/VTK.
     */
    class KINVERSE_VISUALIZATION_API KinverseVisualizer {
     public:
      /**
       * @brief Smart pointer to @p KinverseVisualizer
       */
      using Ptr = std::shared_ptr<KinverseVisualizer>;

      /**
       * @brief Smart pointer to const @p KinverseVisualizer
       */
      using ConstPtr = std::shared_ptr<const KinverseVisualizer>;

      /**
       * @brief Constructor without parameters creates everything it needs in order to render gizmos.
       * Behind the scenes, it creates window for rendering and starts a parallel thread for event processing.
       */
      explicit KinverseVisualizer();

      /**
       * @brief Use this constructor in order to integrate @p KinverseVisualizer with Qt/VTK.
       * This constructor expects that user provides totally initialized render window and renderer VTK objects.
       * It doesn't start an event processing loop, so it is up to user to process events.
       * @param[in] renderWindow - VTK render window object (vtkSmartPointer<vtkRenderer>* is cast to void* in order to get rid of VTK dependency)
       * @param[in] renderer - VTK renderer object (vtkSmartPointer<vtkRenderWindow>* is cast to void* in order to get rid of VTK dependency)
       */
      KinverseVisualizer(void* renderWindow, void* renderer);

      /**
       * @brief This destructor stops and joins event processing thread if instance was created with constructor without parameters.
       */
      ~KinverseVisualizer();

      /**
       * @brief This method adds @p IGizmo object to visualizer. You can create any gizmo (MeshGizmo, CoordinateFrameGizmo, etc) and simply add it to @p
       * KinverseVisualizer, visualizer will take care of rendering and updating everything.
       */
      void addGizmo(IGizmo::Ptr gizmo);

     private:
      /**
       * @brief This method runs in parallel thread and is responsible for event processing when instance is created with constructor without parameters.
       */
      void processEvents();

      /**
       * @brief This member variable implements 'pointer to implementation' idiom in order to encapsulate VTK dependency
       */
      std::shared_ptr<KinverseVisualizerImpl> m_pImpl{ nullptr };

      /**
       * @brief Stores array of @p IGizmo objects added to visualizer.
       */
      std::vector<IGizmo::Ptr> m_gizmos{};

      /**
       * @brief Stores event processing thread handle.
       */
      std::thread m_eventProcessingThread;
    };

  }  // namespace visualization
}  // namespace kinverse
