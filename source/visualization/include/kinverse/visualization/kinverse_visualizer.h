/**
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, Sergey Ushakov
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
       * @param[in] gizmo - gizmo to add to visualizer and rendering pipeline
       */
      void addGizmo(IGizmo::Ptr gizmo);

      /**
       * @brief This method removes @p IGizmo object from visualizer. It also removes all objects related to this gizmo from the rendering pipeline.
       * @param[in] gizmo - gizmo to remove from visualizer and rendering pipeline
       */
      void removeGizmo(IGizmo::Ptr gizmo);

     private:
      /**
       * @brief This method runs in parallel thread and is responsible for event processing when instance is created with constructor without parameters.
       */
      void processEvents() const;

      /**
       * @brief This is a helper method that checks if the given gizmo is already added to visualizer.
       * @param[in] gizmo - gizmo to check
       * @return Returns true if gizmo is already added to visualizer, false otherwise.
       */
      bool gizmoIsAdded(IGizmo::Ptr gizmo) const;

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
      std::thread m_eventProcessingThread{};
    };

  }  // namespace visualization
}  // namespace kinverse
