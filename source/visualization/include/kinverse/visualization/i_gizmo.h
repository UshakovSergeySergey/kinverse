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

namespace kinverse {
  namespace visualization {

    /**
     * @brief Color type is simply a tuple of (red, green, blue, alpha)
     */
    using Color = std::tuple<unsigned char, unsigned char, unsigned char, unsigned char>;

    /**
     * @class IGizmoImpl
     * @brief Forward declaration of internal class. This class encapsulates VTK library, so that when using @p visualization module users don't have to link
     * VTK libraries.
     */
    class IGizmoImpl;

    /**
     * @class IGizmo
     * @brief This is a base class for all gizmos. In the context of @p KinverseVisualizer gizmo is any object that can be rendered. It may be a simple cube
     * mesh, or even compound object consisting of several child gizmos (e.g. @p RobotGizmo). @p visualization module has several @p IGizmo implementations
     * related to the inverse kinematics domain (@p JointGizmo, @p RobotGizmo, @p CoordinateFrameGizmo, etc.)
     */
    class KINVERSE_VISUALIZATION_API IGizmo {
     public:
      /**
       * @brief Smart pointer to @p IGizmo
       */
      using Ptr = std::shared_ptr<IGizmo>;

      /**
       * @brief Smart pointer to const @p IGizmo
       */
      using ConstPtr = std::shared_ptr<const IGizmo>;

      /**
       * @brief Simple constructor, initializes member variables.
       * If parent gizmo is nullptr, then, whenever gizmo is updated, it initiates render call in order to update itself on the screen.
       * If gizmo consists of several child gizmos, it becomes too expensive to initiate render call each time something changes.
       * Imagine that we have a @p RobotGizmo that consist of several @p RevoluteJointGizmo's. It would be better to update all of child gizmos and then
       * initiate a single render call, rather than initiate render call after updating every single child.
       * So if the parent is not nullptr, then gizmo wont initiate render call if it state has changed.
       * @param[in] parentGizmo - parent gizmo
       */
      explicit IGizmo(const IGizmo* parentGizmo);

      /**
       * @brief Default virtual destructor.
       */
      virtual ~IGizmo() = default;

     protected:
      /**
       * @brief This method adds all objects in the gizmo and its children to the rendering pipeline
       * @param[in] renderer - vtk renderer object (vtkSmartPointer<vtkRenderer>* is cast to void* in order to get rid of VTK dependency)
       */
      virtual void show(void* renderer = nullptr);

      /**
       * @brief This method removes all objects in the gizmo and its children from the rendering pipeline
       * @param[in] renderer - vtk renderer object (vtkSmartPointer<vtkRenderer>* is cast to void* in order to get rid of VTK dependency)
       */
      virtual void hide(void* renderer = nullptr);

      /**
       * @brief Helper method that adds or removes gizmo to rendering pipeline.
       * @param[in] renderer - vtk renderer object (vtkSmartPointer<vtkRenderer>* is cast to void* in order to get rid of VTK dependency)
       * @param[in] addToPipeline - if set to true object will be added to pipeline, otherwise it will be removed from rendering pipeline
       */
      void addToRenderingPipeline(void* renderer, bool addToPipeline) const;

      /**
       * @brief Each gizmo can have child gizmos, so parent gizmo must have some way to show children.
       * But parent can't access child's show method because it is protected.
       * So in order to get around this constraint, we introduce this static method that simply calls @p show method of the given gizmo.
       *
       * May be it is a dirty hack, and an indicator of bad architecture design.
       * But it doesn't bother me, as soon as it allows to encapsulate implementation details
       * and make show/hide methods unavailable for ordinary users of this class.
       *
       * @param[in] gizmo - gizmo which we want to show
       * @param[in] renderer - vtk renderer object (vtkSmartPointer<vtkRenderer>* is cast to void* in order to get rid of VTK dependency)
       */
      static void show(const Ptr& gizmo, void* renderer = nullptr);

      /**
       * @brief Each gizmo can have child gizmos, so parent gizmo must have some way to hide children.
       * But parent can't access child's hide method because it is protected.
       * So in order to get around this constraint, we introduce this static method that simply calls @p hide method of the given gizmo.
       *
       * May be it is a dirty hack, and an indicator of bad architecture design.
       * But it doesn't bother me, as soon as it allows to encapsulate implementation details
       * and make show/hide methods unavailable for ordinary users of this class.
       *
       * @param[in] gizmo - gizmo which we want to hide
       * @param[in] renderer - vtk renderer object (vtkSmartPointer<vtkRenderer>* is cast to void* in order to get rid of VTK dependency)
       */
      static void hide(const Ptr& gizmo, void* renderer = nullptr);

      /**
       * @brief This method initiates a render call if gizmo is orphan. If gizmo has a parent then this method does nothing.
       */
      void render() const;

      /**
       * @brief This member variable implements 'pointer to implementation' idiom in order to encapsulate VTK dependency
       */
      std::shared_ptr<IGizmoImpl> m_pImpl{ nullptr };

      /**
       * @brief Pointer to parent gizmo
       */
      const IGizmo* m_parentGizmo{ nullptr };

      /**
       * @brief KinverseVisualizer must have access to some private methods of IGizmo in order to visualize it
       */
      friend class KinverseVisualizer;
    };

  }  // namespace visualization
}  // namespace kinverse
