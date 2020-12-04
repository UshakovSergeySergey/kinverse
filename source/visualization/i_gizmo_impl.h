/*
 * Software License Agreement (BSD License)
 *
 * kinverse - www.kinverse.org
 *
 * Copyright (c) 2020, Sergey Ushakov
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the <organization> nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * <COPYRIGHT HOLDER> BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *
 *   * Neither the name of the copyright holder(s) nor the names of its
 *
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 */

#pragma once

namespace kinverse {
  namespace visualization {

    /**
     * @class IGizmoImpl
     * @brief This class is designed in order to unbind dependency from VTK library.
     * It allows user to use VTK rendering pipeline without need to link against large number of VTK libraries.
     */
    class IGizmoImpl {
     public:
      /**
       * @brief Sets the object that will actually be rendered by VTK rendering pipeline.
       * @param[in] viewProp - object for rendering
       */
      void setViewProp(vtkSmartPointer<vtkProp> viewProp);

      /**
       * @brief Returns object that is rendered by VTK rendering pipeline.
       */
      vtkSmartPointer<vtkProp> getViewProp() const;

      /**
       * @brief Sets VTK renderer object, the one which actually does all the work related to rendering.
       * @param[in] renderer - VTK renderer
       */
      void setRenderer(vtkSmartPointer<vtkRenderer> renderer);

      /**
       * @brief Returns VTK renderer object
       */
      vtkSmartPointer<vtkRenderer> getRenderer() const;

     private:
      /**
       * @brief This is the object that is rendered by VTK rendering pipeline.
       */
      vtkSmartPointer<vtkProp> m_viewProp{ nullptr };

      /**
       * @brief Holds the object that will do actual rendering.
       */
      vtkSmartPointer<vtkRenderer> m_renderer{ nullptr };
    };

  }  // namespace visualization
}  // namespace kinverse
