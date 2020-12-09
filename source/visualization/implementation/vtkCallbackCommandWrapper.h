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

namespace kinverse {
  namespace visualization {

    /**
     * @class vtkCallbackCommandWrapper
     * @brief This is a helper class. It allows to set method that needs to be called when some event is emitted.
     * It was designed specially for Text3DGizmo. Text3DGizmo subscribes for camera events and uses this class in order to set event callback.
     * This class is a mediator. Standard VTK callbacks do not allow to use std::function or capturing lambdas,
     * so we designed this class in order to get around this constraint.
     */
    class vtkCallbackCommandWrapper : public vtkCallbackCommand {
     public:
      /**
       * @brief This is a standard VTK method for object instantiation.
       */
      static vtkCallbackCommandWrapper* New();

      /**
       * @brief Overriding standard VTK method so that it initiates @p m_callback call.
       */
      void Execute(vtkObject* caller, unsigned long eid, void* callData) override;

      /**
       * @brief This method sets callback function.
       */
      void SetCustomCallback(std::function<void(vtkObject*, unsigned long, void*)> callback);

     protected:
      /**
       * @brief Constructor is protected, because VTK objects must be created with static @p New method.
       */
      vtkCallbackCommandWrapper() = default;

      /**
       * @brief Method that will be called each time event occurs.
       */
      std::function<void(vtkObject*, unsigned long, void*)> m_callback{};

     public:
      vtkTypeMacro(vtkCallbackCommandWrapper, vtkCommand)
    };

  }  // namespace visualization
}  // namespace kinverse
