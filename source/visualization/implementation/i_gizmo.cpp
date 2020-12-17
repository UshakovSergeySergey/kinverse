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

#include "stdafx.h"
#include "../include/kinverse/visualization/i_gizmo.h"
#include "i_gizmo_impl.h"

kinverse::visualization::IGizmo::IGizmo(const IGizmo* parentGizmo) {
  m_pImpl = std::make_shared<IGizmoImpl>();
  m_parentGizmo = parentGizmo;
}

void kinverse::visualization::IGizmo::show(void* renderer) {
  addToRenderingPipeline(renderer, true);
  render();
}

void kinverse::visualization::IGizmo::hide(void* renderer) {
  addToRenderingPipeline(renderer, false);
  render();
}

void kinverse::visualization::IGizmo::addToRenderingPipeline(void* renderer_, bool addToPipeline) const {
  if (!m_pImpl->getRenderer()) {
    if (renderer_ == nullptr)
      return;

    const auto renderer = *static_cast<vtkSmartPointer<vtkRenderer>*>(renderer_);
    m_pImpl->setRenderer(renderer);

    if (!m_pImpl->getRenderer())
      throw std::invalid_argument("addToRenderingPipeline() failed! Renderer is nullptr! It is an internal error, there is nothing you can do about it!");
  }

  if (!m_pImpl->getViewProp())
    return;

  const auto renderer = m_pImpl->getRenderer();
  const auto viewProp = m_pImpl->getViewProp();

  const bool actorIsAdded = renderer->HasViewProp(viewProp);

  if (addToPipeline && !actorIsAdded)
    renderer->AddActor(viewProp);

  if (!addToPipeline && actorIsAdded)
    renderer->RemoveActor(viewProp);
}

void kinverse::visualization::IGizmo::render() const {
  if (m_parentGizmo)
    return;

  const auto renderer = m_pImpl->getRenderer();
  if (!renderer || !renderer->GetRenderWindow())
    return;

  renderer->GetRenderWindow()->Render();
}

void kinverse::visualization::IGizmo::show(const Ptr& gizmo, void* renderer) {
  gizmo->show(renderer);
}

void kinverse::visualization::IGizmo::hide(const Ptr& gizmo, void* renderer) {
  gizmo->hide(renderer);
}
