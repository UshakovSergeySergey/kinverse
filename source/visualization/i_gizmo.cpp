#include "stdafx.h"
#include "i_gizmo.h"
#include "i_gizmo_impl.h"

kinverse::visualization::IGizmo::IGizmo(const IGizmo* parentGizmo) {
  m_pImpl = std::make_shared<IGizmoImpl>();
  m_parentGizmo = parentGizmo;
}

void kinverse::visualization::IGizmo::show(void* renderer) {
  addToRenderingPipeline(renderer, true);
}

void kinverse::visualization::IGizmo::hide(void* renderer) {
  addToRenderingPipeline(renderer, false);
}

void kinverse::visualization::IGizmo::addToRenderingPipeline(void* renderer_, bool addToPipeline) {
  if (!m_pImpl->getRenderer()) {
    if (renderer_ == nullptr)
      throw std::invalid_argument(
          "addToRenderingPipeline() failed! Received nullptr instead of a valid renderer! It is an internal error, there is nothing you can do about it!");

    const auto renderer = *(reinterpret_cast<vtkSmartPointer<vtkRenderer>*>(renderer_));
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

void kinverse::visualization::IGizmo::render() {
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
