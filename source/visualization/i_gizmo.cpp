#include "stdafx.h"
#include "i_gizmo.h"
#include "i_gizmo_impl.h"

kinverse::visualization::IGizmo::IGizmo(const IGizmo* parentGizmo) {
  m_pImpl = std::make_shared<IGizmoImpl>();
  m_parentGizmo = parentGizmo;
}

void kinverse::visualization::IGizmo::draw(void* renderer_) {
  const vtkSmartPointer<vtkRenderer> renderer = *(reinterpret_cast<vtkSmartPointer<vtkRenderer>*>(renderer_));
  m_pImpl->m_renderer = renderer;
  // we need to check if actor is already added
  m_pImpl->m_renderer->AddActor(m_pImpl->m_actor);
}

void kinverse::visualization::IGizmo::hide(void* renderer) {
}

void kinverse::visualization::IGizmo::update() {
  if (m_parentGizmo)
    return;
  std::cout << "needs to be rerendered" << std::endl;
  if (m_pImpl->m_renderer)
    m_pImpl->m_renderer->GetRenderWindow()->Render();
}
