#include "stdafx.h"
#include "i_gizmo_impl.h"

void kinverse::visualization::IGizmoImpl::setViewProp(vtkSmartPointer<vtkProp> viewProp) {
  m_viewProp = viewProp;
}

vtkSmartPointer<vtkProp> kinverse::visualization::IGizmoImpl::getViewProp() const {
  return m_viewProp;
}

void kinverse::visualization::IGizmoImpl::setRenderer(vtkSmartPointer<vtkRenderer> renderer) {
  m_renderer = renderer;
}

vtkSmartPointer<vtkRenderer> kinverse::visualization::IGizmoImpl::getRenderer() const {
  return m_renderer;
}
