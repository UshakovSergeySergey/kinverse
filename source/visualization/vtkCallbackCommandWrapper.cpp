#include "stdafx.h"
#include "vtkCallbackCommandWrapper.h"

kinverse::visualization::vtkCallbackCommandWrapper* kinverse::visualization::vtkCallbackCommandWrapper::New() {
  return new vtkCallbackCommandWrapper;
};

void kinverse::visualization::vtkCallbackCommandWrapper::Execute(vtkObject* caller, unsigned long eid, void* callData) {
  if (m_callback)
    m_callback(caller, eid, callData);
}

void kinverse::visualization::vtkCallbackCommandWrapper::SetCallback(std::function<void(vtkObject* caller, unsigned long eid, void* calldata)> callback) {
  m_callback = callback;
}
