#pragma once

namespace kinverse {
  namespace visualization {

    class vtkCallbackCommandWrapper : public vtkCallbackCommand {
     public:
      vtkTypeMacro(vtkCallbackCommandWrapper, vtkCommand);
      static vtkCallbackCommandWrapper* New();
      void Execute(vtkObject* caller, unsigned long eid, void* callData) override;
      void SetCallback(std::function<void(vtkObject* caller, unsigned long eid, void* calldata)>);

     protected:
      vtkCallbackCommandWrapper() = default;
      ~vtkCallbackCommandWrapper() override = default;
      std::function<void(vtkObject* caller, unsigned long eid, void* calldata)> m_callback{};
    };

  }  // namespace visualization
}  // namespace kinverse
