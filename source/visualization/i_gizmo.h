#pragma once

#include "exports.h"

/**
 * @todo We need to get rid of a raw pointer to parent, because I don't like raw pointers. But we cant use smart pointer here either, because even if we
 * inherit @p enable_shared_from_this it won't work, as object must be fully constructed before calling @p shared_from_this method
 */

/**
 * @todo We need to make @p show, @p hide and @p update methods private, because users must not have access to these methods. They are a part of implementation.
 */

/**
 * @todo Gizmos are not thread safe yet! Changing something (e.g. MeshGizmo transform) from another thread will result in undefined behaviour.
 */

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

      /**
       * @brief This method adds all objects in the gizmo and its children to the rendering pipeline
       * @param[in] renderer - vtk renderer object (vtkSmartPointer<vtkRenderer>* is cast to void* in order to get rid of VTK dependency)
       */
      virtual void show(void* renderer);

      /**
       * @brief This method removes all objects in the gizmo and its children from the rendering pipeline
       * @param[in] renderer - vtk renderer object (vtkSmartPointer<vtkRenderer>* is cast to void* in order to get rid of VTK dependency)
       */
      virtual void hide(void* renderer);

      /**
       * @brief This method initiates a render call if gizmo is orphan. If gizmo has parent then this method does nothing.
       */
      void update();

     protected:
      /**
       * @brief This member variable implements 'pointer to implementation' idiom in order to encapsulate VTK dependency
       */
      std::shared_ptr<IGizmoImpl> m_pImpl{ nullptr };

      /**
       * @brief Pointer to parent gizmo
       */
      const IGizmo* m_parentGizmo{ nullptr };
    };

  }  // namespace visualization
}  // namespace kinverse
