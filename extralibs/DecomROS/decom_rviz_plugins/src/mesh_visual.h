#ifndef MESH_VISUAL_H
#define MESH_VISUAL_H

#include <decom_rviz_plugins/polyhedron.h>
#include <Eigen/Eigenvalues>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/mesh_shape.h>

namespace decom_rviz_plugins {
  class MeshVisual {
    public:
      MeshVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node);

      virtual ~MeshVisual();

      void setMessage(const vec_E<std::vector<Eigen::Matrix<decimal_t, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 3, 1>>>> &bds);
      void setFramePosition(const Ogre::Vector3 &position);
      void setFrameOrientation(const Ogre::Quaternion &orientation);

      void setColor(float r, float g, float b, float a);

    private:
      std::unique_ptr<rviz::MeshShape> obj_;

      Ogre::SceneNode *frame_node_;

      Ogre::SceneManager *scene_manager_;
  };
}

#endif
