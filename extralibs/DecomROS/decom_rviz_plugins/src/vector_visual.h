#ifndef VECTOR_VISUAL_H
#define VECTOR_VISUAL_H

#include <decom_rviz_plugins/data_type.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <rviz/ogre_helpers/arrow.h>

namespace decom_rviz_plugins {
class VectorVisual {
public:
  VectorVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node);
  ~VectorVisual();

  void setMessage(const vec_E<std::pair<Eigen::Matrix<decimal_t, 3, 1>, Eigen::Matrix<decimal_t, 3, 1>>> &vs);
  void setFramePosition(const Ogre::Vector3 &position);
  void setFrameOrientation(const Ogre::Quaternion &orientation);

  void setColor(float r, float g, float b, float a);
  void setScale(float s);

private:
  std::vector<std::unique_ptr<rviz::Arrow>> objs_;

  Ogre::SceneNode *frame_node_;

  Ogre::SceneManager *scene_manager_;

  float s_ = 1.0;
  vec_E<std::pair<Eigen::Matrix<decimal_t, 3, 1>, Eigen::Matrix<decimal_t, 3, 1>>> vs_;
};
}

#endif
