#ifndef BOUND_VISUAL_H
#define BOUND_VISUAL_H

#include <decom_rviz_plugins/data_type.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/billboard_line.h>


namespace decom_rviz_plugins {
class BoundVisual {
public:
  BoundVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node);
  ~BoundVisual();

  void setMessage(const vec_E<std::vector<Eigen::Matrix<decimal_t, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<decimal_t, 3, 1>>>> &bds);
  void setFramePosition(const Ogre::Vector3 &position);
  void setFrameOrientation(const Ogre::Quaternion &orientation);

  void setColor(float r, float g, float b, float a);
  void setScale(float s);

private:
  std::vector<std::unique_ptr<rviz::BillboardLine>> objs_;

  Ogre::SceneNode *frame_node_;

  Ogre::SceneManager *scene_manager_;
};
}

#endif
