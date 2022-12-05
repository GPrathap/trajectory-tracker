#include <traj_common/param_passer.h>

using std::cout;
using std::endl;
namespace hagen_planner
{
ParamPasser::ParamPasser(ros::NodeHandle& nh)
{
  node = nh;
}

bool ParamPasser::passing_matrix(std::string param_name, Eigen::MatrixXd& mat){
  mat.setZero();
  XmlRpc::XmlRpcValue processNoiseCovarConfig;
  if (node.hasParam(param_name))
  {
    try
    {
      node.getParam(param_name, processNoiseCovarConfig);
      ROS_ASSERT(processNoiseCovarConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);
      int matSize = mat.rows();
      for (int i = 0; i < matSize; i++)
      {
        for (int j = 0; j < matSize; j++)
        {
          try
          {
            std::ostringstream ostr;
            ostr << processNoiseCovarConfig[matSize * i + j];
            std::istringstream istr(ostr.str());
            istr >> mat(i, j);
          }
          catch(XmlRpc::XmlRpcException &e)
          {
            throw e;
          }
          catch(...)
          {
            throw;
          }
        }
      }
    }
    catch (XmlRpc::XmlRpcException &e)
    {
      ROS_ERROR_STREAM("ERROR reading config: " << e.getMessage() << " for "<< param_name << " (type: " << processNoiseCovarConfig.getType() << ")");
    }
    return true;
  }
  cout<< "Param: " << param_name << "not found " << endl;
  return false;
}

bool ParamPasser::passing_vector(std::string param_name, Eigen::MatrixXd& mat){
  mat.setZero();
  XmlRpc::XmlRpcValue processNoiseCovarConfig;
  if (node.hasParam(param_name))
  {
    try
    {
      node.getParam(param_name, processNoiseCovarConfig);
      ROS_ASSERT(processNoiseCovarConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);
      int matSize = mat.rows();
      for (int i = 0; i <mat.rows(); i++)
      {
        for (int j = 0; j < mat.cols(); j++)
        {
          try
          {
            std::ostringstream ostr;
            ostr << processNoiseCovarConfig[matSize * i + j];
            std::istringstream istr(ostr.str());
            istr >> mat(i, j);
          }
          catch(XmlRpc::XmlRpcException &e)
          {
            throw e;
          }
          catch(...)
          {
            throw;
          }
        }
      }
    }
    catch (XmlRpc::XmlRpcException &e)
    {
      ROS_ERROR_STREAM("ERROR reading config: " << e.getMessage() << " for "<< param_name << " (type: " << processNoiseCovarConfig.getType() << ")");
    }
    return true;
  }
  cout<< "Param: " << param_name << "not found " << endl;
  return false;
}

}  // namespace hagen_planner