#ifndef torch_tf_h
#define torch_tf_h

extern "C" {
#include <TH/TH.h>
}

#include <boost/shared_ptr.hpp>
#include <tf/tf.h>

#define TFIMP(return_type, class_name, name) extern "C" return_type TH_CONCAT_4(tf_, class_name, _, name)

typedef boost::shared_ptr<std::vector<std::string> > StringsPtr;

inline void viewMatrix3x3(tf::Matrix3x3& m, THDoubleTensor *t) {
  THDoubleStorage* storage = THDoubleStorage_newWithData(m[0].m_floats, sizeof(m) / sizeof(double));
  THDoubleStorage_clearFlag(storage, TH_STORAGE_FREEMEM | TH_STORAGE_RESIZABLE);
  THDoubleTensor_setStorage2d(t, storage, 0, 3, sizeof(m[0]) / sizeof(double), 3, 1);
  THDoubleStorage_free(storage);   // tensor took ownership
}

inline void viewVector3(tf::Vector3& v, THDoubleTensor *t) {
  THDoubleStorage* storage = THDoubleStorage_newWithData(v.m_floats, sizeof(v.m_floats) / sizeof(double));
  THDoubleStorage_clearFlag(storage, TH_STORAGE_FREEMEM | TH_STORAGE_RESIZABLE);
  THDoubleTensor_setStorage1d(t, storage, 0, 3, 1);
  THDoubleStorage_free(storage);   // tensor took ownership
}

inline void viewQuaternion(tf::Quaternion& q, THDoubleTensor *t) {
  THDoubleStorage* storage = THDoubleStorage_newWithData(static_cast<double*>(q), sizeof(q) / sizeof(double));
  THDoubleStorage_clearFlag(storage, TH_STORAGE_FREEMEM | TH_STORAGE_RESIZABLE);
  THDoubleTensor_setStorage1d(t, storage, 0, 4, 1);
  THDoubleStorage_free(storage);   // tensor took ownership
}

inline void copyVector3ToTensor(const tf::Vector3 &v, THDoubleTensor *tensor) {
  THDoubleTensor_resize1d(tensor, 3);
  THDoubleTensor* output_ = THDoubleTensor_newContiguous(tensor);
  double *data = THDoubleTensor_data(output_);
  data[0] = v.getX();
  data[1] = v.getY();
  data[2] = v.getZ();
  THDoubleTensor_freeCopyTo(output_, tensor);
}

inline void copyTensorToVector3(THDoubleTensor *tensor, tf::Vector3 &v) {
  if (!tensor || THDoubleTensor_nElement(tensor) < 3)
    throw std::runtime_error("A Tensor with at least 3 elements was expected.");
    
  THDoubleTensor *tensor_ = THDoubleTensor_newContiguous(tensor);
  const double *data = THDoubleTensor_data(tensor_);
  v.setX(data[0]);
  v.setY(data[1]);
  v.setZ(data[2]);
  v.setW(0);
  THDoubleTensor_free(tensor_);
}

#endif  // torch_tf_h
