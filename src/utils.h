#ifndef _utils_h
#define _utils_h

/*
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

inline Eigen::Vector4d Tensor2Vec4d(THDoubleTensor *tensor)
{
  if (!tensor || THDoubleTensor_nElement(tensor) < 4)
    throw MoveItWrapperException("A tensor with at least 4 elements was expected.");

  THDoubleTensor *tensor_ = THDoubleTensor_newContiguous(tensor);
  double* data = THDoubleTensor_data(tensor_);
  Eigen::Vector4d v(data[0], data[1], data[2], data[3]);
  THDoubleTensor_free(tensor_);
  return v;
}

inline Eigen::Vector3d Tensor2Vec3d(THDoubleTensor *tensor)
{
  if (!tensor || THDoubleTensor_nElement(tensor) < 3)
    throw MoveItWrapperException("A Tensor with at least 3 elements was expected.");

  THDoubleTensor *tensor_ = THDoubleTensor_newContiguous(tensor);
  double* data = THDoubleTensor_data(tensor_);
  Eigen::Vector3d v(data[0], data[1], data[2]);
  THDoubleTensor_free(tensor_);
  return v;
}

template<int rows, int cols>
inline Eigen::Matrix<double, rows, cols> Tensor2Mat(THDoubleTensor *tensor)
{
  THArgCheck(tensor != NULL && tensor->nDimension == 2 && tensor->size[0] == rows && tensor->size[1] == cols, 1, "invalid tensor");
  tensor = THDoubleTensor_newContiguous(tensor);
  Eigen::Matrix<double, rows, cols> output(Eigen::Map<Eigen::Matrix<double, rows, cols, Eigen::RowMajor> >(THDoubleTensor_data(tensor)));
  THDoubleTensor_free(tensor);
  return output;
}

template<int rows, int cols, int options> void viewMatrix(Eigen::Matrix<double, rows, cols, options> &m, THDoubleTensor *output)
{
  // create new storage that views into the matrix
  THDoubleStorage* storage = NULL;
  if ((Eigen::Matrix<double, rows, cols, options>::Options & Eigen::RowMajor) == Eigen::RowMajor)
    storage = THDoubleStorage_newWithData(m.data(), (m.rows() * m.rowStride()));
  else
    storage = THDoubleStorage_newWithData(m.data(), (m.cols() * m.colStride()));

  storage->flag = TH_STORAGE_REFCOUNTED;
  THDoubleTensor_setStorage2d(output, storage, 0, rows, m.rowStride(), cols, m.colStride());
  THDoubleStorage_free(storage);   // tensor took ownership
}

inline void viewArray(double* array, size_t length, THDoubleTensor *output)
{
  THDoubleStorage* storage = THDoubleStorage_newWithData(array, length);
  storage->flag = TH_STORAGE_REFCOUNTED;
  THDoubleTensor_setStorage1d(output, storage, 0, length, 1);
  THDoubleStorage_free(storage);   // tensor took ownership
}

template<int rows, int cols> void copyMatrix(const Eigen::Matrix<double, rows, cols> &m, THDoubleTensor *output)
{
  THDoubleTensor_resize2d(output, m.rows(), m.cols());
  THDoubleTensor* output_ = THDoubleTensor_newContiguous(output);
  // there are strange static-asserts in Eigen to disallow specifying RowMajor for vectors...
  Eigen::Map<Eigen::Matrix<double, rows, cols, (rows == 1 || cols == 1) ? Eigen::ColMajor : Eigen::RowMajor> >(THDoubleTensor_data(output_)) = m;
  THDoubleTensor_freeCopyTo(output_, output);
}*/

#define DECL_vector2Tensor(T, name) inline void vector2Tensor(const std::vector<T> &v, TH##name##Tensor *output) \
{ \
  TH##name##Tensor_resize1d(output, v.size()); \
  TH##name##Tensor *output_ = TH##name##Tensor_newContiguous(output); \
  std::copy(v.begin(), v.end(), TH##name##Tensor_data(output_)); \
  TH##name##Tensor_freeCopyTo(output_, output); \
}

DECL_vector2Tensor(double, Double)
DECL_vector2Tensor(float, Float)
DECL_vector2Tensor(int, Int)

#define DECL_Tensor2vector(T, name) inline void Tensor2vector(TH##name##Tensor *input, std::vector<T> &v) \
{ \
  long n = TH##name##Tensor_nElement(input); \
  v.resize(n); \
  input = TH##name##Tensor_newContiguous(input); \
  T *begin = TH##name##Tensor_data(input); \
  std::copy(begin, begin + n, v.begin()); \
  TH##name##Tensor_free(input); \
}

DECL_Tensor2vector(double, Double)
DECL_Tensor2vector(float, Float)
DECL_Tensor2vector(int, Int)

#endif //_utils_h
