// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#include "mapora/slam/leo_loam/trans_estimator.hpp"

#include <vector>

#include "mapora/utils.hpp"

namespace mapora
{
namespace slam
{
namespace leo_loam
{
using Point = TransEstimator::Point;
using Cloud = TransEstimator::Cloud;

bool TransEstimator::Compute(const std::vector<Eigen::VectorXd> & Alines, Eigen::VectorXd & X)
{
  size_t sizeMat = Alines.size();
  Eigen::MatrixXd A(sizeMat, 6);
  Eigen::VectorXd B(sizeMat);

  // Fill A and B matrices from the Alines
  for (size_t i = 0; i < sizeMat; ++i) {
    for (size_t j = 0; j < 6; ++j) {
      A.row(i)(j) = Alines[i](j);
    }
    B(i) = -Alines[i](6);
  }

  // Solve Main Equation
  X = A.colPivHouseholderQr().solve(B);

  Transformation += X;
  return RMSE(X);
}

bool TransEstimator::coeffsCalculator(
  const Point & pointOri,
  const Cloud::ConstPtr neighbors,
  bool isEdge,
  Eigen::VectorXd & Aline) const
{
  Eigen::Vector3d newPoint = pointAssociateToMap(pointOri);

  bool result;
  Eigen::Vector3d RPY;
  RPY.setZero();
  RPY << Transformation(0), Transformation(1), Transformation(2);
  if (isEdge) {
    result = point2Line(newPoint, pointOri, neighbors, Aline, RPY);
  } else {
    result = point2Plane(newPoint, pointOri, neighbors, Aline, RPY);
  }
  return result;
}

bool TransEstimator::point2Line(
  const Eigen::Vector3d & pointSel,
  const Point & pointOri,
  const Cloud::ConstPtr neighbors,
  Eigen::VectorXd & Aline,
  const Eigen::Vector3d & RPY) const
{
  double eigenValue;
  Eigen::Vector3d eigenVector, centerPoint(0, 0, 0);
  if (distributionOfNeighbors(neighbors, eigenValue, eigenVector, centerPoint)) {
    Eigen::Vector3d leftPoint = centerPoint + eigenValue * eigenVector;
    Eigen::Vector3d rightPoint = centerPoint - eigenValue * eigenVector;
    Eigen::Vector3d A, B, C;
    makeTriangle(pointSel, leftPoint, rightPoint, A, B, C);
    double parallelogram = (A.cross(B)).norm();
    // Translation coeffs
    Eigen::Vector3d Translations = C.cross(A.cross(B)) / parallelogram / C.norm();
    Eigen::Vector3d temp = jacobian(pointOri, RPY, Translations);
    // Observation value
    double elevation = parallelogram / C.norm();
    Aline << temp(0), temp(1), temp(2), Translations(0), Translations(1), Translations(2),
      elevation;
    return true;
  }
  return false;
}

bool TransEstimator::distributionOfNeighbors(
  const Cloud::ConstPtr neighbors,
  double & eigenValue,
  Eigen::Vector3d & eigenVector,
  Eigen::Vector3d & centerPoint) const
{
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(Covarience(neighbors, centerPoint));
  Eigen::Vector3d eigenValues = eigensolver.eigenvalues();
  Eigen::Matrix3d eigenVectors = eigensolver.eigenvectors();
  int idx;
  if (isWellDistributed(eigenValues, idx)) {
    eigenValue = eigenValues(idx);
    eigenVector = Eigen::Vector3d(eigenVectors(0, 2), eigenVectors(1, 2), eigenVectors(2, 2));
    return true;
  }
  return false;
}

Eigen::Matrix3d TransEstimator::Covarience(
  Cloud::ConstPtr neighbors, Eigen::Vector3d & centerPoint) const
{
  size_t nSize = neighbors->points.size();
  for (size_t i = 0; i < nSize; i++) {
    centerPoint(0) += neighbors->points[i].x / nSize;
    centerPoint(1) += neighbors->points[i].y / nSize;
    centerPoint(2) += neighbors->points[i].z / nSize;
  }
  Eigen::Matrix3d covarianceMat = Eigen::Matrix3d::Zero();
  for (size_t i = 0; i < nSize; i++) {
    Eigen::Vector3d C;
    C(0) = neighbors->points[i].x - centerPoint(0);
    C(1) = neighbors->points[i].y - centerPoint(1);
    C(2) = neighbors->points[i].z - centerPoint(2);
    covarianceMat += C * C.transpose() / nSize;
  }
  return covarianceMat;
}

bool TransEstimator::isWellDistributed(const Eigen::Vector3d & eigenValue, int & idx) const
{
  if (eigenValue(0) > 3 * eigenValue(1) || eigenValue(0) > 3 * eigenValue(2)) {
    idx = 0;
    return true;
  } else if (eigenValue(1) > 3 * eigenValue(0) || eigenValue(1) > 3 * eigenValue(2)) {
    idx = 1;
    return true;
  } else if (eigenValue(2) > 3 * eigenValue(0) || eigenValue(2) > 3 * eigenValue(1)) {
    idx = 2;
    return true;
  }
  return false;
}


void TransEstimator::makeTriangle(
  const Eigen::Vector3d & newtPoint,
  const Eigen::Vector3d & leftPoint,
  const Eigen::Vector3d & righttPoint,
  Eigen::Vector3d & A,
  Eigen::Vector3d & B,
  Eigen::Vector3d & C) const
{
  A = newtPoint - righttPoint;
  B = newtPoint - leftPoint;
  C = righttPoint - leftPoint;
}

Eigen::Vector3d TransEstimator::jacobian(
  const Point & pointOri, const Eigen::Vector3d & RPY, const Eigen::Vector3d & Translations) const
{
  double srx = sin(RPY(0));
  double crx = cos(RPY(0));

  double sry = sin(RPY(1));
  double cry = cos(RPY(1));

  double srz = sin(RPY(2));
  double crz = cos(RPY(2));

  Eigen::Vector3d temp;

  temp(0) =
    (crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y - srx * sry * pointOri.z) *
    Translations(0) +
    (-srx * srz * pointOri.x - crz * srx * pointOri.y - crx * pointOri.z) * Translations(1) +
    (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y - cry * srx * pointOri.z) *
    Translations(2);

  temp(1) = ((cry * srx * srz - crz * sry) * pointOri.x +
    (sry * srz + cry * crz * srx) * pointOri.y + crx * cry * pointOri.z) *
    Translations(0) +
    ((-cry * crz - srx * sry * srz) * pointOri.x +
    (cry * srz - crz * srx * sry) * pointOri.y - crx * sry * pointOri.z) *
    Translations(2);

  temp(2) =
    ((crz * srx * sry - cry * srz) * pointOri.x + (-cry * crz - srx * sry * srz) * pointOri.y) *
    Translations(0) +
    (crx * crz * pointOri.x - crx * srz * pointOri.y) * Translations(1) +
    ((sry * srz + cry * crz * srx) * pointOri.x + (crz * sry - cry * srx * srz) * pointOri.y) *
    Translations(2);

  return temp;
}


bool TransEstimator::point2Plane(
  const Eigen::Vector3d & newPoint,
  const Point & interestPoint,
  Cloud::ConstPtr neighbors,
  Eigen::VectorXd & Aline,
  const Eigen::Vector3d & RPY) const
{
  Eigen::Vector4d planeCoeff;
  if (pointsLieOnPlane(neighbors, planeCoeff)) {
    Eigen::Vector4d ip(newPoint(0), newPoint(1), newPoint(2), 1);
    double elevation = planeCoeff.transpose() * ip;
    Eigen::Vector3d Translations(planeCoeff(0), planeCoeff(1), planeCoeff(2));
    Eigen::Vector3d temp = jacobian(interestPoint, RPY, Translations);
    Aline << temp(0), temp(1), temp(2), Translations(0), Translations(1), Translations(2),
      elevation;
    return true;
  }
  return false;
}

bool TransEstimator::pointsLieOnPlane(Cloud::ConstPtr neighbors, Eigen::Vector4d & planeCoeff) const
{
  size_t sizeP = neighbors->points.size();
  Eigen::MatrixXd A(sizeP, 3);
  Eigen::VectorXd BB(sizeP);
  for (size_t j = 0; j < sizeP; j++) {
    A(j, 0) = neighbors->points[j].x;
    A(j, 1) = neighbors->points[j].y;
    A(j, 2) = neighbors->points[j].z;
    BB(j) = -1;
  }
  Eigen::VectorXd X = A.colPivHouseholderQr().solve(BB);
  Eigen::VectorXd newX = X.normalized();
  Eigen::VectorXd V = A * newX - (BB / X.norm());
  double maxV = V.maxCoeff();
  double minV = V.minCoeff();
  bool planeValid = true;
  if (maxV > 0.2 || minV < -0.2) {planeValid = false;}
  planeCoeff = Eigen::Vector4d(newX(0), newX(1), newX(2), -BB(0) / X.norm());
  return planeValid;
}

bool TransEstimator::RMSE(const Eigen::VectorXd & X)
{
  double deltaR = sqrt(
    pow(((X(0)) * 180 / M_PI), 2) + pow(((X(1)) * 180 / M_PI), 2) + pow(((X(2)) * 180 / M_PI), 2));
  double deltaT = sqrt(pow(X(3) * 100, 2) + pow(X(4) * 100, 2) + pow(X(5) * 100, 2));
  return deltaR < 0.05 && deltaT < 0.05;
}

void TransEstimator::setTransformation(const Eigen::VectorXd & T) {Transformation = T;}

const Eigen::VectorXd & TransEstimator::getTransformation() const {return Transformation;}

Eigen::Vector3d TransEstimator::pointAssociateToMap(const Point & pi) const
{
  float x1 = cos(Transformation[2]) * pi.x - sin(Transformation[2]) * pi.y;
  float y1 = sin(Transformation[2]) * pi.x + cos(Transformation[2]) * pi.y;
  float z1 = pi.z;

  float x2 = x1;
  float y2 = cos(Transformation[0]) * y1 - sin(Transformation[0]) * z1;
  float z2 = sin(Transformation[0]) * y1 + cos(Transformation[0]) * z1;
  Eigen::Vector3d po(
    cos(Transformation[1]) * x2 + sin(Transformation[1]) * z2 + Transformation[3],
    y2 + Transformation[4],
    -sin(Transformation[1]) * x2 + cos(Transformation[1]) * z2 + Transformation[5]);
  return po;
}

Point TransEstimator::pointAssociateToMapPPP(Point pi)
{
  float x1 = cos(Transformation[2]) * pi.x - sin(Transformation[2]) * pi.y;
  float y1 = sin(Transformation[2]) * pi.x + cos(Transformation[2]) * pi.y;
  float z1 = pi.z;

  float x2 = x1;
  float y2 = cos(Transformation[0]) * y1 - sin(Transformation[0]) * z1;
  float z2 = sin(Transformation[0]) * y1 + cos(Transformation[0]) * z1;
  Point po;
  po.x = cos(Transformation[1]) * x2 + sin(Transformation[1]) * z2 + Transformation[3];
  po.y = y2 + Transformation[4];
  po.z = -sin(Transformation[1]) * x2 + cos(Transformation[1]) * z2 + Transformation[5];
  return po;
}


}  // namespace leo_loam
}  // namespace slam
}  // namespace mapora
