#include "mapora/slam/leo_loam/init_trans_helper.hpp"
#include "mapora/utils.hpp"

namespace mapora
{
namespace slam
{
namespace leo_loam
{
using Point = InitTransHelper::Point;
using Cloud = InitTransHelper::Cloud;

bool InitTransHelper::Compute(
  const std::vector<Eigen::VectorXd> & Alines, Eigen::VectorXd & transform)
{
  //  auto pretty_print = [](const auto & mat, const std::string & name) {
  //      Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
  //      std::string sep = "\n----------------------------------------\n";
  //
  //      std::cout << name << ": " << std::endl << mat.format(CleanFmt) << sep << std::endl;
  //    };
  size_t sizeMat = Alines.size();
  if (sizeMat < 10) {return false;}
  Eigen::MatrixXd A(sizeMat, 6);
  Eigen::VectorXd B(sizeMat);

  // Fill A and B matrices from the Alines
  for (size_t i = 0; i < sizeMat; ++i) {
    for (int j = 0; j < 6; ++j) {
      A.row(i)(j) = Alines[i](j);
    }
    B(i) = Alines[i](6);
  }

  //  pretty_print(A, "matA");
  //  pretty_print(B, "matB");

  // Solve Main Equation
  Eigen::VectorXd X = A.colPivHouseholderQr().solve(B);

  //  pretty_print(X, "matX");

  transform += X;

  for (int i = 0; i < 6; i++) {
    if (std::isnan(transform(i))) {transform(i) = 0;}
  }

  return RMSE(X);
}

void InitTransHelper::NeighborSelectionEdge(
  int closestIndex,
  Cloud::Ptr cloudEdgeLast,
  std::vector<int> pointSearchInd,
  std::vector<Point> & edgeNeighbors) const
{
  if (closestIndex < 0) {return;}

  std::vector<Point> edgeNeighborsTemp;
  Point closestPoint = cloudEdgeLast->points[closestIndex];
  edgeNeighborsTemp.push_back(closestPoint);
  double closestVAngle, closestHAngle, neighborVAngle, neighborHAngle;
  int closestID, neighborID;
  PointInfo(closestPoint, closestVAngle, closestHAngle, closestID);
  for (int i : pointSearchInd) {
    Point p = cloudEdgeLast->points[i];
    PointInfo(p, neighborVAngle, neighborHAngle, neighborID);
    if (
      (neighborID < (closestID + 2.5) && neighborID > closestID && closestIndex < i) ||
      (neighborID > (closestID - 2.5) && neighborID < closestID && closestIndex > i))
    {
      edgeNeighborsTemp.push_back(p);
      edgeNeighbors = edgeNeighborsTemp;
      break;
    }
  }
}

void InitTransHelper::NeighborSelectionFlat(
  int closestIndex,
  const Cloud::ConstPtr & cloudPlanarLast,
  std::vector<int> pointSearchInd,
  std::vector<Point> & flatNeighbors) const
{
  std::vector<Point> flatNeighborsTemp;
  flatNeighborsTemp.push_back(cloudPlanarLast->points[closestIndex]);
  double closestVAngle, closestHAngle, neighborVAngle, neighborHAngle;
  int closestID, neighborID;
  bool second(false), third(false);
  PointInfo(cloudPlanarLast->points[closestIndex], closestVAngle, closestHAngle, closestID);
  for (size_t i = 0; i < pointSearchInd.size(); ++i) {
    if (static_cast<int>(i) == closestIndex) {continue;}
    Point p = cloudPlanarLast->points[pointSearchInd[i]];
    PointInfo(p, neighborVAngle, neighborHAngle, neighborID);
    if (
      (neighborID < (closestID + 2.5) && neighborID > closestID &&
      closestIndex < pointSearchInd[i]) ||
      (neighborID > (closestID - 2.5) && neighborID<closestID &&
      closestIndex> pointSearchInd[i]))
    {
      flatNeighborsTemp.push_back(p);
      second = true;
    } else if (
      ((neighborID < (closestID + 2.5) && neighborID >= closestID &&
      closestIndex < pointSearchInd[i]) ||
      (neighborID > (closestID - 2.5) && neighborID <= closestID &&
      closestIndex > pointSearchInd[i])) &&
      second)
    {
      flatNeighborsTemp.push_back(p);
      third = true;
    }
    if (second && third) {
      flatNeighbors = flatNeighborsTemp;
      break;
    }
  }
}

double InitTransHelper::PointToAngleVertical(const Point & point) const
{
  return atan(point.z / sqrt(std::pow(point.x, 2) + std::pow(point.y, 2)));
}

double InitTransHelper::RadToDeg(double angle_rad) const {return (angle_rad / M_PI) * 180;}

void InitTransHelper::PointInfo(
  Point point, double & pointVAngle, double & pointHAngle, int & pointLaseID) const
{
  pointVAngle = std::round(RadToDeg(PointToAngleVertical(point)));
  pointLaseID = pointVAngle < 0.0 ? int(pointVAngle + 15) : int(pointVAngle);

  double alpha = atan(point.x / point.y);
  if (point.y < 0) {
    pointHAngle = alpha + M_PI;
  } else if (point.y >= 0 && point.x < 0) {
    pointHAngle = alpha + 2 * M_PI;
  } else {
    pointHAngle = alpha;
  }
}

bool InitTransHelper::PointToLine(
  Eigen::VectorXd & Aline,
  const Eigen::VectorXd & T,
  Point currPoint,
  Point pointSel,
  const std::vector<Point> & edgeNeighbors,
  int iterCount) const
{
  if (edgeNeighbors.empty()) {return false;}

  Eigen::Vector3d A, B, C;
  MakeTriangle(A, B, C, pointSel, edgeNeighbors);
  double elevation = ((A.cross(B)).norm()) / C.norm();
  double S = 1;
  if (iterCount >= 5) {S = 1 - 1.8 * fabs(elevation);}
  Eigen::Vector3d Translations = C.cross(A.cross(B)).normalized();
  Translations *= S;

  if (elevation != 0 && S > 0.1) {
    Eigen::MatrixXd J =
      Jacobian(T(0), T(1), T(2), T(3), T(4), T(5), currPoint.x, currPoint.y, currPoint.z);

    Eigen::RowVectorXd AA = Translations.transpose() * J;

    Aline << AA(0), AA(1), AA(2), AA(3), AA(4), AA(5), elevation * -0.05;
    return true;
  }
  return false;
}

bool InitTransHelper::PointToPlane(
  Eigen::VectorXd & Aline,
  const Eigen::VectorXd & T,
  Point currPoint,
  Point pointSel,
  const std::vector<Point> & flatNeighbors,
  int iterCount) const
{
  if (flatNeighbors.empty()) {return false;}

  Eigen::Vector4d planeCoeff;
  Point tripod1 = flatNeighbors[0];
  Point tripod2 = flatNeighbors[1];
  Point tripod3 = flatNeighbors[2];
  Eigen::Vector3d A, B, planeNormal, P1;
  A << tripod2.x - tripod1.x, tripod2.y - tripod1.y, tripod2.z - tripod1.z;
  B << tripod3.x - tripod1.x, tripod3.y - tripod1.y, tripod3.z - tripod1.z;
  planeNormal = A.cross(B);
  P1 << tripod1.x, tripod1.y, tripod1.z;
  double Pd = -P1.transpose() * planeNormal;
  Pd /= planeNormal.norm();
  planeNormal.normalize();
  P1 << pointSel.x, pointSel.y, pointSel.z;
  double Pd2 = (P1.transpose() * planeNormal) + Pd;
  double s = 1;
  if (iterCount >= 5) {s = 1 - 1.8 * fabs(Pd2) / sqrt(P1.norm());}
  Eigen::Vector3d Translations;
  Translations(0) = s * planeNormal(0);
  Translations(1) = s * planeNormal(1);
  Translations(2) = s * planeNormal(2);
  double elevation = s * Pd2;

  if (elevation != 0 && s > 0.1) {
    Eigen::MatrixXd J =
      Jacobian(T(0), T(1), T(2), T(3), T(4), T(5), currPoint.x, currPoint.y, currPoint.z);
    Eigen::RowVectorXd AA = Translations.transpose() * J;
    Aline << AA(0), AA(1), AA(2), AA(3), AA(4), AA(5), -0.05 * elevation;
    return true;
  }
  return false;
}

void InitTransHelper::MakeTriangle(
  Eigen::Vector3d & A,
  Eigen::Vector3d & B,
  Eigen::Vector3d & C,
  Point pointSel,
  std::vector<Point> edgeNeighbors) const
{
  A << (pointSel.x - edgeNeighbors[0].x), (pointSel.y - edgeNeighbors[0].y),
  (pointSel.z - edgeNeighbors[0].z);
  B << (pointSel.x - edgeNeighbors[1].x), (pointSel.y - edgeNeighbors[1].y),
  (pointSel.z - edgeNeighbors[1].z);
  C << (edgeNeighbors[0].x - edgeNeighbors[1].x), (edgeNeighbors[0].y - edgeNeighbors[1].y),
  (edgeNeighbors[0].z - edgeNeighbors[1].z);
}

Eigen::MatrixXd InitTransHelper::Jacobian(
  double rx, double ry, double rz, double tx, double ty, double tz, double X, double Y, double Z)
const
{
  double sin_rx = sin(rx);
  double cos_rx = cos(rx);

  double sin_ry = sin(ry);
  double cos_ry = cos(ry);

  double sin_rz = sin(rz);
  double cos_rz = cos(rz);

  Eigen::MatrixXd J(3, 6);
  J << Z * sin_rx * sin_ry - tz * sin_rx * sin_ry + Y * cos_rx * cos_rz * sin_ry -
    X * cos_rx * sin_ry * sin_rz - ty * cos_rx * cos_rz * sin_ry +
    tx * cos_rx * sin_ry * sin_rz,
    tx * (cos_rz * sin_ry + cos_ry * sin_rx * sin_rz) -
    Y * (sin_ry * sin_rz - cos_ry * cos_rz * sin_rx) -
    X * (cos_rz * sin_ry + cos_ry * sin_rx * sin_rz) +
    ty * (sin_ry * sin_rz - cos_ry * cos_rz * sin_rx) - Z * cos_rx * cos_ry +
    tz * cos_rx * cos_ry,
    Y * (cos_ry * cos_rz - sin_rx * sin_ry * sin_rz) -
    X * (cos_ry * sin_rz + cos_rz * sin_rx * sin_ry) +
    tx * (cos_ry * sin_rz + cos_rz * sin_rx * sin_ry) -
    ty * (cos_ry * cos_rz - sin_rx * sin_ry * sin_rz),
    sin_rx * sin_ry * sin_rz - cos_ry * cos_rz, -cos_ry * sin_rz - cos_rz * sin_rx * sin_ry,
    cos_rx * sin_ry,
    Z * cos_rx - tz * cos_rx - Y * cos_rz * sin_rx + X * sin_rx * sin_rz + ty * cos_rz * sin_rx -
    tx * sin_rx * sin_rz,
    0, tx * cos_rx * cos_rz - Y * cos_rx * sin_rz - X * cos_rx * cos_rz + ty * cos_rx * sin_rz,
    cos_rx * sin_rz, -cos_rx * cos_rz, -sin_rx,
    tz * cos_ry * sin_rx - Z * cos_ry * sin_rx - Y * cos_rx * cos_ry * cos_rz +
    X * cos_rx * cos_ry * sin_rz + ty * cos_rx * cos_ry * cos_rz - tx * cos_rx * cos_ry * sin_rz,
    X * (cos_ry * cos_rz - sin_rx * sin_ry * sin_rz) +
    Y * (cos_ry * sin_rz + cos_rz * sin_rx * sin_ry) -
    tx * (cos_ry * cos_rz - sin_rx * sin_ry * sin_rz) -
    ty * (cos_ry * sin_rz + cos_rz * sin_rx * sin_ry) - Z * cos_rx * sin_ry +
    tz * cos_rx * sin_ry,
    Y * (cos_rz * sin_ry + cos_ry * sin_rx * sin_rz) -
    X * (sin_ry * sin_rz - cos_ry * cos_rz * sin_rx) +
    tx * (sin_ry * sin_rz - cos_ry * cos_rz * sin_rx) -
    ty * (cos_rz * sin_ry + cos_ry * sin_rx * sin_rz),
    -cos_rz * sin_ry - cos_ry * sin_rx * sin_rz, cos_ry * cos_rz * sin_rx - sin_ry * sin_rz,
    -cos_rx * cos_ry;
  return J;
}

bool InitTransHelper::RMSE(Eigen::VectorXd X) const
{
  double deltaR = sqrt(
    pow(utils::Utils::rad_to_deg(X(0)), 2) + pow(utils::Utils::rad_to_deg(X(1)), 2) +
    pow(utils::Utils::rad_to_deg(X(2)), 2));
  double deltaT = sqrt(pow(X(3) * 100, 2) + pow(X(4) * 100, 2) + pow(X(5) * 100, 2));

  return deltaR < 0.1 && deltaT < 0.1;
}

}  // namespace leo_loam
}  // namespace slam
}  // namespace mapora
