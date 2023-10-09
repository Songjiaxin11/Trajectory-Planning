#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <stdexcept>
#include <numeric>

using namespace Eigen;
using Eigen::MatrixXd;
typedef std::vector<std::vector<double>> MyDoubleMatrix;
typedef std::vector<double> MyDoubleVector;
std::vector<std::vector<int>> matrix_multiply(const std::vector<std::vector<int>>& a, const std::vector<std::vector<int>>& b) {
    int n = a.size();
    int m = b[0].size();
    int p = b.size();
    std::vector<std::vector<int>> c(n, std::vector<int>(m, 0));
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            for (int k = 0; k < p; k++) {
                c[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    return c;
}


MyDoubleVector linspace(double start, double end, int num)
{
    MyDoubleVector result;
    double step = (end - start) / (num - 1);
    for (int i = 0; i < num; ++i)
    {
        result.push_back(start + i * step);
    }
    return result;
}
MyDoubleMatrix transpose(const MyDoubleMatrix &matrix)
{
    size_t rows = matrix.size();
    size_t cols = matrix[0].size();

    MyDoubleMatrix result(cols, MyDoubleVector(rows));

    for (size_t i = 0; i < rows; ++i)
    {
        for (size_t j = 0; j < cols; ++j)
        {
            result[j][i] = matrix[i][j];
        }
    }

    return result;
}
MyDoubleMatrix getCircle(double R, double targetSpeed, double controllerFreq, double initX, double initY, double initZ)
{
    double stepDis = targetSpeed / controllerFreq;
    double stepTheta = stepDis / R;
    int numPoints = static_cast<int>(2 * M_PI / stepTheta);

    MyDoubleVector theta(numPoints);
    for (int i = 0; i < numPoints; ++i)
    {
        theta[i] = i * stepTheta;
    }

    MyDoubleVector x(numPoints);
    MyDoubleVector y(numPoints);
    MyDoubleVector z(numPoints, initZ);

    for (int i = 0; i < numPoints; ++i)
    {
        x[i] = R * cos(theta[i]) + initX;
        y[i] = R * sin(theta[i]) + initY;
    }

    MyDoubleMatrix speedMatrix(numPoints, MyDoubleVector(numPoints, 0));
    for (int i = 0; i < numPoints; ++i)
    {
        speedMatrix[i][(i - 1 + numPoints) % numPoints] = 1;
    }
    MyDoubleVector vx(numPoints);
    MyDoubleVector vy(numPoints);
    MyDoubleVector vz(numPoints);

    for (int i = 0; i < numPoints; ++i)
    {
        for (int j = 0; j < numPoints; ++j)
        {
            vx[i] += speedMatrix[i][j] * x[j];
            vy[i] += speedMatrix[i][j] * y[j];
            vz[i] += speedMatrix[i][j] * z[j];
        }
        vx[i] *= controllerFreq;
        vy[i] *= controllerFreq;
        vz[i] *= controllerFreq;
    }

    MyDoubleMatrix trajectory(6, MyDoubleVector(numPoints));
    for (int i = 0; i < numPoints; ++i)
    {
        trajectory[0][i] = x[i];
        trajectory[1][i] = y[i];
        trajectory[2][i] = z[i];
        trajectory[3][i] = vx[i];
        trajectory[4][i] = vy[i];
        trajectory[5][i] = vz[i];
    }
    return trajectory;
}

MyDoubleMatrix getCircleHuman(double R, double targetSpeed, double controllerFreq, double initX, double initY, double initZ) // bug:  当频率过小时, 会segmentation fault
{
    double stepDis = targetSpeed / controllerFreq;
    double stepTheta = stepDis / R;
    int numPoints = static_cast<int>(2 * M_PI / stepTheta);

    MyDoubleVector theta(numPoints);
    for (int i = 0; i < numPoints; ++i)
    {
        theta[i] = i * stepTheta;
    }

    MyDoubleVector xInit(numPoints);
    MyDoubleVector yInit(numPoints);

    for (int i = 0; i < numPoints; ++i)
    {
        xInit[i] = R * cos(theta[i]) + initX;
        yInit[i] = R * sin(theta[i]) + initY;
    }

    int changeIndex1 = static_cast<int>(numPoints / 3);
    int changeIndex2 = static_cast<int>(numPoints / 3 * 2);

    double midX = (xInit[changeIndex1] + xInit[changeIndex2]) / 2 - R / 2;
    double midY = (yInit[changeIndex1] + yInit[changeIndex2]) / 2;

    double distanceToMid = sqrt(pow(xInit[changeIndex1] - midX, 2) + pow(yInit[changeIndex1] - midY, 2));
    int num = static_cast<int>(distanceToMid / stepDis);

    MyDoubleVector Xmid1(num);
    MyDoubleVector Ymid1(num);
    MyDoubleVector Xmid2(num);
    MyDoubleVector Ymid2(num);

    for (int i = 0; i < num; ++i)
    {
        Xmid1[i] = xInit[changeIndex1] + i * ((midX - xInit[changeIndex1]) / num);
        Ymid1[i] = yInit[changeIndex1] + i * ((midY - yInit[changeIndex1]) / num);
        Xmid2[i] = midX + i * ((xInit[changeIndex2] - midX) / num);
        Ymid2[i] = midY + i * ((yInit[changeIndex2] - midY) / num);
    }

    MyDoubleVector x;
    MyDoubleVector y;
    for (int i = 0; i < changeIndex1; ++i)
    {
        x.push_back(xInit[i]);
        y.push_back(yInit[i]);
    }
    for (int i = 0; i < num; ++i)
    {
        x.push_back(Xmid1[i]);
        y.push_back(Ymid1[i]);
    }
    for (int i = 0; i < num; ++i)
    {
        x.push_back(Xmid2[i]);
        y.push_back(Ymid2[i]);
    }
    for (int i = changeIndex2; i < numPoints; ++i)
    {
        x.push_back(xInit[i]);
        y.push_back(yInit[i]);
    }

    MyDoubleVector z(x.size(), initZ);

    MyDoubleMatrix speedMatrix(x.size(), MyDoubleVector(x.size(), 0));
    for (int i = 0; i < x.size(); ++i)
    {
        speedMatrix[i][(i - 1 + x.size()) % x.size()] = 1;
    }

    MyDoubleVector vx(x.size(), 0);
    MyDoubleVector vy(x.size(), 0);
    MyDoubleVector vz(x.size(), 0);

    for (int i = 0; i < x.size(); ++i)
    {
        for (int j = 0; j < x.size(); ++j)
        {
            vx[i] += speedMatrix[i][j] * x[j];
            vy[i] += speedMatrix[i][j] * y[j];
            vz[i] += speedMatrix[i][j] * z[j];
        }
        vx[i] *= controllerFreq;
        vy[i] *= controllerFreq;
        vz[i] *= controllerFreq;
    }

    MyDoubleMatrix trajectory(6, MyDoubleVector(x.size()));
    for (int i = 0; i < x.size(); ++i)
    {
        trajectory[0][i] = x[i];
        trajectory[1][i] = y[i];
        trajectory[2][i] = z[i];
        trajectory[3][i] = vx[i];
        trajectory[4][i] = vy[i];
        trajectory[5][i] = vz[i];
    }

    return trajectory;
}

// original getLine ok
Matrix<double, 6, Dynamic> getLine(double distance, double targetSpeed, double controllerFreq, double initX, double initY, double initZ, const std::string &Direction)
{
    double stepDis = targetSpeed / controllerFreq;
    int num = static_cast<int>(distance / stepDis);
    MyDoubleVector x, y;
    if (Direction == "x")
    {
        x = linspace(initX, initX + distance, num);
        y = MyDoubleVector(num, initY);
    }
    else if (Direction == "y")
    {
        x = MyDoubleVector(num, initX);
        y = linspace(initY, initY + distance, num);
    }
    MyDoubleVector z(num, initZ);
    MyDoubleVector vx(num, (Direction == "x") ? targetSpeed : 0.0);
    MyDoubleVector vy(num, (Direction == "y") ? targetSpeed : 0.0);
    MyDoubleVector vz(num, 0.0);
    // Calculate velocity along the trajectory
    MatrixXd vx_mat = Eigen::Map<MatrixXd>(vx.data(), 1, vx.size());
    MatrixXd vy_mat = Eigen::Map<MatrixXd>(vy.data(), 1, vy.size());
    MatrixXd vz_mat = Eigen::Map<MatrixXd>(vz.data(), 1, vz.size());
    Matrix<double, 6, Dynamic> trajectory(6, num);
    trajectory << Eigen::Map<MatrixXd>(x.data(), 1, x.size()),
        Eigen::Map<MatrixXd>(y.data(), 1, y.size()),
        Eigen::Map<MatrixXd>(z.data(), 1, z.size()), vx_mat,
        vy_mat,
        vz_mat;
    // Eigen::Map<MatrixXd>(vx.data(), 1, vx.size()),
    // Eigen::Map<MatrixXd>(vy.data(), 1, vy.size()),
    // Eigen::Map<MatrixXd>(vz.data(), 1, vz.size());
    return trajectory;
}

class MinimumTrajPlanner
{
private:
    int r;
    MyDoubleMatrix path;
    double arvSpeed;
    double Freq;
    MyDoubleMatrix v0;
    MyDoubleMatrix a0;
    MyDoubleMatrix vt;
    MyDoubleMatrix at;
    MyDoubleVector ts;
    double T;
    int order;

public:
    MinimumTrajPlanner(const MyDoubleMatrix &path, double arvSpeed, double controllerFreq,
                       const MyDoubleMatrix &v0,
                       const MyDoubleMatrix &a0,
                       const MyDoubleMatrix &vt,
                       const MyDoubleMatrix &at, int r)
    {
        this->r = r;
        this->path = path;
        this->arvSpeed = arvSpeed;
        this->Freq = controllerFreq;

        // Check dimensions
        if (v0.size() != 3 || a0.size() != 3 || vt.size() != 3 || at.size() != 3 ||
            v0[0].size() != 1 || a0[0].size() != 1 || vt[0].size() != 1 || at[0].size() != 1)
        {
            throw std::invalid_argument("v0, a0, vt, at must be 3*1");
        }

        this->v0 = transpose(v0);
        this->a0 = transpose(a0);
        this->vt = transpose(vt);
        this->at = transpose(at);

        this->ts = MyDoubleVector();
        this->T = 0.0;
        this->order = 5;
    }

    void arrangeT()
    {
        MyDoubleMatrix dx;
        for (size_t i = 1; i < path.size(); ++i)
        {
            MyDoubleVector diff;
            for (size_t j = 0; j < path[i].size(); ++j)
            {
                double difference = path[i][j] - path[i - 1][j];
                diff.push_back(difference);
            }
            dx.push_back(diff);
        }

        double distance = 0;
        for (size_t i = 0; i < dx.size(); ++i)
        {
            double dot_product = std::inner_product(dx[i].begin(), dx[i].end(), dx[i].begin(), 0.0);
            distance += std::sqrt(dot_product);
        }

        T = distance / arvSpeed;
        ts.push_back(0);

        for (size_t i = 0; i < path.size() - 1; ++i)
        {
            double dot_product = std::inner_product(dx[i].begin(), dx[i].end(), dx[i].begin(), 0.0);
            double dt = std::sqrt(dot_product) / arvSpeed;

            if (i == 0)
            {
                ts.push_back(ts[i] + 0.5 * dt);
            }
            else if (i == path.size() - 2)
            {
                ts.push_back(ts[i] + 0.5 * dt);
                ts.push_back(ts[i] + dt);
            }
            else
            {
                ts.push_back(ts[i] + dt);
            }
        }
        for (size_t i = 0; i < ts.size(); ++i)
        {
            std::cout << "Timestamp " << i << ": " << ts[i] << std::endl;
        }
    }

    MyDoubleMatrix computeQ(int n, int r, double t1, double t2)
    {
        MyDoubleMatrix Q(n + 1, MyDoubleVector(n + 1, 0.0));
        MyDoubleVector T((n - r) * 2 + 1, 0.0);
        for (int i = 0; i < (n - r) * 2 + 1; ++i)
        {
            T[i] = pow(t2, i + 1) - pow(t1, i + 1);
        }

        for (int i = r + 1; i < n + 2; ++i)
        {
            for (int j = i; j < n + 2; ++j)
            {
                int k1 = i - r - 1;
                int k2 = j - r - 1;
                int k = k1 + k2 + 1;
                double prod1 = 1.0;
                double prod2 = 1.0;
                for (int m = k1 + 1; m <= k1 + r; ++m)
                {
                    prod1 *= m;
                }
                for (int m = k2 + 1; m <= k2 + r; ++m)
                {
                    prod2 *= m;
                }
                Q[i - 1][j - 1] = prod1 * prod2 / k * T[k - 1];
                Q[j - 1][i - 1] = Q[i - 1][j - 1];
            }
        }

        return Q;
    }


    MyDoubleMatrix computeSingleAxisTraj(MyDoubleVector path, double v0, double a0, double vt, double at)
    {
        // 计算单维度轨迹多项式参数

        int n_coef = order + 1;  // 多项式系数个数
        int n_seg = path.size() - 1;  // 轨迹段数

        // compute Q
        MyDoubleMatrix Q(n_coef * n_seg, MyDoubleVector(n_coef * n_seg, 0.0));
        for (int k = 0; k < n_seg; ++k)
        {
            MyDoubleMatrix Q_k = computeQ(order, r, ts[k], ts[k + 1]);
            for (int i = 0; i < n_coef; ++i)
            {
                for (int j = 0; j < n_coef; ++j)
                {
                    Q[k * n_coef + i][k * n_coef + j] = Q_k[i][j];
                }
            }
        }

        // compute Tk Tk(i,j) = ts(i)^(j)
        MyDoubleMatrix Tk(n_seg + 1, MyDoubleVector(n_coef, 0.0));
        for (int i = 0; i < n_coef; ++i)
        {
            for (int j = 0; j < n_seg + 1; ++j)
            {
                Tk[j][i] = pow(ts[j], i);
            }
        }

        // compute A
        int n_continuous = 3;  // 1:p  2:pv  3:pva  4:pvaj  5:pvajs
        MyDoubleMatrix A(n_continuous * 2 * n_seg, MyDoubleVector(n_coef * n_seg, 0.0));
        for (int i = 1; i < n_seg + 1; ++i)
        {
            for (int j = 1; j < n_continuous + 1; ++j)
            {
                for (int k = j; k < n_coef + 1; ++k)
                {
                    double t1 = Tk[i - 1][k - j];
                    double t2 = Tk[i][k - j];
                    double prod = 1.0;
                    for (int m = k - j + 1; m <= k; ++m)
                    {
                        prod *= m;
                    }
                    A[n_continuous * 2 * (i - 1) + j - 1][n_coef * (i - 1) + k - 1] = prod * t1;
                    A[n_continuous * 2 * (i - 1) + j - 1 + n_continuous][n_coef * (i - 1) + k - 1] = prod * t2;
                }
            }
        }

        // compute M
        int num_d = n_continuous * (n_seg + 1);
        MyDoubleMatrix M(n_continuous * 2 * n_seg, MyDoubleVector(n_continuous * (n_seg + 1), 0.0));
        for (int i = 1; i < 2 * n_seg + 1; ++i)
        {
            int j = std::floor(i / 2) + 1;
            int rbeg = (i - 1) * n_continuous;
            int cbeg = (j - 1) * n_continuous;
            for (int k = 0; k < n_continuous; ++k)
            {
                M[rbeg + k][cbeg + k] = 1.0;
            }
        }

        // compute C
        MyDoubleMatrix C(num_d, MyDoubleVector(num_d, 0.0));
        MyDoubleVector df;
        df.reserve(num_d);
        for (int i = 0; i < path.size(); ++i)
        {
            df.push_back(path[i]);
        }
        df.push_back(v0);
        df.push_back(a0);
        df.push_back(vt);
        df.push_back(at);
        std::vector<int> fix_idx;
        for (int i = 1; i < num_d; i += n_continuous)
        {
            fix_idx.push_back(i);
        }
        fix_idx.push_back(2);
        fix_idx.push_back(3);
        fix_idx.push_back(num_d - 1);
        fix_idx.push_back(num_d);
        std::vector<int> free_idx;
        for (int i = 1; i <= num_d; ++i)
        {
            if (std::find(fix_idx.begin(), fix_idx.end(), i) == fix_idx.end())
            {
                free_idx.push_back(i);
            }
        }
        for (int i = 0; i < num_d; ++i)
        {
            for (int j = 0; j < fix_idx.size(); ++j)
            {
                C[i][j] = (i == fix_idx[j] - 1) ? 1.0 : 0.0;
            }
            for (int j = 0; j < free_idx.size(); ++j)
            {
                C[i][fix_idx.size() + j] = (i == free_idx[j] - 1) ? 1.0 : 0.0;
            }
        }

        MyDoubleMatrix AiMC = matrix_multiply(matrix_multiply(matrix_inverse(A), M), C);
        MyDoubleMatrix R = matrix_multiply(matrix_multiply(matrix_transpose(AiMC), Q), AiMC);

        int n_fix = fix_idx.size();
        MyDoubleMatrix Rfp(n_fix, MyDoubleVector(n_coef * n_seg - n_fix, 0.0));
        MyDoubleMatrix Rpp(n_coef * n_seg - n_fix, MyDoubleVector(n_coef * n_seg - n_fix, 0.0));
        for (int i = 0; i < n_fix; ++i)
        {
            for (int j = 0; j < n_coef * n_seg - n_fix; ++j)
            {
                Rfp[i][j] = R[i][n_fix + j];
                Rpp[i][j] = R[n_fix + i][n_fix + j];
            }
        }

        MyDoubleMatrix dp = matrix_multiply(matrix_multiply(matrix_inverse(Rpp), matrix_transpose(Rfp)), matrix_transpose(df));
        MyDoubleVector dp_flat;
        for (int i = 0; i < dp.size(); ++i)
        {
            for (int j = 0; j < dp[i].size(); ++j)
            {
                dp_flat.push_back(dp[i][j]);
            }
        }

        MyDoubleVector df_dp = df;
        for (int i = 0; i < dp_flat.size(); ++i)
        {
            df_dp.push_back(dp_flat[i]);
        }

        MyDoubleMatrix p = matrix_multiply(AiMC, matrix_transpose(matrix_from_list(df_dp)));
        MyDoubleMatrix ploys(n_coef, MyDoubleVector(n_seg, 0.0));
        for (int i = 0; i < n_coef; ++i)
        {
            for (int j = 0; j < n_seg; ++j)
            {
                ploys[i][j] = p[i][j];
            }
        }

        return ploys;
    }

};

int main()
{
    // Usage example
    double R = 100.0;
    double targetSpeed = 100;
    double controllerFreq = 10.0;
    double initX = 0.0;
    double initY = 0.0;
    double initZ = 0.0;
    double distance = 1000.0;
    char Direction = 'x';
    /*print circle
    MyDoubleMatrix circleTrajectory= getCircle(R, targetSpeed, controllerFreq, initX, initY, initZ); // getCircleHuman目前调不出来
     for (int i = 0; i < circleTrajectory[0].size(); ++i) {
    std::cout << "x: " << circleTrajectory[0][i] << ", y: " << circleTrajectory[1][i] << ", z: " << circleTrajectory[2][i]
              << ", vx: " << circleTrajectory[3][i] << ", vy: " << circleTrajectory[4][i] << ", vz: " << circleTrajectory[5][i] << std::endl;
    */
    // print circleHuman
    MyDoubleMatrix circleTrajectory = getCircleHuman(R, targetSpeed, controllerFreq, initX, initY, initZ);
    // Print the trajectory
    for (int i = 0; i < circleTrajectory[0].size(); ++i)
    {
        std::cout << "x: " << circleTrajectory[0][i] << ", y: " << circleTrajectory[1][i] << ", z: " << circleTrajectory[2][i]
                  << ", vx: " << circleTrajectory[3][i] << ", vy: " << circleTrajectory[4][i] << ", vz: " << circleTrajectory[5][i] << std::endl;
    }

    // Matrix<double, 6, Dynamic> lineTrajectory = getLine(distance, targetSpeed, controllerFreq, initX, initY, initZ, "x");
    // std::cout<<lineTrajectory<<std::endl;

    return 0;
}
