#ifndef _MYKALMAN_H
#define _MYKALMAN_H
#endif
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Dense>

/**
 * @brief 卡尔曼滤波器
 * @param stateSize 状态矩阵大小
 * @param meaSize 测量矩阵大小
 * @param uSize 控制矩阵大小
 * @param x 初始状态
 * @param z 测量值
 * @param A 状态转移矩阵
 * @param B 控制矩阵
 * @param u 控制值
 * @param P 协方差矩阵
 * @param H 测量矩阵
 * @param R 测量噪声协方差
 * @param Q 过程噪声协方差
 */
class KalmanFilter{
private:
	int stateSize; //state variable's dimenssion
	int measSize; //measurement variable's dimession
	int uSize; //control variables's dimenssion
	Eigen::VectorXd x;
	Eigen::VectorXd z;
	Eigen::MatrixXd A;
	Eigen::MatrixXd B;
	Eigen::VectorXd u;
	Eigen::MatrixXd P;//coveriance
	Eigen::MatrixXd H;
	Eigen::MatrixXd R;//measurement noise covariance
	Eigen::MatrixXd Q;//process noise covariance

public:
	/**
	 * @brief 构造函数
	 * @param stateSize_ 状态矩阵大小
	 * @param measSize_ 测量矩阵大小
	 * @param uSize_ 控制矩阵大小
	 */
	KalmanFilter(int stateSize_, int measSize_,int uSize_);

	KalmanFilter() = default;
	~KalmanFilter() = default;
	/**
	 * @brief 初始化
	 * @param x_ 初始状态
	 * @param P_ 协方差矩阵
	 * @param R_ 测量噪声协方差
	 * @param Q_ 过程噪声协方差
	 */
	void init(Eigen::VectorXd &x_, Eigen::MatrixXd& P_,Eigen::MatrixXd& R_, Eigen::MatrixXd& Q_);

	/**
	 * @brief 预测，不带控制
	 * @param A_ 状态转移矩阵
	 */
	Eigen::VectorXd predict(Eigen::MatrixXd& A_);

	/**
	 * @brief 预测，带控制
	 * @param A_ 状态转移矩阵
	 * @param B_ 控制矩阵
	 * @param u_ 控制值
	 */
	Eigen::VectorXd predict(Eigen::MatrixXd& A_, Eigen::MatrixXd &B_, Eigen::VectorXd &u_);

	/**
	 * @brief 更新
	 * @param H_ 测量矩阵
	 * @param z_meas 测量值
	 */
	void update(Eigen::MatrixXd& H_,Eigen::VectorXd z_meas);
};