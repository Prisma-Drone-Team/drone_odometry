#ifndef _utilities_hpp
#define _utilities_hpp

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>

// #include "geometry_msgs/PoseStamped.h"
// #include "geometry_msgs/TransformStamped.h"
// #include "nav_msgs/Odometry.h"

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

using namespace std;  //calling the standard directory
using namespace Eigen;

/*transformation matrix convention :
  R_m_b = R^{m}_{b} (m is superscript, b is subscript) R_m_b express cooridnates of frame b in frame m 

*/
 

namespace utilities{ 

	typedef Eigen::Matrix<double, 6, 1> Vector6d;
	typedef Eigen::Matrix<double, 6, 6> Matrix6d;

	inline bool isnan(double x){
		return (x!=x);
	}
	inline bool isnan(float x){
		return (x!=x);
	}

	inline double angleError(double target, double actual){
		double MAX_VALUE = 2.0*M_PI;
		double signedDiff = 0.0;
		double raw_diff = actual > target ? actual - target : target - actual;
		double mod_diff = fmod(raw_diff, MAX_VALUE); //equates rollover values. E.g 0 == 360 degrees in circle

		if(mod_diff > (MAX_VALUE/2) ){
			//There is a shorter path in opposite direction
			signedDiff = (MAX_VALUE - mod_diff);
			if(target>actual) signedDiff = signedDiff * -1;
		} else {
			signedDiff = mod_diff;
			if(actual>target) signedDiff = signedDiff * -1;
		}

		return signedDiff;

	}

	inline Matrix3d rotx(float alpha){
		Matrix3d Rx;
		Rx << 	1,0,0,
			0,cos(alpha),-sin(alpha),
			0,sin(alpha), cos(alpha);
		return Rx;
	}

	inline Matrix3d roty(float beta){
		Matrix3d Ry;
		Ry << 	cos(beta),0,sin(beta),
			0,1,0,
			-sin(beta),0, cos(beta);
		return Ry;
	}

	inline Matrix3d rotz(float gamma){
		Matrix3d Rz;
		Rz << 	cos(gamma),-sin(gamma),0,
			sin(gamma),cos(gamma),0,
			0,0, 1;
		return Rz;
	}


	inline Matrix4d rotx_T(float alpha){
		Matrix4d Tx = Matrix4d::Identity();
		Tx.block(0,0,3,3) = rotx(alpha);
		return Tx;
	}

	inline Matrix4d roty_T(float beta){
		Matrix4d Ty = Matrix4d::Identity();
		Ty.block(0,0,3,3) = roty(beta);
		return Ty;
	}

	inline Matrix4d rotz_T(float gamma){
		Matrix4d Tz = Matrix4d::Identity();
		Tz.block(0,0,3,3) = rotz(gamma);
		return Tz;
	}

	inline Matrix3d skew(Vector3d v)
	{
		Matrix3d S;
		S << 0,	-v[2],	 v[1],		//Skew-symmetric matrix
			v[2],	    0,	-v[0],
			-v[1],	 v[0], 	   0;
		return S;
	}


	inline Matrix3d L_matrix(Matrix3d R_d, Matrix3d R_e)
	{
		Matrix3d L = -0.5 * (skew(R_d.col(0))*skew(R_e.col(0)) + skew(R_d.col(1))*skew(R_e.col(1)) + skew(R_d.col(2))*skew(R_e.col(2)));
		return L;
	}



	inline Vector3d rotationMatrixError(Matrix4d Td, Matrix4d Te)
	{
		
		Matrix3d R_e = Te.block(0,0,3,3);		//Matrix.slice<RowStart, ColStart, NumRows, NumCols>();	
		Matrix3d R_d = Td.block(0,0,3,3);
		
		Vector3d eo = 0.5 * (skew(R_e.col(0))*R_d.col(0) + skew(R_e.col(1))*R_d.col(1) + skew(R_e.col(2))*R_d.col(2)) ;
		return eo;
	}


	inline Vector3d r2quat(Matrix3d R_iniz, float &eta)
	{
		Vector3d epsilon;
		int iu, iv, iw;

		if ( (R_iniz(0,0) >= R_iniz(1,1)) && (R_iniz(0,0) >= R_iniz(2,2)) )
		{
			iu = 0; iv = 1; iw = 2;
		}
		else if ( (R_iniz(1,1) >= R_iniz(0,0)) && (R_iniz(1,1) >= R_iniz(2,2)) )
		{
			iu = 1; iv = 2; iw = 0;
		}
		else
		{
			iu = 2; iv = 0; iw = 1;
		}

		float r = sqrt(1 + R_iniz(iu,iu) - R_iniz(iv,iv) - R_iniz(iw,iw));
		Vector3d q;
		q <<  0,0,0;
		if (r>0)
		{
		float rr = 2*r;
		eta = (R_iniz(iw,iv)-R_iniz(iv,iw)/rr);
		epsilon[iu] = r/2;
		epsilon[iv] = (R_iniz(iu,iv)+R_iniz(iv,iu))/rr;
		epsilon[iw] = (R_iniz(iw,iu)+R_iniz(iu,iw))/rr;
		}
		else
		{
		eta = 1;
		epsilon << 0,0,0;
		}
		return epsilon;
	}


	inline Vector4d rot2quat(Matrix3d R){

		float m00, m01, m02, m10, m11, m12, m20, m21, m22;

		m00 = R(0,0);
		m01 = R(0,1);
		m02 = R(0,2);
		m10 = R(1,0);
		m11 = R(1,1);
		m12 = R(1,2);
		m20 = R(2,0);
		m21 = R(2,1);
		m22 = R(2,2);

		float tr = m00 + m11 + m22;
		float qw, qx, qy, qz, S;
		Vector4d quat;

		if (tr > 0) { 
		  S = sqrt(tr+1.0) * 2; // S=4*qw 
		  qw = 0.25 * S;
		  qx = (m21 - m12) / S;
		  qy = (m02 - m20) / S; 
		  qz = (m10 - m01) / S; 
		} else if ((m00 > m11)&(m00 > m22)) { 
		  S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx 
		  qw = (m21 - m12) / S;
		  qx = 0.25 * S;
		  qy = (m01 + m10) / S; 
		  qz = (m02 + m20) / S; 
		} else if (m11 > m22) { 
		  S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
		  qw = (m02 - m20) / S;
		  qx = (m01 + m10) / S; 
		  qy = 0.25 * S;
		  qz = (m12 + m21) / S; 
		} else { 
		  S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
		  qw = (m10 - m01) / S;
		  qx = (m02 + m20) / S;
		  qy = (m12 + m21) / S;
		  qz = 0.25 * S;
		}

		quat << qw, qx, qy, qz;
		return quat;

	}
	
	
		 //Matrix ortonormalization
	inline Matrix3d matrixOrthonormalization(Matrix3d R){

		SelfAdjointEigenSolver<Matrix3d> es(R.transpose()*R);
		Vector3d D = es.eigenvalues();
		Matrix3d V = es.eigenvectors();
		R = R*((1/sqrt(D(0)))*V.col(0)*V.col(0).transpose() + (1/sqrt(D(1)))*V.col(1)*V.col(1).transpose() + (1/sqrt(D(2)))*V.col(2)*V.col(2).transpose());

		return R;
	}




	//******************************************************************************
	inline Vector3d quaternionError(Matrix4d Tbe, Matrix4d Tbe_d)
	{
		float eta, eta_d;
		Matrix3d R = Tbe.block(0,0,3,3);		//Matrix.slice<RowStart, ColStart, NumRows, NumCols>();	
		Vector3d epsilon = r2quat(R, eta);
		Matrix3d R_d = Tbe_d.block(0,0,3,3);
		Vector3d epsilon_d = r2quat(R_d, eta_d);
		Matrix3d S = skew(epsilon_d);
		Vector3d eo = eta*epsilon_d-eta_d*epsilon-S*epsilon;
		return eo;
	}


	inline Vector3d versorError(Vector3d P_d, Matrix3d Tbe_e, float &theta)
	{
		Vector3d P_e = Tbe_e.block(0,3,3,1);
		Vector3d u_e = Tbe_e.block(0,0,3,1);

		Vector3d u_d = P_d - P_e;
		u_d = u_d/ u_d.norm();
		
		Vector3d r = skew(u_e)*u_d;
		float nr = r.norm();
			
	  if (nr >0){
			r = r/r.norm();
			

			//be carefult to acos( > 1 )
			float u_e_d = u_e.transpose()*u_d;		
			if( fabs(u_e_d) <= 1.0 ) {
				theta = acos(u_e.transpose()*u_d);
			}
			else {
				theta = 0.0;
			}
			
			Vector3d error = r*sin(theta);
			return error;
		}else{
			theta = 0.0;
			return Vector3d::Zero();
		}
	}


	/*Matrix3d utilities::rotation ( float theta, Vector3d r ) {
	     
	    Matrix3d R = Zeros;
	 
	    R[0][0] = r[0]*r[0]*(1-cos(theta)) + cos(theta);
	    R[1][1] = r[1]*r[1]*(1-cos(theta)) + cos(theta);
	    R[2][2] = r[2]*r[2]*(1-cos(theta)) + cos(theta);
	 
	    R[0][1] = r[0]*r[1]*(1-cos(theta)) - r[2]*sin(theta);
	    R[1][0] = r[0]*r[1]*(1-cos(theta)) + r[2]*sin(theta);
	 
	    R[0][2] = r[0]*r[2]*(1-cos(theta)) + r[1]*sin(theta);
	    R[2][0] = r[0]*r[2]*(1-cos(theta)) - r[1]*sin(theta);
	     
	    R[1][2] = r[1]*r[2]*(1-cos(theta)) - r[0]*sin(theta);
	    R[2][1] = r[1]*r[2]*(1-cos(theta)) + r[0]*sin(theta);
	     
	 
	    return R;
	}*/


	inline Matrix3d XYZ2R(Vector3d angles) {
	  	
	  	Matrix3d R = Matrix3d::Zero(); 
	  	Matrix3d R1 = Matrix3d::Zero(); 
	  	Matrix3d R2 = Matrix3d::Zero(); 
	  	Matrix3d R3 = Matrix3d::Zero();

		float cos_phi = cos(angles[0]);
		float sin_phi = sin(angles[0]);
		float cos_theta = cos(angles[1]);
		float sin_theta = sin(angles[1]);
		float cos_psi = cos(angles[2]);
		float sin_psi = sin(angles[2]);

		R1  << 1, 0      , 0, 
			        0, cos_phi, -sin_phi, 
			        0, sin_phi, cos_phi;

		R2  << cos_theta , 0, sin_theta, 
			        0        , 1, 0       , 
			        -sin_theta, 0, cos_theta;

		R3  << cos_psi, -sin_psi, 0, 
			        sin_psi, cos_psi , 0,
			        0      , 0       , 1;

		R = R1*R2*R3;

		return R;
	}

	// This method computes the XYZ Euler angles from the Rotational matrix R.
	inline Vector3d R2XYZ(Matrix3d R) {
		double phi=0.0, theta=0.0, psi=0.0;
		Vector3d XYZ = Vector3d::Zero();
		
		theta = asin(R(0,2));
		
		if(fabsf(cos(theta))>pow(10.0,-10.0))
		{
			phi=atan2(-R(1,2)/cos(theta), R(2,2)/cos(theta));
			psi=atan2(-R(0,1)/cos(theta), R(0,0)/cos(theta));
		}
		else
		{
			if(fabsf(theta-M_PI/2.0)<pow(10.0,-5.0))
			{
				psi = 0.0;
				phi = atan2(R(1,0), R(2,0));
				theta = M_PI/2.0;
			}
			else
			{
				psi = 0.0;
				phi = atan2(-R(1,0), R(2,0));
				theta = -M_PI/2.0;
			}
		}
		
		XYZ << phi,theta,psi;
		return XYZ;
	}

	inline Matrix3d angleAxis2Rot(Vector3d ri, float theta){
	Matrix3d R;
	R << ri[0]*ri[0] * (1 - cos(theta)) + cos(theta)           , ri[0] * ri[1] * (1 - cos(theta)) - ri[2] * sin(theta) , ri[0] * ri[2] * (1 - cos(theta)) + ri[1] * sin(theta),
	         ri[0] * ri[1] * (1 - cos(theta)) + ri[2] * sin(theta) , ri[1]*ri[1] * (1 - cos(theta)) + cos(theta)           , ri[1] * ri[2] * (1 - cos(theta)) - ri[0] * sin(theta),
	         ri[0] * ri[2] * (1 - cos(theta)) - ri[1] * sin(theta) , ri[1] * ri[2] * (1 - cos(theta)) + ri[0] * sin(theta) , ri[2]*ri[2] * (1 - cos(theta)) + cos(theta);

	return R;

	}

	inline Matrix3d versor2rotm(Vector3d v){ //used in koala pipe detection
		v = v/v.norm();
		double rx = v[0];
		double ry = v[1];
		double rz = v[2];
		double sa = ry/sqrt(rx*rx+ry*ry);
		double ca = rx/sqrt(rx*rx+ry*ry);
		double sb = sqrt(rx*rx+ry*ry);
		double cb = rz;
	
		Matrix3d Rza;
		Matrix3d Ryb;
		Matrix3d R;
		Rza << ca, -sa, 0,
			   sa,  ca, 0,
			   0,  0 , 1 ;
		Ryb << cb,  0,  sb,
			   0,   1,  0,
			   -sb, 0,  cb;
		R =  Rza*Ryb;

		Vector3d nx = R.block(0,0,3,1);
		Vector3d ny = R.block(0,1,3,1);
		Vector3d nz = R.block(0,2,3,1);
		R.block(0,0,3,1)= nz;
		R.block(0,1,3,1)= -ny;
		R.block(0,2,3,1)= nx;
		return R;

		// //correct rotation
		// Vector3d x_ref;
		// x_ref << 1.0, 0.0, 0.0;

		// Vector3d x = R.block(0,0,3,1); 
		// Vector3d xp = x;
		// Vector3d zp = x.cross(x_ref);
		// zp = zp/zp.norm();

		// Vector3d yp = xp.cross(zp);
		// yp = yp/yp.norm();


		// R.block(0,0,3,1)= xp;
		// R.block(0,1,3,1)= yp;
		// R.block(0,2,3,1)= zp;
		// return R;

	}



	inline Vector3d butt_filter(Vector3d x, Vector3d x1, Vector3d x2, float omega_n, float zita, float ctrl_T){
		//applico un filtro di Butterworth del secondo ordine (sfrutto Eulero all'indietro)
		return x1*(2.0 + 2.0*omega_n*zita*ctrl_T)/(omega_n*omega_n*ctrl_T*ctrl_T + 2.0*omega_n*zita*ctrl_T + 1.0) - x2/(omega_n*omega_n*ctrl_T*ctrl_T + 2.0*omega_n*zita*ctrl_T + 1.0) + x*(omega_n*omega_n*ctrl_T*ctrl_T)/(omega_n*omega_n*ctrl_T*ctrl_T + 2.0*omega_n*zita*ctrl_T + 1.0);
	}



	/// converts a rate in Hz to an integer period in ms.
	inline uint16_t rateToPeriod(const float & rate) {
		if (rate > 0)
			return static_cast<uint16_t> (1000.0 / rate);
		else
			return 0;
	}


	    //Quaternion to rotration Matrix
	inline Matrix3d QuatToMat(Vector4d Quat){
		Matrix3d Rot;
		float s = Quat[0];
		float x = Quat[1];
		float y = Quat[2];
		float z = Quat[3];
		Rot << 1-2*(y*y+z*z),2*(x*y-s*z),2*(x*z+s*y),
		2*(x*y+s*z),1-2*(x*x+z*z),2*(y*z-s*x),
		2*(x*z-s*y),2*(y*z+s*x),1-2*(x*x+y*y);
		return Rot;
	}


	/*ROS2 odometry and tf conversions*/

	inline Matrix4d T_from_odom(nav_msgs::msg::Odometry od){
		Matrix4d T = Matrix4d::Identity();
		Vector4d q(od.pose.pose.orientation.w,od.pose.pose.orientation.x,od.pose.pose.orientation.y,od.pose.pose.orientation.z);
		Vector4d p(od.pose.pose.position.x,od.pose.pose.position.y,od.pose.pose.position.z,1.0);
		T.block(0,0,3,3) = QuatToMat(q);
		T.block(0,3,4,1) = p;
		return T;
	}

	inline Matrix4d T_from_tf(geometry_msgs::msg::TransformStamped tf){
		Matrix4d T = Matrix4d::Identity();
		Vector4d q(tf.transform.rotation.w,tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z);
		Vector4d p(tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z,1.0);
		T.block(0,0,3,3) = QuatToMat(q);
		T.block(0,3,4,1) = p;
		return T;
	}

	inline void tf_from_T(geometry_msgs::msg::TransformStamped& tf ,Matrix4d T){

		Matrix3d R =  T.block(0,0,3,3);
		Vector3d p = T.block(0,3,3,1);
		Vector4d q = rot2quat(R);
		tf.transform.rotation.w = q(0);
		tf.transform.rotation.x = q(1);
		tf.transform.rotation.y = q(2);
		tf.transform.rotation.z = q(3);

		tf.transform.translation.x = p(0);
		tf.transform.translation.y = p(1);
		tf.transform.translation.z = p(2);

	}

	inline void tf_from_odom(geometry_msgs::msg::TransformStamped& tf, const nav_msgs::msg::Odometry& od){
		tf.header.stamp = od.header.stamp;
		tf.header.frame_id = od.header.frame_id;
		tf.child_frame_id = od.child_frame_id;

		tf.transform.rotation.w = od.pose.pose.orientation.w;
		tf.transform.rotation.x = od.pose.pose.orientation.x;
		tf.transform.rotation.y = od.pose.pose.orientation.y;
		tf.transform.rotation.z = od.pose.pose.orientation.z;

		tf.transform.translation.x = od.pose.pose.position.x;
		tf.transform.translation.y = od.pose.pose.position.y;
		tf.transform.translation.z = od.pose.pose.position.z;
	}

	inline Vector6d twits_from_odom(const nav_msgs::msg::Odometry& od){
		Vector6d tw;
		tw << od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.linear.z,  
			  od.twist.twist.angular.x, od.twist.twist.angular.y, od.twist.twist.angular.z; 
		return tw;	  
	}

	// inline Matrix6d adjoint(const Matrix4d & T){
	// 	Matrix6d Adj;
	// 	Matrix3d R = T.block(0,0,3,3);
	// 	Vector3d r = T.block(0,3,3,1);

	// 	Adj.block(0,0,3,3) = R;
	// 	Adj.block(0,3,3,3) = R*skew(r);
	// 	Adj.block(3,0,3,3) = Matrix3d::Zero();
	// 	Adj.block(3,3,3,3)	= R_b_a;
	// }

	inline void odom_from_tf_and_twist(nav_msgs::msg::Odometry& od, const Matrix4d & T_o_b, const Vector6d & v_b_b){
		
		Matrix3d R =  T_o_b.block(0,0,3,3);
		Vector3d p = T_o_b.block(0,3,3,1);
		Vector4d q = rot2quat(R);

		od.pose.pose.position.x = p[0];
		od.pose.pose.position.y = p[1];
		od.pose.pose.position.z = p[2];
		od.pose.pose.orientation.w = q[0];
		od.pose.pose.orientation.x = q[1];
		od.pose.pose.orientation.y = q[2];
		od.pose.pose.orientation.z = q[3];

		od.twist.twist.linear.x = v_b_b[0];
		od.twist.twist.linear.y = v_b_b[1];
		od.twist.twist.linear.z = v_b_b[2];
		od.twist.twist.angular.x = v_b_b[3];
		od.twist.twist.angular.y = v_b_b[4];
		od.twist.twist.angular.z = v_b_b[5];
	}

	inline void odom_from_tf(nav_msgs::msg::Odometry& od, const Matrix4d & T_o_b){
		
		Matrix3d R =  T_o_b.block(0,0,3,3);
		Vector3d p = T_o_b.block(0,3,3,1);
		Vector4d q = rot2quat(R);

		od.pose.pose.position.x = p[0];
		od.pose.pose.position.y = p[1];
		od.pose.pose.position.z = p[2];
		od.pose.pose.orientation.w = q[0];
		od.pose.pose.orientation.x = q[1];
		od.pose.pose.orientation.y = q[2];
		od.pose.pose.orientation.z = q[3];
	}

	inline Matrix4d T_inverse(Matrix4d T){
		Matrix4d Tinv = Matrix4d::Identity();
		
		Matrix3d R = T.block(0,0,3,3).transpose(); //Rinv
		Vector3d o = T.block(0,3,3,1); //col vect

		Tinv.block(0,0,3,3) = R;
		Tinv.block(0,3,3,1) = -R*o;

		return Tinv;
	}

	inline Matrix4d T_clear_translation(Matrix4d T){
		
		// T.block(0,0,3,3); //Rinv
		T.block(0,3,3,1) = Vector3d::Zero().transpose(); //col vect

		return T;
	}

	
	
	inline Vector6d rotate_twist(const Vector6d &v_a_a, const Matrix4d & T_b_a){
		Matrix3d R_b_a = T_b_a.block(0,0,3,3);
		Vector3d r_b_ab = T_b_a.block(0,3,3,1);
		Matrix6d Adj;

		Adj.block(0,0,3,3) = R_b_a;
		Adj.block(0,3,3,3) = skew(r_b_ab)*R_b_a; //skew(-r_b_ab)*R_b_a;
		Adj.block(3,0,3,3) = Matrix3d::Zero();
		Adj.block(3,3,3,3)	= R_b_a;

		Vector6d v_b_b = Adj*v_a_a;
		return v_b_b;
	}

	inline Matrix6d rotate_twist_cov(const Matrix6d &cov_a_a, const Matrix4d & T_b_a){
		Matrix3d R_b_a = T_b_a.block(0,0,3,3);
		Vector3d r_b_ab = T_b_a.block(0,3,3,1);
		Matrix6d Adj;

		Adj.block(0,0,3,3) = R_b_a;
		Adj.block(0,3,3,3) = skew(r_b_ab)*R_b_a; //skew(-r_b_ab)*R_b_a;
		Adj.block(3,0,3,3) = Matrix3d::Zero();
		Adj.block(3,3,3,3)	= R_b_a;

		Matrix6d cov_b_b = Adj*cov_a_a*Adj.transpose();
		return cov_b_b;
	}

	inline Matrix6d rotate_pose_cov(const Matrix6d &cov_a_a, const Matrix4d & T_b_a){
		Matrix3d R_b_a = T_b_a.block(0,0,3,3);
		// Vector3d r_b_ab = T_b_a.block(0,3,3,1);
		Matrix6d Adj;

		Adj.block(0,0,3,3) = R_b_a;
		Adj.block(0,3,3,3) = Matrix3d::Zero();
		Adj.block(3,0,3,3) = Matrix3d::Zero();
		Adj.block(3,3,3,3)	= R_b_a;

		Matrix6d cov_b_b = Adj*cov_a_a*Adj.transpose();
		return cov_b_b;
	}

	inline Matrix6d cov_to_mat(const std::array<double, 36> & msg_cov){
		Eigen::Map<const Matrix6d> matrix_map(msg_cov.data());
    	return matrix_map; 
	}

	inline void mat_to_cov(std::array<double, 36> & msg_cov, Matrix6d cov){
		msg_cov = {
			cov(0,0), cov(0,1), cov(0,2),cov(0,3), cov(0,4), cov(0,5),
			cov(1,0), cov(1,1), cov(1,2),cov(1,3), cov(1,4), cov(1,5),
			cov(2,0), cov(2,1), cov(2,2),cov(2,3), cov(2,4), cov(2,5),
			cov(3,0), cov(3,1), cov(3,2),cov(3,3), cov(3,4), cov(3,5),
			cov(4,0), cov(4,1), cov(4,2),cov(4,3), cov(4,4), cov(4,5),
			cov(5,0), cov(5,1), cov(5,2),cov(5,3), cov(5,4), cov(5,5),
		};
	}
	
	static inline const Matrix3d R_enu_ned = (Matrix3d() << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0).finished();
	static inline const Matrix3d R_frd_flu = (Matrix3d() << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0).finished();

	static inline const Matrix3d R_ned_enu = R_enu_ned.transpose(); //is the same !!
	static inline const Matrix3d R_flu_frd = R_frd_flu.transpose();

	inline float rad2deg(float rad){
		float deg;
		deg = 180.0*rad/M_PI;
		return deg;
	}
	
	inline float deg2rad(float deg){
		float rad;
		rad = M_PI*deg/180.0;
		return rad;
	}


	/*vinsco addon*/

	inline Vector3d MatToRpy( Matrix3d R ) {

		Vector3d rpy;


		rpy(0) = atan2(    R(2,1), R(2,2) );  // roll
		rpy(1) = atan2( -R(2,0), sqrt( R(2,1)*R(2,1) + R(2,2)*R(2,2))); 	//pitch
		rpy(2) = atan2(R(1,0),R(0,0)); 	//yaw
		
		return rpy;
	}


	
	inline Matrix3d RpyToMat( Vector3d rpy) {
		double r, p, y;
		r = rpy(0);
		p = rpy(1);
		y = rpy(2);

		double cf = cos(y);
		double sf = sin(y);

		double ct = cos(p);
		double st = sin(p);

		double cp = cos(r);
		double sp = sin(r);

    	Matrix3d R;
    	R << cf*ct, cf*st*sp-sf*cp, cf*st*cp + sf*sp,
            sf*ct, sf*st*sp+cf*cp, sf*st*cp - cf*sp,
            -st, ct*sp, ct*cp;

	    return R;
  	}

	  inline Vector4d quat_product( Vector4d q1, Vector4d q2 ) {
		Vector4d q_out;

		q_out[0] = q1[0]*q2[0] - q1.block<3,1>(1,0).transpose()*q2.block<3,1>(1,0);
		q_out.block<3,1>(1,0) = q1[0]*q2.block<3,1>(1,0) + q2[0]*q1.block<3,1>(1,0) + q1.block<3,1>(1,0).cross(q2.block<3,1>(1,0));
		return q_out;
	}

    inline Matrix3d Q2R( Vector4d q ) //Quaternion in form [w,x,y,z]
    {
        Eigen::Matrix3d R;
		R(0,0) = 2*(q[0]*q[0]+q[1]*q[1])-1;
		R(0,1) = 2*(q[1]*q[2]-q[0]*q[3]);
		R(0,2) = 2*(q[1]*q[3]+q[0]*q[2]);
		R(1,0) = 2*(q[1]*q[2]+q[0]*q[3]);
		R(1,1) = 2*(q[0]*q[0]+q[2]*q[2])-1;
		R(1,2) = 2*(q[2]*q[3]-q[0]*q[1]);
		R(2,0) = 2*(q[1]*q[3]-q[0]*q[2]);
		R(2,1) = 2*(q[2]*q[3]+q[0]*q[1]);
		R(2,2) = 2*(q[0]*q[0]+q[3]*q[3])-1;
        // R << 2*(q[0]*q[0]+q[1]*q[1])-1, 2*(q[1]*q[2]-q[0]*q[3]), 2*(q[1]*q[3]+q[0]*q[2]),
        //     2*(q[1]*q[2]+q[0]*q[3]), 2*(q[0]*q[0]+q[2]*q[2])-1, 2*(q[2]*q[3]-q[0]*q[1]),
        //     2*(q[1]*q[3]-q[0]*q[2]), 2*(q[2]*q[3]+q[0]*q[1]), 2*(q[0]*q[0]+q[3]*q[3])-1;

        return R;
    }
	
	inline Vector4d R2Q( Matrix3d R)
	{
		Vector4d q;
		double tr = R.trace();
		if( tr > 0.0 )
		{
			q[0] = 0.5*sqrt(1+tr);
			q[1] = (R(2,1)-R(2,0))/(4*q[0]);
			q[2] = (R(0,2)-R(2,0))/(4*q[0]);
			q[3] = (R(1,0)-R(0,1))/(4*q[0]);
		}
		else if(R(0,0)>R(1,1) && R(0,0)>R(2,2))
			{
				q[1] = 0.5*sqrt(1+R(0,0)-R(1,1)-R(2,2));
				q[0] = (R(2,1)-R(1,2))/(4*q[1]);
				q[2] = (R(0,1)-R(1,0))/(4*q[1]);
				q[3] = (R(0,2)-R(2,0))/(4*q[1]);
			}
		else if ( R(1,1)>R(0,0) && R(1,1)>R(2,2) )
			{
				q[2] = 0.5*sqrt(1+R(1,1)-R(0,0)-R(2,2));
				q[0] = (R(0,2)-R(2,0))/(4*q[2]);
				q[1] = (R(0,1)-R(1,0))/(4*q[2]);
				q[3] = (R(1,2)-R(2,1))/(4*q[2]);
			}
		else if ( R(2,2)>R(1,1) && R(2,2)>R(0,0) )
			{
				q[3] = 0.5*sqrt(1+R(2,2)-R(0,0)-R(1,1));
				q[0] = (R(1,0)-R(0,1))/(4*q[3]);
				q[1] = (R(0,2)-R(2,0))/(4*q[3]);
				q[2] = (R(1,2)-R(2,1))/(4*q[3]);
			}	

		return q;
	}


}

#endif