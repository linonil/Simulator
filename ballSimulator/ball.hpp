#ifndef _BALL_HPP_
#define _BALL_HPP_

#include <opencv2/core.hpp>

class Ball
{
private:
/********ATTRIBUTES****************************************************/
	float mR;
	
	cv::Mat mK, mKInv;
		
	cv::Mat mRefImage;
	
	bool mfUseColorMode = false;
	cv::Vec3b mColorMode;
	
	/**PRIVATE METHODS*/
	bool inBall(cv::Mat, cv::Mat);
	void computeColorMode();
	cv::Vec3b getRandomColorFromRefImage();
	
public:
/********CONSTRUCTORS & DESTRUCTORS************************************/
	Ball(float, cv::Mat, cv::Mat);
	//~Ball();
	
/********METHODS*******************************************************/	
	void insertBall(cv::Mat &, cv::Mat);
	cv::Vec3b colorMode();
	void useColorMode(bool);
};

#endif //_BALL_HPP_
