#include "ball.hpp"

#include <iostream>

Ball::Ball(float radius, cv::Mat refImage, cv::Mat K)
{
	mR = radius;
	mRefImage = refImage.clone(); 
	mK = K.clone();
	mKInv = mK.inv();
	
	computeColorMode();
}

bool Ball::inBall(cv::Mat m, cv::Mat X0)
{					
	//IF DELTA = b*b - 4*a*c >= 0 SOLUTION EXISTS RETURN TRUE
	return ((cv::Mat)(m.t()*mKInv.t()*X0*m.t()*mKInv.t()*X0 - 
		m.t()*mKInv.t()*mKInv*m*(X0.t()*X0 - mR*mR))).at<float>(0) >= 0;
}

void Ball::computeColorMode()
{
	//CREATE VECTORIZED HISTOGRAM
	cv::Mat hist = cv::Mat(256*256*256, 1, CV_32S); hist = 0;
	
	for(uint i = 0; i < (uint)mRefImage.rows; i++)
		for(uint j = 0; j < (uint)mRefImage.cols; j++)
		{
			cv::Vec3b c = mRefImage.at<cv::Vec3b>(cv::Point(i, j));
			//DISCARD WHITE PIXELS
			if(c[0] > 245 && c[1] > 240 && c[2] > 240)
				continue;
			hist.at<int>(256*256*c[0] + 256*c[1] + c[2])++;
		}
		
	//GET MAX BIN
	int maxIdx;
	cv::minMaxIdx(hist, NULL, NULL, NULL, &maxIdx);
	
	//SET COLOR
	mColorMode[0] = maxIdx/(256*256);
	mColorMode[1] = maxIdx%(256*256)/256;
	mColorMode[2] = maxIdx%(256*256)%256;
}

cv::Vec3b Ball::colorMode(){return mColorMode;}

void Ball::useColorMode(bool flag){mfUseColorMode = flag;}

cv::Vec3b Ball::getRandomColorFromRefImage()
{						
	cv::Vec3b c;
	cv::Mat ia = cv::Mat(1, 1, CV_32S);
	cv::Mat ja = cv::Mat(1, 1, CV_32S);
	
	//PICK A COLOR FROM REFERENCE IMAGE DIFFERENT FROM WHITE
	do
	{
		cv::randu(ia, 0, mRefImage.cols);
		cv::randu(ja, 0, mRefImage.rows);
		uint i = ia.at<int>(0), j = ja.at<int>(0);
		c = mRefImage.at<cv::Vec3b>(cv::Point(i, j));	
	}while(c[0] > 240 && c[1] > 240 && c[2] > 240);

	return c;
}

void Ball::insertBall(cv::Mat &image, cv::Mat t)
{
	cv::Mat m = cv::Mat(3, 1, CV_32F);	//PIXEL
	cv::Mat nx = cv::Mat(1, 1, CV_32F); //UNIFORM DISTRIBUTION
	cv::Mat ny = cv::Mat(1, 1, CV_32F); //UNIFORM DISTRIBUTION

	for(uint j = 0; j < (uint)image.rows; j++)
		for(uint i = 0; i < (uint)image.cols; i++)
		{
			//GET PERCENTAGE OF BALL INSIDE OF PIXEL
			uint ct = 0;
			cv::randu(nx, 0, 1), cv::randu(ny, 0, 1);
			for(uint w = 0; w < (uint)nx.rows; w++)
			{
				//SET PIXEL
				m.at<float>(0) = i + nx.at<float>(w); 
				m.at<float>(1) = j + ny.at<float>(w); 
				m.at<float>(2) = 1;
				if(inBall(m, t.t()))
					ct++;
			}
			
			float a = ct*1.0/nx.rows;
			
			//MAKE LINEAR INTERPOLATION
			cv::Mat c = cv::Mat(3, 1, CV_32F); //OBJECT COLOR
			if(mfUseColorMode)
			{
				c.at<float>(0) = mColorMode[0];
				c.at<float>(1) = mColorMode[1];
				c.at<float>(2) = mColorMode[2];
			}
			else
			{				
				cv::Vec3b r = getRandomColorFromRefImage();
				c.at<float>(0) = r[0];
				c.at<float>(1) = r[1];
				c.at<float>(2) = r[2];
			}
					
			//BACKGROUND COLOR
			cv::Vec3b bv = image.at<cv::Vec3b>(cv::Point(i, j));
			cv::Mat b = cv::Mat(3, 1, CV_32F);
			b.at<float>(0) = bv[0];
			b.at<float>(1) = bv[1];
			b.at<float>(2) = bv[2];
			
			//WRITE TO IMAGE
			c = c*a + (1-a)*b; 
			image.at<cv::Vec3b>(cv::Point(i, j)) = 
				{(uint8_t)c.at<float>(0), 
				 (uint8_t)c.at<float>(1), 
				 (uint8_t)c.at<float>(2)};
		}
}
