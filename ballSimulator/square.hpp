#ifndef _SQUARE_HPP_
#define _SQUARE_HPP_

#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>

#define ARUCO_SIZE 200 //FLOAT

class Square
{
private:
/********ATTRIBUTES****************************************************/
	cv::Mat mX0 = cv::Mat(3, 1, CV_32F); //SQUARE CENTER
	cv::Mat mN = cv::Mat(3, 1, CV_32F); //SQUARE NORMAL
	cv::Mat mSVs[4];	//SQUARE VERTICES

	cv::Vec3b mSqColor;
	
	cv::Mat mK, mKInv, mSInv;

	cv::Mat mMCs[4]; // MARKERS CENTER
	cv::Mat mMVs[4][4]; // MARKERS VERTICES
	cv::Mat mMInv[4]; // MARKER MATRICES

	float mMarkerSize;

	/**ArUCo Markers*/
 	cv::Mat mMarkerIm[4];
	uint mMarkerID[4] = {0, 1, 2, 3};
	
	cv::Ptr<cv::aruco::Dictionary> mDict = 
		cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);	

	/**PRIVATE METHODS*/
	void inSquare(cv::Mat, cv::Mat, cv::Mat, float *);
	void inMarker(cv::Mat, uint, float *);
	bool markerPtIsBlack(float *, uint);
	int getColorIdx(std::vector<cv::Vec3b> &, cv::Vec3b);
	void colorInterp(float, cv::Vec3b, cv::Point, cv::Mat &);
	void colorInterp(float, cv::Point, cv::Mat &im);
	cv::Mat Rodrigues(cv::Mat, cv::Mat, double);	
	
public:
/********CONSTRUCTORS & DESTRUCTORS************************************/
	Square(float *, float *, float *, float, cv::Vec3b, cv::Mat);
	//~Square();	
/********METHODS*******************************************************/
	void insertSquare(cv::Mat &);
};

#endif //_SQUARE_HPP_



