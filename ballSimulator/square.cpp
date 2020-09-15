#include "square.hpp"

#define PI 3.1415926535897932384626433832795

cv::Mat Square::Rodrigues(cv::Mat v, cv::Mat n, double t)
{
	return v*cos(t) + (n.cross(v))*sin(t) + n*(n.t()*v)*(1 - cos(t));
}

Square::Square(float *sCenter, float *sNormal, float *V1, 
			   float markerSize, cv::Vec3b pColor, cv::Mat K)
{
	//GET SQUARE CENTER
	*mX0.ptr<float>(0, 0) = sCenter[0];
	*mX0.ptr<float>(1, 0) = sCenter[1];
	*mX0.ptr<float>(2, 0) = sCenter[2];
	
	//GET NORMAL TO SQUARE
	*mN.ptr<float>(0, 0) = sNormal[0];
	*mN.ptr<float>(1, 0) = sNormal[1];
	*mN.ptr<float>(2, 0) = sNormal[2];
	
	//NORMALIZE VETOR
	*mN.ptr<float>(0, 0) /= norm(mN);
	*mN.ptr<float>(1, 0) /= norm(mN);
	*mN.ptr<float>(2, 0) /= norm(mN);
	
	//GET FIRST VERTICE
	cv::Mat Va = cv::Mat(3, 1, CV_32F); 
	*Va.ptr<float>(0, 0) = V1[0];
	*Va.ptr<float>(1, 0) = V1[1];
	*Va.ptr<float>(2, 0) = V1[2];
	
	//GET SQUARE VERTICES
	for(uint i = 0; i < 4; i++)
		mSVs[i] = mX0 + Rodrigues(Va - mX0, mN, i*PI/2.0);

	//GET SQUARE MATRIX
	cv::Mat S = cv::Mat(3, 3, CV_32F);
	S.col(0) = 2*mSVs[0] - mSVs[0];
	S.col(1) = mSVs[1] - mSVs[0];
	S.col(2) = mSVs[3] - mSVs[0];

	mSInv = S.inv();

	mSqColor = pColor;
	
	mK = K.clone();
	mKInv = mK.inv();	


/********MARKERS*******************************************************/
	//GET MARKER SIZE | SIZE BETWEEN [0, 1]
	if(markerSize > 1 || markerSize < 0) 
		markerSize = 1;

	mMarkerSize = markerSize; 

	//GET MARKERS CENTER
	cv::Mat aux1 = (mX0 + mSVs[0])/2.0; 
	cv::Mat aux2 = (mX0 + mSVs[1])/2.0;
	cv::Mat aux3 = (mX0 + mSVs[2])/2.0; 
	cv::Mat aux4 = (mX0 + mSVs[3])/2.0;		
	
	mMCs[0] = aux1.clone();
	mMCs[1] = aux2.clone();
	mMCs[2] = aux3.clone();
	mMCs[3] = aux4.clone();

	//GET FIRST VERTICES
	cv::Mat VMa[4];
	for(uint i = 0; i < 4 ; i++)
		VMa[i] = mMCs[i] + (mSVs[0] - mX0)/2.0*mMarkerSize;

	//GET MARKERS VERTICES	
	for(uint i = 0; i < 4; i++)
		for(uint j = 0; j < 4; j++)
			mMVs[i][j] = mMCs[i] + Rodrigues(VMa[i] - mMCs[i], mN, j*PI/2.0);
		
	for(uint i = 0; i < 4; i++)
	{
		std::cout << "MCenter: " << mMCs[i].t() << ' ';
		for(uint j = 0; j < 4; j++)
			std::cout << mMVs[i][j].t() << " "; 
		std::cout << "\n";
	}
	
	//GET MARKERS MATRICES
	for(uint i = 0; i < 4; i++)
	{
		cv::Mat M = cv::Mat(3, 3, CV_32F);
		M.col(0) = 2*mMVs[i][0] - mMVs[i][0];
		M.col(1) = mMVs[i][1] - mMVs[i][0];
		M.col(2) = mMVs[i][3] - mMVs[i][0];
		mMInv[i] = M.inv();
	}

	// GET MARKER TEXTURE
	for(uint i = 0; i < 4; i++)
	{
		cv::aruco::drawMarker(mDict, mMarkerID[i], ARUCO_SIZE, 
				mMarkerIm[i], 1);
		cv::imwrite("ArUCoMarkers/marker" + std::to_string(mMarkerID[i]) 
				+ ".png", mMarkerIm[i]);
	}
}

void Square::inSquare(cv::Mat m, cv::Mat X0, cv::Mat S, float *d)
{
	cv::Mat w = S*((cv::Mat)(mN.t()*X0/(mN.t()*mKInv*m))).at<float>(0)*
				mKInv*m;
	d[0] = *w.ptr<float>(1), d[1] = *w.ptr<float>(2);  
}

void Square::inMarker(cv::Mat m, uint i, float *d)
{
	inSquare(m, mMCs[i], mMInv[i], d);
}

bool Square::markerPtIsBlack(float *d, uint i)
{
	uint alpha = d[0]*ARUCO_SIZE + 0.5;
	uint beta = d[1]*ARUCO_SIZE + 0.5;
	
	return *mMarkerIm[i].ptr<uint8_t>(alpha, beta);
}

void Square::colorInterp(float a, cv::Vec3b c, cv::Point p, cv::Mat &im)
{
	//MAKE LINEAR INTERPOLATION					
	cv::Mat cMat = cv::Mat(3, 1, CV_32F); //OBJECT COLOR
	*cMat.ptr<float>(0) = c[0];
	*cMat.ptr<float>(1) = c[1];
	*cMat.ptr<float>(2) = c[2];
	
	//BACKGROUND COLOR
	cv::Vec3b b = im.at<cv::Vec3b>(p);
	cv::Mat bMat = cv::Mat(3, 1, CV_32F);
	*bMat.ptr<float>(0) = b[0];
	*bMat.ptr<float>(1) = b[1];
	*bMat.ptr<float>(2) = b[2];

	//WRITE TO IMAGE
	cMat = cMat*a + (1-a)*bMat; 
	im.at<cv::Vec3b>(cv::Point(p)) = {(uint8_t)cMat.at<float>(0), 
									  (uint8_t)cMat.at<float>(1), 
									  (uint8_t)cMat.at<float>(2)};
}

void Square::colorInterp(float a, cv::Point p, cv::Mat &im)
{
	//MAKE LINEAR INTERPOLATION					
	cv::Mat cMat = cv::Mat(3, 1, CV_32F); //OBJECT COLOR
	*cMat.ptr<float>(0) = 0;
	*cMat.ptr<float>(1) = 0;
	*cMat.ptr<float>(2) = 0;
	
	//BACKGROUND COLOR
	cv::Mat bMat = cv::Mat(3, 1, CV_32F);
	*bMat.ptr<float>(0) = 255;
	*bMat.ptr<float>(1) = 255;
	*bMat.ptr<float>(2) = 255;

	//WRITE TO IMAGE
	cMat = cMat*a + (1-a)*bMat; 
	im.at<cv::Vec3b>(cv::Point(p)) = {(uint8_t)cMat.at<float>(0), 
									  (uint8_t)cMat.at<float>(1), 
									  (uint8_t)cMat.at<float>(2)};
}

void Square::insertSquare(cv::Mat &im)
{	
	cv::Mat m = cv::Mat(3, 1, CV_32F); //PIXEL
	cv::Mat nx = cv::Mat(100, 1, CV_32F); //UNIFORM DISTRIBUTION
	cv::Mat ny = cv::Mat(100, 1, CV_32F); //UNIFORM DISTRIBUTION

	for(uint j = 0; j < (uint)im.rows; j++)
		for(uint i = 0; i < (uint)im.cols; i++)
		{
			//GET PERCENTAGE OF SQUARE INSIDE OF PIXEL
			uint ct = 0;
			cv::randu(nx, 0, 1), cv::randu(ny, 0, 1);
			for(uint w = 0; w < (uint)nx.rows; w++)
			{
				m.at<float>(0) = i + nx.at<float>(w); 
				m.at<float>(1) = j + ny.at<float>(w); 
				m.at<float>(2) = 1;
				float d[2];
				inSquare(m, mX0, mSInv, d);
				if(d[0] >= 0 && d[0] < 1 && d[1] >= 0 && d[1] < 1)
					ct++;
			}
			colorInterp(ct*1.0/nx.rows, mSqColor, cv::Point(i, j), im);
		}

	for(uint j = 0; j < (uint)im.rows; j++)
		for(uint i = 0; i < (uint)im.cols; i++)
		{
			//GET PERCENTAGE OF MARKER INSIDE OF PIXEL
			uint ct = 0;
			cv::randu(nx, 0, 1), cv::randu(ny, 0, 1);
			for(uint w = 0; w < (uint)nx.rows; w++)
			{
				m.at<float>(0) = i + nx.at<float>(w); 
				m.at<float>(1) = j + ny.at<float>(w); 
				m.at<float>(2) = 1;
				float d[2];
				for(uint k = 0; k < 4; k++)
				{
					inMarker(m, k, d);
					if(d[0] >= 0 && d[0] < 1 && d[1] >= 0 && d[1] < 1)
					{	
						if(!markerPtIsBlack(d, k))
							ct++;
						break;
					}
				}
			}
			if(ct)
				colorInterp(ct*1.0/nx.rows, cv::Point(i, j), im);
		}
}
